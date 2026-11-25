"""Follow Cartesian waypoints from measured state and command drum positions."""

from collections import deque
from dataclasses import dataclass, replace
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ropes_interfaces.action import FollowCartesianPath
from ropes_interfaces.msg import HardwareStatus
from ropes_kinematics.joints import joint_positions
from ropes_kinematics.model import CableModel

from .hardware_readiness import HardwareHeartbeat, HardwareReadiness
from .planner import (
    ActuatorAngleLimits, CartesianPlan, CartesianPlanner, ScheduledWaypoint, TrajectoryLimits,
    Vector,
)


_BASE_FRAME = "base_link"
_NANOSECONDS_PER_SECOND = 1_000_000_000
_SOURCE_CLOCK_TOLERANCE_NS = 50_000_000
_FEEDBACK_BUFFER_SIZE = 8
_ZERO_VELOCITY = (0.0, 0.0, 0.0)


def _xyz(point):
    return point.x, point.y, point.z


def _stamp_ns(stamp):
    return stamp.sec * _NANOSECONDS_PER_SECOND + stamp.nanosec


def _drum_error(measured, target):
    return max(abs(actual - desired) for actual, desired in zip(measured, target))


@dataclass(frozen=True)
class _MeasuredPose:
    message: PointStamped
    received_ns: int
    source_ns: int


@dataclass(frozen=True)
class _MeasuredDrums:
    angles: Vector
    received_ns: int
    source_ns: int


@dataclass(frozen=True)
class _Feedback:
    pose: _MeasuredPose
    drums: _MeasuredDrums | None = None


@dataclass
class _Execution:
    handle: ServerGoalHandle
    completed: Future
    plan: CartesianPlan | None = None
    started_ns: int = 0
    last_tick_ns: int | None = None
    waypoint_command_ns: int | None = None
    settled_since_ns: int | None = None
    last_angles: Vector | None = None
    hardware_waypoints: tuple[ScheduledWaypoint, ...] = ()
    waypoint_cursor: int = 0


class TrajectoryServer(Node):
    """Use ROS time for execution and a steady timer to allow cancellation when paused."""

    def __init__(self):
        super().__init__("trajectory_server")
        description = Path(get_package_share_directory("ropes_description"))
        default_config = description / "config" / "cables.json"
        config_path = self.declare_parameter("config_path", str(default_config)).value
        self.model = CableModel.from_file(config_path)
        defaults = vars(TrajectoryLimits())
        limits = TrajectoryLimits(**{
            name: self.declare_parameter(name, default).value
            for name, default in defaults.items()
        })
        actuator_cap = self.declare_parameter("actuator_max_drum_speed_rad_s", 0.0).value
        if not math.isfinite(actuator_cap) or actuator_cap < 0.0:
            raise ValueError("actuator_max_drum_speed_rad_s must be finite and nonnegative")
        if 0.0 < actuator_cap < limits.max_drum_speed_rad_s:
            limits = replace(limits, max_drum_speed_rad_s=actuator_cap)
        angle_defaults = ActuatorAngleLimits()
        read_only = ParameterDescriptor(read_only=True)
        actuator_limits = ActuatorAngleLimits(
            lower_rad=self.declare_parameter(
                "actuator_lower_limits_rad", list(angle_defaults.lower_rad), descriptor=read_only
            ).value,
            upper_rad=self.declare_parameter(
                "actuator_upper_limits_rad", list(angle_defaults.upper_rad), descriptor=read_only
            ).value,
        )
        self.planner = CartesianPlanner(self.model, limits, actuator_limits)
        self.command_period_s = 1.0 / self._positive_parameter("command_rate_hz", 50.0)
        self.pose_timeout_s = self._positive_parameter("pose_timeout_s", 0.50)
        self.goal_tolerance_m = self._positive_parameter("goal_tolerance_m", 0.005)
        self.settle_timeout_s = self._positive_parameter("settle_timeout_s", 3.0)
        self.settle_dwell_s = self._positive_parameter("settle_dwell_s", 0.15)
        if self.settle_dwell_s >= self.settle_timeout_s:
            raise ValueError("settle_dwell_s must be shorter than settle_timeout_s")
        self.max_waypoints = self.declare_parameter("max_waypoints", 100).value
        if not 1 <= self.max_waypoints <= 2 ** 32:
            raise ValueError("max_waypoints must be between 1 and 2**32")
        self.require_hardware_ready = self.declare_parameter(
            "require_hardware_ready", False, descriptor=read_only,
        ).value
        self.actuator_goal_tolerance_rad = self._positive_parameter(
            "actuator_goal_tolerance_rad", 0.02, descriptor=read_only
        )

        self._pose: _MeasuredPose | None = None
        self._execution: _Execution | None = None
        self._commands = self.create_publisher(JointState, "drum_command", 1)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._paths = self.create_publisher(PathMessage, "planned_path", path_qos)
        if self.require_hardware_ready:
            self._pose_samples = deque(maxlen=_FEEDBACK_BUFFER_SIZE)
            self._drum_samples = deque(maxlen=_FEEDBACK_BUFFER_SIZE)
            self._hardware = HardwareReadiness(
                self.pose_timeout_s * _NANOSECONDS_PER_SECOND, _SOURCE_CLOCK_TOLERANCE_NS
            )
            self._hardware_subscription = self.create_subscription(
                HardwareStatus, "hardware/status", self._receive_hardware_status, path_qos
            )
            self._hardware_stop = self.create_client(Trigger, "hardware/stop")
            self._drum_subscription = self.create_subscription(
                JointState, "joint_states", self._receive_drums, qos_profile_sensor_data
            )
        self._pose_subscription = self.create_subscription(
            PointStamped, "end_effector_pose", self._receive_pose, qos_profile_sensor_data
        )
        self._server = ActionServer(
            self, FollowCartesianPath, "follow_cartesian_path", self._execute,
            goal_callback=self._check_goal,
            handle_accepted_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
        )
        self._timer = self.create_timer(
            self.command_period_s, self._advance, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _positive_parameter(self, name, default, *, descriptor=None):
        value = self.declare_parameter(name, default, descriptor=descriptor).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _receive_pose(self, message):
        if message.header.frame_id != _BASE_FRAME:
            return
        if not all(map(math.isfinite, _xyz(message.point))):
            return
        source_ns = _stamp_ns(message.header.stamp)
        self._pose = _MeasuredPose(message, self.get_clock().now().nanoseconds, source_ns)
        if self.require_hardware_ready:
            self._pose_samples.append(self._pose)

    def _receive_drums(self, message):
        if message.header.frame_id != _BASE_FRAME:
            return
        source_ns = _stamp_ns(message.header.stamp)
        if source_ns <= 0:
            return
        try:
            positions = joint_positions(message, self.model.joint_names)
        except ValueError:
            return
        self._drum_samples.append(
            _MeasuredDrums(positions, self.get_clock().now().nanoseconds, source_ns)
        )

    def _fresh_sample(self, sample, now_ns):
        max_age_ns = self.pose_timeout_s * _NANOSECONDS_PER_SECOND
        return (
            0 <= now_ns - sample.received_ns <= max_age_ns
            and -_SOURCE_CLOCK_TOLERANCE_NS <= now_ns - sample.source_ns <= max_age_ns
        )

    def _fresh_feedback(self, now_ns):
        if not self.require_hardware_ready:
            if self._pose is not None and self._fresh_sample(self._pose, now_ns):
                return _Feedback(self._pose)
            return None

        drums_by_stamp = {
            drums.source_ns: drums
            for drums in self._drum_samples
            if self._fresh_sample(drums, now_ns)
        }
        pose = max(
            (pose for pose in self._pose_samples
             if pose.source_ns in drums_by_stamp and self._fresh_sample(pose, now_ns)),
            key=lambda sample: sample.source_ns,
            default=None,
        )
        return _Feedback(pose, drums_by_stamp[pose.source_ns]) if pose is not None else None

    def _receive_hardware_status(self, message):
        now_ns = self.get_clock().now().nanoseconds
        steady_ns = time.monotonic_ns()
        heartbeat = HardwareHeartbeat(
            _stamp_ns(message.header.stamp),
            steady_ns, message.state, message.detail, message.referenced, message.armed,
        )
        self._hardware.receive(heartbeat, now_ns)
        execution = self._execution
        if (execution is not None and execution.plan is not None
                and not execution.handle.is_cancel_requested):
            problem = self._hardware.problem(now_ns, steady_ns)
            if problem is not None:
                self._finish(execution, "abort", problem)

    def _hardware_problem(self, now_ns):
        if self.require_hardware_ready:
            return self._hardware.problem(now_ns, time.monotonic_ns())
        return None

    def _check_goal(self, request):
        now_ns = self.get_clock().now().nanoseconds
        hardware_problem = self._hardware_problem(now_ns)
        if self._execution is not None:
            reason = "Another Cartesian path is executing"
        elif request.header.frame_id != _BASE_FRAME:
            reason = f"Waypoint frame must be {_BASE_FRAME}"
        elif not 1 <= len(request.waypoints) <= self.max_waypoints:
            reason = f"Expected 1 to {self.max_waypoints} waypoints"
        elif any(not all(map(math.isfinite, _xyz(point))) for point in request.waypoints):
            reason = "Waypoint coordinates must be finite"
        elif hardware_problem is not None:
            reason = hardware_problem
        elif self._fresh_feedback(now_ns) is None:
            reason = (
                "No fresh time-matched end_effector_pose and joint_states feedback"
                if self.require_hardware_ready
                else f"No fresh {_BASE_FRAME} pose is available on end_effector_pose"
            )
        else:
            return GoalResponse.ACCEPT
        self.get_logger().warning(reason)
        return GoalResponse.REJECT

    def _accept_goal(self, handle):
        self._execution = _Execution(handle, Future())
        handle.execute()

    def _cancel_goal(self, handle):
        execution = self._execution
        if execution is not None and handle == execution.handle:
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    async def _execute(self, handle):
        execution = self._execution
        if handle.is_cancel_requested:
            self._finish(execution, "canceled", "Canceled before execution")
            return await execution.completed
        now_ns = self.get_clock().now().nanoseconds
        hardware_problem = self._hardware_problem(now_ns)
        if hardware_problem is not None:
            self._finish(execution, "abort", hardware_problem)
            return await execution.completed
        feedback = self._fresh_feedback(now_ns)
        if feedback is None:
            self._finish(execution, "abort", "Measured feedback became stale before planning")
            return await execution.completed

        try:
            start = _xyz(feedback.pose.message.point)
            if feedback.drums is not None:
                measured = feedback.drums.angles
                ranges = ((angle, angle) for angle in measured)
                self.planner.actuator_limits.validate(self.model.joint_names, ranges)
                if _drum_error(measured, self.model.inverse(start)) > (
                    self.actuator_goal_tolerance_rad
                ):
                    raise ValueError("Measured end effector pose disagrees with drum positions")
            execution.plan = self.planner.plan(
                start, (_xyz(point) for point in handle.request.waypoints)
            )
        except ValueError as error:
            self._finish(execution, "abort", str(error))
            return await execution.completed

        now = self.get_clock().now()
        execution.started_ns = now.nanoseconds
        if self.require_hardware_ready:
            execution.hardware_waypoints = execution.plan.waypoint_schedule()
        self._publish_path(execution.plan, now.to_msg())
        return await execution.completed

    def _advance(self):
        execution = self._execution
        if execution is None or execution.plan is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if execution.handle.is_cancel_requested:
            message = "Canceled before sending a drum command"
            if self.require_hardware_ready:
                message = "Trajectory canceled"
            elif execution.last_angles is not None:
                self._command(execution.last_angles, now.to_msg(), _ZERO_VELOCITY)
                message = "Canceled. Holding the last commanded drum angles"
            self._finish(execution, "canceled", message)
            return
        hardware_problem = self._hardware_problem(now_ns)
        if hardware_problem is not None:
            self._finish(execution, "abort", hardware_problem)
            return
        if now_ns < execution.started_ns or (
            execution.last_tick_ns is not None and now_ns < execution.last_tick_ns
        ):
            self._finish(execution, "abort", "ROS clock moved backward")
            return
        if now_ns == execution.last_tick_ns:
            return
        previous_ns = execution.last_tick_ns
        if previous_ns is None:
            previous_ns = execution.started_ns
        max_step_s = max(self.pose_timeout_s, 3.0 * self.command_period_s)
        if now_ns - previous_ns > max_step_s * _NANOSECONDS_PER_SECOND:
            self._finish(
                execution, "abort", "ROS time advanced too far between trajectory updates"
            )
            return
        execution.last_tick_ns = now_ns
        feedback = self._fresh_feedback(now_ns)
        if feedback is None:
            message = (
                "Time-matched pose and drum feedback became stale"
                if self.require_hardware_ready else "Measured end effector pose became stale"
            )
            self._finish(execution, "abort", message)
            return

        elapsed_s = (now_ns - execution.started_ns) / _NANOSECONDS_PER_SECOND
        plan = execution.plan
        waypoint = (
            execution.hardware_waypoints[execution.waypoint_cursor]
            if execution.hardware_waypoints else None
        )
        cartesian_velocity = _ZERO_VELOCITY
        if waypoint is not None and elapsed_s >= waypoint.elapsed_s:
            elapsed_s = waypoint.elapsed_s
            position = waypoint.position
            segment_index = waypoint.segment_index
            settling = waypoint
        else:
            sample = plan.sample(elapsed_s)
            position = sample.position
            cartesian_velocity = sample.velocity
            segment_index = sample.segment_index
            settling = None
            if elapsed_s >= plan.duration_s:
                settling = ScheduledWaypoint(plan.duration_s, position, segment_index)
        try:
            angles = self.model.inverse(position)
            velocities = None
            if not self.require_hardware_ready:
                velocities = tuple(
                    math.fsum(
                        derivative * speed for derivative, speed in zip(row, cartesian_velocity)
                    )
                    for row in self.model.jacobian(position)
                )
        except ValueError as error:
            self._finish(execution, "abort", f"Planned sample failed inverse kinematics: {error}")
            return
        stamp = now.to_msg()
        self._command(angles, stamp, velocities)
        execution.last_angles = angles

        progress = 1.0 if elapsed_s >= plan.duration_s else elapsed_s / plan.duration_s
        execution.handle.publish_feedback(FollowCartesianPath.Feedback(
            segment_index=segment_index, progress=progress, position=feedback.pose.message
        ))
        if settling is not None:
            self._settle(execution, now_ns, feedback, settling, angles)

    def _settle(self, execution, now_ns, feedback, waypoint, angles):
        if execution.waypoint_command_ns is None:
            execution.waypoint_command_ns = now_ns
            return

        pose = feedback.pose
        error_m = math.dist(_xyz(pose.message.point), waypoint.position)
        observed_after_command = pose.received_ns > execution.waypoint_command_ns
        within_tolerance = error_m <= self.goal_tolerance_m
        error_rad = None
        if feedback.drums is not None:
            observed_after_command = pose.source_ns > execution.waypoint_command_ns
            error_rad = _drum_error(feedback.drums.angles, angles)
            within_tolerance &= error_rad <= self.actuator_goal_tolerance_rad
        if observed_after_command and within_tolerance:
            if execution.settled_since_ns is None:
                execution.settled_since_ns = pose.source_ns
            stable_s = (pose.source_ns - execution.settled_since_ns) / _NANOSECONDS_PER_SECOND
            if stable_s >= self.settle_dwell_s:
                if waypoint.elapsed_s >= execution.plan.duration_s:
                    self._finish(execution, "succeed", "Cartesian path completed")
                else:
                    # Resume the timed plan at this waypoint once the real drives have arrived.
                    execution.started_ns = now_ns - round(
                        waypoint.elapsed_s * _NANOSECONDS_PER_SECOND
                    )
                    execution.waypoint_cursor += 1
                    execution.waypoint_command_ns = None
                    execution.settled_since_ns = None
                return
        else:
            execution.settled_since_ns = None

        waiting_s = (now_ns - execution.waypoint_command_ns) / _NANOSECONDS_PER_SECOND
        if waiting_s >= self.settle_timeout_s:
            if error_rad is None:
                message = f"End effector did not settle (error {error_m:.4f} m)"
            else:
                message = (
                    f"Waypoint {waypoint.segment_index + 1} did not settle "
                    f"(Cartesian error {error_m:.4f} m, drum error {error_rad:.4f} rad)"
                )
            self._finish(execution, "abort", message)

    def _command(self, angles, stamp, velocities=None):
        message = JointState()
        message.header.stamp = stamp
        message.name = list(self.model.joint_names)
        message.position = list(angles)
        if velocities is not None:
            message.velocity = list(velocities)
        self._commands.publish(message)

    def _publish_path(self, plan, stamp):
        path = PathMessage()
        path.header.frame_id = _BASE_FRAME
        path.header.stamp = stamp
        for x, y, z in plan.points:
            pose = PoseStamped(header=path.header)
            pose.pose.position = Point(x=x, y=y, z=z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._paths.publish(path)

    def _request_hardware_stop(self):
        self._hardware.wait_for_disarm(self.get_clock().now().nanoseconds)
        if not self._hardware_stop.service_is_ready():
            self.get_logger().error("Hardware stop service is unavailable")
            return "Hardware stop service is unavailable"
        try:
            pending = self._hardware_stop.call_async(Trigger.Request())
        except Exception as error:
            self.get_logger().error(f"Could not send hardware stop request: {error}")
            return "Could not send hardware stop request"
        pending.add_done_callback(self._hardware_stop_result)
        return "Hardware stop requested"

    def _hardware_stop_result(self, pending):
        try:
            response = pending.result()
        except Exception as error:
            self.get_logger().error(f"Hardware stop request failed: {error}")
            return
        if not response.success:
            self.get_logger().error(f"Hardware stop declined: {response.message}")

    def _finish(self, execution, outcome, message):
        commands_sent = execution.last_angles is not None
        if self.require_hardware_ready and outcome != "succeed" and commands_sent:
            message = f"{message}. {self._request_hardware_stop()}"
        if outcome == "succeed":
            execution.handle.succeed()
        elif outcome == "canceled":
            execution.handle.canceled()
        else:
            execution.handle.abort()
            self.get_logger().warning(message)
        self._execution = None
        execution.completed.set_result(FollowCartesianPath.Result(
            success=outcome == "succeed", message=message
        ))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
