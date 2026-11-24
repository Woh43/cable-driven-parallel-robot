"""Follow Cartesian waypoints from measured state and command drum positions."""

from dataclasses import dataclass, replace
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState

from ropes_interfaces.action import FollowCartesianPath
from ropes_kinematics.model import CableModel

from .planner import CartesianPlan, CartesianPlanner, TrajectoryLimits, Vector


_BASE_FRAME = "base_link"
_NANOSECONDS_PER_SECOND = 1_000_000_000
_SOURCE_CLOCK_TOLERANCE_NS = 50_000_000


def _xyz(point):
    return point.x, point.y, point.z


@dataclass(frozen=True)
class _MeasuredPose:
    message: PointStamped
    received_ns: int
    source_ns: int


@dataclass
class _Execution:
    handle: ServerGoalHandle
    completed: Future
    plan: CartesianPlan | None = None
    started_ns: int = 0
    last_tick_ns: int | None = None
    final_command_ns: int | None = None
    settled_since_ns: int | None = None
    last_angles: Vector | None = None


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
        self.planner = CartesianPlanner(self.model, limits)
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

        self._pose: _MeasuredPose | None = None
        self._execution: _Execution | None = None
        self._commands = self.create_publisher(JointState, "drum_command", 1)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._paths = self.create_publisher(PathMessage, "planned_path", path_qos)
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

    def _positive_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _receive_pose(self, message):
        if message.header.frame_id != _BASE_FRAME:
            return
        if not all(map(math.isfinite, _xyz(message.point))):
            return
        stamp = message.header.stamp
        source_ns = stamp.sec * _NANOSECONDS_PER_SECOND + stamp.nanosec
        self._pose = _MeasuredPose(message, self.get_clock().now().nanoseconds, source_ns)

    def _fresh_pose(self, now_ns):
        pose = self._pose
        if pose is None:
            return None
        max_age_ns = self.pose_timeout_s * _NANOSECONDS_PER_SECOND
        if (0 <= now_ns - pose.received_ns <= max_age_ns and
                -_SOURCE_CLOCK_TOLERANCE_NS <= now_ns - pose.source_ns <= max_age_ns):
            return pose
        return None

    def _check_goal(self, request):
        if self._execution is not None:
            reason = "Another Cartesian path is executing"
        elif request.header.frame_id != _BASE_FRAME:
            reason = f"Waypoint frame must be {_BASE_FRAME}"
        elif not 1 <= len(request.waypoints) <= self.max_waypoints:
            reason = f"Expected 1 to {self.max_waypoints} waypoints"
        elif any(not all(map(math.isfinite, _xyz(point))) for point in request.waypoints):
            reason = "Waypoint coordinates must be finite"
        elif self._fresh_pose(self.get_clock().now().nanoseconds) is None:
            reason = f"No fresh {_BASE_FRAME} pose is available on end_effector_pose"
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
        pose = self._fresh_pose(self.get_clock().now().nanoseconds)
        if pose is None:
            self._finish(execution, "abort", "Measured pose became stale before planning")
            return await execution.completed

        try:
            execution.plan = self.planner.plan(
                _xyz(pose.message.point), (_xyz(point) for point in handle.request.waypoints)
            )
        except ValueError as error:
            self._finish(execution, "abort", str(error))
            return await execution.completed

        now = self.get_clock().now()
        execution.started_ns = now.nanoseconds
        self._publish_path(execution.plan, now.to_msg())
        return await execution.completed

    def _advance(self):
        execution = self._execution
        if execution is None or execution.plan is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if execution.handle.is_cancel_requested:
            if execution.last_angles is not None:
                self._command(execution.last_angles, now.to_msg())
            self._finish(execution, "canceled", "Canceled; holding the last commanded drum angles")
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
        pose = self._fresh_pose(now_ns)
        if pose is None:
            self._finish(execution, "abort", "Measured end effector pose became stale")
            return

        elapsed_s = (now_ns - execution.started_ns) / _NANOSECONDS_PER_SECOND
        plan = execution.plan
        sample = plan.sample(elapsed_s)
        try:
            angles = self.model.inverse(sample.position)
        except ValueError as error:
            self._finish(execution, "abort", f"Planned sample failed inverse kinematics: {error}")
            return
        stamp = now.to_msg()
        self._command(angles, stamp)
        execution.last_angles = angles

        at_end = elapsed_s >= plan.duration_s
        progress = 1.0 if at_end else elapsed_s / plan.duration_s
        execution.handle.publish_feedback(FollowCartesianPath.Feedback(
            segment_index=sample.segment_index, progress=progress, position=pose.message
        ))
        if at_end:
            self._settle(execution, now_ns, pose)

    def _settle(self, execution, now_ns, pose):
        if execution.final_command_ns is None:
            execution.final_command_ns = now_ns
            return

        target = execution.plan.points[-1]
        error_m = math.dist(_xyz(pose.message.point), target)
        if pose.received_ns > execution.final_command_ns and error_m <= self.goal_tolerance_m:
            if execution.settled_since_ns is None:
                execution.settled_since_ns = pose.source_ns
            stable_s = (pose.source_ns - execution.settled_since_ns) / _NANOSECONDS_PER_SECOND
            if stable_s >= self.settle_dwell_s:
                self._finish(execution, "succeed", "Cartesian path completed")
                return
        else:
            execution.settled_since_ns = None

        elapsed_s = (now_ns - execution.final_command_ns) / _NANOSECONDS_PER_SECOND
        if elapsed_s >= self.settle_timeout_s:
            self._finish(
                execution, "abort", f"End effector did not settle (error {error_m:.4f} m)"
            )

    def _command(self, angles, stamp):
        message = JointState()
        message.header.stamp = stamp
        message.name = list(self.model.joint_names)
        message.position = list(angles)
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

    def _finish(self, execution, outcome, message):
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
