"""Bridge three serial shaft encoders and stepper drives to the ROS robot model."""

import math
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ropes_interfaces.msg import HardwareStatus
from ropes_kinematics.joints import joint_positions
from ropes_kinematics.model import CableModel
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .motor_targets import quantized_motor_target
from .settings import HardwareSettings
from .zdt_emm_v5 import DriveError, MoveInterrupted, ZdtEmmV5


_BASE = 'base_link'
_CARTESIAN = ('end_effector_x_joint', 'end_effector_y_joint', 'end_effector_z_joint')
_NS = 1_000_000_000
_STATIONARY_SECONDS = 0.25


class HardwareDriver(Node):
    """A single worker owns serial I/O; ROS callbacks maintain the arming interlock."""

    def __init__(self, drive_factory=ZdtEmmV5):
        super().__init__('hardware_driver')
        description = Path(get_package_share_directory('ropes_description'))
        drivers = Path(get_package_share_directory('ropes_hardware'))
        config = self.declare_parameter('config_path', str(description / 'config/cables.json'))
        settings = self.declare_parameter(
            'hardware_config_path', str(drivers / 'config/hardware.example.yaml')
        )
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Hardware requires the system clock (use_sim_time=false)')

        self.model = CableModel.from_file(config.value)
        self.settings = HardwareSettings.from_file(settings.value, self.model.joint_names)
        self._bluetooth = self.settings.transport == 'bluetooth'
        self._motor_step_rad = tuple(
            math.tau / axis.pulses_per_revolution for axis in self.settings.axes
        )
        rounding_margin = max(
            step / (2.0 * axis.gear_ratio)
            for step, axis in zip(self._motor_step_rad, self.settings.axes)
        )
        self._lead_limit_rad = self.settings.max_command_lead_rad - rounding_margin
        if self._lead_limit_rad <= 0.0:
            raise ValueError('max_command_lead_rad must exceed the drum pulse quantization margin')
        self._drive_factory = drive_factory
        self._read_max_age_s = max(0.25, 3.0 / self.settings.poll_rate_hz)
        self._lock = Lock()
        self._exit = Event()
        self._wake = Event()
        self._samples = None
        self._sample_time = 0.0
        self._stamp = None
        self._stationary_at = None
        self._stable_angles = None
        self._home = None
        self._pose = None
        self._measured = None
        self._verified = False
        self._armed = False
        self._armed_stamp = 0
        self._last_command_stamp = 0
        self._desired = None
        self._desired_deadline = 0.0
        self._dispatched = None
        self._needs_hold = False
        self._stop_pending = False
        label = 'ESP32 Bluetooth' if self._bluetooth else 'USB serial'
        self._detail = f'Connecting to the three configured motor drivers via {label}'
        self._fault = None

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status = self.create_publisher(HardwareStatus, 'hardware/status', status_qos)
        self._motor_states = self.create_publisher(JointState, 'hardware/motor_states', 10)
        self._states = self.create_publisher(JointState, 'joint_states', 10)
        self._poses = self.create_publisher(PointStamped, 'end_effector_pose', 10)
        self.create_subscription(JointState, 'drum_command', self._command, 1)
        self.create_service(Trigger, 'hardware/reference_home', self._reference_home)
        self.create_service(Trigger, 'hardware/arm', self._arm)
        self.create_service(Trigger, 'hardware/stop', self._stop)
        self.create_timer(1.0 / self.settings.poll_rate_hz, self._publish_feedback)
        self._worker = Thread(target=self._serial_loop, name='rope_serial_io', daemon=True)
        self._worker.start()

    def _fresh(self, now):
        return self._samples is not None and now - self._sample_time <= self._read_max_age_s

    def _stationary(self):
        return (self._stationary_at is not None
                and self._sample_time - self._stationary_at >= _STATIONARY_SECONDS)

    def _reference_home(self, _, response):
        with self._lock:
            now = monotonic()
            if self._fault or not self._verified or not self._fresh(now):
                response.message = 'Reference requires healthy, fresh feedback on all three ports'
            elif self._armed or self._stop_pending:
                response.message = 'Stop and wait for the drives before referencing'
            elif not self._stationary():
                response.message = 'All three shafts must be stationary before referencing'
            else:
                self._home = tuple(sample.position_rad for sample in self._samples)
                self._measured = (0.0, 0.0, 0.0)
                self._pose = self.model.home
                self._detail = 'Physical home recorded as in-memory zero for each shaft'
                response.success = True
                response.message = self._detail
        if response.success:
            self._publish_feedback()
        return response

    def _arm(self, _, response):
        with self._lock:
            now = monotonic()
            if not self.settings.allow_motion:
                response.message = 'This session is configured for read-only probing'
            elif self._fault:
                response.message = self._fault
            elif self._armed:
                response.message = 'Hardware is already armed'
            elif self._stop_pending or not self._verified or not self._fresh(now):
                response.message = 'Wait for healthy, fresh feedback and a completed stop'
            elif self._home is None or self._pose is None:
                response.message = 'Reference the stationary payload at the physical home first'
            elif not self._stationary():
                response.message = 'All three shafts must be stationary before arming'
            elif any(not motor.enabled or motor.stalled for motor in self._samples):
                response.message = 'Each controller must be enabled and free of stall flags'
            else:
                self._armed = True
                self._needs_hold = True
                self._armed_stamp = self.get_clock().now().nanoseconds
                self._last_command_stamp = 0
                self._dispatched = self._quantize(tuple(m.position_rad for m in self._samples))
                self._detail = 'Armed and waiting for a fresh drum command'
                response.success = True
                response.message = self._detail
        if response.success:
            self._publish_feedback()
        return response

    def _stop(self, _, response):
        with self._lock:
            if self._fault:
                response.message = self._fault
            elif self._needs_hold or self._stop_pending:
                self._request_stop('Stop requested. Waiting for all three drivers to acknowledge')
                response.success = True
                response.message = self._detail
            else:
                response.success = True
                response.message = 'Hardware command gate is disarmed'
        return response

    def _request_stop(self, reason):
        """Disarm immediately under the lock; the I/O worker handles the serial stop."""
        self._armed = False
        self._desired = None
        self._stop_pending = self._needs_hold
        self._detail = reason
        self._wake.set()

    def _command(self, message):
        with self._lock:
            if not self._armed:
                return
            stamp = message.header.stamp.sec * _NS + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / _NS
            if stamp <= max(self._armed_stamp, self._last_command_stamp):
                return
            if not -0.05 <= age <= self.settings.command_timeout_s:
                return
            try:
                angles = joint_positions(message, self.model.joint_names)
                self.settings.check_drums(angles)
                self.model.forward(angles)
            except ValueError as error:
                self._request_stop(f'Rejected unreachable drum command. {error}')
                return
            self._desired = angles
            self._desired_deadline = monotonic() + self.settings.command_timeout_s - max(age, 0.0)
            self._last_command_stamp = stamp

    def _serial_loop(self):
        drive = None
        try:
            axes = self.settings.axes
            if self._bluetooth:
                from .zdt_bluetooth import BluetoothZdtEmmV5

                drive = BluetoothZdtEmmV5(self.settings)
            else:
                drive = self._drive_factory(
                    tuple(axis.port for axis in axes), tuple(axis.address for axis in axes),
                    tuple(axis.pulses_per_revolution for axis in axes),
                    self.settings.baudrate, self.settings.timeout_s,
                )
            versions = drive.read_firmware()
            measured_versions = ', '.join(
                f'{name}: firmware 0x{firmware:02X}, hardware 0x{hardware:02X}'
                for name, (firmware, hardware) in zip(self.model.joint_names, versions)
            )
            self.get_logger().info(measured_versions)
            for name, axis, (firmware, _) in zip(self.model.joint_names, axes, versions):
                if axis.expected_firmware is not None and axis.expected_firmware != firmware:
                    raise DriveError(f'{name}: unexpected firmware byte 0x{firmware:02X}')
            if drive.verify_configuration() != versions:
                raise DriveError('Drive versions changed during the startup handshake')
            with self._lock:
                self._verified = True
                if self.settings.allow_motion:
                    self._detail = 'Verified drivers. Awaiting physical home reference'
                else:
                    self._detail = 'Verified read-only probe. ' + measured_versions

            period = 1.0 / self.settings.poll_rate_hz
            while not self._exit.is_set():
                start = monotonic()
                if self._finish_stop(drive):
                    continue
                samples = drive.read_state()
                pending = self._process_feedback(samples)
                if pending is not None:
                    self._dispatch(drive, *pending)
                if self._bluetooth and self._may_renew_lease():
                    drive.keepalive()
                self._wake.wait(max(0.0, period - (monotonic() - start)))
                self._wake.clear()
        except (DriveError, OSError, ValueError) as error:
            with self._lock:
                self._armed = False
                self._home = None
                self._pose = None
                self._fault = f'{self.settings.transport} hardware fault. {error}'
                self._detail = self._fault
            self.get_logger().error(self._fault)
        finally:
            if drive is not None:
                with self._lock:
                    should_hold = self._needs_hold
                    self._armed = False
                    self._desired = None
                if should_hold:
                    try:
                        drive.stop_all()
                    except DriveError as error:
                        warning = f'Could not confirm all drive stops. {error}'
                        with self._lock:
                            if self._fault:
                                self._fault = f'{self._fault}. {warning}'
                                self._detail = self._fault
                        self.get_logger().error(warning)
                try:
                    drive.close()
                except (DriveError, OSError) as error:
                    self.get_logger().warning(f'Could not close the hardware connection. {error}')

    def _finish_stop(self, drive):
        with self._lock:
            if not self._stop_pending:
                return False
        drive.stop_all()
        self._stop_acknowledged()
        return True

    def _stop_acknowledged(self, reason=None):
        with self._lock:
            self._armed = False
            self._desired = None
            self._needs_hold = False
            self._stop_pending = False
            self._dispatched = None
            self._samples = None
            self._stable_angles = None
            self._stationary_at = None
            self._detail = f'{reason or self._detail}. Stop acknowledged by all three drives'

    def _may_renew_lease(self):
        with self._lock:
            return self._armed and not self._exit.is_set() and self._fresh(monotonic())

    def _dispatch(self, drive, motor_angles, deadline):
        def can_start():
            with self._lock:
                now = monotonic()
                return (self._armed and not self._exit.is_set()
                        and now <= deadline and self._fresh(now))

        if can_start():
            try:
                drive.move_absolute(
                    motor_angles, self.settings.speeds_rpm, self.settings.acceleration,
                    can_start=can_start,
                )
            except MoveInterrupted as error:
                self._stop_acknowledged(str(error))
                return
            with self._lock:
                self._dispatched = motor_angles

    def _quantize(self, motor_angles):
        return tuple(
            math.copysign(round(abs(angle) / step) * step, angle)
            for angle, step in zip(motor_angles, self._motor_step_rad)
        )

    def _process_feedback(self, samples):
        now = monotonic()
        timestamp = self.get_clock().now().to_msg()
        motor_angles = tuple(motor.position_rad for motor in samples)
        with self._lock:
            self._samples = samples
            self._sample_time = now
            self._stamp = timestamp
            if self._stable_angles is None or any(
                abs(angle - baseline) / axis.gear_ratio > 0.005
                for angle, baseline, axis in zip(motor_angles, self._stable_angles,
                                                 self.settings.axes)
            ):
                self._stable_angles = motor_angles
                self._stationary_at = now
            if self._home is None:
                return None

            self._pose = None
            try:
                self._measured = self.settings.to_drum_angles(motor_angles, self._home)
                self.settings.check_drums(self._measured)
                self._pose = self.model.forward(self._measured)
            except ValueError as error:
                if self._armed:
                    raise DriveError(f'Measured drum positions are outside the model. {error}')
                return None
            if not self._armed:
                return None
            if any(not motor.enabled or motor.stalled for motor in samples):
                raise DriveError('A driver reported loss of enable or a stall')
            if self._desired is None:
                return None

            delta = tuple(target - actual for target, actual in zip(self._desired, self._measured))
            distance = max(map(abs, delta))
            deadline = self._desired_deadline
            if now > deadline:
                inflight = self.settings.to_drum_angles(self._dispatched, self._home)
                pending_travel = max(abs(a - b) for a, b in zip(self._measured, inflight))
                if max(distance, pending_travel) > self.settings.hold_tolerance_rad:
                    self._request_stop('Drum command expired before the drives reached the target')
                self._desired = None
                return None
            scale = min(1.0, self._lead_limit_rad / distance) if distance else 1.0
            bounded = tuple(actual + scale * step for actual, step in zip(self._measured, delta))
            try:
                motors = quantized_motor_target(
                    self.model, self.settings, bounded, self._measured, self._home,
                    self._motor_step_rad,
                )
            except ValueError as error:
                self._request_stop(str(error))
                return None
            if motors == self._dispatched:
                return None
            return motors, deadline

    def _publish_feedback(self):
        with self._lock:
            stamp = self._stamp
            referenced = self._home is not None
            fresh = self._fresh(monotonic())
            if self._armed and not fresh:
                self._request_stop('Drive feedback expired while armed')
            armed = self._armed
            samples = self._samples if fresh and not self._fault else None
            position = self._pose if fresh and not self._fault else None
            angles = self._measured
            if self._fault:
                state = 'fault'
            elif self._stop_pending:
                state = 'stopping'
            elif armed:
                state = 'armed'
            elif not self._verified:
                state = 'connecting'
            elif not self.settings.allow_motion:
                state = 'probe'
            elif referenced:
                state = 'disarmed'
            else:
                state = 'awaiting_reference'
            detail = self._detail

        status = HardwareStatus(state=state, detail=detail, referenced=referenced, armed=armed)
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = _BASE
        if samples is not None:
            motor_state = JointState(name=list(self.model.joint_names))
            motor_state.header.stamp = stamp
            motor_state.position = [sample.position_rad for sample in samples]
            self._motor_states.publish(motor_state)
        if position is not None:
            pose = PointStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = _BASE
            pose.point.x, pose.point.y, pose.point.z = position
            self._poses.publish(pose)
            state = JointState(header=pose.header, name=[*self.model.joint_names, *_CARTESIAN])
            offsets = [actual - home for actual, home in zip(position, self.model.home)]
            state.position = [*angles, *offsets]
            self._states.publish(state)
        self._status.publish(status)

    def close(self):
        self._exit.set()
        self._wake.set()
        self._worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
