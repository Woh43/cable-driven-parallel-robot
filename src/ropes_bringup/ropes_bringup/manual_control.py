"""Validate manual Cartesian and drum inputs before sending simulator commands."""

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ropes_kinematics.joints import joint_positions
from ropes_kinematics.model import CableModel
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .description import BASE_FRAME, CARTESIAN_JOINTS, DEFAULT_GEOMETRY_PATH, slider_description


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.mode = self.declare_parameter('control_mode', 'drum').value
        config_path = self.declare_parameter('config_path', str(DEFAULT_GEOMETRY_PATH)).value
        use_gui = self.declare_parameter('use_gui', False).value
        if self.mode not in ('xyz', 'drum'):
            raise ValueError('control_mode must be xyz or drum')

        self.model = CableModel.from_file(config_path)
        self.last_error = None
        self.command = self.create_publisher(JointState, 'drum_command', 10)
        if self.mode == 'drum':
            self.create_subscription(
                JointState, 'drum_joint_states', self.receive_drums, qos_profile_sensor_data
            )
        else:
            self.create_subscription(
                JointState, 'end_effector_offsets', self.receive_offsets, qos_profile_sensor_data
            )
            self.create_subscription(
                PointStamped, 'end_effector_target', self.receive_target, qos_profile_sensor_data
            )
        if use_gui:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.slider_model = self.create_publisher(String, 'slider_controls_description', qos)
            self.slider_model.publish(String(data=slider_description(self.model, self.mode)))

    def receive_drums(self, message):
        try:
            angles = joint_positions(message, self.model.joint_names)
            self.model.forward(angles)
        except ValueError as error:
            self.reject(error)
            return
        self.publish(angles)

    def receive_offsets(self, message):
        try:
            offsets = joint_positions(message, CARTESIAN_JOINTS)
            position = tuple(home + offset for home, offset in zip(self.model.home, offsets))
            angles = self.model.inverse(position)
        except ValueError as error:
            self.reject(error)
            return
        self.publish(angles)

    def receive_target(self, message):
        if message.header.frame_id != BASE_FRAME:
            self.reject(f'Target frame must be {BASE_FRAME}')
            return
        p = message.point
        try:
            angles = self.model.inverse((p.x, p.y, p.z))
        except ValueError as error:
            self.reject(error)
            return
        self.publish(angles)

    def reject(self, error):
        description = str(error)
        if description != self.last_error:
            self.get_logger().warning(f'Holding last drum command: {description}')
            self.last_error = description

    def publish(self, angles):
        self.last_error = None
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.model.joint_names)
        message.position = list(angles)
        self.command.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualControl()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
