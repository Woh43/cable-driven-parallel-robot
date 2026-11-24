"""Update nominal drum and point positions with the closed-form cable geometry."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ropes_kinematics.joints import joint_positions
from ropes_kinematics.model import CableModel
from ropes_simulation.world import CARTESIAN_JOINTS


class KinematicSimulator(Node):
    def __init__(self):
        super().__init__('kinematic_simulator')
        share = Path(get_package_share_directory('ropes_description'))
        default_path = str(share / 'config' / 'cables.json')
        config_path = self.declare_parameter('config_path', default_path).value
        rate = self.declare_parameter('state_rate_hz', 50.0).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('state_rate_hz must be a finite positive value')
        self.model = CableModel.from_file(config_path)
        self.angles = (0.0,) * 3
        self.position = self.model.home
        self.last_error = None
        self.states = self.create_publisher(JointState, 'joint_states', 10)
        self.poses = self.create_publisher(PointStamped, 'end_effector_pose', 10)
        self.commands = self.create_subscription(
            JointState, 'drum_command', self.receive_command, 5)
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

    def receive_command(self, message):
        try:
            angles = joint_positions(message, self.model.joint_names)
            position = self.model.forward(angles)
        except ValueError as error:
            description = str(error)
            if description != self.last_error:
                self.get_logger().warning(f'Holding last reachable state. {description}')
            self.last_error = description
            return
        self.angles = angles
        self.position = position
        self.last_error = None

    def publish_state(self):
        pose = PointStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.point.x, pose.point.y, pose.point.z = self.position
        self.poses.publish(pose)

        state = JointState()
        state.header = pose.header
        state.name = [*self.model.joint_names, *CARTESIAN_JOINTS]
        offsets = tuple(actual - zero for actual, zero in zip(self.position, self.model.home))
        state.position = [*self.angles, *offsets]
        self.states.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicSimulator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
