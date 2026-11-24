"""ROS 2 services for geometric forward and inverse cable kinematics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ropes_interfaces.srv import ComputeForwardKinematics, ComputeInverseKinematics

from .joints import joint_positions
from .model import CableModel


class KinematicsServer(Node):
    """Provide stateless, frame-checked solvers through ROS 2 services."""

    def __init__(self):
        super().__init__("kinematics_server")
        config_path = self.declare_parameter("config_path", "").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        if not self.base_frame:
            raise ValueError("base_frame must be nonempty")
        if not config_path:
            share = Path(get_package_share_directory("ropes_description"))
            config_path = share / "config" / "cables.json"
        self.model = CableModel.from_file(config_path)
        self.inverse_service = self.create_service(
            ComputeInverseKinematics, "compute_inverse_kinematics", self.inverse_kinematics
        )
        self.forward_service = self.create_service(
            ComputeForwardKinematics, "compute_forward_kinematics", self.forward_kinematics
        )

    def inverse_kinematics(self, request, response):
        target = request.target
        if target.header.frame_id != self.base_frame:
            response.message = f"Target frame must be {self.base_frame}"
            return response
        try:
            point = target.point
            angles = self.model.inverse((point.x, point.y, point.z))
        except ValueError as error:
            response.message = str(error)
            return response

        response.drum_state.header.stamp = self.get_clock().now().to_msg()
        response.drum_state.name = list(self.model.joint_names)
        response.drum_state.position = angles
        response.success = True
        return response

    def forward_kinematics(self, request, response):
        try:
            angles = joint_positions(request.drum_state, self.model.joint_names)
            position = self.model.forward(angles)
        except ValueError as error:
            response.message = str(error)
            return response

        response.position.header.stamp = self.get_clock().now().to_msg()
        response.position.header.frame_id = self.base_frame
        response.position.point = Point(x=position[0], y=position[1], z=position[2])
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicsServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
