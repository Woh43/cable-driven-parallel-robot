"""Draw the surveyed cable leads and measured moving segments in RViz."""

import math

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ropes_kinematics.model import CableModel
from visualization_msgs.msg import Marker, MarkerArray

from .description import BASE_FRAME, DEFAULT_GEOMETRY_PATH


class CableVisualizer(Node):
    def __init__(self):
        super().__init__('cable_visualizer')
        config_path = self.declare_parameter('config_path', str(DEFAULT_GEOMETRY_PATH)).value
        width = self.declare_parameter('cable_display_width_m', 0.002).value
        if not math.isfinite(width) or width <= 0.0:
            raise ValueError('cable_display_width_m must be finite and positive')
        model = CableModel.from_file(config_path)
        self.paths = MarkerArray()
        for index, cable in enumerate(model.cables, start=1):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.ns = 'cable'
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b = map(float, cable['color_rgb'])
            marker.color.a = 1.0
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in cable['fixed_path_m']]
            marker.points.append(Point())
            self.paths.markers.append(marker)

        self.publisher = self.create_publisher(MarkerArray, 'cable_paths', 10)
        self.subscription = self.create_subscription(
            PointStamped, 'end_effector_pose', self.update, qos_profile_sensor_data
        )

    def update(self, pose):
        if pose.header.frame_id != BASE_FRAME:
            return
        if not all(math.isfinite(value) for value in (pose.point.x, pose.point.y, pose.point.z)):
            return
        for marker in self.paths.markers:
            marker.header.stamp = pose.header.stamp
            marker.points[-1] = pose.point
        self.publisher.publish(self.paths)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CableVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
