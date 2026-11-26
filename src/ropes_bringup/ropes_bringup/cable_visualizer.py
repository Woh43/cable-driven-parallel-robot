"""Draw the cable paths and static workspace in RViz."""

import math

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ropes_kinematics.model import CableModel
from visualization_msgs.msg import Marker, MarkerArray

from .description import BASE_FRAME, DEFAULT_GEOMETRY_PATH


def _workspace_markers(footprint, heights):
    low, high = heights
    floor = [Point(x=x, y=y, z=low) for x, y in footprint]
    ceiling = [Point(x=x, y=y, z=high) for x, y in footprint]
    surface = Marker(id=0, type=Marker.TRIANGLE_LIST)
    outline = Marker(id=1, type=Marker.LINE_LIST)
    for marker in (surface, outline):
        marker.header.frame_id = BASE_FRAME
        marker.ns = 'static_workspace'
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

    surface.scale.x = surface.scale.y = surface.scale.z = 1.0
    surface.color.r, surface.color.g, surface.color.b, surface.color.a = 0.06, 0.52, 0.76, 0.23
    surface.points = [*reversed(floor), *ceiling]
    outline.scale.x = 0.002
    outline.color.r, outline.color.g, outline.color.b, outline.color.a = 0.04, 0.33, 0.60, 0.95
    for index in range(len(floor)):
        following = (index + 1) % len(floor)
        a, b = floor[index], floor[following]
        c, d = ceiling[following], ceiling[index]
        surface.points.extend((a, b, c, a, c, d))
        outline.points.extend((a, b, d, c, a, d))
    return MarkerArray(markers=[surface, outline])


class CableVisualizer(Node):
    def __init__(self):
        super().__init__('cable_visualizer')
        config_path = self.declare_parameter('config_path', str(DEFAULT_GEOMETRY_PATH)).value
        width = self.declare_parameter('cable_display_width_m', 0.002).value
        if not math.isfinite(width) or width <= 0.0:
            raise ValueError('cable_display_width_m must be finite and positive')
        model = CableModel.from_file(config_path)
        self.workspace_publisher = self.create_publisher(
            MarkerArray, 'static_workspace',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        bounds = model.static_workspace_bounds()
        if bounds is None:
            self.get_logger().warning('The calibration leaves no static workspace prism to draw')
            markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        else:
            markers = _workspace_markers(*bounds)
        self.workspace_publisher.publish(markers)

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
