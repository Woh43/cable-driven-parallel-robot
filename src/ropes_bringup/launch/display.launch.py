"""Start manual XYZ or drum sliders on the kinematic backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ropes_bringup.description import DEFAULT_GEOMETRY_PATH


def generate_launch_description():
    share = Path(get_package_share_directory('ropes_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'control', default_value='drum', choices=['drum', 'xyz'],
            description='Control drum angles or Cartesian offsets.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the sliders.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'], description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'config_path', default_value=str(DEFAULT_GEOMETRY_PATH),
            description='Path to the cable geometry calibration JSON.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'simulation.launch.py')),
            launch_arguments={
                'backend': 'kinematic',
                'control': LaunchConfiguration('control'),
                'slider_gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'config_path': LaunchConfiguration('config_path'),
            }.items(),
        ),
    ])
