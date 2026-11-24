"""Run the robot stack with the USB serial motor backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory('ropes_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_config_path', description='Path to the commissioned hardware YAML.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'simulation.launch.py')),
            launch_arguments={
                'backend': 'hardware',
                'control': 'trajectory',
                'hardware_config_path': LaunchConfiguration('hardware_config_path'),
            }.items(),
        ),
    ])
