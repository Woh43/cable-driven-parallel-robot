"""Launch the Gazebo Classic server with the calibrated cable robot world."""

from pathlib import Path
from tempfile import TemporaryDirectory
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

from ropes_simulation.world import make_world


def create_simulation(context):
    simulation_share = Path(get_package_share_directory('ropes_simulation'))
    description_share = Path(get_package_share_directory('ropes_description'))
    config_path = LaunchConfiguration('config_path').perform(context)
    settings_path = LaunchConfiguration('settings_path').perform(context)
    use_sim_time = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
    world = make_world(simulation_share / 'worlds' / 'cable_robot.world', description_share,
                       config_path, settings_path, use_sim_time)
    temporary = TemporaryDirectory(prefix='ropes_gazebo_')
    world_path = Path(temporary.name) / 'cable_robot.world'
    world.write(world_path, encoding='utf-8', xml_declaration=True)

    paths = [str(Path(get_package_prefix(package)) / 'lib')
             for package in ('ropes_simulation', 'gazebo_ros')]
    existing = os.environ.get('GAZEBO_PLUGIN_PATH')
    if existing:
        paths.append(existing)
    env = {'GAZEBO_PLUGIN_PATH': os.pathsep.join(paths), 'GAZEBO_MODEL_DATABASE_URI': ''}
    ros_params = simulation_share / 'config' / 'gazebo_ros.yaml'
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: temporary.cleanup())])),
        ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so', str(world_path),
                            '--ros-args', '--params-file', str(ros_params), '--'],
                       additional_env=env, output='screen',
                       on_exit=Shutdown(reason='Gazebo server exited')),
        ExecuteProcess(cmd=['gzclient'], additional_env=env, output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
    ]


def generate_launch_description():
    description_share = Path(get_package_share_directory('ropes_description'))
    simulation_share = Path(get_package_share_directory('ropes_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Show the Gazebo Classic client.'),
        DeclareLaunchArgument('config_path',
                              default_value=str(description_share / 'config' / 'cables.json'),
                              description='Path to the cable geometry calibration JSON.'),
        DeclareLaunchArgument('settings_path',
                              default_value=str(simulation_share / 'config' / 'simulation.yaml'),
                              description='Path to the Gazebo physics settings YAML.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'],
                              description='Stamp feedback with the Gazebo clock.'),
        OpaqueFunction(function=create_simulation),
    ])
