"""Start the cable robot with a selected simulator and control source."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ropes_bringup.description import DEFAULT_GEOMETRY_PATH, DESCRIPTION_SHARE, robot_description
from ropes_kinematics.model import CableModel


def create_nodes(context):
    backend = LaunchConfiguration('backend').perform(context)
    control = LaunchConfiguration('control').perform(context)
    slider_gui = IfCondition(LaunchConfiguration('slider_gui')).evaluate(context)
    if slider_gui and control == 'trajectory':
        raise RuntimeError('slider_gui requires control:=xyz or control:=drum')

    config_path = LaunchConfiguration('config_path').perform(context)
    model = CableModel.from_file(config_path)
    use_sim_time = backend == 'gazebo'
    clock_parameters = {'use_sim_time': use_sim_time}
    model_parameters = {'config_path': config_path, **clock_parameters}
    nodes = [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description(model), **clock_parameters}],
        ),
        Node(
            package='ropes_kinematics', executable='kinematics_server', output='screen',
            parameters=[model_parameters],
        ),
        Node(
            package='ropes_bringup', executable='cable_visualizer', output='screen',
            parameters=[model_parameters],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(DESCRIPTION_SHARE / 'rviz' / 'cables.rviz')],
            parameters=[clock_parameters],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]
    backend_limits = {}
    if use_sim_time:
        from ropes_simulation.world import SimulationSettings

        sim_share = Path(get_package_share_directory('ropes_simulation'))
        settings_path = LaunchConfiguration('simulation_config_path').perform(context)
        settings = SimulationSettings.from_file(settings_path)
        backend_limits = {
            'gravity_m_s2': float(settings.gravity_m_s2),
            'actuator_max_drum_speed_rad_s': float(settings.max_drum_speed_rad_s),
        }
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'gazebo.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gazebo_gui'),
                'config_path': config_path,
                'settings_path': settings_path,
                'use_sim_time': 'true',
            }.items(),
        ))
    else:
        nodes.append(Node(
            package='ropes_simulation', executable='kinematic_simulator', output='screen',
            parameters=[model_parameters],
        ))

    if control == 'trajectory':
        trajectory_config = LaunchConfiguration('trajectory_config_path').perform(context)
        nodes.append(Node(
            package='ropes_motion', executable='trajectory_server', output='screen',
            parameters=[trajectory_config, {**model_parameters, **backend_limits}],
        ))
    else:
        nodes.append(Node(
            package='ropes_bringup', executable='manual_control', output='screen',
            parameters=[{**model_parameters, 'control_mode': control, 'use_gui': slider_gui}],
        ))
        if slider_gui:
            topic = 'drum_joint_states' if control == 'drum' else 'end_effector_offsets'
            nodes.append(Node(
                package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                output='screen', parameters=[{'rate': 20, **clock_parameters}],
                remappings=[
                    ('joint_states', topic),
                    ('robot_description', 'slider_controls_description'),
                ],
            ))
    return nodes


def generate_launch_description():
    motion_share = Path(get_package_share_directory('ropes_motion'))
    sim_share = Path(get_package_share_directory('ropes_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='kinematic', choices=['kinematic', 'gazebo'],
            description='Ideal kinematics or Gazebo Classic cable dynamics.',
        ),
        DeclareLaunchArgument(
            'control', default_value='trajectory', choices=['trajectory', 'xyz', 'drum'],
            description='Use the Cartesian Action or accept manual Cartesian or drum commands.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'], description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo client when using the Gazebo backend.',
        ),
        DeclareLaunchArgument(
            'slider_gui', default_value='false', choices=['true', 'false'],
            description='Start sliders when using manual Cartesian or drum control.',
        ),
        DeclareLaunchArgument(
            'config_path', default_value=str(DEFAULT_GEOMETRY_PATH),
            description='Path to the cable geometry calibration JSON.',
        ),
        DeclareLaunchArgument(
            'trajectory_config_path',
            default_value=str(motion_share / 'config' / 'trajectory.yaml'),
            description='Path to the trajectory server ROS parameters YAML.',
        ),
        DeclareLaunchArgument(
            'simulation_config_path',
            default_value=str(sim_share / 'config' / 'simulation.yaml'),
            description='Path to the Gazebo physics settings YAML.',
        ),
        OpaqueFunction(function=create_nodes),
    ])
