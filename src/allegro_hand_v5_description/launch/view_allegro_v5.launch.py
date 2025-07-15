import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Evaluate launch arguments at runtime
    hand = LaunchConfiguration('HAND').perform(context)
    hand_type = LaunchConfiguration('TYPE').perform(context)
    urdf_model = LaunchConfiguration('urdf_model').perform(context)
    rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'

    pkg_share = get_package_share_directory('allegro_hand_v5_description')

    # Build URDF path
    urdf_path = os.path.join(pkg_share, f'urdf/allegro_hand_description_{hand}_{hand_type}.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    nodes = []

    if gui:
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ))
    else:
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ))

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    ))

    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('allegro_hand_v5_description')
    default_urdf_path = os.path.join(pkg_share, 'urdf/allegro_hand_description_right_B.urdf')
    default_rviz_path = os.path.join(pkg_share, 'rviz/allegro_hand.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('HAND', default_value='right', description='Select hand: left or right'),
        DeclareLaunchArgument('TYPE', default_value='B', description='Select type: A or B'),
        DeclareLaunchArgument('urdf_model', default_value=default_urdf_path, description='URDF model path'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_path, description='RViz config path'),
        DeclareLaunchArgument('gui', default_value='true', description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if available'),
        OpaqueFunction(function=launch_setup)
    ])

