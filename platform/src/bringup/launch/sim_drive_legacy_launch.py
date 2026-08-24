import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'joy_teleop.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'mux.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs in sim')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs in sim')

    ld = LaunchDescription([joy_la, mux_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd', 'drive')]
    )
    safety_node = Node(
        package='safety_node',
        executable='safety_node_legacy',
        name='safety_node',
        remappings=[('scan', 'sim/scan'), ('ego_racecar/odom', 'sim/ego_racecar/odom')]
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(safety_node)

    return ld
