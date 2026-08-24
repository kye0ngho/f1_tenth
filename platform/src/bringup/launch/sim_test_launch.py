import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    # include f1tenth_gym_ros launch file
    launch_f1tenth_gym_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_gym_ros'),
                'launch/gym_bridge_launch.py'))
    )

    # include particle_filter launch file
    launch_particle_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('particle_filter'),
                'launch/localize_launch.py'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('bringup'),
                'launch/pf_mpc.rviz')]
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        remappings=[('ackermann_cmd', 'drive')]
    )

    safety_node = Node(
        package='safety_node',
        executable='safety_node',
        name='safety_node',
        remappings=[('scan', 'sim/scan'), ('ego_racecar/odom', 'sim/ego_racecar/odom')]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'sim', 'map']
    )

    return LaunchDescription([
        launch_particle_filter,
        # rviz_node,
        ackermann_mux_node,
        safety_node,
        static_tf_node,
        launch_f1tenth_gym_ros,
    ])
