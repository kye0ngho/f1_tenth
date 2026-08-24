import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

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

    return LaunchDescription([
        launch_particle_filter,
        launch_f1tenth_gym_ros,
        rviz_node,
    ])
