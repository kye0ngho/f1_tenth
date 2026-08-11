from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share_dir = get_package_share_directory('f1tenth_kkh')

    controller = LaunchConfiguration('controller')
    nonlinear_params_file = LaunchConfiguration('nonlinear_params_file')
    mpcc_params_file = LaunchConfiguration('mpcc_params_file')
    lab8_params_file = LaunchConfiguration('lab8_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='nonlinear',
            description='Controller: nonlinear, mpcc, or lab8'
        ),
        DeclareLaunchArgument(
            'nonlinear_params_file',
            default_value=os.path.join(
                share_dir, 'config', 'nonlinear_mpc_params.yaml'),
            description='Path to nonlinear_mpc_node params.yaml'
        ),
        DeclareLaunchArgument(
            'mpcc_params_file',
            default_value=os.path.join(
                share_dir, 'config', 'mpcc_params.yaml'),
            description='Path to mpcc_node params.yaml'
        ),
        DeclareLaunchArgument(
            'lab8_params_file',
            default_value=os.path.join(
                share_dir, 'config', 'lab8_mpc_params.yaml'),
            description='Path to lab8_mpc_node params.yaml'
        ),

        Node(
            package='f1tenth_kkh',
            executable='nonlinear_mpc_node',
            name='nonlinear_mpc_node',
            output='screen',
            parameters=[nonlinear_params_file],
            condition=IfCondition(PythonExpression([
                "'", controller, "' == 'nonlinear'"
            ]))
        ),

        Node(
            package='f1tenth_kkh',
            executable='mpcc_node',
            name='mpcc_node',
            output='screen',
            parameters=[mpcc_params_file],
            condition=IfCondition(PythonExpression([
                "'", controller, "' == 'mpcc'"
            ]))
        ),

        Node(
            package='f1tenth_kkh',
            executable='lab8_mpc_node',
            name='lab8_mpc_node',
            output='screen',
            parameters=[lab8_params_file],
            condition=IfCondition(PythonExpression([
                "'", controller, "' == 'lab8'"
            ]))
        ),
    ])
