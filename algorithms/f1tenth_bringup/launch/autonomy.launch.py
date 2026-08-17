from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    drive_mode = LaunchConfiguration('drive_mode')
    waypoint_csv = LaunchConfiguration('waypoint_csv')
    localization_mode = LaunchConfiguration('localization_mode')
    map_yaml = LaunchConfiguration('map_yaml')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    source_odom_topic = LaunchConfiguration('source_odom_topic')
    localized_odom_topic = LaunchConfiguration('localized_odom_topic')
    global_frame_id = LaunchConfiguration('global_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    share_dir = get_package_share_directory('f1tenth_gym_ros')
    default_map_yaml = os.path.join(share_dir, 'maps', 'track02.yaml')
    default_amcl_params = os.path.join(share_dir, 'config', 'amcl.yaml')

    use_odom_relay = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'odom'"
    ]))
    use_amcl = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'amcl'"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_mode',
            default_value='sim',
            description='Vehicle output mode: sim or real'
        ),

        DeclareLaunchArgument(
            'waypoint_csv',
            default_value='/sim_ws/src/planning/waypoints/waypoints.csv',
            description='Path to waypoint CSV file'
        ),

        DeclareLaunchArgument(
            'localization_mode',
            default_value='amcl',
            description='Localization mode: amcl or odom'
        ),

        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Map yaml for nav2_map_server when localization_mode:=amcl'
        ),

        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=default_amcl_params,
            description='nav2_amcl parameter yaml'
        ),

        DeclareLaunchArgument(
            'source_odom_topic',
            default_value='/ego_racecar/odom',
            description='Raw wheel/sim odometry topic used for twist'
        ),

        DeclareLaunchArgument(
            'localized_odom_topic',
            default_value='/car_state/odom',
            description='Standard localized odometry topic for controllers'
        ),

        DeclareLaunchArgument(
            'global_frame_id',
            default_value='map',
            description='Global localization frame'
        ),

        DeclareLaunchArgument(
            'base_frame_id',
            default_value='ego_racecar/base_link',
            description='Vehicle base frame'
        ),

        # Fallback/debug localization: raw odom relay only. Competition and
        # real-car runs should use localization_mode:=amcl.
        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            condition=use_odom_relay,
            parameters=[{
                'input_odom_topic': source_odom_topic,
                'output_odom_topic': localized_odom_topic,
                'output_pose_topic': '/localization/pose',
            }]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=use_amcl,
            parameters=[{
                'yaml_filename': map_yaml,
                'topic': 'map',
                'frame_id': 'map',
                'use_sim_time': False,
            }]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=use_amcl,
            parameters=[amcl_params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=use_amcl,
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]
        ),

        Node(
            package='localization',
            executable='localized_odom_node',
            name='localized_odom_node',
            output='screen',
            condition=use_amcl,
            parameters=[{
                'source_odom_topic': source_odom_topic,
                'amcl_pose_topic': '/amcl_pose',
                'output_odom_topic': localized_odom_topic,
                'global_frame_id': global_frame_id,
                'base_frame_id': base_frame_id,
                'publish_rate': 50.0,
                'odom_timeout': 0.50,
                'amcl_timeout': 2.00,
                'require_amcl_pose': False,
            }]
        ),

        Node(
            package='planning',
            executable='waypoint_planner_node',
            name='waypoint_planner_node',
            output='screen',
            parameters=[{
                'waypoint_csv': waypoint_csv,
                'path_topic': '/planning/path',
                'marker_topic': '/planning/markers',
                'frame_id': 'map',
                'publish_rate': 2.0,
            }]
        ),

        Node(
            package='control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'drive_mode': 'sim',
                'global_frame_id': global_frame_id,
                'base_frame_id': base_frame_id,
                'odom_topic': localized_odom_topic,
                'path_topic': '/planning/path',
                'sim_drive_topic': '/control/drive_cmd',
                'real_speed_topic': '/unused/commands/motor/speed',
                'real_servo_topic': '/unused/commands/servo/position',
                'wheelbase': 0.33,
                'lookahead_distance': 1.0,
                'max_steering_angle': 0.4189,
                'target_speed': 1.0,
                'min_speed': 0.0,
                'max_speed': 1.0,
                'corner_slowdown_gain': 0.5,
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.05,
                'speed_to_erpm_gain': 3000.0,
                'speed_to_erpm_offset': 0.0,
                'servo_center': 0.5,
                'servo_gain': 1.0,
                'servo_min': 0.0,
                'servo_max': 1.0,
                'control_rate': 30.0,
            }]
        ),

        Node(
            package='vehicle_interface',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
            parameters=[{
                'drive_mode': drive_mode,
                'input_drive_topic': '/control/drive_cmd',
                'sim_drive_topic': '/drive',
                'real_speed_topic': '/commands/motor/speed',
                'real_servo_topic': '/commands/servo/position',
                'safety_stop_topic': '/safety/stop_required',
                'use_safety_stop': True,
                'watchdog_timeout': 0.5,
                'publish_rate': 50.0,
                'speed_to_erpm_gain': 3000.0,
                'speed_to_erpm_offset': 0.0,
                'servo_center': 0.5,
                'servo_gain': 1.0,
                'servo_min': 0.0,
                'servo_max': 1.0,
            }]
        ),
    ])
