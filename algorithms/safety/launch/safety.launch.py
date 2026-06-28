from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety',
            executable='safety_brake_node',
            name='safety_brake_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'odom_topic': '/ego_racecar/odom',
                'drive_input_topic': '/control/drive',
                'drive_output_topic': '/drive',
                'brake_status_topic': '/safety/braking',
                'min_ttc': 0.5,
                'brake_distance': 0.3,
                'scan_field_deg': 60.0,
            }]
        ),
    ])
