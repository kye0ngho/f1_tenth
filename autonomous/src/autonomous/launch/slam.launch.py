"""Map building with slam_toolbox.

slam_toolbox is used from the ROS binary package; the 53MB source checkout it
replaced was never modified in any way this stack depended on.  Only the mapper
presets are ours, and they live in config/slam.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# preset -> config/slam/mapper_<preset>.yaml
PRESETS = ('loop', 'noloop', 'highres')

# mode -> slam_toolbox executable.  The binary package ships no separate
# offline node; its own offline_launch.py runs the sync node too, and offline
# differs only in following the bag clock.
MODES = {
    'online': 'async_slam_toolbox_node',
    'sync': 'sync_slam_toolbox_node',
    'offline': 'sync_slam_toolbox_node',
}


def _setup(context):
    preset = LaunchConfiguration('preset').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    if preset not in PRESETS:
        raise RuntimeError(
            'preset must be one of %s; got %r' % (', '.join(PRESETS), preset))
    if mode not in MODES:
        raise RuntimeError(
            'mode must be one of %s; got %r' % (', '.join(sorted(MODES)), mode))

    params = os.path.join(
        get_package_share_directory('autonomous'),
        'config', 'slam', 'mapper_%s.yaml' % preset)

    # offline mode replays a bag, so it follows the bag clock.
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    if use_sim_time == 'auto':
        use_sim_time = 'true' if mode == 'offline' else 'false'

    return [
        Node(
            package='slam_toolbox',
            executable=MODES[mode],
            name='slam_toolbox',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time == 'true'}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'preset', default_value='loop',
            description='mapper preset: loop (loop closing), '
                        'noloop, or highres (0.02m resolution)'),
        DeclareLaunchArgument(
            'mode', default_value='online',
            description='online (async), sync, or offline (replay a bag)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='auto',
            description='auto is true for offline, false otherwise'),
        OpaqueFunction(function=_setup),
    ])
