from setuptools import setup
from glob import glob
import os

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'waypoints'),
            glob('waypoints/*.csv')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonbotdae',
    maintainer_email='jeonbotdae@example.com',
    description='Waypoint path publisher for F1TENTH planning.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_planner_node = planning.waypoint_planner_node:main',
            'waypoint_recorder_node = planning.waypoint_recorder_node:main',
            'velocity_profile_node = planning.velocity_profile_node:main',
            'lap_timer_node = planning.lap_timer_node:main',
            'data_logger_node = planning.data_logger_node:main',
            'waypoint_manager_node = planning.waypoint_manager_node:main',
            'goal_pose_planner_node = planning.goal_pose_planner_node:main',
            'telemetry_node = planning.telemetry_node:main',
            'sector_timer_node = planning.sector_timer_node:main',
            'race_line_optimizer_node = planning.race_line_optimizer_node:main',
            'overtake_planner_node = planning.overtake_planner_node:main',
            'trajectory_evaluator_node = planning.trajectory_evaluator_node:main',
        ],
    },
)
