from setuptools import setup
from glob import glob
import os

package_name = 'autonomous'

setup(
    name=package_name,
    version='0.1.0',
    # control/ and planning/ stay as Python sub-packages: merging the ROS
    # packages removed a boundary that bought nothing, but the code is still
    # organised along that split.
    packages=[
        package_name,
        package_name + '.control',
        package_name + '.planning',
    ],
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
        # Config keeps one directory per original package.  control and
        # planning both ship a params.yaml, so flattening them would silently
        # drop one of the two.
        (
            os.path.join('share', package_name, 'config', 'control'),
            glob('config/control/*')
        ),
        (
            os.path.join('share', package_name, 'config', 'planning'),
            glob('config/planning/*')
        ),
        (
            os.path.join('share', package_name, 'config', 'bringup'),
            glob('config/bringup/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonbotdae',
    maintainer_email='iwagoho@gmail.com',
    description='F1TENTH planning and control nodes with their launch files.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = autonomous.control.pure_pursuit_node:main',
            'racing_v1_pp_node = autonomous.control.racing_v1_pp_node:main',
            'racing_v2_pp_node = autonomous.control.racing_v2_pp_node:main',
            'racing_v3_pp_node = autonomous.control.racing_v3_pp_node:main',
            'linear_mpc_node = autonomous.control.linear_mpc_node:main',
            'nonlinear_mpcc_node = autonomous.control.nonlinear_mpcc_node:main',
            'unicorn_l1_node = autonomous.control.unicorn_l1_node:main',
            'woong_pp_node = autonomous.control.woong_pp_node:main',
            'forza_map_node = autonomous.control.forza_map_node:main',
            'kill_switch_node = autonomous.control.kill_switch_node:main',
            'kill_switch_demo_node = autonomous.control.kill_switch_demo_node:main',
            'waypoint_planner_node = autonomous.planning.waypoint_planner_node:main',
            'local_obstacle_planner_node = autonomous.planning.local_obstacle_planner_node:main',
        ],
    },
)
