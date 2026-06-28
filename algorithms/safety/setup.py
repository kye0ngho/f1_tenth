from setuptools import setup
from glob import glob
import os

package_name = 'safety'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='ssi45di@gmail.com',
    description='LiDAR-based safety brake for F1TENTH autonomy stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_brake_node = safety.safety_brake_node:main',
            'scan_preprocessor_node = safety.scan_preprocessor_node:main',
            'watchdog_node = safety.watchdog_node:main',
            'opponent_tracker_node = safety.opponent_tracker_node:main',
            'emergency_recovery_node = safety.emergency_recovery_node:main',
        ],
    },
)
