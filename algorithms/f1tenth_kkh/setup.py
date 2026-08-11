from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'f1tenth_kkh'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
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
    maintainer='jeonbotdae',
    maintainer_email='jeonbotdae@example.com',
    description='Comparison MPC controller variants (nonlinear/SLSQP, MPCC, lab8-style) benchmarked against the tuned Linear MPC baseline in control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nonlinear_mpc_node = f1tenth_kkh.nonlinear_mpc_node:main',
            'mpcc_node = f1tenth_kkh.mpcc_node:main',
            'lab8_mpc_node = f1tenth_kkh.lab8_mpc_node:main',
        ],
    },
)
