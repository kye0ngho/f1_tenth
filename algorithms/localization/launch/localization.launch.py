import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('localization')
    params_file = os.path.join(pkg, 'config', 'params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    amcl_params = os.path.join(pkg, 'config', 'amcl_params.yaml')

    localization_mode = LaunchConfiguration('localization_mode')
    drive_mode = LaunchConfiguration('drive_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_mode',
            default_value='passthrough',
            description=(
                'Localization backend: '
                'passthrough | particle_filter | slam | amcl'
            )
        ),
        DeclareLaunchArgument(
            'drive_mode',
            default_value='sim',
            description='sim | real'
        ),

        # ── 3D→2D: 실차 전용 (drive_mode:=real 시에만 기동) ──────────
        # VLP-16 / Ouster PointCloud2 → /scan (LaserScan)
        # 입력 토픽: /velodyne_points 또는 /ouster/points → remapping 필요
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/velodyne_points'),   # VLP-16 기본값
                ('scan', '/scan'),
            ],
            parameters=[params_file],
            condition=LaunchConfigurationEquals('drive_mode', 'real')
        ),

        # ── passthrough: 시뮬 odom 그대로 전달 ────────────────────────
        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('localization_mode', 'passthrough')
        ),

        # ── particle_filter: 기존 커스텀 MCL ──────────────────────────
        Node(
            package='localization',
            executable='map_server_node',
            name='map_server_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('localization_mode', 'particle_filter')
        ),
        Node(
            package='localization',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('localization_mode', 'particle_filter')
        ),

        # ── slam: SLAM Toolbox 온라인 매핑 ────────────────────────────
        # 지도 저장: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
        # 시뮬: odom 프레임이 없어 ground-truth TF(map→ego_racecar/base_link)를
        #        odom 대용으로 사용, 지도는 /slam_map으로 분리(시뮬 /map과 충돌 방지)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {
                'base_frame': 'ego_racecar/base_link',
                'odom_frame': 'map',
                'map_frame': 'slam_map',
            }],
            remappings=[('/map', '/slam_map')],
            condition=IfCondition(PythonExpression([
                "'", localization_mode, "' == 'slam' and '",
                drive_mode, "' == 'sim'"
            ]))
        ),
        # 실차: VESC odom TF(odom→base_link) 전제
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            condition=IfCondition(PythonExpression([
                "'", localization_mode, "' == 'slam' and '",
                drive_mode, "' == 'real'"
            ]))
        ),

        # ── amcl: nav2_amcl + amcl_bridge ─────────────────────────────
        # 전제: map.yaml 로드 완료 + RViz에서 initial pose 설정
        Node(
            package='localization',
            executable='map_server_node',
            name='map_server_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('localization_mode', 'amcl')
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],
            condition=LaunchConfigurationEquals('localization_mode', 'amcl')
        ),
        # nav2_amcl은 lifecycle 노드 — configure/activate 자동화 필수
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['amcl'],
            }],
            condition=LaunchConfigurationEquals('localization_mode', 'amcl')
        ),
        Node(
            package='localization',
            executable='amcl_bridge_node',
            name='amcl_bridge_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('localization_mode', 'amcl')
        ),

        # ── IMU 오도메트리 융합 (실차 권장, 모드 무관) ────────────────
        Node(
            package='localization',
            executable='imu_odometry_node',
            name='imu_odometry_node',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('drive_mode', 'real')
        ),
    ])
