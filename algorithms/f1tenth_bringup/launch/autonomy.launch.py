import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drive_mode = LaunchConfiguration('drive_mode')
    waypoint_csv = LaunchConfiguration('waypoint_csv')
    localization_mode = LaunchConfiguration('localization_mode')

    loc_pkg = get_package_share_directory('localization')
    loc_params = os.path.join(loc_pkg, 'config', 'params.yaml')
    slam_params = os.path.join(loc_pkg, 'config', 'slam_toolbox_params.yaml')
    amcl_params = os.path.join(loc_pkg, 'config', 'amcl_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_mode',
            default_value='sim',
            description='Drive output mode: sim | real'
        ),

        DeclareLaunchArgument(
            'waypoint_csv',
            default_value='/sim_ws/src/planning/waypoints/waypoints.csv',
            description='Path to waypoint CSV file'
        ),

        DeclareLaunchArgument(
            'localization_mode',
            default_value='passthrough',
            description=(
                'Localization backend: '
                'passthrough(sim) | particle_filter | slam | amcl(real)'
            )
        ),

        # ── 3D→2D: 실차 전용 ─────────────────────────────────────────
        # VLP-16 기본: cloud_in = /velodyne_points  (Ouster면 /ouster/points)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('scan', '/scan'),
            ],
            parameters=[loc_params],
            condition=LaunchConfigurationEquals('drive_mode', 'real')
        ),

        # ── IMU 오도메트리 융합 (실차 전용) ──────────────────────────
        Node(
            package='localization',
            executable='imu_odometry_node',
            name='imu_odometry_node',
            output='screen',
            parameters=[loc_params],
            condition=LaunchConfigurationEquals('drive_mode', 'real')
        ),

        # ── Localization: passthrough ─────────────────────────────────
        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{
                'input_odom_topic': '/ego_racecar/odom',
                'output_odom_topic': '/localization/odom',
                'output_pose_topic': '/localization/pose',
            }],
            condition=LaunchConfigurationEquals('localization_mode', 'passthrough')
        ),

        # ── Localization: particle_filter (커스텀 MCL) ────────────────
        Node(
            package='localization',
            executable='map_server_node',
            name='map_server_node',
            output='screen',
            parameters=[{
                'map_yaml': '/sim_ws/src/localization/maps/map.yaml',
                'map_topic': '/map',
                'frame_id': 'map',
            }],
            condition=LaunchConfigurationEquals('localization_mode', 'particle_filter')
        ),
        Node(
            package='localization',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'scan_topic': '/scan',
                'odom_topic': '/ego_racecar/odom',
                'output_odom_topic': '/localization/odom',
                'output_pose_topic': '/localization/pose',
                'particle_cloud_topic': '/localization/particles',
                'frame_id': 'map',
                'num_particles': 500,
                'downsample_beams': 20,
                'max_range': 10.0,
                'alpha1': 0.1,
                'alpha2': 0.1,
                'alpha3': 0.05,
                'alpha4': 0.05,
                'z_hit': 0.8,
                'z_rand': 0.2,
                'sigma_hit': 0.15,
                'initial_spread_xy': 0.5,
                'initial_spread_theta': 0.3,
            }],
            condition=LaunchConfigurationEquals('localization_mode', 'particle_filter')
        ),

        # ── Localization: slam (SLAM Toolbox 온라인 매핑) ─────────────
        # 지도 저장: ros2 service call /slam_toolbox/save_map ...
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

        # ── Localization: amcl (저장된 지도 + nav2_amcl) ──────────────
        # 전제: SLAM으로 저장한 map.yaml 존재, RViz에서 initial pose 설정
        Node(
            package='localization',
            executable='map_server_node',
            name='map_server_node',
            output='screen',
            parameters=[{
                'map_yaml': '/sim_ws/src/localization/maps/map.yaml',
                'map_topic': '/map',
                'frame_id': 'map',
            }],
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
            parameters=[{
                'amcl_pose_topic': '/amcl_pose',
                'raw_odom_topic': '/ego_racecar/odom',
                'output_odom_topic': '/localization/odom',
                'output_pose_topic': '/localization/pose',
                'map_frame': 'map',
                'base_frame': 'base_link',
            }],
            condition=LaunchConfigurationEquals('localization_mode', 'amcl')
        ),

        # ── Path Smoother (컨트롤러에 부드러운 경로 제공) ────────────
        Node(
            package='planning',
            executable='path_smoother_node',
            name='path_smoother_node',
            output='screen',
            parameters=[{
                'input_topic': '/planning/path',
                'output_topic': '/planning/smooth_path',
                'output_points': 300,
            }]
        ),

        # ── Planning ──────────────────────────────────────────────────
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
            package='planning',
            executable='lap_timer_node',
            name='lap_timer_node',
            output='screen',
            parameters=[{
                'odom_topic': '/ego_racecar/odom',
                'trigger_radius': 0.5,
                'min_lap_distance': 2.0,
            }]
        ),

        # ── Control ───────────────────────────────────────────────────
        # MPC node (use_mpc:=true 시 behavior_selector의 waypoint 입력으로 사용)
        Node(
            package='control',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            parameters=[{
                'odom_topic': '/localization/odom',
                'path_topic': '/planning/path',
                'drive_topic': '/mpc/drive',
                'pred_path_topic': '/mpc/predicted_path',
                'frame_id': 'map',
                'wheelbase': 0.33,
                'dt': 0.1,
                'horizon': 10,
                'max_steering': 0.4189,
                'max_speed': 2.0,
                'min_speed': 0.3,
                'max_accel': 2.0,
                'target_speed': 1.5,
                'w_cte': 2.0,
                'w_eth': 1.5,
                'w_v': 0.5,
                'w_delta': 0.1,
                'w_a': 0.05,
                'w_ddelta': 2.0,
                'w_da': 0.5,
                'control_rate': 20.0,
            }]
        ),

        # pure_pursuit → /pure_pursuit/drive
        Node(
            package='control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'drive_mode': drive_mode,

                'odom_topic': '/localization/odom',
                'path_topic': '/planning/path',

                'sim_drive_topic': '/pure_pursuit/drive',

                'real_speed_topic': '/commands/motor/speed',
                'real_servo_topic': '/commands/servo/position',

                'wheelbase': 0.33,
                'lookahead_distance': 1.0,
                'max_steering_angle': 0.4189,

                'target_speed': 1.0,
                'min_speed': 0.4,
                'max_speed': 2.0,
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

        # gap_follow → /gap_follow/drive
        Node(
            package='control',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'odom_topic': '/ego_racecar/odom',
                'drive_topic': '/gap_follow/drive',
                'max_range': 3.0,
                'safety_bubble': 0.3,
                'scan_field_deg': 180.0,
                'min_gap_size': 3,
                'max_speed': 1.5,
                'min_speed': 0.5,
                'corner_slowdown_gain': 1.5,
                'max_steering_angle': 0.4189,
                'wheelbase': 0.33,
            }]
        ),

        # behavior_selector: WAYPOINT / GAP_FOLLOW / OVERTAKE → /control/drive
        Node(
            package='control',
            executable='behavior_selector_node',
            name='behavior_selector_node',
            output='screen',
            parameters=[{
                'waypoint_drive_topic': '/pure_pursuit/drive',
                'gap_drive_topic': '/gap_follow/drive',
                'overtake_drive_topic': '/adaptive_pp/drive',
                'overtake_active_topic': '/overtake/active',
                'output_drive_topic': '/control/drive',
                'scan_topic': '/scan',
                'mode_topic': '/behavior/mode',
                'obstacle_range': 1.5,
                'obstacle_clear_range': 2.0,
                'scan_field_deg': 60.0,
            }]
        ),

        # ── Safety ────────────────────────────────────────────────────
        # /scan → scan_preprocessor → /scan_processed
        Node(
            package='safety',
            executable='scan_preprocessor_node',
            name='scan_preprocessor_node',
            output='screen',
            parameters=[{
                'scan_input_topic': '/scan',
                'scan_output_topic': '/scan_processed',
                'median_window': 3,
                'range_min_override': 0.05,
                'range_max_override': 20.0,
            }]
        ),

        # /emergency/drive + /scan + /watchdog/alert → safety_brake_node → /drive (최종, 단일 퍼블리셔)
        Node(
            package='safety',
            executable='safety_brake_node',
            name='safety_brake_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'odom_topic': '/ego_racecar/odom',
                'drive_input_topic': '/emergency/drive',
                'drive_output_topic': '/drive',
                'brake_status_topic': '/safety/braking',
                'watchdog_alert_topic': '/watchdog/alert',
                'min_ttc': 0.5,
                'brake_distance': 0.3,
                'scan_field_deg': 60.0,
            }]
        ),

        # 토픽 헬스 감시 → /watchdog/alert (Bool). 실제 estop은 safety_brake_node가 처리
        Node(
            package='safety',
            executable='watchdog_node',
            name='watchdog_node',
            output='screen',
            parameters=[{
                'alert_topic': '/watchdog/alert',
                'odom_timeout': 1.0,
                'path_timeout': 3.0,
                'drive_timeout': 1.0,
            }]
        ),

        # ── Debug / Logging ───────────────────────────────────────────
        Node(
            package='control',
            executable='debug_marker_node',
            name='debug_marker_node',
            output='screen',
            parameters=[{
                'output_topic': '/debug/markers',
                'frame_id': 'map',
                'scan_frame_id': 'ego_racecar/laser',
                'publish_rate': 20.0,
                'scan_danger_range': 1.0,
            }]
        ),

        Node(
            package='planning',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            parameters=[{
                'odom_topic': '/ego_racecar/odom',
                'drive_topic': '/drive',
                'braking_topic': '/safety/braking',
                'mode_topic': '/behavior/mode',
                'log_dir': '/sim_ws/src/planning/logs',
                'log_rate': 20.0,
                'auto_start': True,
            }]
        ),

        Node(
            package='planning',
            executable='goal_pose_planner_node',
            name='goal_pose_planner_node',
            output='screen',
            parameters=[{
                'odom_topic': '/ego_racecar/odom',
                'goal_topic': '/goal_pose',
                'path_topic': '/planning/goal_path',
                'status_topic': '/goal_planner/status',
                'frame_id': 'map',
                'num_points': 50,
                'goal_radius': 0.3,
            }]
        ),

        Node(
            package='planning',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{
                'log_dir': '/sim_ws/src/planning/logs',
                'publish_rate': 1.0,
                'enable_file_log': True,
            }]
        ),

        Node(
            package='planning',
            executable='sector_timer_node',
            name='sector_timer_node',
            output='screen',
            parameters=[{
                'sector_csv': '/sim_ws/src/planning/waypoints/sectors.csv',
                'default_radius': 1.0,
                'log_dir': '/sim_ws/src/planning/logs',
            }]
        ),

        # ── 상대 추적 / 복구 ──────────────────────────────────────────
        Node(
            package='safety',
            executable='opponent_tracker_node',
            name='opponent_tracker_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'max_range': 6.0,
                'cluster_dist': 0.3,
            }]
        ),

        # /control/drive → emergency_recovery (패스스루+복구 override) → /emergency/drive
        Node(
            package='safety',
            executable='emergency_recovery_node',
            name='emergency_recovery_node',
            output='screen',
            parameters=[{
                'odom_topic': '/ego_racecar/odom',
                'drive_input_topic': '/control/drive',
                'drive_output_topic': '/emergency/drive',
                'stuck_detect_time': 2.0,
                'reverse_speed': -0.5,
                'reverse_time': 1.5,
            }]
        ),

        # ── 적응형 Pure Pursuit (오버테이크 시 /planning/routed_path 자동 전환) ─
        Node(
            package='control',
            executable='adaptive_pure_pursuit_node',
            name='adaptive_pure_pursuit_node',
            output='screen',
            parameters=[{
                'odom_topic': '/localization/odom',
                'path_topic': '/planning/path',
                'overtake_path_topic': '/planning/routed_path',
                'overtake_active_topic': '/overtake/active',
                'drive_topic': '/adaptive_pp/drive',
                'wheelbase': 0.33,
                'L_min': 0.5,
                'L_max': 3.0,
                'k_v': 0.6,
                'target_speed': 1.5,
                'min_speed': 0.4,
                'max_speed': 2.5,
            }]
        ),

        # ── Corridor Follow (웨이포인트 없이 벽 중앙 추종) ────────────
        Node(
            package='control',
            executable='corridor_follow_node',
            name='corridor_follow_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'drive_topic': '/corridor/drive',
                'k_p': 0.8,
                'max_speed': 2.0,
                'min_speed': 0.5,
            }]
        ),

        # ── Disparity Extender (선택적: gap_follow 대체) ──────────────
        Node(
            package='control',
            executable='disparity_extender_node',
            name='disparity_extender_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'drive_topic': '/disparity_ext/drive',
                'car_width': 0.30,
                'max_range': 4.0,
                'disparity_threshold': 0.5,
                'max_speed': 2.0,
                'min_speed': 0.5,
            }]
        ),

        # ── Stanley Controller (선택적: pure_pursuit 대체) ────────────
        Node(
            package='control',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[{
                'odom_topic': '/localization/odom',
                'path_topic': '/planning/path',
                'drive_topic': '/stanley/drive',
                'wheelbase': 0.33,
                'k': 1.0,
                'k_soft': 0.3,
                'target_speed': 1.5,
                'min_speed': 0.4,
                'max_speed': 2.5,
            }]
        ),

        # ── 오버테이크 / 평가 / 레이싱 라인 ─────────────────────────
        Node(
            package='planning',
            executable='overtake_planner_node',
            name='overtake_planner_node',
            output='screen',
            parameters=[{
                'path_topic': '/planning/path',
                'output_topic': '/planning/routed_path',
                'obstacles_dist_topic': '/obstacles/nearest_dist',
                'overtake_trigger_dist': 2.5,
                'lateral_offset': 0.6,
            }]
        ),

        Node(
            package='planning',
            executable='trajectory_evaluator_node',
            name='trajectory_evaluator_node',
            output='screen',
            parameters=[{
                'odom_topic': '/ego_racecar/odom',
                'path_topic': '/planning/path',
                'target_speed': 1.5,
            }]
        ),

        Node(
            package='planning',
            executable='race_line_optimizer_node',
            name='race_line_optimizer_node',
            output='screen',
            parameters=[{
                'input_csv': '/sim_ws/src/planning/waypoints/waypoints.csv',
                'output_csv': '/sim_ws/src/planning/waypoints/race_line.csv',
                'half_track_width': 0.8,
                'auto_optimize_on_start': False,
            }]
        ),
    ])
