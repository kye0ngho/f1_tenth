from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    pure_pursuit_params_file = LaunchConfiguration('pure_pursuit_params_file')
    auto_enable = LaunchConfiguration('auto_enable')
    rviz = LaunchConfiguration('rviz')
    local_replanner = LaunchConfiguration('local_replanner')
    obstacle_detection = LaunchConfiguration('obstacle_detection')
    obstacle_scan_topic = LaunchConfiguration('obstacle_scan_topic')
    obstacle_detection_range = LaunchConfiguration('obstacle_detection_range')
    obstacle_interest_horizon = LaunchConfiguration('obstacle_interest_horizon')
    obstacle_lane_offset = LaunchConfiguration('obstacle_lane_offset')
    speed_profile_max_speed = LaunchConfiguration('speed_profile_max_speed')
    speed_profile_min_speed = LaunchConfiguration('speed_profile_min_speed')
    speed_profile_lateral_accel = LaunchConfiguration('speed_profile_lateral_accel')
    speed_profile_accel = LaunchConfiguration('speed_profile_accel')
    speed_profile_decel = LaunchConfiguration('speed_profile_decel')
    avoidance_speed_cap = LaunchConfiguration('avoidance_speed_cap')
    blocked_speed_cap = LaunchConfiguration('blocked_speed_cap')
    avoidance_cap_decel_mps2 = LaunchConfiguration('avoidance_cap_decel_mps2')
    avoidance_cap_accel_mps2 = LaunchConfiguration('avoidance_cap_accel_mps2')
    stall_speed_threshold_mps = LaunchConfiguration('stall_speed_threshold_mps')
    stall_command_speed_threshold_mps = LaunchConfiguration(
        'stall_command_speed_threshold_mps')
    stall_timeout_s = LaunchConfiguration('stall_timeout_s')
    avoidance_max_lane_offset_m = LaunchConfiguration('avoidance_max_lane_offset_m')
    avoidance_min_wall_clearance_m = LaunchConfiguration(
        'avoidance_min_wall_clearance_m')
    avoidance_vehicle_width_m = LaunchConfiguration('avoidance_vehicle_width_m')
    avoidance_safety_margin_m = LaunchConfiguration('avoidance_safety_margin_m')
    avoidance_obstacle_path_clearance_m = LaunchConfiguration(
        'avoidance_obstacle_path_clearance_m')
    avoidance_ramp_in_m = LaunchConfiguration('avoidance_ramp_in_m')
    avoidance_ramp_out_m = LaunchConfiguration('avoidance_ramp_out_m')
    avoidance_state_switch_hold_s = LaunchConfiguration(
        'avoidance_state_switch_hold_s')
    avoidance_default_side = LaunchConfiguration('avoidance_default_side')
    avoidance_target_lateral_rate_limit_mps = LaunchConfiguration(
        'avoidance_target_lateral_rate_limit_mps')

    share_dir = get_package_share_directory('f1tenth_gym_ros')
    # Not share_dir/maps -- that directory doesn't exist in the installed
    # package; maps live in the docker-compose ./maps:/root/maps mount
    # shared by both the sim and real-car containers.
    default_map_yaml = '/root/maps/track02.yaml'
    default_amcl_params = os.path.join(share_dir, 'config', 'amcl.yaml')

    use_odom_relay = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'odom'"
    ]))
    use_amcl = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'amcl'"
    ]))
    start_local_replanner = IfCondition(local_replanner)
    start_obstacle_detection = IfCondition(obstacle_detection)

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_mode',
            default_value='sim',
            description='Vehicle output mode: sim or real'
        ),

        DeclareLaunchArgument(
            'waypoint_csv',
            default_value=os.path.join(
                get_package_share_directory('planning'),
                'waypoints', 'track02_raceline_safe.csv'),
            description=(
                'Path to waypoint CSV file. Was a small placeholder file '
                '(waypoints.csv, a handful of points near the origin) that '
                "did not match track02's spawn pose -- now defaults to the "
                'same validated raceline launch/gym_bridge_launch.py (sim) '
                'uses.'
            )
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

        DeclareLaunchArgument(
            'pure_pursuit_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('control'),
                'config',
                'params.yaml',
            ]),
            description=(
                'Pure pursuit parameter file -- same file tuned/validated '
                'in launch/gym_bridge_launch.py (sim) so drive_mode:=real '
                'runs with identical lookahead/speed tuning as the last '
                'sim run, instead of the separate hardcoded values this '
                'launch used to carry.'
            )
        ),

        DeclareLaunchArgument(
            'auto_enable',
            default_value='false',
            description=(
                'Let pure_pursuit_node self-enable once path/odom/TF are '
                'ready, instead of requiring a manual /control/enable '
                'service call. Defaults false -- on the real car this '
                'should stay an explicit human action; only opt in for a '
                'supervised bench/track test.'
            )
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the shared gym_bridge display config.'
        ),

        DeclareLaunchArgument(
            'local_replanner',
            default_value='true',
            description=(
                'Route /planning/path through local_avoidance_planner_node '
                '(occupancy-grid based). Required for /planning/path to be '
                'published at all -- waypoint_planner_node only publishes '
                'the raw /planning/global_path.'
            )
        ),

        DeclareLaunchArgument(
            'obstacle_detection',
            default_value='false',
            description=(
                'Start scan_obstacle_detector_node for /planning/'
                'detected_obstacle_markers diagnostics/RViz only -- '
                'local_avoidance_planner_node reads the LaserScan directly '
                'and does not depend on this node.'
            )
        ),

        DeclareLaunchArgument(
            'obstacle_scan_topic',
            default_value='/scan',
            description='LaserScan topic used by the avoidance/detection nodes.'
        ),

        DeclareLaunchArgument(
            'obstacle_detection_range',
            default_value='7.0',
            description='Maximum LaserScan range used for obstacle detection.'
        ),

        DeclareLaunchArgument(
            'obstacle_interest_horizon',
            default_value='7.5',
            description=(
                'Forward path distance where detected obstacles trigger '
                'local replanning. Must stay close to avoidance_ramp_in_m '
                '-- see the matching comment in launch/gym_bridge_launch.py.'
            )
        ),

        DeclareLaunchArgument(
            'obstacle_lane_offset',
            default_value='0.22',
            description='Lateral offset used by the local avoidance path.'
        ),

        DeclareLaunchArgument(
            'speed_profile_max_speed',
            default_value='1.20',
            description=(
                '2026-08-21: lowered from the sim-tuned 5.0 -- the '
                'official IFAC 2026 Busan track surface is hardened '
                'concrete + urethane coating (confirmed lower-friction '
                'than typical asphalt) and this stack has never been '
                'driven on it. Conservative starting point for the first '
                'supervised shakedown; raise only after empirical '
                'validation at the track (see AVOIDANCE_TUNING.md).'
            )
        ),

        DeclareLaunchArgument(
            'speed_profile_min_speed',
            default_value='0.50',
            description=(
                '2026-08-21: lowered from 0.8 alongside speed_profile_max_speed '
                '-- conservative starting point pending real-surface validation.'
            )
        ),

        DeclareLaunchArgument(
            'speed_profile_lateral_accel',
            default_value='2.50',
            description=(
                '2026-08-21: lowered from the sim-tuned 5.5 -- unvalidated '
                'on the competition\'s lower-friction real surface. '
                'Conservative starting point pending empirical validation.'
            )
        ),

        DeclareLaunchArgument(
            'speed_profile_accel',
            default_value='3.00',
            description=(
                '2026-08-21: lowered from the sim-tuned 7.0 -- longitudinal '
                'grip is also surface-limited, not just lateral. '
                'Conservative starting point pending empirical validation.'
            )
        ),

        DeclareLaunchArgument(
            'speed_profile_decel',
            default_value='3.00',
            description=(
                '2026-08-21: lowered from the sim-tuned 6.5 -- braking '
                'distance directly affects the stall-watchdog/avoidance '
                'interplay too. Conservative starting point pending '
                'empirical validation.'
            )
        ),

        DeclareLaunchArgument(
            'avoidance_speed_cap',
            default_value='1.80',
            description=(
                'pure_pursuit_node speed ceiling while /planning/replan_state '
                'is LOCAL_AVOIDANCE_* (ramped). Ported from mpcc_node.py -- '
                'unvalidated at pure_pursuit speeds.'
            )
        ),

        DeclareLaunchArgument(
            'blocked_speed_cap',
            default_value='0.0',
            description=(
                'pure_pursuit_node speed ceiling while /planning/replan_state '
                'is BLOCKED (snaps immediately, not ramped).'
            )
        ),

        DeclareLaunchArgument(
            'avoidance_cap_decel_mps2',
            default_value='6.00',
            description='Ramp-down rate toward avoidance_speed_cap.'
        ),

        DeclareLaunchArgument(
            'avoidance_cap_accel_mps2',
            default_value='5.00',
            description='Ramp-up rate back toward max_speed after avoidance clears.'
        ),

        DeclareLaunchArgument(
            'stall_speed_threshold_mps',
            default_value='0.15',
            description=(
                'pure_pursuit_node stall watchdog: measured speed below '
                'this while commanding stall_command_speed_threshold_mps+ '
                'counts as not making progress.'
            )
        ),

        DeclareLaunchArgument(
            'stall_command_speed_threshold_mps',
            default_value='0.30',
            description='Stall watchdog: minimum commanded speed to evaluate at all.'
        ),

        DeclareLaunchArgument(
            'stall_timeout_s',
            default_value='1.00',
            description='Stall watchdog: seconds of no-progress before auto-disabling.'
        ),

        DeclareLaunchArgument(
            'avoidance_max_lane_offset_m',
            default_value='0.52',
            description='local_avoidance_planner_node max_lane_offset_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_min_wall_clearance_m',
            default_value='0.18',
            description='local_avoidance_planner_node min_wall_clearance_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_vehicle_width_m',
            default_value='0.31',
            description='local_avoidance_planner_node vehicle_width_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_safety_margin_m',
            default_value='0.08',
            description='local_avoidance_planner_node safety_margin_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_obstacle_path_clearance_m',
            default_value='0.55',
            description='local_avoidance_planner_node obstacle_path_clearance_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_ramp_in_m',
            default_value='6.00',
            description='local_avoidance_planner_node ramp_in_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_ramp_out_m',
            default_value='3.00',
            description='local_avoidance_planner_node ramp_out_m.'
        ),

        DeclareLaunchArgument(
            'avoidance_state_switch_hold_s',
            default_value='0.30',
            description='local_avoidance_planner_node state_switch_hold_s.'
        ),

        DeclareLaunchArgument(
            'avoidance_default_side',
            default_value='right',
            description='local_avoidance_planner_node default_side.'
        ),

        DeclareLaunchArgument(
            'avoidance_target_lateral_rate_limit_mps',
            default_value='0.40',
            description=(
                'local_avoidance_planner_node target_lateral_rate_limit_mps '
                '-- how fast the committed lateral offset itself is '
                'allowed to slew. Lower for a gentler steer-in.'
            )
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
                'path_topic': '/planning/global_path',
                'marker_topic': '/planning/global_markers',
                'frame_id': 'map',
                'publish_rate': 2.0,
            }]
        ),

        Node(
            package='planning',
            executable='speed_profile_node',
            name='speed_profile_node',
            output='screen',
            parameters=[{
                'waypoint_csv': waypoint_csv,
                'speed_profile_topic': '/planning/speed_profile',
                'marker_topic': '/planning/speed_profile_markers',
                'frame_id': 'map',
                'publish_rate': 2.0,
                'max_speed': speed_profile_max_speed,
                'min_speed': speed_profile_min_speed,
                'lateral_accel_limit': speed_profile_lateral_accel,
                'accel_limit': speed_profile_accel,
                'decel_limit': speed_profile_decel,
                'corner_slowdown_gain': 0.20,
                'use_csv_speed_limit': False,
                'smoothing_passes': 2,
            }]
        ),

        Node(
            package='planning',
            executable='scan_obstacle_detector_node',
            name='scan_obstacle_detector_node',
            output='screen',
            condition=start_obstacle_detection,
            parameters=[{
                'scan_topic': obstacle_scan_topic,
                'path_topic': '/planning/global_path',
                'obstacle_topic': '/planning/detected_obstacles',
                'marker_topic': '/planning/detected_obstacle_markers',
                'global_frame_id': global_frame_id,
                'obstacle_corridor_m': 0.35,
                'obstacle_max_range_m': obstacle_detection_range,
            }]
        ),

        Node(
            package='planning',
            executable='local_avoidance_planner_node',
            name='local_avoidance_planner_node',
            output='screen',
            condition=start_local_replanner,
            parameters=[{
                'global_path_topic': '/planning/global_path',
                'scan_topic': obstacle_scan_topic,
                'odom_topic': localized_odom_topic,
                'output_path_topic': '/planning/path',
                'marker_topic': '/planning/local_replan_markers',
                'state_topic': '/planning/replan_state',
                'global_frame_id': global_frame_id,
                'publish_rate': 8.0,
                'interest_horizon_m': obstacle_interest_horizon,
                'lane_offset_m': obstacle_lane_offset,
                'max_lane_offset_m': avoidance_max_lane_offset_m,
                'vehicle_width_m': avoidance_vehicle_width_m,
                'obstacle_radius_m': 0.18,
                'safety_margin_m': avoidance_safety_margin_m,
                'obstacle_path_clearance_m': avoidance_obstacle_path_clearance_m,
                'min_wall_clearance_m': avoidance_min_wall_clearance_m,
                'ramp_in_m': avoidance_ramp_in_m,
                'ramp_out_m': avoidance_ramp_out_m,
                'obstacle_half_length_m': 0.35,
                'center_deadband_m': 0.18,
                'default_side': avoidance_default_side,
                # See the matching comment in launch/gym_bridge_launch.py
                # for why a too-short state_switch_hold_s broke avoidance
                # (flapped across the feasibility threshold and never
                # committed a real offset).
                'state_switch_hold_s': avoidance_state_switch_hold_s,
                'target_lateral_rate_limit_mps':
                    avoidance_target_lateral_rate_limit_mps,
            }]
        ),

        Node(
            package='control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[
                pure_pursuit_params_file,
                {
                    'drive_mode': 'sim',
                    'global_frame_id': global_frame_id,
                    'base_frame_id': base_frame_id,
                    'odom_topic': localized_odom_topic,
                    'path_topic': '/planning/path',
                    'speed_profile_topic': '/planning/speed_profile',
                    'sim_drive_topic': '/control/drive_cmd',
                    'real_speed_topic': '/unused/commands/motor/speed',
                    'real_servo_topic': '/unused/commands/servo/position',
                    # 2026-08-21: pure_pursuit_params_file's target_speed/
                    # max_speed are the SIM-tuned values (shared with
                    # gym_bridge_launch.py, now up to 8 m/s) -- without
                    # this override, the real car's hard speed ceiling
                    # and stale-profile fallback target would silently
                    # follow whatever sim is tuned to instead of staying
                    # tied to this launch's own conservative
                    # speed_profile_max_speed/min_speed (item 1).
                    'target_speed': speed_profile_max_speed,
                    'min_speed': speed_profile_min_speed,
                    'max_speed': speed_profile_max_speed,
                    'auto_enable': auto_enable,
                    'avoidance_speed_cap': avoidance_speed_cap,
                    'blocked_speed_cap': blocked_speed_cap,
                    'avoidance_cap_decel_mps2': avoidance_cap_decel_mps2,
                    'avoidance_cap_accel_mps2': avoidance_cap_accel_mps2,
                    'stall_speed_threshold_mps': stall_speed_threshold_mps,
                    'stall_command_speed_threshold_mps':
                        stall_command_speed_threshold_mps,
                    'stall_timeout_s': stall_timeout_s,
                },
            ],
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(share_dir, 'launch', 'gym_bridge.rviz')],
            condition=IfCondition(rviz)
        ),
    ])
