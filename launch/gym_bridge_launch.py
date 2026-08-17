# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    autonomy = LaunchConfiguration('autonomy')
    auto_enable = LaunchConfiguration('auto_enable')
    reset_pose = LaunchConfiguration('reset_pose')
    rviz = LaunchConfiguration('rviz')
    localization = LaunchConfiguration('localization')
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    waypoint_csv = LaunchConfiguration('waypoint_csv')
    mpcc_params_file = LaunchConfiguration('mpcc_params_file')
    speed_profile_max_speed = LaunchConfiguration('speed_profile_max_speed')
    speed_profile_min_speed = LaunchConfiguration('speed_profile_min_speed')
    speed_profile_lateral_accel = LaunchConfiguration('speed_profile_lateral_accel')
    speed_profile_accel = LaunchConfiguration('speed_profile_accel')
    speed_profile_decel = LaunchConfiguration('speed_profile_decel')
    virtual_obstacles = LaunchConfiguration('virtual_obstacles')
    virtual_obstacle_list = LaunchConfiguration('virtual_obstacle_list')
    local_replanner = LaunchConfiguration('local_replanner')
    obstacle_detection = LaunchConfiguration('obstacle_detection')
    obstacle_scan_topic = LaunchConfiguration('obstacle_scan_topic')
    obstacle_detection_range = LaunchConfiguration('obstacle_detection_range')
    obstacle_interest_horizon = LaunchConfiguration('obstacle_interest_horizon')
    obstacle_lane_offset = LaunchConfiguration('obstacle_lane_offset')

    start_localization = PythonExpression([
        "'", localization, "' == 'amcl'"
    ])
    start_planner = PythonExpression([
        "'", autonomy, "' == 'true' or '", planner,
        "' == 'waypoint'"
    ])
    start_mpcc = PythonExpression([
        "'", autonomy, "' == 'true' or '", controller, "' == 'mpcc'"
    ])
    start_local_replanner = PythonExpression([
        "('", autonomy, "' == 'true' or '", planner,
        "' == 'waypoint') and '", local_replanner, "' == 'true'"
    ])
    start_obstacle_detection = PythonExpression([
        "('", autonomy, "' == 'true' or '", planner,
        "' == 'waypoint') and '", obstacle_detection, "' == 'true'"
    ])
    start_reset = PythonExpression([
        "'", reset_pose, "' == 'true' and ('", autonomy,
        "' == 'true' or '", controller, "' != 'none')"
    ])
    start_pose_msg = (
        '{header: {frame_id: map}, pose: {pose: {position: '
        '{x: 0.2985288, y: 0.5926084, z: 0.0}, orientation: '
        '{x: 0.0, y: 0.0, z: -0.3053138966348658, '
        'w: 0.9522517653024511}}, covariance: [0.05, 0, 0, 0, '
        '0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, '
        '0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0.0, 0, '
        '0, 0, 0, 0, 0, 0.05]}}'
    )

    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    amcl_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'amcl.yaml'
        )
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'launch',
            'gym_bridge.rviz')],
        condition=IfCondition(rviz)
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': False}],
        condition=IfCondition(start_localization)
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
        condition=IfCondition(start_localization)
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}],
        condition=IfCondition(start_localization)
    )

    localized_odom_node = Node(
        package='localization',
        executable='localized_odom_node',
        name='localized_odom_node',
        output='screen',
        parameters=[{
            'source_odom_topic': '/ego_racecar/odom',
            'amcl_pose_topic': '/amcl_pose',
            'output_odom_topic': '/car_state/odom',
            'global_frame_id': 'map',
            'base_frame_id': 'ego_racecar/base_link',
            'publish_rate': 50.0,
            'odom_timeout': 0.50,
            'amcl_timeout': 2.00,
            'require_amcl_pose': False,
        }],
        condition=IfCondition(start_localization)
    )

    waypoint_planner_node = Node(
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
        }],
        condition=IfCondition(start_planner)
    )

    speed_profile_node = Node(
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
        }],
        condition=IfCondition(start_planner)
    )

    virtual_obstacle_scan_node = Node(
        package='planning',
        executable='virtual_obstacle_scan_node',
        name='virtual_obstacle_scan_node',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_with_obstacle',
            'marker_topic': '/planning/virtual_obstacles',
            'global_frame_id': 'map',
            'obstacles': virtual_obstacle_list,
        }],
        condition=IfCondition(virtual_obstacles)
    )

    scan_obstacle_detector_node = Node(
        package='planning',
        executable='scan_obstacle_detector_node',
        name='scan_obstacle_detector_node',
        output='screen',
        parameters=[{
            'scan_topic': obstacle_scan_topic,
            'path_topic': '/planning/global_path',
            'obstacle_topic': '/planning/detected_obstacles',
            'marker_topic': '/planning/detected_obstacle_markers',
            'global_frame_id': 'map',
            'obstacle_corridor_m': 0.35,
            'obstacle_max_range_m': obstacle_detection_range,
        }],
        condition=IfCondition(start_obstacle_detection)
    )

    local_avoidance_planner_node = Node(
        package='planning',
        executable='local_avoidance_planner_node',
        name='local_avoidance_planner_node',
        output='screen',
        parameters=[{
            'global_path_topic': '/planning/global_path',
            'obstacle_topic': '/planning/detected_obstacles',
            'odom_topic': '/car_state/odom',
            'output_path_topic': '/planning/path',
            'marker_topic': '/planning/local_replan_markers',
            'state_topic': '/planning/replan_state',
            'publish_rate': 8.0,
            'interest_horizon_m': obstacle_interest_horizon,
            'obstacle_corridor_m': 0.35,
            'lane_offset_m': obstacle_lane_offset,
            'max_lane_offset_m': 0.52,
            'vehicle_width_m': 0.31,
            'obstacle_radius_m': 0.18,
            'safety_margin_m': 0.08,
            'obstacle_path_clearance_m': 0.40,
            'min_wall_clearance_m': 0.18,
            'ramp_in_m': 1.80,
            'ramp_out_m': 2.40,
            'obstacle_half_length_m': 0.35,
            'center_deadband_m': 0.18,
            'default_side': 'right',
        }],
        condition=IfCondition(start_local_replanner)
    )

    mpcc_node = Node(
        package='f1tenth_kkh',
        executable='mpcc_node',
        name='mpcc_node',
        output='screen',
        parameters=[mpcc_params_file],
        condition=IfCondition(start_mpcc)
    )

    reset_sim_pose_action = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--times',
                    '5',
                    '--rate',
                    '5',
                    '/sim_reset_pose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    start_pose_msg,
                ],
                output='screen',
                condition=IfCondition(start_reset),
            )
        ],
        condition=IfCondition(start_reset)
    )

    reset_amcl_pose_action = TimerAction(
        period=2.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--times',
                    '5',
                    '--rate',
                    '5',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    start_pose_msg,
                ],
                output='screen',
                condition=IfCondition(start_reset),
            )
        ],
        condition=IfCondition(start_reset)
    )

    enable_mpcc_action = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/control/enable',
                    'std_srvs/srv/SetBool',
                    '{data: true}',
                ],
                output='screen',
                condition=IfCondition(auto_enable),
            )
        ],
        condition=IfCondition(start_mpcc)
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    # finalize
    ld.add_action(DeclareLaunchArgument(
        'autonomy',
        default_value='false',
        description='Start waypoint planner and MPCC with the simulator.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Call /control/enable automatically after startup.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the full-stack display config.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'localization',
        default_value='amcl',
        description='Localization module: amcl or none.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'planner',
        default_value='none',
        description='Planner module: waypoint or none. autonomy:=true also starts waypoint.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'controller',
        default_value='none',
        description='Controller module: mpcc or none. autonomy:=true also starts mpcc.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'reset_pose',
        default_value='true',
        description='Reset simulator and AMCL initial pose to the raceline start.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'waypoint_csv',
        default_value=PathJoinSubstitution([
            FindPackageShare('planning'),
            'waypoints',
            'track02_raceline_safe.csv',
        ]),
        description='Raceline CSV published to /planning/global_path.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'mpcc_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth_kkh'),
            'config',
            'mpcc_params.yaml',
        ]),
        description='MPCC parameter file.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_profile_max_speed',
        default_value='3.5',
        description='Straight-line cap for /planning/speed_profile.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_profile_min_speed',
        default_value='0.8',
        description='Minimum speed in /planning/speed_profile.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_profile_lateral_accel',
        default_value='4.2',
        description='Lateral acceleration limit used by speed profile.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_profile_accel',
        default_value='5.5',
        description='Forward acceleration limit used by speed profile.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_profile_decel',
        default_value='4.8',
        description='Backward braking limit used by speed profile.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'virtual_obstacles',
        default_value='false',
        description='Inject map-frame circular obstacles into /scan.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'virtual_obstacle_list',
        default_value='6.74,0.41,0.18',
        description='Semicolon-separated x,y,r circles in map frame.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'local_replanner',
        default_value='true',
        description='Publish /planning/path through the local avoidance replanner.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'obstacle_detection',
        default_value='false',
        description='Detect on-track obstacles from LaserScan.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'obstacle_scan_topic',
        default_value='/scan',
        description='LaserScan topic used by scan_obstacle_detector_node.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'obstacle_detection_range',
        default_value='7.0',
        description='Maximum LaserScan range used for obstacle detection.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'obstacle_interest_horizon',
        default_value='7.0',
        description='Forward path distance where detected obstacles trigger local replanning.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'obstacle_lane_offset',
        default_value='0.22',
        description='Lateral offset used by the local avoidance path.'
    ))
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(localized_odom_node)
    ld.add_action(reset_sim_pose_action)
    ld.add_action(reset_amcl_pose_action)
    ld.add_action(waypoint_planner_node)
    ld.add_action(speed_profile_node)
    ld.add_action(virtual_obstacle_scan_node)
    ld.add_action(scan_obstacle_detector_node)
    ld.add_action(local_avoidance_planner_node)
    ld.add_action(mpcc_node)
    ld.add_action(enable_mpcc_action)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
