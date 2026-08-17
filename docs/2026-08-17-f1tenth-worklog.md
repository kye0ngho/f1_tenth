# 2026-08-17 F1TENTH Worklog

## Goal

IFAC Busan 2026 RoboRacer/F1TENTH competition prep. Build our own Humble full stack from perception to control while benchmarking the public UNICORN racing stack concepts without copying their code.

## Benchmark Direction

UNICORN stack structure observed locally at `unicorn-racing-stack/`:

- `planner/spliner`: static obstacle spline avoidance.
- `planner/spliner_planner`: dynamic avoidance with Frenet `s,d`, horizon, boundary checks, and prediction inputs.
- `planner/lane_change_planner`: lane/evasion path generation, side hysteresis, map-boundary validation.
- `race_utils/f110_utils/libs/vel_planner`: curvature and acceleration limited velocity planning.
- `state_machine/config/planners/*`: planner mode separation for start/static avoidance/dynamic avoidance/recovery.

Our adaptation target:

- Keep Humble compatibility.
- Use LiDAR `/scan`; do not depend on depth.
- Use Nav2 AMCL for localization.
- Use local replanning concepts from UNICORN: obstacle in Frenet/path coordinates, side selection by free space, boundary clearance validation, hysteresis, and speed reduction during avoidance.
- Avoid direct code copy.

## Implemented

- Added AMCL-based localized odometry path:
  - `map_server` and `amcl` launch support.
  - `localized_odom_node` publishes `/car_state/odom` from AMCL pose plus simulator odom velocity.
  - Launch order fixed so `map_server` and `amcl` reach `active`.
- Added autonomy launch controls in `launch/gym_bridge_launch.py`:
  - `autonomy`, `auto_enable`, `rviz`, `localization`, `planner`, `controller`, `reset_pose`.
  - waypoint planner, speed profile, virtual obstacle scan injection, scan obstacle detector, local replanner, MPCC.
- Added velocity planning:
  - `/planning/speed_profile` publishes `[s, v]` pairs.
  - Curvature speed cap and forward/backward acceleration/deceleration passes.
  - MPCC consumes this profile.
- Added obstacle/replanning path:
  - `virtual_obstacle_scan_node` injects map-frame circular obstacles into `/scan_with_obstacle`.
  - `scan_obstacle_detector_node` detects on-track LiDAR clusters and publishes `/planning/detected_obstacles`.
  - `local_avoidance_planner_node` publishes `/planning/path` and `/planning/replan_state`.
  - Replan states: `GLOBAL`, `LOCAL_AVOIDANCE_LEFT`, `LOCAL_AVOIDANCE_RIGHT`, `BLOCKED`.
- Improved MPCC:
  - dry-run visualization while disabled.
  - native OSQP option and solve diagnostics.
  - speed profile support.
  - steering-rate limiting.
  - avoidance speed cap and blocked speed cap.
  - actual `/drive` speed now respects active avoidance/BLOCKED cap.
  - safety stop on broad control-loop exceptions.
- Added configs:
  - `mpcc_params_fast_sim.yaml`
  - `mpcc_params_obstacle_sim.yaml`
  - `mpcc_params_replan_sim.yaml`
  - `mpcc_params_replan_3mps_sim.yaml`
  - `mpcc_params_replan_1p5_sim.yaml`
- RViz config updated for AMCL, path, speed profile, obstacle, local replanning, MPCC predicted/reference markers.

## Current Runtime Status

Last stable low-speed launch used:

```bash
docker exec -d f1tenth-sim-1 bash -lc 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/humble/setup.bash && source /sim_ws/install/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py autonomy:=true auto_enable:=false reset_pose:=true rviz:=true virtual_obstacles:=true virtual_obstacle_list:="6.74,0.41,0.18" obstacle_detection:=true obstacle_scan_topic:=/scan_with_obstacle obstacle_detection_range:=8.0 obstacle_interest_horizon:=8.0 obstacle_lane_offset:=0.22 local_replanner:=true mpcc_params_file:=/sim_ws/install/f1tenth_kkh/share/f1tenth_kkh/config/mpcc_params_replan_1p5_sim.yaml speed_profile_max_speed:=1.5 speed_profile_min_speed:=0.45 speed_profile_lateral_accel:=2.2 speed_profile_accel:=2.0 speed_profile_decel:=2.5 >/tmp/mpcc_replan_rviz.log 2>&1'
```

Verified before commit:

- Node graph was clean: one `bridge`, `amcl`, `map_server`, `localized_odom_node`, `waypoint_planner_node`, `speed_profile_node`, `scan_obstacle_detector_node`, `local_avoidance_planner_node`, `mpcc_node`, `rviz`.
- `/map_server`: `active [3]`.
- `/amcl`: `active [3]`.
- MPCC low-speed params:
  - `target_speed`: `1.2`
  - `max_speed`: `1.5`
  - `avoidance_speed_cap`: `1.2`
  - `blocked_speed_cap`: `0.0`
- Enabling MPCC succeeded.
- With the test virtual obstacle near `(6.74, 0.41, r=0.18)`, local replanner entered `BLOCKED`.
- Collision stayed false.
- MPCC published `speed: 0.0` when `BLOCKED`, as intended.
- MPCC was stopped before committing.

## Known Issue To Continue

The current local replanner is still too simple compared with UNICORN:

- It mostly shifts the global raceline laterally around the obstacle.
- Candidate set is too small.
- At the current obstacle location it evaluates:
  - left score about `-0.01 m`
  - right score about `0.10 m`
  - required wall clearance `0.18 m`
- Result: `BLOCKED`, not a usable avoidance path.

This is safer than driving into the wall, but it is not competition-grade replanning yet.

Next implementation step:

- Replace single lateral target per side with multiple Frenet-style `d` candidates.
- Score candidates by:
  - minimum map/wall clearance,
  - obstacle clearance,
  - curvature/smoothness,
  - distance from current vehicle lateral position,
  - side hysteresis.
- Only publish a local avoidance path if it is wider than the current/global option and passes clearance.
- If no candidate passes, keep `BLOCKED` and speed cap at zero.

## 2026-08-18 Replanner Fix

The local replanner no longer evaluates only one lateral offset per side.
It now samples multiple Frenet-style lateral `d` candidates per side, scores
feasible candidates by wall clearance, obstacle clearance, smoothness,
current ego lateral offset, and side hysteresis, and keeps `BLOCKED` only
when no candidate satisfies both wall and obstacle clearance.

Validation after the change:

- `colcon build --packages-select planning` passed in `f1tenth-sim-1`.
- Offline reproduction for virtual obstacle `(6.74, 0.41, r=0.18)` selected
  left `d=+0.463 m`, wall clearance `0.300 m`, obstacle distance `0.465 m`,
  obstacle margin `+0.050 m`.
- Short headless launch with the same obstacle repeatedly logged
  `avoidance selected left target_d=0.42 m`; no lingering ROS processes after
  the 35s timeout.

## Commands For Next Session

Enter container:

```bash
docker exec -it f1tenth-sim-1 bash
```

Build:

```bash
source /opt/ros/humble/setup.bash
cd /sim_ws
colcon build --packages-select f1tenth_gym_ros f1tenth_kkh planning localization
source install/setup.bash
```

Check AMCL:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map ego_racecar/base_link
```

Enable/disable MPCC:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

Minimal status check:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic echo --once /drive
ros2 topic echo --once /ego_racecar/collision
ros2 topic echo --once /planning/replan_state
ros2 topic echo --once /planning/detected_obstacles
ros2 topic echo --once /f1tenth_kkh/mpcc/solve_time_ms
```

Avoid running many `ros2 topic echo` commands in parallel. It caused DDS/SHM noise and made topics look unstable.

