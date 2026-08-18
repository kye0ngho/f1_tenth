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

## 2026-08-18 Dawn Resume Note

If the user says "8/18 dawn work" or asks to continue the MPCC/UNICORN
replanner work, resume from this state:

- Latest functional commit before this note: `81c5872 Improve local replanner
  candidate selection`.
- The issue diagnosed on 2026-08-17 was not MPCC failure. The local
  replanner was publishing `BLOCKED`, so MPCC correctly applied
  `blocked_speed_cap: 0.0`.
- On 2026-08-18, `local_avoidance_planner_node.py` was changed from one
  lateral shift per side to multi-candidate Frenet-style lateral `d`
  sampling and scoring.
- Verified: `colcon build --packages-select planning` passed in
  `f1tenth-sim-1`. Offline and short headless launch both selected a left
  avoidance path for `(6.74, 0.41, r=0.18)` instead of `BLOCKED`.
- Next action: run full MPCC enable test, preferably headless first and RViz
  second. Watch `/planning/replan_state`, `/drive`,
  `/ego_racecar/collision`, `/f1tenth_kkh/mpcc/solve_time_ms`, and local
  replanner logs.
- Expected first result: `/planning/replan_state` should become
  `LOCAL_AVOIDANCE_LEFT` near the virtual obstacle, MPCC should publish a
  nonzero speed capped by `avoidance_speed_cap`, and collision should remain
  false.
- If it still fails, debug MPCC tracking of the modified `/planning/path`
  next, not obstacle detection first, because the detector/replanner already
  selected a feasible path in the short launch.

## 2026-08-18 RViz End-to-End Verification (resolves Dawn Resume Note)

Ran the exact next action from the Dawn Resume Note: full MPCC-enabled
launch, headless first, then RViz, with the same virtual obstacle
`(6.74, 0.41, r=0.18)`.

**Headless run found a real bug, not the anticipated "MPCC tracking of
modified path" issue.** `/planning/replan_state` was chattering: it flipped
between `GLOBAL`, `LOCAL_AVOIDANCE_LEFT`, `LOCAL_AVOIDANCE_RIGHT`, and
`BLOCKED` every 100-300ms while the car passed the obstacle (confirmed with
a synchronized rclpy watcher subscribing to `/planning/replan_state`,
`/planning/detected_obstacles`, `/drive`, and
`/f1tenth_kkh/mpcc/solve_time_ms` together — a single `ros2 topic echo`
per topic is not enough to see this, the transitions are faster than a human
can correlate by eye across separate terminals). Root cause: the multi-
candidate scorer added on 2026-08-18 (commit `81c5872`) recomputes the
winning side/state from scratch every 8Hz tick with only a weak
`side_switch_margin_m: 0.08` nudge toward the previous choice — small,
legitimate per-tick jitter in wall/obstacle clearance (grid-quantized) and
in the vehicle's own measured lateral offset (itself perturbed by the
resulting steering response — a real feedback loop) was enough to flip the
winning candidate almost every publish. Every flip re-routed
`local_avoidance_planner_node` -> `/planning/path` -> `mpcc_node`, so MPCC
was being handed a different reference geometry several times a second
during every avoidance encounter.

Fix (`algorithms/planning/planning/local_avoidance_planner_node.py`):
added a `state_switch_hold_s` (default 0.30s) debounce layer in `publish()`
— `_debounce()`/`_commit()` — that only lets the outward-facing
`/planning/path` and `/planning/replan_state` change once a newly computed
candidate state has won for that many seconds straight; a single noisy tick
no longer flips the published output. Also applied two smaller latent-bug
fixes found via code review before the live test:
`algorithms/f1tenth_kkh/f1tenth_kkh/mpcc_node.py`'s `path_callback` change-
detection tolerance widened `1e-6` -> `1e-3` (was forcing a full raceline
rebuild on essentially every republish even for sub-millimeter noise), and
`algorithms/f1tenth_kkh/f1tenth_kkh/common/raceline.py`'s
`ClosedRaceline.update()` now keeps the nearest-point search-hint
(`_nearest_index`) across same-length updates instead of unconditionally
discarding it (avoids an O(N) full-path scan fallback on every rebuild).

Re-verified headless after the fix, 3 separate obstacle encounters across
repeated laps: every encounter now shows exactly one clean
`GLOBAL -> LOCAL_AVOIDANCE_LEFT -> GLOBAL` transition (durations 1.4-5.4s
depending on approach speed), zero chatter, `/ego_racecar/collision` stayed
`false` throughout, `/f1tenth_kkh/mpcc/solve_time_ms` stayed at 0.3-0.9ms
(control budget at 10Hz is 100ms — large margin), no `MPCC rejected path`
or solver errors, node graph stayed clean (exactly one `/drive` publisher).

Then re-ran with `rviz:=true` (needed `xhost +local:` on the host — X11
access control was blocking the container's `rviz2` with "Authorization
required, but no authorization protocol specified" / `qt.qpa.xcb: could not
connect to display`, unrelated to this bug, just an X11-session detail worth
remembering for future sessions on this machine). User visually confirmed
in RViz: `PlanningPath` bends around the virtual obstacle marker and
`MPCCPredicted` tracks the bent path correctly, no visible jitter.

**Status: the local replanner -> MPCC RViz pipeline works end-to-end for a
single static obstacle now.** Two operational notes for next session:

- One transient startup issue seen mid-session: after force-killing a
  previous launch's processes with `pkill -9` (rather than `docker restart`
  or letting `ros2 launch` shut its own process tree down cleanly),
  `map_server` got stuck at `Configuring` indefinitely on the next launch
  and `ros2 node list` showed a phantom duplicate `/map_server` entry. A
  full `docker restart f1tenth-sim-1` cleared it every time; `pkill -9` on
  the launch tree did not. Prefer letting `ros2 launch` process groups exit
  on their own (Ctrl-C / `kill` the `ros2 launch` PID, not its children) or
  `docker restart` between headless test iterations.
- `state_switch_hold_s` (0.30s) is untuned — it was picked to be clearly
  longer than the observed 100-300ms chatter period and clearly shorter
  than a real avoidance encounter (1.4-5.4s observed). Not stress-tested
  yet with a *moving* obstacle or two obstacles at once, where a longer
  hold could delay a legitimate side switch.

## 2026-08-18 Straight-Line Speed Increase

User asked to raise speed, but only on straights -- `speed_profile_node`
already separates the two: cornering speed is capped independently via
`sqrt(lateral_accel_limit / curvature)` (`speed_profile_node.py:178-183`),
straight sections (curvature ~0) get pushed up to `max_speed`. So raising
`max_speed` alone (both `mpcc_node`'s own and the speed-profile ceiling)
raises straight-line speed without touching cornering behavior, as long as
`lateral_accel_limit`/`max_lateral_accel` stay unchanged.

Added `algorithms/f1tenth_kkh/config/mpcc_params_replan_straight_sim.yaml`
-- a copy of `mpcc_params_replan_1p5_sim.yaml` with `target_speed` 1.20 ->
1.60, `max_speed`/`v_theta_max` 1.50/1.50 -> 2.00/2.00, everything else
(`max_lateral_accel: 2.20`, `corner_slowdown_gain: 0.55`, weights,
`avoidance_speed_cap: 1.20`) unchanged. Launched with
`speed_profile_max_speed:=2.0` (was 1.5), `speed_profile_lateral_accel:=2.2`
unchanged.

Verified (headless-equivalent, watched via the same synchronized-topic
watcher, ~60s / 4 obstacle passes): straight-line speed now reaches
~1.85-1.99 m/s (was capped ~1.3-1.4 m/s), corner-floor speed unchanged at
~1.2-1.38 m/s (confirms cornering is genuinely curvature-governed, not
just riding the old ceiling), `/ego_racecar/collision` stayed `false`
throughout, no MPCC solver errors, `solve_time_ms` still 0.4-0.9ms.

One rough patch noted: right after switching to this config (car still
settling from the config swap / MPCC re-enable), the first obstacle pass
showed 3 replan-state commits within ~17s instead of one clean
`GLOBAL -> LOCAL_AVOIDANCE_LEFT -> GLOBAL` (each commit itself was still
held for the full `state_switch_hold_s`, so no rapid flicker returned --
just more back-and-forth than ideal at this specific pass). The next 2
passes were clean single transitions again. Not yet clear if this was a
one-off startup transient or something that recurs at the higher approach
speed -- worth a longer multi-lap soak before trusting this config for a
timed run, consistent with this project's standing rule to verify
reproducibility over repeated trials, not a single pass.

## 2026-08-18 Straight-Line Speed Raised Further (2.8-3.0 m/s) + Obstacle Braking Onset Fixed

User asked to push straight-line speed to ~2.8-3.0 m/s. Added
`algorithms/f1tenth_kkh/config/mpcc_params_replan_3mps_gentle_sim.yaml` --
same as `mpcc_params_replan_straight_sim.yaml` but `target_speed` 1.60 ->
2.80, `max_speed`/`v_theta_max` 2.00 -> 3.00; `max_lateral_accel: 2.20`
(corner governor) intentionally unchanged. Launched with
`speed_profile_max_speed:=3.0`, `speed_profile_decel:=1.1` (see below).
Verified: profile peak ~2.99 m/s on straights, corner floor still ~1.38 m/s
(unchanged -- confirms cornering really is curvature-governed, not just
riding the old ceiling).

User also asked for the deceleration into corners to start gradually well
before the corner, not right at the corner entry. `speed_profile_node`
already supports this for free -- `build_velocity_profile()`'s backward
pass (`speed_profile_node.py:195-199`) walks backward from each low-speed
point capping every preceding point so the drop is achievable within
`decel_limit`; a smaller `decel_limit` forces the ramp to start farther
back. Lowered the launch arg `speed_profile_decel` from 2.5 -> 1.1 (accel
side, `speed_profile_accel`, left at 2.0 -- only asked to change
deceleration). Confirmed via the full `[s, v]` profile dump: braking into
the tightest corner now ramps down over roughly 5-7m instead of ~1.5m.

**Follow-up bug found from live testing**: user reported the car braking
"way too early" specifically around the *obstacle*, distinct from the
corner-ramp change above. Root cause: `mpcc_node._active_speed_cap()`
(`mpcc_node.py:447-459`) is a hard clamp -- the instant
`/planning/replan_state` leaves `GLOBAL`, every horizon sample's reference
speed is clamped to `avoidance_speed_cap` immediately, no ramp. And that
state flip happens as soon as the obstacle is within
`obstacle_interest_horizon` (launch arg, was `8.0`) of the vehicle along
the path -- at the new ~3 m/s cruise speed, 8m of lead distance reads as
very premature braking. Fixed by lowering `obstacle_interest_horizon` to
`4.0` (a `local_avoidance_planner_node` launch param, not a code change --
the geometric path-shift itself only needs `ramp_in_m: 1.20` of lead, so
shrinking the horizon only affects when the state flip -- and thus the
speed cap -- engages). Verified via a synchronized odom-distance-vs-speed
watcher across ~5 obstacle passes at 1.7-2.8 m/s approach speed: state now
commits to `LOCAL_AVOIDANCE_LEFT` (and the speed clamp engages) at
roughly 2.8-4.0m from the obstacle, consistently, instead of ~8m before.
`/ego_racecar/collision` stayed `false` throughout all passes.

Note this only moved *when* the hard clamp engages, not the hard-clamp
mechanism itself -- the drop from cruise speed to `avoidance_speed_cap`
(1.20) still happens as a step once state commits, over roughly 1m/0.3s,
not a gradual ramp like the corner-speed fix above. If that still feels
abrupt at this speed, the next step would be making `_active_speed_cap()`
itself distance-proportional (mirroring the corner-speed approach) instead
of just moving the trigger point closer -- not done yet, flagging for next
session if it comes up again.

Launch command used for this config (headless-equivalent + `rviz:=true`):
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py autonomy:=true auto_enable:=true \
  reset_pose:=true rviz:=true virtual_obstacles:=true \
  virtual_obstacle_list:="6.74,0.41,0.18" obstacle_detection:=true \
  obstacle_scan_topic:=/scan_with_obstacle obstacle_detection_range:=8.0 \
  obstacle_interest_horizon:=4.0 obstacle_lane_offset:=0.22 local_replanner:=true \
  mpcc_params_file:=/sim_ws/install/f1tenth_kkh/share/f1tenth_kkh/config/mpcc_params_replan_3mps_gentle_sim.yaml \
  speed_profile_max_speed:=3.0 speed_profile_min_speed:=0.45 \
  speed_profile_lateral_accel:=2.2 speed_profile_accel:=2.0 speed_profile_decel:=1.1
```

## 2026-08-18 UNICORN Comparison + Debounce Bug Found

User asked for a comparison pass against the locally-cloned UNICORN stack
(`unicorn-racing-stack/`, gitignored, reference only) to look for logic
flaws or things worth adding. Findings:

**Bug fixed**: `local_avoidance_planner_node.py`'s `_debounce()` (added
earlier today) only updated `committed_points`/`committed_side_out` inside
`_commit()`, which only runs on an actual state-label transition. While the
label stays e.g. `LOCAL_AVOIDANCE_LEFT`, `_build_replanned_points` keeps
recomputing a fresh candidate every 8Hz tick (`target_d` legitimately
bounced 0.32-0.52m across a single pass in the logs), but the *published*
path stayed frozen at whatever the very first commit tick computed. Didn't
cause a visible failure in today's static-single-obstacle tests (values
stayed close enough), but defeats the point of continuous re-scoring and
would matter once obstacles move. Fixed: the `state == self.committed_state`
branch now also refreshes `committed_points`/`committed_side_out` every
tick -- only the state *label* switch is debounced, not the geometry within
an already-committed state. Re-verified: state transitions still clean
single commits (no chatter returned), collision stayed false.

**UNICORN comparison** (read `planner/spliner`, `planner/lane_change_planner`,
`race_utils/f110_utils/libs/vel_planner`; did not reach `spliner_planner`
source or `state_machine` source itself, only its `config/planners/*`):

- Our today's `state_switch_hold_s` debounce pattern matches UNICORN's
  `lane_change_planner/change_avoidance_node.py::_apply_side_hysteresis`
  almost exactly (committed/pending/counter, reset-on-agreement) -- just
  frame-count-based there vs. wall-clock here. Validates the approach was
  right, not a hack.
- UNICORN additionally hard-blocks a side switch whenever the car is
  meaningfully off-centerline mid-maneuver (`abs(cur_d) > 0.25`) --
  a spatial complement to our time-based hysteresis. Not adopted yet.
- UNICORN fits an actual spline (`InterpolatedUnivariateSpline` +
  Savitzky-Golay smoothing) through pre-apex/apex/post-apex Frenet points
  for the evasion path, vs. our discrete cosine-window shift of existing
  raceline points. Given our raceline is coarse (~116 points / 23m ≈ 0.2m
  spacing), worth adapting if avoidance-path smoothness becomes a problem.
  Not adopted yet.
- UNICORN picks the avoidance side using track curvature ahead (avoid
  toward the *outside* of an upcoming corner), not just the obstacle's own
  lateral offset like `_preferred_side()` does. Not adopted yet -- an
  obstacle near a corner apex could currently get avoided toward the
  tighter line.
- UNICORN's avoidance speed comes from per-point speeds on the evasion
  spline itself (reusing the same curvature-based profile, no separate
  hard cap), which is the "correct" version of the rough edge we already
  flagged earlier today (`mpcc_node._active_speed_cap()`'s hard step).
  Not adopted yet -- would need `local_avoidance_planner_node` to publish
  per-point speeds on the shifted path instead of mpcc clamping a scalar.
- UNICORN has real obstacle-motion prediction (propagates `s,d` forward by
  measured velocity). Confirms our "static obstacles only, no prediction"
  gap is real, but it's a legitimately separate, larger subsystem (needs a
  tracker, not just per-frame clustering) -- already correctly flagged as
  out of scope in earlier memory.
- UNICORN's `vel_planner` couples lateral/longitudinal tire budget via a
  friction-ellipse-style model; ours (`speed_profile_node`) treats
  `corner_slowdown_gain`/`lateral_accel_limit`/`accel_limit`/`decel_limit`
  as independent caps. More physically correct in UNICORN, but a big
  vehicle-dynamics-modeling investment and unlikely to be the binding
  constraint yet at 1-3 m/s test speeds -- flagged as "someday."

None of the above were implemented -- reported to the user as a prioritized
list for them to choose from, per project convention (benchmark UNICORN's
concepts, never copy its code directly).

## 2026-08-18 Items 1-3 Implemented; Item 2 (Dynamic Speed Cap) Reverted

Implemented the three UNICORN-inspired items from the earlier comparison,
in the order the user asked for:

1. **Corner-aware side preference** -- `path_utils.py`'s `ClosedPath` now
   also computes `curvature` (same formula as `ClosedRaceline`) and exposes
   `mean_curvature_ahead(s, lookahead_m)`. `local_avoidance_planner_node`'s
   `_preferred_side()` now uses this (new `_corner_outside_side()`) as a
   tie-break when the obstacle itself is near-centered: prefer the outside
   of any corner starting within `corner_lookahead_m` (2.0m default) if
   `|mean_curvature| > corner_curvature_threshold` (0.15 rad/m default).
   Obstacle-position-based side selection (the safety-primary signal)
   is unchanged when the obstacle is clearly off-center.
2. **Avoidance-path speed profile (attempted, reverted)** -- see below.
3. **Spatial side-switch lock** -- `_build_replanned_points()` now also
   locks to `self.committed_side` (not just re-scoring both sides) whenever
   `abs(ego_lateral) > side_lock_ego_offset_m` (0.25m default) and the
   freshly preferred side differs from committed -- mid-maneuver and
   meaningfully off centerline, don't even consider flipping sides.
   Complements the time-based `state_switch_hold_s` debounce with a
   spatial one, matching UNICORN's `_check_ot_side_possible`.

**Item 2 was implemented, tested live, found broken, and reverted --
important lesson for next time this comes up.** Built a
`/planning/avoidance_speed_cap` topic: `local_avoidance_planner_node`
computed a smoothly-varying cap as a function of arc-length distance to
the obstacle (first a linear ramp, then a physically-motivated
`sqrt(avoidance_speed_cap^2 + 2*decel*distance)` braking curve after the
linear version turned out to interpolate up to an unusably large sentinel),
and `mpcc_node._active_speed_cap()` consumed it, prioritized over the old
flat `avoidance_speed_cap` clamp.

**Live testing at both 1.5 m/s and 3.0 m/s tiers showed the car's actual
speed barely responded to the ramping cap at all** (e.g. cap dropping from
99 to 1.3 over ~1s while measured `/drive` speed stayed flat around
2.8 m/s) **-- and because the dynamic cap took priority over the old flat
clamp via an `elif` chain, this silently disabled the one mechanism that
actually worked.** At 3.0 m/s this let the car swerve through the full
~0.5m avoidance offset at nearly full cruise speed, which on one occasion
left it stopped in `BLOCKED` with only 0.15m wall clearance (never an
actual collision -- `/ego_racecar/collision` stayed `false` throughout all
testing -- but a real stability regression the user caught live: "아직도
장애물 피하다가 휘청거리거나 벽 박는다").

Root cause understood after reverting: this MPCC formulation is an
economic-MPC that balances `q_speed` against `q_progress` and
`rd_acceleration`/`rd_v_theta` (rate-of-change penalties) -- it only
reliably converges to a speed target that holds still across several
consecutive solves. A target that changes every single 15Hz tick (the
"smooth ramp") never gets tracked; a flat target held for the whole
avoidance-state duration (the old approach) does. This is NOT the same
situation as the corner-speed fix earlier today, which works because
`speed_profile_node` bakes the ramp into a static profile that the
controller sees across its *entire prediction horizon* every solve, not
as a receding real single scalar.

Reverted: `mpcc_node._active_speed_cap()` back to the original two-branch
form (`BLOCKED` hard stop, flat `avoidance_speed_cap` clamp), removed the
`avoidance_speed_cap_callback`/subscription/params from `mpcc_node.py`,
removed `_speed_cap_for_ds`/the `Float32` publisher/params from
`local_avoidance_planner_node.py`, removed the `avoidance_speed_cap`
launch arg and its wiring from `gym_bridge_launch.py`. Re-verified at
3.0 m/s after reverting: 6 consecutive clean obstacle passes, every one
showing genuine deceleration to ~1.20-1.34 m/s during avoidance (not just
a cap value that never bites), clean single state transitions, transient
one-tick "blocked" warnings correctly absorbed by the `state_switch_hold_s`
debounce without stopping the car, `/ego_racecar/collision` false
throughout.

**If "gradual avoidance speed" comes up again**: the fix has to bake the
target into the actual predicted reference across the horizon (extend
`_build_progress_reference`'s per-step sampling to pull from a
distance-based curve, the way `_profile_speed` already samples the
corner/straight profile per horizon step) rather than clamping a single
receding scalar cap re-issued each solve. That's a real change inside
`mpcc_node.py`'s reference-building, not something `local_avoidance_planner_node`
can do alone by publishing a smarter number.

Items 1 and 3 were not implicated in the instability (the BLOCKED-stuck
incident showed *both* sides scoring infeasible, not a bad side lock) and
were kept as-is after the item-2 revert -- no issues seen across the 6
re-verification passes, but only tested at the single fixed-obstacle
scenario used all session; not yet stress-tested with an obstacle actually
near a real corner (the case item 1 targets) or repeated side-flip
scenarios (the case item 3 targets).

## 2026-08-18 Critical Finding: Continuous-Refresh Fix Caused a Real Collision

User asked to keep stress-testing after items 1-3 landed. A long
(multi-minute) unattended run at 3.0 m/s turned up a serious regression
traced back to **today's earlier "fix" for the frozen-path bug** (the one
found via the UNICORN comparison, where `_debounce()` only updated
`committed_points` on a state-label transition, not every tick).

**What happened**: with committed-state geometry refreshing every 8Hz
tick, `mpcc_node` received a new `/planning/path` and rebuilt its raceline
almost every tick *for the entire duration of every avoidance encounter*
(212 "MPCC received closed path" log lines in one run, not just a brief
burst during EMA convergence as originally assumed). The underlying cause:
`_build_replanned_points`'s best-candidate selection is not smooth even
for a completely stationary obstacle -- `target_d` swung as much as
~0.25m tick-to-tick (`0.27 -> 0.52 -> 0.31 -> 0.52 -> 0.38 -> 0.32` inside
one encounter), and on two occasions **the selected side itself flipped**
(`right -> left` within ~1s, twice in the same run) while nominally still
"debounced" -- each flip individually held past the 0.3s hold, so it
wasn't chatter, but the underlying decision itself was unstable at a
~1-9s timescale. Continuously feeding that wobbling geometry into MPCC's
tracked raceline eventually caused a real collision
(`MPCC disabled: simulator collision reported`), after which the vehicle
ended up with its heading ~170° off from the path direction and sat there
correctly refusing to drive (`MPCC safety stop: heading error is 170.2 deg`)
for the rest of the run -- a second, different safety-governor trip than
the wall-clearance one seen earlier today.

**Fix**: reverted to freezing `committed_points`/`committed_side_out` at
the moment a state label commits (not refreshing every tick within an
already-committed state) -- i.e. the "bug" the UNICORN comparison flagged
this morning is knowingly being left in place for now. This is a deliberate
trade-off, not an oversight: continuous re-scoring is correct in principle
for a *moving* obstacle, but this stack has no obstacle-motion tracking at
all yet (`_stabilize_obstacle` only EMA-smooths a static position), so
freezing costs nothing real today and buys real stability. Re-verified
with an extended (~3 min, 17 encounter) stress test at 3.0 m/s: raceline
rebuild count dropped from 212 to 30 over a comparable run, zero side
flips within an encounter, zero `collision reported` or `heading error`
safety-stop lines, `/ego_racecar/collision` false throughout, every
encounter showed real deceleration to ~1.20-1.34 m/s and a single clean
state transition.

**Real follow-up item, not yet done**: `_build_replanned_points`'s
candidate scoring is not smooth/stable across ticks for a static obstacle,
which is the actual root cause both here and of the frozen-geometry
trade-off being necessary at all. Likely culprits: `_point_wall_clearance`
is grid-cell-discretized (jumps in `map_resolution` steps) and
`_candidate_laterals_for_side` sweeps a `lateral_candidate_step_m`-spaced
discrete set each tick, so the arg-max can hop between nearby candidates
under tiny input changes. Before re-attempting continuous re-scoring (item
2's original intent, or real moving-obstacle support), the fix should be a
slew-rate limit / smoothing directly on the *chosen* `target_lateral`
(and possibly `best_side`) across ticks, not just on the raw obstacle
position like `_stabilize_obstacle` already does -- smoothing the input
isn't enough if the argmax over discretized candidates is itself noisy.

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

