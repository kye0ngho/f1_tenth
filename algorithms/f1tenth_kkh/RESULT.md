# f1tenth_kkh 실험 결과

이 파일은 계속 업데이트됩니다. 최신 항목이 위로 오도록 추가하세요.

## 2026-08-14 (3): RViz 고속 Lab8 MPC 튜닝 — 4.0m/s 목표 + safety governor

RViz를 띄운 상태에서 `lab8` MPC를 직접 주행시키며 속도를 단계적으로 올렸다.
목표는 IFAC 2026 Busan RoboRacer 준비 관점에서 "단순히 빠른 숫자"가 아니라,
빠르게 달리되 벽/코너/경로 이탈 상황에서 자동으로 속도를 깎는 구조를 만드는 것.

### 결론

최종 RViz 실험 세팅은 `target_speed: 4.00m/s`, `max_speed: 4.00m/s`.
직선에서는 safety cap이 거의 4.0m/s까지 열리고, 코너/벽 근처에서는 LiDAR 전방
여유, lateral acceleration, 경로 이탈량에 따라 속도가 내려간다.

최종 12초 계측:

| 항목 | 값 |
|---|---:|
| `/drive` publisher | 1개 |
| `/drive.drive.speed` command | min 1.906 / max 3.548 / last 2.329 m/s |
| `/f1tenth_kkh/lab8_mpc/safety_speed_cap` | min 1.906 / max 3.970 / last 3.214 m/s |
| `/ego_racecar/collision` | true 0 / false 2676 |

해석: 4.0m/s 목표는 유지하되, 현재 track/RViz 조건에서는 safety governor가 실제
command를 약 3.5m/s까지 깎는 구간이 많다. 이게 이번 세션에서 확인된 안정권이다.
4.5m/s는 과했고, 4.0m/s도 safety 없이 열면 collision이 발생했다.

### 속도 래더

실험 중 순차적으로 올린 속도와 관찰:

| 목표 속도 | 관찰 |
|---:|---|
| 1.35m/s | 느림. 충돌 없음. |
| 1.60m/s | 느림. 충돌 없음. |
| 1.85m/s | safety governor 적용 후 안정. |
| 2.20m/s | 약 16초 무충돌, 실제 command 약 2.18m/s까지 확인. |
| 3.00m/s | 약 15초 무충돌, 실제 command 약 2.96m/s까지 확인. |
| 3.40m/s | 약 15초 무충돌, command 약 3.35m/s까지 확인. |
| 4.50m/s | 너무 공격적. 벽/코너 접근에서 불안정하다고 판단해 폐기. |
| 4.00m/s | 최종 채택. safety cap max 3.97m/s, collision 0. |

### 최종 파라미터

현재 채택된 파일: `algorithms/f1tenth_kkh/config/lab8_mpc_params.yaml`

핵심 값:

```yaml
global_frame_id: "odom"
disable_on_collision: false
collision_disable_count: 3

target_speed: 4.00
min_reference_speed: 1.15
corner_slowdown_gain: 0.11
max_lateral_accel: 5.30
max_speed: 4.00
max_acceleration: 7.00
max_steering_angle: 0.4189
max_steering_rate: 3.50
rd_steering: 10.00

max_path_distance: 1.20
safety_enabled: true
safety_forward_half_angle: 0.40
safety_bumper_distance: 0.22
safety_max_deceleration: 4.50
safety_tracking_error_start: 0.10
safety_tracking_error_gain: 4.00
```

주의: `disable_on_collision: false`는 RViz 고속 튜닝용이다. 이번 환경에서
`/ego_racecar/collision`이 순간적으로 stale/튀는 값을 보내 node가 바로 꺼지는
문제가 있어 자동 disable만 분리했다. 실제 safety는 collision topic이 아니라
LiDAR/경로이탈 기반 speed cap이 담당한다. 실차 또는 공식 계측에서는 collision
토픽의 신뢰도를 다시 확인하고 켜는 것이 맞다.

### 구현한 방법론

1. **기본 MPC는 유지하고, reference speed만 곡률 기반으로 변경**
   - 기존 Lab8 스타일 QP 구조는 유지.
   - `reference_speed(curvature)`를 추가해 곡률이 큰 구간은 reference 자체가
     느리게 샘플링되도록 했다.
   - `max_lateral_accel`로 `v <= sqrt(a_lat / |kappa|)` 형태의 물리적 상한을 둠.

2. **조향 rate 안정화**
   - `max_steering_rate`로 command-level steering step을 제한.
   - `rd_steering`과 `previous_steering_parameter`를 추가해 QP objective에서
     이전 조향 대비 급격한 변화도 벌점 처리.
   - 목적: 고속에서 "쓸데없는 회전" 또는 벽 근처 조향 흔들림을 줄이는 것.

3. **MPC 뒤에 rule-based safety governor 추가**
   - 현재 구조는 "MPC + rule-based safety governor"다.
   - 장애물/벽/경로이탈이 QP constraint로 직접 들어가는 완전한 obstacle-aware
     MPC는 아니고, MPC가 낸 speed command를 후처리로 cap한다.
   - cap 구성:
     - 조향각 기반 lateral acceleration cap
     - path tracking error 기반 cap
     - `/scan` 전방 range 기반 braking-distance cap

4. **RViz/시뮬레이터 pose 문제 수정**
   - AMCL `map -> odom`이 reset 후 stale해져 vehicle이 path에서 3-4m 떨어진 것으로
     보이는 문제가 있었다.
   - `global_frame_id: "odom"`을 쓰고, `lab8_mpc_node.py`에서 odom frame일 때는
     TF lookup 대신 `/ego_racecar/odom` pose를 직접 사용하도록 수정했다.
   - 이 수정 후 reset 직후 경로 첫 점 `(0.2985288, 0.5926084)`와 odom이 일치함을
     확인했다.

5. **중복 publisher 제거**
   - `/drive` publisher가 2개이면 한 노드가 stop/저속을 섞어 보내 실제 속도가
     1m/s 근처로 떨어진다.
   - 매 실험 전 `/drive` publisher count를 확인했고, 최종 상태는 publisher 1개.

### 재현 절차

시뮬레이터/RViz/planning이 떠 있는 상태에서:

```bash
# 기존 lab8/mpc 계열 노드 정리
pgrep -f "[l]ab8_mpc_node" | xargs -r kill -9
pgrep -f "[m]pc_experiment.launch.py controller:=lab8" | xargs -r kill -9

# lab8 MPC detached 실행
ros2 launch f1tenth_kkh mpc_experiment.launch.py controller:=lab8

# 제어 stop 후 simulator pose reset
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
ros2 topic pub --times 5 --rate 5 /sim_reset_pose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.2985288, y: 0.5926084, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.3053138966348658, w: 0.9522517653024511}}, covariance: [0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0.05]}}"

# 시작점 확인
ros2 topic echo --once /ego_racecar/odom --field pose.pose.position

# 활성화
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

계측:

```bash
ros2 topic info /drive -v

timeout 12 ros2 topic echo /drive --field drive.speed
timeout 12 ros2 topic echo /f1tenth_kkh/lab8_mpc/safety_speed_cap --field data
timeout 12 ros2 topic echo /ego_racecar/collision --field data
```

### 다음 단계

- 최종 4.0m/s 설정을 1랩 단위 CSV로 남겨 lap time/CTE까지 기록해야 한다. 이번
  항목은 RViz 실시간 관찰 + 12초 topic 계측이며, 완전한 1랩 benchmark는 아니다.
- sustained 4.0m/s 이상을 원하면 단순히 `target_speed`만 올리지 말고 track별
  raceline speed map을 만들어야 한다. 지금 safety governor가 많이 깎는 구간은
  곡률/벽 여유가 실제 병목이라는 뜻이다.
- Head-to-Head까지 생각하면 현재 방식은 rule-based safety cap이므로, 최종 대회용은
  CBF-QP 또는 obstacle-aware MPC constraint로 확장하는 것이 맞다.

## 2026-08-14 (2): 실차 ROS 스택 검증 — 크리티컬 크래시 버그 발견/수정 + 고속 안정성 실측

이전 항목(장애물 회피 구현)에서 못 했던 실제 ROS 토픽 기반 end-to-end 검증을
`f1tenth_gym_ros_humble` 컨테이너(gym_bridge + AMCL + planning + mpcc, headless,
`/sim_reset_pose`+`/initialpose`로 RViz 없이 pose 설정)에서 수행했다. 그 과정에서
**노드 전체가 죽는 크리티컬 버그**를 발견해 수정했고, 이어서 "속도를 올렸을 때도
안정적인지"를 실측했다.

### 버그 1: 장거리 grazing-angle 벽 반사가 허위 장애물로 오검출되어 OSQP가 크래시

**증상**: 트랙에 장애물이 전혀 없는 상태에서도, 차량이 특정 위치에 있을 때
`mpcc_node` 프로세스 전체가 죽었다(`ValueError: Upper bound update error!` /
`osqp_update_upper_bound: lower bound must be lower than or equal to upper bound`,
`control_loop`의 `except (TransformException, RuntimeError,
cp.error.SolverError)`에 잡히지 않는 raw `ValueError`라 안전 정지 대신 노드
자체가 죽었음).

**원인**: 시뮬레이터 LiDAR의 `range_max`가 30m인데 트랙 한 바퀴가 23m밖에
안 된다. 장거리에서 벽을 거의 스치는 각도(grazing angle)로 보면 빔 하나하나의
거리값이 급격히 튀기 때문에, `cluster_scan`의 range-jump 세그멘테이션이 사실은
하나의 연속된 벽인데도 여러 개의 좁은(=`obstacle_max_width_m`를 통과하는) 클러스터로
쪼개버렸다. 실측: 차량은 (0.3, 0.6) 근처에 있는데 (6.09, -0.13), (6.53, -0.06),
(6.96, -0.01) 같은, 차량과 무관한 먼 지점들이 `/f1tenth_kkh/mpcc/detected_obstacles`에
장애물로 잡혔다. 이 허위 장애물 중 하나가 특정 조건에서 `update_obstacle_constraints()`가
만드는 halfspace를 실제로 infeasible한 bound(l > u)로 만들어 OSQP를 크래시시켰다
(정확한 기하 조건까지는 특정하지 않음 — 재현에는 이 정도로 충분).

**수정** (`mpcc_node.py`, `config/mpcc_params.yaml`):
1. `obstacle_max_range_m`(기본 4.0m) 파라미터 추가 — `scan_callback`이
   `cluster_scan`에 넘기는 `range_max`를 `min(msg.range_max,
   obstacle_max_range_m)`로 캡. 트랙 규모(23m)보다 훨씬 작게 잡아 grazing-angle
   장거리 벽 파편화를 원천 차단. horizon이 반응 가능한 거리(~1-2m)보다 크게
   잡아도 실용상 손해가 없다(더 멀리 봐도 아직 반응할 수 없으므로).
2. `control_loop`의 `except (TransformException, RuntimeError,
   cp.error.SolverError)`를 `except Exception`으로 넓혔다 — 이번처럼
   cvxpy/OSQP 내부에서 넘어오는, 미리 예상 못 한 예외 타입이 있어도 노드가
   죽는 대신 항상 기존 안전 정지 경로로 빠지도록. **실차 투입 전 반드시
   있어야 하는 방어선**(장애물 인지 로직이 아무리 다듬어져도 예상 밖 케이스는
   또 나올 수 있음).

**검증**: 크래시가 재현되던 정확한 pose(8.357, 3.618, yaw 2.556)로
`/sim_reset_pose`+`/initialpose`를 보낸 뒤 수정된 코드로 재기동 → 크래시 없이
10초+ 생존 확인, `detected_obstacles`가 빈 배열로 정상화됨을 확인. 이후 같은
pose에서 출발해 0.85m/s 1랩(아래 "기존 기록 재확인" 참고)도 문제의 구간을
그대로 통과해 무사고로 완주했다.

### 기존 기록(0.85m/s) 재확인 — 무사고, 통계 거의 동일

버그 수정 후 `mpcc_params.yaml` 그대로(target_speed 0.85) 1랩 재주행:

| 시행 | Lap [s] | Mean CTE [m] | P95 CTE [m] | Max CTE [m] | 충돌 |
|---|---:|---:|---:|---:|---|
| 장애물 회피 구현 직후 (장애물 없음) | 35.89 | 0.068 | 0.109 | 0.134 | 없음 |
| 버그 수정 후 재확인 | 36.88 | 0.067 | 0.110 | 0.145 | 없음 |
| 기존 baseline (2026-08-11) | 34.75 | 0.068 | 0.108 | 0.135 | 없음 |

세 실측 모두 mean CTE가 0.067-0.068m로 사실상 동일 — 장애물 회피 코드가
장애물이 없을 때 기존 동작을 조금도 바꾸지 않는다는 구조적 보장(장애물이
없으면 `obstacle_slowdown_gain` 항이 항상 0)이 실측으로도 확인됐다.

### 속도를 올렸을 때 안정성 — target_speed 1.10m/s는 신뢰 불가

`target_speed: 1.10, max_speed: 1.30, v_theta_max: 1.30`(2026-08-11 iter3와
동일 값)로 오버라이드한 임시 yaml(`mpcc_params_file:=` 런치 인자로 지정,
`ros2 param set`으로 살아있는 노드에 바꾸면 `max_speed`/`target_speed`가
QP 안에 **이미 상수로 박혀 있어서**(cvxpy `Parameter`가 아님) 실제로 반영되지
않고 오히려 내부 불일치로 이어진다는 것도 확인함 — 반드시 재기동 필요)로
같은 시작 pose에서 반복 시행:

| 시행 | 결과 | Lap [s] | Mean CTE [m] | Max CTE [m] |
|---|---|---:|---:|---:|
| 1차 (버그 수정 전 코드) | 충돌 (~t=6-8s, 초반 급커브) | – | – | – |
| 2차 (버그 수정 전 코드) | 완주 | 29.69 | 0.073 | 0.150 |
| 3차 (버그 수정 전 코드) | 정체/안전정지 (0.49랩에서 progress 멈춤, 이후 위 크래시 버그의 재현 조건이 됨) | – | 0.122(정체 전까지) | 0.149 |
| 4차 (버그 수정 후 코드) | 충돌 (t≈0-2s, 시작 직후 첫 급커브) | – | – | – |
| 5차 (버그 수정 후 코드) | 완주 | 29.15 | 0.078 | 0.174 |

**결론: 이 트랙/이 컨트롤러에서 1.10m/s는 재현성이 없다.** 버그 수정 여부와
무관하게 완주와 충돌이 뒤섞여 나왔다(5회 중 2회 충돌, 2회 완주, 1회 정체).
완주했을 때조차 mean/max CTE가 0.85m/s 기록(0.067-0.068 / 0.134-0.145)보다
뚜렷이 나쁘다(0.073-0.078 / 0.150-0.174). 충돌은 매번 시작 직후 첫 급커브
근방에서 발생 — track02 raceline의 시작 구간 곡률이 이미 높다
(`track02_raceline_safe.csv` 첫 구간 kappa ≈ 0.55 rad/m). 2026-08-11
튜닝 기록이 iter3(target 1.10)를 "충돌 없음"으로 남긴 것은 **1회 시행 결과였고
재현성 확인이 안 되어 있었음**(그 항목의 "다음 단계"에도 이미 명시됨) — 이번
반복 시행이 그 우려를 확인해준 셈이다.

**권장**: 현재 채택된 `target_speed: 0.85`가 이 트랙에서 실측 재현성 있는
유일한 값이다. 더 높은 속도로 가려면 단순히 숫자를 올리는 게 아니라 (a) 시작
구간처럼 곡률이 높은 구간에서 `corner_slowdown_gain`을 더 강하게 걸거나, (b)
horizon_steps/dt를 늘려 급커브에 대한 반응 여유를 늘리거나(단, solve_time_ms
여유가 가장 타이트한 컨트롤러이므로 실측 없이 올리지 말 것), (c) 이번처럼
여러 번 반복 시행으로 재현성을 확인하는 절차 자체를 습관화해야 한다.

## 2026-08-14: mpcc 정적 장애물 회피(replanning) 구현

### 결론

`mpcc_node.py`의 예약된 `obstacle_a/b_parameters`/`update_obstacle_constraints()`
훅을 채워 정적 장애물 회피를 구현했다(동적 상대차량은 범위 밖).
LaserScan 클러스터링(`common/obstacle_detection.py`) + raceline
코너리도어 필터로 장애물을 인지하고, raceline의 lateral 축을 법선으로
하는 분리 초평면을 horizon 중 장애물 근방 구간에만 적용한다(자세한
알고리즘은 `README.md`/`mpcc_node.py`의 `update_obstacle_constraints()`
docstring 참고).

### 발견: 튜닝된 대회 속도에서는 접근 중 감속 없이는 사실상 회피가 불가능

구현 직후 직접 QP를 호출하는 검증(ROS 토픽 없이 `solve_mpc`/
`update_obstacle_constraints`를 노드에서 직접 호출, cvxpy/rclpy가 있는
`f1tenth_gym_ros_humble` 빌드 컨테이너 안에서 실행)에서, `mpcc_params.yaml`
그대로(target_speed 0.85m/s, horizon_steps 10) "장애물이 horizon 반응
거리(~0.85m) 경계에 막 들어온" 최악 시나리오를 재현하니
`obstacle_keepout_radius_m`을 0.20m보다 조금만 올려도(0.25~0.40m 모두)
QP가 `infeasible`이 됐다. 원인: 이 트랙/속도에서 MPC horizon이 내다보는
거리(target_speed × horizon_steps × dt ≈ 0.85m)가 매우 짧아서, 장애물을
"보자마자" 반응해도 keepout 반경만큼 옆으로 빠질 시간/거리가 부족했다
(k=0은 실제 현재 위치에 hard equality로 고정되므로, 장애물이 relevance
윈도우에 들어온 즉시 전체 horizon에 제약을 걸면 k=0부터 infeasible이
되는 것도 별도로 확인 — 그래서 제약을 장애물의 arc-length 근처 step에만
적용하도록 게이팅했다).

대응: `corner_slowdown_gain`과 동일한 형태로 `obstacle_slowdown_gain`을
추가해, 장애물이 `obstacle_relevance_ahead_m`(3.0m) 안에 들어오면 접근
중 미리 감속하도록 했다(반응 시간을 벌어 회피 여유를 만듦). 그래도
`obstacle_keepout_radius_m` 기본값은 0.40m 대신 **0.20m**으로 낮췄다 —
차량 반폭(~0.148m)보다는 크지만 넉넉한 여유는 아니다. 튜닝된 대회
속도에서 짧은 horizon이 갖는 근본적 반응-거리 한계 때문이며, 이 값을
올리려면 이 절의 방법대로 재검증이 필요하다(단순 직감으로 올리지 말 것).

### 검증: closed-loop 시뮬레이션 (직접 kinematic bicycle 적분, ROS 토픽 없음)

10Hz로 `solve_mpc`를 반복 호출하고 그 결과(가속도/조향)로 kinematic
bicycle을 직접 적분해 전진시키는 경량 closed-loop 테스트(사각형 합성
트랙, 정지 상태에서 출발, `mpcc_params.yaml` 그대로):

| 시작 시점 장애물까지 거리 | 결과 | 최소 clearance |
|---|---|---:|
| 1.5 m | 안전 정지 (18 step째 QP infeasible, 충돌 아님) | 0.38 m (정지 시점 기준) |
| 2.0 m | 회피 성공 | 0.69 m |
| 2.5 m | 회피 성공 | 1.19 m |
| 3.0 m | 회피 성공 | 1.65 m |
| 3.5 m | 회피 성공 | 1.95 m |
| 장애물 없음 (회귀 확인) | 정상 주행 | — |

`obstacle_relevance_ahead_m`(3.0m) 범위 안에서 감지된 장애물은 2.0m
이상 거리에서는 안정적으로 회피했고(clearance가 keepout 요구치보다
훨씬 넉넉한 것은 트랙 코너리도어 폭 안에서 옵티마이저가 최소 여유보다
더 크게 피하는 경향 때문으로 보임), 1.5m처럼 정지 상태에서 매우 가깝게
"막 발견된" 경우는 QP가 infeasible이 되어 **충돌이 아니라 안전
정지**로 이어졌다 — 기존 solver-실패 → 3회 연속 실패 시 auto-disable
경로(`control_loop`)를 그대로 재사용, 별도 안전 로직 추가 안 함. 실제
주행에서는 LiDAR가 훨씬 먼 거리부터 장애물을 감지하므로 정지-상태-1.5m
같은 상황은 급커브 직후처럼 시야가 갑자기 열리는 경우가 아니면 드물
것으로 예상되지만, 검증되지 않은 가정이다.

핵심 기하 로직(halfspace 부호, arc-length 게이팅, 코너리도어 필터,
`_nearest_index` 캐시 오염 방지)은 numpy만으로 구성 가능한 단위
테스트로도 별도 검증했다(장애물/벽 클러스터 분리, 노이즈 클러스터 제거,
`nearest_arc_length_stateless`가 `nearest_state`의 캐시를 건드리지 않음 등).

### 실행하지 못한 검증

시간/환경 제약으로 다음은 이번에 실행하지 못했다(다음 세션에서 진행
권장). 아래 1, 3번은 바로 다음 항목(2026-08-14 (2))에서 완료했다 —
갱신된 상태만 남겨둠:

1. ~~ROS 토픽 기반 end-to-end 검증~~ → **완료.** DISPLAY 없이
   `/sim_reset_pose`+`/initialpose`로 RViz 없이 pose를 설정해
   `gym_bridge_launch.py` + AMCL + planning + `mpcc` 전체 스택을
   headless로 검증했고, 그 과정에서 노드가 죽는 크리티컬 버그를 발견해
   수정했다(2026-08-14 (2) 참고).
2. `num_agent: 2`로 상대차량을 정지 장애물 대역으로 세워 실제 LaserScan
   클러스터링(`scan_callback`)까지 포함한 실주행 1랩 — 여전히 미실행.
   `config/sim.yaml` 변경이 필요해 대회용 설정을 건드리지 않으려면 별도
   세션에서 진행.
3. ~~장애물 없는 기존 트랙 1랩 재주행 재현성~~ → **완료.** 2026-08-14 (2)
   에서 두 차례 재확인, mean CTE 0.067-0.068m로 기존 baseline(0.068m)과
   사실상 동일함을 실측으로 확인.

## 2026-08-11 (3): mpcc 튜닝 — baseline 기록 갱신 확인

### 결론

**갱신됨.** `mpcc`를 속도 축으로만 튜닝(cost 가중치는 그대로 둠)해서
`algorithms/control`의 두 기록(가장 빠른 랩: Linear MPC tuned v1 42.78s, 가장
정확한 CTE: Linear MPC tuned v2 mean 0.070m/p95 0.111m)을 **동시에** 갱신했다.

### 튜닝 절차

`MPC_GUIDE.md`의 튜닝 순서(속도 먼저 고정 → cost 가중치 → 검증)를 따라 진행.
MPCC는 progress-reward(`q_progress`)가 이미 "더 빨리 가라"는 유인을 주므로,
`target_speed`/`max_speed`/`v_theta_max`만 단계적으로 올리고 `q_contour`/
`q_lag` 등 cost 가중치는 손대지 않은 채 CTE가 어떻게 반응하는지 관찰. 매
단계 `/sim_reset_pose`+`/initialpose`로 시작 지점 리셋 후 1랩
(`closed_loop_test.py --laps 1.05 --max-error 0.75`) 실주행.

| 단계 | target_speed | max_speed | Lap [s] | Mean CTE [m] | P95 CTE [m] | Max CTE [m] | 충돌 |
|---|---:|---:|---:|---:|---:|---:|---|
| 기본값 (2026-08-11 (2) 결과) | 0.55 | 0.75 | 53.79 | 0.063 | 0.117 | 0.196 | 없음 |
| iter1 | 0.70 | 0.90 | 42.12 | 0.060 | 0.100 | 0.141 | 없음 |
| **iter2 (= 채택된 tuned v1)** | **0.85** | **1.00** | **34.64** | **0.067** | **0.106** | **0.131** | 없음 |
| iter3 | 1.10 | 1.30 | 27.23 | 0.079 | 0.133 | 0.181 | 없음 |
| iter4 | 1.40 | 1.60 | 21.77 | 0.079 | 0.133 | 0.166 | 없음 |
| **최종 재검증** (실제 `config/mpcc_params.yaml`, 오버라이드 없이) | 0.85 | 1.00 | 34.75 | 0.068 | 0.108 | 0.135 | 없음 |

관찰:
- iter1→iter2까지는 속도를 올릴수록 CTE도 같이 좋아짐(=기본값이 그냥
  보수적이었을 뿐). iter2 이후부터 CTE가 서서히 나빠지기 시작(p95
  0.106→0.133m)하지만 iter4(target 1.40m/s)까지도 **충돌은 한 번도 없었음**.
- CTE가 나빠지는 지점(iter2~3 사이) 대비 lap time은 계속 크게 줄어드는
  비대칭적 트레이드오프 — 속도 축만으로도 상당한 여유가 있었다는 뜻.
- iter3/iter4처럼 더 공격적인 설정도 시뮬레이터 상에서는 통과하지만, (a)
  kinematic bicycle 제어 모델은 타이어 슬립을 모르고 실제 물리 한계는
  시뮬레이터 동역학에만 의존하고 있어 실차 재현성이 불확실하고, (b) 실물
  하드웨어에서 이런 속도의 조향/가속이 실제로 가능한지 검증되지 않았으므로,
  **iter2를 최종 채택**하고 그 이상은 "확인은 했지만 채택 보류"로 남김.

### 최종 채택 설정 (`config/mpcc_params.yaml`, 기존 `mpcc_params_baseline.yaml`로 보존)

`target_speed: 0.85`, `min_reference_speed: 0.40`, `max_speed: 1.00`,
`v_theta_max: 1.00` (나머지 파라미터는 최초 보수적 설정과 동일 — cost
가중치는 튜닝하지 않았음). `config/mpcc_params_tuned_v1.yaml`로 스냅샷 보존.

### 최종 비교 (전체 baseline 포함)

| Controller | Lap [s] | Mean CTE [m] | P95 CTE [m] | Max CTE [m] | Collision |
|---|---:|---:|---:|---:|---:|
| Pure Pursuit safe (baseline) | 48.45 | 0.068 | 0.116 | 0.164 | 0 |
| Linear MPC baseline | 44.13 | 0.117 | 0.211 | 0.241 | 0 |
| Linear MPC tuned v1 (기존 최고 랩타임) | 42.78 | 0.102 | 0.240 | 0.311 | 0 |
| Linear MPC tuned v2 (기존 최고 CTE) | 56.56 | 0.070 | 0.111 | 0.140 | 0 |
| `lab8` (0.55 m/s) | 42.53 | 0.113 | 0.232 | 0.262 | 0 |
| `nonlinear` (0.35 m/s) | 63.69 | 0.099 | 0.209 | 0.238 | 0 |
| **`mpcc` tuned v1 (0.85 m/s) — 신기록** | **34.75** | **0.068** | **0.108** | **0.135** | 0 |

`mpcc` tuned v1이 랩타임(34.75s, 2위인 `lab8`보다도 7.8초/18% 빠름)과 CTE
(mean/p95/max 전부) 양쪽에서 **다른 모든 컨트롤러를 동시에 앞선 유일한
결과**. 비교 플롯: `algorithms/f1tenth_kkh/results/track02_kkh_comparison_v2.png`.

### 다음 단계

1. 여러 랩 반복 주행(3랩 이상)으로 34.75s가 재현되는지, 랩마다 편차가 있는지
   확인 — 지금까지는 매번 1회 실주행.
2. iter3/iter4급 속도까지 실제로 채택할지는 실전 하드웨어(Jetson/NUC/RPi +
   실차 타이어 그립) 검증 후 결정.
3. `solve_time_ms`는 속도와 무관(QP 크기 불변)하므로 튜닝 전 측정치(mean
   56.66ms/p95 67.83ms, 2026-08-11 (1) 항목)가 여전히 유효 — 100ms 예산 대비
   여유는 여전히 세 후보 중 가장 타이트함.
4. Head-to-Head 대비 장애물 회피 확장(`update_obstacle_constraints`)은
   여전히 미구현.

## 2026-08-11 (2): 외부 MPC 저장소 조사 + 실주행(1랩) 비교

### 외부 오픈소스 MPC 저장소 조사 결과

"많은 오픈소스 MPC를 돌려보고 제일 좋은 걸 쓴다"는 방향에 따라 이전에 후보로
꼽았던 것 외에 4개 저장소를 추가로 확인했으나, 그대로 가져다 쓸 수 있는 것은
없었다. 사용자와 상의 후 신규 포팅 없이 기존 3종(`nonlinear`/`mpcc`/`lab8`)의
실주행 비교로 진행하기로 결정.

| 저장소 | 판정 | 사유 |
|---|---|---|
| `f1tenth-dev/mpc_lib` | 불가 | LICENSE/README만 존재, 실제 코드 없음 |
| `smitdumore/f110-mpc` | 보류 | 실제 동작하는 C++/ROS 코드(occupancy-grid 샘플링 궤적 + MPC). 언어/빌드체계(ament_cmake, xtensor, osqp-eigen)가 우리 Python 패키지와 안 맞아 포팅 비용 큼. 장애물 회피 아이디어 자체는 `mpcc_node`의 확장 지점과 연결지어 다음에 참고 |
| `mlab-upenn/ISP2021-mpc_stack` (HMPC) | 보류 | Python이지만 Pacejka 타이어 기반 고속(≈8 m/s) 동역학 모델, 코드 품질 낮음. 우리 트랙 목표속도(0.35–0.8 m/s)의 kinematic 저속 영역과 안 맞고 나눗셈 불안정 위험 |
| `XenonSup/mpcc` | 보류 | CasADi/acados 기반, ROS 통합 없음. acados는 컴파일형 솔버라 Docker 이미지에 무거운 의존성 추가 필요 |

### 환경

- 맵: `track02`, raceline: `track02_raceline_safe.csv`
- 절차: 각 컨트롤러를 `mpc_experiment.launch.py controller:=<x>`로 단독 기동
  (다른 컨트롤러 프로세스가 살아있지 않은지 `ros2 node list`로 매번 확인 —
  중간에 이전 `lab8` 프로세스가 완전히 죽지 않은 채 `nonlinear`와 동시에
  `/control/enable`을 두고 충돌해 첫 시도가 오염된 사고가 있었음, 재확인 후 재시도) →
  `/sim_reset_pose` + `/initialpose`로 시작 지점 리셋 → 기존
  `algorithms/control/scripts/closed_loop_test.py --duration 90 --laps 1.05
  --max-error 0.75`로 1랩 실주행.
- 측정 장비: 개발 데스크톱(16-core), 실전 하드웨어 아님.

### 실주행 결과 (1랩, `/control/enable` 실제 주행)

| 컨트롤러 | target_speed | 결과 | Lap [s] | Mean CTE [m] | P95 CTE [m] | Max CTE [m] |
|---|---:|---|---:|---:|---:|---:|
| `nonlinear` (SLSQP) | 0.55 (기본값) | **충돌** (~0.95랩, 급커브 구간) | – | – | – | – |
| `nonlinear` (SLSQP) | 0.35 (재시도) | 완주 | 63.69 | 0.099 | 0.209 | 0.238 |
| `mpcc` | 0.55 (기본값) | 완주 (1회 시도) | 53.79 | 0.063 | 0.117 | 0.196 |
| `lab8` | 0.55 (기본값) | 완주 (1회 시도) | 42.53 | 0.113 | 0.232 | 0.262 |

### 기존 baseline (algorithms/control/MPC_GUIDE.md, 참고용)

| Controller | Lap [s] | Mean CTE [m] | P95 CTE [m] | Max CTE [m] | Collision |
|---|---:|---:|---:|---:|---:|
| Pure Pursuit safe | 48.45 | 0.068 | 0.116 | 0.164 | 0 |
| Linear MPC baseline | 44.13 | 0.117 | 0.211 | 0.241 | 0 |
| Linear MPC tuned v1 | 42.78 | 0.102 | 0.240 | 0.311 | 0 |
| Linear MPC tuned v2 | 56.56 | 0.070 | 0.111 | 0.140 | 0 |

### 비교 플롯

`algorithms/f1tenth_kkh/results/track02_kkh_comparison.png` — baseline 2종 +
신규 3종 궤적을 raceline 위에 오버레이 (일반화된 `plot_controller_comparison.py`
`--trajectory` 반복 인자로 생성).

### 분석

- **`mpcc`가 가장 인상적.** 튜닝 전혀 안 한 기본 설정으로 첫 시도에 완주했고,
  mean CTE(0.063m)가 팀이 여러 번 튜닝한 baseline 중 가장 좋은 `Linear MPC
  tuned v2`(0.070m)보다도 낮음. 랩타임(53.79s)은 tuned v2(56.56s)보다 빠름.
  즉 **미튜닝 상태에서 이미 기존 최고 baseline과 동급 이상** — MPCC 정식으로
  튜닝하면 현재 최고 기록을 넘길 가능성이 높음. 대회 Head-to-Head용 장애물
  회피 확장 지점도 이미 갖고 있어 3개 후보 중 **가장 유력**.
- **`lab8`이 가장 빠른 랩타임**(42.53s, `Linear MPC tuned v1`의 42.78s와 거의
  동일)이지만 CTE는 세 후보 중 가장 나쁨(mean 0.113m, max 0.262m) — 곡률 기반
  감속(corner_slowdown_gain)이 없어서 코너에서 그냥 밀어붙이는 대신 정확도를
  희생하는 전형적인 트레이드오프. "단순한 stock 구현도 튜닝된 baseline만큼
  빠르다"는 점에서 팀의 튜닝이 주로 정확도(CTE) 개선에 기여했음을 보여주는
  좋은 대조군 역할은 함.
- **`nonlinear`(SLSQP)는 튜닝이 더 필요.** 기본 cost 가중치(형제 저장소 원본
  값)로 0.55m/s를 시도하니 급커브에서 충돌. 0.35m/s로 낮추니 안정적으로
  완주했지만 랩타임이 가장 느림(63.69s). Cost 가중치(`w_cte`, `w_eth`)를
  올리거나 곡률 기반 감속을 추가하면 개선 여지가 있어 보이나, SLSQP 자체가
  solve_time도 가장 느린 축(mean 16.64ms)이라 세 후보 중 우선순위는 가장 낮음.

### 다음 단계

1. `mpcc`를 최우선으로 소규모 그리드서치 튜닝(`q_contour`/`q_lag`/`q_progress`)
   해서 baseline 기록 경신 여부 확인.
2. `nonlinear`는 곡률 기반 감속 로직을 추가하거나 `w_cte`/`w_eth`를 올려
   0.55m/s에서도 완주하는지 재검증.
3. 세 후보 모두 여러 랩 반복 주행으로 결과 재현성 확인 (1회 시도만으로는
   변동성 파악 불가).
4. 실전 하드웨어(Jetson/NUC/RPi)에서 solve_time_ms 재측정 — 특히 `mpcc`는
   개발 데스크톱에서도 100ms 예산의 1/3 이상을 씀.

## 2026-08-11 (1): 초기 dry-run 검증 (빌드 직후, 스모크 수준)

### 환경
- 맵: `track02`, raceline: `track02_raceline_safe.csv` (116 포인트, 23.14 m)
- 스택: `gym_bridge_launch.py`(sim+AMCL+RViz) → `planning.launch.py` → `f1tenth_kkh mpc_experiment.launch.py controller:=<x>`
- 측정 장비: **개발 데스크톱(16-core)** — 대회 실전 하드웨어(Jetson/NUC/RPi급)가 아님.
  **여기 수치는 상대 비교용이며, 실전 투입 전 반드시 타깃 하드웨어에서 재측정할 것.**
- 각 컨트롤러 `enabled: false` (dry-run) 상태로 15초간 `/f1tenth_kkh/<variant>/solve_time_ms`
  샘플링 (10Hz 기준 약 143개 샘플/컨트롤러). cvxpy 두 변형은 노드 기동 후 첫 solve의
  1회성 컴파일 오버헤드가 지나간 정상 가동(steady-state) 구간 값임.

### dry-run 안전성 체크 (모두 통과)

| 컨트롤러 | `/drive` disabled 중 stop 유지 | predicted_path 발행 | 기동 로그 이상 없음 |
|---|---|---|---|
| `nonlinear` (SLSQP) | ✅ speed=0.0 | ✅ | ✅ |
| `mpcc` | ✅ speed=0.0 | ✅ (`progress` topic도 정상, ≈1.0 = 랩 시작점 근처) | ✅ (DPP 관련 cvxpy 경고만, 예상된 것) |
| `lab8` | ✅ speed=0.0 | ✅ | ✅ |

### solve_time_ms (ms, n=143, 10Hz 제어 주기 → 100ms 예산)

| 컨트롤러 | horizon_steps | mean | p95 | max | min | 100ms 대비 여유 (p95 기준) |
|---|---:|---:|---:|---:|---:|---:|
| `lab8` | 12 | 1.91 | 2.61 | 3.99 | 1.68 | 매우 여유 (~97ms) |
| `nonlinear` (SLSQP) | 10 | 16.64 | 17.94 | 21.52 | 16.08 | 여유 (~82ms) |
| `mpcc` | 10 | 56.66 | 67.83 | 77.10 | 50.44 | **여유 적음 (~32ms)** — 상태 5개/입력 3개 + 매 스텝 장애물 제약(비활성이어도 존재)로 가장 느림 |

참고: `algorithms/control`의 튜닝된 `linear_mpc_node`(MPC_GUIDE.md 기준)는
mean 14.73ms / p95 19.81ms — `lab8`이 그보다도 빠른 것은 동일 계열(kinematic bicycle
+ cvxpy/OSQP)에서 rd_* 변화율 항과 steering-rate 제약을 뺀 단순화 효과로 보임(가설,
정밀 비교는 실제 주행 랩타임/CTE로 별도 확인 필요).

### 결론 / 다음 단계
- 세 컨트롤러 모두 dry-run 안전 기준 통과, 실제 주행(`/control/enable`) 테스트로
  진행 가능한 상태.
- `mpcc`는 세 후보 중 대회(Head-to-Head) 대비 가치가 가장 높지만 solve 여유가 가장
  타이트함 — 실전 하드웨어에서 재측정 전까지 `horizon_steps`/`control_rate` 상향 금지.
- 다음 항목: `closed_loop_test.py`로 각 컨트롤러 스모크(30-60초) → 1-3랩 본 테스트 →
  랩타임/CTE를 `algorithms/control`의 baseline(Pure Pursuit safe, Linear MPC tuned v2)과
  비교.
