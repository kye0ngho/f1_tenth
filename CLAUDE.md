# CLAUDE.md — F1TENTH ROS2 Humble Autonomy Stack

## 에이전트 운영 원칙

Claude Code가 자율적으로 작업한다. 아래 원칙을 따른다:

- **확인 없이 진행**: 파일 생성/수정/빌드/실행은 모두 자율 수행
- **커밋**: 사용자가 명시적으로 요청할 때만 커밋
- **금지 작업**: `rm -rf`, 워크스페이스 외부 경로 수정, git force push
- **작업 단위**: 노드 하나를 완전히 구현 → 빌드 → 검증까지 한 사이클로 완료
- **막히면**: 에러 로그를 분석하고 3회까지 자가 수정 시도. 그래도 안 되면 원인과 시도 내역을 보고
- **완성 기준**: 빌드 성공 + 시뮬에서 토픽 출력 확인 + params.yaml 파라미터 동작 확인

## 자가 수정 루프

```bash
# 표준 작업 사이클
colcon build --packages-select <패키지> 2>&1 | tee /tmp/build.log
# 빌드 실패 시: /tmp/build.log 분석 → 수정 → 재빌드
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim &
sleep 3 && ros2 topic list | grep <예상_토픽>
# 토픽 없으면: ros2 node list, ros2 doctor 로 진단
```

## 검증 체크리스트 (노드 완성 기준)

새 노드 구현 후 반드시 확인:
- [ ] `colcon build` 에러 없음
- [ ] `ros2 node list`에 노드 등재
- [ ] 출력 토픽 `ros2 topic echo --once` 로 데이터 확인
- [ ] `params.yaml` 값 변경 시 노드 동작 반영
- [ ] `autonomy.launch.py`에 등록됨

---

## 프로젝트 목표

F1TENTH 1:10 스케일 자율주행 레이싱카 개발.
ROS2 Humble 기반 시뮬레이터에서 알고리즘을 개발하고, 최종적으로 실제 차량에 배포한다.

---

## 환경

| 항목 | 값 |
|------|-----|
| ROS | ROS2 Humble |
| 언어 | Python (ament_python) |
| 컨테이너 | Docker + rocker (X11) |
| 워크스페이스 | `/sim_ws/` |

### 소스 마운트 구조

```
./algorithms/localization   → /sim_ws/src/localization
./algorithms/planning       → /sim_ws/src/planning
./algorithms/control        → /sim_ws/src/control
./algorithms/safety         → /sim_ws/src/safety
./algorithms/f1tenth_bringup → /sim_ws/src/f1tenth_bringup
```

### 주요 명령어

```bash
# 빌드
colcon build --packages-select localization planning control f1tenth_bringup

# 시뮬 실행
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim

# 실차 실행
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=real
```

---

## 패키지 구조

```
algorithms/
├── localization/       # 현재: odom 패스스루. 목표: particle filter
├── planning/           # waypoint_planner_node, waypoint_recorder_node
├── control/            # pure_pursuit_node (PID 속도 제어 포함)
├── safety/             # safety_brake_node (LiDAR TTC 기반 긴급 정지)
└── f1tenth_bringup/    # autonomy.launch.py — 전체 스택 통합
```

새 패키지를 만들 때도 동일한 구조를 따른다:

```
algorithms/<패키지명>/
├── config/params.yaml
├── launch/<패키지명>.launch.py
├── <패키지명>/<노드명>.py
├── package.xml
└── setup.py
```

---

## 토픽 흐름

```
/ego_racecar/odom
      ↓
localization_node → /localization/odom
      ↓
pure_pursuit_node ← /planning/path → /pure_pursuit/drive ──┐
                                                            ↓
gap_follow_node ← /scan ──────────── /gap_follow/drive ──→ behavior_selector_node
                                                            │ (/scan으로 자동 전환)
                                                            ↓
                                                     /control/drive
                                                            ↓
                                              safety_brake_node ← /scan
                                                            ↓
                                                         /drive (최종)

waypoint_recorder_node → waypoints.csv
velocity_profile_node  → waypoints.csv (속도 최적화)
waypoint_planner_node  ← waypoints.csv → /planning/path, /planning/markers
lap_timer_node ← /ego_racecar/odom → /lap_timer/lap_time, /lap_timer/status
```

---

## 핵심 파라미터

### 차량 물리 (F1TENTH 1:10 스케일)

| 파라미터 | 값 |
|----------|-----|
| wheelbase | 0.33 m |
| max_steering_angle | 0.4189 rad (~24°) |

### Pure Pursuit

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| lookahead_distance | 1.0 m | 전방 추종 거리 |
| target_speed | 1.0 m/s | 목표 속도 |
| min_speed | 0.4 m/s | 최저 속도 |
| max_speed | 2.0 m/s | 최고 속도 |
| corner_slowdown_gain | 0.5 | 코너에서 최대 50% 감속 |
| control_rate | 30 Hz | 제어 루프 주기 |

### PID 속도 제어

| 파라미터 | 값 |
|----------|-----|
| kp | 1.0 |
| ki | 0.0 |
| kd | 0.05 |

### 실차 변환

| 파라미터 | 값 |
|----------|-----|
| speed_to_erpm_gain | 3000.0 |
| speed_to_erpm_offset | 0.0 |
| servo_center | 0.5 |
| servo_gain | 1.0 |
| servo_min / servo_max | 0.0 / 1.0 |

### Waypoint Recorder

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| min_distance | 0.2 m | 웨이포인트 최소 간격 |
| speed_smooth_window | 5 | 속도 이동 평균 윈도우 |
| loop_closure_distance | 0.5 m | 루프 클로저 감지 반경 |
| loop_closure_min_waypoints | 20 | 루프 클로저 활성화 최소 웨이포인트 수 |
| autosave_interval | 30.0 s | 자동 백업 저장 주기 |

Recorder 서비스:

```bash
ros2 service call /waypoint_recorder_node/record std_srvs/srv/SetBool "{data: true}"   # 녹화
ros2 service call /waypoint_recorder_node/record std_srvs/srv/SetBool "{data: false}"  # 정지
ros2 service call /waypoint_recorder_node/save std_srvs/srv/Trigger {}                 # 저장
ros2 service call /waypoint_recorder_node/clear std_srvs/srv/Trigger {}               # 초기화
```

CSV 컬럼: `x, y, yaw, speed`

---

## 현재 완성된 노드

| 노드 | 패키지 | 상태 | 설명 |
|------|--------|------|------|
| localization_node | localization | 완성 | 시뮬 odom 패스스루 |
| particle_filter_node | localization | 완성 | MCL — likelihood field. use_particle_filter:=true로 활성화 |
| waypoint_recorder_node | planning | 완성 | 텔레옵 경로 기록, 루프 클로저, 속도 스무딩, 서비스 제어 |
| waypoint_planner_node | planning | 완성 | CSV → /planning/path + MarkerArray (2Hz) |
| velocity_profile_node | planning | 완성 | 곡률 기반 속도 최적화 → CSV 덮어쓰기 |
| lap_timer_node | planning | 완성 | 출발점 통과 감지 → 랩 타임 측정/로그 |
| data_logger_node | planning | 완성 | 주행 상태 CSV 저장 (x,y,yaw,speed,steering,braking,mode) |
| waypoint_manager_node | planning | 완성 | 다중 CSV 관리, 런타임 전환, 재로드 서비스 |
| goal_pose_planner_node | planning | 완성 | /goal_pose → /planning/goal_path (선형 보간) |
| pure_pursuit_node | control | 완성 | Pure Pursuit + PID 속도 제어. 출력: /pure_pursuit/drive |
| gap_follow_node | control | 완성 | Follow the Gap 반응형 회피. 출력: /gap_follow/drive |
| behavior_selector_node | control | 완성 | /scan 기반 waypoint ↔ gap_follow 자동 전환 |
| mpc_node | control | 완성 | Kinematic Bicycle + SLSQP. 출력: /mpc/drive. N=10, dt=0.1s |
| debug_marker_node | control | 완성 | RViz 종합 디버그 마커 (차량/경로/LiDAR/모드 텍스트) |
| vehicle_interface_node | control | 완성 | 실차 VESC 인터페이스 + 조이스틱 수동/자율 전환 |
| adaptive_pure_pursuit_node | control | 완성 | 속도 적응형 룩어헤드 (L_d=k_v*v+L_min) + 전방 곡률 피드포워드 감속 |
| safety_brake_node | safety | 완성 | TTC 긴급 정지. /control/drive → /drive 인터셉터 |
| scan_preprocessor_node | safety | 완성 | NaN 제거 + 미디언 필터. /scan → /scan_processed |
| watchdog_node | safety | 완성 | 토픽 헬스 감시, 타임아웃 시 비상정지 |
| opponent_tracker_node | safety | 완성 | LiDAR 클러스터링 → 동적 장애물 추적 + 속도 추정 (EMA) |
| emergency_recovery_node | safety | 완성 | 고착 감지 (2s) → 후진→회전→재출발 자동 복구 |
| telemetry_node | planning | 완성 | 속도/제동/모드/랩타임 집계 → JSON 퍼블리시 + jsonl 로그 |
| sector_timer_node | planning | 완성 | 트랙 구간 타이머. CSV 정의 섹터 순서 통과 감지 + 베스트 기록 |
| disparity_extender_node | control | 완성 | Disparity Extender — 차폭 기반 장애물 그림자 확장. gap_follow보다 정밀 |
| stanley_controller_node | control | 완성 | Stanley 횡방향 제어 (δ = ψ_e + atan(k·e/v)). CTE 직접 보정 |
| race_line_optimizer_node | planning | 완성 | Laplacian Smoothing 최소 곡률 최적화 → race_line.csv 저장 |
| overtake_planner_node | planning | 완성 | 장애물 감지 → 횡방향 오프셋 경로 자동 생성 (/planning/routed_path) |
| trajectory_evaluator_node | planning | 완성 | CTE (현재/RMS/최대) + 속도 오차 실시간 계산 + CSV 저장 |

---

## 확장 포인트

- **MPC를 기본 컨트롤러로**: `behavior_selector`의 `waypoint_drive_topic` → `/mpc/drive`
- **Stanley 사용**: `behavior_selector`의 `waypoint_drive_topic` → `/stanley/drive`
- **Disparity Extender 사용**: `behavior_selector`의 `gap_drive_topic` → `/disparity_ext/drive`
- **오버테이크 활성화**: `pure_pursuit`의 `path_topic` → `/planning/routed_path`
- **레이싱 라인 적용**: `waypoint_planner`의 `waypoint_csv` → `/sim_ws/src/planning/waypoints/race_line.csv`
- **실차 배포**: `drive_mode:=real` + `use_particle_filter:=true` + `vehicle_interface_node` 활성화
- **전처리 scan 사용**: `gap_follow_node` / `safety_brake_node`의 `scan_topic` → `/scan_processed`
- **섹터 정의**: `/sim_ws/src/planning/waypoints/sectors.csv` (x,y,radius,name 컬럼)

---

## 코딩 규칙

- 파라미터는 반드시 `declare_parameter` 후 `params.yaml`에 기본값 정의
- 노드 시작 시 모든 파라미터 값을 `get_logger().info()`로 출력
- `drive_mode` (`sim` / `real`) 분기를 항상 고려해 구현
- 주석은 WHY가 명확한 경우에만 작성 (WHAT은 코드로 충분)
- 새 노드 추가 시 `autonomy.launch.py`에도 반드시 등록
- `std_srvs` 서비스로 런타임 제어를 노출하는 것을 권장
