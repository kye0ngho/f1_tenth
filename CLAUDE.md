# CLAUDE.md — F1TENTH ROS2 Humble Autonomy Stack

## 에이전트 운영 원칙

Claude Code가 자율적으로 작업한다. 아래 원칙을 따른다:

- **확인 없이 진행**: 파일 생성/수정/빌드/실행은 모두 자율 수행
- **커밋 규칙**: 노드 하나 완성될 때마다 한국어 커밋 메시지로 커밋 (푸시는 사용자가 직접)
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
localization_node
      ↓
/localization/odom, /localization/pose
      ↓
pure_pursuit_node ← /planning/path
      ↓
/control/drive  (raw — 안전 검사 전)
      ↓
safety_brake_node ← /scan (LiDAR)
      ↓
/drive  (AckermannDriveStamped — 시뮬레이터 입력)
/safety/braking (Bool — 제동 상태)

waypoint_recorder_node → waypoints.csv (x, y, yaw, speed)
waypoint_planner_node  ← waypoints.csv → /planning/path, /planning/markers
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

| 노드 | 상태 | 설명 |
|------|------|------|
| localization_node | 완성 (기능 제한) | 시뮬 odom 패스스루. 실차용 localization 미구현 |
| waypoint_recorder_node | 완성 | 텔레옵 경로 기록, 루프 클로저, 속도 스무딩, RViz 마커, 서비스 제어 |
| waypoint_planner_node | 완성 | CSV → Path + MarkerArray 퍼블리시 (2Hz) |
| pure_pursuit_node | 완성 | Pure Pursuit + PID 속도 제어, sim/real 두 모드. 출력: /control/drive |
| safety_brake_node | 완성 | LiDAR TTC 기반 긴급 정지. /control/drive → /drive 인터셉터 |

---

## 다음 구현 노드 (우선순위 순)

### 1. `safety_brake_node` ✅ 완성

### 2. `velocity_profile_node`

웨이포인트 CSV의 곡률을 분석해 구간별 최적 속도를 재계산.
직선 → max_speed, 코너 → 곡률에 반비례해 감속.
출력: 속도가 최적화된 새 waypoints.csv

### 3. `particle_filter_node` (또는 nav2 AMCL 연동)

LiDAR + 맵 기반 실제 위치 추정. localization_node를 대체.
시뮬에서는 odom이 ground truth이므로 실차 배포 단계에서 필요.

### 4. `gap_follow_node`

LiDAR 기반 반응형 장애물 회피 (Follow the Gap).
behavior_selector_node와 함께 waypoint 추종 ↔ 장애물 회피 전환.

### 5. `lap_timer_node`

출발점 통과 감지 → 랩 타임 측정 및 로그 저장.

### 6. `mpc_node` (장기 목표)

Model Predictive Control로 Pure Pursuit 대체. 고속 정밀 주행.

---

## 코딩 규칙

- 파라미터는 반드시 `declare_parameter` 후 `params.yaml`에 기본값 정의
- 노드 시작 시 모든 파라미터 값을 `get_logger().info()`로 출력
- `drive_mode` (`sim` / `real`) 분기를 항상 고려해 구현
- 주석은 WHY가 명확한 경우에만 작성 (WHAT은 코드로 충분)
- 새 노드 추가 시 `autonomy.launch.py`에도 반드시 등록
- `std_srvs` 서비스로 런타임 제어를 노출하는 것을 권장
