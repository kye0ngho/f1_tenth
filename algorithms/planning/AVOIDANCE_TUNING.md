# 회피/속도 튜닝 가이드 -- 파라미터가 "뭘 하는지" (정답 숫자 아님)

이 문서는 각 파라미터가 **기계적으로 무엇을 하는지**만 설명한다. "이 값으로
설정하라"는 답은 의도적으로 담지 않았다 -- IFAC 2026 Busan RoboRacer 실제
트랙(최소 폭 1m, 저마찰 콘크리트+우레탄 코팅)은 아직 실측 전이고, 최종 값은
직접 SLAM한 트랙에서 실측하며 정해야 한다. 이 문서의 역할은 "파라미터 X를
바꾸면 무슨 일이 일어나는지"를 빠르게, 경험적으로 확인할 수 있는 도구를
설명하는 것.

## 빠른 실험 루프 (제너레이트 → 실행 → 관찰 → 비교)

맵 형태는 **둥근 모서리 직사각형**(스타디움 아님) -- `--width-m`/`--length-m`
(전체 바운딩박스, 기본값이 대회 공식 규격 8x22m)과 `--corner-radius-m`(코너
회전 반경, 폭/길이와 완전히 독립적으로 조절 가능)을 따로 조절할 수 있어서,
직선 가감속 테스트(긴 직선, 완만한 코너)와 실제 코너링 테스트(짧은 코너
반경)를 원하는 대로 조합할 수 있다.

```bash
# 1) 원하는 규격의 합성 트랙 맵 생성 (몇 초 안에 재생성 가능)
#    기본값 = 대회 공식 규격(8m x 22m), corridor 1.0m, 코너 반경 0.6m
python3 algorithms/planning/scripts/generate_synthetic_corridor_map.py \
  --corridor-width-m 1.00 --width-m 8.0 --length-m 22.0 \
  --corner-radius-m 0.6 \
  --output-yaml maps/synthetic_competition_22x8m.yaml \
  --output-image maps/synthetic_competition_22x8m.png

# 2) 매칭되는 waypoint CSV 생성 (기존 도구 그대로 사용) -- 스크립트가 출력한
#    "Next: ..." 줄의 --start-x/--start-y를 그대로 쓸 것 (트랙 형태 바뀌면
#    시작 좌표도 바뀜)
python3 algorithms/planning/scripts/generate_centerline.py \
  --map-yaml maps/synthetic_competition_22x8m.yaml \
  --output algorithms/planning/waypoints/synthetic_competition_22x8m_centerline.csv \
  --start-x 0.0 --start-y -3.5 --speed 5.0

# 3) (선택) 클리어런스 확인
python3 algorithms/planning/scripts/validate_raceline.py \
  --map-yaml maps/synthetic_competition_22x8m.yaml \
  --raceline algorithms/planning/waypoints/synthetic_competition_22x8m_centerline.csv \
  --preview /tmp/validate_preview.png

# 4) sim을 이 맵으로 띄우고, 바꿔보고 싶은 파라미터를 launch arg로 오버라이드
ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  map_path:=/root/maps/synthetic_competition_22x8m \
  waypoint_csv:=/sim_ws/src/planning/waypoints/synthetic_competition_22x8m_centerline.csv \
  controller:=pure_pursuit auto_enable:=true local_replanner:=true \
  virtual_obstacles:=true virtual_obstacle_list:='-8.5,-3.35,0.25' \
  reset_pose:=false \
  avoidance_max_lane_offset_m:=0.35

# reset_pose:=false인 이유: start_pose_msg가 track02 스폰 지점 기준이라
# 합성 맵에는 안 맞음 -- RViz의 "2D Pose Estimate"로 직접 찍어주거나
# /sim_reset_pose + /initialpose를 맵 시작 좌표로 발행할 것.

# 5) 관찰 -- 두 스크립트를 다른 터미널에서
python3 scripts/watch_replan_state_vs_obstacle_distance.py \
  --waypoint-csv algorithms/planning/waypoints/synthetic_competition_22x8m_centerline.csv \
  --obstacle-x -8.5 --obstacle-y -3.35

python3 scripts/watch_lateral_deviation.py

# 6) 파라미터 하나 바꿔서 재실행, 출력된 요약 숫자 비교
```

`watch_replan_state_vs_obstacle_distance.py`의 "BLOCKED 비율" 숫자와
`watch_lateral_deviation.py`의 max/RMS lateral_deviation 숫자가 파라미터
변경 전후를 비교할 수 있는 핵심 지표다.

- `--corridor-width-m`을 0.85/1.00/1.30로 바꿔가며 재생성하면, 같은 파라미터
  세트가 폭이 좁아질수록 어떻게 무너지는지(또는 안 무너지는지) 직접 볼 수 있다.
- `--corner-radius-m`을 줄이면(예: 0.4~0.6m) 급격한 코너링 성능을, `--length-m`/
  `--width-m`을 키우면 고속 직선 가감속을 각각 독립적으로 테스트할 수 있다.
- `focused_test_occupancy_grid.py`의 Test 8은 기본값으로 생성된
  `maps/synthetic_competition_22x8m.yaml`을 fixture로 그대로 사용한다 --
  트랙 규격을 바꿔서 재생성하면 이 테스트도 그 맵 기준으로 다시 확인해볼 것.

## `local_avoidance_planner_node` 파라미터

- **`max_lane_offset_m`**: 중심선 기준 좌우로 밀 수 있는 절대 상한. 코리도어
  폭이 `2*max_lane_offset_m + vehicle_width_m`보다 좁으면, 벽 클리어런스를
  체크하기도 전에 이 상한 자체가 회피를 불가능하게 만든다. 값을 낮추면 넓은
  구간에서는 회피 여유가 줄고, 좁은 구간에서는 애초에 불가능한 후보를
  덜 시도하게 된다(무해).
- **`min_wall_clearance_m`**: 후보 경로가 정적 맵(`/map`) 상 벽으로부터
  유지해야 하는 최소 거리. 낮추면 더 좁은 통로를 "가능"으로 판정하지만
  실제 여유는 그만큼 줄어든다 -- 안전 마진이지 튜닝 여유가 아님, 함부로
  낮추지 말 것.
- **`vehicle_width_m` / `safety_margin_m`**: 클리어런스 요구치 계산에
  들어가는 차량 폭과 추가 안전 여유(`0.5*vehicle_width_m + safety_margin_m`
  또는 `+obstacle_radius_m` 형태로 조합됨, `local_avoidance_planner_node.py`
  참고).
- **`obstacle_path_clearance_m`**: 장애물 중심으로부터 유지해야 하는 최소
  거리 -- 이 값이 크면 회피 오프셋이 커지고, 코리도어가 좁을수록
  `max_lane_offset_m`과 충돌할 가능성이 커진다.
- **`ramp_in_m` / `ramp_out_m`**: 장애물 접근/이탈 시 오프셋이 0에서
  목표값까지 부드럽게 커지는/줄어드는 구간 길이. 속도가 빠를수록 같은
  물리적 거리를 지나는 시간이 짧아지므로, 속도를 올렸다면 이 값도 같이
  늘려야 램프가 끝나기 전에 장애물에 도달하는 상황을 피할 수 있다.
- **`state_switch_hold_s`**: `_debounce`가 새 상태(BLOCKED/
  LOCAL_AVOIDANCE_*)를 실제로 커밋하기 전에 그 상태가 연속으로 이겨야 하는
  시간. **2026-08-20/21에 발견**: 8Hz 틱 주기(0.125s)에서 이 값을 0.15s로
  줄이면 노이즈 필터링이 거의 안 돼서, 벽/장애물 클리어런스가 판정
  경계선에서 매 틱 뒤집히는 정상적인 상황(장애물을 스칠 때 클리어런스가
  몇 cm 차이로 왔다갔다 하는 것)에서 상태가 계속 리셋되며 첫 판정에
  갇혀버린다 -- 첫 판정이 우연히 BLOCKED였다면 그게 그대로 굳어서
  장애물을 관통하는 경로가 발행된다. 0.30s(틱 주기의 ~2.4배)가 검증된
  기본값. 너무 크게 올리면 진짜 회피가 필요한 순간에도 반응이 늦어짐 --
  `ramp_in_m`과의 트레이드오프.
- **`default_side`**: 장애물의 좌우 어느 쪽도 확실히 유리하지 않을 때
  기본으로 미는 방향.

## `speed_profile_node` 파라미터

- **`lateral_accel_limit`**: 커브 곡률 기반 속도 상한을 계산하는 데 쓰이는
  가정 횡가속도 한계 (`v = sqrt(lateral_accel_limit / |kappa|)`). 실제
  노면 마찰이 이 값보다 낮으면 코너에서 슬립함 -- 저마찰 노면(대회 공식
  트랙: 콘크리트+우레탄 코팅)에서는 sim 튠 값(5.5)을 그대로 믿지 말 것.
- **`accel_limit` / `decel_limit`**: 직선 구간에서 속도가 얼마나 빨리
  올라가고/내려갈 수 있는지의 상한. 노면 마찰이 낮으면 특히 `decel_limit`을
  낙관적으로 잡으면 제동 거리가 예상보다 길어짐 -- stall watchdog/회피
  타이밍과도 연결됨.
- **`corner_slowdown_gain`**: (fallback 경로에서만 쓰임, `use_speed_profile`이
  꺼졌거나 profile이 stale할 때) 조향각 크기에 비례해 속도를 얼마나
  깎을지.

## `pure_pursuit_node`의 새 파라미터 (2026-08-21 추가)

- **`avoidance_speed_cap` / `blocked_speed_cap` / `avoidance_cap_decel_mps2`
  / `avoidance_cap_accel_mps2`**: `/planning/replan_state`를 구독해서
  BLOCKED면 즉시(램프 없이) `blocked_speed_cap`으로, LOCAL_AVOIDANCE_*면
  `avoidance_cap_decel_mps2`/`avoidance_cap_accel_mps2` 속도로 램프하며
  `avoidance_speed_cap`으로 속도를 캡한다. `f1tenth_kkh/mpcc_node.py`의
  이미 검증된 로직을 그대로 이식한 것 -- 단, mpcc의 `max_speed`는 1.0m/s라
  `avoidance_speed_cap=1.80`이 실제로는 한 번도 1.0 이상으로 작동해본 적이
  없다. pure_pursuit의 실제 속도대(최대 5.5m/s)에서는 검증 안 된 시작값이니
  이 문서의 실험 루프로 제일 먼저 확인해볼 후보.
- **`stall_speed_threshold_mps` / `stall_command_speed_threshold_mps` /
  `stall_timeout_s`**: 명령 속도가 `stall_command_speed_threshold_mps`
  이상인데 실측 속도(`/car_state/odom`의 twist)가
  `stall_speed_threshold_mps` 미만인 상태가 `stall_timeout_s`초 지속되면
  자동으로 `/control/enable false`와 동일하게 정지한다. 좁은 코리도어에서
  저속으로 크리핑하는 정상 상황과 진짜 멈춤(벽에 끼임 등)을 구분하는 게
  이 파라미터들의 역할 -- 실제 좁은 코너 통과 속도를 실측하기 전엔
  false-positive 여부를 확신할 수 없음, 반드시 실측 후 조정.

## Kill switch (2026-08-21 추가)

공식 룰: "toggle 방식(누르고 있는 방식 아님)의 kill switch를 소프트웨어
인스펙션에서 시연해야 함." 표준 F1TENTH 방식은 **VESC 펌웨어 레벨**(App
Settings → General → Kill Switch Mode)에서 RC 수신기 신호선에 물린 물리
토글로 차단 -- 이건 온보드 컴퓨터/ROS와 완전히 무관하게 동작해서, 소프트웨어가
멈추거나 크래시해도 여전히 작동한다. **이게 진짜 준수 요건.**

`vehicle_interface_node`(`algorithms/vehicle_interface/`)에는 이미
`/safety/stop_required`(`std_msgs/Bool`) 훅이 있었지만 아무도 발행하지 않는
상태였다 -- 이번에 `gym_bridge_launch.py`에도 `vehicle_interface_node`를
추가해서(기존엔 `autonomy.launch.py`에만 있었음) sim에서도 이 경로를 테스트할
수 있게 했다. 수동 확인:

```bash
ros2 topic pub /safety/stop_required std_msgs/msg/Bool "{data: true}"
# -> 즉시 정지, pure_pursuit가 뭘 명령하든 무시하고 유지되는지 확인
ros2 topic pub /safety/stop_required std_msgs/msg/Bool "{data: false}"
# -> 깨끗하게 재개되는지 확인
```

**이 ROS 레벨 레이어는 VESC 하드웨어 킬스위치를 대체하지 않는다.** 보조
레이어일 뿐 -- 실차 하드웨어에 VESC Kill Switch Mode가 실제로 설정/배선돼
있는지는 별도로 물리적으로 확인해야 한다(VESC Tool 연결 후 App Settings →
General → Kill Switch Mode 확인, RC 수신기 신호선에 토글 스위치가 인라인으로
물려있는지 확인). 처음 토글 테스트는 반드시 바퀴를 땅에서 띄운 상태로
할 것 -- 일부 VESC 펌웨어 버전은 킬스위치 해제 시 모터가 예상치 못하게
튀는 버그가 있었음(VESC 프로젝트 포럼 확인됨).
