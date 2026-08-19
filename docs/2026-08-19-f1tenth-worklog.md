# 2026-08-19 시뮬레이터 재검증 + 코너링/장애물회피 안정성 개선

이 문서는 `docs/2026-08-18-codex-worklog.md`(커밋 `020e118`)에서 중단된 시뮬레이터
검증을 재개하고, "코너링과 장애물 회피 시 안정적인 제어"를 목표로 진행한
세션의 요약이다. 다음 세션에서 "8/19일 작업 이어가자"라고 하면 이 문서를
먼저 읽고 이어간다.

## 적용한 코드 변경

- `launch/gym_bridge_launch.py`
  - `use_sim_time` launch arg 추가, `map_server_node`/`amcl_node`/
    `nav_lifecycle_node`에 하드코딩되어 있던 `use_sim_time: False`를
    파라미터로 노출(기본값은 기존과 동일하게 `false`).
  - 재검증 결과: 이번 세션에서는 `docker restart`로 깨끗하게 재시작한
    launch 두 번 모두 `use_sim_time` 오버라이드 없이도 `map_server`/`amcl`이
    바로 `active [3]`까지 정상적으로 올라왔다. 8/18 세션에서 겪은 수동
    configure/activate 필요 문제는 (이번 재현 범위 안에서는) 순전히
    `pkill -9`로 프로세스만 죽이고 재실행했기 때문이었던 것으로 보이고,
    `use_sim_time` 하드코딩 자체가 원인은 아니었을 가능성이 높다. 다만 이
    수정 자체는 낮은 리스크의 방어적 개선이라 그대로 유지.
- `algorithms/planning/planning/local_avoidance_planner_node.py`
  - `_build_replanned_points()`가 반환하는 `side`/committed geometry를
    raw argmax winner(`best_side`)가 아니라 rate-limited
    `smoothed_target_lateral`의 부호에서 유도하도록 변경. slew 중간
    지점(특히 side 전환 중 d≈0 부근)이 infeasible이면 이전 값으로 revert.
  - `_debounce()`/`_commit()`의 freeze-at-commit 동작은 그대로 유지(변경 없음).
- `algorithms/f1tenth_kkh/f1tenth_kkh/mpcc_node.py`
  - `replan_state_callback()`에서 GLOBAL -> LOCAL_AVOIDANCE_* 전이를
    감지해 `_last_speed_cap_update`를 리셋 -- 회피 에피소드 진입마다
    "cap 한 틱 유지 후 ramp"가 동작하도록 수정(기존엔 노드 부팅 시
    1회만 동작).

## Focused unit test (신규, `scripts/`에 저장 -- 재사용 가능)

이전 세션들은 "curvature focused test", "speed-cap focused test",
동기화된 rclpy watcher를 매번 즉석 스크립트로 짜고 저장하지 않아 다음
세션에서 다시 만들어야 했다. 이번엔 두 개를 파일로 저장:

- `scripts/focused_test_avoidance_slew.py`: `rclpy.init()`만으로 실제
  노드를 인스턴스화하고, `_point_wall_clearance`/`_score_candidate`를
  모킹하고 FakeClock으로 tick을 제어해서 (1) 매 틱 argmax가 다른 side를
  선호해도 committed side가 한 틱 안에 안 바뀌는지, (2) 한쪽이 충분히
  오래(min_flip_ticks 이상) 계속 이기면 결국 flip이 일어나는지(영구
  락이 아님), (3) infeasible한 중간 지점(d≈0 부근)을 향해 전진하지 않고
  이전 값에서 hold하는지 검증. 호스트(rclpy jazzy)/컨테이너(humble)
  양쪽에서 통과 확인.
- `scripts/focused_test_speed_cap_reset.py`: 실제 `MpccNode`를
  인스턴스화(파라미터만 3mps_gentle 설정값으로 덮어씀 -- 기본값
  `max_speed=0.80 &lt; avoidance_speed_cap=1.80`이라 기본값으로는 회피
  진입 시 캡이 아예 안 떨어져서 버그가 안 보임, 주의)해 (1) 최초 진입
  hold-then-ramp(non-regression), (2) **두 번째 이후 회피 에피소드
  진입에서도 hold-then-ramp가 재현되는지(이번 세션 버그의 핵심 재현
  케이스)**, (3) BLOCKED 경유 후 재진입도 정상 동작하는지 검증. 컨테이너
  안에서(cvxpy 등 의존성 필요) 통과 확인.

두 스크립트 모두 `docker exec f1tenth-sim-1 bash -lc "source
/opt/ros/humble/setup.bash && source /sim_ws/install/setup.bash && cd
/sim_ws/src/f1tenth_gym_ros && python3 scripts/focused_test_*.py"`로
재실행 가능(시뮬레이터/토픽 그래프 불필요, 수 초 안에 끝남).

## 실제 시뮬레이터 검증

`docker restart f1tenth-sim-1` -> `colcon build --symlink-install
--packages-select f1tenth_gym_ros f1tenth_kkh planning localization` ->
`ros2 launch f1tenth_gym_ros gym_bridge_launch.py ...`(8/17 워크로그와
동일한 정적 장애물 인자) 순서로 재개, `map_server`/`amcl` 모두 수동
개입 없이 `active`까지 정상 진행, `map -> odom -> ego_racecar/base_link`
TF도 정상 확인.

### 3.0 m/s (`mpcc_params_replan_3mps_gentle_sim.yaml`) -- 실제 충돌 재현

MPCC enable 후 17초 만에 `MPCC disabled: simulator collision reported`.
로그 상 `target_d`가 좌(+0.52) -> 좌(+0.32) -> 우(-0.22) -> 좌(+0.52,
충돌 직전) -> 우(-0.22)로 몇 초에 걸쳐 서서히 드리프트했다. 이 각각의
전환은 이번에 만든 hard slew-rate limit(target_lateral_rate_limit_mps=1.5)
범위 안에서 "정당하게" 일어난 것으로, 틱 단위 급격한 flip은 아니었다 --
즉 이번 수정이 막으려던 문제(순간적 argmax 노이즈)는 실제로 안 나타났다.
그런데도 충돌한 이유: 3.0 m/s에서는 이 몇 초짜리 드리프트 동안 차량이
너무 먼 물리적 거리를 이동해버려서, 마지막 전환이 벽에 바짝 붙은 순간과
겹쳤다. 사용자가 라이브로 "너무 빠르다"고 바로 잡아냄.

### 2.0 m/s (`mpcc_params_replan_straight_sim.yaml`)로 재검증 -- 통과

동일한 조건, 3분+ 연속 주행: 87회 avoidance 판정, 45회 raceline rebuild,
**충돌/safety-stop 0건**. 동일한 좌우 드리프트 패턴 자체는 여전히
관찰됨(근본 원인이 안 고쳐졌으므로 당연함) -- 하지만 속도가 낮아 벽에
닿기 전에 안전하게 재수렴할 시간/거리 여유가 생김. B2(회피 진입마다
cap hold-then-ramp)도 이 3분 동안 여러 번의 GLOBAL->LOCAL_AVOIDANCE
재진입에서 정상 동작 확인(실측 `/drive` speed로 확인, 이전처럼 최초
1회만 동작하던 버그 재현 안 됨).

## 결론 -- 현재 검증된 안전 속도는 2.0 m/s

이번 세션에서 만든 B1(선택 자체의 hard slew-rate limit)은 **틱 단위
급격한 side flip은 확실히 막는다** (focused test로 검증). 하지만
`_build_replanned_points`의 candidate 스코어링 자체가 정지 장애물에서도
몇 초 단위로 서서히 흔들리는 근본 문제(8/18 메모에 이미 "진짜 남은
문제"로 남아있던 것 -- 아마 `_point_wall_clearance`의 grid-cell 양자화 +
`_candidate_laterals_for_side`의 이산 후보 스윕)는 이번 세션에서 손대지
않았다. 3.0 m/s처럼 이 드리프트가 소화할 물리적 여유보다 빠른 속도에서는
여전히 위험하다.

**다음에 3.0 m/s(또는 그 이상)를 다시 시도하려면**: `_build_replanned_points`의
스코어링 안정성 자체를 고쳐야 한다(위 root cause). 후보:
1. `_point_wall_clearance`를 grid-cell 양자화 대신 연속적인 값(예:
   bilinear interpolation)으로 바꿔서 근접 후보 간 미세한 입력 변화에도
   argmax가 안 튀도록.
2. `target_continuity_weight`(현재 0.80)를 높이거나 `_score_candidate`에
   진짜 hysteresis(현재 committed side에 유리한 방향으로 비대칭적
   margin)를 추가.
3. (더 큰 변경) `_candidate_laterals_for_side`의 이산 스윕 대신 연속
   최적화(예: 국소 경사 하강)로 후보 자체를 줄이기.
당장은 사용자가 2.0 m/s를 이번 세션의 검증된 안전 속도로 확정하고
세션을 마무리하기로 결정.

## 2026-08-19 후속 (같은 날 연장 세션): 장애물 회피 파이프라인 재설계 -- 3개 버그 수정, 1개 미해결로 중단

이어서 3.0 m/s 반복 시행(멀티 트라이얼) 안정성을 계속 조사. 트라이얼 2에서
128초 지점 충돌 발견, 회피가 활성 상태에서 발생 -- `scripts/
watch_detected_obstacles.py`(신규, 읽기 전용 진단)로 `/planning/
detected_obstacles`를 직접 찍어보니 **정지 장애물 하나가 근접 거리에서
동시에 2~4개의 별도 클러스터로 쪼개져 검출됨**(s가 5.6~7.4 사이를 1초 내에
왔다갔다, `scan_obstacle_detector_node`의 range-jump 클러스터링이 곡면
근접 물체의 접선 근처에서 쪼개는 게 원인, 시뮬 아님 -- `virtual_obstacle_
scan_node.py`의 ray-circle 교차 수식은 별도로 검증해서 정확함을 확인).
`_select_obstacle`/`_stabilize_obstacle`(obs_s 종방향 앵커)가 이 조각들
사이를 매 틱 옮겨다니며 충돌의 근본 원인이 됨.

사용자가 참고 레포 `CL2-UWaterloo/f1tenth_ws`(로컬에 `refer/f1tenth_ws`로
clone, **`.gitignore`에 추가함, 커밋 안 됨**)의 `stanley_avoidance.py`를
보여주고 "최대한 재사용" 지시 -- 이 파일은 장애물 위치를 아예 추적 안 하고
매 틱 raw scan으로 occupancy grid를 새로 만들어 line-of-sight 체크만
하는 방식. `/plans/dynamic-splashing-brooks.md`에 상세 설계 문서화 후 구현.

**구현한 것**:
- `algorithms/planning/planning/occupancy_grid_utils.py`(신규): `stanley_
  avoidance.py`에서 좌표 인덱스 스켈레톤(Bresenham `traverse_grid`,
  `check_collision`/`check_collision_loose` 구조)만 가져오고, map-frame으로
  재구현(원본은 ego-local + 270도/45도 하드코딩 FOV 가정이라 우리 쪽 scan_
  callback의 이미 TF-보정된 map-frame 포인트가 더 일반적).
- `local_avoidance_planner_node.py`: `_select_obstacle`/`_stabilize_
  obstacle`/`obstacle_confirm_ticks`/클러스터 구독 전부 제거. 대신
  `_build_local_grid`(매 틱 raw scan으로 그리드 재생성, 상태 없음) +
  `_first_blocked_point`(ego_s부터 raceline을 따라 전진 스캔, live scan
  clearance가 부족해지는 첫 지점의 s를 반환 -- 클러스터 identity에 전혀
  의존 안 하므로 조각남에 영향 안 받음) + `_lateral_bias_at` +
  `_synthetic_obstacle`(옛 `_select_obstacle`과 동일한 6-tuple shape으로
  포장해서 `_build_points_for_target`/`_score_candidate`/
  `_build_replanned_points`는 안 건드림 -- "연결만, 재설계는 없는 부분만"
  지시 그대로).
- 신규 포커스드 테스트 `scripts/focused_test_occupancy_grid.py`(6개 체크,
  fragmentation invariance 직접 회귀 테스트 포함) + 기존 3개 테스트 전부
  통과 확인.

**라이브 테스트에서 순서대로 발견/수정한 버그 3개** (포커스드 테스트로는
전혀 안 잡힘, 전부 실제 시뮬 주행에서만 드러남):
1. **직선거리 코너 컷 문제**: `_corridor_blocked`가 ego→8m 앞 lookahead
   지점까지 **직선**으로 collision 체크 -- 트랙이 곡선이라 직선이 코너를
   가로질러 실제 레이스라인은 안전한 벽을 잘못 침. 차가 시작 위치에서
   15초+ 전혀 안 움직이는 것으로 발견(odom 위치 고정 확인). 수정: raceline을
   따라 세그먼트별로 체크하도록 변경.
2. **ramp 앵커 위치 오류**: 위 수정 후에도 여전히 멈춤 -- 알고 보니 ramp
   중심(`obs_s`)을 실제 장애물 위치가 아니라 **고정 lookahead 거리**(ego_s+8m)
   에 앵커링하고 있었음. 실제 장애물(s=6.74)이 ramp-in 이징 구간 안에
   걸려서 전체 강도로 안 밀림 -> 어느 쪽으로 옵셋을 줘도 obs≈0.02m(거의
   붙음)로 항상 infeasible. 수정: `_first_blocked_point`로 실제 라이브
   데이터에서 막히는 지점을 찾아 거기에 앵커링(여전히 상태 없음, 매 틱
   재계산).
3. **클리어런스 이중 계산**: 그리드 inflate 반경에 `obstacle_radius+
   safety_margin`(0.26m)을 넣고 `_score_candidate`의 corridor_clear
   체크에서 차량 반폭(0.155m)을 또 마진으로 더해서 요구 클리어런스가
   0.415m로 뻥튀기됨(원래 검증된 값은 0.235m, `scan_obstacle_clearance`).
   스캔 포인트는 이미 장애물의 실제 표면 위에 있으므로 `obstacle_radius`를
   또 더하면 안 됨 -- 그리드 inflate는 `safety_margin`만 쓰도록 수정.

**미해결로 세션 중단**: 위 3개 수정 후 2.0 m/s 라이브 테스트에서 차가
정상 주행(거의 한 바퀴 완주)했지만, **19.9초 지점에서 실제 충돌** 발생.
로그 분석: 같은 장애물 인카운터에서 `avoidance selected left target_d=0.52`가
**16초간 그대로 얼어붙어 있었음**(`_debounce`의 freeze-at-commit -- state
label이 GLOBAL/LOCAL_AVOIDANCE_LEFT/RIGHT/BLOCKED 중 하나로 유지되는 동안
committed geometry가 갱신 안 됨, 8/18에 원래 이유가 있어 의도적으로 넣은
동작). 그 16초 동안 매 틱 새로 계산되는(로그만 찍히고 발행은 안 되는)
"avoidance candidates" 점수는 obs 0.41→0.22로 서서히 나빠지다 결국 충돌.

역설적 원인: 오늘 고친 fragmentation 버그 덕분에 시스템이 예전보다 훨씬
더 안정적으로 같은 방향에 확신을 갖게 됨 -> state label이 한 번도 안
바뀌어서 freeze가 16초나 지속됨(예전엔 불안정해서 state가 자주 바뀌며
의도치 않게 geometry가 자주 리프레시됐던 것). `_debounce`의 주석에 이미
"장애물이 실제로 움직이면 안 맞을 수 있다"고 경고돼 있었는데, 여기선
장애물이 아니라 **차량 자신이 16초간 상당히 이동**하면서 같은 문제가
드러남. `dynamic-splashing-brooks.md` 계획서의 "Phase 2(freeze 완화)는
나중에, Phase 1은 그대로 유지"가 예상했던 정확히 그 리스크가 실제로
발현된 것 -- Phase 1 그대로는 충분하지 않았다는 뜻.

**다음 세션에서 이어할 것 (우선순위 순)**:
1. `_debounce`/`_commit`의 freeze-at-commit 메커니즘을 다시 설계해야 함.
   완전히 매 틱 갱신(예전에 두 번 실패)도, 완전히 얼리기(지금 실패)도
   아닌 중간 지점 필요 -- 예: 상태 라벨은 그대로 debounce하되 committed
   geometry는 주기적으로(예: 1~2초마다) 갱신, 또는 ego가 obs_s에 얼마나
   가까워졌는지에 따라 갱신 주기를 조절.
2. 2.0 m/s부터 다시 검증(3분+, 이번 리팩터로 인한 회귀 없는지) -> 3.0 m/s
   멀티 트라이얼로 진행.
3. RViz로 최종 시각 확인은 아직 안 함(2.0 m/s 회귀 통과 후 계획서 (c) 단계).
4. `local_avoidance_planner_node.py`가 더 이상 `/planning/detected_obstacles`를
   구독하지 않음 -- `scan_obstacle_detector_node.py` 자체는 그대로 두었고
   (다른 소비자 없음, 안전하게 확인함) launch에서도 안 건드림, 언젠가
   정리하고 싶으면 `obstacle_detection` launch arg로 끌 수 있음.
5. `refer/f1tenth_ws`는 `.gitignore`에 추가됨 -- 참고용 로컬 clone,
   커밋 안 됨. 다시 필요하면 `stanley_avoidance.py`의 `check_collision`/
   `traverse_grid` 패턴을 다시 참고할 것.
