# 2026-08-18 Codex 작업 요약

이 문서는 Codex가 2026-08-18에 진행한 정적 장애물 및 코너링 안정화 작업의 재개 지점이다.

## 목표

- 동적 장애물은 범위에서 제외한다.
- 정적 장애물 회피와 코너링 안정성을 우선 검증한다.
- 다음 세션에서 사용자가 `8/18일 작업 이어가자`라고 하면 이 문서를 먼저 읽고 이어간다.

## 적용한 코드 변경

- `algorithms/planning/planning/local_avoidance_planner_node.py`
  - 후보 scoring에 이전 lateral target과의 연속성 비용을 추가했다.
  - scoring 동점 시 기존 target과 committed side를 우선한다.
  - 빈 active-index 후보에도 `continuity_cost`를 반환하도록 수정했다.
  - 기존의 side lock, target slew, state debounce, static geometry freeze 로직을 유지했다.
- `algorithms/planning/planning/path_utils.py`
  - 폐곡선 path seam에서 yaw 차이를 `atan2(sin, cos)`로 계산하도록 수정했다.
  - curvature와 corner outside-side 판단의 seam wrap 오류를 줄였다.
- `algorithms/f1tenth_kkh/f1tenth_kkh/mpcc_node.py`
  - 정적 장애물 회피 cap의 진입/복귀 ramp 파라미터를 추가했다.
  - 회피 첫 tick은 현재 cap을 유지하고 다음 tick부터 bounded ramp를 적용한다.
  - `BLOCKED`는 즉시 `blocked_speed_cap`으로 내려간다.

## 검증 결과

- `PYTHONPYCACHEPREFIX=/tmp/f1tenth-pycache python3 -m compileall algorithms/planning/planning algorithms/f1tenth_kkh/f1tenth_kkh`: 통과
- `git diff --check`: 통과
- 폐곡선 원형 path curvature focused test: 통과. curvature 약 `1.0001`로 seam spike 없음.
- speed-cap focused test: 통과. 회피 첫 tick cap 유지, 이후 `1.5 m/s^2` 감속, BLOCKED 즉시 정지, 복귀 ramp 확인.
- planning pytest: `1 skipped, 2 failed`. 기능 테스트가 아니라 저장소 전체 flake8/pep257 검사이며 기존 저장소에 대량 lint 오류가 있다.

## 실제 시뮬레이터 검증 상태

- 컨테이너: `f1tenth-sim-1`
- 최신 source를 `/sim_ws`에 bind한 상태에서 `planning`, `f1tenth_kkh`를 symlink build했다.
- 정적 장애물 실행 조건: obstacle `(6.74, 0.41, 0.18)`, `target_speed:=2.8`, `max_speed:=3.0`, `obstacle_interest_horizon:=4.0`, `speed_profile_decel:=1.1`.
- 첫 측정은 기존 process의 clock/TF가 오래된 상태라 무효였다.
- process와 bridge를 재시작한 뒤 map_server/AMCL이 launch 자동 lifecycle 전환에 실패해 `unconfigured` 상태에 머물렀다.
- 수동으로 `/map_server`와 `/amcl`을 configure/activate하고 `use_sim_time:=true`를 맞춘 뒤 `map -> odom` TF가 생성되는 것까지 확인했다.
- 따라서 이번 세션에는 최신 코드로 유효한 주행 안정성 수치, collision count, corner speed를 아직 확정하지 않았다.

## 다음 재개 절차

1. `f1tenth-sim-1`의 현재 launch를 유지하거나 깨끗하게 재시작한다.
2. `/map_server`, `/amcl`에 `use_sim_time:=true`를 launch 단계에서 적용하거나 수동 configure/activate한다.
3. 초기 pose를 `/initialpose`에 publish하고 `map -> odom -> ego_racecar/base_link` TF를 확인한다.
