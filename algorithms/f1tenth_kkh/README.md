# f1tenth_kkh

`algorithms/control`의 튜닝된 Linear MPC / Pure Pursuit을 기준선(baseline)으로 두고,
서로 다른 접근법의 MPC 3종을 시도/비교하기 위한 실험 패키지입니다.
IFAC 2026 RoboRacer(부산) 대회를 위해 어떤 MPC가 가장 나은지 고르는 것이 목적입니다.

`algorithms/control`은 이 패키지에서 수정하지 않습니다.

## 컨트롤러

| controller 값 | 노드 | 특징 |
|---|---|---|
| `nonlinear` | `nonlinear_mpc_node` | scipy SLSQP, kinematic bicycle. 형제 저장소 포팅. |
| `mpcc` | `mpcc_node` | Contouring Control (cvxpy/OSQP), progress 상태 확장. 장애물 회피 확장 지점 보유(미구현). |
| `lab8` | `lab8_mpc_node` | F1TENTH 공식 Lab8 스타일의 최소 QP MPC. 튜닝 없는 "스톡" 대조군. |

## 실행

시뮬레이터 + AMCL + planning이 이미 떠 있는 상태에서(README.md 최상위 문서의
"빠른 시작" 참고), 다음 중 하나만 실행합니다. **`control control.launch.py`와
동시에 띄우지 마세요** — 둘 다 `/drive`를 발행하려 하면 충돌합니다.

```bash
ros2 launch f1tenth_kkh mpc_experiment.launch.py controller:=nonlinear
# 또는 controller:=mpcc / controller:=lab8
```

Dry-run 상태(기본 `enabled: false`)에서도 계속 solve하며 예측/레퍼런스 경로와
solve_time을 발행하지만 `/drive`에는 항상 정지 명령을 보냅니다. 확인 후 활성화:

```bash
ros2 topic echo /drive --once   # speed: 0.0 확인
ros2 topic echo /f1tenth_kkh/<controller>/solve_time_ms
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

정지:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

## 비교 테스트 (기존 스크립트 재사용)

`algorithms/control/scripts/closed_loop_test.py`는 어떤 패키지의 컨트롤러든
`/planning/path`, `/control/enable`, TF만으로 동작하는 범용 스크립트입니다.
복사하지 말고 그대로 호출합니다:

```bash
python3 /sim_ws/src/control/scripts/closed_loop_test.py \
  --duration 60 --max-error 0.75 \
  --output /sim_ws/src/f1tenth_kkh/results/track02_nonlinear_mpc_smoke.csv
```

## 비교 플롯

`algorithms/control/scripts/plot_controller_comparison.py`를 5개 궤적(기존 2개 +
신규 3개) 오버레이가 가능하도록 `--trajectory LABEL:COLOR:CSV_PATH` 반복 인자로
일반화했습니다:

```bash
python3 /sim_ws/src/control/scripts/plot_controller_comparison.py \
  --map-yaml /root/maps/track02.yaml \
  --raceline /sim_ws/src/planning/waypoints/track02_raceline_safe.csv \
  --trajectory "Pure Pursuit:#2474d2:/sim_ws/src/control/results/track02_pure_pursuit_safe.csv" \
  --trajectory "Linear MPC (tuned v2):#f28e2b:/sim_ws/src/control/results/track02_mpc_tuned_v2_01.csv" \
  --trajectory "Nonlinear MPC (SLSQP):#9467bd:/sim_ws/src/f1tenth_kkh/results/track02_nonlinear_mpc_01.csv" \
  --trajectory "MPCC:#d62728:/sim_ws/src/f1tenth_kkh/results/track02_mpcc_01.csv" \
  --trajectory "Lab8 MPC (stock):#17becf:/sim_ws/src/f1tenth_kkh/results/track02_lab8_mpc_01.csv" \
  --output /sim_ws/src/f1tenth_gym_ros/results/track02_controller_comparison_v2.png
```

## 대회 관련 메모

- Time Trial: 랩타임/CTE로 위 3종 + 기존 2종을 비교. 실측 결과는
  `RESULT.md` 참고 — **`mpcc` tuned v1(`config/mpcc_params.yaml`, target_speed
  0.85m/s)이 baseline 최고 기록 2개(가장 빠른 랩 Linear MPC tuned v1 42.78s,
  가장 정확한 CTE Linear MPC tuned v2 mean 0.070m)를 동시에 갱신** —
  34.75s/mean CTE 0.068m, 충돌 없음.
- RViz 고속 시각 검증용으로는 `lab8` MPC에 별도 4.0m/s 프로파일을 적용했다
  (`config/lab8_mpc_params.yaml`). 최종 topic 계측은 12초 기준 collision 0,
  `/drive.drive.speed` max 3.548m/s, safety cap max 3.970m/s. 이 설정은
  `MPC + rule-based safety governor` 구조이며, 완전한 obstacle-aware MPC가
  아니다. 상세 방법론/파라미터/재현 명령은 `RESULT.md`의
  `2026-08-14 (3): RViz 고속 Lab8 MPC 튜닝` 항목 참고.
- Head-to-Head: **정적 장애물 회피를 `mpcc`에 구현했습니다** (2026-08-14,
  동적 상대차량 회피는 이번 라운드 범위 밖). LaserScan을 range-jump로
  클러스터링하고(`common/obstacle_detection.py`), 폭이 좁고(벽이 아니고)
  raceline 코너리도어 안에 있는 클러스터만 장애물로 인정합니다
  (`mpcc_node.py`의 `scan_callback`). 진행방향으로 가장 가까운 장애물 하나에
  대해 raceline의 lateral(contouring) 축을 법선으로 하는 분리 초평면을
  세우고(`update_obstacle_constraints()`), horizon 중 그 장애물의 arc-length
  근처 구간에만 적용합니다 — 매 제어 주기(10Hz) 현재 위치 기준으로 다시
  계산되므로 이것이 replanning에 해당합니다. 접근 중에는
  `corner_slowdown_gain`과 같은 방식으로 속도를 미리 줄이는
  `obstacle_slowdown_gain`도 추가했는데, 이게 없으면 이 트랙의 대회 속도
  (target_speed 0.85m/s)에서 MPC horizon이 확보하는 반응 거리(~0.85-1.4m)가
  keepout 회피 기동을 완주하기엔 부족해서 QP가 바로 infeasible이 됩니다
  (실측: `RESULT.md`). 이 infeasible 상황은 새로 안전 로직을 추가하지 않고
  기존 solver-실패 → 안전 정지 경로를 그대로 재사용합니다. **범위 밖**:
  동적 상대차량(속도/진행방향 추정 필요), 장애물 2개 이상 동시 처리, 매우
  가까운 거리(실측상 ~1.5m 미만, 정지 상태 기준)에서 갑자기 나타난 장애물—
  이 경우 회피 대신 안전 정지가 발생합니다.
  **주의(2026-08-14 (2) 실측)**: 이 트랙에서 `target_speed`를 0.85m/s보다
  올리면(예: 1.10m/s) 재현성이 없습니다 — 같은 시작 pose에서 반복 시행 시
  완주와 시작 직후 급커브에서의 충돌이 뒤섞여 나왔습니다(`RESULT.md`의
  5회 시행 표 참고). 대회에는 반드시 `mpcc_params.yaml`의 채택된
  0.85m/s를 그대로 사용하세요. 또한 이 검증 과정에서 장거리(수 미터)
  grazing-angle 벽 반사가 허위 장애물로 오검출되어 `mpcc_node` 프로세스
  전체가 죽는 버그를 발견해 수정했습니다(`obstacle_max_range_m` 추가 +
  `control_loop`의 예외 처리를 `except Exception`으로 확장 — 자세한 내용은
  `RESULT.md`).
- 온보드(Jetson/NUC/RPi급) 실시간성이 실전 투입 기준입니다. 각 컨트롤러의
  `solve_time_ms` p95가 10Hz(100ms) 예산 안에 드는지 실측 없이 `control_rate`나
  `horizon_steps`를 올리지 마세요. 개발 데스크톱 실측(dry-run, n=143, 자세한 내용은
  `RESULT.md`): `lab8` mean 1.91ms/p95 2.61ms, `nonlinear` mean 16.64ms/p95
  17.94ms, `mpcc` mean 56.66ms/p95 67.83ms — `mpcc`는 100ms 예산의 1/3 이상을
  써서 세 후보 중 여유가 가장 적음.
