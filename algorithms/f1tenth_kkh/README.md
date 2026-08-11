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
- Head-to-Head: 장애물/상대차량 회피가 필요하지만 이번 라운드에는 구현하지
  않았습니다. `mpcc_node.py`의 `obstacle_a_parameters`/`obstacle_b_parameters`/
  `update_obstacle_constraints()`가 나중에 얹을 확장 지점입니다.
  (`smitdumore/f110-mpc`의 occupancy-grid 샘플링 아이디어를 참고할 수 있음 —
  `RESULT.md`의 외부 저장소 조사 참고.)
- 온보드(Jetson/NUC/RPi급) 실시간성이 실전 투입 기준입니다. 각 컨트롤러의
  `solve_time_ms` p95가 10Hz(100ms) 예산 안에 드는지 실측 없이 `control_rate`나
  `horizon_steps`를 올리지 마세요. 개발 데스크톱 실측(dry-run, n=143, 자세한 내용은
  `RESULT.md`): `lab8` mean 1.91ms/p95 2.61ms, `nonlinear` mean 16.64ms/p95
  17.94ms, `mpcc` mean 56.66ms/p95 67.83ms — `mpcc`는 100ms 예산의 1/3 이상을
  써서 세 후보 중 여유가 가장 적음.
