# f1tenth_kkh

`algorithms/control`의 컨트롤러들을 기준선(baseline)으로 두고, 서로 다른 접근법의
MPC 3종을 시도/비교하기 위한 실험 패키지입니다. IFAC 2026 RoboRacer(부산) 대회를
위해 어떤 MPC가 가장 나은지 고르는 것이 목적입니다.

`algorithms/control`은 이 패키지에서 수정하지 않습니다.

> **이식 안내**: 이 패키지는 원래 이 저장소에서 갈라져 나온 다른 fork
> (`Kimz1xq/f1tenth`, `linear_mpc_node.py`/`MPC_GUIDE.md`/AMCL+TF 기반 스택이
> 있던 버전)에서 개발·검증됐고, 이번에 이 저장소(`kye0ngho/f1_tenth`) 위에
> 브랜치로 이식했습니다. `nonlinear_mpc_node.py`는 오히려 **이 저장소 자신의
> `algorithms/control/control/mpc_node.py`를 원본으로 포팅**한 것입니다.
> `RESULT.md`의 실측 랩타임/CTE/solve_time은 모두 그 다른 fork의 스택
> (AMCL이 `map -> ego_racecar/base_link` TF를 직접 채우고, `odom_topic` 기본값이
> `/ego_racecar/odom`)에서 나온 값입니다. 이 저장소는 `control` 패키지 기본
> `odom_topic`이 `/localization/odom`(`amcl_bridge_node` 경유)이고, `cvxpy`/`osqp`가
> Dockerfile에 없습니다 — 아래 "이 저장소에서 쓰기 전에" 참고.

## 컨트롤러

| controller 값 | 노드 | 특징 |
|---|---|---|
| `nonlinear` | `nonlinear_mpc_node` | scipy SLSQP, kinematic bicycle. 이 저장소의 `algorithms/control/control/mpc_node.py`를 TF/`/control/enable` 계약에 맞게 포팅. |
| `mpcc` | `mpcc_node` | Contouring Control (cvxpy/OSQP), progress 상태 확장. 장애물 회피 확장 지점 보유(미구현). |
| `lab8` | `lab8_mpc_node` | F1TENTH 공식 Lab8 스타일의 최소 QP MPC. 튜닝 없는 "스톡" 대조군. |

## 이 저장소에서 쓰기 전에

1. **의존성**: `mpcc_node`/`lab8_mpc_node`는 `cvxpy`+`osqp`가 필요합니다(`nonlinear_mpc_node`는 `scipy`만 필요, 이미 Dockerfile에 있음). `Dockerfile`에
   ```dockerfile
   RUN python3 -m pip install --no-cache-dir cvxpy==1.3.2 osqp==0.6.3
   ```
   추가 후 이미지 재빌드 필요.
2. **odom_topic**: 이 저장소의 `control` 패키지는 `/localization/odom`
   (AMCL → `amcl_bridge_node` 경유)을 기본으로 씁니다. `f1tenth_kkh`의 세 노드는
   기본값이 `/ego_racecar/odom`이지만 파라미터라 `config/*.yaml`에서
   `odom_topic: "/localization/odom"`으로 바꾸면 됩니다 — 다만 세 노드 모두
   실제 위치/헤딩은 odom 메시지가 아니라 **TF `map -> ego_racecar/base_link`**로
   조회하므로, AMCL이 그 TF를 실제로 채우고 있는지(즉 `amcl_bridge_node`가
   토픽만 relay하는 게 아니라 AMCL 자체가 살아서 TF도 브로드캐스트하는지)
   먼저 확인하세요.
3. **build/mount**: `docker-compose.yml`에 `./algorithms/f1tenth_kkh:/sim_ws/src/f1tenth_kkh`
   마운트를 추가해뒀습니다(다른 `algorithms/*`는 이 저장소 compose에 원래
   마운트되어 있지 않으므로, 이미지 재빌드가 기본 워크플로입니다 — 필요하면
   f1tenth_kkh도 재빌드 방식으로 맞춰도 됩니다).
4. 위 사항 확인 전까지는 `RESULT.md`의 수치를 이 저장소 환경의 실측치로
   취급하지 마세요 — 다른 fork에서 측정한 값입니다.

## 실행

```bash
ros2 launch f1tenth_kkh mpc_experiment.launch.py controller:=nonlinear
# 또는 controller:=mpcc / controller:=lab8
```

**`control control.launch.py`(있다면)와 동시에 띄우지 마세요** — 둘 다 `/drive`를
발행하려 하면 충돌합니다.

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

## 비교 테스트

`scripts/closed_loop_test.py`(이 패키지 안에 자체 포함 — `algorithms/control`에
의존하지 않음)는 `/planning/path`, `/control/enable`, TF만으로 동작하는 범용
스크립트입니다:

```bash
python3 /sim_ws/src/f1tenth_kkh/scripts/closed_loop_test.py \
  --duration 60 --max-error 0.75 \
  --output /sim_ws/src/f1tenth_kkh/results/track02_nonlinear_mpc_smoke.csv
```

## 비교 플롯

`scripts/plot_controller_comparison.py`(자체 포함)는 `--trajectory
LABEL:COLOR:CSV_PATH` 반복 인자로 여러 궤적을 오버레이합니다:

```bash
python3 /sim_ws/src/f1tenth_kkh/scripts/plot_controller_comparison.py \
  --map-yaml /root/maps/track02.yaml \
  --raceline /sim_ws/src/planning/waypoints/track02_raceline_safe.csv \
  --trajectory "Nonlinear MPC (SLSQP):#9467bd:/sim_ws/src/f1tenth_kkh/results/track02_nonlinear_mpc_01.csv" \
  --trajectory "MPCC:#d62728:/sim_ws/src/f1tenth_kkh/results/track02_mpcc_01.csv" \
  --trajectory "Lab8 MPC (stock):#17becf:/sim_ws/src/f1tenth_kkh/results/track02_lab8_mpc_01.csv" \
  --output /sim_ws/src/f1tenth_kkh/results/track02_controller_comparison.png
```

## 대회 관련 메모

- Time Trial: 랩타임/CTE로 세 후보를 비교. **다른 fork에서의** 실측 결과는
  `RESULT.md` 참고 — `mpcc` tuned v1(`config/mpcc_params.yaml`, target_speed
  0.85m/s)이 그쪽 baseline 최고 기록 2개를 동시에 갱신(34.75s/mean CTE
  0.068m, 충돌 없음). 이 저장소 환경에서 재검증 필요.
- Head-to-Head: 장애물/상대차량 회피가 필요하지만 이번 라운드에는 구현하지
  않았습니다. `mpcc_node.py`의 `obstacle_a_parameters`/`obstacle_b_parameters`/
  `update_obstacle_constraints()`가 나중에 얹을 확장 지점입니다.
  (`smitdumore/f110-mpc`의 occupancy-grid 샘플링 아이디어를 참고할 수 있음 —
  `RESULT.md`의 외부 저장소 조사 참고.)
- 온보드(Jetson/NUC/RPi급) 실시간성이 실전 투입 기준입니다. `solve_time_ms`는
  다른 fork/개발 데스크톱 실측치이므로 이 저장소·실제 하드웨어에서
  재측정 전까지 `control_rate`/`horizon_steps`를 올리지 마세요.
