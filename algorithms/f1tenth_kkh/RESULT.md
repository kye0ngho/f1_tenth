# f1tenth_kkh 실험 결과

이 파일은 계속 업데이트됩니다. 최신 항목이 위로 오도록 추가하세요.

> **주의**: 아래 모든 실측치(랩타임/CTE/solve_time)는 `Kimz1xq/f1tenth` fork
> (AMCL+TF 직접 조회, `odom_topic=/ego_racecar/odom`, `algorithms/control`에
> `linear_mpc_node.py`/`MPC_GUIDE.md`/`scripts/closed_loop_test.py` 존재)에서
> 측정한 것입니다. 이 저장소(`kye0ngho/f1_tenth`)로 브랜치 이식하며 코드는
> 그대로 가져왔지만, 이 저장소 환경(`odom_topic=/localization/odom` 등)에서
> 아직 재검증하지 않았습니다. `algorithms/f1tenth_kkh/README.md`의 "이
> 저장소에서 쓰기 전에" 참고.

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
