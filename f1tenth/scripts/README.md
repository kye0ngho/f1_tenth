# 표면 그립 테스트

`max_lateral_acceleration`, `max_longitudinal_acceleration`,
`max_longitudinal_deceleration` 같은 값들을 추정치가 아니라 실측으로
구하는 방법. 이 실차/노면 조합에서만 유효하므로, 타이어를 바꾸거나
노면이 바뀌면 다시 측정해야 함.

## 스크립트

- `surface_grip_test.py`: 일정 조향각으로 원을 그리며 속도를 계단식으로
  올려서, 손으로는 못 내는 그립 한계까지 직접 밀어붙임. `/auto`에 직접
  publish하므로 **run_autonomy.sh와 절대 동시 실행 금지**(같은 토픽에
  컨트롤러가 0을 50Hz로 겹쳐 씀). 진짜 비상정지는 조이스틱
  **데드맨(L1)** — 잡고 스틱 중립이면 `/teleop`이 우선순위로 덮어써서
  정지함. 저희가 만든 `/safety/kill_switch`(L2)는 autonomy 미실행 시
  아무 관여 안 함.
- `analyze_grip_bag.py`: 녹화된 bag에서 코너링 구간의 실측 yaw rate 대
  운동학 모델 yaw rate 비율을 스텝별로 비교, 그립 한계(비율이 기준선
  대비 급락하는 지점)를 찾고 `max_lateral_acceleration` 권장값 계산.
  같은 bag에서 급정지 구간 감속도도 같이 리포트.
- `analyze_accel.py`: 커맨드 속도가 계단식으로 올라가는 구간에서 실제
  속도가 목표치의 90%에 도달하는 시간으로 직선 가속도 측정
  (`analyze_grip_bag.py`의 제동 리포트와 대칭 관계).

## 사용법

```bash
# 터미널 1: bringup만 (run_autonomy.sh 금지)
ros2 launch f1tenth_stack bringup_launch.py

# 터미널 2: 녹화
ros2 bag record -a -o grip_test_$(date +%H%M%S)

# 터미널 3: 차를 스탠드에 올려 데드맨(L1) 동작부터 확인한 뒤 바퀴 내리고
python3 surface_grip_test.py --steering 0.30 --speeds 0.8,1.2,1.6,2.0,2.4
# 필요하면 --brake-from 2.0 추가해서 제동 구간도 같이 측정

# 녹화 종료(Ctrl+C) 후 분석
python3 analyze_grip_bag.py <bag>/<bag>_0.db3
python3 analyze_accel.py <bag>/<bag>_0.db3
```

## 측정 시 주의

- 처음엔 낮은 속도 범위로 시작해서, 그립 한계(ratio가 baseline 대비
  급락하는 지점)가 안 나오면 다음 회차에서 더 높은 스텝을 이어서
  측정할 것.
- `analyze_accel.py`/`analyze_grip_bag.py` 모두 `/auto`, `/drive`,
  `/teleop`을 구분 없이 "commanded speed"로 합쳐서 본다. 테스트 중
  데드맨(L1)을 잡으면 `/teleop`이 섞여 들어가 그 이후 구간 데이터가
  오염된다 — 데드맨은 정말 비상시에만 잡고, 가능하면 각 회차를 짧게
  끊어서(레코딩을 매번 새로 시작) 오염 구간을 격리할 것.
- 결과로 나온 세 값은 `racing_v1_pp`/`racing_v2_pp`/`racing_pp` 등
  `pure_pursuit` 계열 컨트롤러의 `max_lateral_acceleration`,
  `max_longitudinal_acceleration`, `max_longitudinal_deceleration`
  파라미터(노드 자체 기본값 또는 launch 오버라이드)에 반영하는 값입니다.
