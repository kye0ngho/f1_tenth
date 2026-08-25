# F1TENTH 실차 자율주행 스택 — race_v1

ROS 2 Humble 기반 실차 배포 브랜치입니다. 시뮬레이션 공통 저장소에서
실차에 필요한 부분만 떼어낸 스냅샷이며, 대회장(부산) 맵과 그립테스트
실측값이 기본값으로 들어가 있습니다.

## 저장소 구조

```text
autonomy_ws/src/planning        raceline 추종, LiDAR 정적 장애물 회피, AEB
autonomy_ws/src/control         Pure Pursuit 계열, UNICORN L1, ForzaETH MAP, MPC/MPCC, 킬스위치
autonomy_ws/src/f1tenth_bringup 공통 launch, track/vehicle/AMCL 설정
config/amcl.yaml                실차 AMCL 파라미터
maps/                           실차 map (busan, track05)
vehicle_overrides/              f1tenth_stack·VESC·joy_teleop·slam_toolbox 수정본
scripts/                        그립/가속 실측 및 bag 분석 도구
run_autonomy.sh                 프로세스 그룹까지 정리하는 launch 래퍼
```

제어 출력은 `AckermannDriveStamped` 공통 규약입니다. 시뮬레이션과의
차이는 입출력 어댑터에만 남아 있고, `mode:=real`이 자동으로 선택합니다.

| 항목 | 시뮬레이션 | 실차 (이 저장소) |
|---|---|---|
| 차량/센서 | Gym bridge | VESC + URG LiDAR |
| base frame | `ego_racecar/base_link` | `base_link` |
| odometry | `/ego_racecar/odom` | `/odom` |
| 제어 출력 | `/drive` | `/auto` → mux → VESC |
| 초기 자세 | raceline에서 자동 설정 | RViz `2D Pose Estimate` |
| localization | launch 내부 | 터미널 2/3에서 별도 기동 |

차량 공통 모델은 `autonomy_ws/src/f1tenth_bringup/config/vehicle_model.yaml`,
AMCL 공통 모델은 같은 폴더의 `amcl_common.yaml`에서 관리합니다.

## 접속

```bash
ssh jeonbotdae@192.168.1.7
docker exec -it f1tenth bash
```

IP는 네트워크에 따라 바뀝니다. 컨테이너가 꺼져 있으면 먼저
`docker start f1tenth`.

컨테이너 안 공통 환경(**모든 터미널 필수** — `ROS_DOMAIN_ID`를 빠뜨리면
노드끼리 서로 보이지 않습니다):

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
```

## 빌드 (한 번만)

```bash
cd /home/misys/shared_dir/autonomy_ws
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup
source install/setup.bash
```

## 세션 시작 전 초기화 (매번)

이전 세션이 비정상 종료됐으면 프로세스가 남아 `/map_server`, `/amcl`이
중복으로 뜨고 충돌합니다. 실행 전 한 번:

```bash
docker restart f1tenth
docker exec -it f1tenth bash
rm -f /dev/shm/fastrtps_*
ros2 daemon stop && ros2 daemon start
```

## 실차 — 터미널 6개

### 터미널 1 — 하드웨어 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

`Opened joystick`, `Connected to VESC`, LiDAR `Connected to a network device`
확인. **이 터미널은 절대 두 번 실행하지 마세요** — 같은 시리얼 포트를 두
프로세스가 잡으면 VESC 통신이 깨지고 드라이버가 죽습니다
(`Out-of-sync with VESC`, segfault). 헷갈리면
`ps aux | grep bringup_launch | grep -v grep`으로 확인.

### 터미널 2 — Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/busan.yaml \
  -p topic:=map -p frame_id:=map -p use_sim_time:=false
```

### 터미널 3 — AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4 — Lifecycle Activation

터미널 1~3이 다 뜬 뒤:

```bash
ros2 daemon stop && ros2 daemon start
sleep 2

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server   # active [3] 확인

ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 lifecycle get /amcl         # active [3] 확인
```

### 터미널 5 — RViz (노트북 호스트, SSH 아님)

```bash
xhost +si:localuser:root
docker exec -it -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=30 -e ROS_LOCALHOST_ONLY=0 \
  f1tenth_gym_ros_humble-sim-1 \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /sim_ws/install/setup.bash
    exec rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
  '
```

**2D Pose Estimate**로 차량 위치/방향을 지정합니다(터미널 3의 AMCL이
살아있어야 반영됨). LaserScan(빨간 점)이 지도 벽에 맞는지 확인.

### 터미널 6 — Autonomy

```bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
cd /home/misys/shared_dir
./run_autonomy.sh \
  mode:=real track:=busan controller:=racing_v3_pp \
  speed:=1.0 maximum_speed:=5.5 localization:=false
```

`localization:=false` 필수 — 안 붙이면 터미널 2/3의 map server/AMCL과
내부 localization이 중복으로 떠서 서로 충돌합니다.

로그에 `num waypoints: 470` 확인(busan raceline, 트랙 길이 44.0 m).

가속/감속은 `max_longitudinal_acceleration:=` /
`max_longitudinal_deceleration:=`로 덮어쓰지 마세요 — `racing_v3_pp`의
기본값이 2026-08-24 부산 대회장 그립테스트 실측값이고, 8.0 같은 미실측
값을 주면 곡률 기반 감속 프리뷰가 차가 낼 수 없는 제동력을 믿고 늦게
브레이크를 밟습니다.

실차 속도는 1 m/s부터 단계적으로 올립니다. `maximum_speed`는 소프트웨어
명령 허용 상한일 뿐이며 VESC·배터리·모터·기어·타이어의 안전 한계를
해제하지 않습니다.

## 시작·정지

모든 제어기는 비활성 상태로 시작합니다.

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

물리 킬스위치는 조이스틱 **L2 (buttons[6])**. enable 하기 전에 항상 확인:

```bash
ros2 topic echo /safety/kill_switch
```

`false`로 뜨고 L2를 누르면 `true`로 바뀌어야 합니다. 아무것도 안 뜨면
`kill_switch_node`가 launch에 안 붙은 것이므로
`control/launch/control.launch.py`의 해당 컨트롤러 분기에
`kill_switch_node` Node()가 있는지 확인하세요.

## 킬스위치 단독 시연 (매핑 안 된 환경)

지도·위치추정·경로가 전혀 필요 없습니다. 고정 속도로 직진하다가
킬스위치에 반응하는 `kill_switch_demo_node` 하나만 사용합니다. Bringup의
`joy_teleop`은 조이스틱을 실제로 조작할 때만 `/teleop`에 publish하므로
시연 중 L2 외의 스틱/버튼은 건드리지 마세요 — 건드리는 순간 mux
우선순위상 teleop이 앞서서 데모 노드의 `/auto` 명령을 덮습니다.

**터미널 1 — Bringup**

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**터미널 2 — 킬스위치**

```bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
ros2 run control kill_switch_node --ros-args -p kill_switch_button:=6
```

**터미널 3 — 확인 후 데모 실행**

먼저 킬스위치 확인(`false` → L2 → `true`):

```bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
ros2 topic echo /safety/kill_switch
```

확인되면 데모 실행. `start_delay`초 후 자동 직진을 시작하고,
`max_duration`초 후 무조건 정지합니다(킬스위치와 별개인 안전장치):

```bash
ros2 run control kill_switch_demo_node --ros-args \
  -p speed:=1.0 \
  -p start_delay:=3.0 \
  -p max_duration:=10.0
```

차가 움직이면 원하는 타이밍에 L2로 정지, 다시 눌러 재개까지 보여주면
토글임이 증명됩니다. 앞에 최소 3~4 m 공간을 확보하세요.

## 제어기 선택

launch의 `controller:=` 값만 변경합니다.

| 이름 | 방식 | 비고 |
|---|---|---|
| `pure_pursuit` (별칭 `racing_pp`) | 속도 비례 lookahead + 곡률 기반 감속 | 기본, 가장 검증됨 |
| `racing_v1_pp` | racing_pp 2026-08-23 고정 스냅샷 | 이후 수정 안 함, 항상 되돌아갈 기준점 |
| `racing_v2_pp` | racing_v1_pp에서 튜닝 | 연습 트랙 실측 (횡 4.43 / 가속 1.8 / 감속 3.04) |
| `racing_v3_pp` | racing_v2_pp 클론 | **부산 대회장 실측 (횡 3.27 / 가속 0.95 / 감속 5.20) — 대회장에서는 이것** |
| `unicorn_l1` | HMCL-UNIST adaptive L1/PP | 곡률 기반 lookahead 상한 패치 적용 |
| `woong_pp` | unicorn_l1 + 장애물회피 안정화 포크 | 시뮬 1.5 m/s에서만 검증됨 — 저속부터 |
| `forza_map` | ForzaETH MAP pursuit | 7 m/s LUT 범위 내 |
| `mpc` / `mpcc` | 선형 / nonlinear MPC | 실험적, 검증 부족 |

## 현재 기준선 — busan

`maps/busan.yaml` + `busan_raceline.csv` 기준입니다.

| 항목 | 값 |
|---|---:|
| raceline 점 개수 | 470 |
| 트랙 길이 | 44.03 m |
| closure gap | 0.095 m |
| 중심선 최소 벽 여유 | 0.680 m |
| 차체 footprint 최소 여유 | 0.354 m |

시뮬레이션(`mode:=sim track:=busan controller:=racing_v3_pp speed:=2.0`,
장애물 없음) 70초 주행에서 2.72랩, 충돌·경고·에러 0으로 확인했습니다.
이는 경로/제어 체인이 정상 동작한다는 확인이며 실차 고속 허가 기준이
아닙니다. 실차는 1 m/s부터 단계적으로 올립니다.

검증 재실행:

```bash
python3 autonomy_ws/src/planning/scripts/validate_raceline.py \
  --map-yaml maps/busan.yaml \
  --raceline autonomy_ws/src/planning/waypoints/busan_raceline.csv \
  --preview /tmp/busan_preview.png
```

## 새 맵 추가

맵마다 launch 파일을 만들지 않습니다. 다음 파일을 추가하고
`autonomy_ws/src/f1tenth_bringup/config/tracks.yaml`에 한 번 등록합니다.

```text
maps/<track>.pgm
maps/<track>.yaml
autonomy_ws/src/planning/waypoints/<track>_raceline.csv
```

1. slam_toolbox로 매핑 → `maps/<track>.pgm`, `.yaml`
2. 센터라인 생성:
   `python3 autonomy_ws/src/planning/scripts/generate_centerline.py --map-yaml ... --output <track>_centerline.csv`
3. Raceline-Optimization(min-curvature)으로 raceline 생성 후
   `validate_raceline.py`로 벽 클리어런스·곡률·조향각 한계 검증
4. `tracks.yaml`에 항목 추가

## 알려진 이슈

- **VESC 속도 상한** — `vesc.yaml`의 `speed_max`가 예전엔 23250(≈5.57 m/s)로
  캡되어 있었음. 20 m/s(83468)로 상향 완료. `speed:=`가 반영 안 되는 것
  같으면 이 값부터 확인.
- **install/ 심볼릭 링크** — `colcon build --symlink-install` 이후 새로
  추가된 파일(raceline csv 등)은 install/에 자동 링크되지 않음. 수동으로
  `cp src/... install/.../share/...`까지 해야 반영됨.
- **DDS 공유메모리 잔여물** — 세션을 여러 번 강제 종료하면
  `/dev/shm/fastrtps_*`가 쌓여 `RTPS_TRANSPORT_SHM Error`나 AMCL 타임스탬프
  오류가 발생. `docker restart f1tenth` + `rm -f /dev/shm/fastrtps_*`로 정리.
- **params.yaml 런타임 반영** — `max_heading_error`, `max_path_distance` 등
  파일 기반 파라미터가 런타임에 반영 안 되는 문제가 있음(원인 불명).
  급한 경우 `control.launch.py`의 인라인 파라미터 dict로 직접 오버라이드.

## 출처

- [F1TENTH Pure Pursuit](https://github.com/f1tenth-dev/pure_pursuit)
- [Nav2 Regulated Pure Pursuit](https://arxiv.org/abs/2305.20026)
- [HMCL-UNIST UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [TUM/CL2-UWaterloo Global Racetrajectory Optimization](https://github.com/CL2-UWaterloo/f1tenth_ws)

이 저장소는 위 저장소 전체를 복사하지 않고 ROS 2 Humble 공통 입출력
규약에 맞춘 어댑터와 필요한 제어 전략만 유지합니다.
