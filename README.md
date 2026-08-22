# F1TENTH 온보드 — race_v1

ROS 2 Humble 기반 실차 배포 브랜치입니다. `track04`(신규 매핑 트랙)를
기본으로 하며, 6개 터미널로 하드웨어/localization/시각화/자율주행을
나눠서 띄우는 방식을 기준으로 합니다.

## 한 번만 빌드

온보드 컨테이너에서:

```bash
cd /home/misys/shared_dir/autonomy_ws
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup
source install/setup.bash
```

## 실차 실행 — 터미널 6개

각 터미널마다 로봇에 접속:

```bash
ssh jeonbotdae@192.168.1.7
docker exec -it f1tenth bash
```

(컨테이너가 꺼져 있으면 먼저 `docker start f1tenth`)

컨테이너 안 공통 환경 (**모든 터미널에서 필수** — 특히 `ROS_DOMAIN_ID`
빠뜨리면 노드끼리 서로 안 보입니다):

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
```

### 터미널 1 — 하드웨어 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

`Opened joystick`, `Connected to VESC`, LiDAR `Connected to a network device`
확인. **이 터미널은 절대 두 번 실행하지 마세요** — 같은 시리얼 포트를
두 프로세스가 잡으면 VESC 통신이 깨지고 드라이버가 죽습니다
(`Out-of-sync with VESC`, segfault). 이미 떠 있는지 헷갈리면:
```bash
ps aux | grep bringup_launch | grep -v grep
```

### 터미널 2 — Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/track04.yaml \
  -p topic:=map -p frame_id:=map -p use_sim_time:=false
```

### 터미널 3 — AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4 — Lifecycle Activation

Terminal 1~3이 다 뜬 뒤:

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
docker start f1tenth_gui2_host >/dev/null 2>&1 || true
docker exec -it -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=30 -e ROS_LOCALHOST_ONLY=0 \
  f1tenth_gui2_host \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /sim_ws/install/setup.bash
    exec rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
  '
```

`f1tenth_gui2_host`는 노트북에 이미 떠 있는 RViz 전용 컨테이너입니다
(`docker ps -a`로 확인). RViz는 `ROS_DOMAIN_ID=30`으로 로봇 네트워크에
붙어 토픽만 구독하므로, 컨테이너 안에 마운트된 `control`/`planning`
소스가 이 리포 것인지는 무관합니다 — 순수 시각화 용도입니다.

**2D Pose Estimate**로 실제 차량 위치/방향을 지정합니다 (Terminal 3의
AMCL이 살아있어야 반영됨). LaserScan(빨간 점)이 지도 벽에 맞는지 확인.

### 터미널 6 — Autonomy

```bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
cd /home/misys/shared_dir
./run_autonomy.sh mode:=real track:=track04 controller:=pure_pursuit \
  speed:=10.0 maximum_speed:=20.0 localization:=false \
  max_longitudinal_acceleration:=10.0 \
  max_longitudinal_deceleration:=8.0
```

`localization:=false` 필수 — 안 붙이면 Terminal 2/3과 내부 localization이
중복으로 떠서 `/map_server`, `/amcl`이 2개씩 생기고 서로 충돌합니다.

로그에 `num waypoints: 452` 확인 (최적화된 track04 raceline).

## 시작/정지 · 킬스위치

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

물리 킬스위치는 조이스틱 **L2 (buttons[6])** — enable 하기 전에 항상 확인:
```bash
ros2 topic echo /safety/kill_switch
```
`false`로 뜨고 L2 누르면 `true`로 바뀌는지 확인. 안 뜨면 `kill_switch_node`가
launch에 안 붙은 것이니 `control/launch/control.launch.py`의 해당 컨트롤러
분기에 `kill_switch_node` Node()가 있는지 확인하세요.

## 컨트롤러 선택

`controller:=`만 바꿉니다.

| 이름 | 방식 | 비고 |
|---|---|---|
| `pure_pursuit` | 속도 비례 lookahead + 곡률 기반 감속 | **기본, 가장 검증됨** |
| `unicorn_l1` | HMCL-UNIST adaptive L1/PP | 곡률 기반 lookahead 상한 추가 패치 적용됨 |
| `forza_map` | ForzaETH MAP pursuit | 7 m/s LUT 범위 내 |
| `mpc` / `mpcc` | 선형/nonlinear MPC | 실험적, 검증 부족 |

## 트랙 추가 (신규 매핑)

1. slam_toolbox로 매핑 → `maps/<name>.pgm`, `.yaml`
2. 센터라인 생성: `python3 scripts/generate_centerline.py --map-yaml ... --output waypoints/<name>_centerline.csv`
3. `scripts/generate_racetrack_bounds.py` + Raceline-Optimization(min-curvature) +
   `scripts/global_smooth_raceline.py`로 최적화된 raceline 생성 (벽 클리어런스 +
   곡률/조향각 한계 자동 검증)
4. `f1tenth_bringup/config/tracks.yaml`에 항목 추가

## 알려진 이슈 / 주의사항

- **VESC 속도 상한**: `vesc.yaml`의 `speed_max`가 예전엔 23250(≈5.57 m/s)로
  캡되어 있었음 → 20 m/s(83468)로 상향 완료. `speed:=` 값이 실제로 반영 안
  되는 것 같으면 이 값부터 확인.
- **install/ 심볼릭 링크**: `colcon build --symlink-install` 이후 새로 추가된
  파일(raceline csv 등)은 자동으로 install/에 링크되지 않음 — 수동으로
  `cp src/... install/.../share/...`까지 해야 반영됨.
- **DDS 공유메모리 잔여물**: 세션을 여러 번 강제 종료하면 `/dev/shm/fastrtps_*`
  파일이 쌓여서 `RTPS_TRANSPORT_SHM Error`나 AMCL 타임스탬프 오류가 날 수
  있음 → `docker restart f1tenth` + `rm -f /dev/shm/fastrtps_*`로 정리.
- **`max_heading_error`/`max_path_distance` 등 params.yaml 값**: 파일 기반
  파라미터가 런타임에 반영 안 되는 문제가 있어(원인 불명), 급한 경우
  `control.launch.py`의 인라인 파라미터 dict로 직접 오버라이드하는 편이
  확실함.

## 출처

- [F1TENTH Pure Pursuit](https://github.com/f1tenth-dev/pure_pursuit)
- [Nav2 Regulated Pure Pursuit](https://arxiv.org/abs/2305.20026)
- [HMCL-UNIST UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [TUM/CL2-UWaterloo Global Racetrajectory Optimization](https://github.com/CL2-UWaterloo/f1tenth_ws) (raceline 최적화)
