
# F1Tenth ROS2 Humble

f1tenth_gym_ros를 ROS2 Humble로 포팅한 버전

## 업데이트 로그
### 코너링시 속도저하 , 직선구간에선 속도 향상 버전 
[스크린캐스트 2026-07-07 14-19-45.webm](https://github.com/user-attachments/assets/45663c64-6c07-4f88-9496-fb6078f1ffd5)
### 기본 버전
[스크린캐스트 2026-07-07 14-02-26.webm](https://github.com/user-attachments/assets/b9fb5079-82ec-4758-93c6-cbd7d4e9f304)

### 2026-07-06
Slam Toolbox이용 매핑 
<img width="911" height="301" alt="image" src="https://github.com/user-attachments/assets/5f0bd115-62af-46ae-b640-4de3de4c0bea" />
왼쪽 GT , 오른쪽 매핑

### 2026-07-05
- `centerline_extractor_node` 구현 — 점유격자 지도에서 트랙 중심선 waypoint 자동 추출 (지도만 있으면 주행 없이 waypoints.csv 생성)
- levine 트랙 실제 waypoint 279개 생성 (64.4m 루프) — 기존 데모용 사각형 대체
- pure_pursuit 주행 검증: 추출된 중심선을 따라 벽 거리 0.7~1.0m 유지하며 정상 주행 확인
- gap_follow 조향 버그 수정 (최장거리 동률 시 벽 방향으로 쏠리던 문제)

### 2026-07-04
- 로컬라이제이션 백엔드 확장: `localization_mode:=passthrough|particle_filter|slam|amcl` 4가지 선택 지원
- `amcl_bridge_node` 추가 — nav2_amcl 출력을 `/localization/odom`으로 통일
- 웨이포인트 속도(CSV) 파이프라인을 pure_pursuit까지 연결 (전에는 계산만 되고 실제 주행엔 반영 안 되던 문제 수정)
- 시뮬 SLAM 매핑 → 지도 저장까지 컨테이너에서 검증 완료

## 사전 요구사항 설치

### 1. Docker
```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
newgrp docker
```

### 2. nvidia-docker2 (NVIDIA GPU 있는 경우)
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 3. rocker
```bash
pip install rocker
```

## 시뮬레이터 설치 및 실행

```bash
# 1. 클론
git clone https://github.com/SeEEEun/f1tenth_gym_ros_humble
cd f1tenth_gym_ros_humble

# 2. 이미지 빌드 (최초 1회, 약 10분 소요)
docker build -t f1tenth_gym_ros_humble -f Dockerfile .

# 3. 실행 (NVIDIA GPU)
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros_humble

# 3. 실행 (GPU 없는 경우)
docker-compose up
```

## 컨테이너 안에서 시뮬 실행

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## 텔레옵 (새 터미널에서)

```bash
docker exec -it $(docker ps -q) /bin/bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

키 조작: i=전진, k=정지, u/o=전진+회전, m/.=후진+회전

## SLAM으로 맵 저장

터미널 1 - 시뮬 실행 후, 터미널 2:
```bash
docker exec -it $(docker ps -q) /bin/bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

터미널 3 - 텔레옵으로 맵 그리기 후 저장:
```bash
docker exec -it $(docker ps -q) /bin/bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

# F1TENTH ROS2 Humble Autonomy Stack

This repository provides a ROS2 Humble based F1TENTH simulation environment with additional autonomy packages for study, simulation, and future real-car deployment.

## Included Autonomy Packages

The additional autonomy packages are located in:

~~~bash
algorithms/
├── localization
├── planning
├── control
├── safety
└── f1tenth_bringup
~~~

Current node structure:

~~~text
/ego_racecar/odom
        ↓
localization_node
        ↓
/localization/odom
        ↓
pure_pursuit_node
        ↑
/planning/path          /control/drive (raw)
        ↑                      ↓
waypoint_planner_node  safety_brake_node ← /scan
        ↑                      ↓
waypoints.csv               /drive
~~~

Output mode:

~~~text
drive_mode:=sim
    → /drive
    → ackermann_msgs/msg/AckermannDriveStamped

drive_mode:=real
    → /commands/motor/speed
    → /commands/servo/position
    → std_msgs/msg/Float64
~~~

---

## Quick Start

Clone this repository:

~~~bash
cd ~

git clone https://github.com/SeEEEun/f1tenth_gym_ros_humble
cd f1tenth_gym_ros_humble
~~~

Build Docker image:

~~~bash
docker build -t f1tenth_gym_ros_humble -f Dockerfile .
~~~

Run container without NVIDIA GPU:

~~~bash
rocker --x11 \
  --volume .:/sim_ws/src/f1tenth_gym_ros \
  --volume ./algorithms/localization:/sim_ws/src/localization \
  --volume ./algorithms/planning:/sim_ws/src/planning \
  --volume ./algorithms/control:/sim_ws/src/control \
  --volume ./algorithms/safety:/sim_ws/src/safety \
  --volume ./algorithms/f1tenth_bringup:/sim_ws/src/f1tenth_bringup \
  -- f1tenth_gym_ros_humble
~~~

Run container with NVIDIA GPU:

~~~bash
rocker --nvidia --x11 \
  --volume .:/sim_ws/src/f1tenth_gym_ros \
  --volume ./algorithms/localization:/sim_ws/src/localization \
  --volume ./algorithms/planning:/sim_ws/src/planning \
  --volume ./algorithms/control:/sim_ws/src/control \
  --volume ./algorithms/safety:/sim_ws/src/safety \
  --volume ./algorithms/f1tenth_bringup:/sim_ws/src/f1tenth_bringup \
  -- f1tenth_gym_ros_humble
~~~

Inside the container, build the autonomy packages:

~~~bash
cd /sim_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select localization planning control f1tenth_bringup

source install/local_setup.bash
~~~

Launch simulator:

~~~bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~

In another terminal, enter the same container:

~~~bash
docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}\t{{.Status}}"

docker exec -it <CONTAINER_ID> /bin/bash -lc "cd /sim_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && bash"
~~~

Launch autonomy stack:

~~~bash
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim
~~~

Check output:

~~~bash
ros2 node list
ros2 topic info /drive
ros2 topic echo /drive --once
~~~

---

## Detailed Usage Guide

For full installation steps, Docker explanation, current node descriptions, simulation usage, real-car deployment notes, and recommended future nodes, see:

[usage.md](./usage.md)

---

## Recommended Next Nodes

The current stack is a basic structure for simulation and future real-car deployment. Recommended next nodes are:

~~~text
1. safety_brake_node
2. goal_pose_planner_node
3. vehicle_interface_node
4. debug_marker_node
5. waypoint_manager_node
6. velocity_profile_node
7. data_logger_node
8. lap_timer_node
9. scan_preprocessor_node
10. particle_filter_node
11. gap_follow_node
12. behavior_selector_node
13. calibration tool
14. watchdog_node
15. mpc_node
~~~

The first recommended package to implement next is:

~~~text
safety
├── safety_brake_node.py
├── config/params.yaml
└── launch/safety.launch.py
~~~

Reason:

~~~text
- Current Pure Pursuit does not use LiDAR.
- The car may hit walls if the waypoint is bad.
- Safety brake is essential for real-car testing.
- It is useful for studying LaserScan and emergency stop logic.
~~~

