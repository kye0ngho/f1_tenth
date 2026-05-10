# F1Tenth ROS2 Humble

f1tenth_gym_ros를 ROS2 Humble로 포팅한 버전

## 설치

### 사전 요구사항
- Ubuntu 22.04
- Docker
- nvidia-docker2
- rocker (`pip install rocker`)

## 실행

```bash
git clone https://github.com/SeEEEun/f1tenth_gym_ros_humble
cd f1tenth_gym_ros_humble
docker build -t f1tenth_gym_ros_humble -f Dockerfile .
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros_humble
```

## 컨테이너 안에서

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## 텔레옵 (새 터미널)

```bash
docker exec -it $(docker ps -q) /bin/bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
