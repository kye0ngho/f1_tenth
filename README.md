# f1tenth final_ver

Self-contained CycloneDDS + F1TENTH sim setup. Everything needed to build
and run is vendored in this repo (no other clones or private-repo access
required) — `f1tenth/` is the onboard autonomy stack, `f1tenth_gym_ros/` is
the simulator bridge.

## Setup

```bash
git clone -b final_ver git@github.com:kye0ngho/f1_tenth.git
cd f1_tenth
xhost +si:localuser:root   # allow X11 access; resets on every host login
docker compose up -d --build f1tenth-sim
```

## Run

```bash
docker exec -it f1tenth_sim bash
source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=sim track:=busan controller:=racing_v1_pp speed:=2.0
```

RViz (separate terminal):

```bash
docker exec -it f1tenth_sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /sim_ws/install/setup.bash
  rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz'
```

`ros2-cyclonedds` (plain CycloneDDS/RMW test container, no sim inside) is a
separate service in `docker-compose.yml` for checking discovery/networking
in isolation — start it with `docker compose up -d ros2-cyclonedds`.
