# platform

Third-party packages the car needs at runtime, vendored as source.

Each one is here for a reason: either we modified it, or no ROS binary package
exists for it. Anything that was neither got dropped — `slam_toolbox` came from
a 53MB checkout that shadowed the already-installed `ros-humble-slam-toolbox`,
and `realsense-ros`, `bringup`, `mpc` and `particle_filter` were unreachable
from any launch file.

The upstream `.git` directories (102MB) are not kept. The coordinates below are
the record of where each package came from and what we changed; everything
after the import is in this repository's own history.

## Modified — must stay as source

| package | upstream | commit | our change |
|---|---|---|---|
| `vesc` | github.com/f1tenth/vesc `ros2` | `153998df` | odometry: servo command delay compensation |
| `f1tenth_system` | github.com/goodash2one/f1tenth_system `humble-devel` | `569444ce` | VESC steering calibration and speed envelope; urg_node2 replaces urg_node in bringup |
| `teleop_tools` | github.com/ros-teleop/teleop_tools `master` | `ac63b924` | joy QoS RELIABLE to BEST_EFFORT |
| `urg_node2` | github.com/Hokuyo-aut/urg_node2 `main` | `f02d8af2` | urg_library submodule pinned forward |
| `f1tenth_gym_ros` | github.com/goodash2one/f1tenth_gym_ros `main` | `bb8c93bf` | drive topics made relative |

`urg_node2` carries the `urg_library` submodule from
github.com/UrgNetwork/urg_library. It is a plain directory here, not a
submodule.

## Unmodified — no binary package exists

| package | upstream | commit |
|---|---|---|
| `ackermann_mux` | github.com/f1tenth/ackermann_mux `foxy-devel` | `b3c0b083` |
| `safety_node` | github.com/goodash2one/CL2-UWaterloo, sparse checkout | — |

`safety_node` was checked out under a `CL2-UWaterloo/` directory alongside a
`pure_pursuit` package that nothing referenced and whose config still pointed at
`/home/user/`. That package is gone and `safety_node` sits at the top level.

## Unmodified, and a binary does exist

`transport_drivers` (`io_context`, `serial_driver`, `asio_cmake_module`) and
`udp_msgs` are vendored only because they were already here. Both are on the
ROS index as `ros-humble-io-context`, `ros-humble-serial-driver` and
`ros-humble-udp-msgs`; switching costs an image rebuild and saves ~470KB, so it
has not been done. `udp_driver`, which nothing depends on, was dropped.

## Building

`platform` is not built from this tree yet. The overlay still comes from
`/home/misys/f1tenth_ws/install` inside the image, so edits here do not take
effect until the workspace is built from the mount.
