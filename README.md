# f1tenth-docker

F1TENTH ROS 2 Humble 개발 환경 이미지 (`misys:f1tenth`)를 다른 장비에서 재현하기 위한 Dockerfile입니다.

원본 이미지의 `docker history` 87개 레이어에서 복원했습니다.

## 요구 사항

| 항목 | 값 |
|---|---|
| 아키텍처 | **arm64 (aarch64) 전용** |
| 하드웨어 | NVIDIA Jetson (JetPack 6 / L4T **r36.3**) |
| 베이스 이미지 | `nvcr.io/nvidia/l4t-jetpack:r36.3.0` |
| 최종 이미지 크기 | 약 **19 GB** |
| 빌드 시간 | 수 시간 |

> x86_64 PC에서는 빌드도 실행도 되지 않습니다. `librealsense`, `range_libc`가 CUDA로 컴파일되고
> 베이스 이미지가 Tegra 전용입니다.

## 포함된 것

- ROS 2 Humble Desktop + `ros-dev-tools`
- CUDA 지원 `librealsense` (`/opt/ros/humble`에 설치) + `realsense-ros`
- `f1tenth_system`, `vesc`, `urg_node2` (Hokuyo LiDAR), `ackermann_mux`, `teleop_tools`
- `particle_filter` — `range_libc`를 CUDA(`rmgpu`)로 빌드
- `slam_toolbox` — 실차용 + 시뮬레이터용(`*_sim`) 설정 분리
- `f1tenth_gym`, `f1tenth_gym_ros` 시뮬레이터
- `pure_pursuit`, `safety_node` (CL2-UWaterloo), `mpc` (derekhanbaliq)
- `Raceline-Optimization` — Python 3.8 venv(`raceline`) + Jupyter 커널 등록
- 워크스페이스는 `~/f1tenth_ws`에 `colcon build`까지 완료된 상태

## 빌드

```bash
git clone https://github.com/Kimz1xq/f1tenth-docker.git
cd f1tenth-docker
./build.sh
```

컨테이너 사용자 이름은 호스트 계정을 따라갑니다. 고정하려면:

```bash
UNAME_ARG=misys UID_ARG=1000 ./build.sh
```

> 이미지 안의 설정 파일 다수가 사용자 이름 `misys` 기준 절대 경로(`/home/misys/...`)를 씁니다.
> 원본과 동일하게 쓰려면 `UNAME_ARG=misys`를 권장합니다.

## 실행

```bash
./run.sh
```

`--runtime nvidia`, X11 포워딩, `/dev` 마운트(조이스틱·VESC·LiDAR), `--network host`로 실행합니다.
`~/shared_dir`이 컨테이너 안에 마운트됩니다.

컨테이너 안에서:

```bash
underlay              # source /opt/ros/humble/setup.bash
cd f1tenth_ws
overlay               # source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

## 이미지를 그대로 옮기는 방법 (재빌드 없이, 권장)

재빌드는 수 시간 걸리고 업스트림 변동에 취약합니다. 빌드 완료된 이미지가 GHCR에 올라가 있습니다.

```
ghcr.io/kimz1xq/f1tenth:latest
ghcr.io/kimz1xq/f1tenth:jetpack-r36.3
digest: sha256:504244173e0628df75dacf3d69bd4b56fe65d57f013156bfb1aa6c44f058b34e
```

```bash
# 받는 쪽 (Jetson)
docker pull ghcr.io/kimz1xq/f1tenth:latest
docker tag ghcr.io/kimz1xq/f1tenth:latest misys:f1tenth
./run.sh
```

패키지가 private이면 먼저 로그인이 필요합니다:

```bash
echo <PAT> | docker login ghcr.io -u <USER> --password-stdin   # read:packages 필요
```

이미지를 다시 올릴 때는 `write:packages` scope가 있는 **classic PAT**이 필요합니다.
fine-grained 토큰(`github_pat_...`)은 GHCR에서 동작하지 않습니다.

**외장 디스크/네트워크로 직접:**

```bash
# 보내는 쪽 (약 6~8GB로 압축됨)
docker save misys:f1tenth | zstd -T0 -3 > f1tenth.tar.zst

# 받는 쪽
zstd -d -c f1tenth.tar.zst | docker load
```

> 이미지 tarball은 GitHub 저장소에 올릴 수 없습니다 (파일당 100MB 제한, Git LFS도 2GB 제한).

## 알려진 제약

- 여러 패키지를 브랜치 고정 없이 `git clone` 하므로, 업스트림이 바뀌면 재빌드 결과가 원본과
  달라질 수 있습니다.
- `sed -i "<행번호>s/..."` 형태의 패치가 많습니다. 업스트림 파일의 행 번호가 바뀌면 **조용히
  엉뚱한 줄을 수정**합니다. 빌드 후 `mpc_node.py`, `bringup_launch.py`, `sim.yaml`을 확인하세요.
- `goodash2one/*` 포크에 의존합니다. 해당 저장소가 사라지면 빌드가 실패합니다.
