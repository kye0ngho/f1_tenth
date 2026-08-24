#!/usr/bin/env bash
# Start the F1TENTH container with GPU, X11 (RViz/Gazebo) and USB device access.
set -euo pipefail

IMAGE="${IMAGE:-misys:f1tenth-cyclone}"
NAME="${NAME:-f1tenth}"
REPO="${REPO:-$HOME/f1tenth}"

# The repo is mounted at the SAME absolute path inside the container.  The host
# user is jeonbotdae and the container user is misys, so any other choice would
# leave colcon's absolute paths valid on only one side of the mount.
MOUNT="$REPO"

# Build artefacts live in a named volume, never in the repo: they are ~500MB,
# they hard-code container paths, and keeping them out of the bind mount means
# recreating the container does not force a full rebuild.
BUILD_VOLUME="${BUILD_VOLUME:-f1tenth-build}"

[ -d "$REPO" ] || { echo "ERROR: repo not found at $REPO" >&2; exit 1; }
xhost +local:docker >/dev/null 2>&1 || true

if [ "$(docker ps -aq -f name="^${NAME}$")" ]; then
  echo "Attaching to existing container '$NAME'..."
  docker start "$NAME" >/dev/null
  exec docker exec -it "$NAME" bash
fi

# A fresh named volume is created root-owned, which the container user cannot
# write to.  Claim it once, on creation.
claim_build_volume() {
  docker exec "$NAME" sudo chown "$(docker exec "$NAME" id -un)" /opt/f1tenth-build
}

echo "Creating container '$NAME' from $IMAGE"
echo "  repo   $REPO -> $MOUNT"
echo "  build  volume '$BUILD_VOLUME' -> /opt/f1tenth-build"

exec docker run -it \
  --name "$NAME" \
  --runtime nvidia \
  --network host \
  --privileged \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e SDL_JOYSTICK_DEVICE=/dev/input/js0 \
  -e F1TENTH_REPO="$MOUNT" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "$REPO:$MOUNT" \
  -v "$BUILD_VOLUME:/opt/f1tenth-build" \
  "$IMAGE" \
  bash
