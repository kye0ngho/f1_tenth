#!/usr/bin/env bash
# Start the F1TENTH container with GPU, X11 (RViz/Gazebo) and USB device access.
set -euo pipefail

IMAGE="${IMAGE:-misys:f1tenth}"
NAME="${NAME:-f1tenth}"
SHARED_DIR="${SHARED_DIR:-$HOME/shared_dir}"

mkdir -p "$SHARED_DIR"
xhost +local:docker >/dev/null 2>&1 || true

if [ "$(docker ps -aq -f name="^${NAME}$")" ]; then
  echo "Attaching to existing container '$NAME'..."
  docker start "$NAME" >/dev/null
  exec docker exec -it "$NAME" bash
fi

exec docker run -it \
  --name "$NAME" \
  --runtime nvidia \
  --network host \
  --privileged \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e SDL_JOYSTICK_DEVICE=/dev/input/js0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "$SHARED_DIR:/home/$(id -un)/shared_dir" \
  "$IMAGE" \
  bash
