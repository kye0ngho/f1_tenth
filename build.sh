#!/usr/bin/env bash
# Rebuild the F1TENTH image on a Jetson (arm64 / JetPack 6, L4T r36.3).
# Takes several hours -- most of it is `apt install ros-humble-desktop`,
# the CUDA librealsense build and `colcon build`.
set -euo pipefail

IMAGE="${IMAGE:-misys:f1tenth}"
UNAME_ARG="${UNAME_ARG:-$(id -un)}"
UID_ARG="${UID_ARG:-$(id -u)}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "ERROR: this image is Jetson/arm64 only (current arch: $(uname -m))." >&2
  exit 1
fi

echo "Building $IMAGE  (user=$UNAME_ARG uid=$UID_ARG)"
DOCKER_BUILDKIT=1 docker build \
  --build-arg UID="$UID_ARG" \
  --build-arg UNAME="$UNAME_ARG" \
  --build-arg PW="${PW_ARG:-root}" \
  -t "$IMAGE" \
  "$(dirname "$0")"

echo "Done. Run it with: ./run.sh"
