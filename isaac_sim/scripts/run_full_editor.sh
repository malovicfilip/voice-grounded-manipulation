#!/usr/bin/env bash
# Run on the existing Brev VM after stopping the manipulation session.
set -euo pipefail
if [[ $# != 1 || ! $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Usage: bash $0 SERVER_IPV4" >&2
  exit 2
fi
if [[ ${ACCEPT_EULA:-} != Y ]]; then
  echo 'Set ACCEPT_EULA=Y after accepting the NVIDIA Isaac Sim EULA.' >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq '^vgm-(integrated-|full-editor$)'; then
  echo 'Stop the existing simulation/editor before starting this editor.' >&2
  exit 1
fi
ROOT=/home/ubuntu/workspace
test -d "$ROOT/isaac_sim/_output"
# Read-only source; save edited scenes under /workspace/isaac_sim/_output.
# Isolate ROS discovery from the normal manipulation domain (0).
exec docker run --rm --name vgm-full-editor --gpus all --network=host \
  -e ACCEPT_EULA=Y -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=42 \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display \
  -v /home/ubuntu/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v /home/ubuntu/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v /home/ubuntu/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v /home/ubuntu/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v /home/ubuntu/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v /home/ubuntu/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v /home/ubuntu/.cache/ov/hub:/var/cache/hub:rw \
  -v "$ROOT:/workspace:ro" \
  -v "$ROOT/isaac_sim/_output:/workspace/isaac_sim/_output:rw" \
  -u 1234:1234 --entrypoint /isaac-sim/isaac-sim.streaming.sh \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp="$1" \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998 \
  --/app/window/width=1280 --/app/window/height=720 \
  --/app/window/hideUi=false
