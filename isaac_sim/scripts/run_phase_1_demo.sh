#!/usr/bin/env bash
set -euo pipefail

if [[ "${ACCEPT_EULA:-}" != "Y" ]]; then
  echo 'Set ACCEPT_EULA=Y only after accepting the NVIDIA Isaac Sim EULA.' >&2
  exit 2
fi

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
OUTPUT_ROOT="${REPOSITORY_ROOT}/isaac_sim/_output"
RUN_ID="${1:-demo-$(date -u +%Y%m%dT%H%M%SZ)}"
HOST_DEMO_DIRECTORY="${OUTPUT_ROOT}/${RUN_ID}"
CONTAINER_DEMO_DIRECTORY="/workspace/isaac_sim/_output/${RUN_ID}"
IMAGE="nvcr.io/nvidia/isaac-sim:6.0.1"

if [[ -e "${HOST_DEMO_DIRECTORY}" ]]; then
  echo "Demo output already exists: ${HOST_DEMO_DIRECTORY}" >&2
  exit 2
fi

ISAAC_VOLUMES=(
  -v /home/ubuntu/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw
  -v /home/ubuntu/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw
  -v /home/ubuntu/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw
  -v /home/ubuntu/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw
  -v /home/ubuntu/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw
  -v /home/ubuntu/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw
  -v /home/ubuntu/.cache/ov/hub:/var/cache/hub:rw
  -v "${REPOSITORY_ROOT}:/workspace:ro"
  -v "${OUTPUT_ROOT}:/workspace/isaac_sim/_output:rw"
)

docker run --rm --gpus all \
  -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 -u 1234:1234 \
  "${ISAAC_VOLUMES[@]}" \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  "${IMAGE}" -u \
  /workspace/isaac_sim/scripts/capture_phase_1_demo.py \
  --output-directory "${CONTAINER_DEMO_DIRECTORY}"

echo "Demo artifacts: ${HOST_DEMO_DIRECTORY}"
