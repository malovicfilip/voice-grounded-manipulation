#!/usr/bin/env bash
set -euo pipefail

if [[ "${ACCEPT_EULA:-}" != "Y" ]]; then
  echo 'Set ACCEPT_EULA=Y only after accepting the NVIDIA Isaac Sim EULA.' >&2
  exit 2
fi

TARGET="${1:-extended}"
case "${TARGET}" in
  ready|extended|transport) ;;
  *)
    echo "Rejected target '${TARGET}'. Allowed: ready, extended, transport" >&2
    exit 2
    ;;
esac

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
ROS_WORKSPACE="${REPOSITORY_ROOT}/ros2_ws"
PIXI_BIN="${PIXI_BIN:-${HOME}/.pixi/bin/pixi}"
NVIDIA_ROS_WORKSPACE="${NVIDIA_ROS_WORKSPACE:-${HOME}/IsaacSim-ros_workspaces}"
FAST_DDS_PROFILE="${NVIDIA_ROS_WORKSPACE}/jazzy_ws/fastdds.xml"
OUTPUT_ROOT="${REPOSITORY_ROOT}/isaac_sim/_output"
RUN_ID="${2:-moveit-$(date -u +%Y%m%dT%H%M%SZ)}"
HOST_OUTPUT="${OUTPUT_ROOT}/${RUN_ID}"
CONTAINER_OUTPUT="/workspace/isaac_sim/_output/${RUN_ID}"
CONTAINER_NAME="vgm-moveit-${RUN_ID,,}"
IMAGE="nvcr.io/nvidia/isaac-sim:6.0.1"
MOVEIT_PID=""
ISAAC_PID=""

for required_path in \
  "${PIXI_BIN}" \
  "${ROS_WORKSPACE}/pixi.toml" \
  "${ROS_WORKSPACE}/install/setup.bash" \
  "${FAST_DDS_PROFILE}"; do
  if [[ ! -e "${required_path}" ]]; then
    echo "Missing required setup path: ${required_path}" >&2
    exit 2
  fi
done

if [[ -e "${HOST_OUTPUT}" ]]; then
  echo "Demo output already exists: ${HOST_OUTPUT}" >&2
  exit 2
fi

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "Demo container already exists: ${CONTAINER_NAME}" >&2
  exit 2
fi

sudo install -d -o "$(id -u)" -g 1234 -m 0770 "${HOST_OUTPUT}"

run_ros() {
  FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}" \
    "${PIXI_BIN}" run --manifest-path "${ROS_WORKSPACE}/pixi.toml" \
    bash -c 'source "$1"; shift; exec "$@"' \
    bash "${ROS_WORKSPACE}/install/setup.bash" "$@"
}

cleanup() {
  set +e
  if [[ -n "${MOVEIT_PID}" ]] && kill -0 "${MOVEIT_PID}" 2>/dev/null; then
    kill -INT -- "-${MOVEIT_PID}" 2>/dev/null
    for _ in {1..10}; do
      kill -0 "${MOVEIT_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-${MOVEIT_PID}" 2>/dev/null
  fi
  if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
    docker stop --time 15 "${CONTAINER_NAME}" >/dev/null
  fi
  [[ -z "${ISAAC_PID}" ]] || wait "${ISAAC_PID}" 2>/dev/null
}
trap cleanup EXIT INT TERM

ISAAC_VOLUMES=(
  -v /home/ubuntu/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw
  -v /home/ubuntu/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw
  -v /home/ubuntu/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw
  -v /home/ubuntu/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw
  -v /home/ubuntu/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw
  -v /home/ubuntu/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw
  -v /home/ubuntu/.cache/ov/hub:/var/cache/hub:rw
  -v "${FAST_DDS_PROFILE}:/fastdds.xml:ro"
  -v "${REPOSITORY_ROOT}:/workspace:ro"
  -v "${OUTPUT_ROOT}:/workspace/isaac_sim/_output:rw"
)

docker run --rm --name "${CONTAINER_NAME}" --gpus all --network=host \
  -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 \
  -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
  -u 1234:1234 "${ISAAC_VOLUMES[@]}" \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  "${IMAGE}" -u /workspace/isaac_sim/scripts/run_moveit_scene.py \
  --headless --output-directory "${CONTAINER_OUTPUT}" \
  >"${HOST_OUTPUT}/isaac.log" 2>&1 &
ISAAC_PID=$!

for _ in {1..360}; do
  [[ -f "${HOST_OUTPUT}/initial.ready" ]] && break
  if ! kill -0 "${ISAAC_PID}" 2>/dev/null; then
    echo "Isaac Sim exited before the initial capture was ready." >&2
    exit 1
  fi
  sleep 2
done
if [[ ! -f "${HOST_OUTPUT}/initial.ready" ]]; then
  echo "Timed out waiting for Isaac Sim and the initial camera frame." >&2
  exit 1
fi

run_ros ros2 daemon stop >/dev/null 2>&1 || true
run_ros timeout 30 ros2 topic echo --once \
  /isaac_joint_states sensor_msgs/msg/JointState \
  >"${HOST_OUTPUT}/initial_joint_state.yaml"

setsid env FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}" \
  "${PIXI_BIN}" run --manifest-path "${ROS_WORKSPACE}/pixi.toml" \
  bash -c 'source "$1"; shift; exec "$@"' \
  bash "${ROS_WORKSPACE}/install/setup.bash" \
  ros2 launch vgm_moveit_demo moveit_headless.launch.py \
  ros2_control_hardware_type:=isaac use_sim_time:=true \
  >"${HOST_OUTPUT}/moveit.log" 2>&1 &
MOVEIT_PID=$!

controllers_ready=false
for _ in {1..60}; do
  controllers="$(run_ros ros2 control list_controllers 2>/dev/null || true)"
  if grep -q '^joint_state_broadcaster.*active' <<<"${controllers}" && \
     grep -q '^panda_arm_controller.*active' <<<"${controllers}"; then
    controllers_ready=true
    break
  fi
  sleep 1
done
if [[ "${controllers_ready}" != "true" ]]; then
  echo "Timed out waiting for active MoveIt controllers." >&2
  exit 1
fi

run_ros ros2 launch vgm_moveit_demo safe_named_pose.launch.py \
  target:="${TARGET}" | tee "${HOST_OUTPUT}/skill.log"
grep -q "VGM_DEMO_RESULT target=${TARGET} plan=success execution=success" \
  "${HOST_OUTPUT}/skill.log"

run_ros timeout 30 ros2 topic echo --once \
  /isaac_joint_states sensor_msgs/msg/JointState \
  >"${HOST_OUTPUT}/final_joint_state.yaml"
touch "${HOST_OUTPUT}/capture_final.request"

for _ in {1..60}; do
  [[ -s "${HOST_OUTPUT}/isaac_capture.json" ]] && break
  kill -0 "${ISAAC_PID}" 2>/dev/null || break
  sleep 1
done
if [[ ! -s "${HOST_OUTPUT}/initial.png" || \
      ! -s "${HOST_OUTPUT}/final.png" || \
      ! -s "${HOST_OUTPUT}/isaac_capture.json" ]]; then
  echo "Isaac Sim did not produce both visual captures." >&2
  exit 1
fi

python3 - "${HOST_OUTPUT}/manifest.json" "${RUN_ID}" "${TARGET}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

path, run_id, target = sys.argv[1:]
manifest = {
    "artifacts": {
        "final_joint_state": "final_joint_state.yaml",
        "final_rgb": "final.png",
        "initial_joint_state": "initial_joint_state.yaml",
        "initial_rgb": "initial.png",
        "isaac_capture": "isaac_capture.json",
        "isaac_log": "isaac.log",
        "moveit_log": "moveit.log",
        "skill_log": "skill.log",
    },
    "completed_at": datetime.now(timezone.utc).isoformat(),
    "isaac_sim_version": "6.0.1",
    "result": "success",
    "ros_distro": "jazzy",
    "run_id": run_id,
    "target": target,
}
Path(path).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
PY

echo "MoveIt demo succeeded: ${HOST_OUTPUT}"
