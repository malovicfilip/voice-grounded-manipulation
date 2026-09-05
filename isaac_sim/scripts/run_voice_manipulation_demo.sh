#!/usr/bin/env bash
set -euo pipefail

if [[ "${ACCEPT_EULA:-}" != "Y" ]]; then
  echo 'Set ACCEPT_EULA=Y only after accepting the NVIDIA Isaac Sim EULA.' >&2
  exit 2
fi

TRANSCRIPT=""
AUDIO_PATH=""
PROVIDER="openai"
SESSION=false
STREAM_ARGS=()
RUN_ID="voice-demo-$(date -u +%Y%m%dT%H%M%SZ)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --stream-host)
      STREAM_ARGS=(--stream-host "${2:?Provide the server IPv4 address}")
      shift 2
      ;;
    --session)
      SESSION=true
      shift
      ;;
    --transcript)
      TRANSCRIPT="${2:-}"
      shift 2
      ;;
    --audio)
      AUDIO_PATH="${2:-}"
      shift 2
      ;;
    --provider)
      PROVIDER="${2:-}"
      shift 2
      ;;
    --run-id)
      RUN_ID="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "${SESSION}" != true ]] && { [[ -n "${TRANSCRIPT}" && -n "${AUDIO_PATH}" ]] || \
   [[ -z "${TRANSCRIPT}" && -z "${AUDIO_PATH}" ]]; }; then
  echo 'Provide exactly one of --transcript or --audio.' >&2
  exit 2
fi
if [[ "${PROVIDER}" != "openai" && "${PROVIDER}" != "rules" ]]; then
  echo 'Provider must be openai or rules.' >&2
  exit 2
fi
if [[ -n "${AUDIO_PATH}" && "${PROVIDER}" != "openai" ]]; then
  echo 'Audio mode currently requires the validated OpenAI intent provider.' >&2
  exit 2
fi
if [[ ! "${RUN_ID}" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$ ]]; then
  echo 'Run ID must contain only letters, numbers, underscores, and hyphens.' >&2
  exit 2
fi

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
ROS_WORKSPACE="${REPOSITORY_ROOT}/ros2_ws"
PIXI_BIN="${PIXI_BIN:-${HOME}/.pixi/bin/pixi}"
NVIDIA_ROS_WORKSPACE="${NVIDIA_ROS_WORKSPACE:-${HOME}/IsaacSim-ros_workspaces}"
FAST_DDS_PROFILE="${NVIDIA_ROS_WORKSPACE}/jazzy_ws/fastdds.xml"
OUTPUT_ROOT="${REPOSITORY_ROOT}/isaac_sim/_output"
HOST_OUTPUT="${OUTPUT_ROOT}/${RUN_ID}"
CONTAINER_OUTPUT="/workspace/isaac_sim/_output/${RUN_ID}"
CONTAINER_NAME="vgm-integrated-${RUN_ID,,}"
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
if [[ -n "${AUDIO_PATH}" && ! -s "${AUDIO_PATH}" ]]; then
  echo "Audio file is missing or empty: ${AUDIO_PATH}" >&2
  exit 2
fi
if [[ "${SESSION}" != true && "${PROVIDER}" == "openai" && ! -s "${REPOSITORY_ROOT}/.env.local" && \
      -z "${OPENAI_API_KEY:-}" ]]; then
  echo 'OPENAI_API_KEY is unavailable; use an ignored .env.local or the environment.' >&2
  exit 2
fi
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
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display \
  -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 \
  -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
  -u 1234:1234 "${ISAAC_VOLUMES[@]}" \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  "${IMAGE}" -u /workspace/isaac_sim/scripts/run_integrated_scene.py \
  --headless --output-directory "${CONTAINER_OUTPUT}" \
  "${STREAM_ARGS[@]}" \
  >"${HOST_OUTPUT}/isaac.log" 2>&1 &
ISAAC_PID=$!

for _ in {1..180}; do
  [[ -f "${HOST_OUTPUT}/bridge.ready" ]] && break
  if ! kill -0 "${ISAAC_PID}" 2>/dev/null; then
    echo 'Isaac Sim exited before the integrated bridge was ready.' >&2
    exit 1
  fi
  sleep 2
done
if [[ ! -f "${HOST_OUTPUT}/bridge.ready" ]]; then
  echo 'Timed out waiting for the integrated Isaac bridge.' >&2
  exit 1
fi

touch "${HOST_OUTPUT}/capture_initial.request"
for _ in {1..420}; do
  [[ -s "${HOST_OUTPUT}/initial_grounded_scene.json" ]] && break
  kill -0 "${ISAAC_PID}" 2>/dev/null || break
  sleep 2
done
if [[ ! -s "${HOST_OUTPUT}/initial_grounded_scene.json" ]]; then
  echo 'Timed out waiting for the initial RGB-D grounding result.' >&2
  exit 1
fi

if [[ "${SESSION}" == true ]]; then
  : # Workstation client supplies confirmed high-level tasks over SSH.
elif [[ -n "${TRANSCRIPT}" ]]; then
  run_ros ros2 run vgm_runtime intent_cli \
    --transcript "${TRANSCRIPT}" \
    --scene-json "${HOST_OUTPUT}/initial_grounded_scene.json" \
    --provider "${PROVIDER}" \
    --audit-jsonl "${HOST_OUTPUT}/audit.jsonl" \
    >"${HOST_OUTPUT}/decision.json"
else
  run_ros ros2 run vgm_runtime voice_cli "${AUDIO_PATH}" \
    --scene-json "${HOST_OUTPUT}/initial_grounded_scene.json" \
    --audit-jsonl "${HOST_OUTPUT}/audit.jsonl" \
    >"${HOST_OUTPUT}/decision.json"
fi

setsid env FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}" \
  "${PIXI_BIN}" run --manifest-path "${ROS_WORKSPACE}/pixi.toml" \
  bash -c 'source "$1"; shift; exec "$@"' \
  bash "${ROS_WORKSPACE}/install/setup.bash" \
  ros2 launch vgm_moveit_demo moveit_headless.launch.py \
  ros2_control_hardware_type:=isaac use_sim_time:=true \
  base_x:=-0.45 base_y:=0.0 base_z:=0.75 base_yaw:=0.0 \
  >"${HOST_OUTPUT}/moveit.log" 2>&1 &
MOVEIT_PID=$!

controllers_ready=false
for _ in {1..90}; do
  controllers="$(run_ros ros2 control list_controllers 2>/dev/null || true)"
  if grep -q '^joint_state_broadcaster.*active' <<<"${controllers}" && \
     grep -q '^panda_arm_controller.*active' <<<"${controllers}"; then
    controllers_ready=true
    break
  fi
  sleep 1
done
if [[ "${controllers_ready}" != "true" ]]; then
  echo 'Timed out waiting for active MoveIt controllers.' >&2
  exit 1
fi

if [[ "${SESSION}" == true ]]; then
  touch "${HOST_OUTPUT}/session.ready"
  echo "Reusable simulator session ready: ${HOST_OUTPUT}"
  # Bound the session independently of the external Brev cost guard.
  session_deadline=$((SECONDS + 7200))
  while [[ ! -f "${HOST_OUTPUT}/shutdown.request" && ${SECONDS} -lt ${session_deadline} ]]; do
    kill -0 "${ISAAC_PID}" 2>/dev/null || exit 1
    kill -0 "${MOVEIT_PID}" 2>/dev/null || exit 1
    sleep 1
  done
  exit 0
fi

touch "${HOST_OUTPUT}/capture_verification.request"
for _ in {1..30}; do
  [[ -s "${HOST_OUTPUT}/verification_grounded_scene.json" ]] && break
  sleep 1
done
if [[ ! -s "${HOST_OUTPUT}/verification_grounded_scene.json" ]]; then
  echo 'Timed out waiting for pre-execution RGB-D revalidation.' >&2
  exit 1
fi

run_ros ros2 run vgm_runtime execution_gate \
  --decision-json "${HOST_OUTPUT}/decision.json" \
  --latest-scene-json "${HOST_OUTPUT}/verification_grounded_scene.json" \
  --output-json "${HOST_OUTPUT}/execution_plan.json" \
  >"${HOST_OUTPUT}/execution_gate.log"

mapfile -t EXECUTION_ARGUMENTS < <(run_ros python - "${HOST_OUTPUT}/execution_plan.json" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
skill = value["validated_skill"]
if skill["skill"] != "pick_and_place":
    raise SystemExit("demo requires a validated pick_and_place skill")
positions = [
    item["position_m"]
    for item in value["plan"]["primitives"]
    if item["kind"] == "move_cartesian"
]
if len(positions) != 6:
    raise SystemExit("validated plan does not have the expected bounded shape")
print(skill["request_id"])
print(skill["object_id"])
print(skill["target_id"])
print(positions[0][0])
print(positions[0][1])
from vgm_runtime.config import load_json_config
print(positions[0][2] - load_json_config("safety_policy.json")["pick"]["approach_height_m"])
PY
)
if [[ "${#EXECUTION_ARGUMENTS[@]}" -ne 6 ]]; then
  echo 'Failed to derive the bounded MoveIt invocation.' >&2
  exit 1
fi

python3 - "${HOST_OUTPUT}/expected_outcome.json" \
  "${EXECUTION_ARGUMENTS[1]}" "${EXECUTION_ARGUMENTS[2]}" <<'PY'
import json
import sys
from pathlib import Path

path, object_id, target_id = sys.argv[1:]
Path(path).write_text(
    json.dumps({"object_id": object_id, "target_id": target_id}, indent=2) + "\n",
    encoding="utf-8",
)
PY

run_ros timeout 130 ros2 launch vgm_moveit_demo safe_pick_and_place.launch.py \
  request_id:="${EXECUTION_ARGUMENTS[0]}" \
  object_id:="${EXECUTION_ARGUMENTS[1]}" \
  target_id:="${EXECUTION_ARGUMENTS[2]}" \
  object_x:="${EXECUTION_ARGUMENTS[3]}" \
  object_y:="${EXECUTION_ARGUMENTS[4]}" \
  object_z:="${EXECUTION_ARGUMENTS[5]}" \
  scene_file:="${HOST_OUTPUT}/verification_grounded_scene.json" \
  | tee "${HOST_OUTPUT}/execution.log"
grep -q 'VGM_PICK_PLACE_RESULT .*plan=success execution=success' \
  "${HOST_OUTPUT}/execution.log"

run_ros python -c 'import json,sys; from vgm_runtime.session_backend import robot_state; from pathlib import Path; Path(sys.argv[1]).write_text(json.dumps(robot_state()))' \
  "${HOST_OUTPUT}/final_robot_state.json"
touch "${HOST_OUTPUT}/capture_final.request"
for _ in {1..90}; do
  [[ -s "${HOST_OUTPUT}/isaac_capture.json" ]] && break
  kill -0 "${ISAAC_PID}" 2>/dev/null || break
  sleep 1
done
if [[ ! -s "${HOST_OUTPUT}/final.png" || \
      ! -s "${HOST_OUTPUT}/final_grounded_scene.json" || \
      ! -s "${HOST_OUTPUT}/isaac_capture.json" ]]; then
  echo 'Isaac Sim did not produce the final RGB-D evidence.' >&2
  exit 1
fi

run_ros ros2 run vgm_runtime validate_outcome \
  --execution-plan "${HOST_OUTPUT}/execution_plan.json" \
  --final-scene "${HOST_OUTPUT}/final_grounded_scene.json" \
  --previous-scene "${HOST_OUTPUT}/rest_start_grounded_scene.json" \
  --robot-state "${HOST_OUTPUT}/final_robot_state.json" \
  --output-json "${HOST_OUTPUT}/outcome_validation.json"

python3 - "${HOST_OUTPUT}/manifest.json" "${RUN_ID}" "${PROVIDER}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

path, run_id, provider = sys.argv[1:]
manifest = {
    "artifacts": sorted(
        item.name for item in Path(path).parent.iterdir()
        if item.is_file() and item.name != Path(path).name
    ),
    "completed_at": datetime.now(timezone.utc).isoformat(),
    "intent_provider": provider,
    "isaac_sim_version": "6.0.1",
    "result": "success",
    "ros_distro": "jazzy",
    "run_id": run_id,
}
Path(path).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
PY

echo "Voice-grounded manipulation demo succeeded: ${HOST_OUTPUT}"
