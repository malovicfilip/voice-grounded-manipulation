"""One JSON request over stdin for an existing headless simulator session.

Run through the locked ROS environment. SSH is the transport; this module opens
no network listener. Only the deterministic validator may invoke the executor.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .config import load_json_config
from .outcome import validate_placement_outcome
from .serialization import load_scene
from .tasks import check_preconditions
from .types import GroundedScene
from .validator import SkillValidator


def write_json(path: Path, value) -> None:
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def robot_state(*, cancel: bool = False) -> dict:
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("vgm_session_observer_" + uuid.uuid4().hex[:8])
    try:
        stop_applied = False
        if cancel:
            client = node.create_client(Trigger, "/vgm/stop")
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
                stop_applied = future.done() and future.result() is not None and future.result().success
        samples = []
        def on_state(message):
            positions = {name: float(position) for name, position in zip(message.name, message.position)}
            if all(f"panda_joint{i}" in positions for i in range(1, 8)):
                samples.append((time.monotonic(), [positions[f"panda_joint{i}"] for i in range(1, 8)]))
        node.create_subscription(JointState, "/joint_states", on_state, 10)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(samples) >= 3 and samples[-1][0] - samples[0][0] >= 1.0:
                break
        if len(samples) < 3:
            raise RuntimeError("fresh robot joint-state samples are unavailable")
        # Ignore the first half-second of cancellation deceleration.
        tail = [positions for stamp, positions in samples if stamp >= samples[-1][0] - 0.5]
        drift = max(max(values) - min(values) for values in zip(*tail))
        client = node.create_client(GetPlanningScene, "/get_planning_scene")
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("MoveIt planning scene service is unavailable")
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("MoveIt held-object state is unavailable")
        held = [item.object.id for item in future.result().scene.robot_state.attached_collision_objects]
        if len(held) > 1:
            raise RuntimeError("unexpected multiple attached objects")
        return {"positions": samples[-1][1], "drift_rad": drift,
                "stationary": math.isfinite(drift) and drift <= 0.01,
                "held_object_id": held[0] if held else None, "stop_applied": bool(stop_applied)}
    finally:
        node.destroy_node()
        rclpy.shutdown()


class SimulatorSession:
    def __init__(self, directory: Path):
        self.directory = directory
        self.policy = load_json_config("safety_policy.json")
        if not (directory / "session.ready").is_file():
            raise RuntimeError("simulator session is not ready")

    def state(self):
        path = self.directory / "session_state.json"
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
        return {"expected_positions": {}, "used_requests": []}

    def capture(self, expected=None):
        prefix = "capture-" + uuid.uuid4().hex[:16]
        hints = self.state()["expected_positions"] if expected is None else expected
        write_json(self.directory / (prefix + ".json"), {"expected_positions": hints})
        deadline = time.monotonic() + 30
        while not (self.directory / (prefix + ".ready")).is_file():
            if time.monotonic() > deadline:
                raise RuntimeError("RGB-D capture timed out")
            time.sleep(0.1)
        scene = load_scene(self.directory / (prefix + "_grounded_scene.json"))
        state = robot_state()
        return GroundedScene(scene.revision, scene.captured_at_s, scene.objects, state["held_object_id"])

    def stop(self):
        (self.directory / "stopped").touch()
        state = robot_state(cancel=True)
        result = {"status": "stopped" if state["stationary"] else "stopping", "robot": state}
        write_json(self.directory / "stop_evidence.json", result)
        return result

    def recover(self):
        if (self.directory / "execution.active").exists():
            raise RuntimeError("cannot recover while an executor is active")
        scene = self.capture()
        state = robot_state()
        if not state["stationary"] or scene.held_object_id is not None:
            raise RuntimeError("recovery requires a stationary robot with no held object")
        # Explicit recovery performs no motion and never resumes a failed task.
        (self.directory / "stopped").unlink(missing_ok=True)
        result = {"status": "recovered", "scene_revision": scene.revision, "robot": state}
        write_json(self.directory / "recovery_evidence.json", result)
        return result

    def execute(self, proposal: dict, baseline: dict, *, inject_fault=False):
        with (self.directory / "execution.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if (self.directory / "stopped").exists():
                raise RuntimeError("session is stopped; explicit recovery is required")
            scene = self.capture()
            validator = SkillValidator()
            validator.validate(proposal, scene)
            check_preconditions(proposal, scene.held_object_id, scene, self.policy)
            state = self.state()
            request_id = proposal["request_id"]
            if request_id in state["used_requests"]:
                raise ValueError("execution request was already consumed")
            selected = scene.objects.get(proposal["object_id"])
            if proposal["skill"] in {"pick", "pick_and_place"}:
                position = baseline.get("object_position")
                if not isinstance(position, list) or len(position) != 3:
                    raise ValueError("initial object observation is missing")
                drift = math.dist(position, selected.position_m)
                if not math.isfinite(drift) or drift > self.policy["maximum_object_drift_m"]:
                    raise ValueError("object moved before execution")
            state["used_requests"].append(request_id)
            write_json(self.directory / "session_state.json", state)
            if proposal["skill"] == "inspect":
                return {"status": "succeeded", "observation": selected.to_mapping()}
            if proposal["skill"] not in {"pick", "place", "pick_and_place", "move_named_pose", "open_gripper", "close_gripper"}:
                raise ValueError("unsupported executable skill")
            work = self.directory / request_id
            work.mkdir(mode=0o770)
            scene_path = work / "scene.json"
            # Preserve conservative collision boxes for occluded objects at
            # their last verified destination, never revert a moved cube to its spawn.
            collision_objects = {
                cube["object_id"]: {"object_id": cube["object_id"], "position_m": cube["position"]}
                for cube in load_json_config("phase_1_scene.json")["cubes"]
            }
            for object_id, position in state["expected_positions"].items():
                collision_objects[object_id]["position_m"] = position
            for observation in scene.objects.values():
                collision_objects[observation.object_id] = observation.to_mapping()
            write_json(scene_path, {"objects": list(collision_objects.values())})
            write_json(work / "proposal.json", proposal)
            xyz = selected.position_m if selected else (0.0, 0.0, 0.775)
            command = ["ros2", "launch", "vgm_moveit_demo", "safe_pick_and_place.launch.py",
                       f"request_id:={request_id}", f"skill:={proposal['skill']}",
                       f"object_id:={proposal['object_id'] or ''}", f"target_id:={proposal['target_id'] or ''}",
                       f"pose_name:={proposal['pose_name'] or ''}", f"scene_file:={scene_path}",
                       f"object_x:={xyz[0]}", f"object_y:={xyz[1]}", f"object_z:={xyz[2]}"]
            if inject_fault:
                command.append("fault_before_primitive:=pick_approach")
            active = self.directory / "execution.active"
            active.touch()
            process = None
            try:
                if (self.directory / "stopped").exists():
                    raise RuntimeError("stop arrived before launch")
                with (work / "execution.log").open("w") as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    deadline = time.monotonic() + 130
                    while process.poll() is None:
                        if time.monotonic() > deadline:
                            raise subprocess.TimeoutExpired(command, 130)
                        if (self.directory / "stopped").exists():
                            robot_state(cancel=True)
                        time.sleep(0.1)
                output = (work / "execution.log").read_text(encoding="utf-8")
                marker = f"VGM_SKILL_RESULT skill={proposal['skill']} status=succeeded"
                if process.returncode != 0 or marker not in output or (self.directory / "stopped").exists():
                    raise RuntimeError("MoveIt skill failed or was cancelled")
                result = {"status": "succeeded", "skill": proposal["skill"], "request_id": request_id}
                if proposal["skill"] in {"place", "pick_and_place"}:
                    target = self.policy["targets"][proposal["target_id"]]["position_m"]
                    hints = dict(state["expected_positions"])
                    hints[proposal["object_id"]] = [target[0], target[1], target[2] + 0.025]
                    time.sleep(0.6)
                    final_scene = self.capture(hints)
                    result["outcome"] = validate_placement_outcome({"validated_skill": proposal}, final_scene, self.policy)
                    state["expected_positions"] = hints
                    write_json(self.directory / "session_state.json", state)
                    write_json(work / "final_scene.json", final_scene.to_mapping())
                write_json(work / "result.json", result)
                return result
            except BaseException:
                self.stop()
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                raise
            finally:
                active.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.session):
        parser.error("invalid session identifier")
    root = Path(__file__).resolve().parents[4]
    directory = root / "isaac_sim" / "_output" / args.session
    try:
        request = json.loads(sys.stdin.read(100_000))
        session = SimulatorSession(directory)
        operation = request.get("operation")
        if operation == "capture":
            result = session.capture().to_mapping()
        elif operation == "execute":
            result = session.execute(request["proposal"], request["baseline"])
        elif operation == "fault_test":
            result = session.execute(request["proposal"], request["baseline"], inject_fault=True)
        elif operation == "stop":
            result = session.stop()
        elif operation == "recover":
            result = session.recover()
        elif operation == "robot_state":
            result = robot_state()
        elif operation == "execution_progress":
            used = session.state()["used_requests"]
            lines = []
            if used:
                path = directory / used[-1] / "execution.log"
                if path.is_file():
                    lines = [line for line in path.read_text(encoding="utf-8").splitlines() if "VGM_" in line]
            result = {"active": (directory / "execution.active").exists(), "events": lines}
        elif operation == "shutdown":
            session.stop()
            (directory / "shutdown.request").touch()
            result = {"status": "shutdown_requested"}
        elif operation == "transcribe":
            from .whisper_transcriber import FasterWhisperTranscriber
            # Restrict remote audio reads to recordings in this session.
            audio = directory / "command.wav"
            transcript = FasterWhisperTranscriber().transcribe(audio)
            result = {"text": transcript.text, "confidence": transcript.confidence, "language": transcript.language}
        else:
            raise ValueError("unknown session operation")
        print(json.dumps({"ok": True, "result": result}, allow_nan=False))
    except Exception as error:
        print(json.dumps({"ok": False, "error": type(error).__name__, "message": str(error)}))
        raise SystemExit(2)


if __name__ == "__main__":
    main()
