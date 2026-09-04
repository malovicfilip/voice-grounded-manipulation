#!/usr/bin/env python3
"""Live acceptance campaign for the configured, reusable Isaac/MoveIt session.

The existing simulator session must already be running with a verified Brev
cost guard. This script neither starts cloud compute nor transfers API keys.
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/vgm_runtime"))
from vgm_runtime.audit import AuditLogger
from vgm_runtime.openai_intent import load_local_api_key
from vgm_runtime.ssh_backend import SSHBackend
from vgm_runtime.task_cli import upload_and_transcribe
from vgm_runtime.tasks import OpenAITaskModel, TaskSession
from vgm_runtime.validator import SkillValidator


def proposal(scene):
    return {"schema_version": 1, "request_id": "acceptance_" + uuid.uuid4().hex[:16],
            "skill": "pick_and_place", "object_id": "red_cube", "target_id": "blue_target",
            "pose_name": None, "reason": None, "scene_revision": scene.revision}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def boundary(backend):
    scene = backend.capture()
    before = backend.request("robot_state")
    rejected = []
    fixtures = {
        "direct_control": {**proposal(scene), "joint_targets": [0.0]},
        "unknown_skill": {**proposal(scene), "skill": "execute_trajectory"},
        "missing_grounding": {**proposal(scene), "object_id": "invisible_cube"},
        "unknown_target": {**proposal(scene), "target_id": "outside_table"},
    }
    for name, value in fixtures.items():
        # Renew only the fixture revision; every rejection traverses the live
        # remote validator and must fail before it launches a MoveIt executor.
        latest = backend.capture()
        value["scene_revision"] = latest.revision
        try:
            backend.execute(value, latest)
        except RuntimeError as error:
            rejected.append({"fixture": name, "refusal": str(error)})
        else:
            raise AssertionError(f"invalid request was accepted: {name}")
    after = backend.request("robot_state")
    delta = max(abs(a - b) for a, b in zip(before["positions"], after["positions"]))
    require(delta < 0.01, "robot moved during rejected requests")
    return {"rejected": rejected, "joint_delta_rad": delta}


def live_stop(backend):
    scene = backend.capture()
    value = proposal(scene)
    initial = backend.request("robot_state")
    result = []
    def run():
        try:
            result.append(backend.execute(value, scene))
        except Exception as error:
            result.append({"status": "failed", "message": str(error)})
    worker = threading.Thread(target=run)
    worker.start()
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        events = backend.request("execution_progress")
        if any("VGM_PRIMITIVE_START kind=pick_approach" in line for line in events["events"]):
            break
        if not worker.is_alive():
            raise AssertionError(f"executor ended before the stop fixture: {result}")
        time.sleep(.2)
    else:
        backend.stop()
        raise AssertionError("arm motion did not start before the test deadline")
    moving = backend.request("robot_state")
    stop_result = backend.stop()
    worker.join(timeout=20)
    require(not worker.is_alive(), "executor did not finish after cancellation")
    settled = backend.request("robot_state")
    events = backend.request("execution_progress")
    require(any("VGM_EXECUTION_STOP reason=operator_stop" in line for line in events["events"]),
            "live executor did not acknowledge the operator stop")
    require(not any("VGM_PRIMITIVE_START kind=pick_descend" in line for line in events["events"]),
            "a following primitive started after stop")
    require(settled["stationary"], "robot did not settle after cancellation")
    moved = max(abs(a - b) for a, b in zip(initial["positions"], moving["positions"]))
    require(moved > .01, "stop test did not observe actual arm movement")
    recovery = backend.recover()
    require(recovery["status"] == "recovered", "explicit recovery failed")
    return {"motion_before_stop_rad": moved, "stop": stop_result, "settled": settled,
            "events": events["events"], "execution": result, "recovery": recovery}


def fault(backend):
    scene = backend.capture()
    value = proposal(scene)
    try:
        backend.request("fault_test", proposal=value,
                        baseline={"object_position": list(scene.objects["red_cube"].position_m)})
    except RuntimeError as error:
        refusal = str(error)
    else:
        raise AssertionError("injected backend fault was not reported")
    events = backend.request("execution_progress")
    require(any("VGM_FAULT_INJECTED" in line for line in events["events"]), "fault fixture was not reached")
    require(not any("VGM_PRIMITIVE_START kind=pick_approach" in line for line in events["events"]),
            "motion continued after the injected fault")
    state = backend.request("robot_state")
    require(state["stationary"], "robot is moving after the backend fault")
    try:
        backend.execute(proposal(backend.capture()), backend.capture())
    except RuntimeError:
        pass
    else:
        raise AssertionError("faulted session accepted motion without recovery")
    recovery = backend.recover()
    return {"refusal": refusal, "events": events["events"], "robot": state, "recovery": recovery}


def task(backend, audit, transcript):
    session = TaskSession(OpenAITaskModel(), backend, audit)
    pending = session.prepare(transcript)
    require(pending["status"] == "awaiting_confirmation", f"task was not confirmable: {pending}")
    # Running this acceptance script authorizes its explicit, fixed test tasks.
    result = session.confirm(pending["confirmation"])
    require(result["status"] == "completed", f"task failed: {result}")
    return {"transcript": transcript, "proposal": pending, "execution": result}


def dialogue(backend, audit):
    session = TaskSession(OpenAITaskModel(), backend, audit)
    before = backend.request("robot_state")
    ambiguous = session.prepare("Move the cube to the target.")
    require(ambiguous["status"] == "refused", "ambiguous command did not request clarification")
    corrected = session.prepare("I mean inspect the red cube only, without moving it.")
    require(corrected["status"] == "awaiting_confirmation", "clarified command was not confirmable")
    require(all(step["skill"] == "inspect" for step in corrected["steps"]), "clarification changed requested inspection")
    after = backend.request("robot_state")
    require(max(abs(a-b) for a,b in zip(before["positions"], after["positions"])) < .01,
            "robot moved before confirmation")
    result = session.confirm(corrected["confirmation"])
    require(result["status"] == "completed", "clarified inspection failed")
    return {"ambiguous": ambiguous, "corrected": corrected, "result": result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--audio", type=Path)
    parser.add_argument("--suite", choices=("all", "boundary", "stop", "fault", "voice", "sequence", "dialogue"), default="all")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.suite in {"all", "voice"} and args.audio is None:
        parser.error("voice acceptance requires --audio with the red-cube-to-blue-target command")
    backend = SSHBackend(args.session)
    audit = AuditLogger(args.output.with_suffix(".audit.jsonl"))
    load_local_api_key()
    report = {"session": args.session, "started_at_s": time.time(), "tests": {}}
    cases = {
        "boundary": lambda: boundary(backend),
        "stop": lambda: live_stop(backend),
        "fault": lambda: fault(backend),
        "dialogue": lambda: dialogue(backend, audit),
        "voice": lambda: {"audio": (speech := upload_and_transcribe(backend, args.audio)),
                          "task": task(backend, audit, speech["text"])},
        "sequence": lambda: task(backend, audit,
            "Inspect the red cube. Then pick the red cube up, place it on the yellow target, and inspect it again. Use separate pick and place skills."),
    }
    try:
        for name, run in cases.items():
            if args.suite not in {"all", name}:
                continue
            print(f"Running {name} acceptance", flush=True)
            evidence = run()
            report["tests"][name] = {"status": "passed", "evidence": evidence}
            print(f"PASS {name}", flush=True)
        report["status"] = "passed"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = str(error)
        print(f"FAIL {error}", flush=True)
    finally:
        report["finished_at_s"] = time.time()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if report["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
