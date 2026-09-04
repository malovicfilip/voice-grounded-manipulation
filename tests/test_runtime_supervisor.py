"""Fault, deadline, and stop acceptance tests for execution supervision."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "vgm_runtime"))

from vgm_runtime.supervisor import ExecutionSupervisor, SupervisorState  # noqa: E402
from vgm_runtime.types import MotionPrimitive, TaskPlan  # noqa: E402


class FakeBackend:
    def __init__(self, callback=None, fail_at=None):
        self.executed = []
        self.cancel_count = 0
        self.callback = callback
        self.fail_at = fail_at

    def execute_primitive(self, primitive):
        self.executed.append(primitive.kind)
        if self.callback:
            self.callback(len(self.executed))
        if self.fail_at == len(self.executed):
            raise RuntimeError("simulated backend fault")

    def cancel_all(self):
        self.cancel_count += 1


class ExecutionSupervisorTest(unittest.TestCase):
    def plan(self, skill="pick"):
        return TaskPlan(
            "req_supervisor",
            skill,
            tuple(MotionPrimitive(f"step_{index}") for index in range(3)),
        )

    def test_success_executes_every_primitive(self):
        supervisor = ExecutionSupervisor()
        backend = FakeBackend()
        result = supervisor.execute(self.plan(), backend)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.completed_primitives, 3)
        self.assertEqual(supervisor.state, SupervisorState.IDLE)

    def test_stop_prevents_all_following_primitives(self):
        supervisor = ExecutionSupervisor()
        backend = FakeBackend()
        backend.callback = lambda count: supervisor.request_stop(backend) if count == 1 else None
        result = supervisor.execute(self.plan(), backend)
        self.assertEqual(result.status, "stopped")
        self.assertEqual(backend.executed, ["step_0"])
        self.assertEqual(backend.cancel_count, 1)

    def test_backend_fault_cancels_and_latches_fault(self):
        supervisor = ExecutionSupervisor()
        backend = FakeBackend(fail_at=2)
        result = supervisor.execute(self.plan(), backend)
        self.assertEqual(result.code, "backend_fault")
        self.assertEqual(backend.executed, ["step_0", "step_1"])
        self.assertEqual(backend.cancel_count, 1)
        self.assertEqual(supervisor.state, SupervisorState.FAULTED)
        supervisor.reset_fault()
        self.assertEqual(supervisor.state, SupervisorState.IDLE)

    def test_timeout_cancels_before_next_primitive(self):
        ticks = iter((0.0, 0.0, 125.0))
        supervisor = ExecutionSupervisor(clock=lambda: next(ticks))
        backend = FakeBackend()
        result = supervisor.execute(self.plan(), backend)
        self.assertEqual(result.code, "execution_timeout")
        self.assertEqual(backend.executed, ["step_0"])
        self.assertEqual(backend.cancel_count, 1)

    def test_stop_plan_never_executes_a_motion_primitive(self):
        supervisor = ExecutionSupervisor()
        backend = FakeBackend()
        result = supervisor.execute(TaskPlan("stop_1", "stop", ()), backend)
        self.assertEqual(result.status, "stopped")
        self.assertEqual(backend.executed, [])
        self.assertEqual(backend.cancel_count, 1)


if __name__ == "__main__":
    unittest.main()
