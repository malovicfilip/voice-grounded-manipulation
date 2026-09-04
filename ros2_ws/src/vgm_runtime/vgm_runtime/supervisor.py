"""Fail-closed execution state machine with cancellation and deadlines."""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Protocol

from .config import load_json_config
from .types import ExecutionResult, MotionPrimitive, TaskPlan


class ExecutionBackend(Protocol):
    def execute_primitive(self, primitive: MotionPrimitive) -> None: ...

    def cancel_all(self) -> None: ...


class SupervisorState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"
    FAULTED = "faulted"


class ExecutionSupervisor:
    """Serialize execution and guarantee cancellation after stops or faults."""

    def __init__(self, policy=None, *, clock=time.monotonic) -> None:
        self.policy = dict(policy or load_json_config("safety_policy.json"))
        self.clock = clock
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._state = SupervisorState.IDLE

    @property
    def state(self) -> SupervisorState:
        with self._lock:
            return self._state

    def request_stop(self, backend: ExecutionBackend) -> None:
        """Request an idempotent stop and synchronously signal the backend."""
        self._stop_requested.set()
        with self._lock:
            if self._state == SupervisorState.RUNNING:
                self._state = SupervisorState.STOPPING
        backend.cancel_all()

    def reset_fault(self) -> None:
        with self._lock:
            if self._state != SupervisorState.FAULTED:
                raise RuntimeError("reset_fault requires a faulted supervisor")
            self._state = SupervisorState.IDLE
            self._stop_requested.clear()

    def execute(self, plan: TaskPlan, backend: ExecutionBackend) -> ExecutionResult:
        if plan.skill == "stop":
            self.request_stop(backend)
            return ExecutionResult(plan.request_id, "stopped", 0, "stop", "stop applied")
        if plan.skill == "refuse":
            return ExecutionResult(plan.request_id, "refused", 0, "refuse", "no motion")

        with self._lock:
            if self._state != SupervisorState.IDLE:
                raise RuntimeError(f"supervisor is not idle: {self._state.value}")
            self._state = SupervisorState.RUNNING
            self._stop_requested.clear()

        completed = 0
        deadline = self.clock() + self.policy["maximum_execution_time_s"]
        try:
            for primitive in plan.primitives:
                if self._stop_requested.is_set():
                    return ExecutionResult(
                        plan.request_id,
                        "stopped",
                        completed,
                        "stop_requested",
                        "execution stopped before the next primitive",
                    )
                if self.clock() > deadline:
                    self.request_stop(backend)
                    with self._lock:
                        self._state = SupervisorState.FAULTED
                    return ExecutionResult(
                        plan.request_id,
                        "failed",
                        completed,
                        "execution_timeout",
                        "execution deadline exceeded",
                    )
                backend.execute_primitive(primitive)
                completed += 1
                if self._stop_requested.is_set():
                    return ExecutionResult(
                        plan.request_id,
                        "stopped",
                        completed,
                        "stop_requested",
                        "execution stopped after the current primitive",
                    )
            return ExecutionResult(
                plan.request_id,
                "succeeded",
                completed,
                "ok",
                "all deterministic primitives completed",
            )
        except Exception as error:
            backend.cancel_all()
            with self._lock:
                self._state = SupervisorState.FAULTED
            return ExecutionResult(
                plan.request_id,
                "failed",
                completed,
                "backend_fault",
                f"backend fault: {type(error).__name__}",
            )
        finally:
            with self._lock:
                if self._state in {SupervisorState.RUNNING, SupervisorState.STOPPING}:
                    self._state = SupervisorState.IDLE
            self._stop_requested.clear()
