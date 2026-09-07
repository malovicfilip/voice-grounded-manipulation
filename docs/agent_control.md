# Closed-loop LLM control

## Design goal

The LLM is allowed to control **what high-level capability the robot should attempt
next**, while deterministic software retains exclusive authority over **how physical
motion is produced and whether it succeeded**.

This layer is implemented in `vgm_runtime/agent.py`. It is additive: it does not
replace `SkillValidator`, the execution-time fresh-scene gate, the session backend,
MoveIt, STOP handling, or outcome verification.

## Authority model

```text
operator request
    │
    ▼
mission model
    │
    ▼
semantic authority envelope
    ├── goal summary
    ├── object IDs
    ├── target IDs
    ├── named poses
    ├── deterministic completion conditions
    └── maximum action count
    │
    ▼
OPERATOR CONFIRMATION
    │
    ▼
loop:
    fresh scene / robot state
        ↓
    semantic state (NO XYZ / joints / trajectories)
        ↓
    deterministic allowed-capability set
        ↓
    LLM chooses exactly one offered capability
        ↓
    capability + mission-scope validation
        ↓
    existing robot-skill schema + state validation
        ↓
    existing backend capture/revalidation/MoveIt path
        ↓
    sanitized semantic result
        └─────────────────────────── back to loop
```

The operator confirmation does not authorize arbitrary future model decisions. It
binds a finite semantic envelope and a machine-checkable completion contract. The
agent cannot add another object, target, named pose, or success condition after
confirmation, and it cannot exceed the reviewed action budget.

## Capabilities

`config/agent_policy.json` currently permits:

- `observe_workspace`
- `inspect`
- `pick`
- `place`
- `pick_and_place`
- `move_named_pose`
- `finish`
- `request_human_help`
- `stop`

Capabilities are not all offered at every step. `capability_options()` derives the
exact options from the fresh scene and robot holding state using the same canonical
scene/observation validity rules as `SkillValidator`. A stale scene exposes only
STOP/help. Object and target observations that are stale, below the confidence
threshold, in the wrong frame, non-finite, or outside the configured workspace are
not exposed as executable capabilities.

Examples:

- Empty gripper: inspect/pick/transfer/named-pose actions can be offered within the
  confirmed scope.
- A standalone `pick` is offered only when there is enough action budget left to
  place the object afterward.
- While holding an object: only allowlisted placement pairs, human-help, and STOP
  are offered. `finish`, another pick, inspect, workspace observation, and named-pose
  motion are absent.
- Occupied targets are removed from placement options.
- `finish` is offered only when **every confirmed completion condition is true**.

The model still has to return a strict action JSON document. Selecting a capability
or ID that was not offered is rejected before backend execution.

## Deterministic completion contract

A mission contains one or more operator-reviewed `success_conditions`. These are
semantic predicates, not coordinates:

- `object_on_target(object_id, target_id)` — evaluated from fresh grounded scene
  geometry and target occupancy; the object must not be held.
- `object_inspected(object_id)` — satisfied only after the validated `inspect`
  capability succeeds in this mission.
- `workspace_observed` — satisfied only after the no-motion observation capability
  executes in this mission.
- `named_pose_reached(pose_name)` — satisfied only after the validated named-pose
  execution succeeds in this mission.

The LLM proposes these conditions when forming the mission, but they are schema- and
scope-validated and become part of the exact confirmation hash. During execution the
supervisor evaluates them. The LLM cannot simply claim success: `finish` is absent
from `allowed_capabilities` until the completion contract is satisfied, and the
`finish` branch checks the contract again defensively.

## Model-visible semantic state

The LLM gets IDs and relations, not geometry. The semantic scene includes:

- scene revision and a freshness assertion produced by deterministic capture;
- confirmed objects and whether each is visible, safely grounded, or held;
- coarse confidence labels (`high`, `acceptable`, `low`, `unavailable`);
- confirmed targets, visibility, safe-grounding status, and semantic `occupied_by` identity;
- current held object ID;
- prior sanitized result;
- exact supervisor-generated capability options;
- remaining action budget;
- the confirmed completion conditions and their deterministic satisfied/not-satisfied state.

It intentionally excludes:

- `position_m`, X/Y/Z values, transforms, quaternions;
- joint state or joint targets;
- velocities, accelerations, torques, efforts or motor commands;
- trajectories or waypoints;
- collision-world edits;
- motion scaling and workspace limits.

`semantic_result()` also strips inspected object coordinates before a result is fed
back to the model.

## Physical execution remains on the old path

For `inspect`, `pick`, `place`, `pick_and_place`, and `move_named_pose`, the agent
action is converted to the original `robot_skill.schema.json` representation.
Then:

1. `SkillValidator` validates the skill against the decision-time fresh scene.
2. Existing state/target-occupancy preconditions run.
3. The backend captures again immediately before motion.
4. The existing execution gate rejects stale state or excess drift.
5. Deterministic geometry and policy offsets are used to invoke MoveIt.
6. Placement uses the existing two-frame outcome verification.

The agent cannot mint a `ValidatedSkill`, edit collision objects, or call a joint
controller directly.

## Confirmation semantics

Agent mode intentionally differs from planned-task mode.

**Planned-task confirmation** binds the operator to an exact ordered list of skills.
Existing drift checks compare execution against that confirmed task snapshot.

**Agent confirmation** binds the operator to semantic identities/poses and a finite
number of decisions. The agent is then allowed to react to fresh semantic changes
inside that reviewed envelope. Each individual executable decision still has a
fresh decision-time scene and the backend's immediate execution-time revalidation.

Use planned-task mode when exact pre-reviewed sequencing is the requirement. Use
agent mode when closed-loop high-level autonomy is the research/demo goal.

## Failure policy

The agent may adapt after successful observation/execution results, but **physical
execution failure is not an autonomous retry condition**. If an executable backend
result is not successful, or the execution call raises, the agent faults and requests
STOP. Explicit recovery remains required.

This avoids allowing the language model to expand its authority during uncertain
robot state.

Action-budget exhaustion is also not success. It yields `needs_human` and ends the
autonomous authority window.

## STOP

Typed/spoken STOP-family phrases are detected before the mission model. The browser
STOP button bypasses the normal work queue and sets the agent stop latch while also
requesting the backend's existing STOP path. Late model/capture responses cannot
restore a cancelled mission.

This remains software cancellation, not a certified hardware emergency stop.

## Tests

`tests/test_agent_control.py` verifies, among other cases:

- no robot operation before mission confirmation;
- semantic scope and action budget binding;
- closed-loop transfer followed by supervisor-gated `finish`;
- premature/dishonest `finish` is never offered and is rejected without motion;
- completion-condition identities are bound into operator confirmation;
- mandatory place-only state after standalone pick;
- model-visible state contains no positions/joint/trajectory/control fields;
- out-of-scope IDs cannot execute;
- nested direct-control injection is identified and rejected;
- excessive mission budget is refused;
- budget exhaustion is not success;
- physical failure stops without autonomous retry;
- STOP text bypasses the mission model;
- the API-free replay agent exercises the same capability gateway;
- stale scenes expose no executable capability;
- low-confidence objects/targets remain non-authoritative and cannot be offered
  for motion or satisfy completion predicates.

The agent layer has offline regression coverage, but it must not be represented as
live accepted until a fresh Isaac/MoveIt acceptance campaign is completed.
