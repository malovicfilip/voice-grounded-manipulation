# Safety hardening: implementation and validation

These changes preserve the authority boundary:

```text
language → ID-only skill/task proposal → JSON Schema → state/safety checks
         → fresh measured XYZ grounding → deterministic coordinator
         → MoveIt → execution → two-frame perception verification
```

The LLM receives object/target IDs, not scene coordinates. It never outputs
joint angles, Cartesian commands, velocities, motor commands, or trajectories.
The operator must still confirm a non-stop task before execution.

## Changes

1. **Released-object collisions.** `safe_pick_and_place.cpp` waits for confirmed
   detachment, requires the released object to exist in the collision world,
   and synchronously applies a conservative world-aligned box spanning its
   transformed release pose and gravity settling toward the measured target
   support height. Retreat is planned only afterward. Missing world geometry,
   a failed update, or a failed retreat plan aborts; the object is never removed
   to make retreat succeed. The box accounts for both object and shape poses,
   as specified by the [Jazzy CollisionObject message](https://docs.ros.org/en/ros2_packages/jazzy/api/moveit_msgs/msg/CollisionObject.html).
2. **Measured XYZ.** Visible horizontal cube faces are offset inward by half
   the known cube size; complete vertical faces estimate center height from
   measured extents. Ambiguous fragments are refused. Expected positions guide
   color-region association in XY but never supply the output XYZ. This remains
   a configured, upright-cube estimator, not arbitrary-shape 6D pose estimation.
3. **Perceived targets.** `GroundedScene.targets` contains immutable, timestamped
   world-frame observations. Color cues plus planar circular geometry identify
   configured markers. Circle fitting allows central occlusion while requiring
   broad angular support. Authored target XYZ is never a runtime fallback.
   Missing, stale, uncertain, ambiguous or out-of-workspace targets refuse a
   placement. Target positions participate in scene revisions, confirmation
   drift checks, coordinator plans and deterministic C++ launch parameters.
4. **Robot state.** `SkillValidator` checks state before minting or revalidating
   a skill. A held object permits only a matching `place` (or non-motion
   stop/refusal). Another pick, opening, closing, or named-pose move is refused.
   Standalone closing is refused even with an empty hand because it lacks a
   validated grasp context; the deterministic pick sequence still closes.
   Downstream task/backend/C++ checks remain in place.
5. **Canonical configuration.** `config/safety_policy.json` owns safety limits
   and references the authored geometry file. IDs, cube sizes and table
   geometry are derived from that file rather than copied into policy or C++.
   CMake runs `generate_safety_header.py` to generate `vgm_safety_config.hpp`
   in the build directory for both executors. Input edits trigger regeneration.
   The launch-time effective-policy digest must match the compiled digest;
   changing policy or using a mismatched `VGM_CONFIG_DIRECTORY` requires a
   rebuild. Runtime target poses come from the fresh scene, not this header.
6. **Actual schema enforcement.** `jsonschema.Draft202012Validator` executes
   `robot_skill.schema.json` on raw proposals first. Skill-specific field
   combinations live in the schema; task-step schemas are derived from it.
   Only after syntax passes do semantic, state, freshness and workspace checks
   run. Provider-side structured output is an aid, never the safety boundary.
7. **Deterministic STOP.** Whole-word stop/cancel/abort/halt/freeze recognition
   covers phrases including “stop it”, “please stop”, “stop the robot” and
   “emergency stop”. STOP bypasses model calls, captures, confirmation and
   ordinary task-state gating. Cancellation invalidates pending work and late
   responses cannot restore it. Whole-word matching deliberately favors stopping
   even in a longer/negated sentence; substrings such as “stopper” do not match.
   Speech still incurs recording/Whisper latency. Use the browser STOP button
   or terminal Ctrl+C during execution. This is not a safety-rated hardware
   emergency-stop system.
8. **Structural clarification.** The original request and question/answer turns
   are stored separately, capped at eight turns/10,000 serialized characters,
   and rendered afresh for each model call. Cancel/stop clears clarification;
   previous rendered prompts are never recursively concatenated.

## Placement success criteria

The current canonical policy requires all of:

- Fresh world-frame object and target observations with confidence ≥ 0.75.
- XY center error ≤ 0.06 m relative to the measured target center.
- Z error ≤ 0.015 m relative to measured target surface Z + half cube size.
- No attached object in either end-of-task observation.
- Two object observations at least 0.5 seconds apart, with measured 3D speed
  ≤ 0.01 m/s and both within the normal scene-freshness window.
- In reusable sessions, the target must not have moved beyond the existing
  0.01 m drift tolerance during placement.

This is evidence consistent with resting, not proof of continuous zero velocity
or contact-force sensing. A cube suspended over a target, still attached,
moving, or missing from depth cannot pass merely because its XY is correct.
The reusable backend stores the measured final XYZ as its association hint,
not the commanded destination. The older single-run demo now also captures a
rest-start frame and explicit ROS attachment-state evidence.

## Verification performed locally

No dependency installation, cloud deployment, instance restart, port exposure,
paid API call, or robot motion was performed for this change.

Run the offline suite from the repository root:

```bash
python3 -m unittest discover -s tests
```

It tests the existing confirmation/replay/fault behavior plus measured target
resolution, absent/stale/moved targets, measured cube height, two-frame XYZ
outcomes, carrying preconditions, raw-schema enforcement, structural dialogue,
and typed/transcribed STOP variants (including while a model call is pending).
One browser test binds a temporary loopback-only socket. The generated C++
header is also syntax-checked with the installed `g++`, and source-contract
tests prohibit collision-object removal before retreat. Python compilation,
shell syntax and `git diff --check` were checked separately.

Saved RGB-D replay (not a new simulator run) recovered six cubes and both
target markers from `full-demo-video-v1/capture-103f53f1a04c47e3`.
The measured target surfaces were approximately 0.752 m high. Another saved
capture retained a red-cube center at approximately 0.900 m instead of forcing
0.775 m. Recordings remain ignored and are not required by the portable tests.

## Deployment gate — required before claiming live acceptance

### September 6 evening live regression: not accepted

Session `safety-acceptance-v1` ran the deployed safety runtime. The corrected
workstation acceptance runner passed boundary, live stop, and fault recovery
with its deterministic provider (no LLM calls). Boundary rejection produced
zero sampled joint change. The stop fixture observed 0.738 rad of joint change
before cancellation, no following descent, and explicit recovery.

The `inspect → pick → place → inspect` sequence **failed** after release, before
retreat. MoveIt `CheckStartStateCollision` reported `panda_leftfinger - red_cube`
and `panda_link0 - table`. The runtime cancelled and reported stationary with
no attached object. No collision objects were removed to bypass the failure.
The retained red-cube world box was approximately 0.05028 × 0.05036 × 0.06598 m;
this includes the conservative settling envelope. A later RGB-D observation
did not see the red cube or yellow target, so placement was not verified.

This blocks acceptance: inspect the gripper/release collision geometry and
settling model, then rerun in a fresh instrumented session. Do not assume a
missing object was correctly placed, relax collision checking, or promote
the sequence result to a pass. The other new live fixtures below remain pending.

Reports remain in ignored `isaac_sim/_output/safety-{boundary-20260906-v2,
stop-20260906,fault-20260906,sequence-20260906}.json`. Failed and corrected
runner results are preserved separately. The workstation runner now tests
current refusal messages, rejects unrelated transport failures, uses one
capture for its fault-latch check, and cancels if the stop fixture raises.
The updated local suite passed 124 tests, including three runner regressions.

### Release-feedback correction

Follow-up inspection of the stopped scene returned `valid: true` from MoveIt's
state-validity service with both fingers at 0.0399 m, without deleting or
resizing the released obstacle. This points to action-completion/state-feedback
timing at release rather than proving a persistent geometric overlap.

Commit `16b8fe9` adds a bounded open-hand feedback gate before detachment and
again before retreat. It reads the arm interface's robot state, compares the
hand's named-open variables within the canonical policy tolerance, and requires
a stable interval. Retreat explicitly uses that observed start state. Missing,
nonfinite, mismatched or timed-out feedback fails closed. Collision geometry,
collision checking and 3D outcome thresholds are unchanged. Both ROS packages
built on Brev and 125 local tests passed. A fresh live retry is required before
claiming this resolves the observed failure; perception after retreat remains
part of that acceptance, not an assumed success.

The initial WSL verification could not build the complete MoveIt nodes.
On September 6, 2026, both packages were successfully built and deployed on
Brev; 117 ROS-environment tests and four recording tests in the cached Isaac
image passed. A deliberately mismatched policy was rejected by the installed
binary with exit code 2 before robot setup. Full live manipulation acceptance
is still separate from build/deployment acceptance.

Deployment details:

- Safety implementation: `b984742`; dependency lock/import correction: `c8cdb1f`.
- The first startup (`safety-live-v1`) exposed an eager package import of the
  validator inside Isaac. Validator exports are now lazy: perception can load
  without `jsonschema`, while requesting validation without it still fails.
  A regression test and actual Isaac-image import smoke test verify this.
- Fresh replacement session: `safety-live-v2`. The failed session's logs remain
  in the ignored output directory; no manipulation was executed there.
- Passive live checks passed after camera warm-up: both controllers activated,
  a fresh RGB-D capture observed all six cubes and both target markers in XYZ,
  and robot state reported stationary, zero sampled joint drift, and no held
  object. Object confidence exceeded 0.81; target confidence exceeded 0.86.
  Windows opened the viewer and localhost review console; an external streaming
  signaling connection and GPU encoder activity were observed. These checks
  do not constitute live manipulation acceptance or visual user confirmation.
- Installed Python runtime files were compared with source. The actual C++
  executable embeds the matching effective-policy digest
  `48fe885554e9d1d89d3a5aaed0843860525da163c47e0c6c1b3a9c98d7e05840`.
- Existing cloud recording/streaming edits were preserved in the Git stash
  named `pre-safety-deploy-2026-09-06`, not overwritten or discarded. The
  resolved dependency diff also has a named stash backup before synchronization.
- Streaming host firewall was restored for the previously approved client IP
  only. No new cloud firewall ports or paid instances were created.
- The Windows guard was rearmed and verified for September 6 at 17:53:27
  Toronto time. This is a dated deployment record, not a reusable deadline:
  rearm it for future sessions and keep Windows powered and online.

1. Keep the Brev cost/shutdown guard in place. Stop or finish the old robot
   session before updating software; never mix an old binary with new policy.
2. Verify `jsonschema` is importable in the runtime environment. After explicit
   approval, the September 6 deployment added `jsonschema` 4.26.0 and its four
   required support packages to the Pixi manifest/lock and installed with
   `--locked`. Existing package versions and GPU drivers were unchanged.
3. Rebuild `vgm_runtime` and `vgm_moveit_demo` in the existing supported MoveIt
   environment. Verify the generated digest matches `policy_digest()` and that
   a deliberately mismatched digest is rejected before movement.
4. Start a fresh Isaac session. Confirm all needed cubes and both markers are
   observed in XYZ. Move a marker in the simulator: execution must follow its
   new measured pose only after a fresh reviewed command. Occlude a marker:
   placement must refuse, never use its spawn pose.
5. Execute one confirmed placement. Inspect MoveIt's collision world after
   detachment and during retreat: the placed cube must remain present. Add a
   blocking obstacle: planning must refuse rather than remove collision boxes.
6. Verify suspended/stacked-height, still-attached, moving-object, stale-depth
   and disappeared-object cases fail the outcome gate. Verify a settled cube
   passes with two fresh observations.
7. Exercise STOP during perception, parsing, confirmation, planning and motion.
   Verify no following primitive runs and explicit recovery is required.

Historical acceptance reports remain records of the earlier implementation;
they do not certify these new invariants. Old scene JSON without perceived
targets may still be read but cannot authorize a placement.
