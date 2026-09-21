# Terminal sway implementation plan

> **For agentic workers:** Execute the approved plan inline with executing-plans and test-driven-development. Preserve the existing dev_incl workspace and uncommitted changes. No hardware operations or commits are part of this delivery.

**Goal:** Reduce residual sway at the stop, with trustworthy per-axis feedback and honest stopped-plant comparisons.

**Architecture:** Natural PD motion qualifies each angle channel against a forced pendulum. A terminal policy ramps eligible feedback before arrival and bounds the final settling stage. The simulation continues with zero commands after completion.

**Tech Stack:** Python, NumPy, existing control loop and physical pendulum, pytest, Matplotlib.

## Global constraints

- Follow `doc/site_context.md`; original site data remains read-only.
- Sensor on gripper frame, effective length H−Z; no automatic sign calibration.
- Terminal residual and stable-ready time are primary. Keep individual-axis and position checks.
- Real PLC and external writes are outside this task.

## Task 1: Qualify feedback from natural motion

Files: `sway_qualification.py`, `tests/test_sway_qualification.py`.

Interface: `SwayFeedbackQualifier.update(theta_x, theta_y, x, y, length, dt)` returns two booleans; `reset()` clears evidence; `snapshot()` reports diagnostics.

- [x] Write failing behavioral tests using the independent plant: positive transfer qualifies, a reversed axis and 1 s delay never qualify, motionless free swing is unidentifiable, gaps invalidate evidence.
- [x] Integrate forced and homogeneous states using support displacement; fit constant/trend/free modes plus forced angle/quadrature. Require independent excitation, positive transfer with limited phase, fit and held-out prediction quality, and successive passes.
- [x] Freeze a passed predictor, revoke on sustained residual mismatch, and run qualification tests.

## Task 2: Terminal policy and runtime integration

Files: `anti_sway_runtime.py`, `crane_model.py`, `operation_scheduler.py`, `site_config.py`, `site_config.json`, `tests/test_terminal_sway.py`.

- [x] Add failing tests: unqualified correction stays zero, correct-transfer feedback produces real damping, cargo restriction survives, safe-height equal-Z transport can intervene. No field sign is flipped.
- [x] Add `anti_sway_terminal_enabled` config and optional safe-height context. Per-axis terminal distance uses `max(approach_pos, abs(v)*period + v²/(2a))` with speed capped at its configured limit; use a bounded blend to establish correction.
- [x] Keep qualifier and growth checks in the common runtime. Observation continues when output is prohibited. Never let recenter bypass per-axis qualification.
- [x] Separate position arrival from terminal completion; finite hold uses a full-period PP and position/speed tolerances. Missing evidence falls back to PD completion; timeout is explicit.
- [x] Run runtime, scheduler, and simulation regression tests. Existing tests of legacy behavior explicitly select legacy policy.

## Task 3: Stopped-plant comparison and field evidence

Files: `script/compare_terminal_sway.py`, `sway_simulation.py`, site documentation.

- [x] Compare real closed-loop commands, with zero commands after the controller finishes; report both first arrival and stable completion.
- [x] Export per-axis plots and JSON for ideal, bias, delayed, reversed and variable-height cases. Include qualification-disabled prior policies and PD controls with the same physical parameters; do not mislabel those as exact historical ffe67ac.
- [x] Run fixed-input field checks only as qualification diagnostics; do not call them new field motion.
- [x] Run full pytest, `git diff --check`, site data fingerprints, and independent code review.
- [x] Update persistent field context and report actual passes and remaining model limits.

## Verified delivery

- 263 tests pass (100.77 s), including gap invalidation, withdrawal position/velocity recovery, real-mode measured-state checks despite snapped arrival display, and three causal field-sampling checks.
- 9 sensor/plant scenarios pass common-absolute-time per-axis stopped residual and final position checks.
- Reproducible 36-case model sweep: 30 active, 6 PD fallbacks, 0 residual/position failures. Six do not reach offline stable-ready; one online settle timeout later becomes stable in the model. Not all-online-settled.
- Fixed field inputs: six selected 002 windows qualify neither channel under explicitly labelled encoder X/-map Y proxies. Field active-damping improvement remains unverified.
- Independent review completed; identified observation-gap and withdrawal recovery defects fixed and regression-tested. Evidence script and final real-mode measured-state recovery fix reviewed separately.
- Raw fingerprints unchanged; no PLC operation, commit, push or branch integration. Keep the existing workspace as approved.
- Scheduler retains a separate settle wait and legacy timeout policy; controller stop and offline stable-ready are not full scheduler-cycle time.
