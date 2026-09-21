# Inclinometer diagnosis and simulation

**Goal:** Explain the 0918 PD/inclinometer comparison with traceable evidence and add optional closed-loop sway experiments to the existing simulator.

**User acceptance clarified:** Reduce sway during motion, or match pure PD; zero sway is not required. Primary comparisons must use the same travel path/initial state and must not improve RMS by hiding a worse peak. Report position-path departure and arrival time separately. Hold experiments are secondary diagnostics.

**Architecture:** Preserve the existing controller and hardware settings. An optional plant adds two driven pendulums to the existing trolley velocity servos. A separate measurement channel represents ideal rope angle or a parameterized gripper-mounted angle, including static B2 calibration readings, scale/sign, filtering, and delay. These are sensitivity experiments, not an identified double-pendulum model.

**Tech stack:** Python, existing NumPy/Matplotlib/pytest; rosbags only for offline recording inspection.

## Constraints

- Source recordings remain read-only. 001 has no PLC feedback; only 002 supports height/gripper phase matching.
- Match operating phase and height, not recording timestamps or whole-recording duration.
- Reject invalid PLC sentinel samples; topic names alone do not establish command semantics.
- Static loaded offsets are evidence; sensor delay, attitude transfer and actuator dynamics remain hypotheses.
- Default simulator behavior and PLC control settings remain unchanged.
- Pendulum truth must be independent of the synthetic measurement. Arrival must not erase physical sway or force the trolley to its target in the optional plant.
- Simulation comparisons run for equal durations, including after the position controller declares arrival.

## Implementation

1. Add behavioral tests in `tests/test_sway_simulation.py`: free-pendulum period/energy, motion after axis lock, delayed measurement, loaded static bias, ideal-feedback damping versus wrong-direction amplification, CLI enable/disable behavior.
2. Add `sway_simulation.py`: `SwaySimulationConfig`, `SimulatedInclinometer`, `SwayCranePlant`, output plots/JSON. Integrate through one plant update method and optional fixed simulation duration in `crane_model.py`; connect in `main.run_simulation`.
3. Add `--sim-sway ideal|inclinometer`, measurement parameters, fixed duration and output flags to `main.py`; support explicit `--no-anti-sway`. Reject simulation flags with a real PLC target before connection.
4. Add `script/compare_sway_sim.py` for identical-condition travel: PD, current site gating, ideal feedback, static-bias, delayed and reversed-signal cases. Explicitly check RMS and peak against PD with zero allowed increase by default. Preserve the selected controller settings in the output; distinguish experimental travel/hold settings from site defaults. Keep `--scenario hold` for isolating measurement effects.
5. Save phase-matched evidence, static offsets, limitations and reproducible commands in `doc/inclinometer_diagnosis_0918.md`. Include the effect of shared axis gating and the difference between measured angle and rope angle.

## Verification

- Run new tests first and inspect the expected failures before implementation.
- Run new physical/sensor tests and existing control tests after integration.
- Run the comparison script and inspect the generated plots and numerical metrics.
- Run the complete pytest suite once after the final functional changes; review `git diff --check`.
