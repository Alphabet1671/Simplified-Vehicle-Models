# IEM24 Murphy Validation Plan

> **For agentic workers:** Execute inline; steps use checkbox syntax.

**Goal:** Store a separate real-IEM24 Murphy model, simulate one MIS endurance lap, and compare it with the measured first-stint velocity, acceleration, and yaw-rate traces from `~/Desktop/0615CompEndurance.mat`.

**Architecture:** Clone the existing Murphy component graph under explicit `iem24_murphy_real` paths, changing only the provided real measurements and the APPS/controller logic. Condition the logged navigation-unit channels in `tools/iem24_telemetry.py`, segment the stint into laps by position closure, project every measured sample onto the signed closed MIS centreline, and compare both traces on one physical lap-distance axis.

**Tech Stack:** XML model graph, Python, NumPy, SciPy (io/signal/spatial), Matplotlib, unittest.

**Spec:** User-provided values: front motors disconnected; APPS 0–100% requests 0–10.5 N·m; regen disabled; mechanical brakes only; LLTD 42%; car 209 kg plus 78 kg driver; CL*A 3.5; CD*A 1.55; aerodynamic balance 50%; CGx 45% of wheelbase; CG height 0.300 m; first stint 105–938 s; nav-unit motion data are skewed and must be corrected; include motor torque (`Tq_RL_Req`, `Tq_RR_Req`) and pack voltage (`VPack`) in the comparison; take the pack starting voltage from the data as the model lap's initial condition.

## Global Constraints
- Keep all real-IEM24 XMLs separate from the existing placeholder/Murphy model files.
- Interpret CGx = 45% as measured from the front axle; convert to `front_mass_fraction = 0.55` because that parameter is the rearward CG fraction used for static front load.
- Use total vehicle mass 287 kg (209 kg car + 78 kg driver).
- Front wheel torque share is zero, rear wheels RL/RR are the only driven wheels, and motor count is two.
- The comparison must run on a track with a turn direction; the legacy V11 radius profile stores a magnitude only.
- Exclude the standing-start lap from flying-lap statistics, and state every value that was carried over rather than measured.

---

### Task 1: Real-IEM24 model graph

**Files:** created independent vehicle, aero, tire, battery, motor, powertrain, controller, transmission, drivetrain, and formulation XMLs under `models/`; modified controller compilation and transmission driven-wheel validation.

**Interfaces:** `controller-model/pedal-map` with `maximum_drive_motor_torque_nm=10.5` and `regen_enabled=0`; `compile_fixed_gear_transmission` now accepts any unique nonempty driven-wheel subset, and the real model declares only `rl, rr` with front torque fraction 0.

- [x] Add controller settings for the APPS torque cap and a regen on/off switch; disable regen so brake force goes directly to hydraulics.
- [x] Create the separate real vehicle graph with mass=287 kg, LLTD=.42, CG fraction .55, h=.300 m, CA=3.5, CDA=1.55, aero balance=.50, motor_count=2, APPS cap=10.5 N·m, and rear-only drive.
- [x] Compile the new formulation and verify the resolved model uses the real controller, two rear motors, and the specified values.

### Task 2: Telemetry conditioning

**Files:** create `tools/iem24_telemetry.py`, `tools/extract_iem24_telemetry.py`, `tests/test_iem24_telemetry.py`, and `docs/validation/iem24-murphy/measured_stint.csv` plus `telemetry_extraction.json`.

**Interfaces:** `load_nav_channels`, `estimate_nav_skew_deg`, `rotate_to_body_frame`, `zero_phase_lowpass`, `integrate_position`, `travel_distance`, `segment_laps`, `lap_distance_axis`.

- [x] Read the `Time`/`Value`/`Units` channel structures straight from the MATLAB 5.0 export; the log holds 94 channels on one 100 Hz base.
- [x] Estimate the nav-unit yaw skew from straight high-speed running and rotate `VelX`/`VelY` and `AccelX`/`AccelY` into the body frame. The estimate is 8.55°, and the correction lifts the longitudinal-accelerometer fit from 0.816 to 0.984 correlation.
- [x] Segment the stint into laps by position closure; flag the lap that starts from rest.
- [x] Export the conditioned 20 Hz stint channels, including integrated position and pack voltage, with a JSON quality-control block.
- [x] Read `VPack` and report the pack voltage at rest, at launch, at every lap boundary, and through the stint; compile the resting value into the model battery as its initial bus voltage.

### Task 3: MIS run and trace comparison

**Files:** create `tools/compare_iem24_murphy_stint.py`, modify `tools/run_iem24_murphy_lap.py`, create `tests/test_iem24_trace_comparison.py`, and write `docs/validation/iem24-murphy/{comparison.csv,comparison_summary.json,comparison.png,alignment.png,report.md}`.

**Interface:** the run tool takes a `--track` argument and refuses a track without a turn direction; the comparison tool fits a constant translation from the dead-reckoned measured path onto the centreline, projects each sample to a track distance, folds the flying laps onto a shared grid, and writes per-channel metrics plus a Markdown report with every figure embedded.

- [x] Confirm the telemetry is available and parse it directly from the MATLAB export.
- [x] Run the real-model lap on `models/tracks/2024_michigan_endurance_closed.xml`, the signed centreline rebuilt from the same log.
- [x] Put measured and simulated traces on one physical axis by projection rather than an inferred phase offset.
- [x] Compare speed, longitudinal acceleration, lateral acceleration, yaw rate, per-motor torque request, and pack terminal voltage; embed both figures in the report.
- [x] Run targeted tests and `git diff --check`.
