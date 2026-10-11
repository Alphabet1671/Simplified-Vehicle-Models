# Point-Mass MIS Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the XML-configured point-mass vehicle model and live-limit lap solver, then validate flying-lap and standing-start results on the legacy 2024 Michigan Endurance track.

**Architecture:** XML files declare the tire, aero, drivetrain, vehicle assembly, and point-mass formulation. Python compiles them into immutable parameter dataclasses. The point-mass model calculates four-wheel normal loads and live combined-force limits, while a spatial forward/backward solver generates flying and standing-start speed profiles. The legacy MATLAB track is converted once into a repository CSV so runtime code remains Python-standard-library plus NumPy.

**Tech Stack:** Python 3.12, NumPy, standard-library XML/CSV/dataclasses/unittest.

**Spec:** `docs/project-master-plan.md`

## Global Constraints

- Use SI units for all calculations.
- Use lowercase `snake_case` for Python modules, functions, variables, parameters, and attributes.
- Use `CapWords` for classes and `UPPER_SNAKE_CASE` for constants.
- All source comments and public docstrings must follow ASD-STE100 Simplified Technical English.
- Core runtime dependencies are Python standard library and NumPy only.
- XML selects and parameterizes registered behavior; Python implements behavior.
- The point-mass formulation consumes a generic tire force envelope.
- Wheel order is `(FL, FR, RL, RR)`.
- Do not modify either legacy MATLAB repository.
- Use the legacy Murphy 2024 parameters without tuning them to force a target lap time.

---

### Task 1: Package, XML Models, and Configuration Compiler

**Files:**
- Create: `pyproject.toml`
- Create: `src/simplified_vehicle_models/__init__.py`
- Create: `src/simplified_vehicle_models/parameters.py`
- Create: `src/simplified_vehicle_models/xml_loader.py`
- Create: `models/tires/correlated_comp_2024.xml`
- Create: `models/aero/murphy_2024.xml`
- Create: `models/drivetrains/murphy_4wd.xml`
- Create: `models/vehicles/murphy_2024.xml`
- Create: `models/formulations/murphy_point_mass.xml`
- Create: `tests/test_xml_loader.py`

**Interfaces:**
- Produces immutable `TireEnvelopeParameters`, `AeroParameters`, `DrivetrainParameters`, and `PointMassVehicleParameters` dataclasses.
- Produces `load_point_mass_vehicle(path: str | Path) -> PointMassVehicleParameters`.
- XML references resolve relative to the XML file containing the reference.

- [ ] **Step 1: Write tests for XML loading**

Test that the formulation loads these exact legacy values: total mass `299.0 kg`, wheelbase `1.535 m`, track width `1.016 m`, CG height `0.3 m`, front mass fraction `0.47`, LLTD `0.44`, tire scales `0.25` and `0.4745`, tire load slope `-0.0018`, tire intercept `2.9`, effective radius `0.193675 m`, downforce area `4.2 m²`, drag area `1.8 m²`, aero balance `0.44`, air density `1.22 kg/m³`, motor count `4`, gear ratio `10.9`, peak motor torque `21.834945614425717 N·m`, power limit `80000 W`, efficiency `0.95`, maximum motor speed `20000 rpm`, and brake bias `0.65`.

Also test rejection of an unknown model type, a missing parameter, and a component-reference cycle.

- [ ] **Step 2: Run the tests and verify failure**

Run: `PYTHONPATH=src python -m unittest tests.test_xml_loader -v`
Expected: FAIL because package modules and model XML files do not exist.

- [ ] **Step 3: Implement immutable parameter types and XML compilation**

Use `xml.etree.ElementTree`. Accept only registered type/subtype pairs needed by this milestone. Parse floats strictly, reject duplicate parameter names, reject missing required parameters, and report the reference chain for cycles.

- [ ] **Step 4: Run the configuration tests**

Run: `PYTHONPATH=src python -m unittest tests.test_xml_loader -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml src models tests/test_xml_loader.py
git commit -m "feat: add XML point-mass model configuration"
```

### Task 2: Legacy MIS Track Data and Loader

**Files:**
- Create: `models/tracks/2024_michigan_endurance_v11.csv`
- Create: `models/tracks/2024_michigan_endurance_v11.xml`
- Create: `src/simplified_vehicle_models/tracks.py`
- Create: `tests/test_tracks.py`

**Interfaces:**
- Produces immutable `TrackProfile(distance, segment_length, radius, curvature, is_closed, title)` arrays.
- Produces `load_track(path: str | Path) -> TrackProfile` for a track XML that references a CSV.
- CSV contains the exact `D`, `stepD`, and `softR` arrays from the legacy MATLAB `PATH` structure. The runtime uses `D[-1]` as lap length and the first 1406 `stepD` values as physical segments; the final stored `stepD` is a duplicated source artifact.

- [ ] **Step 1: Export the legacy track without changing its values**

Read `/Users/alphabet1671/Dev/lap-sim-2025/Models/Legacy Models/Tracks/2024_Michigan_Endurance_V11.mat` as MATLAB v5 data. Export columns `distance`, `segment_length`, and `radius` in their original row order. Record the legacy source path and title in the XML metadata.

- [ ] **Step 2: Write track-loader tests**

Assert `1407` nodes, closed-track status, title `2024 Michigan Endurance V11`, lap length and physical-segment sum `1014.32575 m`, minimum radius approximately `5.210433541 m`, all physical segment lengths positive, and `curvature == 1 / radius`. Verify that the loader excludes the duplicated final `stepD` value from integration.

- [ ] **Step 3: Run the tests and verify failure**

Run: `PYTHONPATH=src python -m unittest tests.test_tracks -v`
Expected: FAIL because the loader does not exist.

- [ ] **Step 4: Implement the track loader and validation**

Use `csv` and NumPy. Reject nonfinite data, nonpositive segment lengths/radii, nonmonotonic cumulative distance, inconsistent row counts, and unsupported track subtypes.

- [ ] **Step 5: Run the track tests**

Run: `PYTHONPATH=src python -m unittest tests.test_tracks -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add models/tracks src/simplified_vehicle_models/tracks.py tests/test_tracks.py
git commit -m "feat: add legacy MIS track profile"
```

### Task 3: Point-Mass Vehicle Physics and Live Limits

**Files:**
- Create: `src/simplified_vehicle_models/point_mass.py`
- Create: `tests/test_point_mass.py`

**Interfaces:**
- Produces `WheelLoads`, `AccelerationLimits`, and diagnostic active-constraint data.
- Produces `normal_loads(vehicle, speed, longitudinal_acceleration, lateral_acceleration) -> np.ndarray`.
- Produces `tire_force_capacities(vehicle, normal_loads) -> tuple[np.ndarray, np.ndarray]`.
- Produces `maximum_corner_speed(vehicle, radius, maximum_speed=60.0) -> float`.
- Produces `acceleration_limits(vehicle, speed, curvature) -> AccelerationLimits`.

- [ ] **Step 1: Write physics tests**

Cover static wheel-load sum, aero load scaling with speed squared, longitudinal load-transfer direction, lateral load-transfer conservation, decreasing effective friction coefficient with increasing load, zero capacity at zero load, finite straight-line acceleration/braking at zero and nonzero speed, power limiting at high speed, and decreasing corner speed with decreasing radius.

Use force balance and inequality assertions rather than snapshotting implementation details.

- [ ] **Step 2: Run the tests and verify failure**

Run: `PYTHONPATH=src python -m unittest tests.test_point_mass -v`
Expected: FAIL because `point_mass.py` does not exist.

- [ ] **Step 3: Implement the four-wheel point-mass physics**

Use:

```text
mu_x(Fz) = longitudinal_mu_scale * (load_sensitivity_slope * Fz + load_sensitivity_intercept)
mu_y(Fz) = lateral_mu_scale * (load_sensitivity_slope * Fz + load_sensitivity_intercept)
```

Clip capacities to zero for unloaded wheels. Allocate lateral force in proportion to each tire's available lateral capacity. Calculate remaining longitudinal capacity with an elliptical combined-force relationship. Apply AWD tire capacity, motor torque, motor-speed, total-power, drag, and fixed brake-bias limits. Solve acceleration and braking limits as bounded scalar equilibrium problems because load transfer depends on acceleration.

- [ ] **Step 4: Run the point-mass tests**

Run: `PYTHONPATH=src python -m unittest tests.test_point_mass -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/point_mass.py tests/test_point_mass.py
git commit -m "feat: calculate live point-mass acceleration limits"
```

### Task 4: Flying-Lap and Standing-Start Solvers

**Files:**
- Create: `src/simplified_vehicle_models/lap_solver.py`
- Create: `tests/test_lap_solver.py`

**Interfaces:**
- Produces `LapMode` with `FLYING` and `STANDING_START`.
- Produces immutable `LapResult` containing speed, elapsed time, longitudinal/lateral acceleration, throttle, brake, lap time, convergence status, and iteration count.
- Produces `solve_lap(vehicle, track, mode, tolerance=1e-6, maximum_iterations=200) -> LapResult`.

- [ ] **Step 1: Write synthetic-track solver tests**

Test a constant-radius closed track, a straight standing start, a straight-corner-straight profile, flying-lap periodicity, standing-start zero initial speed, nonnegative finite speeds/times, mutually exclusive throttle/brake, and explicit nonconvergence status when the iteration limit is intentionally insufficient.

- [ ] **Step 2: Run the tests and verify failure**

Run: `PYTHONPATH=src python -m unittest tests.test_lap_solver -v`
Expected: FAIL because the solver does not exist.

- [ ] **Step 3: Implement spatial forward/backward reachability**

Calculate live corner-speed ceilings. Treat the first and last track rows as the same start/finish geometry with 1406 physical segments between them. For flying laps, enforce `speed[0] == speed[-1]` and alternate forward acceleration and backward braking sweeps until the maximum speed change is below tolerance. For standing starts, enforce zero initial speed and leave terminal speed free subject to the final corner. Evaluate live acceleration limits at each transition. Recover controls from required longitudinal acceleration. Integrate segment time with `2 * ds / (v_start + v_end)`.

- [ ] **Step 4: Run the solver tests**

Run: `PYTHONPATH=src python -m unittest tests.test_lap_solver -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/lap_solver.py tests/test_lap_solver.py
git commit -m "feat: solve flying and standing point-mass laps"
```

### Task 5: MIS Validation Command and Regression Test

**Files:**
- Create: `examples/run_mis_point_mass.py`
- Create: `tests/test_mis_regression.py`
- Create: `docs/validation/point-mass-mis.md`

**Interfaces:**
- The example prints one JSON object containing flying and standing-start summary metrics.
- The regression test loads repository XML files and runs both modes end to end.

- [ ] **Step 1: Write the end-to-end regression test**

Require both solves to converge. Require finite lap times between `50 s` and `100 s`, standing-start time greater than flying-lap time, a standing-start penalty below `15 s`, maximum speed below the motor-speed limit, and no tire-utilization or force-balance violation above numerical tolerance.

- [ ] **Step 2: Run the regression and verify failure**

Run: `PYTHONPATH=src python -m unittest tests.test_mis_regression -v`
Expected: FAIL because the example and validation integration do not exist.

- [ ] **Step 3: Implement the example and validation report**

Run both modes with the unchanged legacy parameters. Record track length, lap times, average speeds, maximum speeds, convergence iterations, and comparison against the measured Lap 8 trace span of approximately `70.30 s`, the cited fastest-lap reference of `69.98 s`, and the legacy workbook's broader event range. Explain that these are measured/event references, not outputs from the unfinished `lap-sim-2025` MATLAB solver.

- [ ] **Step 4: Run all tests and the example**

Run:

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python examples/run_mis_point_mass.py
```

Expected: all tests PASS and the example reports plausible finite results for both modes.

- [ ] **Step 5: Commit**

```bash
git add examples tests/test_mis_regression.py docs/validation/point-mass-mis.md
git commit -m "test: validate point-mass laps on MIS track"
```
