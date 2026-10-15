# Drivetrain Pedal Controller Implementation Plan

> **For agentic workers:** Implement inline with tests. Steps use checkbox syntax.

**Goal:** Put accelerator and brake pedal interpretation in a dedicated drivetrain controller model while keeping the 7-DOF optimizer's accelerator input in TPS percent and brake input in physical pedal force.

**Architecture:** Add a controller parameter/component under the drivetrain assembly and a pure pedal-mapping module. The planar OCP keeps three controls but changes their units: TPS is 0–100%, brake is N at the pedal, steering remains radians. The node dynamics map TPS to the constant-torque motor request (then existing torque/power/speed constraints cap delivery), split regen using brake bias, and map force above the load-cell range to hydraulics.

**Tech Stack:** Python dataclasses, XML component models, CasADi, NumPy, unittest.

**Spec:** User request: TPS 0–100%; throttle requests 0-to-maximum motor torque at constant-torque shaft speed and actual delivery is capped by motor capability. Brake pedal first uses a 0–30 kgf load cell mapped to 0-to-−10 N·m motor regen torque; after load-cell saturation, remaining pedal force actuates hydraulic brakes. Regen axle split equals brake bias.

## Global Constraints
- Keep the optimizer-facing throttle in the physical range 0–100% and brake input in newtons at the pedal.
- Preserve the public three-control layout `[TPS_percent, brake_pedal_force_N, steering_rad]` and do not introduce controller-state dynamics.
- Interpret 10 N·m as per-motor shaft-torque request; cap actual regen by each motor's available voltage/speed/regen envelope.
- Distribute regen wheel torque by the assembly brake bias, equally within each axle (FL, FR, RL, RR).
- Use 30 kgf = 30 × 9.80665 N as the load-cell saturation threshold. Hydraulic pedal force is `max(total_pedal_force_N - threshold_N, 0)`.
- Leave the unrelated regenerative-comparison report and existing workspace edits intact.

---

### Task 1: Add configurable drivetrain pedal controller

**Files:** create `src/simplified_vehicle_models/drivetrain/controller.py`; create `models/controllers/placeholder_2024_pedal_controller.xml`; modify `src/simplified_vehicle_models/powertrain/assembly.py`, `models/drivetrains/murphy_2024_drivetrain.xml`, `src/simplified_vehicle_models/parameters/vehicle.py`, and compiler construction in `src/simplified_vehicle_models/formulations/planar_7dof.py`; test `tests/test_planar_ocp.py` or a new `tests/test_pedal_controller.py`.

**Interface:** `PedalControllerParameters(maximum_tps_percent, regen_pedal_force_limit, regen_motor_torque_limit)` with `map_throttle(tps_percent, constant_torque_nm, available_motor_torque_nm) -> (requested_nm, delivered_nm)` and `map_brake(pedal_force_n, available_regen_motor_torque_nm) -> BrakePedalCommand(regen_motor_torque_nm, hydraulic_pedal_force_n, load_cell_fraction, regen_torque_limited)`. Regen torque is signed negative. Validate finite inputs and documented ranges; saturate motor requests at capability.

- [x] Test 0/50/100% TPS, throttle torque cap, 0/15/30 kgf regen progression, exact hydraulic onset above 30 kgf, and regen capability clipping.
- [x] Add/register a closed-schema `controller-model/pedal-map` XML parser; compile it from optional drivetrain role `controller` and expose the compiled parameters on the planar vehicle. Preserve existing assemblies lacking that role with explicit conservative defaults.
- [x] Reference the placeholder controller from the placeholder drivetrain assembly and use it when compiling the 7-DOF vehicle.
- [x] Run controller/parser and formulation compile tests.

### Task 2: Use pedal units and brake-bias regen in the 7-DOF OCP

**Files:** modify `src/simplified_vehicle_models/solvers/planar_ocp.py`, `src/simplified_vehicle_models/solvers/planar_equilibrium.py`, and `tests/test_planar_ocp.py`.

**Interface:** Keep `CONTROL_COUNT=3`, with control rows `[TPS_percent, brake_pedal_force_N, steering_rad]`. Convert drive command with `TPS/100`; constrain actual wheel drive torque by voltage-scaled motor torque/power/speed bounds. The brake load-cell force commands regen motor torque up to 10 N·m per motor; use brake-bias wheel shares for regen. Hydraulic brake pedal force starts above the 30 kgf threshold; preserve the existing brake model's full-pedal torque reference and bound pedal force by the assembly's maximum pedal force.

- [x] Add OCP tests for control bounds/units, 65/35 regen brake-bias allocation for the placeholder, zero regen and hydraulics below threshold, regen at saturation, hydraulics only above threshold, per-wheel capability limits, and signed bus power.
- [x] Update cold-start/equilibrium seeds, projection, overlap constraints, result documentation, and motor/electrical residuals to use the new control units consistently.
- [x] Run targeted planar and pedal tests, then the full unit suite and `git diff --check`.
