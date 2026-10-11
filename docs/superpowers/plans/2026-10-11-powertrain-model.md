# Powertrain Model Implementation Plan

**Status:** DRAFT v2 — decisions D1, D3, D4, D10, D11, and D12 are recorded below. No code, model, or test change accompanies this document.
**Date:** 2026-10-11
**Branch intended for execution:** see Decision D5, still open

## 1\. Goal

Replace the single constant-torque drivetrain block with a full electric-vehicle model. The model has three physical subsystems, in two assemblies.

**Powertrain assembly** — torque production and energy source. It contains:

1. **Power plant** — the torque-producing device. It stores a torque envelope, a speed envelope, and a loss map. The envelope and the loss depend on the available bus voltage.
2. **Energy storage** — the energy source. It stores state of charge, an open-circuit voltage curve, an internal resistance curve, and charge and discharge power limits. It supplies the bus voltage.

**Drivetrain assembly** — torque transmission. It contains:

3. **Transmission** — gearing and torque split. It converts motor speed and torque to wheel speed and torque.

Decision D1 keeps these two assemblies separate, as the master plan requires. The master plan reserves `drivetrain` for torque transmission and physical torque-split devices, and `powertrain` for torque production and energy source.

The compiled model resolves the subsystems into the two quantities the vehicle models already consume:

- the maximum available wheel torque or drive force at a given speed and energy state;
- the bus power draw, so the model can track energy use and state of charge.

**Shared data across fidelities.** This plan also carries a cross-cutting requirement. One canonical vehicle dataset must serve every model fidelity. A point-mass formulation must consume a full 14-DOF dataset and use the part that it needs. A later high-fidelity addition must not invalidate any existing formulation. Section 5.5 defines the four mechanisms that implement the requirement and the two promises that test it.

## 2\. Current state

The point-mass branch contains one lumped model only.

- `models/drivetrains/murphy_4wd.xml` declares `drivetrain-model` / `four-wheel-electric` with `motor_count`, `gear_ratio`, `peak_motor_torque`, `power_limit`, `efficiency`, and `maximum_motor_speed`.
- `src/simplified_vehicle_models/point_mass.py` uses those values inside `_available_drive_force`. It selects the minimum of a tire limit, a constant motor-torque limit, and a power limit.
- There is no loss map, no energy state, no bus voltage, and no energy accounting. The lap solver cannot report energy use or state of charge.

The legacy MATLAB work contains the physics and the data this plan must use.

- `/Users/alphabet1671/Dev/lap-sim-2025/Models/Modification/generateDrivetrainModel.m` builds a motor loss map from `AMK_Characteristic_100C.mat` with `gearRatio = 10.9`, `count = 4`, `totalPowerLimit = 80000`, `inverterEfficiency = 0.95`, and `motorlossScalar = 1.60`.
- `/Users/alphabet1671/Dev/lap-sim-2025/Models/Legacy Models/Drivetrain Models/AMK_Characteristic_100C.mat` holds the motor characteristic. SHA-256 `1bff99223cb4537245350e6b15f2a669d2da85ac44154453146ab00f252802fd`. A byte-identical copy is at `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/Latest Input Files/AMK_Characteristic_100C.mat`.
- `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/Battery Modelling/cosmx_cell_data_curves.csv` holds cell voltage, internal resistance, energy, and state of charge on 18 data rows. SHA-256 `4f9cf03ecb298c2e153cfadd7b57ff83f8e4a9f4f27559ea9ec8f508d7c47314`.
- `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/Battery Modelling/accumModelCreation.m` defines the pack as 144 series cells and 1 parallel string, with `Vmax = 4.28 V` and `Vmin = 3.0 V`.
- `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Execute/Battery Sim/batteryEnduranceSim.m` re-solves the lap with an updated power limit after each energy pass. That two-level structure is the model this plan adopts.

## 3\. Scope

**In scope**

- Electric motor with a voltage-dependent torque envelope, a loss map, and a field-weakening loss correction.
- Battery pack with open-circuit voltage, internal resistance, capacity, and state of charge.
- Machine parameter identification from the legacy motor characteristic.
- Fixed-ratio transmission with efficiency and driven-wheel torque split.
- Energy accounting over a lap and over a multi-lap stint.
- XML declaration, capability contracts, and compiler validation for the new components.
- Cross-fidelity data sharing, through capability requests and registered reductions.
- The master plan package tree, adopted in one move-only commit.
- Point-mass integration and validation on the legacy MIS track, plus one synthetic high-speed case.

**Out of scope for this plan**

- Battery thermal and electrochemical dynamics. Decision D3.
- Cell-to-cell imbalance, aging, and state-of-health estimation.
- Inverter switching models and bus voltage dynamics. The bus voltage is quasi-static.
- Shiftable gearboxes and clutch dynamics. The plan keeps an extension point only.
- Combustion engine and fuel tank. The plan defines the capability so these fit later. It implements no engine map now.
- Torque vectoring control law. This plan exposes the left and right split plant only. The controller remains separate work.
- Replacement of the master plan package restructure. See Decision D6.

## 4\. Physical definitions

### 4.1 Sign and unit conventions

All quantities are SI in Python. XML carries the unit attribute, and the loader converts on compile.

| Quantity | Symbol | Unit | Sign convention |
|---|---|---|---|
| Motor angular speed | `motor_speed` | rad/s | Positive for forward wheel rotation |
| Motor shaft torque | `motor_torque` | N*m | Positive when the motor drives the wheel |
| Shaft power | `shaft_power` | W | `motor_torque * motor_speed`; positive is motoring, negative is regen |
| Bus power | `bus_power` | W | Positive discharges the storage, negative charges the storage |
| Bus voltage | `bus_voltage` | V | Positive DC terminal voltage |
| Direct-axis current | `direct_current` | A | Negative weakens the field |
| Quadrature-axis current | `quadrature_current` | A | Positive produces driving torque |
| Storage current | `storage_current` | A | Positive discharges the storage |
| State of charge | `state_of_charge` | 1 | From 0 to 1 |
| Gear ratio | `gear_ratio` | 1 | Motor speed divided by wheel speed; greater than one for a speed reducer |

The efficiency definition is one equation for both directions:

```text
bus_power = shaft_power + loss_power
efficiency = abs(shaft_power) / abs(bus_power)
```

`loss_power` is never negative. In regen, `shaft_power` is negative, so `abs(bus_power)` is smaller than `abs(shaft_power)`. This form removes the two-efficiency defect that appears when a single scalar is applied in both directions.

### 4.2 Power plant with voltage-dependent field weakening

Decision D10. The power plant stores:

- a reference torque envelope `reference_torque(motor_speed)` at a stated reference bus voltage;
- a reference loss map `loss_map(motor_speed, abs(motor_torque))`;
- the identified machine parameters `resistance`, `flux_linkage`, `direct_inductance`, `quadrature_inductance`, and `pole_pairs`;
- a maximum motor speed;
- a unit power limit and a unit count;
- an inverter efficiency;
- a minimum bus voltage for the voltage calculation.

**Available voltage**

```text
bus_voltage = energy_storage terminal voltage                       (from Section 4.4)
available_voltage = bus_voltage / sqrt(3) - resistance * current - device_drop
```

The `1 / sqrt(3)` factor is the space-vector linear modulation limit. The plan stores `device_drop` as one parameter and defaults it to zero.

**The voltage constraint**

In the steady-state d/q frame with peak-valued quantities:

```text
v_d = resistance * direct_current - electrical_speed * quadrature_inductance * quadrature_current
v_q = resistance * quadrature_current + electrical_speed * (direct_inductance * direct_current + flux_linkage)
electrical_speed = pole_pairs * motor_speed
v_d^2 + v_q^2 <= available_voltage^2
```

**Corner speed and the envelope law**

Neglecting the resistive terms, the constraint becomes an ellipse whose size is set by `available_voltage / electrical_speed`:

```text
(quadrature_inductance * quadrature_current)^2 + (direct_inductance * direct_current + flux_linkage)^2
    <= (available_voltage / electrical_speed)^2
```

The corner speed is the speed at which the ellipse first touches the current limit with `direct_current` equal to zero:

```text
corner_speed = available_voltage / sqrt((quadrature_inductance * current_limit)^2 + flux_linkage^2)
```

`available_voltage` is the **peak phase voltage**. For space-vector modulation it is `bus_voltage / sqrt(3)` less the drops. State the convention wherever this formula appears, because a line-to-line value changes the result by a factor of `sqrt(3)`.

**This formula is an idealization.** It assumes a machine with constant parameters and a zero direct current at the corner. The measured map reaches its voltage limit at 16 300 rpm with a direct current of -18 A, and its inductance falls as the current rises. Use the formula to reason about the direction and the linearity of the scaling. Do not use it to predict an absolute speed. Section 4.3 records the measured limits.

Therefore the corner speed is proportional to the available voltage. The plan implements this as a speed-axis scaling of the reference curve:

```text
maximum_torque(motor_speed, available_voltage) = reference_torque(motor_speed * reference_voltage / available_voltage)
```

This single expression produces both required effects:

- the corner moves in direct proportion to the available voltage;
- the field-weakening power falls in direct proportion to the available voltage.

For an ideal curve of the form `minimum(torque_limit, corner_power / speed)`, the two statements are identical, because `reference_torque(speed * scale)` equals `minimum(torque_limit, corner_power / scale / speed)`.

**Loss with field weakening**

Decision D10 keeps the reference loss map as the baseline and adds one closed-form correction. The reference map already contains the real field-weakening current at the reference voltage, so the baseline is correct without any additional assumption.

```text
solve for direct_current:
    (resistance * direct_current - electrical_speed * quadrature_inductance * quadrature_current)^2
  + (resistance * quadrature_current + electrical_speed * (direct_inductance * direct_current + flux_linkage))^2
  = available_voltage^2

delta_copper_loss = 1.5 * resistance * (direct_current^2 - reference_direct_current^2)
loss_power = loss_map(motor_speed, abs(motor_torque)) + delta_copper_loss
bus_power = shaft_power + loss_power
```

The solve is a quadratic in `direct_current`. It needs no iteration. The plan requires the root nearest zero that satisfies the constraint, and a reported infeasible status when no root exists.

The quadrature current does not change with voltage, because the torque demand sets it. Only the direct-axis trim moves. This is why the correction is small.

**Tier A and Tier B**

- **Tier A** is the speed-axis envelope scaling plus the closed-form loss correction. The lap solver uses Tier A. It is exact at the reference voltage, where the correction is zero.
- **Tier B** solves the d/q operating point and sums the loss components. It validates the identified parameters against the source map and confirms Tier A at the reference voltage.

**Iron and mechanical loss**

Decision D3 and D10 hold both at the reference map value. Iron loss scales with flux and frequency, and field weakening reduces flux, so the true iron loss falls slightly at low voltage. Holding it constant therefore overestimates low-voltage loss. The choice is conservative and does not affect the torque envelope, because iron loss does not set the envelope. Mechanical loss is exactly voltage-independent.

**Reduction to a scalar efficiency**

The scalar-efficiency reduction remains available for a simple model. `bus_power = shaft_power / efficiency` while motoring, and `bus_power = shaft_power * efficiency` while regenerating. The reduction must also accept a constant reference torque envelope and skip the voltage correction.

### 4.3 Machine parameter identification

Decision D11. The plan identifies the machine parameters from `AMK_Characteristic_100C.mat` by least squares. It does not accept guessed values.

The source map supplies, for every one of the 4 221 operating points, the shaft torque, the d/q currents, the d/q voltages, the electrical frequency, and the loss components.

Task P1 confirmed that the map also stores the speed axis directly, in a field named `Speed` whose array class is `mxDOUBLE` and whose element type is `miUINT16`. It holds 201 unique values from 0 to 20 000 rpm in 100 rpm steps. Task P3 must use that field for the speed axis, and must use `Frequency` only as a cross-check that confirms 5 pole pairs.

Read the element type, not the array class. Several fields in this file declare the `mxDOUBLE` class while storing integers with a narrow element type. A reader that dispatches on the class alone fails on `Speed`, `MaxRPM`, `type`, and `maxPower`.

Task P1 also confirmed these relations against all points:

- `total_loss` equals `stator_copper_loss + iron_loss + magnet_loss + mechanical_loss` exactly.
- `total_loss` equals `input_power - shaft_power`.
- `input_power` equals `shaft_power + total_loss`. The relation holds on the 4 020 points with positive shaft torque, with a median relative error of 0.000 percent.

Identification equations:

```text
resistance        : stator_copper_loss = 1.5 * resistance * (direct_current^2 + quadrature_current^2)
flux_linkage and  : v_q / electrical_speed = flux_linkage + direct_inductance * direct_current
direct_inductance    + resistance * quadrature_current / electrical_speed
quadrature_inductance : v_d = resistance * direct_current - electrical_speed * quadrature_inductance * quadrature_current
pole_pairs        : electrical_frequency = pole_pairs * motor_speed / (2 * pi)
```

The plan requires a joint least-squares fit over all points rather than a median of per-point estimates. A first pass with per-point medians gave `resistance ≈ 0.249 ohm`, `flux_linkage ≈ 0.044 Wb`, `direct_inductance ≈ 1.20 mH`, and `quadrature_inductance ≈ 0.95 mH`, with the linear voltage residual holding to a few millivolt-seconds. That first pass is a starting point only.

Acceptance requirements for the fitted set:

| Requirement | Floor | Tolerance | Basis |
|---|---|---|---|
| Copper loss | above 1000 W | 6.1 percent | the joint optimum for this table |
| Phase voltage | above 60 V | 12.2 percent | the joint optimum for this table leaves this rule inactive |
| Shaft torque | above 5 N*m | 18.2 percent | the joint optimum for this table |
| Zero-speed required voltage | the 1 rpm row | 5.1 percent | the joint optimum for this table |
| Parameter stability under a 10 percent row removal | — | 5 percent relative | guards against a fit dominated by a few points |
| Reproduction of the committed parameter file | — | exact | reproducibility |

A relative error against a near-zero reference value is meaningless. Each tolerance therefore applies only above the stated floor.

**The zero-speed criterion is like for like.** It compares the model's required voltage at the 1 rpm anchor row against the measured voltage at that same row. It does not compare the resistive drop `resistance * current_limit` against the measured total, because the model also predicts the small rotational term at that row. The task must state which definition it uses, because the two differ by 0.045 percent in the resistance.

**Each tolerance carries a margin over the joint optimum, and the real margin is about 1.4 percent.** The margin is a deliberate guard band, so that the test fires only on a degradation greater than about one percent. The reviewer measured the optimiser's own repeatability at about 0.002 percent of the ratio, which is far tighter than the margin. The margin is therefore not needed to absorb optimiser variation, and an earlier justification that called a tighter tolerance "optimiser noise" was quantitatively wrong.

The margin is not free room to relax. The four tolerances are the last adjustment. A further conflict means the model form is at fault, not the table, and it becomes a new decision.

**The fit must minimize the largest error-to-tolerance ratio.** The acceptance criterion is a set of maximum relative errors, so the fitting objective must be that same quantity. A least-squares fit minimizes a different objective and does not reach the joint optimum.

**The fitted parameters are one point in a shallow valley.** Two parameter sets that differ by about two percent in the inductances differ by about `1.5e-5` in the error-to-tolerance ratio. The ratio is well determined at `0.986489`; the individual parameters are not. The reviewer found an independent optimum with `direct_inductance = 1.13234 mH` and `quadrature_inductance = 0.89143 mH` against the delivered `1.10986 mH` and `0.86585 mH`, for the same ratio. A later reader must compare the ratio, not the individual inductances.

**Voltage basis.** The motor map stores its d/q voltages so that `hypot(v_d, v_q)` reaches 600 V at 16 300 rpm at maximum current, and it names that quantity `voltage_phase_peak`. Its own `voltage_line_rms` equals that value divided by `sqrt(2)`, which is the phase root-mean-square value, not the line value. The map is therefore self-consistent with a 600 V bus only if its d/q voltages are **line-to-line peak** values, and the true per-phase peak is `600 / sqrt(3)`.

The fitted `flux_linkage` and inductances inherit that basis, because they were fitted against those voltages. Section 4.2 uses `available_voltage = bus_voltage / sqrt(3)`, which is a per-phase basis. Task P5 must not combine the two without a conversion. The plan requires P5 to work in one stated basis and to prove it by reproducing the measured reference point.

A further factor of `sqrt(3)` would change every envelope speed by that factor, so the basis is settled before Task P5 computes an envelope.

**The per-equation bounds are not simultaneously achievable.** The best set for phase voltage alone gives a torque error of 88 percent. The best set for torque alone gives a phase-voltage error of 31.85 percent. The two equations pull in opposite directions, so only a joint bound is meaningful.

**Known limitations of the model form.** Task P3 measured these effects:

| Effect in the map | Evidence | Consequence |
|---|---|---|
| Resistance rises with speed | per-point resistance from 0.2416 to 0.2700 ohm above 1000 W | 5.55 percent copper loss is the best a constant resistance can do |
| Inductance falls as current rises | per-point quadrature inductance from 1.16 to 0.79 mH | saturation, which a constant inductance cannot follow |
| A drag torque is present | the shaft torque is -0.8491 N*m at zero current | the model equation lacks the term |

Decision D22 records these as known limitations of the model form. They are not defects of the fit, and this work does not remove them. A later task may add saturation and drag terms.

**The parameters do not produce the torque envelope.** The envelope comes from the map, through the scaling law in Section 4.2. The parameters serve two narrower purposes: the direct-axis current that satisfies the voltage constraint at a reduced bus voltage, and the diagnostic quantities of the table above. Task P5 must not use the parameters to predict an absolute torque.

The original tolerances of 3, 5, and 5 percent, and the corner-speed criterion, were written for an ideal constant-parameter machine. The data disproves that model form. The corner-speed criterion is withdrawn, because the envelope comes from the map rather than from the parameters.

### 4.4 Energy storage

Decision D3 limits this model to the electrical behavior. There is no thermal state.

**Battery pack**

Pack quantities from cell quantities:

```text
pack_open_circuit_voltage = series_count * cell_open_circuit_voltage(state_of_charge)
pack_resistance = series_count * cell_resistance(state_of_charge) / parallel_count
pack_capacity = parallel_count * cell_capacity
```

Terminal power as a function of current, with positive current discharging:

```text
terminal_power = pack_open_circuit_voltage * current - current^2 * pack_resistance
terminal_voltage = pack_open_circuit_voltage - current * pack_resistance
```

The terminal voltage is the bus voltage that the power plant consumes in Section 4.2. This is Decision D12: pack sag drives the corner speed directly.

Current from a required terminal power:

```text
current = (pack_open_circuit_voltage - sqrt(pack_open_circuit_voltage^2 - 4 * pack_resistance * terminal_power)) / (2 * pack_resistance)
```

This single expression serves both directions. A negative `terminal_power` gives a negative current, which charges the pack.

Numerical limits:

- The discriminant must stay positive. A discharge request above `pack_open_circuit_voltage^2 / (4 * pack_resistance)` has no solution. The model must report the request as infeasible and clamp it to the peak transfer value.
- `pack_resistance` must be positive. The loader must reject zero.
- At low speed the power plant may request a very large current. The storage power limit is the governing constraint there, and it must clamp the bus power before the torque conversion.

State evolution:

```text
d(state_of_charge)/dt = -current / pack_capacity
```

Pack power limits come from a maximum cell current or from an explicit power table:

```text
discharge_power_limit = pack_open_circuit_voltage * discharge_current_limit - discharge_current_limit^2 * pack_resistance
charge_power_limit = -(pack_open_circuit_voltage * charge_current_limit + charge_current_limit^2 * pack_resistance)
```

Both current limits are positive magnitudes. The charge power limit is negative on the bus convention, because charging stores energy. The implementation must use one function for both directions and must not duplicate the expression.

The Murphy pack uses 200 A discharge and 100 A charge. Section 12 records the resulting power limits against state of charge.

Usable energy uses the same convention as the legacy model. The legacy code integrates cell energy against voltage and evaluates the difference between the maximum and minimum cell voltages. The plan reproduces that calculation once at load time and stores `usable_energy`.

**Super capacitor and fuel tank**

Both use the same capability contract, so the vehicle models do not change when they are added.

- Super capacitor: `open_circuit_voltage = capacitor_voltage`, `resistance = equivalent_series_resistance`, stored energy `0.5 * capacitance * voltage^2`, and state of charge from the voltage ratio.
- Fuel tank: state is `fuel_mass`, consumption is `brake_specific_fuel_consumption(shaft_power) * shaft_power`, and `energy_remaining = fuel_mass * lower_heating_value`. A fuel tank has no charge path and no power limit.

### 4.5 Transmission

Fixed-ratio transmission:

```text
motor_speed = gear_ratio * wheel_speed
wheel_torque = gear_ratio * motor_torque * gear_efficiency      (motoring)
wheel_torque = gear_ratio * motor_torque / gear_efficiency      (reverse power flow)
```

Torque split:

- a driven-wheel list, for example all four wheels;
- a front axle fraction, from 0 to 1;
- a per-axle side split. The first implementation uses an equal side split for each axle.

With four motors and four driven wheels, each motor drives one wheel. The transmission must state this explicitly, and the compiler must verify it. The compiler must reject a configuration where the motor count does not match the driven-wheel count, unless a differential device resolves the mismatch.

The transmission belongs to the drivetrain assembly under Decision D1. It therefore cannot read the powertrain directly. The compiler resolves the two assemblies through the `drive-force-limit` capability.

### 4.6 Composite resolution order

The compiled model resolves limits in this order. Each step can be the active constraint.

1. Read the wheel speeds and calculate each motor speed through the gear ratio.
2. Read the storage state and calculate the bus voltage and the storage power limits.
3. Calculate `available_voltage` from the bus voltage.
4. Calculate the motor torque envelope at `available_voltage` with the Section 4.2 scaling law.
5. Convert the torque demand to a shaft power demand, then to a bus power demand with the loss map and the field-weakening correction.
6. Clamp the bus power to the storage limits.
7. Convert the clamped bus power back to an achievable shaft power. Repeat to convergence, because the loss depends on the torque.
8. Convert the shaft torque through the transmission to per-wheel torque.
9. Report the per-wheel torque, the bus power, the bus voltage, the active constraint, and the storage state derivative.

Step 7 is a small fixed-point solve. The plan requires an explicit residual test and a reported convergence flag. A silent iteration cap is not acceptable.

### 4.7 Regeneration as a pack power limit

Decision D16. The model declares no maximum regen torque. The governing limit is the power that flows into the pack.

**Charge power limit**

The charge current limit is a positive magnitude. The terminal power is negative on the bus convention, because charging stores energy:

```text
charge_power_limit = pack_open_circuit_voltage * charge_current_limit + charge_current_limit^2 * pack_resistance
```

The first term is the power that enters the electrochemistry. The second term heats the pack. The plan calls `charge_power_limit` the terminal power, because the terminal power is what limits the motor.

**Regen force from the power limit**

The flow reverses during braking: wheel, then gearbox, then motor shaft, then motor electrical, then inverter, then pack. The available braking force at the wheels follows from the charge power limit:

```text
regen_force_torque_limited = |motor_regeneration_torque(motor_speed)| * gear_ratio * gear_efficiency / effective_radius
regen_force_power_limited = charge_power_limit * inverter_efficiency * gear_efficiency / speed
regen_force_limit = minimum(regen_force_torque_limited, regen_force_power_limited)
```

The power-limited term is inversely proportional to speed. Regeneration force is therefore large at low speed and small at high speed. This is the opposite shape to a maximum-torque limit, and it is the reason for the change.

The motor envelope still applies. It is not removed. The power limit becomes the governing limit over the operating range where it is smaller.

The motor loss makes the power-limited term implicit, because the loss depends on the torque. The plan requires one fixed-point update on the torque, with the residual reported.

**Braking resolution**

```text
friction_brake_limit = from the brake bias, the tire capacities, and the brake actuation
total_braking_limit = minimum(tire_longitudinal_capacity, regen_force_limit + friction_brake_limit)
regen_force_used = minimum(braking_demand, regen_force_limit)
friction_force_used = braking_demand - regen_force_used
recovered_bus_power = regen_force_used * speed * gear_efficiency * inverter_efficiency - motor_loss
```

Two consequences follow, and the plan states both:

1. When the friction brakes can reach the tire limit alone, the regeneration power limit does not change the deceleration limit. It changes only the recovered energy. Task P9 uses this to keep the braking trace identical when regeneration is enabled.
2. With four motors on four wheels, regeneration is available at every wheel, so no axle split applies. This simplification holds only for the four-motor car.

**Crossover speed**

The two limits cross where the power-limited force equals the tire-limited force:

```text
crossover_speed = charge_power_limit * inverter_efficiency * gear_efficiency / tire_braking_force
```

For the Murphy pack and tire this speed is about 26 m/s, which is above the maximum speed on the MIS track. The MIS lap is therefore tire-limited in braking, and the pack charge limit is inactive there. Section 12 records the numbers.

## 5\. Declarative interface

The XML selects and parameterizes the behavior. Python implements the behavior.

### 5.1 New model types and subtypes

| Type | Subtype | Assembly | Purpose | Phase |
|---|---|---|---|---|
| `power-plant` | `electric-motor` | Powertrain | Envelope, loss map, machine parameters | 1 |
| `power-plant` | `combustion-engine` | Powertrain | Engine torque and fuel map | Later |
| `energy-storage` | `battery-pack` | Powertrain | Cell curves, series and parallel counts, limits | 1 |
| `energy-storage` | `super-capacitor` | Powertrain | Capacitance, resistance, voltage limits | Later |
| `energy-storage` | `fuel-tank` | Powertrain | Fuel mass and consumption map | Later |
| `powertrain-assembly` | `electric` | — | References the power plant and the energy storage | 1 |
| `transmission` | `fixed-gear` | Drivetrain | One ratio, one efficiency, one split | 1 |
| `transmission` | `shifted-gearbox` | Drivetrain | Ratio table and shift schedule | Later |
| `drivetrain-assembly` | `electric-four-wheel` | — | References the transmission and any split device | 1 |

### 5.2 Capabilities

| Capability | Provider | Required methods |
|---|---|---|
| `torque-source` | Power plant | `maximum_torque(motor_speed, available_voltage)`, `evaluate(motor_speed, motor_torque, available_voltage)` |
| `energy-buffer` | Energy storage | `bus_voltage(state, current)`, `power_limits(state)`, `integrate(state, bus_power, duration)`, `usable_energy` |
| `torque-transmission` | Transmission | `motor_speed(wheel_speed)`, `wheel_torque(motor_torque)`, `driven_wheels` |
| `drive-force-limit` | Powertrain plus drivetrain | `maximum_drive_force(speed, storage_state)`, `bus_power(speed, force, storage_state)` |

`drive-force-limit` is the compiled capability that the point-mass model already needs. It becomes a compiler output instead of a hand-written formula. The current `drivetrain-model` subtype remains valid as a reduction that produces the same capability from a single lumped model.

### 5.3 XML examples

**Power plant**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="amk-dd5-100c" type="power-plant" subtype="electric-motor">
    <parameters>
        <parameter name="unit_count" value="4" unit="1"/>
        <parameter name="maximum_torque" value="21.834945614425717" unit="N*m"/>
        <parameter name="maximum_speed" value="18600" unit="rpm"/>
        <parameter name="unit_power_limit" value="20000" unit="W"/>
        <parameter name="inverter_efficiency" value="0.95" unit="1"/>
        <parameter name="device_drop" value="0.0" unit="V"/>
        <parameter name="reference_bus_voltage" value="600.0" unit="V"/>
        <parameter name="resistance" value="0.2488" unit="ohm"/>
        <parameter name="flux_linkage" value="0.0442" unit="Wb"/>
        <parameter name="direct_inductance" value="0.0012045" unit="H"/>
        <parameter name="quadrature_inductance" value="0.0009525" unit="H"/>
        <parameter name="pole_pairs" value="5" unit="1"/>
    </parameters>
    <data ref="../data/amk_dd5_100c_torque_map.csv" kind="torque-map"/>
    <data ref="../data/amk_dd5_100c_loss_map.csv" kind="loss-map"/>
</model>
```

The machine parameter values above are placeholders from the first identification pass. Task P3 replaces them with the joint-fit result.

**Energy storage**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="murphy-2024-battery" type="energy-storage" subtype="battery-pack">
    <parameters>
        <parameter name="series_count" value="144" unit="1"/>
        <parameter name="parallel_count" value="1" unit="1"/>
        <parameter name="cell_maximum_voltage" value="4.227" unit="V"/>
        <parameter name="cell_minimum_voltage" value="3.117" unit="V"/>
        <parameter name="cell_capacity" value="14.146" unit="A*h"/>
        <parameter name="discharge_current_limit" value="200" unit="A"/>
        <parameter name="charge_current_limit" value="100" unit="A"/>
        <parameter name="initial_state_of_charge" value="1.0" unit="1"/>
    </parameters>
    <data ref="../data/cosmx_cell_curves.csv" kind="cell-curve"/>
</model>
```

**Transmission**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="murphy-2024-fixed-gear" type="transmission" subtype="fixed-gear">
    <parameters>
        <parameter name="gear_ratio" value="10.9" unit="1"/>
        <parameter name="gear_efficiency" value="0.95" unit="1"/>
        <parameter name="front_torque_fraction" value="0.5" unit="1"/>
    </parameters>
    <configuration>
        <driven-wheel position="fl"/>
        <driven-wheel position="fr"/>
        <driven-wheel position="rl"/>
        <driven-wheel position="rr"/>
    </configuration>
</model>
```

**Powertrain assembly**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="murphy-2024-powertrain" type="powertrain-assembly" subtype="electric">
    <components>
        <component role="power-plant" ref="../power-plants/amk_dd5_100c.xml"/>
        <component role="energy-storage" ref="../energy-storages/murphy_2024_battery.xml"/>
    </components>
</model>
```

**Drivetrain assembly**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="murphy-2024-drivetrain" type="drivetrain-assembly" subtype="electric-four-wheel">
    <components>
        <component role="transmission" ref="../transmissions/murphy_2024_fixed_gear.xml"/>
    </components>
</model>
```

**Vehicle assembly**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="murphy-2024" type="vehicle-assembly" subtype="four-wheel">
    <parameters>
        <parameter name="total_mass" value="299.0" unit="kg"/>
        <parameter name="wheelbase" value="1.535" unit="m"/>
        <parameter name="track_width" value="1.016" unit="m"/>
        <parameter name="cg_height" value="0.3" unit="m"/>
        <parameter name="front_mass_fraction" value="0.47" unit="1"/>
        <parameter name="lateral_load_transfer_distribution" value="0.44" unit="1"/>
        <parameter name="brake_bias" value="0.65" unit="1"/>
    </parameters>
    <components>
        <component role="tire" ref="../tires/correlated_comp_2024.xml"/>
        <component role="aero" ref="../aero/murphy_2024.xml"/>
        <component role="powertrain" ref="../powertrains/murphy_2024.xml"/>
        <component role="drivetrain" ref="../drivetrains/murphy_2024_drivetrain.xml"/>
    </components>
</model>
```

### 5.4 Data files

| File | Source | Content |
|---|---|---|
| `models/data/amk_dd5_100c_torque_map.csv` | `AMK_Characteristic_100C.mat` | Electrical frequency and maximum shaft torque at full current |
| `models/data/amk_dd5_100c_loss_map.csv` | `AMK_Characteristic_100C.mat` | Speed, torque, total loss in W |
| `models/data/amk_dd5_100c_machine_parameters.csv` | Fitted in Task P3 | Fitted d/q parameters with fit residuals |
| `models/data/cosmx_cell_curves.csv` | `cosmx_cell_data_curves.csv` | Voltage, internal resistance, energy, state of charge |
| `models/data/provenance.md` | Both sources | Source paths, SHA-256 digests, and unit conventions |

The legacy repositories stay read-only.

### 5.5 Fidelity sharing across models

Decision D18. One canonical vehicle dataset serves every fidelity. A lower-fidelity formulation consumes that dataset and uses the part it needs. A later high-fidelity addition must not invalidate an existing formulation.

Four mechanisms implement the requirement.

**1. The assembly is fidelity neutral.**

The vehicle assembly describes the physical car at its highest available fidelity. It does not name a simulation model. Each formulation references the assembly and declares the fidelity that it simulates.

```text
models/vehicles/murphy_2024.xml        one file, full fidelity, every component
models/formulations/murphy_point_mass.xml
models/formulations/murphy_planar_7dof.xml
models/formulations/murphy_full_14dof.xml
```

`models/vehicles/murphy_2024.xml` declared `type="vehicle-assembly" subtype="point-mass"`. That welded the assembly to one fidelity. Task P0B changed the subtype to `four-wheel`, which describes the car rather than the simulation. `models/formulations/murphy_point_mass.xml` already followed the correct pattern.

**2. The formulation declares capability requests.**

A formulation states what it needs, not which component provides it. The point-mass formulation requests:

```text
tire-force-envelope      four contact patches
aero-loads
drive-force-limit
brake-limits
```

The assembly may satisfy a request with any component that provides the capability. The compiler resolves each request against the assembly.

**3. Reductions carry high-fidelity data down.**

When no component provides a requested capability directly, the compiler inserts a registered reduction adapter. The adapter chain is the mechanism that makes one dataset serve every fidelity.

```text
transient-combined-slip  ->  steady-combined-slip  ->  tire-force-envelope
full powertrain          ->  drive-force-limit
```

Each adapter is keyed by the source capability and the target capability. The compiler must select a chain when one exists, and must report a clear error when none exists.

This plan builds the mechanism and registers the first concrete reduction: the full powertrain and drivetrain compile into the `drive-force-limit` capability that the point mass consumes. The tire reduction follows when the UniTire models arrive.

**4. A compile manifest reports what was used.**

The compiled model reports:

- every component that it loaded;
- every capability that it satisfied directly;
- every capability that it satisfied through a reduction, with the chain;
- every assembly component that it did not use.

The manifest makes the promise testable. A silent discard is not acceptable.

**Component schemas are closed. Assemblies are open.**

A component declares a closed parameter set. An unknown parameter in a component is an error, because it is usually a spelling mistake.

An assembly may contain components that a given formulation does not use. Those components are never loaded, so their parameters are never validated against that formulation. This is the rule that lets a point-mass formulation consume a 14-DOF dataset.

The current loader violates this rule. `src/simplified_vehicle_models/xml_loader.py` rejects unknown parameters with the message `invalid parameters; missing=..., unknown=...`. A 14-DOF assembly therefore fails to compile for the point mass today. Task P0B must change the rule to the following:

- a **required** parameter that is absent is an error;
- a parameter that belongs to a component the formulation does not request is ignored, and the component is recorded in the manifest as unused;
- a parameter that belongs to a requested component but is unknown to that component remains an error.

**Two enforceable promises**

**Promise 1, downward.** Compiling a formulation against the full high-fidelity assembly must reproduce the results from a minimal assembly that contains only the components that formulation needs, to a difference of `1e-9`.

**Promise 2, upward.** Adding a component or a parameter to the assembly must leave every existing formulation's compiled model and results unchanged.

Both promises are acceptance tests. Task P0B adds the harness, and Task P7 enforces them.

**Test fixture for the promise before the 14-DOF model exists**

The full 14-DOF model does not exist yet. The plan therefore proves the promise with a fixture assembly that contains high-fidelity components the point mass cannot use:

- a transient tire component, which provides `transient-combined-slip` and not `tire-force-envelope`;
- a suspension lookup-table component, which no current formulation requests;
- the full powertrain assembly.

The point-mass formulation must compile against that fixture, produce results identical to the minimal assembly, and report both unused components in the manifest. When the UniTire models arrive, the tire adapter replaces the fixture tire and the same test must still pass.

The fixture models live in `tests/fixtures/models/`, not in `models/`. Their schema types are registered by the test module, not by the production bootstrap. Task P0B made this change after review: the production registry must not carry a type whose only purpose is a test. A separate test starts a fresh interpreter and requires the fixture type to be absent from the production registry.

## 6\. Python interfaces

```python
@dataclass(frozen=True, slots=True)
class ElectricMotorParameters:
    unit_count: int
    maximum_torque: float
    maximum_speed: float
    unit_power_limit: float
    inverter_efficiency: float
    device_drop: float
    reference_bus_voltage: float
    resistance: float
    flux_linkage: float
    direct_inductance: float
    quadrature_inductance: float
    pole_pairs: int
    torque_map: TorqueMap
    loss_map: LossMap
    efficiency: float | None

@dataclass(frozen=True, slots=True)
class BatteryPackParameters:
    series_count: int
    parallel_count: int
    cell_capacity: float
    discharge_current_limit: float
    charge_current_limit: float
    initial_state_of_charge: float
    curve: CellCurve

@dataclass(frozen=True, slots=True)
class FixedGearParameters:
    gear_ratio: float
    gear_efficiency: float
    front_torque_fraction: float
    driven_wheels: tuple[WheelPosition, ...]

@dataclass(frozen=True, slots=True)
class PowertrainParameters:
    power_plant: ElectricMotorParameters
    energy_storage: EnergyStorageParameters

@dataclass(frozen=True, slots=True)
class DrivetrainParameters:
    transmission: FixedGearParameters

@dataclass(frozen=True, slots=True)
class EnergyStorageState:
    state_of_charge: float

@dataclass(frozen=True, slots=True)
class StorageLimits:
    bus_voltage: float
    discharge_power_limit: float
    charge_power_limit: float
    usable_energy: float

@dataclass(frozen=True, slots=True)
class RegenLimit:
    force_limit: float
    force_from_torque: float
    force_from_charge_power: float
    charge_power_limit: float
    crossover_speed: float
    active_constraint: str

@dataclass(frozen=True, slots=True)
class BrakingResult:
    total_force_limit: float
    regen_force_used: float
    friction_force_used: float
    recovered_bus_power: float

@dataclass(frozen=True, slots=True)
class ShaftPowerResult:
    bus_power: float
    loss_power: float
    delta_copper_loss: float
    direct_current: float
    efficiency: float
    limited_by: str

@dataclass(frozen=True, slots=True)
class DriveLimit:
    maximum_drive_force: float
    maximum_wheel_torque: float
    bus_voltage: float
    bus_power_at_limit: float
    active_constraint: str
    storage_state_rate: float
```

Public functions:

```python
def load_powertrain(path: str | Path) -> PowertrainParameters: ...
def load_drivetrain(path: str | Path) -> DrivetrainParameters: ...
def identify_machine_parameters(source: Path) -> MachineIdentificationResult: ...
def evaluate_storage(storage, state) -> StorageLimits: ...
def integrate_storage(storage, state, bus_power, duration) -> EnergyStorageState: ...
def calculate_available_voltage(motor, bus_voltage, current) -> float: ...
def maximum_motor_torque(motor, motor_speed, available_voltage) -> float: ...
def solve_direct_current(motor, motor_speed, quadrature_current, available_voltage) -> float | None: ...
def evaluate_shaft_power(motor, motor_speed, motor_torque, available_voltage) -> ShaftPowerResult: ...
def motor_speed_from_wheel_speed(transmission, wheel_speed) -> float: ...
def wheel_torque_from_motor_torque(transmission, motor_torque) -> FloatArray: ...
def maximum_drive_force(powertrain, drivetrain, speed, state) -> DriveLimit: ...
def bus_power_demand(powertrain, drivetrain, speed, drive_force, state) -> float: ...
def maximum_regen_force(powertrain, drivetrain, speed, state) -> RegenLimit: ...
def resolve_braking(powertrain, drivetrain, tire_capacity, friction_limit, braking_demand, speed, state) -> BrakingResult: ...
```

Every function name uses the project verb conventions from the master plan: `load_`, `evaluate_`, `integrate_`, `calculate_`, `solve_`, `identify_`, `compile_`.

## 7\. Module layout

Decision D6 adopts the master plan package tree. The tree is adopted in one move-only commit before any new behavior module appears, so that review of the moves stays separate from review of the new work.

The tree below creates only the packages that this plan fills. The master plan lists further packages, such as `signals/` and `suspension/`, which arrive with the work that needs them.

```text
src/simplified_vehicle_models/
├── __init__.py
├── core/
│   ├── conventions.py        # Gravity and shared constants
│   ├── numerics.py           # Monotone interpolation, smoothing, root finding
│   └── types.py              # FloatArray and WheelPosition
├── xml/
│   ├── loader.py             # Parse, resolve references, validate one component
│   ├── model.py              # Model and Reference dataclasses
│   └── paths.py
├── compiler/
│   ├── registry.py           # (type, subtype) to component descriptor
│   ├── capabilities.py       # Capability names and Protocols
│   ├── adapters.py           # Registered reduction adapters
│   ├── compile.py            # Formulation plus assembly to runtime model
│   └── manifest.py           # Loaded, reduced, and unused records
├── parameters/
│   ├── vehicle.py
│   ├── tire.py
│   ├── aero.py
│   ├── drivetrain.py
│   └── powertrain.py
├── tracks/
│   ├── types.py              # TrackProfile
│   └── profile.py            # load_track
├── tires/
│   ├── friction_ellipse.py   # Load-sensitive force envelope
│   └── compile_envelope.py   # Reduction target for the UniTire models
├── powertrain/
│   ├── power_plant.py        # Envelope scaling, loss map, field-weakening correction
│   ├── energy_storage.py     # Battery, capacitor, and tank storage models
│   └── assembly.py           # Power plant and energy storage resolution
├── drivetrain/
│   ├── transmission.py       # Ratio and efficiency
│   ├── lumped.py             # Lumped reduction, retained as a second path
│   └── assembly.py           # Transmission and torque-split resolution
├── aero/
│   └── fixed_aero.py         # Drag and downforce
├── formulations/
│   └── point_mass.py         # Point-mass physics and live limits
├── solvers/
│   └── forward_backward.py   # Flying and standing lap solver
└── results/
    ├── traces.py             # LapResult and energy report
    └── diagnostics.py        # Independent check helpers

models/
├── vehicles/
│   └── murphy_2024.xml               # Canonical assembly, fidelity neutral
├── formulations/
│   └── murphy_point_mass.xml
├── tires/
│   └── correlated_comp_2024.xml
├── aero/
│   └── murphy_2024.xml
├── power-plants/
│   └── amk_dd5_100c.xml
├── energy-storages/
│   └── murphy_2024_battery.xml
├── transmissions/
│   └── murphy_2024_fixed_gear.xml
├── powertrains/
│   └── murphy_2024.xml
├── drivetrains/
│   ├── murphy_2024_drivetrain.xml
│   └── murphy_4wd.xml                # Lumped reduction, retained
├── tracks/
│   ├── 2024_michigan_endurance_v11.xml
│   └── 2024_michigan_endurance_v11.csv
└── data/
    ├── amk_dd5_100c_torque_map.csv
    ├── amk_dd5_100c_loss_map.csv
    ├── amk_dd5_100c_identification.csv
    ├── amk_dd5_100c_machine_parameters.csv
    ├── cosmx_cell_curves.csv
    └── provenance.md

tools/
└── export_legacy_powertrain_data.py   # One-time, read-only legacy export

tests/
├── test_xml_loader.py
├── test_export_data.py
├── test_numerics.py
├── test_identification.py
├── test_power_plant.py
├── test_energy_storage.py
├── test_transmission.py
├── test_powertrain.py
├── test_point_mass.py
├── test_lap_solver.py
├── test_tracks.py
├── test_mis_regression.py
├── test_cross_fidelity.py
├── test_energy_lap.py
└── test_high_speed_regression.py
```

The powertrain models live in `powertrain/`, not in `controls/`. The master plan lists `powertrain.py`, `brakes.py`, and `torque_split_device.py` under `controls/`. A battery, a motor, and a gearbox are plant models, not controllers. Decision D19 records this correction to the master plan tree. The `controls/` package keeps the controllers that the master plan lists, such as the yaw reference, the yaw controller, the traction controller, and the torque allocator.

**Task P0A moves whole modules. It does not split behavior.**

A move is safe to review. A split that decides which function belongs to which domain is a judgment call and does not belong in a move-only commit. Task P0A therefore relocates every existing module and performs only the splits that are pure relocation:

| Current | Target |
|---|---|
| `parameters.py` | `parameters/vehicle.py`, `parameters/tire.py`, `parameters/aero.py`, `parameters/drivetrain.py` |
| `xml_loader.py` | `xml/loader.py`, plus `xml/model.py` for the two dataclasses |
| `tracks.py` | `tracks/types.py` and `tracks/profile.py` |
| `lap_solver.py` | `solvers/forward_backward.py`, plus `results/traces.py` for `LapMode` and `LapResult` |
| `point_mass.py` | `formulations/point_mass.py` |
| The `GRAVITY` constant and the `FloatArray` alias | `core/conventions.py` and `core/types.py` |

`formulations/point_mass.py` keeps its aero and tire functions until Task P0B, which splits them into `aero/fixed_aero.py` and `tires/friction_ellipse.py` at the same time as it introduces the capability seam. The seam makes the boundary obvious. Splitting them during the move would require the same judgment twice.

A module that a later task fills, such as `aero/fixed_aero.py`, `tires/friction_ellipse.py`, or the `compiler/` and `powertrain/` modules, appears in Task P0A only when that task creates it.

The public names exported by `__init__.py` do not change. Tests may change their import paths and nothing else.

`tools/` is not part of the installed package. It runs one time and records the source digest.

## 8\. Integration with the point-mass solver

### 8.1 Drive force limit

`_available_drive_force` in `point_mass.py` currently selects the minimum of a tire limit, a constant torque limit, and a power limit. It becomes a call to `maximum_drive_force`. The active-constraint label set grows to `tire`, `motor_torque`, `power`, `motor_speed`, `voltage`, `storage_power`, and `current_limit`.

### 8.2 Energy-coupled lap solve

State of charge changes during a lap, and both the storage power limit and the bus voltage depend on the state. The lap solve therefore needs an outer iteration.

```text
solve_lap_with_energy(vehicle, track, mode, initial_state, tolerance, maximum_iterations):
    state_trace = constant initial state
    for iteration in 1..maximum_iterations:
        result = solve_lap(vehicle, track, mode, state_trace)
        new_state_trace = integrate_storage_along_track(result)
        change = maximum absolute state change
        if change <= tolerance: stop with convergence
        state_trace = new_state_trace
    return result, state_trace, energy_report
```

Two loops now exist:

- the inner profile iteration over the speed trace;
- the outer energy iteration over the state-of-charge trace.

The plan requires separate convergence flags and iteration counts for both loops.

### 8.3 Multi-lap stint

An endurance helper repeats the lap with carried-over state and reports per-lap time, per-lap energy, and final state of charge. This gives the quantity the legacy `batteryEnduranceSim` produces.

### 8.4 Transparency requirement

The result object must expose enough data for an independent check without calling the production solver again:

- per-segment bus power, shaft power, and bus voltage;
- per-segment state of charge;
- per-segment active constraint;
- the total consumed energy and recovered energy.

### 8.5 Expected effect on the MIS lap

The tire unit correction of decision D17 changed the picture. Before the correction the validated lap peaked at 10 334 motor rpm and the powertrain limits were far away. After the correction the same lap peaks at **15 551 motor rpm**, and the wheel power reaches **58.4 kW**.

| Quantity | Value | Comparison |
|---|---:|---|
| Peak motor speed | 15 551 rpm | 95 percent of the 16 300 rpm corner at 600 V |
| Peak wheel power | 58.4 kW | About 69 kW at the bus, within 15 percent of the 80 kW limit |
| Positive work per lap | 619.0 kJ | 0.172 kWh at the shaft |
| Braking work per lap | -318.7 kJ | Recoverable when regeneration is enabled |

The consequence is that the voltage model is **not** a no-op on MIS, as an earlier version of this plan assumed. Two effects decide the matter:

1. **Pack sag at full load.** At 100 percent state of charge the pack open-circuit voltage is 608.7 V with a resistance of 0.5311 ohm. A 69 kW bus load draws about 113 A, which sags the bus to about 549 V. The corner then falls to about 14 900 rpm, below the 15 551 rpm that the tire-limited lap wants.
2. **State of charge.** The open-circuit voltage falls from 608.7 V to 448.8 V across the usable range, which scales the corner speed by the same factor.

The two effects act together, so an endurance stint becomes progressively voltage-limited. The lap solve is self-consistent, so a voltage-limited car goes slower, draws less current, sags less, and recovers some corner speed. The equilibrium is the quantity that Task P10 must measure.

The plan therefore expects a small lap-time increase over the current 69.3558 s, and it expects the increase to grow as the state of charge falls. It does not expect the large change that the pre-correction model would have produced.

Section 12 records the motor-speed margin and the state-of-charge trend.

## 9\. Migration and backward compatibility

- `models/drivetrains/murphy_4wd.xml` stays valid during the migration. It becomes a reduction that produces the same `drive-force-limit` capability from one lumped model.
- The existing MIS regression tests must keep their documented lap times when the reduction is selected. This proves the reduction path is faithful.
- The full powertrain and drivetrain assemblies become the default for new models after the reduction path passes.
- The XML `schema-version` stays `1.0`. The new types are additive. No existing file becomes invalid.
- The loader must report a clear message when a vehicle assembly declares both a lumped `drivetrain-model` and the new assemblies, or when a role appears twice.

## 10\. Verification strategy

| Check | Method |
|---|---|
| Unit conversion | Load each XML file and compare every compiled value with the XML text |
| Parameter identification | Reproduce copper loss, d/q voltage, and shaft torque within the Section 4.3 tolerances |
| Zero-speed voltage anchor | Require the model to reproduce the measured 20.7 V at full current within 5 percent |
| Corner speed | Require the model to reproduce the measured 16 300 rpm corner within 5 percent |
| Envelope scaling | Require the corner speed and the field-weakening torque to scale linearly with available voltage |
| Exact at reference | Require zero loss correction and an unchanged envelope at the reference bus voltage |
| Loss direction | Compare motoring and regen at equal torque magnitude; require `abs(bus_power) < abs(shaft_power)` in regen |
| Efficiency range | Require `0 < efficiency <= 1` over a dense speed and torque grid |
| Voltage floor | Require a reported infeasible status when the requested torque needs more than the available voltage at full current |
| Battery current | Substitute the solved current into `terminal_power = u*i - i^2*r` and require a small residual |
| Discharge limit | Require the discharge power limit to fall below the system limit only below the recorded state of charge |
| Charge limit | Require the charge power limit to match the recorded table within 1 percent |
| Regen force shape | Require the power-limited regen force to fall as `1 / speed` |
| Regen crossover | Require the crossover speed where the power limit meets the tire limit to match the recorded value within 5 percent |
| Regen is energy-only | Require the braking trace to be identical with regeneration enabled and disabled when the friction brakes can reach the tire limit |
| Regen zero at rest | Require the power-limited regen force to be unbounded only in the limit of zero speed, and require the tire limit to cap it |
| Recoverable power | Require the recovered bus power to be zero at zero braking demand and to increase monotonically with braking demand |
| Battery energy | Integrate a constant-power discharge and compare the released energy with `usable_energy` |
| Charge conservation | Charge for a known duration, then discharge by the same charge, and compare the final state with the initial state |
| Bus voltage coupling | Require the corner speed to fall when the state of charge falls |
| Transmission | Require `wheel_torque * wheel_speed == gear_efficiency * motor_torque * motor_speed` while motoring |
| Split | Require the per-wheel torque sum to equal the total torque and the front fraction to match |
| Assembly validation | Feed mismatched motor count, driven-wheel count, and split, and require a clear rejection |
| Reduction fidelity | Require the lumped reduction to reproduce the current MIS lap times to 1e-6 s |
| Energy balance | Require bus energy to equal the storage energy change plus the storage losses |
| Outer loop | Require the energy iteration to converge below tolerance within the cap, and require a reported flag when it does not |
| Constraint agreement | Require the reported active constraint to equal an independently calculated minimum |
| Cross-fidelity, downward | Require a formulation compiled against the full assembly to reproduce the minimal-assembly result to `1e-9` |
| Cross-fidelity, upward | Require the result to stay unchanged after a component is added to the assembly |
| Manifest completeness | Require the manifest to list every loaded, reduced, and unused component |
| Missing capability | Require a clear error when a formulation requests a capability with no provider and no reduction chain |
| Closed component schema | Require an unknown parameter inside a requested component to remain an error |
| Open assembly | Require an assembly with unused components to compile without an error |
| High-speed case | Require a converged lap on a synthetic fast track whose motor speed exceeds the corner speed |

Every task runs the full suite before commit.

## 11\. Tasks

Each task is one implementer dispatch with its own tests and commit.

Task P0A and Task P0B run after Task P1 completes. Task P1 touches `tools/`, `models/data/`, and one test file, so it does not conflict with the source moves. Running the moves while Task P1 executes its test suite would produce a spurious failure, so the order is P1, then P0A, then P0B.

### Task P0A — Move to the master plan package tree

**Deliverables**

- The package tree in Section 7.
- Updated imports in every moved module, in the tests, and in `__init__.py`.
- An unchanged public API through `simplified_vehicle_models`.

**Method**

- Move files only. Do not change logic.
- Keep the public names in `__init__.py` unchanged, so `from simplified_vehicle_models import load_point_mass_vehicle` and `load_track` keep working.
- Keep the class and function names unchanged.
- Do not create empty packages for work that does not exist yet. Create only the directories that Section 7 fills.

**Tests**

- The full suite passes with no test edits other than import paths.
- The MIS regression reproduces 69.3558 s and 70.5198 s exactly.

**Depends on:** P1.

### Task P0B — Capability registry, reductions, and open assemblies

**Deliverables**

- `compiler/registry.py`, `compiler/capabilities.py`, `compiler/adapters.py`, `compiler/manifest.py`, and `compiler/compile.py`.
- Per-component compile functions moved out of `xml/loader.py` and into their component modules.
- The parameter rule from Section 5.5: closed component schemas, open assemblies.
- `tests/test_cross_fidelity.py` with the fixture assembly from Section 5.5.
- A fidelity-neutral subtype on `models/vehicles/murphy_2024.xml`.

**Method**

- Move each per-component compile function beside the component it compiles. The loader keeps reference resolution, file parsing, and dispatch through the registry.
- Declare the capabilities from Section 5.2 as names and Protocols.
- Register one reduction: the full powertrain and drivetrain into `drive-force-limit`.
- Implement the manifest and report loaded, reduced, and unused records.
- Change the parameter rule. A missing required parameter stays an error. A parameter of a component that the formulation does not request is ignored, and the component is reported as unused.
- Add the fixture assembly with an unused transient tire and an unused suspension lookup table.

**Tests**

- Promise 1: the point-mass formulation compiles against the fixture assembly and reproduces the minimal-assembly result to `1e-9`.
- Promise 2: the same formulation, compiled after a component is added to the assembly, reproduces its previous result to `1e-9`.
- The manifest lists both unused components.
- A formulation that requests a capability with no provider and no reduction chain reports a clear error.
- An unknown parameter inside a requested component remains an error.
- The full suite passes.

**Depends on:** P0A.

### Task P1 — Export the legacy powertrain data

**Deliverables**

- `tools/export_legacy_powertrain_data.py`.
- `models/data/amk_dd5_100c_torque_map.csv`.
- `models/data/amk_dd5_100c_loss_map.csv`.
- `models/data/cosmx_cell_curves.csv`.
- `models/data/provenance.md`.

**Method**

- Read `AMK_Characteristic_100C.mat` with a small read-only MATLAB v5 reader, as the track export already did. The file is a compressed MATLAB v5 file whose only variable is the struct `MotorCharacteristic`.
- Export the electrical frequency axis, the maximum shaft torque at full current, and the total loss map. Put the units in the header.
- Also export the d/q currents, d/q voltages, and the loss components, because Task P3 needs them. Keep them in a separate `amk_dd5_100c_identification.csv`.
- Copy the cell curve CSV. Convert the internal resistance to ohms and the state of charge to a fraction.
- Resolve the `motorlossScalar = 1.60` question. The source map already contains `Total_Loss`. Decide whether the scale belongs to this map or only to the older 2025 model, and record the decision in `provenance.md`.
- Record the reference bus voltage. The maximum required line voltage is 424.2641 V, which is `600 / sqrt(2)`. That value is the space-vector linear limit for a 600 V bus.
- Record the speed range and the pole pairs. The frequency axis spans 1 to 20 000 rpm with 5 pole pairs.

**Tests**

- File schema, row count, and column count.
- Peak torque at zero speed equals `21.834945614425717 N*m`.
- Internal resistance and state of charge are monotone.
- Source digests match the values in Section 2.
- The maximum required line voltage equals `600 / sqrt(2)` within 0.01 V.

**Depends on:** nothing.

### Task P2 — Shared lookup-table module

**Deliverables**

- `src/simplified_vehicle_models/luts.py`.
- `tests/test_luts.py`.

**Method**

- Implement monotone grid validation, linear interpolation, and optional smoothing.
- Reject duplicate or unsorted abscissae.
- Reject extrapolation requests, and return a clear error instead.

**Depends on:** nothing. Runs in parallel with P1.

### Task P3 — Identify the machine parameters

**Deliverables**

- `src/simplified_vehicle_models/identification.py`.
- `models/data/amk_dd5_100c_machine_parameters.csv`.
- `tests/test_identification.py`.

**Method**

- Implement the joint least-squares fit from Section 4.3. Fit `flux_linkage`, `direct_inductance`, `quadrature_inductance`, and `resistance` together. Do not use per-point medians for the delivered values.
- Determine the pole pairs from the electrical frequency and the mechanical speed.
- Report the residual for each equation and the count of points used.
- Report the predicted corner speed and compare it with the measured value.

**Tests**

- Every acceptance requirement in the Section 4.3 table.
- The corner prediction is within 5 percent of 16 300 rpm.
- The zero-speed voltage is within 5 percent of 20.7 V.
- A synthetic operating point generated from the fitted parameters is recovered by the fit.

**Depends on:** P1 and P2.

### Task P4 — Energy storage model

**Deliverables**

- `src/simplified_vehicle_models/energy_storage.py`.
- `models/energy-storages/murphy_2024_battery.xml`.
- `tests/test_energy_storage.py`.
- XML loader entries for `energy-storage` / `battery-pack`.

**Method**

- Implement the pack equations from Section 4.4.
- Use the recorded values: 144 series, 1 parallel, 200 A discharge, 100 A charge, 14.146 A*h, and clamping cell voltage limits of 3.117 V and 4.227 V.
- Convert the exported cell energy from watt-hours to joules. The exported curve keeps watt-hours so that the capacity integration yields ampere-hours directly.
- Derive `usable_energy` from the clamped range, and record the value.
- Implement `bus_voltage`, `power_limits`, `integrate`, and `usable_energy`.
- Implement one quadratic solver for both current directions. Do not duplicate the expression.
- Reject a zero or negative resistance and a series or parallel count below one.

**Tests**

- The power limits reproduce the Section 12 table within 1 percent.
- The discharge limit falls below the 80 kW system limit only below the recorded state of charge.
- The measured resistance and state of charge are monotone where the source is monotone.
- The derived capacity is 14.146 A*h within 0.1 percent.

**Depends on:** P2.

### Task P5 — Power plant model with voltage dependence

**Deliverables**

- `src/simplified_vehicle_models/power_plant.py`.
- `models/power-plants/amk_dd5_100c.xml`.
- `tests/test_power_plant.py`.
- XML loader entries for `power-plant` / `electric-motor`.

**Method**

- Implement `calculate_available_voltage` with the `1 / sqrt(3)` factor.
- **Settle the voltage basis before computing an envelope.** The fitted parameters are in the map's own basis, which is line-to-line peak, while `calculate_available_voltage` returns a per-phase value. Choose one basis, state it in the module, and convert the other quantity into it. A factor of `sqrt(3)` changes every envelope speed by the same factor.
- **Prove the basis at the reference point.** Reproduce the measured reference condition: the maximum current at 16 300 rpm requires the full reference bus voltage. The task must show the computed voltage at that operating point against the measured value and state the difference.
- Use `maximum_speed = 18600 rpm` for the motor limit, per decision D21. Do not copy the 20 000 rpm table range into the limit.
- Use the loss map unscaled, per decision D20. Declare no loss scale parameter.
- **Do not use the fitted parameters to predict an absolute torque.** They minimize a joint acceptance ratio for a model form with three measured limitations, and they sit in a shallow valley. Use them for the field-weakening loss correction and the voltage solve, and take the envelope from the map.
- Implement the speed-axis envelope scaling from Section 4.2.
- Implement `solve_direct_current` as one quadratic solve, with a reported infeasible status.
- Implement the loss map plus the field-weakening correction.
- Implement the scalar-efficiency reduction, which skips the voltage correction.
- Require the correction to be exactly zero at the reference bus voltage.

**Tests**

- The corner speed scales linearly with available voltage.
- The field-weakening torque scales linearly with available voltage.
- The correction is zero at the reference voltage.
- Efficiency stays within zero and one over a dense grid.
- A request above the voltage ceiling at full current reports infeasible.
- Regen returns less bus power than shaft power in magnitude.

**Depends on:** P2 and P3.

### Task P6 — Transmission model

**Deliverables**

- `src/simplified_vehicle_models/transmission.py`.
- `models/transmissions/murphy_2024_fixed_gear.xml`.
- `tests/test_transmission.py`.
- XML loader entries for `transmission` / `fixed-gear`.

**Method**

- Implement speed and torque conversion with directional efficiency.
- Implement the front fraction and the equal side split.
- Reject a ratio of zero or below, an efficiency outside zero and one, and a front fraction outside zero and one.

**Depends on:** nothing. Runs in parallel with P4 and P5.

### Task P7 — Powertrain and drivetrain assemblies

**Deliverables**

- `powertrain/assembly.py`.
- `models/powertrains/murphy_2024.xml`.
- `models/drivetrains/murphy_2024_drivetrain.xml`.
- `tests/test_powertrain.py`.
- XML loader entries for `powertrain-assembly` / `electric` and `drivetrain-assembly` / `electric-four-wheel`.
- Capability descriptors for `torque-source`, `energy-buffer`, `torque-transmission`, and `drive-force-limit`.
- The reduction that compiles the two assemblies into `drive-force-limit`.

**Method**

- Implement the nine-step resolution in Section 4.6.
- Implement the regeneration resolution in Section 4.7. Declare no maximum regen torque.
- Implement the inner fixed-point on shaft torque with a residual test and a convergence flag.
- Implement the one fixed-point update on regen torque for the motor loss, with the residual reported.
- Validate component compatibility and report every mismatch with the model path.
- Keep the two assemblies separate under Decision D1. Join them only through the compiled capability.
- Preserve the lumped `drivetrain-model` reduction as a second path to `drive-force-limit`, and require it to reproduce the current lap times.
- Register the reduction in the adapter registry from Task P0B. Do not hard-code the chain in the compiler.
- **Allow nested assemblies.** The assembly compiler currently rejects a reference to another assembly. This task introduces `powertrain-assembly` and `drivetrain-assembly`, and the vehicle assembly references both, so the compiler must resolve an assembly reference to the components inside it. Extend the resolution and record each nested assembly in the manifest.
- The vehicle assembly must reference the two nested assemblies and the tire and aero components, so that one canonical dataset carries every fidelity.

**Tests**

- The power-limited regen force falls as `1 / speed`.
- The crossover speed matches the recorded value within 5 percent.
- The torque-limited and power-limited regen forces are both reported, with the minimum active.
- The braking trace is identical with regeneration enabled and disabled when the friction brakes reach the tire limit.
- The active constraint equals an independently calculated minimum.
- The two paths to `drive-force-limit` agree, so the lumped reduction and the full assembly produce the same drive limit at the same operating point.
- Promise 1 and Promise 2 from Section 5.5 still pass with the full assemblies in the fixture.

**Depends on:** P4, P5, P6.

### Task P8 — Point-mass integration

**Deliverables**

- Updated `point_mass.py` and `xml_loader.py`.
- Updated `models/vehicles/murphy_2024.xml` referencing both assemblies.
- Extended `tests/test_point_mass.py` and `tests/test_mis_regression.py`.

**Method**

- Replace the internal drivetrain limit with the compiled capability.
- Keep the lumped reduction selectable, and require identical previous lap times.
- Add the new active-constraint labels.
- Confirm that the MIS result does not change materially, and record the difference.

**Depends on:** P7.

### Task P9 — Energy-coupled lap solve

**Deliverables**

- `solve_lap_with_energy` and `solve_stint` in `lap_solver.py`.
- `tests/test_energy_lap.py`.
- Extended example output with energy, state-of-charge, and bus-voltage traces.

**Method**

- Implement the outer state iteration from Section 8.2 with a separate convergence flag.
- Implement the multi-lap stint helper.
- Report consumed energy, recovered energy, final state of charge, and the minimum bus voltage.
- With regeneration disabled, report zero recovered energy and require the total to match the work-only calculation.
- With regeneration enabled, require the recovered energy to be limited by the tire braking capacity on the MIS track, because the charge power limit is inactive there.
- Report the net energy per lap and the resulting lap count from the usable pack energy.

**Depends on:** P8.

### Task P10 — Validation and report

**Deliverables**

- `docs/validation/powertrain.md`.
- MIS flying-lap and standing-start results with the full model.
- One synthetic high-speed track whose peak motor speed exceeds the corner speed, with results.
- A multi-lap state-of-charge study for the legacy 22-lap endurance count.
- `tests/test_high_speed_regression.py`.
- An explicit statement of every assumption and every limit of the model.

**Method**

- State clearly that the MIS lap does not exercise the voltage model, and show the motor-speed margin.
- State clearly that the voltage scaling is analytic. The two legacy `AMK_DD5` files are different machines and do not provide a second measured voltage point. See Section 12.
- Report the sensitivity of the lap time to the bus voltage on the synthetic track.

**Depends on:** P9.

## 12\. Measured data reference

These values come from `AMK_Characteristic_100C.mat`. They are the reference for the tasks above.

| Quantity | Value | Note |
|---|---|---|
| Grid size | 201 speeds x 21 torques | 4 221 points |
| Speed range | 1 to 20 000 rpm | 100 rpm per speed step |
| Pole pairs | 5 | From `electrical_frequency = pole_pairs * speed / 60` |
| Peak shaft torque | 21.834945614425717 N*m | Identical to the current point-mass XML value |
| Maximum quadrature current | 83.813 A | At the torque peak |
| Maximum direct current | -43.306 A | Field weakening |
| Maximum current magnitude | 85.7 A | Implied current limit |
| Reference bus voltage | 600 V | From maximum line voltage `600 / sqrt(2)` |
| Zero-speed required voltage | 20.7 V at 83.8 A | Equals `resistance * current = 20.9 V` for the fitted resistance |
| Corner speed | about 16 300 rpm | First point at the voltage limit with full current |
| Torque at the corner | 19.87 N*m | Above the corner the current must fall |
| Current at 17 000 rpm | 77.7 A | Voltage-limited |
| Current at 17 600 rpm | 70.2 A | Voltage-limited |
| Loss components | copper, iron, magnet, mechanical, total | Additive and complete |
| Peak total loss | 6 205 W | Per motor |
| Peak copper loss | 2 832 W | Per motor |
| Peak iron loss | 2 843 W | Per motor |
| Peak mechanical loss | 167 W | Per motor |
| Inverter efficiency | 0.95 | Matches the current XML value |
| Ratio inside the map | 11.9 | The project uses 10.9. Confirm which is correct. |

**First-pass identified parameters.** These came from per-point medians and are a starting point only. Task P3 replaces them.

| Parameter | First-pass value | Spread |
|---|---|---|
| `resistance` | 0.2488 ohm | p10 to p90: 0.242 to 0.267 |
| `flux_linkage` | 0.0442 Wb | p10 to p90: 0.0425 to 0.0466 |
| `quadrature_inductance` | 0.9525 mH | p10 to p90: 0.835 to 1.125 |
| `direct_inductance` | 1.2045 mH | From the joint voltage equation |

The first pass predicted a corner near 14 200 rpm against the measured 16 300 rpm. That 15 percent gap is the reason Task P3 requires a joint fit.

**Voltage sensitivity, computed from the first-pass parameters**

| Available voltage | Corner speed |
|---|---:|
| 100 percent | 14 161 rpm |
| 95 percent | 13 439 rpm |
| 90 percent | 12 717 rpm |
| 85 percent | 11 995 rpm |
| 80 percent | 11 273 rpm |

The scaling is linear to within one percent.

**Copper-loss correction from field weakening**

| Speed | at 100 percent | at 90 percent | at 80 percent |
|---|---:|---:|---:|
| 10 000 rpm | 0 W | 0 W | 0 W |
| 12 000 rpm | 0 W | 0 W | 27 W |
| 14 000 rpm | 0 W | 68 W | infeasible |

Against peak total losses near 6 200 W, the correction is a fraction of one percent of loss below the corner and roughly 1 percent of loss just above it. The dominant voltage effect is the torque envelope, not the loss.

**Motor speed on the MIS lap**

```text
peak vehicle speed     = 28.9352 m/s      (after the D17 tire correction)
effective_radius       = 0.193675 m
gear_ratio             = 10.9
motor speed            = 28.9352 / 0.193675 * 10.9 = 1628 rad/s = 15 551 rpm
corner speed at 600 V  = 16 300 rpm
margin                 = 749 rpm, or 4.6 percent of the corner speed
```

The MIS lap now runs at 95 percent of the corner speed. It did not before the tire correction, when the same lap peaked at 10 334 rpm and 63 percent. The voltage model therefore has a small but real effect on MIS, and the effect grows as the state of charge falls.

For comparison, the pre-correction values were a peak of 28.9352 m/s against 19.2278 m/s and a wheel power of 58.4 kW against 14.7 kW.

**Cell curve detail, from `cosmx_cell_data_curves.csv`**

| Quantity | Value | Note |
|---|---|---|
| Rows | 18 | State of charge from 100 percent to 0 |
| Voltage range | 4.227 V to 3.117 V | Measured range |
| Internal resistance range | 1.92 to 10.01 milliohm | Minimum near 50 percent state of charge |
| Energy range | 0 to 50.7606 Wh | Cumulative from 0 percent state of charge |
| Legacy voltage limits | 4.28 V and 3.0 V | Both outside the measured range; see decision D15 |

The internal resistance changes by a factor of five across the state-of-charge range, so the resistance curve is required. A constant resistance would misstate the bus sag and therefore the corner speed.

**Pack power limits, from 200 A discharge and 100 A charge**

| State of charge | Pack voltage | Pack resistance | Discharge, bus | Discharge, stored | Resistance heat | Charge, bus | Charge, stored |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0% | 448.8 V | 1.4412 ohm | 32.1 kW | 89.8 kW | 57.7 kW | 59.3 kW | 44.9 kW |
| 1.5% | 468.0 V | 0.9569 ohm | 55.3 kW | 93.6 kW | 38.3 kW | 56.4 kW | 46.8 kW |
| 7.7% | 498.0 V | 0.3459 ohm | 85.8 kW | 99.6 kW | 13.8 kW | 53.3 kW | 49.8 kW |
| 13.8% | 503.6 V | 0.2825 ohm | 89.4 kW | 100.7 kW | 11.3 kW | 53.2 kW | 50.4 kW |
| 50.8% | 530.4 V | 0.2825 ohm | 94.8 kW | 106.1 kW | 11.3 kW | 55.9 kW | 53.0 kW |
| 75.4% | 563.9 V | 0.3632 ohm | 98.3 kW | 112.8 kW | 14.5 kW | 60.0 kW | 56.4 kW |
| 100.0% | 608.7 V | 0.5311 ohm | 100.5 kW | 121.7 kW | 21.2 kW | 66.2 kW | 60.9 kW |

Three consequences:

1. The pack discharge limit stays above the 80 kW system limit until the state of charge falls below roughly 5 percent. The system limit therefore governs normal operation, and the pack limit governs only a nearly empty pack.
2. The charge limit ranges from 53.2 to 66.2 kW and has a minimum near 10 percent state of charge. Regeneration is therefore weakest when the pack is either empty or nearly full.
3. The resistance heat is large at low state of charge. At 0 percent the pack dissipates 57.7 kW of the 89.8 kW that the open-circuit voltage supplies. The electrical model records this power but does not represent the temperature rise, because decision D3 excludes thermal state. This is the physical reason that a real car derates, and it is the reason decision D14 keeps the legacy derating map as an explicit strategy rather than as physics.

**Regeneration force from the charge power limit**

| Speed | Force at 50 percent state of charge | Force at 100 percent state of charge |
|---:|---:|---:|
| 8.0 m/s | 6 301 N | 7 466 N |
| 12.0 m/s | 4 201 N | 4 977 N |
| 16.0 m/s | 3 151 N | 3 733 N |
| 19.2 m/s | 2 626 N | 3 111 N |

The tire-limited corner braking force is about 1 900 N with the corrected load-sensitivity units of decision D17. The pack charge limit is therefore inactive on the MIS track at every speed, and the crossover speed is about 26 m/s. The MIS lap is tire-limited in braking. The regeneration power limit matters on a faster track, not on MIS.

**Legacy power derating, from `batteryEnduranceSim.m`**

| Time, minutes | Allowed power, kW |
|---|---:|
| 0 | 80 |
| 10 | 80 |
| 20 | 55 |
| 30 | 40 |
| 40 | 30 |
| 50 | 30 |

The legacy script clips this map against the pack peak power and recomputes the pack peak power after every segment. It also holds a separate `endurancePowerLimit` of 55 kW. See decision D14.

**The two `AMK_DD5` files are not a voltage pair.** `AMK_DD5_100.mat` has a peak torque of 21.8349 N*m with a gentle falloff, and `AMK_DD5_532V.mat` has a peak torque of 18.0 N*m with a flat region to 13 000 rpm and then a constant-power falloff. Different peak torques mean different machines or different limits, so the pair cannot measure the voltage law. The voltage dependence in this plan is analytic and is validated against the identified parameters and the corner speed, not against a second measured voltage.

## 13\. Risks

| Risk | Effect | Control |
|---|---|---|
| The joint fit does not reproduce the corner speed | The envelope scaling uses wrong parameters | Task P3 makes the corner agreement the primary acceptance test |
| The `motorlossScalar` belongs to a different map | Bus power is wrong by a constant factor | Task P1 resolves it and records the decision |
| The map ratio of 11.9 differs from the project value of 10.9 | Motor speed and the corner margin are wrong | Task P1 records both; the project keeps 10.9 for the reduction test |
| Loss map extrapolation near zero speed | Nonphysical regen or drive force | Task P2 rejects extrapolation, and Task P5 clamps to the envelope |
| Large current request at low speed | Nonphysical power spike | The storage limit clamps bus power before the torque conversion |
| Energy iteration does not converge | Long run time or silent wrong answer | Explicit cap, tolerance, and reported flag |
| Reduction path drifts from the lumped model | The MIS regression becomes meaningless | Task P8 requires identical lap times to 1e-6 s |
| The MIS lap does not exercise the voltage model | A defect stays hidden | Task P10 adds a synthetic high-speed case |
| New modules duplicate the master plan restructure | Extra churn later | Decision D6 records the accepted cost |
| The constant power limit hides the endurance derating | The state-of-charge study is optimistic | Decision D14 records the choice; the endurance study states the assumption |
| Pack limits are invented rather than measured | Usable energy and power limits are wrong | Decision D15 requires a derived capacity and a measured current limit |
| The measured cell voltage range is narrower than the legacy limits | The usable energy extrapolates | Section 12 records the range; decision D15 requires an explicit choice |
| A silent iteration cap repeats the earlier defect | Unverified result | Sections 4.6 and 8.2 both require a residual and a flag |

## 14\. Decisions

### 14.1 Recorded

**D1 — Separate `drivetrain` and `powertrain`. RECORDED.**
The master plan structure applies. `powertrain` contains the power plant and the energy storage. `drivetrain` contains the transmission and any physical torque-split device. The vehicle assembly references both. The two assemblies join only through the compiled `drive-force-limit` capability.

**D3 — Ignore thermal behavior. RECORDED.**
The energy storage model is electrical only. No pack temperature, no thermal limit, no cooling model.

**D4 — Regeneration. RECORDED.**
The physics implements the regen path. A `regeneration_enabled` switch defaults to `false`, so the first validation matches the legacy endurance simulation, which disables regen. Both settings are validated.

**D5 — Branch. RECORDED.**
The work proceeds on a new branch `feature/powertrain`, branched from `feature/point-mass`. The reviewed point-mass work stays intact and mergeable on its own.

**D10 — Voltage-dependent motor model. RECORDED.**
The envelope scales as `reference_torque(speed * reference_voltage / available_voltage)`. The loss uses the reference loss map plus one closed-form correction from the field-weakening current. Iron loss and mechanical loss stay at their map values. `available_voltage` includes the `1 / sqrt(3)` modulation factor and a device drop.

**D11 — Identify the machine parameters. RECORDED.**
`flux_linkage`, `direct_inductance`, `quadrature_inductance`, and `resistance` come from a joint least-squares fit of `AMK_Characteristic_100C.mat`. No guessed values. The corner speed agreement within 5 percent is the primary acceptance test.

**D12 — Bus voltage from the energy storage. RECORDED.**
The power plant reads the terminal voltage from the energy storage model as `bus_voltage`. Pack sag therefore reduces the corner speed directly.

**D15 — Pack limits and capacity source. RECORDED.**
The pack uses 144 series cells and 1 parallel string. The steady-state current limits are 200 A discharge and 100 A charge. The cell capacity is 14.146 A*h, derived by integrating `dE / V` over the measured discharge curve. The measured cell voltage range is 3.117 V to 4.227 V, and the plan clamps the usable range to those values instead of extrapolating to the legacy 3.0 V and 4.28 V. Section 12 records the resulting power limits.

**D16 — Regeneration as a pack power limit. RECORDED.**
Regeneration is limited by the power that flows into the pack, not by a maximum regen torque. The model declares no regen torque limit. The charge current limit of 100 A produces a charge power limit from 53.2 to 66.2 kW across the state-of-charge range, which the model converts to a speed-dependent braking force. Section 4.7 records the calculation.

**D17 — Tire load-sensitivity units. RECORDED.**
The legacy MATLAB fit uses pound-force loads. The Python model evaluated the coefficients against newton loads. The model now uses the converted coefficient `-0.000404656098` per newton, and a regression test requires the lateral friction to stay between 1.0 and 1.4 over corner loads from 700 N to 1200 N.

The correction changed the validated MIS results:

| Quantity | Before | After |
|---|---:|---:|
| Flying lap | 95.2064 s | **69.3558 s** |
| Standing start | 96.2448 s | **70.5198 s** |
| Peak vehicle speed | 19.2278 m/s | 28.9352 m/s |
| Peak wheel power | 14.7 kW | 58.4 kW |
| Peak motor speed | 10 334 rpm | 15 551 rpm |

The corrected flying lap is 0.94 s faster than the measured Lap 8 trace of about 70.30 s and 0.62 s faster than the cited 69.98 s fastest-lap reference. The unit defect was the dominant cause of the earlier 24.9 s gap.

**D6 — Package layout. RECORDED.**
The master plan package tree applies. The tree is adopted in one move-only commit, Task P0A, so that review of the moves stays separate from review of the new behavior. Only the packages that this plan fills are created. The master plan's remaining packages arrive with the work that needs them.

**D18 — Fidelity sharing across models. RECORDED.**
One canonical vehicle dataset serves every fidelity. Section 5.5 defines the four mechanisms: a fidelity-neutral assembly, capability requests in the formulation, registered reduction adapters, and a compile manifest. Component schemas stay closed and assemblies stay open, so a component that a formulation does not request is never loaded or validated against it. Two promises are acceptance tests: a lower-fidelity formulation compiled against the full assembly must reproduce the minimal-assembly result to `1e-9`, and adding a component to the assembly must leave every existing formulation unchanged.

**D19 — Powertrain models live outside `controls/`. RECORDED.**
The master plan lists `powertrain.py`, `brakes.py`, and `torque_split_device.py` under `controls/`. A battery, a motor, and a gearbox are plant models. This plan places them in `powertrain/`. The `controls/` package keeps the controllers that the master plan lists.

### 14.2 Still open

**D20 — The loss scale does not apply to the motor map. RECORDED.**
`generateDrivetrainModel.m` stores `motorlossScalar = 1.60`, but it scales nothing. Task P1 proved this from the saved `Drivetrain_Murphy_4WD.mat`: the `lossMap` maximum is 5867.858325, which equals the unscaled `Total_Loss` interpolation. A scale of 1.60 would give 9388.573. The controller verified this independently.

The same factor appears in the older `lap-sim` repository as `motorLossCorrectionFactor` and `motorLossCoefficient`, where it multiplies `Total_Loss` for the cooling-load calculation only. The power plant therefore uses `Total_Loss` unscaled, and the power plant model declares no loss scale parameter.

**D21 — Motor speed limit and table range are different values. RECORDED.**
The motor map declares `MaxRPM = 18600`, and its speed table spans 0 to 20 000 rpm in 100 rpm steps. The current `models/drivetrains/murphy_4wd.xml` sets `maximum_motor_speed = 20000`, which copies the table range into the motor limit. The motor limit is 18 600 rpm.

The controller verified both values from the source map. The map also declares `type = 4` and `maxPower = 80`, which confirms four motors and the 80 kW system limit that decision D14 retains.

The MIS lap peaks at 15 551 rpm, so this correction does not change the MIS results. It changes the maximum vehicle speed from 37.2 m/s to 34.6 m/s, and it matters for the synthetic high-speed case in Task P10.

**D22 — The constant-parameter machine model has known limitations. RECORDED.**
Task P3 fitted the machine parameters jointly and measured the deviations between the map and a constant-parameter model. The deviations are physical, and the best achievable error for the model form is larger than the plan first assumed:

| Effect | Measured | Consequence |
|---|---|---|
| Resistance rises with speed | 0.2416 to 0.2700 ohm above 1000 W | 5.55 percent copper loss is the best a constant resistance can do |
| Inductance falls as the current rises | 1.16 to 0.79 mH | saturation, which a constant inductance cannot follow |
| A drag torque is present | -0.8491 N*m at zero current | the model equation lacks the term |

**The per-equation bounds cannot be met at the same time.** The set that best fits phase voltage alone gives a torque error of 88 percent. The set that best fits torque alone gives a phase-voltage error of 31.85 percent. Only a joint bound is meaningful.

A joint search over 120 starts and a 40 000-point sample gives the best joint set: 5.86 percent copper, 11.56 percent phase voltage, 17.33 percent torque, and 4.98 percent zero-speed voltage. The smallest achievable largest-ratio against the original table of 6, 10, 15, and 5 percent is 1.156, so that table was not attainable by any parameter set. Section 4.3 carries the corrected table.

A second measurement tightened those numbers, because the zero-speed criterion must compare like with like. Comparing the model phase voltage at the 1 rpm anchor row against the measurement at that row gives a joint optimum of 6.03 percent copper, 12.05 percent phase voltage, 18.07 percent torque, and 5.02 percent zero-speed voltage. The earlier zero-speed figure of 4.98 percent compared the resistive drop alone. The two definitions differ by 0.045 percent in the resistance, and the copper criterion pulls the opposite way, so the definition decides which of the two binds.

Section 4.3 places each tolerance a guard band above the joint optimum, so the test fires only on a degradation greater than about one percent. The optimiser is repeatable to about 0.002 percent, so the band is deliberate rather than necessary.

A least-squares fit does not reach the joint optimum, because it minimizes a different objective. The fit must minimize the largest error-to-tolerance ratio.

The implementer reported both the miss and the deeper cause, and changed no parameter and no tolerance while the question was open. The controller verified the reasoning before replacing the table.

**D2 — Loss model fidelity.**
*Recommendation:* the two-dimensional loss map is the default, consistent with D11. The scalar-efficiency reduction stays available for a simple model.

**D7 — Transmission scope.**
*Recommendation:* fixed ratio only in this plan. The shiftable gearbox stays an extension point.

**D8 — Torque vectoring.**
*Recommendation:* expose the left and right split plant only. The vectoring controller remains separate work, as the master plan requires.

**D9 — Endurance stint length.**
The legacy track stores `energyLapCount = 22`.
*Recommendation:* use 22 laps for the state-of-charge study and report the result as a study, not as a race prediction.

**D13 — Gear ratio.**
The source map stores `ratio = 11.9`. The 2025 generator and the current project XML use 10.9. The 532 V file stores 12.953858.
*Recommendation:* keep 10.9 for the current car and the reduction-fidelity test, record 11.9 as map metadata, and confirm the physical ratio with the team. The ratio changes the motor-speed margin and therefore the corner headroom.

**D14 — Power limit schedule.**
The legacy endurance script does not use a constant power limit. `batteryEnduranceSim.m` holds `powerMap = [0, 80; 10, 80; 20, 55; 30, 40; 40, 30; 50, 30]`, with time in minutes and power in kW. It clips that map against the pack peak power, and it recomputes `maxAccumPower` from the simulated peak cell power after every segment. It also carries a separate `endurancePowerLimit = 55 kW`. The current project model uses one constant `power_limit = 80000 W`.
Options:

1. Keep the constant 80 kW system limit. This preserves the current MIS numbers exactly.
2. Implement the time-based endurance derating map as declared data.
3. Derive the limit from the pack state at every segment, using the available peak cell power.

*Recommendation:* option 1 for the first implementation, because it keeps the reduction-fidelity test meaningful. Add option 2 or 3 as a separate later task for the endurance study.
*If wrong:* the endurance energy and state-of-charge study is optimistic, because the real car loses about 60 percent of its allowed power over a stint.

## 15\. Legacy defects not to replicate

1. `maxTireLongAccel.m` uses `rearLoad` inside the front-axle traction expression. The front axle limit therefore depends on the rear load. The new model must use the applicable axle load.
2. `accumModelCreation.m` uses `spcHeatCapacity` before it is defined in the file. The plan defers thermal modeling, so this defect does not carry over, but the exported data must not depend on it.
3. `energySimUpdatedMotor.m` converts energy with a fixed `tStep = 0.1` before overwriting it with the real step. The new model must use the solved segment duration.
4. `regenCalculator.m` mixes `v` in mph and m/s in the same expression, and applies three separate scalar efficiencies. The new model uses SI only and one loss equation for both directions.
5. The two `AMK_DD5` files have different peak torques but similar names. A reader can mistake them for one machine at two voltages. The provenance file must state the difference.

## 16\. Definition of done

- The power plant, the energy storage, and the transmission exist as XML-declared, capability-bearing components.
- `powertrain` and `drivetrain` are separate assemblies that join only through the compiled capability.
- The package tree matches Section 7, and the moves landed in a commit that changed no behavior.
- The vehicle assembly is fidelity neutral, and a formulation selects the fidelity.
- The point-mass formulation compiles against an assembly that contains components it cannot use.
- Promise 1 holds: the full assembly reproduces the minimal-assembly result to `1e-9`.
- Promise 2 holds: adding a component to the assembly leaves every existing formulation unchanged.
- The compile manifest reports every loaded, reduced, and unused component.
- The machine parameters come from the joint fit and meet every acceptance tolerance in Section 4.3.
- The envelope scaling reproduces the measured corner speed within 5 percent and is exactly linear in available voltage.
- The loss model uses the reference map plus the field-weakening correction, and the correction is zero at the reference voltage.
- The bus voltage comes from the energy storage, and the corner speed falls when the state of charge falls.
- The pack uses 200 A discharge and 100 A charge, a capacity of 14.146 A*h, and the clamped measured voltage range.
- Regeneration is limited by the pack charge power, and the model declares no maximum regen torque.
- The power-limited regen force falls as `1 / speed`, and the crossover speed is reported.
- The lumped reduction reproduces the current MIS lap times to 1e-6 s.
- The full model produces a converged flying lap and standing start on the MIS track.
- The synthetic high-speed case produces a converged lap that exceeds the corner speed.
- The energy report closes its own balance and reports both loop convergence flags.
- The full test suite passes.
- The validation document states the assumptions, the sources, the measured anchors, and the remaining limits.
