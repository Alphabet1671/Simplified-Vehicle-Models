# Planar 7-DOF vehicle parameters

## Scope

This report covers the placeholder vehicle that feeds the planar 7-DOF model,
the quasi-static tire-load solve that both this model and the point-mass model
use, and the obfuscation policy for the source design spec sheet.

## Placeholder data policy

The source spreadsheet describes one real car. **The repository must not carry
that car's numbers.** `tools/import_vehicle_spec_sheet.py` reads the sheet and
emits two XML models:

- `models/vehicles/placeholder_2024_formula.xml`
- `models/aero/placeholder_2024_aero.xml`

Every sheet-derived value passes through two steps:

1. Round to **two significant figures**.
2. Multiply by a skew in `1 ± 0.06`. The skew is deterministic in the
   parameter name, so one source file always gives the same export.

The skew is not cryptography. The goal is that the repository does not publish
a real car's numbers, not that the source is unrecoverable. Two significant
figures followed by a six percent skew means the source value cannot be
recovered from the emitted value to better than about eight percent.

The source file stays outside the repository. The emitted XML records the
source file name and its SHA-256, so the export can be audited without
publishing the data.

**Every emitted value must be replaced with a measured value before any
correlated result.** The XML says so in a header comment, and a test asserts
that the comment is present.

The obfuscation mechanism is tested on synthetic numbers only. No test holds a
real car's value, so the test suite is not a second copy of the source data.

## Values that are not from the sheet

These are emitted unchanged and are marked as estimates in the XML:

| Parameter | Value | Basis |
|---|---|---|
| `yaw_radius_of_gyration` | 0.679366 m | A general estimation request: 120 kg·m² at 260 kg |
| `driver_mass` | 68 kg | The sheet records the driver mass only in a row label |
| `wheel_inertia` | 0.247741 kg·m² | A 16 inch by 8 inch solid cylinder of 12 kg |
| `steering_maximum_angle` | 30 deg | A stated placeholder |
| `steering_maximum_rate` | 180 deg/s | A stated placeholder |
| `brake_bias` | 0.65 | A stated placeholder |
| `maximum_brake_torque_front` | 262.967 N·m per wheel | A stated peak deceleration of 1.5 g |
| `maximum_brake_torque_rear` | 141.598 N·m per wheel | The same peak deceleration and the bias |
| `air_density` | 1.162 kg/m³ | The sheet's stated reference condition |

### Yaw inertia

The sheet does not give a yaw inertia. The model derives it from a radius of
gyration:

```text
yaw_radius_of_gyration = sqrt(120 kg*m^2 / 260 kg) = 0.679366 m
yaw_inertia            = total_mass * yaw_radius_of_gyration^2
```

The inertia therefore follows the loaded mass automatically. For the
placeholder mass of 274.934 kg the model reports 126.89 kg·m². Change
`total_mass` and the inertia follows; change the mass distribution and you must
change the radius of gyration.

### Wheel inertia

The sheet does not give a wheel inertia. The model uses a solid cylinder:

```text
wheel_radius  = 16 in / 2 = 0.2032 m
wheel_inertia = 0.5 * 12 kg * wheel_radius^2 = 0.247741 kg*m^2
```

A real wheel assembly carries most of its mass near the rim, so a thin ring
(`m*r^2`, which is twice this value) is the more physical assumption. A solid
cylinder was requested for testing, so the model uses one. Change
`wheel_inertia` when a measured value exists.

### Brake torque

The sheet gives caliper models, rotor sizes, master cylinder bores and line
pressures, but no caliper piston area and no pad friction coefficient, so the
torque at the pad cannot be computed from the sheet. The model instead derives
a per-wheel torque from a stated peak deceleration:

```text
axle_force   = total_mass * 1.5 g
front_torque = axle_force * brake_bias     * effective_radius / 2
rear_torque  = axle_force * (1 - bias)     * effective_radius / 2
```

This is a capability estimate, not a measurement. It assumes the brakes, not
the tires, set the peak deceleration. It must be replaced before any braking
result is trusted.

## Load transfer convention

`src/simplified_vehicle_models/suspension/quasi_static.py` owns the solve. The
point-mass model and the planar model both call it, so the sign convention has
one implementation.

The axle loads are

```text
front_axle = m*g*front_mass_fraction + downforce*aero_balance - m*ax*h/L
rear_axle  = m*g + downforce - front_axle
```

The lateral split is

```text
total_transfer = m*ay*h/track_width
front_transfer = total_transfer * lateral_load_transfer_distribution
FL = front_axle/2 + front_transfer    FR = front_axle/2 - front_transfer
RL = rear_axle/2  + rear_transfer     RR = rear_axle/2  - rear_transfer
```

`ay` is the body lateral acceleration `v_y_dot + r*v_x` in the SAE frame, so a
positive value is a right turn and moves load to the **left** wheels.

Two properties are worth stating because they are easy to get wrong:

- The sign of `ay` changes which wheel gains load but **not** the axle total.
  An axle's tire capacity is a sum over its two wheels, so it depends on the
  split magnitude and not on the loaded side.
- A positive longitudinal acceleration moves load to the **rear**.

## Parameters the model consumes

| Parameter | Unit | Source |
|---|---|---|
| `total_mass` | kg | Obfuscated, plus the exact driver mass |
| `mass_without_driver` | kg | Obfuscated |
| `driver_mass` | kg | Estimate |
| `wheelbase` | m | Obfuscated |
| `track_width` | m | Obfuscated |
| `cg_height` | m | Obfuscated |
| `front_mass_fraction` | 1 | Obfuscated |
| `lateral_load_transfer_distribution` | 1 | Derived from the front and rear roll rates, then obfuscated |
| `yaw_radius_of_gyration` | m | Estimate |
| `wheel_inertia` | kg·m² | Estimate |
| `steer_ratio` | 1 | Obfuscated |
| `steering_maximum_angle` | rad | Estimate |
| `steering_maximum_rate` | rad/s | Estimate |
| `caster_angle` | rad | Obfuscated |
| `kingpin_inclination_angle` | rad | Obfuscated |
| `brake_bias` | 1 | Estimate |
| `maximum_brake_torque_front` | N·m | Estimate |
| `maximum_brake_torque_rear` | N·m | Estimate |

Wheel order is always FL, FR, RL, RR. The wheel positions are
`(4, 2)` arrays of `(x, y)` in body axes: the front axle sits at a positive
`x` and the right wheels sit at a positive `y`.

The lateral load transfer distribution is derived as the front share of the
total roll rate, because both tracks are equal. The sheet gives front and rear
roll rates but not a load transfer distribution.

## Limits

- **The vehicle is a placeholder, not a car.** The parameters are deliberately
  wrong by up to about eight percent, and several are estimates rather than
  measurements.
- The sheet's own aerodynamic rows disagree. The stated coefficients and
  reference area imply about 1378 N of downforce at the stated condition, while
  the stated downforce is 1239 N. The import uses the **force** rows, because
  the forces are what the model needs, and records the discrepancy here.
- The brake torque is a capability estimate from a stated peak deceleration.
  It is not derived from the caliper, rotor, or pad data.
- The model has one `track_width`. The sheet gives equal front and rear tracks,
  so one value is faithful for this car. A car with unequal tracks needs two.
- No suspension geometry is consumed yet. The wheel rates, roll rates, motion
  ratios and anti-geometry values in the sheet are not used by the planar
  model, because it has no vertical or roll state.
- The powertrain and drivetrain still come from the older `murphy_2024`
  models. The sheet gives a different motor speed limit and final drive ratio.
  Replacing them is a separate task, because the motor map is a fitted artifact
  with its own provenance and pinned values.

## Reproduction

```bash
python3 tools/import_vehicle_spec_sheet.py \
  --source /path/to/design-spec-sheet.xlsx \
  --output-vehicle models/vehicles/placeholder_2024_formula.xml \
  --output-aero models/aero/placeholder_2024_aero.xml

PYTHONPATH=src python -m unittest tests.test_planar_7dof -v
```

## Rigid chassis dynamics

`formulations/planar_7dof.py` evaluates the seven-degree-of-freedom rigid
chassis. The state is `v_x`, `v_y`, the yaw rate `r`, and four wheel speeds.
The input is a net torque at each wheel. A positive torque drives the wheel
forward; a braking torque is negative.

### Slip kinematics

The contact velocity of a wheel at body position `(x, y)` is
`(v_x - r*y, v_y + r*x)`. Express it in the wheel axes, which the front wheels
rotate by the steering angle. A positive steering angle turns the wheel to the
right, so a right-steered wheel at zero body slip sees `v_lat = -v_x*sin(delta)`.

```text
slip ratio = (Re*w - v_long) / max(|v_long|, speed_floor)
slip angle = -atan2(v_lat, max(|v_long|, speed_floor))
```

A driving wheel therefore has a positive slip ratio, and a right-steered wheel
has a positive slip angle and gives a positive lateral force. With no body slip
a wheel steered by `delta` has a slip angle of exactly `delta`; a test pins
that.

`speed_floor` regularizes the low-speed singularity. It is a numerical device,
not a physical quantity.

### Force balance

The tire forces rotate from the wheel axes into the body axes. The vertical
moment does not change. The body equations are

```text
m*(v_x_dot - r*v_y) = sum(Fx_body) - drag
m*(v_y_dot + r*v_x) = sum(Fy_body)
Izz*r_dot           = sum(x*Fy_body - y*Fx_body + Mz)
J*omega_dot_i       = T_i - Re_i*Fx_tire_i + M_rolling_i
```

A forward force on the right wheels gives a negative yaw moment, so it turns
the vehicle to the left. That follows the SAE frame.

### The load and force coupling

The vertical load depends on the body acceleration, and the tire force depends
on the vertical load. The solver closes that loop with a bounded fixed point.
It uses the undamped update while the residual contracts, and halves the step
while it does not. It then evaluates once more at the returned acceleration, so
the loads, the slip, and the forces all describe the same state.

Measured behavior over 1875 states from 2 to 35 m/s, with wheel speeds from
half to 1.6 times rolling, torques from -250 to 250 N·m, steering to 25
degrees, and yaw rates to 1 rad/s:

| Result | Count |
|---|---|
| Converged to `1e-9` m/s² | 1845 |
| Not converged within 60 iterations | 30 |
| Worst residual among converged | `1.0e-9` |
| Worst iteration count among converged | 55 |

**Every one of the 30 non-converged states is physically impossible**: a wheel
lifts, the minimum load reaches -309 N, and the lateral acceleration reaches
3.13 g, which is beyond the tire's friction limit. The solver reports those
states as `load_converged=False` and `contact_feasible=False`. A caller must
reject them; it must never integrate them as if they were valid.

### Two defects found and fixed during this work

1. **An overflow on a lifted wheel.** A negative vertical load made the
   pneumatic-trail decay term change sign and grow, so `exp` overflowed. The
   tire now clamps the contact load to a compressive value, so a lifted wheel
   carries no force and the coefficients stay in their physical domain. The
   MATLAB reference vectors are unaffected, because all of their loads are
   positive.
2. **A non-contractive fixed point.** The undamped iteration failed to converge
   in 60 of 1125 test states with the residual stalling near 0.42 m/s². The
   divergence guard reduced that to 30 of 1875 and cut the worst residual from
   50.4 to `1.9e-6`. The remaining 30 are the impossible states above.

### Limits

- The load loop is a NumPy reference device. An optimal-control formulation
  must not nest it. Carry the loads as algebraic variables and impose the
  consistency equations in the residual instead.
- The loop costs 8 to 55 tire evaluations per evaluation. It converges quickly
  in normal driving and slowly near the limit.
- The model has no camber, no tire relaxation, and no suspension compliance.
  The wheel positions are constant in the body frame.
- The tire aligning moment is carried into the yaw moment. The overturning and
  rolling-resistance moments are returned by the tire but only the rolling
  moment enters the wheel equation. This fit has a zero rolling moment and zero
  overturning contribution from the fitted coefficients.

## Torque allocation

`drivetrain/fixed_split.py` maps a drive and a brake demand to four wheel
torques. It is deterministic, holds no state, and runs no iteration.

The drive split sends 20 percent of the torque to the front axle and 80 percent
to the rear. Each axle splits its share equally, so the four shares are 0.1,
0.1, 0.4 and 0.4.

The largest total that every wheel can carry while keeping the split is
`min(limit_i / share_i)`. A larger request is reduced, so **the front torque
falls with the rear torque and the split is never broken**. That is the
requested behavior when the rear reaches its limit. When one rear wheel allows
20 N·m, the total falls to 50 N·m and each front wheel carries 5 N·m.

The brake allocation uses the same rule with the front bias. A real system
without a proportioning valve would lock the rear instead of holding the bias;
this model holds the bias and reports the shortfall in `unmet_total`.

The allocator cannot see the tire. It does not limit a wheel by slip, and it is
not a traction controller. A tire limit belongs to the vehicle residual as a
constraint.
