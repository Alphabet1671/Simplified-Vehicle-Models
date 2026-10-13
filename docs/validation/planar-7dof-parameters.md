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
