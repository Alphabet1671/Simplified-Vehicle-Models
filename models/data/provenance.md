# Provenance of the legacy powertrain data

Export date: 2026-10-11

This file records the source of every exported data file.
It also records the unit conversions and two open data questions.

## Source files

### Motor map

| Field | Value |
|---|---|
| Absolute path | `/Users/alphabet1671/Dev/lap-sim-2025/Models/Legacy Models/Drivetrain Models/AMK_Characteristic_100C.mat` |
| Identical copy | `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/Latest Input Files/AMK_Characteristic_100C.mat` |
| SHA-256 | `1bff99223cb4537245350e6b15f2a669d2da85ac44154453146ab00f252802fd` |
| Size | 488025 bytes |
| Top-level variable | `MotorCharacteristic`, a struct |
| Grid | 201 speeds x 21 torque points |
| Speed range | 1.0000 rpm to 20000.0000 rpm |
| Pole pairs | 5.000000 |
| Reference bus voltage | 600.0 V |
| Maximum line voltage | 424.2641 V, which is 600 / sqrt(2) |
| Peak shaft torque | 21.834945614425717 N*m |
| Inverter efficiency | 0.95 |
| Ratio inside the map | 11.9 |

The tool reads the file with a small MATLAB v5 reader.
The steps follow.

1. Read the header text and the version word at byte offset 124.
2. Read the first data element at byte offset 128. The type is 15, `miCOMPRESSED`.
3. Decompress the payload with `zlib`. The result is one `miMATRIX` element.
4. Parse the array flags, the dimensions, the name, and the field names.
5. Parse one `miMATRIX` element for each field.

The struct is stored column-major.
The tool reshapes each field to `(201, 21)` with Fortran order.
Column 0 of `Frequency` holds the electrical frequency for each speed.
Column 0 of `Shaft_Torque` is zero for every speed.
Column 20 of `Shaft_Torque` holds the maximum torque for each speed.

The speed axis does not use the `Speed` field.
The `Speed` field uses the character element type `miUINT16`.
Its raw values form the integer grid 0, 100, 200, up to 20 000 rpm.
Its first entry is zero, so the field disagrees with `Frequency` on row 0.
The tool derives the speed axis from `Frequency` instead.

**Speed axis.** The derived axis runs from 1.0 rpm to 20 000.0 rpm.
It has 201 strictly increasing entries.
The step is 100 rpm after the first entry.
The tool rounds the derived value to six decimal places, because the
stored frequency is a double and the round trip is not exact.

**Other scalar fields.** The tool records these values for reference.
`type` is 4, which is the motor count.
`maxPower` is 80, in kW.
`MaxRPM` is 18 600, in rpm.
The export uses none of these values.

**Pole pairs.** The tool confirms the pole pairs from the data.
It computes `60 * Frequency / Speed` on rows 1 to 200.
The source stores zero speed on row 0, so the tool skips that row.
Every ratio equals 5 to within 9e-16.
The relation is `electrical_frequency = pole_pairs * speed / 60`.
Therefore the map has 5 pole pairs.

**Reference bus voltage.** The maximum `Voltage_Line_RMS` is 424.2641 V.
That value is `600 / sqrt(2)`, the space-vector linear limit for a 600 V bus.
The tool records 600 V as the reference bus voltage.

### Cell curves

| Field | Value |
|---|---|
| Absolute path | `/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/Battery Modelling/cosmx_cell_data_curves.csv` |
| SHA-256 | `4f9cf03ecb298c2e153cfadd7b57ff83f8e4a9f4f27559ea9ec8f508d7c47314` |
| Size | 732 bytes |
| Data rows | 18 |
| Header in source | `Voltage,IR, Wh,SOC` |
| Encoding | UTF-8 with a byte-order mark |
| Cell capacity from the curve | 14.1460 A*h |

## Unit conversions

| Column | Source unit | Export unit | Conversion |
|---|---|---|---|
| `speed_rpm` | rpm | rpm | none |
| `maximum_shaft_torque` | N*m | N*m | none |
| `shaft_torque` | N*m | N*m | none |
| `total_loss` | W | W | none |
| `id_peak`, `iq_peak` | A | A | none |
| `vd_peak`, `vq_peak` | V | V | none |
| `stator_copper_loss`, `iron_loss` | W | W | none |
| `magnet_loss`, `mechanical_loss` | W | W | none |
| `input_power` | kW | W | multiply by 1000 |
| `state_of_charge` | percent | 1 | divide by 100 |
| `open_circuit_voltage` | V | V | none |
| `internal_resistance` | milliohm | ohm | multiply by 0.001 |
| `cell_energy` | Wh | Wh | none, the header states the unit |

`cell_energy` stays in Wh, because the source stores Wh and the project
declares the cell capacity in A*h. Multiply by 3600 to get joules.

## Source column meanings

**Motor map fields.** The struct `MotorCharacteristic` holds these fields.

| Field | Meaning | Unit |
|---|---|---|
| `Frequency` | Electrical frequency of the stator field | Hz |
| `Shaft_Torque` | Mechanical torque at the shaft | N*m |
| `Id_Peak`, `Iq_Peak` | Direct and quadrature current, peak value | A |
| `Vd_Peak`, `Vq_Peak` | Direct and quadrature voltage, peak value | V |
| `Stator_Copper_Loss` | Resistive loss in the stator winding | W |
| `Iron_Loss` | Hysteresis and eddy loss in the stator core | W |
| `Magnet_Loss` | Eddy loss in the rotor magnets | W |
| `Mechanical_Loss` | Friction and windage loss | W |
| `Total_Loss` | Sum of the four loss terms | W |
| `InputPower` | Electrical input power at the motor terminals | kW |
| `inverterEfficiency` | Efficiency of the inverter | 1 |
| `ratio` | Gear ratio of the gearbox in the source car | 1 |
| `Voltage_Line_RMS` | Line RMS voltage | V |
| `Voltage_Phase_Peak` | Phase peak voltage | V |

The export does not use the fields `Speed`, `MaxRPM`, `type`, and `maxPower`.

**Cell curve columns.** The source header reads `Voltage,IR, Wh,SOC`.

| Column | Meaning | Unit |
|---|---|---|
| `Voltage` | Cell open-circuit voltage | V |
| `IR` | Cell internal resistance | milliohm |
| ` Wh` | Cell energy, cumulative from empty | Wh |
| `SOC` | Cell state of charge | percent |

Note the leading space in the header name ` Wh`.

## Voltage relations

The identification file holds `vd_peak` and `vq_peak`.
The source voltage columns follow from them by the space-vector relations.
The tool confirmed both relations on every grid point.

```text
voltage_phase_peak = hypot(vd_peak, vq_peak)
voltage_line_rms   = voltage_phase_peak / sqrt(2)
```

The largest difference for the first relation is 1.1e-13 V.
The second relation holds exactly.
The source also sets `Voltage_Line_RMS` equal to `Voltage_Phase_RMS`.

## Exported files

| File | Content |
|---|---|
| `amk_dd5_100c_torque_map.csv` | `speed_rpm`, `maximum_shaft_torque` |
| `amk_dd5_100c_loss_map.csv` | `speed_rpm`, `shaft_torque`, `total_loss` |
| `amk_dd5_100c_identification.csv` | The d/q currents, the d/q voltages, and every loss |
| `cosmx_cell_curves.csv` | The cell curve with converted units |

## Verified relations

The tool checked the following relations against the source.

**Input power.** The column `input_power` equals `shaft_power + total_loss`.
The tool compared the two quantities at the 4020 grid points that carry a
positive shaft torque. The median relative error is 9.9e-10, which is
0.000 percent to three decimal places.
The task brief quotes 3959 points. This export counts 4020 points with a
positive shaft torque, so the count differs. The relation and the median
error agree.
The same column equals `(shaft_power + total_loss) / 0.95` with a median
relative error of 5.0000 percent, because the source stores the motor input
power and the inverter efficiency separately.
Therefore the loss map covers the motor losses only.
The inverter efficiency applies on top of the map.

**Loss components.** `total_loss` equals
`stator_copper_loss + iron_loss + magnet_loss + mechanical_loss` at every
grid point. The largest residual is exactly 0.0 W.

**Resistance.** At the zero-speed end the map needs 20.7313 V of phase peak
voltage at `iq_peak = 83.8133 A` and `id_peak = -18.0367 A`.
The quotient `voltage_phase_peak / iq_peak` is 0.24735 ohm.
The plan records a first-pass fitted resistance of 0.2488 ohm.
The two values differ by 0.6 percent, so the map is self-consistent.

## Open questions

### 1. The loss scale

Conclusion: the factor of 1.60 does **not** apply to this map.
It belongs to the older model, and there it corrects a **cooling load**,
not the loss map.

Evidence:

1. `generateDrivetrainModel.m` builds `drivetrain.lossMap` from
   `MotorCharacteristic.Total_Loss` with no multiplication.
   The script only stores `motorlossScalar` as a separate scalar field.
2. The saved `Drivetrain_Murphy_4WD.mat` proves that no scale was applied.
   Its `lossMap` reproduces the linear interpolation of `Total_Loss` at
   integer torques. Its maximum is 5867.858324713112 W.
   The unscaled interpolation gives 5867.858324713111 W.
   A factor of 1.60 would give 9388.573 W, which the file does not hold.
3. `Total_Loss` is a complete loss model.
   It equals `Stator_Copper_Loss + Iron_Loss + Magnet_Loss + Mechanical_Loss`
   at every grid point.
4. `Total_Loss` also equals `input_power - shaft_power`.
   The source already accounts for the full motor loss.
   A further factor of 1.60 would overstate every loss by 60 percent.
5. The older model `lap-sim` holds the origin of the factor.
   `Execute/Calculators/amkCoolingLoadCalculator.m` line 66 sets
   `motorLossCorrectionFactor = 1.60`.
   `Execute/Run Execution/FullCarCoolingSim.m` line 49 sets
   `motorLossCoefficient = 1.60`.
   Both scripts multiply `MotorCharacteristic.Total_Loss` by the factor
   only when they compute the heat that enters the motor thermal mass.
   The factor is a heat-generation margin for the cooling study.
   The lap-sim energy calculation keeps `Total_Loss` unscaled.
6. The exported loss map therefore keeps the source `Total_Loss` unchanged.

Consequence for the powertrain model: use `total_loss` as it stands.
Do not apply 1.60 to the bus power.
The thermal margin is a cooling study input, and this plan excludes
thermal state.

### 2. The gear ratio

Two values exist. They differ.

| Value | Source |
|---|---|
| 11.9 | The `ratio` field inside `AMK_Characteristic_100C.mat` |
| 10.9 | `generateDrivetrainModel.m`, `drivetrain.gearRatio = 10.9` |
| 10.9 | The project XML `models/drivetrains/murphy_4wd.xml`, `gear_ratio` |

The old `lap-sim` model uses the map value.
`amkCoolingLoadCalculator.m` computes the motor speed with
`MotorCharacteristic.ratio`, which is 11.9.
That value describes the gearbox of the car that produced the motor data.
The 2025 generator and the project both use 10.9.
The value of 10.9 describes the Murphy 2024 car.
The project value stays 10.9. This task changes no project value.
Task P6 must decide which value the transmission model uses.
