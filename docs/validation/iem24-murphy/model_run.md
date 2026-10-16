# IEM24 Murphy real-model MIS lap

> **Tools layout migration note.** `tools/` was reorganised into category subpackages (`data`, `tracks`, `model_data`, `run`, `gates`, `reports`, `diagnostics`). Commands and paths in this document use the current locations; `tools/run_iem24_murphy_lap.py` named in the reproduce block is not present in the current tree (it was not part of the migration) and cannot be run as written. See [docs/dev/tools-layout.md](../../dev/tools-layout.md) for the old-to-new mapping. Recorded hashes, attempt directories and frozen artifacts are unchanged.

This is the separate real-IEM24 model run on the 2024 Michigan Endurance V11 track. The point-mass solver completed one flying lap with regen disabled and mechanical braking only.

| Run metric | Result |
|---|---:|
| Track | `2024_michigan_endurance_closed.xml` (signed centreline, 1,016.701 m) |
| Lap time | 77.561 s |
| Average speed | 13.108 m/s |
| Minimum / maximum speed | 8.309 / 20.533 m/s |
| Minimum / maximum yaw rate | −1.219 / 1.344 rad/s |
| Initial state of charge | 0.8291 (from 575.70 V measured at rest) |
| Pack terminal voltage | 574.4 → 560.7 V over the lap |
| Solver | Converged; 3 profile iterations, 2 energy iterations |

The model uses 287 kg total mass (209 kg car + 78 kg driver), 42% LLTD, CGx at 45% of wheelbase from the front axle, 0.300 m CG height, CL·A = 3.5 m², CD·A = 1.55 m², and 50% aero balance. The two rear motors are driven; front drive torque share is zero. APPS is capped at 10.5 N·m per motor and regeneration is off.

The lap runs on the **signed closed centreline**, not the legacy V11 radius profile. The legacy profile stores a radius magnitude with no turn direction, so every corner came out as a right-hander and the simulated yaw rate was never negative. The closed centreline was rebuilt in this repository from the same `0615CompEndurance.mat` log, so it carries the real turn directions and coordinates.

The pack starts from the measured resting voltage: the logged `VPack` channel reads a median 575.70 V (standard deviation 0.048 V) over the 12 s the car sits still before launch, and `models/energy-storages/iem24_murphy_real_battery.xml` compiles that to an initial state of charge of 0.8291. The exported `pack_voltage_v` is the terminal voltage (`V_oc − I·R`), because the logged channel is a terminal measurement.

Wheelbase, track width, tire model, battery, gear ratio, air density, and mechanical brake bias remain copied from the existing Murphy model because no replacement measurements were provided for them. The 45%-from-front CGx interpretation is represented as `front_mass_fraction = 0.55` in this model's convention.

![IEM24 Murphy simulated MIS velocity, acceleration, yaw-rate, and pack-voltage traces](simulated_lap.png)

The measured comparison is in [the stint report](report.md). Reproduce the run with:

```bash
PYTHONPATH=src:/tmp/casadi_pkgs MPLCONFIGDIR=/tmp/matplotlib-config \
  python3 tools/run_iem24_murphy_lap.py
PYTHONPATH=src:/tmp/casadi_pkgs python3 tools/data/extract_iem24_telemetry.py
PYTHONPATH=src:/tmp/casadi_pkgs MPLCONFIGDIR=/tmp/matplotlib-config \
  python3 tools/reports/compare_iem24_murphy_stint.py
```

The first command runs the lap, the second conditions the logged telemetry (navigation-unit skew correction, lap segmentation, 1 Hz low-pass), and the third projects the measured path onto the centreline and writes the comparison.
