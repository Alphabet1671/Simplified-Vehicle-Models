# Closed signed track validation

## Scope

This report covers the rebuilt closed centerline for the 2024 Michigan
endurance track and the `closed-xy-profile` track subtype. It covers no other
model.

The rebuild exists for two reasons. The legacy V11 path stores a radius
**magnitude** only, so it carries no turn direction, and the planar model needs
a signed curvature. The legacy path is also not exactly closed.

The legacy artifact is unchanged. It stays supported for the point-mass model.

## Source

| Role | File | SHA-256 |
|---|---|---|
| Telemetry | `~/Desktop/0615CompEndurance.mat` | `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7` |
| Legacy track | `models/tracks/2024_michigan_endurance_v11.{xml,csv}` | unchanged |

The telemetry holds `xtime`, `VNorth`, and `VEast` as structures with `Time`,
`Value`, and `Units`. Each holds 222394 samples at about 100 Hz. The lap window
is 628.61 s to 698.94 s, as the legacy path-generation script selects. The
window holds 7032 samples.

`tools/build_closed_track.py` rebuilds the path. It needs SciPy for the MAT
file. The runtime loader needs only NumPy.

## World frame

The rebuild uses `X` north, `Y` east, and `Z` down. That frame is right handed,
because north cross east is down. It therefore matches the SAE vehicle frame,
with north as forward and east as right. A right turn has positive curvature
with the standard formula. The measured lap winds by `-2*pi`, which is one
turn in this frame.

The vehicle frame is SAE. See `docs/validation/mis-point-mass.md`.

## Method

1. Integrate the two velocity channels at the logged timestamps with the
   trapezoid rule.
2. Remove the closure gap with a correction that is linear in arc length. The
   correction is zero at the start and at the end, so the result is exactly
   closed without moving the first node.
3. Smooth the position with a periodic Gaussian kernel, so the result stays
   closed.
4. Resample at uniform arc length. Repeat the smoothing and the resample twice,
   so the stored distance is the arc length of the final geometry.
5. Derive the heading and the signed curvature from the final geometry, with a
   wrapped central difference.
6. Repeat the first node at the end, so the stored segment count is one less
   than the stored node count. The existing loader uses that convention.

## Result

| Quantity | Value |
|---|---|
| Raw samples in the window | 7032 |
| Raw integrated length | 1028.464 m |
| Closure gap before correction | 3.448 m |
| Stored nodes | 2034, including the repeated closing node |
| Node spacing | 0.5001 m |
| Stored length | 1016.701 m |
| Winding | `-6.284926` rad against `-2*pi` = `-6.283185` |
| Winding error | 0.0017 rad, which is 0.03 percent of one turn |
| Minimum radius | 6.184 m |
| Radius first percentile | 7.766 m |
| Peak smoothing movement | 0.401 m |
| Minimum self distance | 15.933 m |
| Curvature range | `-0.1392` to `+0.1617` 1/m |

The path carries both turn signs, so the rebuild is the first signed geometry
for this track. The minimum self distance is far above the node spacing, so the
path has no self intersection.

## Comparison with the legacy track

The stored length is 1016.701 m against the legacy 1014.326 m. The difference
is 0.23 percent. The rebuild is **not** rescaled to the legacy length, because
the two paths come from different operations: the legacy path is a circle fit
to a filtered position trace, and the rebuild is an integrated velocity trace.

The legacy minimum radius is 5.21 m and the rebuilt minimum radius is 6.18 m.
The rebuild is smoother, because the periodic Gaussian filter removes the
spurious tight features that the raw integration carries. Without smoothing the
raw path reports a 2.80 m minimum radius, which is not plausible for this car.
The smoothing therefore removes noise rather than real geometry, but it also
removes a little real sharpness. That trade is recorded below.

## Sensitivity

The release gate requires the result to be stable under a change of mesh, of
filter width, and of the lap window.

**Mesh.** At a fixed window and filter width, changing the node spacing from
0.4 m to 0.7 m moves the stored length by less than 0.15 m and keeps the
winding error below 0.01 rad. The result is not mesh sensitive.

**Filter width.** At the chosen window and 0.5 m spacing, the filter width
moves the result as follows:

| Smoothing | Length | Winding error | Minimum radius |
|---|---|---|---|
| 1.5 m | 1021.067 m | 0.0025 rad | 5.626 m |
| 2.0 m | 1016.701 m | 0.0017 rad | 6.184 m |
| 2.5 m | 1011.755 m | 0.0052 rad | 6.578 m |

The chosen width is 2.0 m. It gives the smallest winding error and a minimum
radius between the two neighbours. The width is a documented trade: a wider
filter removes real corner sharpness, and a narrower filter admits noise.

**Lap window.** This is the largest sensitivity. Trimming 0.5 s from each end
shortens the stored length by about 10 m and reports a minimum radius of 1.7 m
to 2.6 m. Extending the window by 0.5 s at each end reports a minimum radius of
4.1 m. The trimmed window is wrong, because a 1.7 m corner is not plausible for
this car. The legacy window is the best behaved of the three, which supports
its use, but the sensitivity means the window is a **selected input** and not a
derived quantity. Change it only with a new measurement.

## Runtime contract

`load_track` accepts two subtypes:

- `distance-radius-profile` keeps the legacy behavior. The radius is a
  magnitude and the curvature is unsigned.
- `closed-xy-profile` loads the signed centerline. It requires
  `is_closed=true`, `terminal_segment_is_duplicate=true`, and a positive
  `maximum_radius`.

For a signed track, `TrackProfile.radius` is the **magnitude**
`min(1/max(|kappa|, 1e-12), maximum_radius)`, so the existing point-mass limit
solver keeps a finite positive radius. `TrackProfile.curvature` is the signed
value, and `TrackProfile.path_coordinates` holds the signed geometry.
`TrackProfile.has_signed_geometry` reports which kind of track a caller holds.

`evaluate_path` interpolates the stored table and wraps any query into one lap.
It reproduces the stored nodes exactly, repeats the position one lap later, and
advances the unwrapped heading by exactly one turn.

## Limits

- **The interpolation is linear.** The returned curvature is continuous and its
  derivative is not. The optimal-control formulation needs a smoother
  representation. Fit that representation from the stored nodes and check the
  fit error against `evaluate_path`. Do not treat the linear curvature as
  smooth.
- The rebuild removes measured drift with a correction that is linear in arc
  length. It does not model the drift, and it does not place the correction
  where the drift occurred.
- The smoothing moves the path by up to 0.401 m. That is the dominant geometric
  approximation in this artifact.
- The lap window is a selected input, and it is the largest sensitivity in this
  report.
- The telemetry window covers one measured lap at one event. The rebuild is one
  centerline, not a track-width model and not a racing line.
- No independent survey of the track was available, so the rebuild is not
  checked against an external geometric reference. The legacy path is a
  different operation on different data, so agreement with it is a consistency
  check and not proof.
- The signed curvature is only as good as the measured velocity. A lateral
  accelerometer or gyro cross-check is possible, but the accelerometer mounting
  is reported to be skewed about the yaw axis, so that check needs a documented
  frame correction first.

## Reproduction

```bash
PYTHONPATH=src python3 tools/build_closed_track.py \
  --output-csv models/tracks/2024_michigan_endurance_closed.csv \
  --output-xml models/tracks/2024_michigan_endurance_closed.xml
PYTHONPATH=src python -m unittest tests.test_closed_track -v
```
