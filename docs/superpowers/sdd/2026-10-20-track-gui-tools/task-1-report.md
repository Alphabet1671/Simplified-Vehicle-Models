# Task 1 implementation report — selectable synchronized MAT channels

## Scope and ownership

Implemented only `tools/track_gui/mat_channels.py` and `tests/test_track_gui_channels.py`, plus this required report. Core module imports no Qt/UI code. MAT I/O uses SciPy lazily and reports an actionable install hint when unavailable.

## API decisions

- Frozen, slotted `ChannelInfo(name, units, samples, start_time, end_time)` and `ChannelSelection(x_channel='VNorth', y_channel='VEast', mode='velocity', x_scale=1.0, y_scale=1.0)` match the requested defaults.
- Added independent `x_allow_unknown_units` and `y_allow_unknown_units` booleans (default false). A non-default numerical scale does **not** implicitly authorize unknown units. Setting the explicit opt-in authorizes conversion of a blank/missing unit label using that channel's scale. Non-empty unsupported unit labels are always rejected, including acceleration labels, so an override cannot silently reinterpret acceleration or other known-incompatible dimensions as velocity. A scale multiplies recognized conversion factors.
- Recognized velocity units: m/s, mps, km/h, kph, mph, ft/s. Recognized position units include metre/meter, km, cm, mm, ft. GPS accepts degrees (including `°`) or blank only with explicit per-channel opt-in; coordinates are converted to local N/E using a spherical tangent-plane approximation at the first sample.
- `list_channels` loads metadata/data to report valid channel fields. `load_coordinates` calls `scipy.io.loadmat` with only the selected channel names and `xtime`, avoiding an all-variable build load.
- Each signal's own `Time` is authoritative. Only a missing `Time` field uses `xtime.Value`; malformed present `Time` is an error. Samples are synchronized on the union of both clocks, clipped to the requested/common interval with exact endpoints, interpolated without extrapolation. Velocity uses trapezoidal integration; position/GPS coordinates are returned relative to their initial point.
- Reject malformed shape/length, nonfinite values, non-strict clocks, absent selections, invalid time ranges, absent common interval, fewer than two in-range samples, invalid GPS bounds, and degenerate or nonfinite paths with actionable `ValueError`s.

## Tests and evidence

Initial requested command, before adding the test module:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_channels -q
ModuleNotFoundError: No module named 'tests.test_track_gui_channels'
(exit code 1; expected initial missing-module evidence)
```

Initial final-validation commands:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_channels -q
Ran 5 tests in 0.008s
OK

PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_closed -q
Ran 7 tests in 0.031s
OK

git diff --check
(exit code 0; no output)
```

Tests cover mismatched clocks, m/s and km/h conversion, position units, GPS mapping, unknown-unit opt-in and acceleration rejection, missing channel, duplicate clock, nonfinite values, and legacy `xtime.Value` fallback. The test module skips synthetic MAT cases cleanly if SciPy is unavailable.

## Self-review

- The union timestamp grid preserves higher-rate events from either selected channel and exact clipped endpoints. One synthetic test verifies an analytic higher-rate triangular velocity pulse.
- The tangent plane is appropriate for local track-sized GPS paths, not large geographic extents.
- Degenerate paths are rejected for velocity and position as well as GPS because a usable track geometry requires nonzero displacement.
- No MAT files, canonical models, unrelated source files, pushes, or publications were changed.

## Fix round 1 validation

Added tests for nonaligned and narrow intervals; a higher-rate y-channel triangular velocity pulse with analytically expected integrated displacement; legacy discovery; `°`; decreasing and malformed/mismatched clocks; too-short values; no overlap; invalid GPS latitude; unknown position units.

Exact validation evidence:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_channels -q
Ran 9 tests in 0.013s
OK

PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_closed -q
Ran 7 tests in 0.028s
OK

git diff --check
(exit code 0; no output)
```
