# Task 1 implementation report — selectable synchronized MAT channels

## Scope and ownership

Implemented only `tools/track_gui/mat_channels.py` and `tests/test_track_gui_channels.py`, plus this required report. Core module imports no Qt/UI code. MAT I/O uses SciPy lazily and reports an actionable install hint when unavailable.

## API decisions

- Frozen, slotted `ChannelInfo(name, units, samples, start_time, end_time)` and `ChannelSelection(x_channel='VNorth', y_channel='VEast', mode='velocity', x_scale=1.0, y_scale=1.0)` match the requested defaults.
- Added independent `x_allow_unknown_units` and `y_allow_unknown_units` booleans (default false). A non-default numerical scale does **not** implicitly authorize unknown units. Setting the explicit opt-in authorizes conversion of a blank/missing unit label using that channel's scale. Non-empty unsupported unit labels are always rejected, including acceleration labels, so an override cannot silently reinterpret acceleration or other known-incompatible dimensions as velocity. A scale multiplies recognized conversion factors.
- Recognized velocity units: m/s, mps, km/h, kph, mph, ft/s. Recognized position units include metre/meter, km, cm, mm, ft. GPS accepts degrees only (or blank only with explicit per-channel opt-in); coordinates are converted to local N/E using a spherical tangent-plane approximation at the first sample.
- `list_channels` loads metadata/data to report valid channel fields. `load_coordinates` calls `scipy.io.loadmat` with only the selected channel names and `xtime`, avoiding an all-variable build load.
- Each signal's own `Time` is authoritative. Only a missing `Time` field uses `xtime.Value`; malformed present `Time` is an error. Samples are synchronized on selected x channel's timestamps inside both channels' common/requested interval, with interpolation and no extrapolation. Velocity uses trapezoidal integration; position/GPS coordinates are returned relative to their initial point.
- Reject malformed shape/length, nonfinite values, non-strict clocks, absent selections, invalid time ranges, absent common interval, fewer than two in-range samples, invalid GPS bounds, and degenerate or nonfinite paths with actionable `ValueError`s.

## Tests and evidence

Initial requested command, before adding the test module:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_channels -q
ModuleNotFoundError: No module named 'tests.test_track_gui_channels'
(exit code 1; expected initial missing-module evidence)
```

Final commands and exact summaries:

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

New tests cover mismatched clocks, m/s and km/h conversion, position units, GPS mapping, unknown-unit opt-in and acceleration rejection, missing channel, duplicate clock, nonfinite values, and legacy `xtime.Value` fallback. Regression test covers the existing closed-track core.

## Self-review / concerns

- Interpolation uses x-channel sample times as the common output grid; it does not inject exact requested boundaries when those are between x samples. This obeys no-extrapolation and preserves the requested sampled-coordinate interface, but produces a clipped grid rather than synthesized endpoints.
- The tangent plane is appropriate for local track-sized GPS paths, not large geographic extents.
- Degenerate paths are rejected for velocity and position as well as GPS because a usable track geometry requires nonzero displacement.
- No MAT files, canonical models, unrelated source files, pushes, or publications were changed.
