# FSAE Acceleration and Skidpad Tracks

Two signed open tracks model the Formula SAE dynamic events. The point-mass lap solver runs them in `LapMode.STANDING_START`. The rules source is *Formula SAE Rules 2026, Version 1.0, 10 Sept 2025*, Sections D.9 and D.10. The PDF was fetched on 2026-10-19 from the [FSAE Series Resources download](https://www.fsaeonline.com/cdsweb/gen/DownloadDocument.aspx?DocumentID=278fd4d7-aa27-4e33-bc4a-090148e662a0) (sha256 `4620c337…f790f7d8`). A later rules issue may differ.

| Track | Files |
|---|---|
| Acceleration | `models/tracks/fsae_2026_acceleration.{xml,csv}` |
| Skidpad | `models/tracks/fsae_2026_skidpad.{xml,csv}` |

Regenerate with `PYTHONPATH=src:. python3 tools/tracks/build_fsae_dynamic_tracks.py`. `tests/test_fsae_dynamic_tracks.py` checks that the committed files match the generator.

## Rule values used

| Event | Rule | Value | Use |
|---|---|---|---|
| Acceleration | D.9.1.1 | 75 m, starting line to finish line | Timed length |
| Acceleration | D.9.2.3 | Staged 0.30 m behind the line; timing starts at the line | Track starts at 0 m; timed section is 0.30 m to 75.30 m |
| Acceleration | D.9.1.2 | Width at least 4.9 m | Recorded only; a centerline path has no width |
| Skidpad | D.10.1.1 | Centers 18.25 m apart; inner circle 15.25 m, outer 21.25 m diameter; 3.0 m path | Centerline radius `(15.25 + 21.25) / 4 = 9.125` m |
| Skidpad | D.10.2.3 | Right lap, timed right lap, left lap, timed left lap, exit | Four consecutive circle laps |
| Skidpad | D.10.4.1 | Time is the mean of the timed right and left laps | Laps 2 and 4 |

## Modelling choices that are not rule values

- The rules show the skidpad entry and exit lanes only in a diagram, which this repository did not read as text. The entry and exit straights are 15 m each (`entry_length_assumed`, `exit_length_assumed`). They only set the speed at which the car reaches the first circle.
- The car follows the ring centerline. A real driver may use a larger radius. The point-mass solver then reports a lower bound on lap time.
- The course heads north from the origin. Right turns have positive curvature, so laps 1 and 2 have positive curvature and laps 3 and 4 have negative curvature.
- The curvature jumps at the touching point of the circles: 0 to +1/R, +1/R to −1/R, and −1/R to 0. A table of nodes cannot store a jump, so each jump is a 0.05 m ramp (`jump_ramp_length`). This adds 0.15 m to the course. The rules give no transition geometry. Each jump needs an infinite steering rate, so a planar model sees a hard steering step there. The 7DoF still follows it (see below), so the rule geometry is kept. `skidpad_course(transition_length=...)` in `tools/tracks/build_fsae_dynamic_tracks.py` builds a clothoid variant for study; it is not the committed track.

## Open-track format

The tracks are `open-xy-profile` models. The format is general, so other open courses work the same way.

- XML: `type="track-model"`, `subtype="open-xy-profile"`, a `<data ref>` CSV, and the parameters `is_closed=false`, `terminal_segment_is_duplicate=false`, and `maximum_radius`.
- CSV columns: `distance,x_north,y_east,heading_rad,curvature`. The first distance is 0. The last row is the end of the course and does not repeat the first row. The world frame is X north, Y east, and a right turn is positive.
- `load_track` checks that the heading is the integral of the curvature, that each position step has the length of its distance step, and that it points along the mean heading. A bad row is reported by its line number.
- `TrackProfile.is_closed` and `PathCoordinates.is_closed` are false. `evaluate_path` does not wrap an open path. Outside the course it continues as a straight line along the end tangent, with zero curvature.
- The radius for the point-mass solver is `1 / |curvature|`, capped at `maximum_radius`.

## Building other open courses

`tools/tracks/open_course.py` builds a course from sections of linearly varying curvature: `straight`, `arc`, `arc_by_angle`, and `transition` (a clothoid with a bounded curvature rate). `build_course` refuses a curvature jump between sections unless `allow_curvature_jumps=True`. `write_course` writes the XML and the CSV. `tests/test_open_course.py` shows examples.

## Using the tracks

The simulation GUI offers only `closed-xy-profile` tracks, so these do not appear there. The forward-driver runner accepts them (see below).

```python
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle

vehicle = load_point_mass_vehicle("models/formulations/murphy_point_mass.xml")
track = load_track("models/tracks/fsae_2026_acceleration.xml")
lap = solve_lap(vehicle, track, LapMode.STANDING_START, tolerance=1.0e-5)
```

The lap time is the time to the end of the track. For the acceleration event, subtract the elapsed time at 0.30 m. For the skidpad, take the elapsed time between the timing distances listed in the track XML.

## Running the 7DoF and 14DoF models from a standing start

Both forward-driver models run any `open-xy-profile` course from 0 m/s:

```bash
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model planar-7dof \
    --track models/tracks/fsae_2026_acceleration.xml --planner live --maximum-speed 40 \
    --method BDF --speed-gain 10 --duration 30 --output-root <dir>
```

Use `--model 14dof-simple --timeout-seconds 1800` for the vertical model. No lateral-acceleration cap is needed: on an open course the live planner uses 90 % of the point-mass cornering grip by default (`--lateral-margin`, see below).

What an open course changes, all keyed on `track.is_closed` / `plan.is_closed` so closed laps are unchanged:

- **Speed plan.** `build_speed_plan(..., open_course=True)` plans a standing start (`speed[0] = 0`) with no end tie, and `SpeedPlan(is_closed=False)` does not wrap and holds its end values.
- **Plants.** `PlanarForwardPlant` and `VerticalForwardPlant` accept an initial speed of 0 and a minimum speed and progress of 0 on an open plan. A closed plan keeps 0.25 m/s.
- **Launch.** The foot starts off the pedal (`initial_pedal=0`) and the driver presses the throttle through the pedal lag. Starting at the driver's requested pedal stalls the 14DoF driven wheel at the first steps.
- **Transient tire.** `RATE_SPEED_FLOOR` in `tires/unitire_transient.py` is now 0.5 m/s (was 0.1). At 0.1 the slip state cannot follow a driven wheel that spins up in milliseconds, and the 14DoF step size collapses at about 0.05 m/s. The floor only matters below 0.5 m/s. The tire parameter record in `docs/validation/14dof-simple-parameters.md` was updated.
- **Lateral margin.** `--lateral-margin` (above 0, at most 1) is the fraction of the point-mass cornering grip the live planner uses. It defaults to 0.90 on an open course and 1 on a closed lap, so closed-lap plans are unchanged. It is applied by scaling the corner radius in the planner's point-mass solve, so acceleration and braking are re-solved on the slower profile and straights are unchanged. `build_speed_plan(..., lateral_margin=)` exposes it.
- **Speed gain.** `--speed-gain` (default 2, the old fixed value) sets the speed-error gain. At 2 the car lags the plan by about 1 m/s from rest and runs 7.99 s over the timed 75 m; 10 follows the plan.
- **Gates.** `acceptance_gate` requires that an open run reaches the end of the course and skips the periodic-state delta. The dense-domain certificate uses a non-wrapping curvature enclosure and no 0.25 m/s margin on an open plan; it still fails on any real reversal (below −1e-9 m/s).
- **Event times.** The summary carries `event_times`, read from the `<name>_start_distance` and `<name>_end_distance` parameters in the track XML: `timing` for the acceleration (0.30 m to the end), and `right_timed_lap` and `left_timed_lap` for the skidpad.

### Results

Live planner, BDF, `--speed-gain 10`, entry and exit straights of 15 m. These are smoke results of the solvers, not a validation against measured data.

| Event | 7DoF | 14DoF |
|---|---|---|
| Acceleration, 0.30 m to 75.30 m | 7.491 s | 7.491 s |
| Skidpad right timed lap | 5.009 s | 5.039 s |
| Skidpad left timed lap | 5.008 s | 5.038 s |
| Skidpad mean | 5.008 s | 5.038 s |

Both models pass the dense-domain, contact, and path-tracking gates on these runs (largest lateral error on the skidpad 0.17 m for the 7DoF and 0.21 m for the 14DoF). Neither run is `usable`: `between_step_validation_complete` and `refinement_validated` are never set by this runner, for flying laps as well.

### Why the skidpad needs a lateral margin

The point-mass planner's corner speed is a steady-state grip limit (about 12.3 m/s, 16.6 m/s² on this car). A car planned at that limit has no grip left to change direction. On the skidpad it holds the right circle, but the right-to-left change then pulls it off the path by about 2.1 m (limit 0.25 m), and the left lap diverges (at a margin of 0.99: 4.81 s right, 5.39 s left). Raising the steering-loop gains (heading 3, lateral 3) and slowing the plan locally around the change did not fix it; slowing the whole circle did. Lap time and tracking against the margin, 7DoF unless noted:

| `--lateral-margin` | Right / left timed lap | Largest lateral error | Tracking valid |
|---|---|---|---|
| 0.99 | 4.812 s / 5.386 s | 2.1 m | no |
| 0.97 | 4.85 s / 5.41 s | 2.2 m | no |
| 0.96 | 4.867 s / 4.866 s | 0.17 m | yes |
| 0.95 | 4.888 s / 4.887 s | 0.17 m | yes |
| 0.90 (default) | 5.009 s / 5.008 s | 0.17 m | yes |
| 0.85 | 5.149 s / 5.148 s | 0.18 m | yes |
| 0.95, 14DoF | 4.940 s / 4.939 s | 0.26 m | no |
| 0.92, 14DoF | 4.996 s / 4.995 s | 0.22 m | yes |
| 0.90, 14DoF | 5.039 s / 5.038 s | 0.21 m | yes |

The 7DoF fails between 0.96 and 0.97 and the 14DoF between 0.92 and 0.95, so 0.90 leaves a small margin on both. The value is empirical, and the bound is a property of this vehicle and this driver, not of the rules. The skidpad time is therefore a lower bound on what the car could do with a better steering controller, not a prediction. An explicit `--lateral-acceleration` cap still works and combines with the margin. Clothoid transitions of 4 to 11 m at the change did not fix the uncapped case; 12 m did, at the cost of 24 m of extra course.

## Point-mass smoke result

With `murphy_point_mass.xml`: acceleration 4.55 s over the timed 75 m (29.9 m/s at the finish). The skidpad figure of about 5.44 s came from the earlier radius-only track and has not been re-run on the current track files. This is a solver sanity check, not a validation against measured Murphy data.
