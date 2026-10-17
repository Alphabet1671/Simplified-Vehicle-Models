# Animation exporter for the vehicle models — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** investigation result and proposed plan. No production code exists yet.

**Goal:** Export a 120 fps video from one forward simulation of any model in the
[fidelity ladder](../../project-master-plan.md), with two side-by-side windows:
a whole-track view on the left, and a three-panel vehicle view on the right.
The exporter consumes one shared run result file and contains no
model-specific physics.

**Architecture:** The forward simulation records a
[run result file](../specs/2026-10-16-run-result-file.md) at every accepted
integrator step. A pure-NumPy resampler turns that record into frames at the
requested rate. A renderer draws each frame from named channels only. A frame
encoder writes raw RGB to `ffmpeg` through one pipe. Every panel is an
orthographic projection of one 3-D scene in body axes, so the box placeholder
and an imported STL mesh use the same transform.

**Tech Stack:** Python 3.12+, NumPy (library), matplotlib and `ffmpeg` (tools
only), unittest. No new library dependency.

**Spec:** [run result file](../specs/2026-10-16-run-result-file.md),
[project-master-plan.md](../../project-master-plan.md) §2, §3.2–3.4, §13.

> Plan style matches the approved vertical 14-DoF plan. Each task gives exact
> interfaces, test assertions, and commands instead of full code bodies.

## Context for the reader

The user asked for one animation, exported as a video at 120 fps, with this
layout:

```text
┌───────────────────────────┬──────────────┬──────────────┐
│  whole track, xy, looking │  rear view:  │  right view: │
│  down, as a line          │  body roll   │  body pitch  │
│  + CG point on the line   │  body y axis │  body x axis │
│  + body forward axis      │  vs horizon  │  vs horizon  │
│  + velocity vector        ├──────────────┴──────────────┤
│  + acceleration vector    │  fixed top view: car's      │
│    (xy planar + z rotation)│  velocity vector always    │
│                           │  points straight up         │
│                           │  + body slip angle          │
│                           │  + tire Fx, Fy, Mz arrows   │
│                           │    in the tire frame        │
│                           │  + four Fz bars             │
│                           │  + throttle/brake/steer     │
└───────────────────────────┴─────────────────────────────┘
```

The left window spans the full height. The right window is one column: two
side-by-side blocks at the top (rear view and right view), and one larger block
below (the top view). The file list writes the right column as one
`GridSpec` with a 3-column, 2-row layout so the two top blocks sit side by side
and the top view spans the full width.

## 1. Constraints and boundaries

| # | Constraint |
|---|---|
| C1 | Another worker is active in this repository. This plan writes documentation only; implementation waits for a separate instruction. |
| C2 | The library under `src/` uses NumPy only. matplotlib, `ffmpeg`, and file parsing for STL are tools-only dependencies. |
| C3 | All rendering code lives under `tools/`. It must not import matplotlib at module import time, so `tools/anim` stays importable in a NumPy-only environment. |
| C4 | Every physical sign follows `core/conventions.py`: SAE J670, `+x` forward, `+y` right, `+z` down, positive yaw to the right, positive lateral acceleration to the right. |
| C5 | One channel source. The exporter reads the run result file and never re-evaluates a plant. |
| C6 | Rendering is deterministic: `Agg` backend, fixed camera, fixed axes limits per run, no wall-clock text, no random layout. |
| C7 | Frame rate is exactly 120 fps unless the user selects another value. The encoder fails loudly rather than silently changing the rate. |
| C8 | The exporter states on every frame when a channel is absent or approximate. It never draws a plausible value for a channel the model does not have. |
| C11 | A window whose model feature does not exist is **hidden**, not drawn empty. The remaining windows grow to use the freed space, and the figure is laid out once per run, never per frame. |
| C12 | Hiding a window is derived from channel availability, so the layout follows the record and no model-name special case exists in the renderer. |
| C9 | `MPLCONFIGDIR` is set to a writable directory before matplotlib import, following `tools/reports/plot_iem24_forward_driver_comparison.py`. |
| C10 | The default output location is inside the attempt directory next to `trace.csv`. No new file is written into a committed report directory without an explicit flag. |

## 2. Answers to the "possible ways" question

### 2.1 Available machinery, measured on 2026-10-16

| Capability | State |
|---|---|
| `ffmpeg` / `ffprobe` | Present at `/opt/homebrew/bin`. `libx264` accepted `-r 120`; a 120-frame probe reported `r_frame_rate=120/1`. |
| matplotlib | 3.11.2 installed, `Agg` works with `MPLCONFIGDIR=/tmp/...`. |
| Pillow / imageio | 12.2.0 / 2.37.3 installed, not needed for this design. |
| `imageio-ffmpeg`, `av`, `cv2` | Absent. Do not add them. |
| `pyvista`, `trimesh`, `numpy-stl` | Absent. The STL reader is written in this plan with NumPy. |
| Existing capture | `trace.csv` stores state only. It has no rate, no forces, and no moment. 12027 rows for a 73.56 s lap. |
| Existing replays | `tools/reports/plot_iem24_forward_driver_comparison.py` and `tools/diagnostics/recompute_forward_driver_accelerations.py` replay every state through the current plant. That replay is the provenance hazard this plan removes. |

### 2.2 Channel reconstruction is feasible today

A direct check on the recorded lap
`docs/validation/iem24-murphy/seven_dof_track_pair/original/final_acceptance_run6/cold/attempt_1792112759108530000`
confirmed every required quantity:

- one `evaluate_planar_dynamics` call returns `normal_loads`,
  `longitudinal_forces`, `lateral_forces`, `aligning_moments`, `slip_ratios`,
  `slip_angles`, `effective_radii`, `body_longitudinal_forces`,
  `body_lateral_forces`, `drag_force`, `yaw_moment`, the three accelerations,
  and the four wheel accelerations;
- observed ranges at sampled steps: `Fz` 74.8–1478.9 N, `|Fx|` up to 1333 N,
  `|Fy|` up to 1485 N, `|Mz|` up to 43.6 N·m; steering up to 0.186 rad,
  throttle up to 0.738, brake up to 674 N;
- body accelerations require the rotating-frame terms
  (`ax = dv_x/dt - r*v_y`, `ay = dv_y/dt + r*v_x`). At one sampled step the
  naive values were `ax = -6.27` and `ay = 11.36 m/s²` with the terms included;
- the world track position follows from the path coordinates:
  `X = x_path - n*sin(psi)`, `Y = y_path + n*cos(psi)`, with `psi` from the path
  heading. The reconstructed plan-view bounds matched the track, and the
  closure error over the recorded lap was 0.15 m, which is the track's own
  closure gap.

### 2.3 Renderer options

| Option | Quality | Speed at 120 fps | Cost | Verdict |
|---|---|---|---|---|
| matplotlib `Agg`, artist updates, pipe to `ffmpeg` | Enough for lines, arrows, bars, 2-D projections | 8 800 frames in an estimated 2–5 min | No new dependency | **Selected** |
| matplotlib `FuncAnimation` + `FFMpegWriter` | Same | Same, but the writer owns the pipe and hides failures | No new dependency | Rejected. Direct pipe gives better errors and a progress line. |
| Pillow frames, then `ffmpeg` | Same | Similar | Reimplement axes, ticks, arrows, and text | Rejected. Not worth it. |
| PyVista / VTK off-screen | True 3-D | Unknown, heavy install | New dependency, headless-GL risk | Rejected for now. Revisit only if a true 3-D camera is required. |
| matplotlib `mplot3d` | Poor perspective, slow | Slower per frame | No new dependency | Rejected. The two side views and the top view are orthographic by definition. |
| Browser/WebGL or HTML video | Best interactivity | n/a for a video file | Out of scope; the master plan defers GUI work | Rejected. |

The three vehicle views are orthographic projections, so a 3-D renderer adds
nothing. Every view is one 3-D scene in body axes projected onto a plane:

| Panel | Scene | Screen `right` | Screen `up` |
|---|---|---|---|
| Track | world `X`,`Y` | `+X` (north) | `+Y` (east) |
| Rear view | body `y`,`z` | `+y` (body right) | `-z` |
| Right view | body `x`,`z` | `+x` (body forward) | `-z` |
| Top view | body `x`,`y`, rotated so the velocity is up | `velocity right` | `velocity forward` |

### 2.4 Frame source options

| Option | Verdict |
|---|---|
| Resample the run record at 120 fps | **Selected.** The record is complete; the resampler is pure NumPy and testable. |
| Replay the plant at 120 fps | Rejected. Expensive, and it cannot reproduce transient tire states. |
| Render directly inside the integrator loop | Rejected. It couples rendering to the solve and breaks for a long solve. |
| Use the BDF dense polynomials as the frame source | Rejected as the primary path. It is BDF-only and the record must stay method-agnostic. It remains available as a cross-check. |

## 3. Panel information architecture

Each item is a required artist. `ch` names a channel from the run result
specification. §6 of that specification defines every channel.

### 3.1 Left panel — whole track, looking down

| Artist | Data | Convention |
|---|---|---|
| Track polyline | `track.path_coordinates` from the record's track XML | `X` north, `Y` east, equal aspect |
| CG marker | `ch:position-x`, `ch:position-y` | one point on the line |
| Body forward axis | unit vector `(cos psi_b, sin psi_b)` where `psi_b = ch:heading-world - ch:body-slip-angle` | body `+x` in world |
| Traveled path | the CG position over `[0, t]` | shows progress along the lap |
| Velocity vector | `speed * (cos psi_v, sin psi_v)` with `psi_v = ch:heading-world` | body velocity in world |
| Acceleration vector (planar) | `(ch:acceleration-longitudinal, ch:acceleration-lateral)` rotated by `psi_b` | body acceleration in world |
| Z-rotation indicator | `ch:yaw-acceleration` | a labelled circular arrow at the CG with a numeric readout. A rotational vector is out of the view plane, so it is not drawn as an arrow. Positive is a right turn. |

Scale rules: the velocity arrow length is `k_v * speed`, and the acceleration
arrow is `k_a` metres per m/s². Both scales are fixed for the whole run from the
run maxima, so arrow length is comparable across frames. Both scales and their
units are printed in the panel corner.

### 3.2 Right top-left block — rear view, roll

| Artist | Data | Convention |
|---|---|---|
| Horizon line | horizontal line through the view centre | the road plane, `z = 0` |
| Body `y` axis line | unit body `+y` projected into the view | see the roll expression below |
| Body outline | the body subset of the vehicle scene, projected | box or imported mesh |
| Roll readout | `ch:roll` | `roll_deg = degrees(ch:roll)` |

Roll expression. The two lines are drawn from their directions, not from a
rotated bitmap:

```text
direction(body y, screen) = ( cos(roll), -sin(roll) )      # screen right, screen up
direction(horizon, screen) = ( 1,          0          )
```

The screen-right unit vector is body `+y` and the screen-up unit vector is body
`-z`, because the view looks along body `+x`. Then a positive roll (right side
down, which is the SAE sense) rotates the body `+y` line clockwise on screen,
and the angle between the two lines equals `roll`. A straight-ahead case is the
zero-roll reference: the body `y` line is parallel to the horizon.

### 3.3 Right top-right block — right view, pitch

| Artist | Data | Convention |
|---|---|---|
| Horizon line | horizontal line through the view centre | the road plane |
| Body `x` axis line | unit body `+x` projected into the view | see the pitch expression below |
| Body outline | the body subset of the vehicle scene, projected | box or imported mesh |
| Pitch readout | `ch:pitch` | `pitch_deg = degrees(ch:pitch)` |

Pitch expression:

```text
direction(body x, screen) = ( cos(pitch),  sin(pitch) )    # screen right, screen up
direction(horizon, screen) = ( 1,         0         )
```

The screen-right unit vector is body `+x` and the screen-up unit vector is body
`-z`, because the view looks along body `+y`. Positive pitch is **nose up**: the
repository's 14-DoF plant fixes "pitch positive nose up (`x` forward)" in
`formulations/vertical_14dof.py`, which is also the right-handed rotation about
`+y` with `z` down, `x -> (cos p, 0, -sin p)`. The body `x` line therefore rises
towards the front for positive pitch. Heave is positive **down**, and roll is
positive right side down.

Both blocks draw the ground as a short thick tick under the wheel contact line so
the horizon is unambiguous, and both freeze their scale for the whole run.

### 3.4 Right bottom block — fixed top view

The view frame is anchored to the **velocity direction**, not to the body and not
to the world:

```text
screen right  = normalize( velocity ) rotated -90 degrees   # world
screen up     = normalize( velocity )                       # world
```

Consequences:

1. The velocity vector is always drawn straight up.
2. The body is rotated by `-slip` on screen, where `slip = ch:body-slip-angle`.
   The body slip angle is therefore visible, and it is also drawn explicitly as
   the angle between a fixed screen-vertical reference and the body `+x` line,
   matching the roll and pitch blocks.
3. The grid and every tire arrow rotate with the velocity direction.

| Artist | Data | Convention |
|---|---|---|
| Vehicle footprint | body outline in body axes, transformed | box or mesh, from `vehicle_geometry.wheel_positions_body_m` |
| Velocity reference | fixed screen-vertical arrow from the CG | unit length, labelled `v` |
| Body slip angle | angle between the vertical reference and the body `+x` line, with an arc and a readout | `slip_deg = degrees(ch:body-slip-angle)` |
| Tire `Fx` arrow | `ch:tire-longitudinal-force` | **tire frame:** `+Fx` along the wheel's forward axis, which is the body `+x` axis rotated by `ch:wheel-steering-angle` |
| Tire `Fy` arrow | `ch:tire-lateral-force` | **tire frame:** `+Fy` along the wheel's right axis, which is the body `+y` axis rotated by the same angle |
| Tire `Mz` symbol | `ch:tire-aligning-moment` | a circular arrow centred on the wheel contact patch; clockwise on screen is a positive moment under SAE |
| `Fz` bars | `ch:tire-vertical-force` | four bars beside the vehicle, one per wheel, zero at the bottom. The full-scale value is fixed per run from the run maximum, and the scale in newtons is printed |
| Throttle overlay | `ch:throttle` | horizontal bar, 0–1 |
| Brake overlay | `ch:brake-fraction` | horizontal bar, 0–1 |
| Steer overlay | `ch:steering` | centre-zero bar, symmetric full scale from the run maximum, with a numeric readout in radians and degrees |

The tire-frame statement above needs one verification step, because a wrong
rotation direction is easy to miss visually. The test is: take a recorded state
with a nonzero steering angle, recompute
`body_longitudinal_forces` from the tire-frame forces with
`rotate_tire_forces_to_body`, and compare it with the recorded body forces.
`planar_7dof.py` states the relation as
`body_longitudinal = Fx*cos(delta) - Fy*sin(delta)` and
`body_lateral = Fx*sin(delta) + Fy*cos(delta)`, so the tire axes are the body
axes rotated by `delta` with no additional sign flip.

### 3.5 Vehicle scene

One scene description drives all three vehicle panels:

```python
@dataclass(frozen=True)
class VehicleScene:
    """Store one rigid 3-D body in body axes, in metres."""
    triangles: np.ndarray   # (n, 3, 3): corner points, body axes, +x fwd, +y right, +z down
    wheels: tuple[tuple[np.ndarray, np.ndarray], ...]  # per wheel: triangles, body origin
```

- The placeholder is one box for the sprung body plus one box or cylinder per
  wheel. The body box uses `wheelbase`, `track_width`, and `cg_height` from
  `vehicle_geometry`, plus an assumed width and height that the CLI exposes.
- The body box is placed at the sprung centre of gravity when the model has one
  (`vehicle_geometry.sprung_cg_height_m`). When it is `null`, the box is placed
  at `cg_height_m` and the frame says "rigid body, no attitude state".
- An imported STL replaces the body box. `--stl-body PATH` and
  `--stl-wheel PATH` are the only new inputs. The reader accepts both ASCII and
  binary STL, returns triangles in the file's own coordinates, and the CLI
  applies one scale and one translation, both of which are printed on the frame
  and in the run summary. **The STL is a display asset. It never enters the
  physics and it never changes a simulation number.**
- Wheels rotate about their own `+y` axis by the integrated wheel angle. The
  record stores wheel speed, and the resampler integrates it from the segment
  start. Wheel rotation is a display effect; it is stated as such.

## 4. File map

| File | Responsibility |
|---|---|
| `src/simplified_vehicle_models/results/run_record.py` | `CHANNEL_CATALOGUE`, `RunRecorder`, `load_run_record`, `resample_run_record`. NumPy only. |
| `src/simplified_vehicle_models/results/monotone_spline.py` | local C1 monotone interpolator if `suspension/monotone_spline.py` is not yet present. Deleted when the suspension module lands. |
| `src/simplified_vehicle_models/solvers/forward_driver.py` | one optional `step_sink` argument on `solve_forward`; each plant exposes `last_evaluation`. |
| `src/simplified_vehicle_models/solvers/planar_forward.py` | store `last_evaluation`; add `planar_channels(vehicle, plan, state, control, evaluation)` that returns the catalogue rows. |
| `src/simplified_vehicle_models/solvers/vertical_forward.py` | the same two additions for the 14-DoF plant (after the vertical plan lands). |
| `tools/run/run_iem24_forward_driver.py` | `--record`, `--record-precision`, and the record metadata block. |
| `tools/anim/__init__.py` | package marker. No matplotlib import. |
| `tools/anim/resample.py` | frame grid construction and the resampling policy from record specification §7. |
| `tools/anim/stl.py` | ASCII and binary STL reader, NumPy only. |
| `tools/anim/scene.py` | `VehicleScene`, the box builder, the transform, and the orthographic projection. |
| `tools/anim/panels.py` | four panel builders: track, rear, right, top. Each returns its artists and an `update(frame)` callable. |
| `tools/anim/render.py` | figure and `GridSpec` assembly, `Agg` setup, artist update loop, frame writer protocol. |
| `tools/anim/encode.py` | raw RGB pipe to `ffmpeg`, `--dry-run` probe, PNG-sequence fallback. |
| `tools/anim/export_animation.py` | CLI. |
| `docs/dev/animation-export.md` | user-facing command reference and the frame legend. |
| `tests/test_run_record.py` | record write, read, and validation checks V1–V12. |
| `tests/test_animation_frames.py` | resampling, sign conventions, and panel channel mapping. |
| `tests/test_animation_stl.py` | STL parsing and the orthographic projection. |
| `tests/test_animation_encode.py` | an `ffprobe` assertion on a short real encode, skipped when `ffmpeg` is absent. |

## 5. Tasks

### Task 1: Run result file core

**Files:**
- Create: `src/simplified_vehicle_models/results/run_record.py`
- Create: `src/simplified_vehicle_models/results/monotone_spline.py` (conditional, see §4)
- Test: `tests/test_run_record.py`

**Interfaces (Produces):**
- `RunRecordManifest` — a frozen dataclass that mirrors the JSON in record
  specification §4 and round-trips through `to_json` / `from_json`.
- `RunRecorder(path, state_names, state_units, control_names, precision="float32")`
  with `.append(time, step_index, state, state_rate, control, channel)` and
  `.close(metadata) -> Path`. It buffers `chunk_rows = 4096` rows and writes each
  chunk as a separate NPZ member through `zipfile.ZipFile(mode="a")`, so a long
  run never holds the whole capture in memory.
- `load_run_record(path) -> RunRecord` with fields `manifest`, `time`,
  `step_index`, `state`, `state_rate`, `control`, `channel`, `channel_valid`.
  It validates V1–V6 and V10 and raises `RunRecordError` with the failing check
  name.
- `resample_run_record(record, frame_rate) -> FrameSet` where
  `FrameSet.time`, `.state`, `.state_rate`, `.control`, `.channel`,
  `.channel_valid`, and `.source_step` hold the grid from record specification
  §7. The state uses monotone cubic Hermite interpolation; the rate, the
  control, and the channels use zero-order hold.
- `CHANNEL_CATALOGUE`: a mapping from channel name to
  `ChannelSpec(name, shape, unit, frame, description)`, containing exactly the
  channels in record specification §6.

- [ ] **Step 1: Write failing tests.**
  - A synthetic record with 3 states and 5 channels round-trips: names, units,
    rows, and precision match.
  - `resample_run_record(record, 2.0)` on a record with `time = [0, 0.4, 1.0]`
    returns frames at `0, 0.5, 1.0`; held channels return the value of the last
    knot at or before the frame time; the interpolated state at a knot equals the
    knot exactly.
  - A cubic test function `f(t) = t**3 - t` sampled at uneven knots is
    reproduced by the state resampler within `1e-9` at 100 interior points, and
    the resampler never overshoots between two equal knots.
  - A record with `maximum_step > 1/120` sets `manifest.coarse_for_120_fps`.
  - A nan in `state` raises `RunRecordError("V6")`; a truncated array raises
    `RunRecordError("V2")`; a source hash that disagrees with the working file
    raises `RunRecordError("V10")` unless `verify_sources=False`.
  - Two captures of the same synthetic run are byte-identical apart from
    `created_utc`, `producer.version`, and `integrator.wall_seconds`.
  - `np.load` with `allow_pickle=False` succeeds on the written container.
- [ ] **Step 2:** Run `PYTHONPATH=src python3 -m unittest tests.test_run_record -q`. Expected: FAIL (module missing).
- [ ] **Step 3:** Implement. NumPy only. Keep every array `read-only` after load, following the damper-table precedent.
- [ ] **Step 4:** Run the same command, then the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add shared run result file`.

### Task 2: Plant channel export

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/planar_forward.py`
- Modify: `src/simplified_vehicle_models/solvers/forward_driver.py`
- Test: `tests/test_run_record_channels.py`

**Interfaces (Produces):**
- `PlanarForwardPlant.last_evaluation: PlanarEvaluation | None`, set at the end
  of every `evaluate` call. No extra physics, no extra cost.
- `planar_channels(vehicle, plan, state, control, evaluation) -> dict[str, np.ndarray]`
  returning the catalogue rows that the planar model can supply. It computes the
  world `position-x`/`position-y` from `plan.path_coordinates` and the body slip
  angle `atan2(v_y, v_x)`.
- `solve_forward(..., step_sink=None)`. When `step_sink` is not `None`, it is
  called with `StepSample(time, state, control, rates)` at each accepted step.
  It is called from the same place that already records `result.time` and
  `result.state`, so it can never see a rejected trial. `step_sink=None`
  preserves the current signature and every current result bit for bit.
- `RunRecorder.from_plant(plant, path, driver, ...)` that resolves the control
  for each step by replaying `driver.command(plant.observe(state))` and the
  recorded pedal position, which is exactly what the existing comparison tool
  does, but without re-evaluating the physics.

- [ ] **Step 1: Write failing tests.**
  - `step_sink` receives exactly `len(result.time)` samples, and their times and
    states equal `result.time` and `result.state.T`.
  - `solve_forward` with `step_sink=None` returns results bitwise equal to the
    recorded fixture, so the change is provably additive.
  - The channel dict for a recorded state contains `tire-vertical-force` with
    shape `(4,)`, and every value equals the matching field of
    `last_evaluation`.
  - `tire-force-in-body-x` recomputed from the tire-frame forces with
    `rotate_tire_forces_to_body` matches `evaluation.body_longitudinal_forces`
    to `1e-9` relative. This is the tire-frame verification from §3.4.
  - `body-slip-angle` has the expected sign for a synthetic state with
    `v_x = 10`, `v_y = +1`.
  - A count of `evaluate` calls during a `step_sink` run equals the count during a
    run without `step_sink`, so recording adds no plant evaluation.
- [ ] **Step 2:** Run `PYTHONPATH=src python3 -m unittest tests.test_run_record_channels -q`. Expected: FAIL.
- [ ] **Step 3:** Implement. Add `last_evaluation` in `evaluate`, and the
  `step_sink` call in `solve_forward`. Do not change any rate.
- [ ] **Step 4:** Run the same tests, then `tests.test_planar_forward`,
  `tests.test_forward_driver_runner`, and the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: export plant channels for a run record`.

### Task 3: Runner integration

**Files:**
- Modify: `tools/run/run_iem24_forward_driver.py`
- Test: `tests/test_forward_driver_runner.py` (extend)

**Interfaces (Produces):**
- CLI: `--record` writes `run_record.npz` inside the attempt directory;
  `--record-precision {float32,float64}` defaults to `float32`;
  `--record-frame-rate FLOAT` records the intended animation rate in the
  metadata and sets the coarse-capture flag when
  `maximum_step > 1 / record_frame_rate`.
- The record writes into the attempt directory beside `trace.csv`,
  `pedal.csv`, and `summary.json`. `summary.json` gains
  `run_record` = the file name, `run_record_rows`, and
  `run_record_channels`.
- Exactly one record attempt guard: a second `--record` run into the same
  attempt directory overwrites, which matches the existing runner behaviour for
  `trace.csv`.

- [ ] **Step 1: Write failing tests.**
  - With a fake plant, `--record` writes a container whose `manifest.state_names`
    equals the plant's `state_names` and whose rows equal
    `len(result.time)`.
  - Without `--record`, no `run_record.npz` is written and every existing
    summary key and value is unchanged.
  - `--record-precision float16` is rejected by the argument parser.
  - The recorded `source_hashes` equal the current `summary.json`
    `source_hashes`.
  - `--record --record-frame-rate 120 --maximum-step 0.02` sets
    `coarse_for_120_fps = true`; `--maximum-step 0.005` does not.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver_runner -q`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the same tests and the full suite. Expected: PASS.
- [ ] **Step 5:** Run one recorded lap to produce a real record for later tasks:
  `PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 --duration 300.0 --timeout-seconds 300.0 --record --record-frame-rate 120.0 --output-root /tmp/iem24-animation-record`.
- [ ] **Step 6:** Commit: `feat: record the shared run result file from the forward runner`.

### Task 4: STL reader and orthographic scene

**Files:**
- Create: `tools/anim/__init__.py`
- Create: `tools/anim/stl.py`
- Create: `tools/anim/scene.py`
- Test: `tests/test_animation_stl.py`

**Interfaces (Produces):**
- `read_stl(path) -> np.ndarray` of shape `(n, 3, 3)`, float64, body-agnostic
  coordinates. It detects ASCII by the leading `solid` token **and** a
  successful parse; it falls back to binary on any parse failure, because a
  binary file may also begin with `solid`. Binary records are read with the
  50-byte triangle stride (12 floats plus a 2-byte attribute count). It raises
  `StlError` on truncation, a non-finite normal, or a triangle count mismatch.
- `bounding_box(triangles) -> tuple[np.ndarray, np.ndarray]`.
- `box_triangles(sizes: tuple[float, float, float]) -> np.ndarray` centred on the
  origin.
- `wheel_cylinder_triangles(radius, width, segments=24) -> np.ndarray` with its
  axis along `+y`.
- `transform(triangles, rotation, translation) -> np.ndarray`.
- `project(triangles, right, up) -> np.ndarray` of shape `(n, 3, 2)`, an
  orthographic projection onto `right` and `up`, both expressed in body axes.
- `body_rotation(yaw, pitch, roll) -> np.ndarray`, an SAE-ordered rotation about
  `+z`, then `+y`, then `+x` (yaw, then pitch, then roll).
- `VIEWS`: the four `(right, up)` pairs from §2.3, as unit vectors in body axes
  for the three vehicle views, plus the fixed world pair for the track.

- [ ] **Step 1: Write failing tests.**
  - A hand-written ASCII STL with one triangle parses to that triangle exactly.
  - A binary STL with two triangles parses to those triangles exactly, including the case where the file starts with the bytes `solid`.
  - A truncated binary file raises `StlError`.
  - `project(box_triangles((2, 1, 1)), right, up)` for the rear view
    (`right = +y`, `up = -z`) returns `y` and `-z` coordinates; for the right
    view (`right = +x`, `up = -z`) returns `x` and `-z`.
  - `body_rotation(0, 0, roll) @ [0, 1, 0]` equals `[0, cos roll, sin roll]`, so a
    positive roll moves the body `+y` axis towards `+z`, which is down. The
    rear-view projection of that axis is then `(cos roll, -sin roll)`, a
    clockwise rotation on screen.
  - `body_rotation(0, pitch, 0) @ [1, 0, 0]` equals `[cos pitch, 0, -sin pitch]`,
    so a positive pitch moves the body `+x` axis towards `-z`, which is up (nose
    up, matching the 14-DoF plant). The right-view projection of that axis is
    `(cos pitch, +sin pitch)`.
  - The composition order is yaw about `+z`, then pitch about `+y`, then roll
    about `+x`: `body_rotation(0, 0, 0)` is the identity, and
    `body_rotation(yaw, 0, 0) @ [1, 0, 0]` equals `(cos yaw, sin yaw, 0)`, so a
    positive yaw turns the body to the right.
  - `wheel_cylinder_triangles` has its axis along `+y` and radius within `1e-12`
    of the request.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_animation_stl -q`. Expected: FAIL.
- [ ] **Step 3:** Implement. NumPy only, no matplotlib import.
- [ ] **Step 4:** Run the same tests and the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add STL reader and orthographic vehicle scene`.

### Task 5: Frame resampling and channel access

**Files:**
- Create: `tools/anim/resample.py`
- Test: `tests/test_animation_frames.py` (first half)

**Interfaces (Produces):**
- `FrameSource(record, frame_rate, channel_names)` with:
  - `.times: np.ndarray` — the frame grid;
  - `.state_at(index) -> np.ndarray`;
  - `.channel_at(name, index) -> np.ndarray | None`, returning `None` when the
    channel is absent and a masked array or a `valid` flag when the sample is
    invalid;
  - `.scales() -> dict[str, float]` — the fixed full-scale values for the
    velocity arrow, the acceleration arrow, the steering bar, the `Fz` bars, and
    the tire-force arrows, computed once from the whole run;
  - `.warnings: tuple[str, ...]` — the human-readable omissions, for example
    `"roll: not available in planar-7dof; showing roll-quasi-static"`.
- `FrameSource.require(name)` raises `MissingChannelError` when a required
  channel is absent, so the exporter fails loudly instead of drawing a zero.

- [ ] **Step 1: Write failing tests.**
  - A synthetic record with 120 fps frames produces
    `floor((t_end - t_0) * 120) + 1` frames, and the last grid time is at or
    before `t_end`.
  - Every frame's `source_step` index is nondecreasing and its recorded time is
    at or before the frame time.
  - `channel_at` for an invalid sample returns the invalid flag, not a value.
  - `require("roll")` on a planar record raises `MissingChannelError`, and
    `warnings` names the quasi-static substitute.
  - `scales()` is identical for two calls and does not depend on frame index.
  - The velocity arrow length at the run maximum speed equals the declared
    maximum arrow length.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_animation_frames -q`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the same tests. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add animation frame source`.

### Task 6: Panels

**Files:**
- Create: `tools/anim/panels.py`
- Test: `tests/test_animation_frames.py` (second half)

**Interfaces (Produces):**
- `build_track_panel(axes, frame_source, track_geometry) -> Panel` where
  `Panel.update(index)` sets artist data only and never creates an artist.
- `build_attitude_panel(axes, frame_source, scene, view) -> Panel` for `rear`
  and `pitch` (right view). `view` selects the two axis lines and the readout
  label.
- `build_top_panel(axes, frame_source, scene) -> Panel`.
- Every panel exposes `.legend_lines() -> tuple[str, ...]`, the exact text it
  prints in its corner, so a test can assert the units and the scale.
- A panel never calls `axes.clear()`, `set_xlim`, or `set_ylim` after
  construction, so a frame is a pure data update and the camera is fixed.

- [ ] **Step 1: Write failing tests.** Use a recorded real lap, not synthetic
  data, for the channel mapping.
  - `build_track_panel` places the CG marker at `position-x`, `position-y` and
    the body forward axis at `heading - slip`, so a synthetic frame with
    `slip = 0.1 rad` rotates the axis by exactly `-0.1 rad` on screen.
  - The rear panel's two line directions match §3.2 for `roll = 0`,
    `roll = +0.05`, and `roll = -0.05` radians, to `1e-12`.
  - The right panel's two line directions match §3.3 for the same three pitch
    values.
  - The top panel keeps the velocity arrow exactly vertical in screen
    coordinates for every frame of the recorded lap, to `1e-12`.
  - The top panel's body `+x` line is rotated from screen vertical by
    `-body-slip-angle` to `1e-12`.
  - The tire-frame `Fx` and `Fy` arrows for `steering = +0.2 rad` are rotated
    by `+0.2 rad` from the body axes in the screen frame.
  - `legend_lines()` for the top panel contains the units `N`, `N*m`, and the
    `Fz` full-scale value in newtons.
  - A planar record whose only attitude channels are the quasi-static pair
    reports `rear` and `right` as hidden by default, and produces exactly one
    warning that names the substitute. With `--quasi-static-attitude` it renders
    both attitude blocks and labels them approximate. No `MissingChannelError` is
    raised in either case.
  - A record with no attitude channel at all produces `window_visibility` with
    `rear` and `right` both false, and `build_attitude_panel` is never called.
  - A record with no `tire-longitudinal-force` channel produces
    `tire-arrows=false` but leaves the top view visible.
  - Rendering every frame of a 5 s excerpt twice produces identical artist data,
    so the renderer is deterministic.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_animation_frames -q`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run the same tests and the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add animation panels`.

### Task 7: Renderer and encoder

**Files:**
- Create: `tools/anim/render.py`
- Create: `tools/anim/encode.py`
- Test: `tests/test_animation_encode.py`

**Interfaces (Produces):**
- `FrameWriter` protocol with `.write(rgb: np.ndarray) -> None` and `.close()`.
- `FfmpegFrameWriter(path, width, height, frame_rate, crf=18, preset="medium")`
  that starts one `ffmpeg` process with
  `-f rawvideo -pix_fmt rgb24 -s WxH -r FPS -i - -an -c:v libx264 -preset P -crf C -pix_fmt yuv420p -movflags +faststart OUT`.
  It raises `EncoderUnavailable` when `ffmpeg` is missing or exits nonzero, and
  it reports the `ffmpeg` stderr tail in the error.
- `PngSequenceWriter(directory)` as the deterministic fallback.
- `NullFrameWriter()` for tests. It records only the frame count and a digest of
  the frame bytes.
- `AnimationStyle` — one frozen dataclass holding figure size, dpi, colours,
  line widths, font size, and arrow scale fractions.
- `WindowVisibility` — a frozen dataclass with boolean fields `track`, `rear`,
  `right`, `top`, `tire_arrows`, `fz_bars`, and `attitude_is_dynamic`, plus
  `.hidden: tuple[str, ...]` and `.notes: tuple[str, ...]`.
- `window_visibility(record) -> WindowVisibility`, a pure function of channel
  availability:
  - `rear` and `right` are true for a dynamic attitude channel (`roll` and
    `pitch`). They are false when only the quasi-static pair exists, unless the
    caller passes `allow_quasi_static=True`;
  - `top` is true whenever the record carries any wheel, tire, or control
    channel; it is false only for a pure scalar point-mass record;
  - `tire_arrows` requires `tire-longitudinal-force` and `tire-lateral-force`;
  - `fz_bars` requires `tire-vertical-force`;
  - `attitude_is_dynamic` is true only when `roll` is a state-derived channel.
- `window_grid(visibility, style) -> GridSpec`, a pure function that returns the
  figure grid for the visible windows. It is the only place that decides
  geometry, so the layout is unit-testable without rendering.
- `build_figure(record, frame_source, scene, track_geometry, style) -> (figure, panels, visibility)`
  that builds only the visible windows, in the order `track`, `rear`, `right`,
  `top`, and returns the visibility it used.
- `render_frames(figure, panels, frame_source, writer, *, progress=None)` that
  updates each panel in order, calls `figure.canvas.draw()` and
  `figure.canvas.buffer_rgba()`, converts to RGB, and writes. `progress` is
  called with `(index, total)` at most once per second of wall time.

- [ ] **Step 1: Write failing tests.**
  - `NullFrameWriter` receives exactly the frame count from §7.1.
  - `window_visibility` returns the expected flags for the four synthetic
    records from Task 10.
  - `window_grid` for each of the layout rows in §6 returns axes whose bounding
    boxes match the expected regions to `1e-9`: in the full layout the track axis
    spans the full figure height; with both attitude windows hidden the top-view
    axis spans the full figure height; with the top view hidden the two attitude
    axes span the full figure height side by side; with only the track visible
    the track axis fills the figure.
  - `build_figure` never creates an artist for a hidden window, so a point-mass
    record produces exactly one axes.
  - Character `"ffmpeg"` missing from `PATH` raises `EncoderUnavailable` with a
    message that names `ffmpeg`.
  - A real encode of 24 frames at 320×240, 120 fps, written to a temporary path,
    is read back with `ffprobe -show_entries stream=r_frame_rate,nb_frames` and
    asserts `120/1` and `24`. The test skips when `ffmpeg` or `ffprobe` is
    absent.
  - `render_frames` on a 10-frame excerpt returns a `NullFrameWriter` with 10
    frames and does not raise.
  - The writer's `write` receives an array whose shape is `(height, width, 3)`
    and `dtype == uint8`.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_animation_encode -q`. Expected: FAIL.
- [ ] **Step 3:** Implement. Set `MPLCONFIGDIR` and `matplotlib.use("Agg")` before
  the first pyplot import, following the existing tool.
- [ ] **Step 4:** Run the same tests and the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add animation renderer and ffmpeg encoder`.

### Task 8: CLI and documentation

**Files:**
- Create: `tools/anim/export_animation.py`
- Create: `docs/dev/animation-export.md`
- Modify: `docs/dev/tools-layout.md` (add a row for the new `tools/anim/` category)

**Interfaces (Produces):**
- CLI:
  `python3 tools/anim/export_animation.py --record PATH --output PATH [--frame-rate 120.0] [--width 1920] [--height 1080] [--crf 18] [--preset medium] [--stl-body PATH] [--stl-wheel PATH] [--body-length M] [--body-width M] [--body-height M] [--wheel-radius M] [--wheel-width M] [--transform-scale M] [--transform-x M] [--transform-z M] [--time-range T0 T1] [--png-sequence DIR] [--force-top-view] [--quasi-static-attitude] [--dry-run] [--progress]`.
- `--dry-run` prints the frame count, the estimated output size, the channel
  list, the visible-window list, the hidden-window list with its reason, and
  every warning, then exits without rendering.
- `--frame-rate` defaults to 120.0 and rejects a non-finite value or a value at
  or below zero.
- `--force-top-view` overrides `WindowVisibility.top` only. It never adds a
  channel, so a forced window still states which channels it lacks.
- `--quasi-static-attitude` overrides `WindowVisibility.rear` and `.right` only,
  and only when the record provides `roll-quasi-static` and
  `pitch-quasi-static`. It never synthesizes an attitude channel, so it has no
  effect on a record without them.
- The CLI prints, once, every `FrameSource.warnings` line, the visibility notes,
  and the scale legend that the panels print on the frames.
- `docs/dev/animation-export.md` states the frame legend, the sign conventions
  from §3, the STL policy from §3.5, the window-availability table from §6.1,
  and the exact commands.

- [ ] **Step 1: Write failing tests.**
  - `--dry-run` on a real record prints a frame count that equals
    `floor((t_end - t_0) * 120) + 1` for the full range.
  - `--time-range 10 12` gives 241 frames at 120 fps.
  - `--frame-rate 0` is rejected with a message that names the argument.
  - `--record` pointing at a missing file exits with a nonzero status and a
    clear message.
  - The dry run reports the planar attitude warning exactly once.
  - The dry run on a point-mass record lists `rear`, `right`, and `top` as
    hidden with a reason, and `--force-top-view` moves `top` to the visible list.
  - The dry run on a planar record lists `rear` and `right` as hidden; adding
    `--quasi-static-attitude` moves both to the visible list and prints the
    approximate label. On a point-mass record the same flag changes nothing.
  - A `--dry-run` image is never written; the output path is untouched.
- [ ] **Step 2:** Run `PYTHONPATH=src:. python3 -m unittest tests.test_animation_encode -q`. Expected: FAIL.
- [ ] **Step 3:** Implement the CLI and the documentation.
- [ ] **Step 4:** Run the same tests and the full suite. Expected: PASS.
- [ ] **Step 5:** Commit: `feat: add animation export CLI`.

### Task 9: Full export and performance report

**Files:**
- Create: `docs/validation/iem24-murphy/animation-export.md`
- Create (generated, not committed by default):
  `docs/validation/iem24-murphy/forward_driver_trials/<attempt>/animation.mp4`

**Deliverable:** one 120 fps video of the recorded IEM24 lap, plus a short report.

- [ ] **Step 1:** Export the full lap from the record produced in Task 3.
- [ ] **Step 2:** Verify with `ffprobe`: `r_frame_rate == 120/1`, the frame count
  equals the dry-run count, the duration equals
  `frame_count / 120` to `0.01 s`, and the stream is H.264 `yuv420p`.
- [ ] **Step 3:** Extract four stills at the run start, peak `Fz`, peak `|ay|`,
  and the lap end. Embed them in the report.
- [ ] **Step 4:** Record the render-to-real-time ratio, the render wall time,
  the run duration, the per-frame time, the output size, the frame count, the
  encoder settings, and the machine. State the measured and estimated figures
  separately. If the ratio exceeds 12×, report the bottleneck and do not claim a
  smaller ratio.
- [ ] **Step 5:** Write `docs/validation/iem24-murphy/animation-export.md` with:
  the record digest, the command, the channel table with units, the warnings, the
  four stills, the `ffprobe` output, and a **Caveats** section that states that
  the planar attitude is quasi-static and that the STL is a display asset.
- [ ] **Step 6:** Run the full suite. Expected: PASS.
- [ ] **Step 7:** Commit: `docs: record the IEM24 animation export`.

### Task 10: Cross-model verification

**Files:**
- Test: `tests/test_animation_models.py`

**Interfaces:** No new production interface. This task proves the exporter is
model-agnostic and states exactly what each model can show.

- [ ] **Step 1:** Build a minimal synthetic record for each of the four model
  classes with a correct state layout and channel set: point-mass,
  planar-7dof, body-10dof, and 14dof-simple.
- [ ] **Step 2:** Export a 1 s / 120-frame excerpt of each with
  `NullFrameWriter`.
- [ ] **Step 3:** Assert the exact window-availability matrix from §6.1:
  point-mass has no wheel force channels and therefore fails
  `require("tire-longitudinal-force")`, and `window_visibility` reports `rear`,
  `right`, and `top` as hidden; planar-7dof has forces and no dynamic attitude,
  so `rear` and `right` are hidden unless the record also carries the
  quasi-static pair and the flag is set, and `attitude_is_dynamic` is false;
  body-10dof and 14dof-simple have a dynamic attitude, so all four windows are
  visible and `attitude_is_dynamic` is true.
- [ ] **Step 4:** Assert that for each model the produced figure has exactly the
  visible windows as axes, and that the freed space follows the §6.2 rules.
- [ ] **Step 5:** Assert that the renderer's warning text is specific and names
  the missing channel, the hidden window, and the model class.
- [ ] **Step 6:** Run the full suite. Expected: PASS.
- [ ] **Step 7:** Commit: `test: verify the animation exporter across models`.

## 6. Degradation: hide the window, grow the rest

A window exists only when its model feature exists. A missing feature removes the
window; the remaining windows grow to fill the space. The figure is laid out once
per run from `window_visibility`, never per frame.

### 6.1 Availability per model

| Model | Track window | Rear roll window | Right pitch window | Top view |
|---|---|---|---|---|
| Point mass | shown | **hidden** | **hidden** | **hidden** by default. `--force-top-view` shows it with the speed vector, the body line, and the inputs only |
| Planar 7-DoF | shown | **hidden** by default; shown from the quasi-static channels with `--quasi-static-attitude`, labelled approximate | same as the rear window | shown: tire arrows, `Fz` bars, slip angle, inputs |
| Body 10-DoF | shown | shown, dynamic | shown, dynamic | shown |
| Full 14-DoF | shown | shown, dynamic, plus a heave readout | shown, dynamic | shown, plus the transient tire readouts |

The planar default follows the standing rule: a feature the model does not have
is hidden. `--quasi-static-attitude` is an explicit opt-in, and the window then
carries an `approximate` label on every frame.

A model with attitude but no wheel data, which is not one of the four models
today, hides the top view for the same reason the point mass does.

### 6.2 Layout rules

`window_grid(visibility, style)` returns exactly one of these grids:

| Visible windows | Layout | The top view |
|---|---|---|
| `track`, `rear`, `right`, `top` | 2 rows × 3 columns: track spans both rows in column 0; rear and right occupy row 0 in columns 1 and 2; top drains both columns in row 1 | bottom right block |
| `track`, `rear`, `right` | 1 row × 3 columns: track in column 0, rear in column 1, right in column 2 | absent, so the attitude blocks take the full figure height |
| `track`, `top` | 1 row × 2 columns: track on the left, top on the right | full figure height |
| `track` only | 1 row × 1 column | absent |

Sub-rules:

1. The track window always spans the full figure height, because it is the
   navigation reference for every other window.
2. The freed height and width go to the top view first, because it carries the
   most information. The attitude blocks receive freed space only when the top
   view is itself hidden.
3. Within the top view, the sub-features degrade independently:
   `tire_arrows=false` removes the four force and moment arrows and keeps the
   vehicle, the slip angle, the `Fz` bars, and the inputs;
   `fz_bars=false` removes the four bars. A missing sub-feature is named in that
   window's legend text, not silently dropped.
4. `--force-top-view` overrides `top=false` for a point-mass record that still
   carries control and load channels. The window then states on its face which
   channels are missing.
5. The aspect ratios are set once. The top view keeps its equal aspect inside
   whatever rectangle it receives, so growing the rectangle adds margin rather
   than distorting the vehicle.

### 6.3 Frame text

Every window carries a legend line built from `legend_lines()`. A hidden window
adds a note to the figure-level footer, for example
`"rear/right hidden: planar-7dof has no dynamic roll or pitch state; pass --quasi-static-attitude to show an estimate"`,
so a viewer always knows what is absent rather than assuming the car does not
roll.

## 7. Performance

The measured IEM24 lap is 73.56 s, so a full-lap video is 8 829 frames at
120 fps.

A prototype of this exact four-panel figure was timed on 2026-10-16 at
1920×1080, using artist-data updates and `buffer_rgba` reading, with the artist
set from §3: the track polyline, the CG marker, the traveled path, and three
world-frame arrows; two attitude panels each with a horizon and an axis line;
and the top panel with the vehicle patch, the velocity reference, four tire
arrows, four `Fz` bars, and the input readout.

| Measurement | Value |
|---|---|
| Frames timed | 60 |
| Video duration of that sample | 0.500 s at 120 fps |
| Render wall time of that sample | 2.15 s |
| **Render-to-real-time ratio** | **4.30×** |
| Per-frame draw plus buffer read | 35.9 ms |
| Frames rendered per wall second | 27.9 |
| Video produced per render minute | 14.0 s |
| Pixel array produced | `(1080, 1920, 3)` `uint8` |

The ratio is the useful figure, because it does not depend on the length of the
run. The lap is 73.56 s of simulated time, which holds 8 828 frames, so at 4.30×
that lap renders in 316 s = **5.3 min**. The 5.3 min and the ratio are the same
measurement:

```text
5.27 min render / 73.56 s of run = 4.30x real time
```

The 4.30× figure is the four-window layout, which is the worst case. A layout
with hidden windows has fewer artists and draws no slower. Task 9 measures the
real ratio per layout instead of assuming the saving.

That projection excludes the `ffmpeg` time, which is small against the draw, so
the expected ratio for a full export is 4.3–6× real time at 1920×1080. The plan
does not assume a smaller figure:

- `render_frames` updates existing artists. It never rebuilds the figure and
  never calls `axes.clear()`.
- The frame loop reuses one `FigureCanvasAgg` buffer.
- Encoding is one long-lived `ffmpeg` process, so the encoder cost is small
  against the draw cost.
- Every text artist is updated through `set_text`, never recreated.
- `quiver` artists are reused through `set_offsets` and `set_UVC`. If a later
  profile shows `quiver` recreation dominating, that is a local fix that does
  not change this design.
- Task 9 records the measured render-to-real-time ratio and wall time for the
  real export. If that ratio exceeds 12×, the report says so and a follow-up plan
  may add `--frame-chunk` multiprocessing, which is safe because frames are
  independent and the encoder accepts an image sequence.

Resolution is the largest lever: 1280×720 for a fast export and 1920×1080 for a
final export. The measured ratio applies to any run length, so a 10 s excerpt
renders in about 43 s and a full lap in about 5.3 min.

## 8. Risks

| # | Risk | Mitigation |
|---|---|---|
| K1 | A sign error in a projected axis line is invisible in a still | Tasks 4 and 6 assert every projection against a synthetic angle before any video exists. |
| K2 | The tire-frame arrow rotation is wrong by one sign | Task 2 verifies the tire-frame forces against the plant's own body-force output. |
| K3 | The planar model has no dynamic attitude, so the roll and pitch windows would be empty or misleading | The windows are hidden by default. `--quasi-static-attitude` shows a labelled approximate estimate when the recorder provides the channels. Record-specification decision D3 fixes the default. |
| K4 | The 14-DoF plant is still being implemented by another worker | Every interface here is additive and reads `state_names` and a channel dict. Task 10 uses synthetic records, so the exporter is tested without the vertical plant. |
| K5 | A 120 fps video is large and a render is slow | The measured ratio is 4.30× real time at 1920×1080, so a 73.56 s lap renders in 5.3 min. Task 9 measures the real ratio. `--frame-rate`, `--time-range`, resolution, and CRF are all exposed. |
| K6 | Float32 rounding in the record changes a rendered number | The precision is in the manifest, the readouts show 2 decimals, and `--record-precision float64` is available. |
| K7 | matplotlib output is not bit-identical across machines | Determinism is claimed only for the artist data, which is tested. The video digest is recorded, not asserted. |
| K8 | A record is rendered after the model changed | V10 source-hash verification, and the exporter prints the mismatch when it renders anyway. |

## 9. Self-review

- **Requirement coverage:** every item in the user's list maps to a panel artist
  in §3: whole-track line, CG point, body forward axis, velocity vector,
  acceleration vector, z rotation, body block with STL option, rear-view roll
  against the horizon, right-view pitch against the horizon, fixed top view with
  the velocity vertical, body slip angle, tire `Fx`/`Fy`/`Mz` in the tire frame,
  four `Fz` bars, and the throttle, brake, and steer overlays. The 120 fps
  requirement is a first-class CLI default, a test assertion, and an `ffprobe`
  gate.
- **Name check:** `RunRecorder`, `load_run_record`, `resample_run_record`,
  `FrameSet`, `FrameSource`, `read_stl`, `VehicleScene`, `body_rotation`,
  `project`, `Panel`, `FrameWriter`, `FfmpegFrameWriter`, `NullFrameWriter`,
  `AnimationStyle`, `render_frames`, and `ChannelSpec` are each defined once and
  used with the same name later.
- **Highest risks:** the frame draw cost at 120 fps (7) and the two sign
  conventions (K1, K2). Both have a named test before any video is produced.
- **Deliberate exclusions:** no racing-line view, no 3-D perspective camera, no
  audio, no live playback, no GUI, and no change to any physics. These match the
  master plan's deferred scope.
