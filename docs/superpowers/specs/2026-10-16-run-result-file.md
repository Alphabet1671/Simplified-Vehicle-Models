# Run result file — shared full-state capture for every vehicle model

**Status:** specification for audit. This document defines a data contract. It is
not an implementation plan. The companion plan is
[the animation exporter plan](../plans/2026-10-16-animation-exporter.md).

**Scope:** one file format that records the complete vehicle state, the applied
control, the state derivative, and every algebraic output of one forward
simulation segment, for the point-mass, planar 7-DoF, and future 10-DoF and
14-DoF plants. The animation exporter, the comparison plots, and any later
offline analysis read this one file instead of replaying a plant.

## 1. Why a new file is needed

The current forward-driver artifact is `trace.csv`. It stores the integrated
state only:

```text
time,s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,omega_fr,omega_rl,omega_rr,soc,steering
```

[tools/reports/plot_iem24_forward_driver_comparison.py](../../../tools/reports/plot_iem24_forward_driver_comparison.py)
states the consequence in its module docstring: the body-axis accelerations and
the yaw acceleration are **not** in the file, so the tool must reconstruct them
by replaying every saved state through the currently installed plant. That
replay is a provenance hazard: a historical attempt is re-interpreted with
today's code, and the tool says so.

Two further limits matter for animation:

| Limit | Consequence |
|---|---|
| State only | Tire forces, moments, loads, slip quantities, and the state derivative must be recomputed. |
| No model metadata | A consumer cannot know the state layout, the channel set, or the source hashes without reading `summary.json` and the code. |
| No channel set | Every consumer invents its own derived quantities, so two tools disagree. |

The run result file replaces that replay with a recorded, self-describing,
provenance-pinned capture.

## 2. Design decisions

| ID | Decision |
|---|---|
| R1 | One container. A single ZIP container with an NPZ payload holds the arrays, and a JSON document inside the same container holds the metadata. One file, one digest, no sidecar drift. |
| R2 | The file records **exactly what the integrator computed**. The derivative and every algebraic output come from the same plant evaluation as the state, never from a re-evaluation. |
| R3 | The state, the derivative, the control, and the algebraic outputs are sampled at the **accepted integrator steps**, not on a fixed grid. The step knots are the solver's own evidence. |
| R4 | Derived channels are stored once, as recorded. A consumer resamples; it does not recompute physics. |
| R5 | Every channel is named by a stable lowercase kebab-case identifier with a declared unit and reference frame. Unsupported channels are absent, never zero-filled. |
| R6 | Source-file hashes, compiler settings, plant metadata, and the channel list are recorded inside the file, so the record is self-describing. |
| R7 | Float32 is the default payload precision. Float64 is selectable. A float32 record must state its precision in the metadata. |
| R8 | The file is appendable in chunks while a long simulation runs, so a 14-DoF lap does not need the whole capture in memory. |
| R9 | The file is model-agnostic. It carries the plant's own state names; it does not assume twelve states, four wheels, or a planar model. |
| R10 | Nothing in this format replaces a certificate. A run result is not acceptance evidence for periodicity, contact, or refinement; those gates stay in `summary.json`. |

## 3. Container layout

```text
run-record.npz                      one ZIP container
├── manifest.json                   UTF-8 JSON document (stored, not compressed)
├── time                            (n,)      float64, s, simulation time
├── step_index                      (n,)      int32,  accepted integrator step
├── state                           (n, n_state)   float32, in `state_names` order
├── state_rate                      (n, n_state)   float32, the derivative the solver used
├── control                         (n, n_control) float32, in `control_names` order
├── channel                         (n, n_channel) float32, in `channel_names` order
└── channel_valid                   (n, ceil(n_channel / 8))  uint8 bitmap
```

`manifest.json` is read first. `time` is the only required array; every other
array is optional and its presence is declared in the manifest.

The container has no NPZ object arrays, so `np.load(..., allow_pickle=False)` is
always safe. This follows the existing NPZ convention in the repository
(`suspension/damper_table.py`, `power_plant.py`).

### 3.1 Sparse and invalid channels

A channel that a model does not provide is **absent** from `channel_names`. A
channel that a model provides but that is physically undefined at some steps
(for example a load on a lifted wheel, or a slip ratio at a standstill) keeps
its slot and is marked invalid in `channel_valid`. A consumer must treat an
invalid sample as "no value" and must not draw an arrow from it.

**Channel presence is the availability signal.** A consumer must not special-case
a model class. It hides the window that a feature needs when the feature's
channels are all absent, and it states the omission. A window is never drawn
empty and no value is ever invented to fill it.

Storing an invalid value as `NaN` in the `channel` array is also permitted and
is the recommended on-disk representation, because float32 `NaN` round-trips
through NPZ. The bitmap exists so that a consumer can test validity without
reading the payload array.

## 4. Manifest

```json
{
  "schema": "run-record-v1",
  "created_utc": "2026-10-16T21:31:09Z",
  "producer": {"tool": "tools/run/run_iem24_forward_driver.py",
               "version": "1.3.0",
               "python": "3.13.13", "numpy": "2.4.4", "scipy": "1.17.1"},
  "model": {"formulation_id": "iem24-murphy-real-planar-7dof",
            "formulation_xml": "models/formulations/iem24_murphy_real_planar_7dof.xml",
            "assembly_xml": "models/vehicles/iem24_murphy_real_planar.xml",
            "plant_class": "PlanarForwardPlant",
            "model_class": "planar-7dof",
            "mechanical_dof": 7,
            "coordinate_frames": {"vehicle": "sae-j670",
                                  "world": "x-north-y-east-z-down",
                                  "tire": "steered-sae-j670"}},
  "sources": {"models/.../x.xml": "sha256:..."},
  "integrator": {"method": "BDF", "rtol": 1e-05, "atol": 1e-07,
                 "maximum_step": 0.02, "duration_limit_s": 300.0,
                 "wall_seconds": 214.5, "evaluations": 41233,
                 "rejected_domain_trials": 0},
  "segment": {"lap_index": 0, "warm_start_from": null,
              "completed": true, "t_start_s": 0.0, "t_end_s": 73.56298424735283,
              "distance_start_m": 0.0, "distance_end_m": 1016.70064959},
  "track": {"xml": "models/tracks/2024_michigan_endurance_closed.xml",
            "lap_length_m": 1016.70064959, "path_coordinates": true,
            "heading_unwrapped": true},
  "vehicle_geometry": {"wheelbase_m": 1.6, "track_width_m": 1.24,
                       "cg_height_m": 0.3, "front_axle_to_cg_m": 0.88,
                       "half_track_m": 0.62,
                       "wheel_positions_body_m": [[0.88, -0.62], [0.88, 0.62],
                                                  [-0.72, -0.62], [-0.72, 0.62]],
                       "sprung_cg_height_m": null},
  "state_names": ["s", "n", "..."],
  "state_units": ["m", "m", "..."],
  "control_names": ["throttle", "brake-force", "steering-rate", "pedal"],
  "control_units": ["1", "N", "rad/s", "1"],
  "channel_names": ["speed", "..."],
  "channel_units": ["m/s", "..."],
  "channel_frames": ["world", "..."],
  "rows": 12027,
  "precision": "float32",
  "notes": ["..."]
}
```

Notes:

- `sources` hashes every file that changes a number, in the same way
  `summary.json` already records `source_hashes`. A consumer may refuse to
  render a record whose hashes disagree with the working tree, and must say so
  when it renders anyway.
- `vehicle_geometry` is copied from the compiled vehicle so the renderer never
  re-derives geometry. `null` means "not modeled by this formulation", which is
  the honest value for `sprung_cg_height_m` in a planar model.
- `mechanical_dof` is recorded separately from the state-vector length, as the
  master plan [§2](../project-master-plan.md) requires.
- `track.heading_unwrapped` records whether the world-heading channel falls by
  one turn across a lap, which a renderer must know to draw a continuous track.

## 5. State layout and control names

The state names are the plant's own `state_names`. The point-mass, planar, and
14-DoF plants have different layouts, and this format imposes none. The only
imposed rule is that `state_names` is complete, ordered, unique, and stable for
a given formulation.

A recommended shared naming for the four models, which follows the master-plan
vocabulary:

| Group | Names |
|---|---|
| Path | `s`, `n`, `heading-error` |
| Body velocity | `v-x`, `v-y`, `v-z` |
| Body rate | `roll-rate`, `pitch-rate`, `yaw-rate` |
| Body attitude | `heave`, `roll`, `pitch` |
| Wheel spin | `wheel-speed-fl`, `wheel-speed-fr`, `wheel-speed-rl`, `wheel-speed-rr` |
| Unsprung | `jounce-fl`, `jounce-fr`, `jounce-rl`, `jounce-rr`, `jounce-rate-*` |
| Energy and actuation | `soc`, `steering`, `pedal` |
| Transient tire | `kappa-effective-*`, `lateral-deformation-*`, `trail-angle-*` |

The current planar plant uses short names (`v_x`, `omega_fl`, `soc`,
`steering`). Renaming existing state names is a breaking change to
`trace.csv`, so the capture must accept the existing names and add the
recommended names only for new states. This plan does not require a rename.

## 6. Channel catalogue

This is the required channel set for the animation. A model provides the rows it
can compute; a channel it cannot compute is absent, and the renderer must state
the omission on the frame.

Notation: `[4]` means one value per wheel in the order FL, FR, RL, RR.

### 6.1 Kinematics and pose

| Channel | Shape | Unit | Frame | Source |
|---|---|---|---|---|
| `speed` | 1 | m/s | world | `hypot(v-x, v-y)` |
| `body-slip-angle` | 1 | rad | vehicle | `atan2(v-y, v-x)` |
| `heading-world` | 1 | rad | world | path heading at `s` |
| `position-x`, `position-y` | 1 | m | world | path position at `s`, offset by `n` |
| `wheel-position-x`, `wheel-position-y` | [4] | m | world | body wheel position, rotated by heading |
| `wheel-steering-angle` | [4] | rad | vehicle | existing `wheel_steering_angles` |
| `wheel-loaded-radius` | [4] | m | – | existing `loaded_radius` |

### 6.2 Acceleration

| Channel | Shape | Unit | Frame | Source |
|---|---|---|---|---|
| `acceleration-longitudinal` | 1 | m/s² | vehicle | state rate plus rotating-frame terms |
| `acceleration-lateral` | 1 | m/s² | vehicle | state rate plus rotating-frame terms |
| `yaw-acceleration` | 1 | rad/s² | vehicle | state-rate row 5 |
| `acceleration-vertical` | 1 | m/s² | vehicle | 14-DoF only |

The planar plant already provides this exact calculation as
`body_accelerations` in
[planar_forward.py](../../../src/simplified_vehicle_models/solvers/planar_forward.py).
The rotating-frame terms are not optional: in a steady corner the lateral
velocity rate is zero while the lateral acceleration is not.

### 6.3 Attitude

| Channel | Shape | Unit | Source |
|---|---|---|---|
| `roll`, `pitch` | 1 | rad | the attitude state for a model that has one |
| `roll-rate`, `pitch-rate` | 1 | rad/s | the attitude rate state |
| `heave` | 1 | m | the heave state (positive down) |

A model without an attitude state must not invent a dynamic attitude. For the
planar 7-DoF model the recorder may add two clearly named quasi-static
estimates, `roll-quasi-static` and `pitch-quasi-static`, computed from
`acceleration-lateral`, `acceleration-longitudinal`, and two suspension-derived
gradients. The default gradients are the real-car-data-derived values
0.289 deg/g roll and 0.20 deg/g pitch at 1 g, and the channel metadata must mark
them approximate.

A consumer must treat the presence of these channels as the availability signal:

| Channels present | Attitude window |
|---|---|
| `roll` and `pitch` | shown, dynamic |
| `roll-quasi-static` and `pitch-quasi-static` only | **hidden** by default; shown with `--quasi-static-attitude`, labelled approximate |
| neither | **hidden** |

That is, a model that has no attitude channel causes the roll and pitch windows
to be hidden rather than drawn empty. The same rule hides the top view when the
wheel, tire, and control channels are all absent. The animation plan §6 defines
the resulting layout.

### 6.4 Tire

| Channel | Shape | Unit | Frame | Source |
|---|---|---|---|---|
| `tire-longitudinal-force` | [4] | N | tire | `PlanarEvaluation.longitudinal_forces` |
| `tire-lateral-force` | [4] | N | tire | `PlanarEvaluation.lateral_forces` |
| `tire-aligning-moment` | [4] | N·m | tire | `PlanarEvaluation.aligning_moments` |
| `tire-vertical-force` | [4] | N | tire | `PlanarEvaluation.normal_loads` |
| `tire-slip-ratio` | [4] | 1 | tire | `PlanarEvaluation.slip_ratios` |
| `tire-slip-angle` | [4] | rad | tire | `PlanarEvaluation.slip_angles` |
| `tire-force-in-body-x`, `tire-force-in-body-y` | [4] | N | vehicle | optional cross-check |
| `wheel-drive-torque`, `wheel-brake-torque` | [4] | N·m | vehicle | powertrain output |

The transient 14-DoF tire adds `tire-effective-kappa`, `tire-lateral-deformation`,
and `tire-trail-angle` per wheel. Those already exist as states, so they are read
from the state array and are not duplicated as channels.

### 6.5 Control and energy

| Channel | Shape | Unit | Source |
|---|---|---|---|
| `throttle` | 1 | 1 | realized driver command |
| `brake-force` | 1 | N | realized driver command |
| `brake-fraction` | 1 | 1 | `brake-force / maximum_pedal_force` |
| `steering` | 1 | rad | state or realized command |
| `steering-rate` | 1 | rad/s | realized driver command |
| `bus-power`, `bus-voltage`, `bus-current` | 1 | W, V, A | powertrain closure |
| `soc` | 1 | 1 | state |

`throttle`, `brake-fraction`, and `steering` are the three overlays the
animation requires at the bottom of the top-view panel.

## 7. Resampling contract

The animation needs frames on a uniform grid at the requested frame rate, and
the integrator steps are not uniform. The resampling policy is part of the
format, so every consumer produces the same frames:

1. The requested grid is `t_k = t_0 + k / frame_rate`, for
   `k = 0 .. floor((t_end - t_0) * frame_rate)`.
2. Every grid time must lie inside `[time[0], time[-1]]`. A grid time past the
   end is not emitted.
3. `state` is resampled with a **monotone cubic Hermite** (Fritsch–Carlson)
   interpolation on `time`, component by component. Knot values are reproduced
   exactly. The repository already has this interpolator for the suspension
   work (`suspension/monotone_spline.py` in the vertical 14-DoF plan); the
   recorder reuses it rather than adding a second one.
4. `channel` is resampled by **zero-order hold**: the frame at `t_k` uses the
   value recorded at the last accepted step at or before `t_k`. A channel is
   never interpolated, because a channel is an algebraic output of the
   enclosing step and does not exist between steps.
5. `control` uses the same zero-order hold as `channel`.
6. `state_rate` uses the same zero-order hold as `channel`. It is recorded
   evidence, not a curve.
7. The resampler reports, for each frame, the enclosing accepted step index.
   The animation must be able to show the step index, and a later diagnostic can
   check that no frame extrapolates.

This split is deliberate. The state is the quantity the integrator certifies as
continuous, so it is interpolated. The forces are step quantities, so they are
held. A consumer that needs force continuity must run a finer integration, not
smooth the record.

### 7.1 Frame-rate feasibility

The integrator does not need to step at the frame rate. On the recorded
73.56 s IEM24 lap the accepted steps are 12027 rows, and the median step is
5.1 ms, which is finer than one 120 fps frame (8.33 ms). The 99th percentile is
20 ms, the `maximum_step` setting. At the far tail, 0.37 % of steps are shorter
than 0.5 ms, so a record at accepted steps is never coarser than the integrator.

A run at `maximum_step = 0.005 s` (the 14-DoF recommendation) gives at most
200 steps per second, which stays finer than 120 fps. A run with
`maximum_step > 1/120 s` is permitted for analysis but must be flagged in the
manifest as too coarse for a 120 fps animation, because the zero-order hold
would repeat a frame.

## 8. Size budget

Measured on the recorded 7-DoF lap: 12027 accepted steps, 73.56 s.

| Model | States | Channels | Control | Rows | float32 payload | Compressed estimate |
|---|---|---|---|---|---|---|
| Planar 7-DoF | 12 | ~45 | 4 | 12 027 | 2.9 MB | 1.5–2 MB |
| 14-DoF simple | 38 | ~55 | 4 | ~15 000 | 5.7 MB | 3–4 MB |

The compressed estimate is from the `np.savez_compressed` behaviour already used
for `dense_polynomials.npz`. The size is acceptable for a committed artifact only
as an explicit exception; the default location is under the attempt directory
beside `trace.csv`, and the plan asks for an explicit retention decision.

## 9. Producing the record

The recorder wraps the plant, so no plant class needs a new method and
`solve_forward` keeps its signature:

```python
class RunRecorder:
    """Record the full state, rate, control, and outputs at each accepted step."""

    def __init__(self, plant, channel_set, *, path, control_names, precision="float32"): ...
    def on_accepted_step(self, time, state, control, rates) -> None: ...
    def close(self, metadata: dict) -> RunRecordSummary: ...
```

The recorder needs the plant's algebraic outputs. Three options were considered:

| Option | Cost | Verdict |
|---|---|---|
| Re-evaluate the plant at each accepted step | One extra plant call per step; 14-DoF calls are the expensive part | Rejected. It also cannot reproduce transient tire states. |
| Emit a channel callback from `solve_forward` | Needs a solver change; the callback would still re-evaluate | Rejected for the first implementation. |
| Read a `last_evaluation` snapshot that each plant already computes | No extra physics; one small attribute per plant | **Selected.** |

The selected option requires each plant to keep its last algebraic snapshot
(`last_evaluation`), which is the object the plant already builds inside
`evaluate`: `PlanarEvaluation` for the planar plant and `VerticalEvaluation` for
the 14-DoF plant. That attribute is also useful to the existing comparison
tools, which currently replay the plant.

`solve_forward` records through `result.time` and `result.state` today. The
recorder needs the per-step control, which is available inside `state_rates`.
The plan therefore wraps the plant's control resolution: the recorder is given
the same `driver` and replays `driver.command(plant.observe(state))` with the
recorded pedal position, exactly as the existing comparison tool does, but
without re-evaluating the physics.

`solve_forward` must expose the accepted-step log. Two acceptable
implementations are listed in the plan: return the log inside `ForwardResult`,
or accept an optional `step_sink` callback. The plan prefers the callback
because it streams and because `ForwardResult` is already large.

## 10. Validation of the record

The record is only useful if it is verifiable. Each of these checks becomes a
test:

| # | Check |
|---|---|
| V1 | `np.load(..., allow_pickle=False)` reads the container; `manifest.json` parses; `schema == "run-record-v1"`. |
| V2 | `state.shape == (rows, len(state_names))`; every array's first axis equals `rows`; names are unique. |
| V3 | `time` is strictly increasing and equals the solver's accepted times. |
| V4 | `state[0]` equals the initial state and `state[-1]` equals the final state within float32 rounding. |
| V5 | `state_rate` reproduces the plant residual at a sample of rows within tolerance when a plant is available. |
| V6 | Every channel in `channel_names` is either finite or marked invalid in `channel_valid`. |
| V7 | Resampling at 120 fps then interrogating at an accepted knot returns the recorded value exactly for every held channel. |
| V8 | Resampling `state` at its own knots returns the recorded values to 1e-6 relative. |
| V9 | A model that lacks attitude states records no `roll` channel and does record `roll-quasi-static` when the option is on. |
| V10 | The manifest's `source_hashes` match the working files at capture time, and a changed source file is detected. |
| V11 | A record whose `maximum_step > 1/120` sets the coarse-capture flag. |
| V12 | Two captures of the same run are byte-identical apart from `created_utc`, `producer.version`, and `integrator.wall_seconds`. |

## 11. Relationship to existing artifacts

| Artifact | Fate |
|---|---|
| `trace.csv`, `pedal.csv` | Unchanged. Existing tools, hashes, and tests keep working. |
| `summary.json` | Unchanged. It remains the acceptance record; the run record is a data record. |
| `dense_polynomials.npz` | Unchanged. It stays the BDF-only certificate artifact. |
| `run_record.npz` | New, written next to them when `--record` is requested. |

No existing file changes format. The new artifact is additive, which keeps every
golden value and every recorded hash intact.

## 12. Open decisions

| # | Decision needed |
|---|---|
| D1 | Commit a run record for one canonical lap, or keep it out of git and regenerate it on demand? The size estimate is 1.5–4 MB per lap. |
| D2 | Payload precision default: float32 (halves size, adds rounding) or float64 (exact)? |
| D3 | The planar 7-DoF model has no attitude state. Choose one: record the labelled `roll-quasi-static` and `pitch-quasi-static` estimates and show the two attitude windows, or record no attitude channel and let the exporter hide those windows. The user's standing instruction is to hide a window whose feature does not exist, so option two is the default until a gradient is confirmed. |
| D4 | Confirm the reference quasi-static pitch gradient (0.20 deg/g is a placeholder derived from real car data). |
| D5 | Confirm that the state names in `trace.csv` are frozen and that the recommended kebab-case names apply only to new states. |
| D6 | Should the record include the per-wheel contact velocity and the per-wheel force application point? Both are cheap and both would let a later renderer re-derive the arrows independently. |
