# Open-loop maneuver GUI

The **Maneuvers** tab in the Simulation GUI defines and runs reproducible, simulation-first steering and pedal experiments. It uses canonical steering-wheel angle (SWA, radians; positive is right) and accelerator/brake travel fractions in `[0, 1]`. Accelerator and brake never overlap.

> **Not vehicle authorization.** The GUI runs simulation only. The robot package contains a fake commissioning harness and no CAN, analog, vendor, or real actuator transport. Never infer a real command interface from this guide.

## Install and start

```bash
python -m pip install -e '.[gui]'
PYTHONPATH=src:. python3 tools/sim_gui/app.py
```

Open a vehicle formulation in the shared **Model** dock, then select **Maneuvers**. This is the same import/component-selection workflow used by lap simulations; Maneuvers intentionally has no second model browser. A track is not required. The selected vehicle is copied into a validated draft; changing vehicle clears the prior maneuver result and draft state. The tab refuses physical backends and hardware arm remains unavailable.

The Maneuvers tab follows the lap-run layout: select an **Export folder**, run or stop the isolated simulation child, and watch the shared progress bar, status line, local bounded log, application Log tab, and standard Queue tab. A Run action creates a native `sim-study-v1` index containing one maneuver case; future batch studies use the same format. Its persistent layout is the normal suite layout:

```text
<study>/definition.study.json       # maneuver-study-v1 input collection
<study>/study.json                  # sim-study-v1 native index
<study>/cases/case_000/case.json    # immutable maneuver-v1 case input
<study>/cases/case_000/attempt_*/run.log
<study>/cases/case_000/attempt_*/summary.json
<study>/cases/case_000/attempt_*/parent.json
<study>/cases/case_000/attempt_*/result/            # immutable validated maneuver evidence
<study>/cases/case_000/attempt_*/run_record.npz     # canonical run-record-v1
```

Completed 14DoF cases create `run_record.npz` at precisely the same standard attempt path as lap studies. It is converted only from completed, hash-validated accepted maneuver evidence, without replaying physics, so the animator consumes it with the standard RunRecord loader. The straight-reference track used in this animator record is display-only. Failed, cancelled, timed-out, or currently unsupported planar maneuver cases remain normal indexed queue attempts but have no animator record. Use **Export animation…** to choose a fresh MP4 destination; the existing bridge still rejects an existing output rather than overwriting it.

A headless simulation run uses the same runner:

```bash
PYTHONPATH=src:. python3 -m tools.maneuver_gui.runner \
  --definition docs/dev/examples/step-steer.maneuver.json --output-root runs/maneuvers
```

## Presets, units, and ownership

Available presets are `step_steer`, `ramp_steer`, `sine_steer`, `straight_maintain`, `straight_accel_brake`, `circular_skidpad`, and `csv_playback`.

A preset provides guidance; a schema-valid non-CSV steering or longitudinal mode remains selectable and owns its matching control. The editor labels this guidance rather than silently rewriting a mode. Examples:

- Peak SWA uses `steering.amplitude_rad`; choose direction `+1` or `-1` for right/left.
- Peak lateral acceleration (`peak_ay`) is an **offline SWA amplitude calibration**, never live ay tracking. Its calibrated fingerprint must match the exact definition before Run.
- Path PID uses a heading target; radius PID uses a radius target; locked SWA is a fixed steering experiment.
- Cruise PID owns longitudinal pedals from speed error; constant accelerator owns accelerator travel. Phase profiles support lead, rise, ramp, hold, release, and recovery timing.
- CSV owns SWA, accelerator, and brake together. It disables controller overlays and PID fields.

Displayed/serialized storage is SI: radians, metres, seconds, m/s, m/s², and pedal fractions. Presentation conversions (degrees, km/h, g, percent, milliseconds) do not silently change stored values.

## CSV import

Map explicit time, SWA, accelerator, and brake columns. Select time and steering/pedal units, interpolation (`hold` or `linear`), rebase behavior, delimiter, decimal convention, or a sample interval when no time column exists. The importer validates UTF-8, size/row limits, monotonic time, pedal overlap, bounds, and interpolation crossings; row errors remain inline.

CSV samples are immutable. The GUI snapshots source SHA-256, size, and mtime; a changed file must be reloaded before Run. Irregular knots remain source knots; there is no extrapolation or controller overlay.

## IEM24 14DoF simulation evidence and animation

The headless runner selects the IEM24 vertical 14DoF path from its compiled parameter type, not a formulation filename. Completed 14DoF artifacts record canonical world-pose channels (`position-x`, `position-y`, `heading-world`), which permit an accepted-evidence-only animation export:

```bash
PYTHONPATH=src:. python3 -m tools.maneuver_gui.export_animation \
  --artifact <completed-artifact-dir> --output <case>.mp4 --frame-rate 30
```

The export bridge verifies the artifact manifest, hashes, completed outcome, exact `14dof-simple` model kind, canonical state width, commands, and pose channels before writing a RunRecord. It never replays plant physics. Existing media, RunRecord, or nonempty PNG destinations fail closed rather than being overwritten. The straight display track and vehicle geometry in the RunRecord are presentation-only; the vertical maneuver adapter is a straight-reference simulation, not track path following.

For a `peak_ay` simulation definition, the runner first performs bounded **simulation-only** pre-run trials, records their accepted ay evidence, requires a matched SWA selection, freezes that SWA into the final artifact definition, and retains the trial record in the summary. This is model calibration evidence—not measured vehicle validation, commissioning evidence, or authorization for physical output.

## Preview, artifacts, and cancellation

The preview separates requested and applied SWA, accelerator, and brake traces on a shared time scale. Simulation outcomes record accepted samples, observations, requested/applied commands, events, termination reason, and summary. Artifact directories publish a manifest, definition, summary, and NPZ evidence atomically; loaders verify hashes.

**Cancel simulation** terminates the runner process and marks its outcome cancelled. A robot controlled stop is a different future, backend-approved action; it is not simulated by writing zero pedals or centering steering.

For a circular skidpad, interpret a physical limit only from accepted aligned evidence: adequate speed, persistent path failure, no saturation/model failure, and a preceding valid point. Otherwise report `limit_not_reached` or a model-limit/result category; do not claim tire-limit discovery from an unbracketed trace.

## Robot commissioning boundary

The fake `RobotSession` contract is a test harness, not a device adapter. A real deployment requires all of the following before an external commissioned adapter may exist:

1. vendor command and telemetry interface identity, authentication, and protocol review;
2. SWA/road-wheel and pedal travel/force calibration with expiration and exact definition binding;
3. measured telemetry, enable/interlock behavior, watchdogs, absolute deadlines, and approved controlled-stop mode;
4. independent vehicle safety hardware, operator stop behavior, test envelope, and commissioning approval;
5. validation of actuator feasibility, fault response, reconnect/rearm workflow, and an auditable acceptance record.

The fake backend deliberately tests stale telemetry, disconnect, deadline miss, interlock loss, stop acknowledgement, and independent fake watchdog behavior. It is not evidence that physical equipment is safe.
