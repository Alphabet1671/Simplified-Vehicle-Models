# Animation export

`tools/anim/export_animation.py` turns a run record (`run_record.npz`) into a 120 fps video. The video has two sides. The left side is the whole track from above. The right side is the vehicle: a fixed top view and, when the model has them, rear and right views that show roll and pitch against the horizon.

The exporter reads the record only. It never re-runs a plant, so a video always shows what the solver recorded. The design and the record format are in [the plan](../superpowers/plans/2026-10-16-animation-exporter.md) and [the record specification](../superpowers/specs/2026-10-16-run-result-file.md).

## Commands

Record a run (any model the runner supports), then export it:

```bash
# 1. Run the forward driver and write run_record.npz next to trace.csv.
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py \
    --model 14dof-simple --track models/tracks/2024_michigan_endurance_closed.xml \
    --planner live --maximum-speed 60.0 --pedal-travel-time 0.175 \
    --maximum-step 0.008 --rtol 1e-05 --method BDF --spacing 0.5 \
    --timeout-seconds 900 --output-root /tmp/runs/14dof --record

# 2. See what would be drawn. Nothing is rendered or written.
python3 tools/anim/export_animation.py --record /tmp/runs/14dof/<attempt>/run_record.npz --dry-run

# 3. Render.
python3 tools/anim/export_animation.py --record /tmp/runs/14dof/<attempt>/run_record.npz \
    --output videos/lap.mp4 --progress
```

Save videos under `videos/` in the repository root. That directory and every video file type are git-ignored, because the files are large; regenerate a video from its run record instead of committing it.

Use `--maximum-step 0.008` or smaller when recording. A record with a larger step is flagged coarse, and the exporter warns that 120 fps frames are interpolated from sparser data.

Useful options:

| Option | Meaning |
|---|---|
| `--time-range T0 T1` | Render only record times `T0`..`T1`. The frame count is `floor((T1 - T0) * rate) + 1`. |
| `--frame-rate R` | Default 120. Must be positive and finite. |
| `--width W --height H` | Default 1920 x 1080. Both must be even (yuv420p). |
| `--crf N --preset P` | x264 quality (default 18) and speed (default `medium`). |
| `--png-sequence DIR` | Write `frame_000000.png` ... instead of a video. Needs no ffmpeg. |
| `--force-top-view` | Draw the top view for a record with no wheel data (speed vector, body line, inputs only). |
| `--quasi-static-attitude` | Draw rear and right views from the approximate quasi-static roll and pitch, when the record has no dynamic ones. |
| `--dry-run` | Print the plan and exit. |
| `--progress` | Print frames per second and an ETA to stderr. |

The video is written to `<output>.partial.mp4` and renamed when ffmpeg exits cleanly. A failed encode leaves no file. Exit status 2 is a bad argument or input, 3 is an ffmpeg failure. If `ffmpeg` is not on `PATH` the message says so; use `--png-sequence`.

## Windows by model

A window exists only when the record holds the channels it draws. A window that cannot be drawn is hidden, not drawn empty, and the other windows grow into the space. The footer of every frame says which windows are hidden and why, and the dry run prints the same text.

| Model | GG | Track | Rear (roll) | Right (pitch) | Top |
|---|---|---|---|---|---|
| Point mass | shown | shown | hidden | hidden | hidden; `--force-top-view` shows speed, body line, inputs |
| Planar 7-DoF | shown | shown | hidden; `--quasi-static-attitude` shows an estimate labelled APPROXIMATE | same | shown |
| Body 10-DoF | shown | shown | shown | shown | shown |
| 14-DoF | shown | shown | shown, plus a heave readout | shown | shown |

Layouts: the GG diagram is a column at the far left. It needs only the two acceleration channels, so every model with them has it. To its right, all four other windows use a 2 x 3 grid (track on the left over the full height; top view above; rear and right below it). Without the attitude windows the top view takes the full height on the right. Without the top view the rear and right views take the full height. A point-mass record is the track alone. Sub-features follow the same rule: no tire force channels removes the arrows, no vertical force channel removes the Fz bars, and the footer names what is missing.

## Reading a frame

All vectors use the SAE J670 body axes: +x forward, +y right, +z down. The world frame has X north and Y east. Yaw is positive clockwise from above. Roll is positive right-side-down. Pitch is positive nose up. Heave is positive down.

**Track window.** The grey line is the centre line of the track. The white line from the CG marker is the body +x axis (heading). The green arrow is the velocity, which points at heading plus body slip angle. The orange arrow is the body acceleration rotated into the world. The violet arc is the yaw acceleration, clockwise positive. Arrow lengths are scaled once per run from the run maxima. The picture carries no legend text: the full-scale values are printed by `--dry-run` and after a render.

**GG window.** Lateral acceleration to the right (body +y is right) and longitudinal acceleration up, both in m/s^2, with rings at round values that contain the run maximum. The dot is the current sample; the line behind it is the last 1.0 s (120 frames at 120 fps) and fades from transparent at its oldest end to opaque at the dot. In the first second of the run the oldest end repeats the first sample.

**Top window.** The body is rotated so the velocity is always up. The angle between the vertical and the white body line is the body slip angle. At each wheel the red arrow is Fx and the blue arrow is Fy, in the tire frame (the body axes rotated by the wheel steer angle). The violet arc is Mz, clockwise positive. The four Fz bars on the right are a 2 x 2 grid that mirrors the car: front-left and front-right on the top row, rear-left and rear-right below. Each prints its instant value in N under the bar. The bars below are throttle, brake and steer (centre zero).

**Rear and right windows.** The dashed line is the horizon. The body outline is rotated by the recorded roll (rear) or pitch (right), and the green line is the body y axis (rear) or x axis (right), so the angle you see is the recorded angle.

## Vehicle shape

The body is a placeholder block (length = wheelbase + 0.9 m, width = track - wheel width, height 0.25 m) with four cylinder wheels. Change it with `--body-length`, `--body-width`, `--body-height`, `--wheel-radius` and `--wheel-width`. Wheel spin is integrated from the recorded wheel speed and is a display effect.

`--stl-body PATH` and `--stl-wheel PATH` replace the shapes. The reader accepts ASCII and binary STL. One scale (`--transform-scale`, for example `0.001` for millimetres) and one shift (`--transform-x`, `--transform-z`) are applied and printed in the footer and the summary. The STL is a display asset: it never enters the physics and never changes a simulation number. Orient the file in body axes (+x forward, +y right, +z down) with the origin at the CG.

## Cost

Rendering is by matplotlib Agg with artist updates, so no axes are rebuilt per frame. The CLI prints the render time and the render-to-real-time ratio after each run.

Measured on an Apple M2 for a full 14-DoF lap at 1920 x 1080: **6.19x real time** (51.6 ms per frame, 412 s for a 66.57 s lap), producing a 14.1 MB file. The first layout measured 9.68x. Most of the time is Agg drawing the axes; the data updates take under 2 ms per frame. Details and the caveats are in [the validation note](../validation/iem24-murphy/animation-export.md). The dry run prints only a rough size estimate, marked "not measured"; on that lap it overestimated the size by 2.7x.

To go faster: lower `--width`/`--height`, use `--time-range`, or render with `--png-sequence`. Multiprocess chunking (`--frame-chunk`) is not implemented, because the measured ratio is under the 12x threshold set in the plan.

## Files

| File | Role |
|---|---|
| `tools/anim/resample.py` | `FrameSource`: 120 fps frames, channel access, run scales, warnings |
| `tools/anim/layout.py` | `window_visibility`, `window_grid` |
| `tools/anim/panels.py` | The track, rear/right and top panels |
| `tools/anim/scene.py`, `stl.py` | Vehicle shape, STL reader, projections |
| `tools/anim/render.py`, `encode.py` | Figure, frame loop, ffmpeg pipe, PNG and null writers |
| `tools/anim/export_animation.py` | The command line |
