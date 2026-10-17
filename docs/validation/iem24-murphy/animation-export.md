# IEM24 Murphy lap animation export

This note records one real export of a 14-DoF lap of the IEM24 endurance track, made on 2026-10-16 with `tools/anim/export_animation.py`. It states what was measured, what was only estimated, and what the video can and cannot be used for.

## Result

| Item | Value | Source |
|---|---|---|
| Model | `14dof-simple` | record manifest |
| Track | `models/tracks/2024_michigan_endurance_closed.xml`, 1016.70 m | run summary |
| Simulated lap time | 66.563 s | run summary |
| Record rows | 18 545 accepted steps, float32 | run summary |
| Frames | 7 988 at 120 fps | `ffprobe -count_frames` |
| Video duration | 66.567 s | `ffprobe` |
| Codec, pixel format | H.264 `yuv420p`, 1920 x 1080, `r_frame_rate=120/1` | `ffprobe` |
| File size | 15.6 MB (15 606 104 bytes) | measured |
| **Render wall time** | **644.2 s** | measured by the CLI |
| **Render / real-time ratio** | **9.68x** (644.2 s / 66.57 s) | measured |
| Per-frame time | 80.6 ms | measured |
| Rendered frames per wall second | 12.4 | measured |
| Machine | Apple M2, 8 cores, 24 GB, macOS, Python 3.13, matplotlib 3.11.2, ffmpeg 8.1.2 | `sysctl`, `ffmpeg -version` |

The frame count equals the dry-run count (`floor(66.563 * 120) + 1 = 7 988`), and the video duration equals frames / 120 to well within 0.01 s. All four windows were shown, and the export printed no warning.

The size estimate in the dry run was 41.4 MB. It is a rough model (0.02 bit per pixel per frame) and it was **wrong by a factor of 2.7**: the measured size is 15.6 MB. The estimate should be treated as an upper bound until the constant is refitted.

## Render cost

The earlier plan predicted 4.30x from a 60-frame prototype and expected 4.3-6x for a full export. The real export is **9.68x**, so the plan figure was optimistic by a factor of 2.3. It is below the 12x threshold at which the plan calls for a `--frame-chunk` multiprocessing follow-up, so no follow-up was started. A full lap of about 66 s of simulated time takes about 10.7 minutes on this machine.

Where the 80.6 ms per frame goes, measured on a 2 s excerpt of a 20 s record with `perf_counter` (a separate profile, not the full lap, and the split is approximate):

| Part | Time per frame |
|---|---|
| `canvas.draw()` | 58 ms |
| Of which the top-view axes | 24 ms |
| Of which the track axes | 18 ms |
| Of which the rear and right axes | 9 ms and 7 ms |
| Copy of the RGBA buffer to a contiguous RGB array | 9 ms |
| Artist data updates for all panels | 1.7 ms |
| Update + draw + RGB copy together | 72.9 ms (measured as one loop) |
| Remainder to the 80.6 ms full-lap figure: pipe write to ffmpeg, encoder back-pressure, run-to-run variation | about 8 ms (by subtraction, not measured directly) |

The bottleneck is matplotlib's Agg rasterisation of each axes, not the data updates or the encoder. Cheap options if the ratio matters: a lower resolution (cost scales roughly with pixel count), a shorter `--time-range`, or `--frame-chunk` multiprocessing (frames are independent). The track axes redraw the whole track polyline and its tick labels on every frame; blitting the static background is the most likely single saving, but it was not tried and its gain is not measured.

## Stills

Each still is a frame decoded from the encoded video, so it shows what a viewer sees.

| Still | Frame | Time | Why |
|---|---|---|---|
| [Start](animation-export/still_start.png) | 0 | 0.000 s | run start |
| [Peak Fz](animation-export/still_peak_fz.png) | 2 350 | 19.583 s | largest single-tire vertical force, 1 590.7 N |
| [Peak lateral acceleration](animation-export/still_peak_ay.png) | 2 413 | 20.108 s | largest \|a_y\|, 17.20 m/s^2 |
| [Lap end](animation-export/still_lap_end.png) | 7 987 | 66.558 s | last frame |

These frames were checked numerically (size, and the share of non-background pixels in each half of the image). **The author of this note could not view the images**, so no claim is made here about how they look. Open them before using the video in a report.

## Caveats

1. **The lap is not a validated result.** The run summary says `usable: false`. `path_tracking_valid` is false (maximum lateral error 0.383 m), `periodicity_valid` is false (the end state differs from the start state by more than the tolerances), and between-step validation is incomplete. The video shows the run exactly as recorded; it does not show a certified lap, and it must not be cited as simulation evidence.
2. **The lap time differs from the earlier plan figure.** This run is 66.563 s. The plan sized the video from a 73.56 s lap (8 828 frames) that came from a different run. The ratio is independent of lap length, but the frame count and the wall time are not.
3. **Attitude is dynamic here.** The 14-DoF model carries roll, pitch and heave as states, so the rear and right windows show recorded angles. For a planar model they would be quasi-static estimates, hidden by default and labelled APPROXIMATE when requested.
4. **The body is a placeholder block.** Length 2.435 m, width 1.060 m, height 0.250 m, with four cylinder wheels. It is a display asset and carries no information about the real car. An STL file changes the picture only and never a simulation number.
5. **Wheel spin is a display effect**, integrated from the recorded wheel speeds.
6. **Float32 record.** Values are rounded to float32, and the readouts show two decimals.
7. **The encoded file is not bit-reproducible across machines.** The artist data is deterministic, but matplotlib text rendering and x264 builds can differ. The video is not committed (`videos/` is git-ignored; it was saved as `videos/iem24_14dof_lap.mp4`); its SHA-256 begins `5ce623bd804df2d9` for this machine and build.

## Reproduce

```bash
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py \
    --model 14dof-simple --track models/tracks/2024_michigan_endurance_closed.xml \
    --planner live --maximum-speed 60.0 --pedal-travel-time 0.175 \
    --maximum-step 0.008 --rtol 1e-05 --method BDF --spacing 0.5 \
    --timeout-seconds 3600 --output-root /tmp/animrec/lap14 --record

MPLCONFIGDIR=/tmp/mplcfg python3 tools/anim/export_animation.py \
    --record /tmp/animrec/lap14/<attempt>/run_record.npz \
    --output videos/iem24_14dof_lap.mp4 --progress
```

The recording took 144 s of wall time. The run is a continuous 66.56 s lap, so the record holds one segment.
