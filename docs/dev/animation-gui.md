# Animation GUI

The animation GUI exports a run record as a video or as PNG frames. It does not render anything itself. It builds the argument list for `tools/anim/export_animation.py` and runs that script as a child process. The command line and the GUI therefore give the same result.

## Install

Install the GUI extra from the repository root:

```bash
python -m pip install -e '.[gui]'
```

The exporter needs matplotlib. An MP4 file also needs `ffmpeg` on `PATH`. See [animation-export.md](animation-export.md).

## Start the GUI

Start the application from the repository root. The optional argument is a run record:

```bash
PYTHONPATH=src:. python3 tools/anim_gui/app.py
PYTHONPATH=src:. python3 tools/anim_gui/app.py runs/my_study_20261019-120000/cases/case_000/attempt_1/run_record.npz
```

## Choose a record

Select **Record…** for one `run_record.npz`. Select **Study…** for a `study.json` of the simulation GUI. The list **Study case** then shows the cases that wrote a record. A choice in the list fills the record field.

When the record is valid, the panel **What the export will do** shows the text of `export_animation.py --dry-run`: the frame count, the windows that are drawn, the windows that are hidden with the reason, the size of the file and the warnings. If the record or an option is not valid, a red message shows the reason and the Export button is off.

## Set the options

| Control | Option of the exporter |
|---|---|
| Format | `--output FILE.mp4` or `--png-sequence DIR` |
| Frame rate | `--frame-rate` (default 120) |
| Size | `--width`, `--height` (even numbers, default 1920 × 1080) |
| Quality, Encoder preset | `--crf` (default 18), `--preset` (default medium) |
| Render only a time range | `--time-range T0 T1` |
| Force the top view, Approximate attitude | `--force-top-view`, `--quasi-static-attitude` |
| Body STL, Wheel STL, wheel size, STL scale | `--stl-body`, `--stl-wheel`, `--wheel-radius`, `--wheel-width`, `--transform-scale` |

If `ffmpeg` is not on `PATH`, the MP4 format is off, the PNG format is selected and a note shows the message of the exporter.

## Export

Select **Export**. The GUI starts the exporter with `--progress` and shows the frame count, the rate and the remaining time. **Cancel** stops the child process. After a cancel, a timeout or an error, the GUI removes the unfinished file: the `<name>.partial.mp4` file of a video, or the frames of a PNG folder that holds only frames. It never removes other files. The status bar shows one sentence: done, cancelled, or the reason of the failure.

| Exit code | Message |
|---|---|
| 0 | Done. |
| 2 | The exporter refused the request, with its own reason. |
| 3 | No video encoder is available. Install ffmpeg, or export a PNG sequence. |

The default output is `videos/<record folder>_<record name>.mp4`, or the folder `videos/<record folder>_<record name>_frames` for PNG. Git ignores `videos/`. The GUI asks before replacing an existing output; non-empty PNG folders are refused rather than replaced.

## Preview a still frame

Move the slider to a time and select **Preview this moment**. The GUI renders a few frames at 640 × 360 into a temporary folder and shows the first one. The GUI has no in-app playback. Export the video to watch the motion.

## Tests

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_anim_gui_settings tests.test_anim_gui_inspect tests.test_anim_gui_progress \
  tests.test_anim_gui_records tests.test_anim_gui_qt -v
```

The Qt tests export a real record of `planar-7dof`. They need about 10 s. The MP4 test skips when `ffmpeg` is not installed. The environment variable `ANIM_GUI_QUIT_AFTER_MS` closes the window after that many milliseconds.

## Limits

- The GUI has no in-app playback.
- The `14dof-simple` runner is not used in the tests, because of a known import error that the exporter does not cause.
- Windows and Linux are not tested. Only macOS was used.
