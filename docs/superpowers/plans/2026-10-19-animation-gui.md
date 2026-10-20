# Animation GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A PyQt6 program, `tools/anim_gui/`, that exports a run record as an MP4 video or a PNG sequence through `tools/anim/export_animation.py`, with a dry-run summary, a progress bar, a cancel button and a still-frame preview.

**Architecture:**
- A Qt-free core (`settings.py`, `inspect_plan.py`, `progress.py`, `records.py`) and a thin `qt/` layer, as in `tools/sim_gui/`.
- The GUI does not render. It builds an argument list and runs `tools/anim/export_animation.py` as a child process through `ProcessJob` (from `tools/gui_common/`). A render crash cannot close the window.
- The dry run uses the exporter's own parser and `describe` function in the same process, so the GUI and the command line cannot disagree.
- There is no in-app playback. A preview is one still PNG frame.

**Tech Stack:** Python 3.12+, PyQt6 ≥ 6.5 (6.11.0 is installed), numpy, `unittest`, `xml.etree.ElementTree`, matplotlib (for the exporter), `ffmpeg` on `PATH` (for MP4 only).

**Spec:** `docs/superpowers/specs/2026-10-19-gui-wrappers-investigation.md`, sections 1, 6, 7 and 8.

## Global Constraints

- Change nothing in `src/`. Do not edit the files of the simulation GUI (`tools/sim_gui/`). Read-only imports from it are listed under Fixed Decisions.
- Only `qt/` modules, `app.py` and the Qt tests import PyQt6. Every other new module imports only the standard library, numpy and the repository.
- PyQt6 stays an optional extra: `gui = ["PyQt6>=6.5"]` is already in `pyproject.toml`. Do not change `pyproject.toml`.
- Make it cross-platform. Use `pathlib`, `sys.executable`, and `QProcess` or `subprocess` with argument lists. Never use a shell. Do not use POSIX-only calls such as signals, `fork` or `timeout`.
- Write files atomically: write `<name>.tmp`, then `os.replace`. Write JSON with `indent=2, sort_keys=True, allow_nan=False`.
- A GUI never writes into `models/` unless the user ticks a confirmation. It never overwrites a file unless the user ticks a second confirmation.
- Write the docs in ASD-STE100 style: short sentences, one idea in each sentence.
- Run tests as `MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.<module> -v`. Qt tests skip when PyQt6 is not installed.
- Every Qt test that creates a window stubs `QMessageBox.question`, `warning` and `information` in `setUp` and restores them with `addCleanup`. An unstubbed modal dialog hangs the test run.
- Name each new tool file so that no other `tools/*/` folder holds a file of the same name. `tests/test_default_track.py` globs `tools/*/<name>` and expects one match.
- Run a time-limited command with a Python `subprocess.run(..., timeout=N)` wrapper. macOS has no `timeout` command.
- Change nothing in `tools/anim/`. The GUI only runs it.
- The default output folder is `videos/`. `/videos/` is already in `.gitignore`.
- A cancel, a timeout or an error removes the unfinished output. It never removes a file that the export did not make. A PNG folder is cleaned only when it holds frames and nothing else.
- When `ffmpeg` is missing, MP4 is off and PNG is selected. The GUI never starts an MP4 export without an encoder.
- A real render needs a record with a track file. A synthetic record has none, so a real render of it exits with code 2. The dry run works.

## Known Baseline Failures

`unittest discover` on `main` at `1e6d510` fails in a fresh checkout. This plan does not cause these failures and does not fix them. The root causes seen are:
- `ImportError: cannot import name 'corner_toe_angles' from 'simplified_vehicle_models.suspension.corner'`, raised at `src/simplified_vehicle_models/results/plant_channels.py:243`. The `14dof-simple` runner child fails with it. It breaks `test_animation_cli.RenderTest` (3 tests), `test_run_record_runner.RealRecordRunTest` (1), `test_animation_panels.RecordedRunTest` (1) and `test_run_record_channels.VerticalChannelTest` (2).
- Git-ignored trace files under `docs/validation/iem24-murphy/14dof_simple_trials/` are missing from a fresh checkout. They break `test_vertical_14dof_evidence` and `test_vertical_between_step`.

A fresh checkout of `main` gives 1,619 tests with 20 failures and 16 errors. The shared package of the track plan adds 18 tests that pass. Three of the failures are in `test_animation_cli.RenderTest`: they are the exporter's own tests, and the animation GUI does not touch them. So the check for every task is not "the suite is green". The check is "the set of failing tests is the same as before". Task 1 records the set. The last task compares it.

Never use the `14dof-simple` runner in a new test. To get a real run record in a test, use `real_child("planar-7dof", "--record")` from `tests.test_run_record_runner`.

## Branch Dependency

This plan builds on `main` at `1e6d510`, **and on Tasks 2 and 3 of the track plan** (`docs/superpowers/plans/2026-10-19-track-gui.md`). Those tasks add `tools/gui_common/__init__.py`, `tools/gui_common/files.py` and `tools/gui_common/qt/process.py` (`ProcessJob`). This plan uses all three.

Check this before Task 2:

```bash
python3 -c "import sys; sys.path[:0] = ['src', '.']; from tools.gui_common.files import check_name, is_inside; from tools.gui_common.qt.process import ProcessJob, python_command; print('ok')"
```

Expect `ok`. If the import fails, merge the track plan first. Do not copy these files into this plan.

The animation exporter (`feature/animation-exporter`) is already in `main`.

## Fixed Decisions

| Topic | Decision |
|---|---|
| Location | Package `tools/anim_gui/`. Start it with `PYTHONPATH=src:. python3 tools/anim_gui/app.py [run_record.npz]`. |
| Programs | Three separate programs. This plan builds the animation GUI. |
| In-app playback | Not offered. (The user chose this.) A preview is one PNG frame from a narrow `--time-range`. |
| Shared code | `tools/gui_common/` from the track plan: `check_name`, `is_inside`, `ProcessJob`. (The user chose a shared package.) |
| Rendering | In a child process: `sys.executable tools/anim/export_animation.py ...` with an argument list and no shell. |
| Dry run | In process, with the exporter's own parser and `describe`. It never starts a render. |
| Inputs | One `run_record.npz`, a folder (the finder lists the records below it, at most 500), or a `study.json` (schema `sim-study-v1`). |
| Options | Format, frame rate, even size, CRF, encoder preset, time range, top view, quasi-static attitude, STL shape files and sizes. Each maps to one exporter option. |
| Exit codes | 0 done. 2 the exporter refused the request (its own reason is shown). 3 no video encoder. |
| Timeouts | Export 6 hours. Preview 120 s. |
| Output | `videos/<record folder>_<record name>.mp4` by default, or the folder `..._frames` for PNG. A confirmation before the GUI replaces a file or a non-empty folder. |
| Reuse from the sim GUI | None, except that the study file format `sim-study-v1` is read. |
| Settings | `QSettings("Simplified-Vehicle-Models", "anim-gui")`. |

## Output Layout

```text
videos/
├── <record folder>_<record name>.mp4       # a finished video
├── <record folder>_<record name>.partial.mp4   # while the export runs; removed on cancel, timeout or error
└── <record folder>_<record name>_frames/frame_000000.png ...   # the PNG sequence
```

A preview writes one PNG in a temporary folder. The GUI deletes the folder when the window closes.

The `--progress` line of the exporter has this form:

```text
frame {done}/{total}  {rate:6.1f} frames/s  eta {remaining:6.0f} s
```

## File Map

| File | Responsibility |
|---|---|
| `tools/anim_gui/settings.py` | `ExportSettings`: check and build the exporter arguments; default output; preview settings |
| `tools/anim_gui/inspect_plan.py` | `inspect`: the dry-run text, the refusal reason, the ffmpeg check |
| `tools/anim_gui/progress.py` | `parse_progress`, `parse_error`, `describe_result`, `format_eta` |
| `tools/anim_gui/records.py` | `find_records`, `study_records`, `remove_partials` |
| `tools/anim_gui/qt/main_window.py` | `MainWindow` |
| `tools/anim_gui/app.py` | Entry point |
| `docs/dev/animation-gui.md` | User guide |
| `docs/dev/tools-layout.md` | Modify: one row |

| Test file | Covers |
|---|---|
| `tests/test_anim_gui_settings.py` | Checks and arguments |
| `tests/test_anim_gui_inspect.py` | Dry run and refusals |
| `tests/test_anim_gui_progress.py` | Progress and exit codes |
| `tests/test_anim_gui_records.py` | Record and study lists, partial files |
| `tests/test_anim_gui_qt.py` | Window, real export, cancel, preview |
| `tests/test_anim_gui_docs.py` | Guide, layout row, `/videos/` ignored |

---

### Task 1: Create the worktree and record the baseline

**Files:** none.

- [ ] **Step 1: Create the worktree.** `.worktrees/` is git-ignored. Branch from `main` after the track plan's Tasks 2 and 3 are merged.

```bash
cd /Users/alphabet1671/Dev/Simplified-Vehicle-Models
git worktree add .worktrees/anim-gui -b feature/anim-gui main
cd .worktrees/anim-gui
git log --oneline -1
```

Expect a commit that is `1e6d510` or later. Run every later command in `.worktrees/anim-gui`.

- [ ] **Step 2: Check the shared package and the tools.**

```bash
python3 -c "import sys; sys.path[:0] = ['src', '.']; from tools.gui_common.files import check_name, is_inside; from tools.gui_common.qt.process import ProcessJob, python_command; print('ok')"
python3 -c "import numpy, matplotlib, PyQt6; print('ok')"
which ffmpeg || echo "no ffmpeg: the MP4 test will skip"
```

Expect `ok` twice. The tests pass without `ffmpeg`.

- [ ] **Step 3: Record the baseline failures.** The run takes about 8 minutes. Run it in the background and keep working.

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -q > /tmp/baseline_full.log 2>&1
grep -E "^(FAIL|ERROR):" /tmp/baseline_full.log | sed "s#$PWD#<root>#g" | sort > /tmp/baseline_fails.txt
wc -l < /tmp/baseline_fails.txt
```

Expect `36`. The log ends with `FAILED (failures=20, errors=16, skipped=4)`. Keep `/tmp/baseline_fails.txt` for the last task.

### Task 2: Add the export settings

`ExportSettings` is one frozen record of every option of the exporter. `check()` returns the reason a setting is not valid, or `None`. `arguments()` builds the argument list of `tools/anim/export_animation.py`. The GUI does not render. It runs the script, so the command line and the GUI give the same result.

**Files:**
- Create: `tools/anim_gui/__init__.py`, `tools/anim_gui/settings.py`
- Test: `tests/test_anim_gui_settings.py`

**Interfaces:**
- Consumes: `tools.gui_common.REPOSITORY_ROOT` and `tools.gui_common.files.check_name` from the track plan (Task 2). `tests.test_animation_cli.record_file` and the constants of `tests.test_animation_panels` build a synthetic record in the tests.
- Produces:
  - `EXPORT_SCRIPT`
  - `VIDEO_DIRECTORY`
  - `PRESETS`
  - `FORMATS`
  - `class ExportSettings` fields `record: Path`, `output: Path`, `format: str = 'mp4'`, `frame_rate: float = 120.0`, `width: int = 1920`, `height: int = 1080`, `crf: int = 18`, `preset: str = 'medium'`, `time_range: tuple[float, float] | None = None`, `stl_body: Path | None = None`, `stl_wheel: Path | None = None`, `body_length: float | None = None`, `body_width: float | None = None`, `body_height: float | None = None`, `wheel_radius: float = 0.2`, `wheel_width: float = 0.18`, `transform_scale: float = 1.0`, `transform_x: float = 0.0`, `transform_z: float = 0.0`, `force_top_view: bool = False`, `quasi_static_attitude: bool = False` methods `check() -> None`; `arguments(*, dry_run: bool=False, progress: bool=False) -> list[str]`
  - `default_output(record: Path, fmt: str, directory: Path=VIDEO_DIRECTORY) -> Path`
  - `preview_settings(settings: ExportSettings, time: float, folder: Path, *, width: int=640, height: int=360, window: float=0.05) -> ExportSettings`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_settings.py`**

```python
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim_gui import settings as S
from tools.gui_common import REPOSITORY_ROOT


def make_settings(record, **changes):
    return replace(S.ExportSettings(record=Path(record), output=Path(record).with_suffix(".mp4")), **changes)


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.record, _ = record_file(self._tmp.name, BASE + WHEEL + DYNAMIC)

    def test_defaults_match_the_exporter_defaults(self):
        from tools.anim import export_animation as cli
        defaults = cli.build_parser().parse_args(["--record", "x"])
        settings = make_settings(self.record)
        self.assertEqual((settings.frame_rate, settings.width, settings.height, settings.crf, settings.preset),
                         (defaults.frame_rate, defaults.width, defaults.height, defaults.crf, defaults.preset))
        self.assertEqual((settings.wheel_radius, settings.wheel_width, settings.transform_scale),
                         (defaults.wheel_radius, defaults.wheel_width, defaults.transform_scale))

    def test_arguments_are_accepted_by_the_real_parser(self):
        from tools.anim import export_animation as cli
        full = make_settings(self.record, time_range=(0.1, 0.3), stl_body=Path("b.stl"), stl_wheel=Path("w.stl"),
                             body_length=2.5, body_width=1.2, body_height=0.9, force_top_view=True,
                             quasi_static_attitude=True, frame_rate=60.0, width=640, height=360, crf=23, preset="fast")
        args = cli.build_parser().parse_args(full.arguments(progress=True))
        self.assertEqual((args.time_range, args.frame_rate, args.width, args.crf, args.preset),
                         ([0.1, 0.3], 60.0, 640, 23, "fast"))
        self.assertTrue(args.force_top_view and args.quasi_static_attitude and args.progress)
        self.assertEqual(args.output, full.output)
        png = replace(full, format="png", output=Path(self._tmp.name) / "frames")
        parsed = cli.build_parser().parse_args(png.arguments())
        self.assertEqual(parsed.png_sequence, png.output)
        self.assertIsNone(parsed.output)

    def test_dry_run_arguments_have_no_output(self):
        arguments = make_settings(self.record).arguments(dry_run=True)
        self.assertIn("--dry-run", arguments)
        self.assertNotIn("--output", arguments)

    def test_check_names_the_problem(self):
        cases = [({"width": 641}, "even"), ({"height": 0}, "even"), ({"frame_rate": 0.0}, "frame rate"),
                 ({"frame_rate": math.nan}, "frame rate"), ({"crf": 52}, "CRF"), ({"preset": "warp"}, "preset"),
                 ({"time_range": (0.3, 0.1)}, "less than"), ({"body_length": -1.0}, "body length"),
                 ({"format": "gif"}, "format"), ({"output": Path("x.avi")}, ".mp4")]
        for changes, word in cases:
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, word):
                make_settings(self.record, **changes).check()
        with self.assertRaisesRegex(ValueError, "does not exist"):
            make_settings(Path(self._tmp.name) / "nope.npz").check()
        make_settings(self.record).check()

    def test_default_output_is_under_videos_and_has_a_safe_name(self):
        path = S.default_output(Path("/a/b/my run (2)/run_record.npz"), "mp4")
        self.assertEqual(path.parent, REPOSITORY_ROOT / "videos")
        self.assertEqual(path.name, "my_run_2_run_record.mp4")
        self.assertEqual(S.default_output(Path("/a/run_record.npz"), "png").name, "a_run_record_frames")

    def test_preview_settings_are_a_small_png_window(self):
        preview = S.preview_settings(make_settings(self.record, width=1920, height=1080), 0.2, Path(self._tmp.name) / "p")
        self.assertEqual((preview.format, preview.width, preview.height, preview.time_range), ("png", 640, 360, (0.2, 0.25)))
        preview.check()


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_settings -v
```
Expect FAIL: `ImportError: Failed to import test module: test_anim_gui_settings`.

- [ ] **Step 3: Implement.**

**`tools/anim_gui/__init__.py`**

```python
"""Animation GUI: a Qt-free core and a thin PyQt6 layer (tools/anim_gui/qt)."""
```

**`tools/anim_gui/settings.py`**

```python
"""Export settings and the command line they make. No Qt, no rendering."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from tools.gui_common import REPOSITORY_ROOT
from tools.gui_common.files import check_name

EXPORT_SCRIPT = REPOSITORY_ROOT / "tools" / "anim" / "export_animation.py"
VIDEO_DIRECTORY = REPOSITORY_ROOT / "videos"
PRESETS = ("ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow", "slower", "veryslow")
FORMATS = ("mp4", "png")


@dataclass(frozen=True, slots=True)
class ExportSettings:
    record: Path
    output: Path                       # a .mp4 file for "mp4", a folder for "png"
    format: str = "mp4"
    frame_rate: float = 120.0
    width: int = 1920
    height: int = 1080
    crf: int = 18
    preset: str = "medium"
    time_range: tuple[float, float] | None = None
    stl_body: Path | None = None
    stl_wheel: Path | None = None
    body_length: float | None = None
    body_width: float | None = None
    body_height: float | None = None
    wheel_radius: float = 0.2
    wheel_width: float = 0.18
    transform_scale: float = 1.0
    transform_x: float = 0.0
    transform_z: float = 0.0
    force_top_view: bool = False
    quasi_static_attitude: bool = False

    def check(self) -> None:
        """Raise ``ValueError`` with a message the user can act on. The CLI checks again."""
        if self.format not in FORMATS:
            raise ValueError(f"unknown format {self.format!r}")
        if not self.record.is_file():
            raise ValueError(f"the record does not exist: {self.record}")
        for label, value in (("frame rate", self.frame_rate), ("wheel radius", self.wheel_radius),
                             ("wheel width", self.wheel_width), ("scale", self.transform_scale)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{label} must be a positive number")
        for label, value in (("width", self.width), ("height", self.height)):
            if value <= 0 or value % 2:
                raise ValueError(f"{label} must be a positive even number of pixels (video needs even sizes)")
        if not 0 <= self.crf <= 51:
            raise ValueError("quality (CRF) must be 0 to 51")
        if self.preset not in PRESETS:
            raise ValueError(f"unknown encoder preset {self.preset!r}")
        if self.time_range is not None:
            start, end = self.time_range
            if not (math.isfinite(start) and math.isfinite(end) and start < end):
                raise ValueError("the time range must have a start that is less than its end")
        for label, value in (("body length", self.body_length), ("body width", self.body_width),
                             ("body height", self.body_height)):
            if value is not None and not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{label} must be a positive number")
        if self.format == "mp4" and self.output.suffix.lower() != ".mp4":
            raise ValueError("the video file name must end in .mp4")

    def arguments(self, *, dry_run: bool = False, progress: bool = False) -> list[str]:
        """The arguments after the script name."""
        args = ["--record", str(self.record)]
        if not dry_run:
            args += ["--png-sequence" if self.format == "png" else "--output", str(self.output)]
        args += ["--frame-rate", repr(self.frame_rate), "--width", str(self.width), "--height", str(self.height),
                 "--crf", str(self.crf), "--preset", self.preset]
        if self.time_range is not None:
            args += ["--time-range", repr(self.time_range[0]), repr(self.time_range[1])]
        for flag, value in (("--stl-body", self.stl_body), ("--stl-wheel", self.stl_wheel),
                            ("--body-length", self.body_length), ("--body-width", self.body_width),
                            ("--body-height", self.body_height)):
            if value is not None:
                args += [flag, str(value)]
        args += ["--wheel-radius", repr(self.wheel_radius), "--wheel-width", repr(self.wheel_width),
                 "--transform-scale", repr(self.transform_scale), "--transform-x", repr(self.transform_x),
                 "--transform-z", repr(self.transform_z)]
        if self.force_top_view:
            args.append("--force-top-view")
        if self.quasi_static_attitude:
            args.append("--quasi-static-attitude")
        if dry_run:
            args.append("--dry-run")
        if progress:
            args.append("--progress")
        return args


def default_output(record: Path, fmt: str, directory: Path = VIDEO_DIRECTORY) -> Path:
    """A name under ``videos/`` built from the record's folder and file name."""
    parent = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(record).parent.name).strip("_.-") or "record"
    stem = f"{parent}_{Path(record).stem}".strip("_.-")
    check_name(stem, "file name")
    return directory / (f"{stem}_frames" if fmt == "png" else f"{stem}.mp4")


def preview_settings(settings: ExportSettings, time: float, folder: Path, *, width: int = 640,
                     height: int = 360, window: float = 0.05) -> ExportSettings:
    """Settings that render a few frames around ``time`` into ``folder`` as PNG (a still preview)."""
    from dataclasses import replace
    return replace(settings, format="png", output=Path(folder), width=width - width % 2, height=height - height % 2,
                   time_range=(time, time + window))
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_settings -v
```
Expect `Ran 6 tests` and `OK`.

- [ ] **Step 5: Commit.**

```bash
git add tools/anim_gui/__init__.py \
  tools/anim_gui/settings.py \
  tests/test_anim_gui_settings.py
git commit -m "feat: add the export settings for the animation GUI"
```


### Task 3: Add the dry-run inspection

`inspect` calls the parser and `describe` of `tools/anim/export_animation.py` in the same process, with `--dry-run`. It never starts a render. It returns the text that the exporter would print and the reason a request is refused.

**Files:**
- Create: `tools/anim_gui/inspect_plan.py`
- Test: `tests/test_anim_gui_inspect.py`

**Interfaces:**
- Consumes: `ExportSettings` (Task 2). `tools.anim.export_animation` (`build_parser`, `describe`, `CliError`). `tools.anim.encode`: `EncoderUnavailable`, `find_ffmpeg`.
- Produces:
  - `class PlanSummary` fields `text: str`, `frames: int`, `first_time: float`, `last_time: float`, `visible: tuple[str, ...]`, `hidden: tuple[str, ...]`, `warnings: tuple[str, ...]`, `model: str`, `has_wheel_data: bool`
  - `encoder_problem() -> str | None`
  - `namespace(settings: ExportSettings) -> argparse.Namespace`
  - `inspect(settings: ExportSettings) -> PlanSummary`
  - `record_span(path: Path) -> tuple[float, float]`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_inspect.py`**

```python
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim.encode import EncoderUnavailable
from tools.anim_gui import inspect_plan, settings as S


def make_settings(record, **changes):
    return replace(S.ExportSettings(record=Path(record), output=Path(record).with_suffix(".mp4")), **changes)


class InspectTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.record, _ = record_file(self._tmp.name, BASE + WHEEL + DYNAMIC)

    def test_inspect_matches_the_dry_run(self):
        summary = inspect_plan.inspect(make_settings(self.record))
        self.assertEqual(summary.frames, 73)
        self.assertEqual((summary.first_time, round(summary.last_time, 3)), (0.0, 0.6))
        self.assertIn("frames: 73 at 120 fps", summary.text)
        self.assertIn("visible windows:", summary.text)
        self.assertTrue(summary.visible)

    def test_range_changes_the_frame_count(self):
        summary = inspect_plan.inspect(make_settings(self.record, time_range=(0.1, 0.3)))
        self.assertEqual(summary.frames, 25)

    def test_range_outside_the_record_is_a_message(self):
        with self.assertRaisesRegex(ValueError, "outside the record"):
            inspect_plan.inspect(make_settings(self.record, time_range=(0.0, 9.0)))

    def test_a_file_that_is_not_a_record(self):
        bad = Path(self._tmp.name) / "bad.npz"
        bad.write_bytes(b"not a record")
        with self.assertRaisesRegex(ValueError, "not a valid run record"):
            inspect_plan.inspect(make_settings(bad))
        with self.assertRaisesRegex(ValueError, "not a valid run record"):
            inspect_plan.record_span(bad)

    def test_record_span(self):
        first, last = inspect_plan.record_span(self.record)
        self.assertEqual((first, round(last, 3)), (0.0, 0.6))

    def test_encoder_problem(self):
        with mock.patch.object(inspect_plan, "find_ffmpeg", side_effect=EncoderUnavailable("ffmpeg was not found on PATH")):
            self.assertIn("ffmpeg was not found", inspect_plan.encoder_problem())
        with mock.patch.object(inspect_plan, "find_ffmpeg", return_value="/usr/bin/ffmpeg"):
            self.assertIsNone(inspect_plan.encoder_problem())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_inspect -v
```
Expect FAIL: `ImportError: Failed to import test module: test_anim_gui_inspect`.

- [ ] **Step 3: Implement.**

**`tools/anim_gui/inspect_plan.py`**

```python
"""Read a record and describe what an export would do. Cheap: no matplotlib, no ffmpeg."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from tools.anim import export_animation as cli
from tools.anim.encode import EncoderUnavailable, find_ffmpeg
from tools.anim_gui.settings import ExportSettings


@dataclass(frozen=True, slots=True)
class PlanSummary:
    text: str                       # the same text as ``--dry-run`` prints
    frames: int
    first_time: float
    last_time: float
    visible: tuple[str, ...]
    hidden: tuple[str, ...]
    warnings: tuple[str, ...]
    model: str
    has_wheel_data: bool


def encoder_problem() -> str | None:
    """None when ffmpeg can be started, else the CLI's own message."""
    try:
        find_ffmpeg()
    except EncoderUnavailable as error:
        return str(error)
    return None


def namespace(settings: ExportSettings) -> argparse.Namespace:
    """Parse the settings with the CLI's own parser, so the GUI cannot drift from it."""
    return cli.build_parser().parse_args(settings.arguments(dry_run=True))


def inspect(settings: ExportSettings) -> PlanSummary:
    """Raise ``ValueError`` with the CLI's message when the record or the options are not usable."""
    settings.check()
    try:
        args = namespace(settings)
    except SystemExit as error:  # argparse prints to stderr and exits; turn it into a message
        raise ValueError("the options were rejected by the exporter") from error
    try:
        plan = cli.Plan(args)
    except cli.CliError as error:
        raise ValueError(str(error)) from error
    return PlanSummary(cli.describe(plan), plan.source.count, float(plan.record.time[0]),
                       float(plan.record.time[-1]), tuple(plan.visibility.visible), tuple(plan.visibility.hidden),
                       tuple(plan.warning_lines()), plan.source.model_name, "top" in plan.visibility.visible)


def record_span(path: Path) -> tuple[float, float]:
    """The first and last time of a record, for the range controls."""
    from simplified_vehicle_models.results.run_record import RunRecordError, load_run_record
    try:
        record = load_run_record(Path(path), verify_sources=False)
    except (RunRecordError, OSError, ValueError) as error:
        raise ValueError(f"{path} is not a valid run record: {error}") from error
    return float(record.time[0]), float(record.time[-1])
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_inspect -v
```
Expect `Ran 6 tests` and `OK`.

- [ ] **Step 5: Commit.**

```bash
git add tools/anim_gui/inspect_plan.py \
  tests/test_anim_gui_inspect.py
git commit -m "feat: add the dry-run inspection for the animation GUI"
```


### Task 4: Add the progress parser

The exporter prints one `--progress` line for each step. `parse_progress` reads it. `describe_result` turns an exit code and the last error line into one sentence for the status bar. `parse_error` finds the exporter's own reason in the output.

**Files:**
- Create: `tools/anim_gui/progress.py`
- Test: `tests/test_anim_gui_progress.py`

**Interfaces:**
- Consumes: Nothing from earlier tasks. The line format is `frame {done}/{total}  {rate:6.1f} frames/s  eta {remaining:6.0f} s`.
- Produces:
  - `EXIT_MESSAGES`
  - `class Progress` fields `done: int`, `total: int`, `frames_per_second: float`, `eta_seconds: float` methods `fraction() -> float`
  - `parse_progress(line: str) -> Progress | None`
  - `parse_error(line: str) -> str | None`
  - `describe_result(code: int, state: str, errors: list[str]) -> str`
  - `format_eta(seconds: float) -> str`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_progress.py`**

```python
import math
import tempfile
import unittest
from unittest import mock

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim_gui import progress


class ProgressTests(unittest.TestCase):
    def test_parses_the_exporters_line(self):
        value = progress.parse_progress("frame 30/72    41.2 frames/s  eta      1 s")
        self.assertEqual((value.done, value.total, value.frames_per_second, value.eta_seconds), (30, 72, 41.2, 1.0))
        self.assertAlmostEqual(value.fraction, 30 / 72)

    def test_nan_eta(self):
        value = progress.parse_progress("frame 1/72     0.0 frames/s  eta    nan s")
        self.assertTrue(math.isnan(value.eta_seconds))
        self.assertEqual(progress.format_eta(value.eta_seconds), "estimating…")

    def test_other_lines_are_not_progress(self):
        for line in ("record: x", "frames: 73 at 120 fps = 0.600 s of video", "wrote out.mp4", ""):
            self.assertIsNone(progress.parse_progress(line))

    def test_the_regex_matches_what_the_exporter_prints(self):
        import contextlib, io
        from tools.anim import export_animation as cli
        text = io.StringIO()
        with mock.patch("tools.anim.render.render_frames") as render, \
                mock.patch("tools.anim.panels.load_track_geometry"), mock.patch("tools.anim.render.build_figure",
                                                                                 return_value=(None, None, None)), \
                mock.patch.object(cli, "make_writer"), contextlib.redirect_stderr(text):
            render.side_effect = lambda figure, panels, source, writer, progress: progress(5, 10)
            tmp = tempfile.mkdtemp()
            record, _ = record_file(tmp, BASE + WHEEL + DYNAMIC)
            plan = cli.Plan(cli.build_parser().parse_args(["--record", str(record), "--png-sequence", tmp + "/f", "--progress"]))
            cli.run(plan)
        value = progress.parse_progress(text.getvalue())
        self.assertEqual((value.done, value.total), (5, 10))

    def test_error_line_and_result_text(self):
        self.assertEqual(progress.parse_error("error: --time-range: bad"), "--time-range: bad")
        self.assertIsNone(progress.parse_error("frame 1/2"))
        self.assertEqual(progress.describe_result(0, "done", []), "Done.")
        self.assertIn("Install ffmpeg", progress.describe_result(3, "done", ["ffmpeg was not found"]))
        self.assertIn("bad range", progress.describe_result(2, "done", ["bad range"]))
        self.assertIn("removed", progress.describe_result(-1, "cancelled", []))
        self.assertIn("exit code 9", progress.describe_result(9, "done", []))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_progress -v
```
Expect FAIL: `ImportError: Failed to import test module: test_anim_gui_progress`.

- [ ] **Step 3: Implement.**

**`tools/anim_gui/progress.py`**

```python
"""Read the exporter's output lines."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

_PROGRESS = re.compile(r"frame\s+(\d+)/(\d+)\s+([\d.]+|nan)\s+frames/s\s+eta\s+(\d+|nan)\s+s")
_ERROR = re.compile(r"^error:\s*(.*)$")

EXIT_MESSAGES = {
    2: "The exporter refused the request.",
    3: "No video encoder is available. Install ffmpeg, or export a PNG sequence.",
}


@dataclass(frozen=True, slots=True)
class Progress:
    done: int
    total: int
    frames_per_second: float
    eta_seconds: float          # nan when unknown

    @property
    def fraction(self) -> float:
        return self.done / self.total if self.total else 0.0


def parse_progress(line: str) -> Progress | None:
    match = _PROGRESS.search(line)
    if match is None:
        return None
    done, total = int(match.group(1)), int(match.group(2))
    rate = float(match.group(3)) if match.group(3) != "nan" else math.nan
    eta = float(match.group(4)) if match.group(4) != "nan" else math.nan
    return Progress(done, total, rate, eta)


def parse_error(line: str) -> str | None:
    match = _ERROR.match(line.strip())
    return match.group(1) if match else None


def describe_result(code: int, state: str, errors: list[str]) -> str:
    """One sentence for the status bar."""
    if state == "cancelled":
        return "Cancelled. The unfinished file was removed."
    if state == "timed_out":
        return "Stopped: the export ran longer than the time limit."
    if state == "failed_to_start":
        return "The exporter could not be started."
    if code == 0:
        return "Done."
    detail = errors[-1] if errors else ""
    return f"{EXIT_MESSAGES.get(code, f'The exporter failed with exit code {code}.')} {detail}".strip()


def format_eta(seconds: float) -> str:
    if not math.isfinite(seconds):
        return "estimating…"
    minutes, rest = divmod(int(seconds + 0.5), 60)
    return f"{minutes} min {rest:02d} s" if minutes else f"{rest} s"
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_progress -v
```
Expect `Ran 5 tests` and `OK`.

- [ ] **Step 5: Commit.**

```bash
git add tools/anim_gui/progress.py \
  tests/test_anim_gui_progress.py
git commit -m "feat: add the progress parser for the animation GUI"
```


### Task 5: Add the record finder

`find_records` lists the `run_record.npz` files below a folder, newest first, with a limit. `study_records` reads a `study.json` of the simulation GUI (schema `sim-study-v1`) and lists the cases that wrote a record. `remove_partials` deletes only the unfinished files of an export.

**Files:**
- Create: `tools/anim_gui/records.py`
- Test: `tests/test_anim_gui_records.py`

**Interfaces:**
- Consumes: `tools.gui_common.files.is_inside` from the track plan (Task 2).
- Produces:
  - `RECORD_NAME`
  - `class RecordChoice` fields `label: str`, `path: Path`
  - `study_records(study_json: Path) -> list[RecordChoice]`
  - `find_records(folder: Path, *, limit: int=500) -> list[RecordChoice]`
  - `remove_partials(output: Path) -> list[Path]`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_records.py`**

```python
import json
import tempfile
import unittest
from pathlib import Path

from tools.anim_gui import records


class RecordsTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def study(self, cases):
        path = self.root / "study.json"
        path.write_text(json.dumps({"schema": "sim-study-v1", "cases": cases}), encoding="utf-8")
        return path

    def test_study_lists_only_cases_with_a_record(self):
        (self.root / "a").mkdir()
        (self.root / "a" / "run_record.npz").write_bytes(b"x")
        path = self.study([{"id": "c1", "label": "Case one", "run_record": "a/run_record.npz"},
                           {"id": "c2", "label": "Case two", "run_record": None},
                           {"id": "c3", "label": "Gone", "run_record": "missing/run_record.npz"},
                           {"id": "c4", "label": "Escape", "run_record": "../outside.npz"}])
        self.assertEqual([c.label for c in records.study_records(path)], ["Case one"])

    def test_bad_study_files(self):
        bad = self.root / "bad.json"
        bad.write_text("{", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "cannot read"):
            records.study_records(bad)
        bad.write_text(json.dumps({"schema": "other", "cases": []}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "not a sim-study-v1"):
            records.study_records(bad)

    def test_find_records(self):
        for name in ("x/y", "z"):
            (self.root / name).mkdir(parents=True)
            (self.root / name / "run_record.npz").write_bytes(b"x")
        self.assertEqual([c.label for c in records.find_records(self.root)], ["z", "x/y"])

    def test_remove_partials_video(self):
        output = self.root / "v.mp4"
        partial = self.root / "v.partial.mp4"
        partial.write_bytes(b"x")
        output.write_bytes(b"keep")
        self.assertEqual(records.remove_partials(output), [partial])
        self.assertTrue(output.exists() and not partial.exists())
        self.assertEqual(records.remove_partials(output), [])

    def test_remove_partials_png_only_when_the_folder_is_all_frames(self):
        frames = self.root / "frames"
        frames.mkdir()
        (frames / "frame_000000.png").write_bytes(b"x")
        self.assertEqual(records.remove_partials(frames), [frames])
        self.assertFalse(frames.exists())
        mixed = self.root / "mixed"
        mixed.mkdir()
        (mixed / "frame_000000.png").write_bytes(b"x")
        (mixed / "notes.txt").write_text("mine")
        self.assertEqual(records.remove_partials(mixed), [])
        self.assertTrue((mixed / "notes.txt").exists())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_records -v
```
Expect FAIL: `ImportError: Failed to import test module: test_anim_gui_records`.

- [ ] **Step 3: Implement.**

**`tools/anim_gui/records.py`**

```python
"""Find run records: one file, a folder, or the cases of a ``study.json``."""
from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

from tools.gui_common.files import is_inside

RECORD_NAME = "run_record.npz"


@dataclass(frozen=True, slots=True)
class RecordChoice:
    label: str
    path: Path


def study_records(study_json: Path) -> list[RecordChoice]:
    """The finished cases of a study that wrote a record. ``ValueError`` for an unreadable study."""
    study_json = Path(study_json).resolve()
    try:
        data = json.loads(study_json.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {study_json}: {error}") from error
    if data.get("schema") != "sim-study-v1" or not isinstance(data.get("cases"), list):
        raise ValueError(f"{study_json} is not a sim-study-v1 file")
    choices = []
    for case in data["cases"]:
        relative = case.get("run_record")
        if not relative:
            continue
        path = (study_json.parent / relative).resolve()
        if is_inside(path, study_json.parent) and path.is_file():
            choices.append(RecordChoice(str(case.get("label") or case.get("id")), path))
    return choices


def find_records(folder: Path, *, limit: int = 500) -> list[RecordChoice]:
    """Every ``run_record.npz`` below ``folder`` (at most ``limit``), nearest first."""
    folder = Path(folder).resolve()
    found = sorted(folder.rglob(RECORD_NAME), key=lambda p: (len(p.parts), str(p)))[:limit]
    return [RecordChoice(path.parent.relative_to(folder).as_posix() or ".", path) for path in found]


def remove_partials(output: Path) -> list[Path]:
    """Delete what a killed export leaves behind. Returns what was removed.

    A video export writes ``<stem>.partial.mp4`` and renames it when it is complete, so a
    killed run leaves the partial file and never a half-written final file. A PNG export
    leaves frames in its folder; they are removed only when the folder holds nothing else.
    """
    output = Path(output)
    removed: list[Path] = []
    if output.suffix.lower() == ".mp4":
        partial = output.with_name(f"{output.stem}.partial.mp4")
        if partial.exists():
            partial.unlink()
            removed.append(partial)
    elif output.is_dir():
        entries = list(output.iterdir())
        if entries and all(p.name.startswith("frame_") and p.suffix == ".png" for p in entries):
            shutil.rmtree(output)
            removed.append(output)
    return removed
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_records -v
```
Expect `Ran 5 tests` and `OK`.

- [ ] **Step 5: Commit.**

```bash
git add tools/anim_gui/records.py \
  tests/test_anim_gui_records.py
git commit -m "feat: add the record finder for the animation GUI"
```


### Task 6: Add the main window and the application

`MainWindow` owns one `ProcessJob` and a mode: `idle`, `export` or `preview`. An export has a timeout of 6 hours. A preview has a timeout of 120 s. The window shows the dry-run text, the progress bar, and a still frame in a `QLabel`. It has no playback. It saves its geometry with `QSettings("Simplified-Vehicle-Models", "anim-gui")`. `ANIM_GUI_QUIT_AFTER_MS` closes the window after that many milliseconds.

**Files:**
- Create: `tools/anim_gui/qt/__init__.py`, `tools/anim_gui/qt/main_window.py`, `tools/anim_gui/app.py`
- Test: `tests/test_anim_gui_qt.py`

**Interfaces:**
- Consumes: `settings`, `inspect_plan`, `progress`, `records` (Tasks 2 to 5). `tools.gui_common.qt.process`: `ProcessJob`, `python_command` (track plan, Task 3). The real-record tests build a run with `real_child("planar-7dof", "--record")` from `tests.test_run_record_runner`.
- Produces:
  - `EXPORT_TIMEOUT_SECONDS`
  - `PREVIEW_TIMEOUT_SECONDS`
  - `class MainWindow`: `current_format() -> str`; `current_settings() -> S.ExportSettings`; `open_study(path: Path) -> bool`; `load_record() -> None`; `refresh_plan() -> None`; `preview_time() -> float`; `start_export() -> None`; `start_preview() -> None`; `cancel() -> None`; `closeEvent(event) -> None`
  - `main(argv: list[str] | None=None) -> int`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_qt.py`**

```python
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

try:
    from PyQt6.QtCore import QEventLoop, QTimer
    from PyQt6.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])


if QApplication is not None:
    from tests.test_animation_cli import record_file
    from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
    from tests.test_run_record_runner import real_child
    from tools.anim_gui import inspect_plan
    from tools.anim_gui.qt.main_window import MainWindow

HAVE_FFMPEG = shutil.which("ffmpeg") is not None


def wait_for_idle(window, seconds=90):
    loop = QEventLoop()
    timer = QTimer()
    timer.timeout.connect(lambda: loop.quit() if window.mode == "idle" else None)
    timer.start(50)
    QTimer.singleShot(int(seconds * 1000), loop.quit)
    loop.exec()
    timer.stop()
    return window.mode == "idle"


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.synthetic, _ = record_file(cls._tmp.name, BASE + WHEEL + DYNAMIC)
        root, code, _ = real_child("planar-7dof", "--record")
        assert code == 0
        cls.real = root / "run_record.npz"

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def setUp(self):
        self.warnings = []
        original = (QMessageBox.question, QMessageBox.warning, QMessageBox.information)
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.Yes)
        QMessageBox.warning = staticmethod(lambda *a, **k: self.warnings.append(a[2]))
        QMessageBox.information = staticmethod(lambda *a, **k: self.warnings.append(a[2]))
        self.addCleanup(lambda: (setattr(QMessageBox, "question", original[0]), setattr(QMessageBox, "warning", original[1]),
                                 setattr(QMessageBox, "information", original[2])))
        self.window = MainWindow()
        self.addCleanup(self.window.close)
        self.out = tempfile.TemporaryDirectory()
        self.addCleanup(self.out.cleanup)

    def open(self, path):
        self.window.record.setText(str(path))
        self.window.load_record()

    def test_a_good_record_shows_the_plan_and_enables_export(self):
        self.open(self.synthetic)
        self.assertIn("frames: 73 at 120 fps", self.window.plan_text.toPlainText())
        self.assertTrue(self.window.export_button.isEnabled())
        self.assertEqual(self.window.problem.text(), "")
        self.assertTrue(self.window.output.text().endswith(".mp4") or self.window.current_format() == "png")

    def test_a_bad_file_shows_the_message_and_disables_export(self):
        bad = Path(self.out.name) / "bad.npz"
        bad.write_bytes(b"nope")
        self.open(bad)
        self.assertIn("not a valid run record", self.window.problem.text())
        self.assertFalse(self.window.export_button.isEnabled())

    def test_changing_the_range_updates_the_plan(self):
        self.open(self.synthetic)
        self.window.use_range.setChecked(True)
        self.window.start.setValue(0.1)
        self.window.end.setValue(0.3)
        self.window.refresh_plan()
        self.assertIn("frames: 25 at 120 fps", self.window.plan_text.toPlainText())

    def test_odd_size_is_explained_not_exported(self):
        self.open(self.synthetic)
        self.window.width.setValue(641)
        self.window.refresh_plan()
        self.assertIn("even", self.window.problem.text())
        self.assertFalse(self.window.export_button.isEnabled())

    def test_without_ffmpeg_mp4_is_disabled_and_png_is_chosen(self):
        with mock.patch.object(inspect_plan, "find_ffmpeg", side_effect=inspect_plan.EncoderUnavailable("ffmpeg was not found on PATH")):
            window = MainWindow()
            self.addCleanup(window.close)
        self.assertFalse(window.format.model().item(window.format.findData("mp4")).isEnabled())
        self.assertEqual(window.current_format(), "png")
        self.assertIn("MP4 export is off", window.encoder_note.text())

    def test_study_lists_cases(self):
        import json
        study = Path(self.out.name) / "study.json"
        shutil.copy(self.real, Path(self.out.name) / "run_record.npz")
        study.write_text(json.dumps({"schema": "sim-study-v1", "cases": [
            {"id": "a", "label": "Case A", "run_record": "run_record.npz"}, {"id": "b", "label": "B", "run_record": None}]}))
        self.assertTrue(self.window.open_study(study))
        self.assertEqual(self.window.cases.count(), 1)
        self.assertEqual(self.window.record.text(), str(Path(self.out.name, "run_record.npz").resolve()))
        self.assertTrue(self.window.export_button.isEnabled())

    def test_png_export_of_a_real_record(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        self.window.output.setText(str(Path(self.out.name) / "frames"))
        self.window.width.setValue(320)
        self.window.height.setValue(240)
        self.window.rate.setValue(20.0)
        self.window.refresh_plan()
        self.window.start_export()
        self.assertEqual(self.window.mode, "export")
        self.assertFalse(self.window.export_button.isEnabled())
        self.assertTrue(self.window.cancel_button.isEnabled())
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        self.assertGreater(len(list((Path(self.out.name) / "frames").glob("frame_*.png"))), 5)
        self.assertEqual(self.window.bar.value(), 1000)
        self.assertIn("Done", self.window.statusBar().currentMessage())

    def test_cancel_removes_the_unfinished_frames(self):
        self.open(self.real)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        frames = Path(self.out.name) / "frames"
        self.window.output.setText(str(frames))
        self.window.width.setValue(1280)
        self.window.height.setValue(720)
        self.window.rate.setValue(120.0)
        self.window.refresh_plan()
        self.window.start_export()
        loop = QEventLoop()
        QTimer.singleShot(2500, loop.quit)
        loop.exec()
        self.window.cancel()
        self.assertTrue(wait_for_idle(self.window, 30))
        self.assertFalse(frames.exists())
        self.assertIn("Cancelled", self.window.statusBar().currentMessage())
        self.assertTrue(self.window.export_button.isEnabled())

    def test_the_synthetic_record_fails_with_the_exporters_message(self):
        # A synthetic record names no track, so a real render exits 2 and the GUI must say why.
        self.open(self.synthetic)
        self.window.format.setCurrentIndex(self.window.format.findData("png"))
        self.window.output.setText(str(Path(self.out.name) / "frames"))
        self.window.start_export()
        self.assertTrue(wait_for_idle(self.window))
        self.assertIn("refused", self.window.statusBar().currentMessage())
        self.assertIn("cannot load the track", self.window.statusBar().currentMessage())

    def test_preview_shows_a_still(self):
        self.open(self.real)
        self.window.time_slider.setValue(500)
        self.window.start_preview()
        self.assertEqual(self.window.mode, "preview")
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        pixmap = self.window.picture.pixmap()
        self.assertFalse(pixmap is None or pixmap.isNull(), self.window.picture.text())

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg is not installed")
    def test_mp4_export_and_cancel_leave_no_partial_file(self):
        self.open(self.real)
        video = Path(self.out.name) / "v.mp4"
        self.window.format.setCurrentIndex(self.window.format.findData("mp4"))
        self.window.output.setText(str(video))
        self.window.width.setValue(320)
        self.window.height.setValue(240)
        self.window.rate.setValue(30.0)
        self.window.refresh_plan()
        self.window.start_export()
        self.assertTrue(wait_for_idle(self.window), self.window.log.toPlainText())
        self.assertTrue(video.is_file() and video.stat().st_size > 0)
        self.assertFalse(video.with_name("v.partial.mp4").exists())

    def test_app_main_quits_after_the_hook(self):
        from tools.anim_gui import app as app_module
        os.environ["ANIM_GUI_QUIT_AFTER_MS"] = "50"
        self.addCleanup(os.environ.pop, "ANIM_GUI_QUIT_AFTER_MS", None)
        self.assertEqual(app_module.main([str(self.synthetic)]), 0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_qt -v
```
Expect FAIL: `ImportError: Failed to import test module: test_anim_gui_qt`.

- [ ] **Step 3: Implement.**

**`tools/anim_gui/qt/__init__.py`** is an empty file.

**`tools/anim_gui/qt/main_window.py`**

```python
"""The animation GUI main window."""
from __future__ import annotations

import tempfile
from dataclasses import replace
from pathlib import Path

from PyQt6.QtCore import QSettings, Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
                             QProgressBar, QPushButton, QSlider, QSpinBox, QSplitter, QVBoxLayout, QWidget)

from tools.anim_gui import inspect_plan, progress, records, settings as S
from tools.gui_common import REPOSITORY_ROOT
from tools.gui_common.qt.process import ProcessJob, python_command

EXPORT_TIMEOUT_SECONDS = 6 * 3600.0
PREVIEW_TIMEOUT_SECONDS = 120.0


def _spin(low, high, value, decimals, step):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    return box


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Animation GUI")
        self.settings = QSettings("Simplified-Vehicle-Models", "anim-gui")
        self.job = ProcessJob(self)
        self.job.line.connect(self._line)
        self.job.finished.connect(self._finished)
        self.mode = "idle"             # idle | export | preview
        self.errors: list[str] = []
        self.summary: inspect_plan.PlanSummary | None = None
        self.span = (0.0, 1.0)
        self._preview_folder: tempfile.TemporaryDirectory | None = None
        self._active_output: Path | None = None

        # record
        self.record = QLineEdit()
        self.record.editingFinished.connect(self.load_record)
        browse = QPushButton("Record…")
        browse.clicked.connect(self._browse_record)
        study = QPushButton("Study…")
        study.clicked.connect(self._browse_study)
        self.cases = QComboBox()
        self.cases.currentIndexChanged.connect(self._case_chosen)
        self.cases.setVisible(False)

        # options
        self.format = QComboBox()
        self.format.addItem("MP4 video (H.264)", "mp4")
        self.format.addItem("PNG sequence", "png")
        self.format.currentIndexChanged.connect(self._format_changed)
        self.output = QLineEdit()
        output_browse = QPushButton("Browse…")
        output_browse.clicked.connect(self._browse_output)
        self.rate = _spin(1.0, 1000.0, 120.0, 1, 10.0)
        self.width = QSpinBox()
        self.width.setRange(2, 7680)
        self.width.setSingleStep(2)
        self.width.setValue(1920)
        self.height = QSpinBox()
        self.height.setRange(2, 4320)
        self.height.setSingleStep(2)
        self.height.setValue(1080)
        self.crf = QSpinBox()
        self.crf.setRange(0, 51)
        self.crf.setValue(18)
        self.preset = QComboBox()
        self.preset.addItems(S.PRESETS)
        self.preset.setCurrentText("medium")
        self.use_range = QCheckBox("Render only a time range")
        self.start = _spin(0.0, 1e6, 0.0, 3, 0.1)
        self.end = _spin(0.0, 1e6, 1.0, 3, 0.1)
        self.top_view = QCheckBox("Force the top view")
        self.quasi = QCheckBox("Approximate attitude (quasi-static)")
        self.stl_body = QLineEdit()
        self.stl_wheel = QLineEdit()
        self.wheel_radius = _spin(0.01, 5.0, 0.2, 3, 0.01)
        self.wheel_width = _spin(0.01, 5.0, 0.18, 3, 0.01)
        self.scale = _spin(1e-6, 1e6, 1.0, 6, 0.001)
        for widget in (self.rate, self.start, self.end, self.wheel_radius, self.wheel_width, self.scale):
            widget.editingFinished.connect(self.refresh_plan)
        for widget in (self.width, self.height, self.crf):
            widget.editingFinished.connect(self.refresh_plan)
        for widget in (self.use_range, self.top_view, self.quasi):
            widget.toggled.connect(self.refresh_plan)
        self.preset.currentIndexChanged.connect(self.refresh_plan)
        self.stl_body.editingFinished.connect(self.refresh_plan)
        self.stl_wheel.editingFinished.connect(self.refresh_plan)

        # plan + preview
        self.plan_text = QPlainTextEdit()
        self.plan_text.setReadOnly(True)
        self.problem = QLabel()
        self.problem.setWordWrap(True)
        self.problem.setStyleSheet("color: #b00020")
        self.encoder_note = QLabel()
        self.encoder_note.setWordWrap(True)
        self.encoder_note.setStyleSheet("color: #8a5a00")
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_label = QLabel("t = 0.000 s")
        self.time_slider.valueChanged.connect(self._slider_moved)
        self.preview_button = QPushButton("Preview this moment")
        self.preview_button.clicked.connect(self.start_preview)
        self.picture = QLabel("No preview yet. In-app playback is not offered; export the video to watch it.")
        self.picture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.picture.setMinimumSize(480, 270)
        self.picture.setWordWrap(True)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.speed = QLabel()
        self.export_button = QPushButton("Export")
        self.export_button.clicked.connect(self.start_export)
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)

        form = QFormLayout()
        top = QHBoxLayout()
        top.addWidget(self.record, 1)
        top.addWidget(browse)
        top.addWidget(study)
        form.addRow("Run record", top)
        form.addRow("Study case", self.cases)
        form.addRow("Format", self.format)
        line = QHBoxLayout()
        line.addWidget(self.output, 1)
        line.addWidget(output_browse)
        form.addRow("Output", line)
        form.addRow("Frame rate (fps)", self.rate)
        size = QHBoxLayout()
        size.addWidget(self.width)
        size.addWidget(QLabel("×"))
        size.addWidget(self.height)
        form.addRow("Size (even pixels)", size)
        form.addRow("Quality (CRF, lower is better)", self.crf)
        form.addRow("Encoder preset", self.preset)
        form.addRow(self.use_range)
        span = QHBoxLayout()
        span.addWidget(self.start)
        span.addWidget(QLabel("to"))
        span.addWidget(self.end)
        form.addRow("Time range (s)", span)
        form.addRow(self.top_view)
        form.addRow(self.quasi)
        shape = QGroupBox("Vehicle shape (display only)")
        shape_form = QFormLayout(shape)
        shape_form.addRow("Body STL", self.stl_body)
        shape_form.addRow("Wheel STL", self.stl_wheel)
        shape_form.addRow("Wheel radius (m)", self.wheel_radius)
        shape_form.addRow("Wheel width (m)", self.wheel_width)
        shape_form.addRow("STL scale", self.scale)
        left = QVBoxLayout()
        left.addLayout(form)
        left.addWidget(shape)
        left.addStretch()
        left_widget = QWidget()
        left_widget.setLayout(left)

        right = QVBoxLayout()
        right.addWidget(self.encoder_note)
        right.addWidget(self.problem)
        right.addWidget(QLabel("What the export will do"))
        right.addWidget(self.plan_text, 2)
        moment = QHBoxLayout()
        moment.addWidget(self.time_slider, 1)
        moment.addWidget(self.time_label)
        moment.addWidget(self.preview_button)
        right.addLayout(moment)
        right.addWidget(self.picture, 3)
        run = QHBoxLayout()
        run.addWidget(self.bar, 1)
        run.addWidget(self.speed)
        run.addWidget(self.export_button)
        run.addWidget(self.cancel_button)
        right.addLayout(run)
        right.addWidget(self.log, 1)
        right_widget = QWidget()
        right_widget.setLayout(right)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(1, 2)
        self.setCentralWidget(splitter)
        self._apply_encoder()
        geometry = self.settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1280, 820)
        self._buttons()

    # -- state ------------------------------------------------------------------------------
    def _apply_encoder(self) -> None:
        problem = inspect_plan.encoder_problem()
        index = self.format.findData("mp4")
        self.format.model().item(index).setEnabled(problem is None)
        self.encoder_note.setText(f"MP4 export is off. {problem}" if problem else "")
        if problem:
            self.format.setCurrentIndex(self.format.findData("png"))

    def current_format(self) -> str:
        return self.format.currentData()

    def _buttons(self) -> None:
        idle = self.mode == "idle"
        ready = idle and self.summary is not None
        self.export_button.setEnabled(ready)
        self.preview_button.setEnabled(ready)
        self.cancel_button.setEnabled(not idle)
        self.record.setEnabled(idle)
        self.format.setEnabled(idle)

    def current_settings(self) -> S.ExportSettings:
        text = self.record.text().strip()
        output = self.output.text().strip()
        fmt = self.current_format()
        optional = lambda widget: Path(widget.text().strip()) if widget.text().strip() else None  # noqa: E731
        return S.ExportSettings(
            record=Path(text), output=Path(output) if output else S.default_output(Path(text), fmt), format=fmt,
            frame_rate=self.rate.value(), width=self.width.value(), height=self.height.value(), crf=self.crf.value(),
            preset=self.preset.currentText(),
            time_range=(self.start.value(), self.end.value()) if self.use_range.isChecked() else None,
            stl_body=optional(self.stl_body), stl_wheel=optional(self.stl_wheel),
            wheel_radius=self.wheel_radius.value(), wheel_width=self.wheel_width.value(),
            transform_scale=self.scale.value(), force_top_view=self.top_view.isChecked(),
            quasi_static_attitude=self.quasi.isChecked())

    # -- choosing a record -------------------------------------------------------------------
    def _browse_record(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Run record", str(REPOSITORY_ROOT), "Run record (*.npz)")
        if path:
            self.cases.setVisible(False)
            self.record.setText(path)
            self.load_record()

    def _browse_study(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Study", str(REPOSITORY_ROOT), "Study (study.json)")
        if path:
            self.open_study(Path(path))

    def open_study(self, path: Path) -> bool:
        try:
            choices = records.study_records(path)
        except ValueError as error:
            QMessageBox.warning(self, "Not opened", str(error))
            return False
        if not choices:
            QMessageBox.information(self, "No records", "None of the cases in this study wrote a run record.")
            return False
        self.cases.blockSignals(True)
        self.cases.clear()
        for choice in choices:
            self.cases.addItem(choice.label, choice.path)
        self.cases.blockSignals(False)
        self.cases.setVisible(True)
        self._case_chosen()
        return True

    def _case_chosen(self) -> None:
        path = self.cases.currentData()
        if path is not None:
            self.record.setText(str(path))
            self.load_record()

    def load_record(self) -> None:
        text = self.record.text().strip()
        self.summary = None
        if not text:
            self._buttons()
            return
        try:
            self.span = inspect_plan.record_span(Path(text))
        except ValueError as error:
            self.problem.setText(str(error))
            self.plan_text.clear()
            self._buttons()
            return
        for box, value in ((self.start, self.span[0]), (self.end, self.span[1])):
            box.blockSignals(True)
            box.setRange(self.span[0], self.span[1])
            box.setValue(value)
            box.blockSignals(False)
        self.output.setText(str(S.default_output(Path(text), self.current_format())))
        self.refresh_plan()

    def _format_changed(self) -> None:
        text = self.record.text().strip()
        if text:
            self.output.setText(str(S.default_output(Path(text), self.current_format())))
        self.refresh_plan()

    def _browse_output(self) -> None:
        if self.current_format() == "png":
            chosen = QFileDialog.getExistingDirectory(self, "Frames folder", self.output.text())
        else:
            chosen, _ = QFileDialog.getSaveFileName(self, "Video file", self.output.text(), "MP4 video (*.mp4)")
        if chosen:
            self.output.setText(chosen)

    # -- the plan ---------------------------------------------------------------------------
    def refresh_plan(self) -> None:
        if not self.record.text().strip() or self.mode != "idle":
            return
        try:
            self.summary = inspect_plan.inspect(self.current_settings())
        except ValueError as error:
            self.summary = None
            self.problem.setText(str(error))
            self.plan_text.clear()
        else:
            self.problem.setText("")
            self.plan_text.setPlainText(self.summary.text)
        self._buttons()

    def _slider_moved(self) -> None:
        self.time_label.setText(f"t = {self.preview_time():.3f} s")

    def preview_time(self) -> float:
        low, high = (self.start.value(), self.end.value()) if self.use_range.isChecked() else self.span
        return low + (high - low) * self.time_slider.value() / 1000.0

    # -- running ----------------------------------------------------------------------------
    def _run(self, settings: S.ExportSettings, mode: str, timeout: float) -> None:
        self.mode, self.errors = mode, []
        self._active_output = settings.output
        self.bar.setValue(0)
        self.speed.setText("")
        self.log.clear()
        self._buttons()
        self.job.start(python_command(S.EXPORT_SCRIPT, *settings.arguments(progress=True)), timeout_seconds=timeout)

    def start_export(self) -> None:
        if self.mode != "idle" or self.summary is None:
            return
        settings = self.current_settings()
        try:
            settings.check()
        except ValueError as error:
            QMessageBox.warning(self, "Not exported", str(error))
            return
        target = settings.output
        if target.exists() and any(target.iterdir() if target.is_dir() else [target]):
            if QMessageBox.question(self, "Replace?", f"{target} already exists. Replace it?") != QMessageBox.StandardButton.Yes:
                return
        target.parent.mkdir(parents=True, exist_ok=True) if settings.format == "mp4" else target.mkdir(parents=True, exist_ok=True)
        self._run(settings, "export", EXPORT_TIMEOUT_SECONDS)

    def start_preview(self) -> None:
        if self.mode != "idle" or self.summary is None:
            return
        self._drop_preview()
        self._preview_folder = tempfile.TemporaryDirectory(prefix="anim-gui-preview-")
        folder = Path(self._preview_folder.name) / "frames"
        folder.mkdir()
        last = self.span[1]
        time = min(self.preview_time(), last - 0.06 if last - self.span[0] > 0.06 else self.span[0])
        settings = S.preview_settings(self.current_settings(), max(time, self.span[0]), folder)
        settings = replace(settings, time_range=(settings.time_range[0], min(settings.time_range[1], last)))
        self._run(settings, "preview", PREVIEW_TIMEOUT_SECONDS)

    def cancel(self) -> None:
        if self.mode != "idle":
            self.job.cancel()

    def _drop_preview(self) -> None:
        if self._preview_folder is not None:
            self._preview_folder.cleanup()
            self._preview_folder = None

    def _line(self, text: str) -> None:
        value = progress.parse_progress(text)
        if value is not None:
            self.bar.setValue(int(value.fraction * 1000))
            self.speed.setText(f"{value.done}/{value.total}  {value.frames_per_second:.1f} fps  eta {progress.format_eta(value.eta_seconds)}")
            return
        error = progress.parse_error(text)
        if error:
            self.errors.append(error)
        self.log.appendPlainText(text)

    def _finished(self, code: int, state: str) -> None:
        mode, output = self.mode, self._active_output
        self.mode = "idle"
        if state != "done" and mode == "export" and output is not None:
            records.remove_partials(output)
        message = progress.describe_result(code, state, self.errors)
        if mode == "preview":
            self._show_preview(code, state, message)
        else:
            self.statusBar().showMessage(message if code or state != "done" else f"Done. Wrote {output}", 15000)
            if code == 0 and state == "done":
                self.bar.setValue(1000)
        self._buttons()

    def _show_preview(self, code: int, state: str, message: str) -> None:
        frames = sorted((Path(self._preview_folder.name) / "frames").glob("frame_*.png")) if self._preview_folder else []
        if code == 0 and state == "done" and frames:
            pixmap = QPixmap(str(frames[0]))
            self.picture.setPixmap(pixmap.scaled(self.picture.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                 Qt.TransformationMode.SmoothTransformation))
            self.statusBar().showMessage("Preview ready.", 5000)
        else:
            self.picture.setPixmap(QPixmap())
            self.picture.setText(f"Preview failed. {message}")
            self.statusBar().showMessage("Preview failed.", 8000)

    def closeEvent(self, event) -> None:
        if self.mode != "idle":
            if QMessageBox.question(self, "Quit", "An export is running. Cancel it and quit?") != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self.job.cancel()
        self._drop_preview()
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()
```

**`tools/anim_gui/app.py`**

```python
"""Start the animation GUI: ``PYTHONPATH=src:. python3 tools/anim_gui/app.py [run_record.npz]``."""
from __future__ import annotations

import os
import sys
from pathlib import Path

if __package__ in {None, ""}:
    ROOT = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(ROOT / "src"), str(ROOT)]

from PyQt6.QtCore import QTimer  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from tools.anim_gui.qt.main_window import MainWindow  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    application = QApplication.instance() or QApplication([sys.argv[0], *arguments])
    application.setApplicationName("Animation GUI")
    window = MainWindow()
    if arguments:
        window.record.setText(arguments[0])
        window.load_record()
    window.show()
    delay = os.environ.get("ANIM_GUI_QUIT_AFTER_MS")
    if delay:
        QTimer.singleShot(int(delay), application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_qt -v
```
Expect `Ran 12 tests` and `OK`. The MP4 test skips when `ffmpeg` is not on `PATH`.

- [ ] **Step 5: Commit.**

```bash
git add tools/anim_gui/qt/__init__.py \
  tools/anim_gui/qt/main_window.py \
  tools/anim_gui/app.py \
  tests/test_anim_gui_qt.py
git commit -m "feat: add the animation GUI main window and application"
```


### Task 7: Write the guide and finish

The guide and the `tools-layout.md` row make the GUI findable. The docs test checks both, and that `/videos/` stays ignored.

**Files:**
- Create: `docs/dev/animation-gui.md`
- Modify: `docs/dev/tools-layout.md`
- Test: `tests/test_anim_gui_docs.py`

**Interfaces:**
- Consumes: The packages of Tasks 2 to 6.
- Produces:
  - `docs/dev/animation-gui.md`
  - a `tools/anim_gui/` row in `docs/dev/tools-layout.md`

- [ ] **Step 1: Write the failing tests.**

**`tests/test_anim_gui_docs.py`**

```python
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class AnimGuiDocsTests(unittest.TestCase):
    def test_the_guide_names_the_start_command(self):
        text = (ROOT / "docs" / "dev" / "animation-gui.md").read_text(encoding="utf-8")
        self.assertIn("python3 tools/anim_gui/app.py", text)

    def test_tools_layout_lists_the_package_and_links_the_guide(self):
        text = (ROOT / "docs" / "dev" / "tools-layout.md").read_text(encoding="utf-8")
        self.assertRegex(text, r"\| `tools/anim_gui/` \|[^\n]*\[animation-gui\.md\]\(animation-gui\.md\)")

    def test_the_default_output_folder_is_ignored(self):
        lines = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()
        self.assertIn("/videos/", lines)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and see them fail.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_docs -v
```
Expect FAIL: `FileNotFoundError: [Errno 2] No such file or directory: '<root>/docs/dev/animation-gui.md'`.

- [ ] **Step 3: Implement.**

**`docs/dev/animation-gui.md`**

````markdown
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

The default output is `videos/<record folder>_<record name>.mp4`, or the folder `videos/<record folder>_<record name>_frames` for PNG. Git ignores `videos/`. The GUI asks before it replaces an existing file or a non-empty folder.

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
````

Add the row. The change is this diff. If the rows of the track GUI or the vehicle GUI are already there, put the new row next to them. The track plan adds the `tools/gui_common/` row; do not add it again. `/videos/` is already in `.gitignore`.

```diff
diff --git a/docs/dev/tools-layout.md b/docs/dev/tools-layout.md
index d3af4bd..59dca9a 100644
--- a/docs/dev/tools-layout.md
+++ b/docs/dev/tools-layout.md
@@ -12,6 +12,7 @@ Run a tool from the repository root by its file path, for example `PYTHONPATH=sr
 | `tools/tracks/` | Build closed track geometry; `open_course.py` builds open courses from straight, arc, and clothoid sections, and `build_fsae_dynamic_tracks.py` makes the FSAE acceleration and skidpad tracks. See [fsae-dynamic-tracks.md](fsae-dynamic-tracks.md) |
 | `tools/model_data/` | Generate or export model data: motor map, tire, legacy powertrain; `generate_component_sweep.py` writes one component file per value for a GUI sweep |
 | `tools/sim_gui/` | Run simulations from a GUI: component swaps, sweeps, run records. See [simulation-gui.md](simulation-gui.md) |
+| `tools/anim_gui/` | Export a run record as a video or PNG frames from a GUI. See [animation-gui.md](animation-gui.md) |
 | `tools/run/` | Run the forward driver and the paired point-mass solver |
 | `tools/gates/` | Acceptance, refinement, tube, and between-step checks |
 | `tools/reports/` | Comparison tools, report writers, plots, correlation export |
```

- [ ] **Step 4: Run the tests and see them pass.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_docs -v
```
Expect `Ran 3 tests` and `OK`.

- [ ] **Step 5: Commit.**

```bash
git add docs/dev/animation-gui.md \
  tests/test_anim_gui_docs.py \
  docs/dev/tools-layout.md
git commit -m "docs: add the animation GUI guide"
```


### Task 8: Check the whole branch

**Files:** none.

- [ ] **Step 1: Run the new tests together.**

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_anim_gui_settings tests.test_anim_gui_inspect tests.test_anim_gui_progress \
  tests.test_anim_gui_records tests.test_anim_gui_qt tests.test_anim_gui_docs -q
```

Expect `Ran 37 tests` and `OK`. One test skips when `ffmpeg` is not installed.

- [ ] **Step 2: Start the application once.** The hook closes the window after 300 ms.

```bash
ANIM_GUI_QUIT_AFTER_MS=300 PYTHONPATH=src:. python3 tools/anim_gui/app.py; echo "exit $?"
```

Expect `exit 0`.

- [ ] **Step 3: Run the full suite and compare it with the baseline of Task 1.** It takes about 8 minutes.

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -q > /tmp/after_full.log 2>&1
grep -E "^(FAIL|ERROR):" /tmp/after_full.log | sed "s#$PWD#<root>#g" | sort > /tmp/after_fails.txt
diff /tmp/baseline_fails.txt /tmp/after_fails.txt && echo "same failures as the baseline"
```

Expect `same failures as the baseline`. The test count in the log is the count of Task 1 plus 37. A line only in `/tmp/after_fails.txt` is a regression. Fix it before you merge.

- [ ] **Step 4: Export one real video by hand.** Make a run record with the planar 7DoF runner (see `docs/dev/animation-export.md`). Start `PYTHONPATH=src:. python3 tools/anim_gui/app.py <record>`. Check that the panel shows the frame count. Set **Render only a time range** to 0 s – 2 s and select **Export**. Check that the progress bar moves and that `videos/<name>.mp4` plays. Start a second export and select **Cancel**. Check that no `.partial.mp4` file stays in `videos/`.

- [ ] **Step 5: Push the branch and open the pull request.** Name the known baseline failures in the description.

## Out of Scope (later work)

- In-app playback, a scrub bar and a live view. (The user chose no playback.)
- Changing the renderer, the panels or the encoder in `tools/anim/`.
- Batch export of a whole study in one click. Export the records one at a time.
- A change of the layout of the animation panels from the GUI.
- Windows and Linux tests. Only macOS is tested.

## Self-Review

1. **Spec coverage.** Section 6 of the investigation lists the options (Task 2), the dry run (Task 3), the progress and exit codes (Task 4), the record finder and the study list (Task 5), the child process, the cancel and the clean-up (Task 6), the preview (Task 6) and the ffmpeg check (Task 3, Task 6). Section 7 lists the shared package (Branch Dependency), the guide (Task 7) and the baseline (Task 1). Section 8 decision 4 (no playback) is in Fixed Decisions.
2. **Placeholders.** Every code step shows the file as tested in a clean clone. Each Expect line comes from a run of this plan on a clone of `main` plus Tasks 2 and 3 of the track plan.
3. **Type consistency.** `ExportSettings`, `PlanSummary`, `Progress`, `RecordChoice`, the mode names `idle`, `export` and `preview`, and the state names of `ProcessJob` (`done`, `cancelled`, `timed_out`, `failed_to_start`) are the same in every task that names them.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-10-19-animation-gui.md`. Two execution options:

1. **Subagent-Driven (recommended).** A fresh subagent runs each task, with a review between tasks.
2. **Inline Execution.** Run the tasks in one session with `executing-plans`, with checkpoints.

**Which approach?**
