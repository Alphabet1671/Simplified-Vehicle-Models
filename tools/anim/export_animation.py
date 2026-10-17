#!/usr/bin/env python3
"""Export a run record as a 120 fps video (or a PNG sequence).

    python3 tools/anim/export_animation.py --record run_record.npz --output lap.mp4
    python3 tools/anim/export_animation.py --record run_record.npz --dry-run

The window set follows the channels in the record: a window whose model feature is
missing is not drawn and the footer says why. ``--dry-run`` prints the plan and
renders nothing. The STL options change the display only; they never reach a
simulation number.

Run from the repository root; the script adds ``src`` and the root to ``sys.path``.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
for _path in (REPOSITORY_ROOT, REPOSITORY_ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy as np  # noqa: E402

from simplified_vehicle_models.results.run_record import (  # noqa: E402
    RunRecordError, check_sources, load_run_record)
from tools.anim.encode import (EncoderUnavailable, FfmpegFrameWriter,  # noqa: E402
                               PngSequenceWriter)
from tools.anim.layout import window_grid, window_visibility  # noqa: E402
from tools.anim.resample import FrameSource  # noqa: E402
from tools.anim.scene import (VehicleScene, default_body_size,  # noqa: E402
                              placeholder_scene, transform)
from tools.anim.stl import StlError, read_stl, wheel_cylinder_triangles  # noqa: E402
from tools.anim.style import AnimationStyle  # noqa: E402

#: Rough H.264 size model for flat vector graphics; replaced by the measured value in
#: docs/validation/iem24-murphy/animation-export.md.
BITS_PER_PIXEL_PER_FRAME = 0.02


class CliError(Exception):
    """A user-facing failure: printed as ``error: ...`` and exits with status 2."""


def positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not np.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(f"must be finite and greater than zero, got {text}")
    return value


def positive_even_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not an integer") from None
    if value <= 0 or value % 2:
        raise argparse.ArgumentTypeError(f"must be a positive even integer, got {text}")
    return value


def finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not np.isfinite(value):
        raise argparse.ArgumentTypeError(f"must be finite, got {text}")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export a run record as a video (two side-by-side windows: the track "
                    "and the vehicle views).")
    parser.add_argument("--record", required=True, type=Path, help="run_record.npz to animate")
    parser.add_argument("--output", type=Path, help="output video (.mp4, H.264 yuv420p)")
    parser.add_argument("--png-sequence", type=Path, metavar="DIR",
                        help="write frame_000000.png ... into DIR instead of a video")
    parser.add_argument("--frame-rate", type=positive_float, default=120.0,
                        help="frames per second (default 120)")
    parser.add_argument("--width", type=positive_even_int, default=1920)
    parser.add_argument("--height", type=positive_even_int, default=1080)
    parser.add_argument("--crf", type=int, default=18, choices=range(0, 52), metavar="0-51",
                        help="x264 quality, lower is better (default 18)")
    parser.add_argument("--preset", default="medium", help="x264 preset (default medium)")
    parser.add_argument("--time-range", type=finite_float, nargs=2, metavar=("T0", "T1"),
                        help="render only record times T0..T1 (seconds)")
    scene = parser.add_argument_group("vehicle shape (display only)")
    scene.add_argument("--stl-body", type=Path, help="STL file replacing the body box")
    scene.add_argument("--stl-wheel", type=Path,
                       help="STL file replacing the wheel cylinders (axis along +y, centred)")
    scene.add_argument("--body-length", type=positive_float, metavar="M")
    scene.add_argument("--body-width", type=positive_float, metavar="M")
    scene.add_argument("--body-height", type=positive_float, metavar="M")
    scene.add_argument("--wheel-radius", type=positive_float, default=0.2, metavar="M")
    scene.add_argument("--wheel-width", type=positive_float, default=0.18, metavar="M")
    scene.add_argument("--transform-scale", type=positive_float, default=1.0,
                       help="scale applied to STL files (e.g. 0.001 for millimetres)")
    scene.add_argument("--transform-x", type=finite_float, default=0.0, metavar="M",
                       help="translation of the STL body along body +x")
    scene.add_argument("--transform-z", type=finite_float, default=0.0, metavar="M",
                       help="translation of the STL body along body +z (down)")
    windows = parser.add_argument_group("windows")
    windows.add_argument("--force-top-view", action="store_true",
                         help="draw the top view even when the record has no wheel data")
    windows.add_argument("--quasi-static-attitude", action="store_true",
                         help="draw rear/right views from the APPROXIMATE quasi-static "
                              "roll and pitch when the record has no dynamic ones")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the plan and exit without rendering or writing")
    parser.add_argument("--progress", action="store_true", help="print progress to stderr")
    return parser


# -- planning -------------------------------------------------------------------------


class Plan:
    """Everything decided before the first frame: source, windows, scene, notes."""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        if not args.record.is_file():
            raise CliError(f"--record: no such file: {args.record}")
        try:
            self.record = load_run_record(args.record, verify_sources=False)
        except RunRecordError as error:
            raise CliError(f"--record: {args.record} is not a valid run record: {error}") \
                from error
        self.extra_warnings: list[str] = []
        try:
            check_sources(self.record.manifest, REPOSITORY_ROOT)
        except RunRecordError as error:
            self.extra_warnings.append(f"record sources: {error} (the animation is unaffected)")
        t_start, t_end = None, None
        if args.time_range is not None:
            t_start, t_end = args.time_range
            first, last = float(self.record.time[0]), float(self.record.time[-1])
            if not t_start < t_end:
                raise CliError(f"--time-range: T0 must be less than T1, got {t_start} {t_end}")
            if t_start < first - 1e-9 or t_end > last + 1e-9:
                raise CliError(f"--time-range: {t_start}..{t_end} s lies outside the record "
                               f"({first:.3f}..{last:.3f} s)")
        self.source = FrameSource(self.record, args.frame_rate, t_start=t_start, t_end=t_end,
                                  allow_quasi_static=args.quasi_static_attitude)
        self.visibility = window_visibility(
            self.source, allow_quasi_static=args.quasi_static_attitude,
            force_top_view=args.force_top_view)
        self.layout = window_grid(self.visibility)
        self.scene, self.scene_notes = self._scene()

    def _scene(self) -> tuple[VehicleScene, tuple[str, ...]]:
        args = self.args
        geometry = self.record.manifest.vehicle_geometry
        if not geometry.get("wheel_positions_body_m"):
            raise CliError("the record has no vehicle_geometry.wheel_positions_body_m; "
                           "the vehicle views need it")
        wheel_radius, wheel_width = args.wheel_radius, args.wheel_width
        default = default_body_size(geometry, wheel_width)
        size = (args.body_length or default[0], args.body_width or default[1],
                args.body_height or default[2])
        base = placeholder_scene(geometry, body_size=size, wheel_radius=wheel_radius,
                                 wheel_width=wheel_width)
        notes = [f"body block {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m (placeholder, "
                 f"display only)"]
        body, wheels = base.triangles, base.wheels
        move = (args.transform_x, 0.0, args.transform_z)
        try:
            if args.stl_body is not None:
                body = transform(read_stl(args.stl_body), np.eye(3), move, args.transform_scale)
                notes = [f"body STL {args.stl_body.name}: scale {args.transform_scale:g}, "
                         f"shift x {args.transform_x:+.3f} m, z {args.transform_z:+.3f} m "
                         f"(display only)"]
            if args.stl_wheel is not None:
                mesh = transform(read_stl(args.stl_wheel), np.eye(3), (0.0, 0.0, 0.0),
                                 args.transform_scale)
                wheels = tuple((mesh, centre) for _, centre in base.wheels)
                notes.append(f"wheel STL {args.stl_wheel.name}: scale {args.transform_scale:g}")
        except (StlError, OSError) as error:
            raise CliError(f"cannot read the STL file: {error}") from error
        return VehicleScene(body, wheels), tuple(notes)

    @property
    def output_target(self) -> Path | None:
        return self.args.png_sequence or self.args.output

    def hidden_reasons(self) -> list[tuple[str, str]]:
        """Each hidden window with the visibility note that explains it."""

        reasons = []
        for name in self.visibility.hidden:
            prefix = "rear/right hidden" if name in ("rear", "right") else f"{name} hidden"
            note = next((n for n in self.visibility.notes if n.startswith(prefix)),
                        f"{name} is hidden")
            reasons.append((name, note))
        return reasons

    def warning_lines(self) -> list[str]:
        return list(self.source.warnings) + self.extra_warnings

    def estimated_bytes(self) -> int:
        pixels = self.args.width * self.args.height
        return int(self.source.count * pixels * BITS_PER_PIXEL_PER_FRAME / 8.0)


def describe(plan: Plan) -> str:
    args, source = plan.args, plan.source
    duration = source.count / args.frame_rate
    lines = [
        f"record: {args.record}",
        f"model: {source.model_name}",
        f"record rows: {plan.record.rows}   time {plan.record.time[0]:.3f}..{plan.record.time[-1]:.3f} s",
        f"frames: {source.count} at {args.frame_rate:g} fps = {duration:.3f} s of video",
        f"size: {args.width}x{args.height}   layout: {plan.layout.description}",
        f"estimated output: ~{plan.estimated_bytes() / 1e6:.1f} MB (rough size model, "
        f"{BITS_PER_PIXEL_PER_FRAME} bit/pixel/frame; not measured)",
        f"channels ({len(source.channel_names)}): {', '.join(source.channel_names)}",
        f"visible windows: {', '.join(plan.visibility.visible)}",
    ]
    hidden = plan.hidden_reasons()
    lines.append("hidden windows: " + ("none" if not hidden else ", ".join(n for n, _ in hidden)))
    for reason in dict.fromkeys(reason for _, reason in hidden):
        lines.append(f"  - {reason}")
    for note in plan.visibility.notes:
        if note not in {reason for _, reason in hidden}:
            lines.append(f"note: {note}")
    if plan.visibility.attitude_is_approximate:
        lines.append("attitude: APPROXIMATE (quasi-static estimate, not a simulated state)")
    for note in plan.scene_notes:
        lines.append(f"scene: {note}")
    lines.append(f"warnings ({len(plan.warning_lines())}):")
    lines.extend(f"  {line}" for line in plan.warning_lines())
    if plan.args.dry_run:
        lines.append("dry run: nothing was rendered or written")
    return "\n".join(lines)


# -- rendering ------------------------------------------------------------------------


def make_writer(plan: Plan):
    args = plan.args
    if args.png_sequence is not None:
        return PngSequenceWriter(args.png_sequence, args.width, args.height)
    return FfmpegFrameWriter(args.output, args.width, args.height, args.frame_rate,
                             crf=args.crf, preset=args.preset)


def run(plan: Plan) -> dict:
    from tools.anim.panels import load_track_geometry
    from tools.anim.render import build_figure, render_frames

    args = plan.args
    try:
        geometry = load_track_geometry(plan.record.manifest, REPOSITORY_ROOT)
    except Exception as error:  # noqa: BLE001 - report any track problem plainly
        raise CliError(f"cannot load the track named by the record: {error}") from error
    style = AnimationStyle().with_size(args.width, args.height)
    figure, panels, _ = build_figure(
        plan.record, plan.source, plan.scene, geometry, style,
        allow_quasi_static=args.quasi_static_attitude, force_top_view=args.force_top_view,
        extra_footer=plan.scene_notes + tuple(plan.extra_warnings))
    writer = make_writer(plan)
    started = time.monotonic()

    def progress(done: int, total: int) -> None:
        if args.progress:
            elapsed = time.monotonic() - started
            rate = done / elapsed if elapsed > 0 else 0.0
            remaining = (total - done) / rate if rate > 0 else float("nan")
            print(f"frame {done}/{total}  {rate:6.1f} frames/s  eta {remaining:6.0f} s",
                  file=sys.stderr, flush=True)

    render_frames(figure, panels, plan.source, writer, progress=progress)
    wall = time.monotonic() - started
    video_seconds = plan.source.count / args.frame_rate
    return {"frames": plan.source.count, "wall_seconds": wall, "video_seconds": video_seconds,
            "ms_per_frame": 1000.0 * wall / max(plan.source.count, 1),
            "render_to_real_ratio": wall / video_seconds if video_seconds else float("nan")}


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.dry_run:
        if args.output is None and args.png_sequence is None:
            parser.error("one of --output or --png-sequence is required (or use --dry-run)")
        if args.output is not None and args.png_sequence is not None:
            parser.error("--output and --png-sequence are exclusive")
    try:
        plan = Plan(args)
        print(describe(plan))
        if args.dry_run:
            return 0
        result = run(plan)
    except CliError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except EncoderUnavailable as error:
        print(f"error: {error}", file=sys.stderr)
        return 3
    print(f"rendered {result['frames']} frames in {result['wall_seconds']:.1f} s "
          f"({result['ms_per_frame']:.1f} ms/frame); video {result['video_seconds']:.2f} s; "
          f"render/real = {result['render_to_real_ratio']:.2f}x")
    print(f"wrote {plan.output_target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
