"""Figure assembly and the frame loop (matplotlib Agg, one canvas for the whole run).

``build_figure`` creates axes only for the visible windows and a footer that lists
every hidden window and warning, so a viewer always knows what is absent. The loop
updates artist data, draws, and hands the RGB buffer to a writer; it does not
create or clear anything per frame.
"""
from __future__ import annotations

import os
import time

os.environ.setdefault("MPLCONFIGDIR", os.path.join(os.environ.get("TMPDIR", "/tmp"), "mplcfg"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import numpy as np  # noqa: E402
from matplotlib.backends.backend_agg import FigureCanvasAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from tools.anim.layout import WindowVisibility, window_grid, window_visibility  # noqa: E402
from tools.anim.panels import build_panels  # noqa: E402
from tools.anim.resample import FrameSource  # noqa: E402
from tools.anim.style import AnimationStyle  # noqa: E402

__all__ = ["build_figure", "render_frames", "footer_lines", "frame_rgb"]

FOOTER_FONT = "monospace"


def footer_lines(visibility: WindowVisibility, warnings=(), extra=()) -> tuple[str, ...]:
    """Everything the footer prints: hidden-window notes, warnings, then ``extra``."""

    return tuple(visibility.notes) + tuple(warnings) + tuple(extra)


def build_figure(record, frame_source: FrameSource, scene, track_geometry, style: AnimationStyle,
                 *, allow_quasi_static: bool = False, force_top_view: bool = False,
                 extra_footer=()):
    """Return ``(figure, panels, visibility)`` for the visible windows only."""

    visibility = window_visibility(frame_source, allow_quasi_static=allow_quasi_static,
                                   force_top_view=force_top_view)
    layout = window_grid(visibility, style)
    figure = Figure(figsize=style.figure_size, dpi=style.dpi, facecolor=style.background)
    FigureCanvasAgg(figure)
    axes = {name: figure.add_axes(layout[name]) for name in layout.windows}
    panels = build_panels(axes, frame_source, scene, track_geometry, style, visibility)
    lines = footer_lines(visibility, frame_source.warnings, extra_footer)
    left, bottom, width, height = layout.footer
    figure.text(left, bottom + 0.5 * height, "\n".join(lines), ha="left", va="center",
                color=style.muted, fontsize=style.font_size * 0.72, family=FOOTER_FONT)
    figure.footer_lines = lines  # for tests and the CLI summary
    return figure, panels, visibility


def frame_rgb(figure: Figure) -> np.ndarray:
    """Draw the figure and return its pixels as ``(height, width, 3)`` ``uint8``."""

    canvas = figure.canvas
    canvas.draw()
    rgba = np.asarray(canvas.buffer_rgba())
    return np.ascontiguousarray(rgba[..., :3])


def render_frames(figure: Figure, panels: dict, frame_source: FrameSource, writer, *,
                  progress=None, indices=None, progress_interval: float = 1.0):
    """Render every frame (or ``indices``) and write it; returns ``writer``.

    ``progress(done, total)`` is called at most once per ``progress_interval``
    seconds of wall time, and once at the end. The writer is closed on success; on
    an error it is closed too so ffmpeg does not linger, and the error propagates.
    """

    order = list(range(frame_source.count)) if indices is None else list(indices)
    total = len(order)
    last_report = time.monotonic()
    try:
        for done, index in enumerate(order, start=1):
            for panel in panels.values():
                panel.update(index)
            writer.write(frame_rgb(figure))
            if progress is not None:
                now = time.monotonic()
                if now - last_report >= progress_interval:
                    progress(done, total)
                    last_report = now
        if progress is not None:
            progress(total, total)
    except BaseException:
        try:
            writer.close()
        except Exception:  # noqa: BLE001 - the original error is the useful one
            pass
        raise
    writer.close()
    return writer
