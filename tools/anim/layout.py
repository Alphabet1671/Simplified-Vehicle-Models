"""Which windows exist and where they go (pure; no matplotlib).

A window exists only when its model feature exists. A missing feature removes the
window and the remaining windows grow to fill the space (plan section 6). The
rectangles are in figure fractions ``(left, bottom, width, height)`` so the layout
is testable without rendering.
"""
from __future__ import annotations

from dataclasses import dataclass

__all__ = ["WindowVisibility", "window_visibility", "window_grid", "Layout", "WINDOWS"]

#: Window names in drawing order.
WINDOWS = ("track", "rear", "right", "top")

#: Outer margin, gap between windows and the footer strip, as figure fractions.
MARGIN = 0.012
GAP = 0.012
FOOTER = 0.05
#: Share of the right half given to the top view when the attitude row exists.
TOP_SHARE = 0.62
#: Width share of the track window when it sits beside two attitude columns.
TRACK_SHARE_THREE = 0.5

_WHEEL_TIRE_PREFIXES = ("wheel-", "tire-")


@dataclass(frozen=True)
class WindowVisibility:
    """Which windows and sub-features are drawn, and why the others are not."""

    track: bool
    rear: bool
    right: bool
    top: bool
    tire_arrows: bool
    fz_bars: bool
    attitude_is_dynamic: bool
    attitude_is_approximate: bool = False
    notes: tuple[str, ...] = ()

    @property
    def hidden(self) -> tuple[str, ...]:
        return tuple(name for name in WINDOWS if not getattr(self, name))

    @property
    def visible(self) -> tuple[str, ...]:
        return tuple(name for name in WINDOWS if getattr(self, name))


def _model_name(record) -> str:
    if getattr(record, "model_name", None):
        return str(record.model_name)
    manifest = getattr(record, "manifest", None)
    model = getattr(manifest, "model", None) or {}
    return str(model.get("model") or model.get("plant_class") or "this model")


def window_visibility(record, *, allow_quasi_static: bool = False,
                      force_top_view: bool = False) -> WindowVisibility:
    """Decide the windows from channel availability alone.

    ``record`` is anything with ``has_channel(name)`` and a ``manifest`` (a
    ``RunRecord`` or a ``FrameSource``). Nothing here looks at the model's name:
    the name only appears in the notes.
    """

    has = record.has_channel
    names = tuple(getattr(record, "channel_names", None)
                  or getattr(getattr(record, "manifest", None), "channel_names", ()))
    model = _model_name(record)
    dynamic = has("roll") and has("pitch")
    quasi = has("roll-quasi-static") and has("pitch-quasi-static")
    show_attitude = dynamic or (quasi and allow_quasi_static)
    notes: list[str] = []
    if not show_attitude:
        if quasi:
            notes.append(f"rear/right hidden: {model} has no dynamic roll or pitch state; "
                         "pass --quasi-static-attitude to show an estimate")
        else:
            notes.append(f"rear/right hidden: {model} has no roll or pitch channel")
    approximate = show_attitude and not dynamic

    has_wheel_data = any(name.startswith(_WHEEL_TIRE_PREFIXES) for name in names)
    top = has_wheel_data or bool(force_top_view)
    if not top:
        notes.append(f"top hidden: {model} has no wheel or tire channels; "
                     "pass --force-top-view to show speed, body line and inputs")
    tire_arrows = top and has("tire-longitudinal-force") and has("tire-lateral-force")
    fz_bars = top and has("tire-vertical-force")
    if top and not tire_arrows:
        notes.append(f"tire arrows hidden: {model} has no tire force channels")
    if top and not fz_bars:
        notes.append(f"Fz bars hidden: {model} has no tire-vertical-force channel")
    return WindowVisibility(
        track=True, rear=show_attitude, right=show_attitude, top=top,
        tire_arrows=tire_arrows, fz_bars=fz_bars, attitude_is_dynamic=dynamic,
        attitude_is_approximate=approximate, notes=tuple(notes))


@dataclass(frozen=True)
class Layout:
    """Figure-fraction rectangles of the visible windows and of the footer."""

    rects: dict[str, tuple[float, float, float, float]]
    footer: tuple[float, float, float, float]
    description: str

    def __getitem__(self, name: str) -> tuple[float, float, float, float]:
        return self.rects[name]

    @property
    def windows(self) -> tuple[str, ...]:
        return tuple(name for name in WINDOWS if name in self.rects)


def window_grid(visibility: WindowVisibility, style=None) -> Layout:
    """Return the rectangles for the visible windows.

    * track, rear, right, top: track on the left over the full height; on the right
      the larger top view above, then rear (left) and right (right) below it.
    * track, rear, right: three columns, so the attitude windows take the full height.
    * track, top: two columns, top at full height.
    * track only: one window.

    ``style`` is accepted for symmetry with the renderer; the geometry depends on
    the visibility only, so a layout is decided once per run.
    """

    if not visibility.track:
        raise ValueError("the track window is always shown")
    if visibility.rear != visibility.right:
        raise ValueError("rear and right are shown together or not at all")
    left, right_edge = MARGIN, 1.0 - MARGIN
    bottom, top_edge = FOOTER, 1.0 - MARGIN
    width, height = right_edge - left, top_edge - bottom
    footer = (left, 0.0, width, FOOTER)
    attitude = visibility.rear and visibility.right

    def column(x0: float, share: float, y0: float = bottom, h: float = height):
        return (x0, y0, share, h)

    if visibility.top and attitude:
        track_w = (width - GAP) * 0.5
        half_x = left + track_w + GAP
        side_w = width - track_w - GAP
        top_h = (height - GAP) * TOP_SHARE
        row_h = height - GAP - top_h
        cell_w = (side_w - GAP) * 0.5
        rects = {
            "track": (left, bottom, track_w, height),
            "top": (half_x, bottom + row_h + GAP, side_w, top_h),
            "rear": (half_x, bottom, cell_w, row_h),
            "right": (half_x + cell_w + GAP, bottom, cell_w, row_h),
        }
        text = "2x3: track | top over rear and right"
    elif attitude:
        track_w = (width - 2 * GAP) * TRACK_SHARE_THREE
        cell_w = (width - 2 * GAP - track_w) * 0.5
        rects = {
            "track": column(left, track_w),
            "rear": column(left + track_w + GAP, cell_w),
            "right": column(left + track_w + GAP + cell_w + GAP, cell_w),
        }
        text = "1x3: track | rear | right"
    elif visibility.top:
        half = (width - GAP) * 0.5
        rects = {"track": column(left, half), "top": column(left + half + GAP, half)}
        text = "1x2: track | top"
    else:
        rects = {"track": (left, bottom, width, height)}
        text = "1x1: track"
    return Layout(rects, footer, text)
