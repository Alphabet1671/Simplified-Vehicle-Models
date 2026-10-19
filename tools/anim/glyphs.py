"""Screen-space glyph geometry for the animation panels (NumPy only).

Screen coordinates are ``(right, up)``. An angle is in degrees; ``clockwise``
means clockwise on screen, which is the sense of positive yaw seen from above.
Every function returns plain arrays so the panels only call ``set_data``.
"""
from __future__ import annotations

import numpy as np

__all__ = ["arrow_polyline", "arc_arrow", "fit_limits", "rotate_clockwise", "ARROW_POINTS"]

#: ``arrow_polyline`` always returns this many points: tail, tip, barb, tip, barb.
ARROW_POINTS = 5


def rotate_clockwise(vector, degrees: float) -> np.ndarray:
    """Rotate a screen ``(right, up)`` vector clockwise on screen."""

    angle = np.radians(degrees)
    c, s = np.cos(angle), np.sin(angle)
    right, up = float(vector[0]), float(vector[1])
    return np.array([c * right + s * up, -s * right + c * up])


def arrow_polyline(origin, vector, head_fraction: float = 0.22,
                   head_degrees: float = 24.0) -> np.ndarray:
    """One polyline ``(5, 2)`` drawing an arrow: shaft, then the two head barbs.

    A zero vector collapses to the origin and a non-finite one to ``NaN`` (which
    matplotlib does not draw), so a missing sample is never drawn as a zero arrow.
    """

    origin = np.asarray(origin, dtype=float)
    vector = np.asarray(vector, dtype=float)
    if not (np.all(np.isfinite(origin)) and np.all(np.isfinite(vector))):
        return np.full((ARROW_POINTS, 2), np.nan)
    length = float(np.hypot(*vector))
    if length == 0.0:
        return np.tile(origin, (ARROW_POINTS, 1))
    tip = origin + vector
    back = -vector * head_fraction
    return np.array([origin, tip, tip + rotate_clockwise(back, head_degrees), tip,
                     tip + rotate_clockwise(back, -head_degrees)])


def arc_arrow(center, radius: float, start_degrees: float, sweep_degrees: float,
              points: int = 24, head_fraction: float = 0.3,
              head_degrees: float = 24.0) -> np.ndarray:
    """A circular arrow ``(points + 3, 2)``: an arc, then the head barbs.

    The arc starts at ``start_degrees`` (counterclockwise from screen right) and
    sweeps ``sweep_degrees`` **clockwise** (negative sweeps run counterclockwise).
    A zero sweep collapses to a point and a non-finite one to ``NaN``.
    """

    center = np.asarray(center, dtype=float)
    if not (np.all(np.isfinite(center)) and np.isfinite(sweep_degrees)):
        return np.full((points + 3, 2), np.nan)
    angles = np.radians(start_degrees - sweep_degrees * np.linspace(0.0, 1.0, points))
    arc = center + radius * np.column_stack((np.cos(angles), np.sin(angles)))
    if sweep_degrees == 0.0:
        return np.vstack((arc, np.tile(arc[-1], (3, 1))))
    phi = angles[-1]
    direction = np.array([np.sin(phi), -np.cos(phi)]) * np.sign(sweep_degrees)
    head = min(head_fraction * radius * np.radians(abs(sweep_degrees)), radius)
    tip = arc[-1]
    back = -direction * head
    return np.vstack((arc, tip + rotate_clockwise(back, head_degrees), tip,
                      tip + rotate_clockwise(back, -head_degrees)))


def fit_limits(x_min: float, x_max: float, y_min: float, y_max: float, aspect: float,
               margin: float = 0.0):
    """Axis limits ``((x0, x1), (y0, y1))`` with equal scale that contain the box.

    ``aspect`` is the window's width over its height in pixels. The box is
    centred; the limit that is not binding grows, so the picture is never
    stretched.
    """

    if not (np.isfinite(aspect) and aspect > 0.0):
        raise ValueError("aspect must be positive and finite")
    cx, cy = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    width = max((x_max - x_min) * (1.0 + margin), 1e-9)
    height = max((y_max - y_min) * (1.0 + margin), 1e-9)
    if width / height < aspect:
        width = height * aspect
    else:
        height = width / aspect
    return (cx - 0.5 * width, cx + 0.5 * width), (cy - 0.5 * height, cy + 0.5 * height)
