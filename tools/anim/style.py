"""Look of the animation: one frozen dataclass, no matplotlib import."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AnimationStyle:
    """Figure size, colours, line widths and arrow sizes of every window."""

    width: int = 1920
    height: int = 1080
    dpi: int = 100
    background: str = "#101418"
    panel_background: str = "#161b21"
    foreground: str = "#e6e9ec"
    muted: str = "#8a949e"
    grid: str = "#2a323a"
    track: str = "#6b7682"
    traveled: str = "#4fa3ff"
    body: str = "#e6e9ec"
    velocity: str = "#3ddc84"
    acceleration: str = "#ff9f43"
    yaw: str = "#d17bff"
    horizon: str = "#7f8c99"
    tire_fx: str = "#ff5d5d"
    tire_fy: str = "#4fa3ff"
    tire_mz: str = "#d17bff"
    fz_bar: str = "#3ddc84"
    throttle: str = "#3ddc84"
    brake: str = "#ff5d5d"
    steer: str = "#4fa3ff"
    line_width: float = 1.6
    thin_line_width: float = 0.9
    font_size: float = 11.0
    #: arrow head length as a fraction of the arrow length, and its half angle
    arrow_head_fraction: float = 0.22
    arrow_head_degrees: float = 24.0
    #: arrow lengths at the run full scale, as a fraction of the window's extent
    track_velocity_fraction: float = 0.10
    track_acceleration_fraction: float = 0.07
    track_body_fraction: float = 0.045

    @property
    def figure_size(self) -> tuple[float, float]:
        return (self.width / self.dpi, self.height / self.dpi)

    def with_size(self, width: int, height: int) -> "AnimationStyle":
        from dataclasses import replace
        return replace(self, width=int(width), height=int(height))
