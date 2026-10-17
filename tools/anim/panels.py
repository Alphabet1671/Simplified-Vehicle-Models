"""The four animation windows (matplotlib artists, updated in place).

Every builder creates its artists once and returns a ``Panel``. ``Panel.update(i)``
only calls ``set_data`` / ``set_segments`` / ``set_text`` style setters: it never
clears the axes and never changes the limits, so a frame costs one draw. A channel
the record does not hold removes its artist (the legend says so); an invalid
sample draws nothing and prints ``n/a`` instead of a zero.

Screen axes are ``(right, up)``:

* track: ``Y`` east to the right, ``X`` north up;
* rear: body ``+y`` right, ``-z`` up; right: body ``+x`` right, ``-z`` up;
* top: the body rotated by ``-beta`` so the velocity is always up.

Each attitude view applies only its own angle (roll in the rear view, pitch in the
right view), so the angle read off the picture is the recorded angle exactly.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from tools.anim.glyphs import ARROW_POINTS, arc_arrow, arrow_polyline, fit_limits
from tools.anim.layout import WindowVisibility
from tools.anim.resample import FrameSource, MissingChannelError
from tools.anim.scene import (VIEWS, VehicleScene, body_rotation, build_vehicle_edges, project)
from tools.anim.style import AnimationStyle

__all__ = ["Panel", "TrackGeometry", "load_track_geometry", "build_track_panel",
           "build_attitude_panel", "build_top_panel", "build_panels"]

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
FONT = "monospace"


@dataclass(frozen=True)
class TrackGeometry:
    """The centre line of the track in world axes (``X`` north, ``Y`` east)."""

    x_north: np.ndarray
    y_east: np.ndarray
    lap_length: float = 0.0
    closed: bool = False

    @classmethod
    def from_path_coordinates(cls, coordinates, closed: bool = False) -> "TrackGeometry":
        return cls(np.asarray(coordinates.x_north, dtype=float),
                   np.asarray(coordinates.y_east, dtype=float),
                   float(getattr(coordinates, "lap_length", 0.0)), bool(closed))

    def polyline(self) -> np.ndarray:
        """Screen ``(Y, X)`` points; a closed track repeats its first point."""

        x, y = self.x_north, self.y_east
        if self.closed and x.size:
            x, y = np.append(x, x[0]), np.append(y, y[0])
        return np.column_stack((y, x))


def load_track_geometry(manifest, root=None) -> TrackGeometry:
    """Load the track named by a record manifest (``manifest.track["file"]``)."""

    from simplified_vehicle_models.tracks.profile import load_track

    name = (manifest.track or {}).get("file")
    if not name:
        raise MissingChannelError("the record names no track file")
    path = Path(name)
    if not path.is_absolute():
        path = Path(root) / path if root is not None else REPOSITORY_ROOT / path
    profile = load_track(path)
    return TrackGeometry.from_path_coordinates(profile.path_coordinates,
                                               closed=bool(profile.is_closed))


class Panel:
    """One window: its axes, its artists, a static legend and a dynamic readout."""

    name = "panel"

    def __init__(self, axes, source: FrameSource, style: AnimationStyle):
        self.axes, self.source, self.style = axes, source, style
        self._legend: list[str] = []
        axes.set_facecolor(style.panel_background)
        axes.set_xticks([])
        axes.set_yticks([])
        for spine in axes.spines.values():
            spine.set_color(style.grid)
        self._title = axes.text(0.5, 0.985, "", transform=axes.transAxes, ha="center",
                                va="top", color=style.muted, fontsize=style.font_size,
                                family=FONT)
        self._readout = axes.text(0.015, 0.985, "", transform=axes.transAxes, ha="left",
                                  va="top", color=style.foreground,
                                  fontsize=style.font_size * 0.9, family=FONT)
        self._legend_text = axes.text(0.985, 0.015, "", transform=axes.transAxes, ha="right",
                                      va="bottom", color=style.muted,
                                      fontsize=style.font_size * 0.8, family=FONT)

    # -- API -------------------------------------------------------------------------

    def update(self, index: int) -> None:  # pragma: no cover - overridden
        raise NotImplementedError

    def legend_lines(self) -> tuple[str, ...]:
        """The exact static text printed in the window's corner."""

        return tuple(self._legend)

    def readout_lines(self, index: int) -> tuple[str, ...]:
        return ()

    # -- helpers ---------------------------------------------------------------------

    def _finish(self, title: str) -> None:
        self._title.set_text(title)
        self._legend_text.set_text("\n".join(self._legend))

    def _set_readout(self, index: int) -> None:
        self._readout.set_text("\n".join(self.readout_lines(index)))

    def _pixel_aspect(self) -> float:
        box = self.axes.get_position()
        width, height = self.axes.figure.get_size_inches()
        return float(box.width * width / (box.height * height))

    def _line(self, color, width=None, **options) -> Line2D:
        line = Line2D([], [], color=color,
                      linewidth=self.style.line_width if width is None else width, **options)
        self.axes.add_line(line)
        return line

    def _set_limits(self, x_min, x_max, y_min, y_max, margin=0.05) -> tuple[float, float]:
        (x0, x1), (y0, y1) = fit_limits(x_min, x_max, y_min, y_max, self._pixel_aspect(), margin)
        self.axes.set_xlim(x0, x1)
        self.axes.set_ylim(y0, y1)
        self.axes.set_aspect("equal", adjustable="box")
        return x1 - x0, y1 - y0


def _set_arrow(line: Line2D, polyline: np.ndarray) -> None:
    line.set_data(polyline[:, 0], polyline[:, 1])


def _fmt(value, template: str, missing: str = "n/a") -> str:
    return missing if value is None else template.format(value)


def _vector_from_polar(heading: float, length: float):
    """Screen ``(Y, X)`` components of a world vector of ``length`` at ``heading``."""

    return np.array([np.sin(heading), np.cos(heading)]) * length


# ======================================================================================
# Track window
# ======================================================================================

class TrackPanel(Panel):
    name = "track"

    def __init__(self, axes, source: FrameSource, geometry: TrackGeometry, style):
        super().__init__(axes, source, style)
        for channel in ("position-x", "position-y", "heading-world"):
            source.require(channel)
        self._x = source.series("position-x")[:, 0]
        self._y = source.series("position-y")[:, 0]
        self._psi = source.series("heading-world")[:, 0]
        beta = source.series("body-slip-angle")
        self._beta = None if beta is None else beta[:, 0]
        self._speed = None if source.series("speed") is None else source.series("speed")[:, 0]
        ax, ay = source.series("acceleration-longitudinal"), source.series("acceleration-lateral")
        self._acc = None if ax is None or ay is None else (ax[:, 0], ay[:, 0])
        yaw = source.series("yaw-acceleration")
        self._yaw = None if yaw is None else yaw[:, 0]
        self.scales = source.scales()

        outline = geometry.polyline()
        pad = np.vstack((outline, np.column_stack((self._y, self._x))))
        pad = pad[np.all(np.isfinite(pad), axis=1)]
        width, height = self._set_limits(pad[:, 0].min(), pad[:, 0].max(), pad[:, 1].min(),
                                         pad[:, 1].max(), margin=0.08)
        self.extent = max(width, height)
        self.track_line = self._line(style.track, style.thin_line_width)
        self.track_line.set_data(outline[:, 0], outline[:, 1])
        self.traveled = self._line(style.traveled, style.line_width)
        self.body_axis = self._line(style.body, style.line_width * 1.4)
        self.velocity = self._line(style.velocity, style.line_width * 1.2)
        self.acceleration = self._line(style.acceleration, style.line_width * 1.2)
        self.yaw_arc = self._line(style.yaw, style.line_width)
        (self.cg_marker,) = axes.plot([], [], "o", color=style.foreground, markersize=6,
                                      markeredgecolor=style.background)
        self.velocity_full = style.track_velocity_fraction * self.extent
        self.acceleration_full = style.track_acceleration_fraction * self.extent
        self.body_length = style.track_body_fraction * self.extent
        self.yaw_radius = 0.022 * self.extent

        self._legend = [
            "white: body +x   green: velocity   orange: acceleration   violet: yaw accel.",
        ]
        if "speed" in self.scales:
            self._legend.append(
                f"velocity arrow full scale = {self.scales['speed']:.1f} m/s (run maximum)")
        else:
            self._legend.append("velocity arrow: not available (no speed channel)")
        if "acceleration" in self.scales and self._acc is not None:
            self._legend.append("acceleration arrow full scale = "
                                f"{self.scales['acceleration']:.1f} m/s^2 (run maximum)")
        else:
            self._legend.append("acceleration arrow: not available")
        if "yaw_acceleration" in self.scales and self._yaw is not None:
            self._legend.append("yaw arc: 300 deg = "
                                f"{self.scales['yaw_acceleration']:.1f} rad/s^2, clockwise = +")
        else:
            self._legend.append("yaw acceleration: not available")
        self._finish("track  (north up)")
        self.update(0)

    def readout_lines(self, index: int) -> tuple[str, ...]:
        lines = [f"t = {self.source.times[index]:8.3f} s"]
        speed = None if self._speed is None else self._speed[index]
        lines.append("v = " + _fmt(None if speed is None or np.isnan(speed) else speed,
                                   "{:6.2f} m/s"))
        if self._acc is not None:
            ax, ay = self._acc[0][index], self._acc[1][index]
            ok = np.isfinite(ax) and np.isfinite(ay)
            lines.append("a = " + (f"({ax:+6.2f}, {ay:+6.2f}) m/s^2" if ok else "n/a"))
        if self._yaw is not None:
            value = self._yaw[index]
            lines.append("yaw accel = " + (f"{value:+7.2f} rad/s^2" if np.isfinite(value)
                                           else "n/a"))
        return tuple(lines)

    def update(self, index: int) -> None:
        position = np.array([self._y[index], self._x[index]])
        psi = self._psi[index]
        self.traveled.set_data(self._y[:index + 1], self._x[:index + 1])
        self.cg_marker.set_data([position[0]], [position[1]])
        _set_arrow(self.body_axis, arrow_polyline(
            position, _vector_from_polar(psi, self.body_length), 0.2, self.style.arrow_head_degrees))
        self._velocity_arrow(index, position, psi)
        self._acceleration_arrow(index, position, psi)
        self._yaw_arc(index, position)
        self._set_readout(index)

    def _velocity_arrow(self, index, position, psi):
        speed = None if self._speed is None else self._speed[index]
        if speed is None or "speed" not in self.scales or self._beta is None:
            _set_arrow(self.velocity, np.full((ARROW_POINTS, 2), np.nan))
            return
        length = self.source.arrow_length("speed", speed, self.velocity_full)
        _set_arrow(self.velocity, arrow_polyline(
            position, _vector_from_polar(psi + self._beta[index], length),
            self.style.arrow_head_fraction, self.style.arrow_head_degrees))

    def _acceleration_arrow(self, index, position, psi):
        if self._acc is None or "acceleration" not in self.scales:
            _set_arrow(self.acceleration, np.full((ARROW_POINTS, 2), np.nan))
            return
        along, across = self._acc[0][index], self._acc[1][index]
        scale = self.acceleration_full / self.scales["acceleration"]
        # body +x is (sin psi, cos psi) on the (Y, X) screen; body +y is 90 deg clockwise
        vector = scale * (along * np.array([np.sin(psi), np.cos(psi)])
                          + across * np.array([np.cos(psi), -np.sin(psi)]))
        _set_arrow(self.acceleration, arrow_polyline(
            position, vector, self.style.arrow_head_fraction, self.style.arrow_head_degrees))

    def _yaw_arc(self, index, position):
        if self._yaw is None or "yaw_acceleration" not in self.scales:
            self.yaw_arc.set_data([], [])
            return
        sweep = 300.0 * self._yaw[index] / self.scales["yaw_acceleration"]
        poly = arc_arrow(position, self.yaw_radius, 120.0, sweep)
        _set_arrow(self.yaw_arc, poly)


def build_track_panel(axes, frame_source: FrameSource, track_geometry: TrackGeometry,
                      style: AnimationStyle | None = None) -> TrackPanel:
    return TrackPanel(axes, frame_source, track_geometry, style or AnimationStyle())


# ======================================================================================
# Rear and right windows
# ======================================================================================

class AttitudePanel(Panel):
    """Rear view (roll) or right view (pitch) against a fixed horizon."""

    def __init__(self, axes, source: FrameSource, scene: VehicleScene, view: str, style):
        if view not in ("rear", "right"):
            raise ValueError("view must be 'rear' or 'right'")
        if source.attitude_source == "none":
            raise MissingChannelError(
                f"{view}: no roll or pitch channel is available in {source.model_name}")
        super().__init__(axes, source, style)
        self.name = view
        self.view = view
        self.right, self.up = VIEWS[view]
        self.edges = build_vehicle_edges(scene)
        self.attitude = source.attitude_series()
        self.angle_column = 0 if view == "rear" else 1
        self.angle_name = "roll" if view == "rear" else "pitch"
        steering = source.series("wheel-steering-angle")
        self.steering = steering
        self.spin = source.wheel_spin_angles()
        self.approximate = source.attitude_source == "quasi-static"
        self.heave = None if source.series("heave") is None else source.series("heave")[:, 0]

        points = self.edges.posed().reshape(-1, 3)
        ground = float(np.max(np.concatenate(
            [self.edges.wheel_centres[:, 2] + self.edges.wheel_radii, [0.0]])))
        self.ground_z = ground
        screen = project(points, self.right, self.up)
        half = max(abs(screen[:, 0]).max(), 0.5)
        x_min, x_max = -1.35 * half, 1.35 * half
        y_min, y_max = -ground - 0.12, screen[:, 1].max() + 0.25
        self._set_limits(x_min, x_max, y_min, y_max, margin=0.0)
        lim = axes.get_xlim()
        self.horizon = self._line(style.horizon, style.thin_line_width, linestyle="--")
        self.horizon.set_data([lim[0], lim[1]], [-ground, -ground])
        self.vehicle = LineCollection([], colors=style.body, linewidths=style.thin_line_width * 1.3)
        axes.add_collection(self.vehicle)
        self.reference = self._line(style.velocity, style.line_width * 1.3)
        self.reference_half = float(half * 0.9)

        self._legend = [f"dashed: horizon   green: body {'y' if view == 'rear' else 'x'} axis",
                        f"{self.angle_name}: {'right side' if view == 'rear' else 'nose'} "
                        f"{'down' if view == 'rear' else 'up'} = +"]
        if self.approximate:
            self._legend.append("APPROXIMATE (quasi-static estimate, not a simulated state)")
        elif self.heave is not None:
            self._legend.append("heave is positive down")
        self._finish(f"{view} view  ({self.angle_name})")
        self.update(0)

    def _angle(self, index):
        if self.attitude is None:
            return None
        value = self.attitude[index, self.angle_column]
        return None if not np.isfinite(value) else float(value)

    def readout_lines(self, index: int) -> tuple[str, ...]:
        angle = self._angle(index)
        label = self.angle_name + (" (approx.)" if self.approximate else "")
        lines = [f"{label} = " + _fmt(None if angle is None else np.degrees(angle),
                                      "{:+6.2f} deg")]
        if self.view == "rear" and self.heave is not None:
            heave = self.heave[index]
            lines.append("heave = " + (f"{1000.0 * heave:+6.1f} mm" if np.isfinite(heave)
                                       else "n/a"))
        return tuple(lines)

    def update(self, index: int) -> None:
        angle = self._angle(index)
        if angle is None:
            self.vehicle.set_segments([])
            self.reference.set_data([], [])
            self._set_readout(index)
            return
        pitch, roll = (0.0, angle) if self.view == "rear" else (angle, 0.0)
        rotation = body_rotation(0.0, pitch, roll)
        steering = None if self.steering is None else self.steering[index]
        spin = None if self.spin is None else self.spin[index]
        if steering is not None and not np.all(np.isfinite(steering)):
            steering = None
        segments = self.edges.posed(steering, spin) @ rotation.T
        self.vehicle.set_segments(project(segments, self.right, self.up))
        axis = np.array([0.0, 1.0, 0.0]) if self.view == "rear" else np.array([1.0, 0.0, 0.0])
        ends = np.array([-axis, axis]) * self.reference_half @ rotation.T
        screen = project(ends, self.right, self.up)
        self.reference.set_data(screen[:, 0], screen[:, 1])
        self._set_readout(index)


def build_attitude_panel(axes, frame_source: FrameSource, scene: VehicleScene, view: str,
                         style: AnimationStyle | None = None) -> AttitudePanel:
    return AttitudePanel(axes, frame_source, scene, view, style or AnimationStyle())


# ======================================================================================
# Top window
# ======================================================================================

class TopPanel(Panel):
    name = "top"

    def __init__(self, axes, source: FrameSource, scene: VehicleScene, style,
                 visibility: WindowVisibility | None = None):
        super().__init__(axes, source, style)
        self.edges = build_vehicle_edges(scene)
        self.scales = source.scales()
        self.visibility = visibility
        beta = source.series("body-slip-angle")
        speed = source.series("speed")
        self._beta = None if beta is None else beta[:, 0]
        self._speed = None if speed is None else speed[:, 0]
        self._steer_angles = source.series("wheel-steering-angle")
        self._fx = source.series("tire-longitudinal-force")
        self._fy = source.series("tire-lateral-force")
        self._mz = source.series("tire-aligning-moment")
        self._fz = source.series("tire-vertical-force")
        self._throttle = source.series("throttle")
        self._brake = source.series("brake-fraction")
        self._brake_force = source.series("brake-force")
        self._steer = source.series("steering")
        want_arrows = visibility.tire_arrows if visibility is not None else True
        want_bars = visibility.fz_bars if visibility is not None else True
        self.has_arrows = bool(want_arrows and self._fx is not None and self._fy is not None
                               and "tire_force" in self.scales)
        self.has_mz = bool(self.has_arrows and self._mz is not None
                           and "tire_moment" in self.scales)
        self.has_bars = bool(want_bars and self._fz is not None
                             and "tire_vertical_force" in self.scales)

        self.reach = self.edges.extent() + 0.5 * self.edges.extent()
        self.arrow_full = 0.5 * self.edges.extent()
        self.velocity_full = 1.15 * self.edges.extent()
        reach = self.reach
        self.bar_x0 = reach + 0.35
        right_edge = self.bar_x0 + 4 * 0.3 + 0.2 if self.has_bars else reach
        self.overlay_top = -reach - 0.15
        self._set_limits(-reach, right_edge, -reach - 1.1, reach, margin=0.0)

        self.vehicle = LineCollection([], colors=style.body, linewidths=style.thin_line_width * 1.3)
        axes.add_collection(self.vehicle)
        self.velocity = self._line(style.velocity, style.line_width * 1.5)
        self.body_axis = self._line(style.body, style.line_width * 1.2)
        self.slip_arc = self._line(style.yaw, style.line_width)
        self.tire_fx = [self._line(style.tire_fx, style.line_width) for _ in range(4)]
        self.tire_fy = [self._line(style.tire_fy, style.line_width) for _ in range(4)]
        self.tire_mz = [self._line(style.tire_mz, style.line_width) for _ in range(4)]
        if not self.has_arrows:
            for line in self.tire_fx + self.tire_fy + self.tire_mz:
                line.set_visible(False)
        elif not self.has_mz:
            for line in self.tire_mz:
                line.set_visible(False)
        self._build_bars()
        self._build_overlays()
        self._legend = self._legend_text_lines()
        self._finish("top view  (velocity up)")
        self.update(0)

    # -- construction ----------------------------------------------------------------

    def _build_bars(self):
        self.fz_bars, self.fz_outline = [], []
        self.fz_height = 2 * self.reach - 0.6
        self.fz_base = -self.reach + 0.1
        if not self.has_bars:
            return
        for k, label in enumerate(("FL", "FR", "RL", "RR")):
            x = self.bar_x0 + 0.3 * k
            outline = Rectangle((x, self.fz_base), 0.22, self.fz_height, fill=False,
                                edgecolor=self.style.grid, linewidth=self.style.thin_line_width)
            bar = Rectangle((x, self.fz_base), 0.22, 0.0, facecolor=self.style.fz_bar,
                            edgecolor="none")
            self.axes.add_patch(outline)
            self.axes.add_patch(bar)
            self.axes.text(x + 0.11, self.fz_base - 0.08, label, ha="center", va="top",
                           color=self.style.muted, fontsize=self.style.font_size * 0.75,
                           family=FONT)
            self.fz_bars.append(bar)
            self.fz_outline.append(outline)

    def _build_overlays(self):
        style, reach = self.style, self.reach
        self.overlay_width = 2.0 * reach - 0.9
        x0 = -reach + 0.9
        rows = (("throttle", self._throttle, style.throttle, False),
                ("brake", self._brake if self._brake is not None else self._brake_force,
                 style.brake, False),
                ("steer", self._steer, style.steer, True))
        self.bars: dict[str, Rectangle] = {}
        self.bar_centered: dict[str, bool] = {}
        for k, (label, data, color, centered) in enumerate(rows):
            y = self.overlay_top - 0.3 * (k + 1)
            self.axes.text(x0 - 0.1, y + 0.1, label, ha="right", va="center", color=style.muted,
                           fontsize=style.font_size * 0.8, family=FONT)
            if data is None:
                self.axes.text(x0, y + 0.1, "not available", ha="left", va="center",
                               color=style.muted, fontsize=style.font_size * 0.8, family=FONT)
                continue
            self.axes.add_patch(Rectangle((x0, y), self.overlay_width, 0.2, fill=False,
                                          edgecolor=style.grid,
                                          linewidth=style.thin_line_width))
            if centered:
                self.axes.plot([x0 + 0.5 * self.overlay_width] * 2, [y, y + 0.2],
                               color=style.muted, linewidth=style.thin_line_width)
            fill = Rectangle((x0, y), 0.0, 0.2, facecolor=color, edgecolor="none")
            self.axes.add_patch(fill)
            self.bars[label] = fill
            self.bar_centered[label] = centered
        self.overlay_x0 = x0

    def _legend_text_lines(self) -> list[str]:
        lines = ["green: velocity (always up)   white: body +x   violet arc: body slip"]
        if self.has_arrows:
            lines.append("red: Fx   blue: Fy  (N, tire frame, full scale = "
                         f"{self.scales['tire_force']:.0f} N)")
            if self.has_mz:
                lines.append("violet arc at wheel: Mz (N*m), full scale = "
                             f"{self.scales['tire_moment']:.1f} N*m, clockwise = +")
            else:
                lines.append("Mz: not available")
        else:
            lines.append("Fx/Fy/Mz arrows: not available (no tire force channels)")
        if self.has_bars:
            lines.append(f"Fz bars: full scale = {self.scales['tire_vertical_force']:.0f} N")
        else:
            lines.append("Fz bars: not available (no tire-vertical-force channel)")
        if "steering" in self.scales:
            lines.append(f"steer bar: +/-{self.scales['steering']:.3f} rad (run maximum)")
        return lines

    # -- per frame -------------------------------------------------------------------

    def _beta_at(self, index):
        if self._beta is None:
            return 0.0
        value = self._beta[index]
        return float(value) if np.isfinite(value) else None

    def _screen(self, points: np.ndarray, beta: float) -> np.ndarray:
        """Body-axis points to the top view with the velocity up."""

        return project(points @ body_rotation(-beta, 0.0, 0.0).T, *VIEWS["top"])

    def readout_lines(self, index: int) -> tuple[str, ...]:
        beta = self._beta_at(index)
        lines = ["slip = " + _fmt(None if beta is None else np.degrees(beta), "{:+6.2f} deg")]
        if self._steer is not None:
            value = self._steer[index, 0]
            lines.append("steer = " + (f"{value:+.3f} rad ({np.degrees(value):+.1f} deg)"
                                       if np.isfinite(value) else "n/a"))
        if self._throttle is not None:
            value = self._throttle[index, 0]
            lines.append("throttle = " + (f"{value:.2f}" if np.isfinite(value) else "n/a"))
        if self._brake_force is not None:
            value = self._brake_force[index, 0]
            lines.append("brake = " + (f"{value:.0f} N" if np.isfinite(value) else "n/a"))
        if self.has_bars:
            loads = self._fz[index]
            lines.append("Fz = " + (" ".join(f"{v:5.0f}" for v in loads) + " N"
                                    if np.all(np.isfinite(loads)) else "n/a"))
        return tuple(lines)

    def update(self, index: int) -> None:
        beta = self._beta_at(index)
        if beta is None:
            self.vehicle.set_segments([])
            for line in [self.velocity, self.body_axis, self.slip_arc, *self.tire_fx,
                         *self.tire_fy, *self.tire_mz]:
                line.set_data([], [])
            self._update_bars(index)
            self._set_readout(index)
            return
        steering = None if self._steer_angles is None else self._steer_angles[index]
        if steering is not None and not np.all(np.isfinite(steering)):
            steering = None
        self.vehicle.set_segments(self._screen(self.edges.posed(steering), beta)[..., :2])
        origin = np.zeros(2)
        speed = None if self._speed is None else self._speed[index]
        if speed is None or not np.isfinite(speed) or "speed" not in self.scales:
            _set_arrow(self.velocity, np.full((ARROW_POINTS, 2), np.nan))
        else:
            length = self.source.arrow_length("speed", speed, self.velocity_full)
            _set_arrow(self.velocity, arrow_polyline(
                origin, np.array([0.0, length]), self.style.arrow_head_fraction,
                self.style.arrow_head_degrees))
        axis = self._screen(np.array([[self.edges.extent() * 1.1, 0.0, 0.0]]), beta)[0]
        self.body_axis.set_data([0.0, axis[0]], [0.0, axis[1]])
        sweep = np.linspace(np.radians(90.0), np.radians(90.0) + beta, 16)
        radius = 0.6 * self.edges.extent()
        self.slip_arc.set_data(radius * np.cos(sweep), radius * np.sin(sweep))
        self._update_tires(index, beta, steering)
        self._update_bars(index)
        self._set_readout(index)

    def _wheel_screen(self, beta):
        centres = self.edges.wheel_centres
        return self._screen(centres, beta)

    def _tire_vector(self, forces: np.ndarray, delta: float, along: float, beta) -> np.ndarray:
        """Screen vector of a tire-frame force along the tire ``x`` (0) or ``y`` (1) axis."""

        axis = np.array([np.cos(delta), np.sin(delta), 0.0]) if along == 0 else np.array(
            [-np.sin(delta), np.cos(delta), 0.0])
        return self._screen(axis[None, :], beta)[0]

    def _update_tires(self, index, beta, steering):
        if not self.has_arrows:
            return
        centres = self._wheel_screen(beta)
        fx, fy = self._fx[index], self._fy[index]
        mz = self._mz[index] if self.has_mz else None
        ok = np.all(np.isfinite(fx)) and np.all(np.isfinite(fy)) and steering is not None
        for k in range(4):
            if not ok:
                for line in (self.tire_fx[k], self.tire_fy[k], self.tire_mz[k]):
                    line.set_data([], [])
                continue
            delta = float(steering[k])
            scale = self.arrow_full / self.scales["tire_force"]
            x_axis = self._tire_vector(None, delta, 0, beta)
            y_axis = self._tire_vector(None, delta, 1, beta)
            _set_arrow(self.tire_fx[k], arrow_polyline(
                centres[k], x_axis * fx[k] * scale, self.style.arrow_head_fraction,
                self.style.arrow_head_degrees))
            _set_arrow(self.tire_fy[k], arrow_polyline(
                centres[k], y_axis * fy[k] * scale, self.style.arrow_head_fraction,
                self.style.arrow_head_degrees))
            if mz is not None and np.isfinite(mz[k]):
                sweep = 270.0 * mz[k] / self.scales["tire_moment"]
                _set_arrow(self.tire_mz[k], arc_arrow(centres[k], 0.16, 120.0, sweep))
            else:
                self.tire_mz[k].set_data([], [])

    def _update_bars(self, index):
        if self.has_bars:
            loads = self._fz[index]
            for k, bar in enumerate(self.fz_bars):
                value = loads[k]
                height = 0.0 if not np.isfinite(value) else (
                    self.fz_height * max(value, 0.0) / self.scales["tire_vertical_force"])
                bar.set_height(height)
        values = {"throttle": self._throttle, "steer": self._steer}
        brake = self._brake
        for label, rect in self.bars.items():
            if label == "brake":
                if brake is not None:
                    value = brake[index, 0]
                else:
                    peak = self.scales.get("brake_force")
                    raw = self._brake_force[index, 0]
                    value = raw / peak if peak else np.nan
            else:
                value = values[label][index, 0]
            if not np.isfinite(value):
                rect.set_width(0.0)
                continue
            if self.bar_centered[label]:
                full = self.scales.get("steering", 1.0)
                half = 0.5 * self.overlay_width
                width = half * np.clip(value / full, -1.0, 1.0)
                rect.set_x(self.overlay_x0 + half + min(width, 0.0))
                rect.set_width(abs(width))
            else:
                rect.set_x(self.overlay_x0)
                rect.set_width(self.overlay_width * float(np.clip(value, 0.0, 1.0)))


def build_top_panel(axes, frame_source: FrameSource, scene: VehicleScene,
                    style: AnimationStyle | None = None,
                    visibility: WindowVisibility | None = None) -> TopPanel:
    return TopPanel(axes, frame_source, scene, style or AnimationStyle(), visibility)


def build_panels(axes_by_name: dict, frame_source: FrameSource, scene: VehicleScene,
                 track_geometry: TrackGeometry, style: AnimationStyle,
                 visibility: WindowVisibility) -> dict[str, Panel]:
    """Build the panel of every visible window, in the order track, rear, right, top."""

    panels: dict[str, Panel] = {}
    if visibility.track:
        panels["track"] = build_track_panel(axes_by_name["track"], frame_source,
                                            track_geometry, style)
    for view in ("rear", "right"):
        if getattr(visibility, view):
            panels[view] = build_attitude_panel(axes_by_name[view], frame_source, scene, view,
                                                style)
    if visibility.top:
        panels["top"] = build_top_panel(axes_by_name["top"], frame_source, scene, style,
                                        visibility)
    return panels
