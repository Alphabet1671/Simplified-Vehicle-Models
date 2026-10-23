"""Energy-consistent cubic-spline minimum-time line OCP in distance.

The lateral offset is one cubic spline of the track distance, so the offset,
heading error, curvature rate, and path curvature all come from the same
representation at the nodes, the collocation midpoints, and the interior
containment samples.  Physical node speeds remain decision variables, while
Hermite--Simpson reconstructs kinetic energy ``W = v²`` with ``W_s = 2 a G``.
Acceleration is a quadratic control at the mesh nodes and midpoint.

The decision vector is grouped as
``[spline coefficients | speed nodes | acceleration nodes | midpoint
acceleration]``.  A closed track uses a periodic spline and a periodic speed
boundary; an open track keeps the natural spline and fixes only the launch
speed, leaving the terminal speed and the endpoint offsets free.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

import numpy as np


class SplineSolveError(RuntimeError):
    """Strict spline-OCP rejection with immutable raw solver diagnostics.

    This is intentionally a :class:`RuntimeError` subclass so existing callers
    retain their failure behavior while diagnostic-aware callers can persist the
    solver's last iterate without treating it as a solution.
    """

    def __init__(self, message: str, diagnostics: dict[str, object], *,
                 failed_values=None, constraint_values=None, dual_x=None,
                 dual_g=None):
        super().__init__(message)
        self.diagnostics = dict(diagnostics)
        self.failed_values = self._immutable_array(failed_values)
        self.constraint_values = self._immutable_array(constraint_values)
        self.dual_x = self._immutable_array(dual_x)
        self.dual_g = self._immutable_array(dual_g)

    @staticmethod
    def _immutable_array(values):
        if values is None:
            return None
        copied = np.array(values, dtype=float, copy=True)
        copied.setflags(write=False)
        return copied

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates
from simplified_vehicle_models.solvers.qss_line_ocp import (
    QssSettings,
    _bilinear,
    _tolerance_schedule,
    _validate_inputs,
    _validate_solver_result,
    track_sample,
)


# This is part of the trajectory representation contract: the optimizer and
# reported physical time both integrate G / sqrt(W_H) at these exact sites.
TIME_QUADRATURE_NAME = "gauss_legendre_16_energy_hermite_knot_union_v1"
_TIME_GL_NODES, _TIME_GL_WEIGHTS = np.polynomial.legendre.leggauss(16)
_TIME_ELEMENT_FRACTIONS = 0.5 * (_TIME_GL_NODES + 1.0)
_TIME_ELEMENT_WEIGHTS = 0.5 * _TIME_GL_WEIGHTS


def _time_quadrature_metadata() -> dict[str, object]:
    """Return the serialized contract for optimizer and reported physical time."""
    return {
        "name": TIME_QUADRATURE_NAME,
        "rule": "gauss_legendre",
        "order": 16,
        "state": "energy_hermite_cubic",
        "integrand": "trajectory_metric / sqrt(energy_hermite)",
        "geometry_panels": "mesh_union_reference_and_offset_knots",
        "element_fraction_nodes": _TIME_ELEMENT_FRACTIONS.tolist(),
        "element_fraction_weights": _TIME_ELEMENT_WEIGHTS.tolist(),
    }


@dataclass(frozen=True, slots=True)
class SplineSolution:
    """One accepted spline transcription solution."""

    distance: FloatArray
    offset: FloatArray
    heading_error: FloatArray
    curvature: FloatArray
    speed: FloatArray
    acceleration: FloatArray
    lap_time: float
    status: str
    iterations: int
    maximum_violation: float
    coefficients: FloatArray
    trajectory: OffsetTrajectory | None = None
    midpoint_acceleration: FloatArray | None = None
    objective_value: float | None = None
    state_evaluator: object | None = None
    state_representation: str = "energy_hs_v1"
    time_quadrature: str = TIME_QUADRATURE_NAME
    time_quadrature_metadata: dict[str, object] = field(default_factory=_time_quadrature_metadata)
    # Explicitly labels the stored controls; omitted metadata on old artifacts is legacy.
    control_representation: str = "quadratic_acceleration_v1"
    off_site_stations: FloatArray | None = None
    off_site_elements: FloatArray | None = None
    off_site_fractions: FloatArray | None = None
    stage_timings: dict[str, float] = field(default_factory=dict)
    # None preserves the historical free-line solve; otherwise this immutable
    # snapshot records coefficients eliminated from the active NLP.
    fixed_coefficients: FloatArray | None = None
    line_optimized: bool = True

    def __post_init__(self):
        if self.fixed_coefficients is not None:
            snapshot = np.array(self.fixed_coefficients, dtype=float, copy=True)
            snapshot.setflags(write=False)
            object.__setattr__(self, "fixed_coefficients", snapshot)


def _reference_curve(track):
    """Create a reference whose nested topology agrees with ``track``."""
    coordinates = track.path_coordinates
    if coordinates is None:
        raise ValueError("spline OCP requires signed path coordinates")
    if bool(coordinates.is_closed) == bool(track.is_closed):
        return ReferenceCurve(track)
    # ``replace(track, is_closed=False)`` is a historical test/caller pattern.
    # Never silently retain the nested closed topology in that cut-open view.
    normalized = PathCoordinates(
        coordinates.distance, coordinates.x_north, coordinates.y_east,
        coordinates.heading, coordinates.curvature, bool(track.is_closed),
    )
    from types import SimpleNamespace
    kwargs = {"path_coordinates": normalized, "is_closed": bool(track.is_closed)}
    if hasattr(track, "source_reference_interpolation_periodic"):
        kwargs["source_reference_interpolation_periodic"] = track.source_reference_interpolation_periodic
    return ReferenceCurve(SimpleNamespace(**kwargs))


def _curvature_pair(track, positions):
    """Legacy diagnostic curvature sampler (the OCP uses ``ReferenceCurve``)."""
    k0 = track_sample(track.curvature, track.distance, positions,
                      closed=bool(track.is_closed), period=track.lap_length)
    dk = np.gradient(track.curvature, track.distance)
    return k0, track_sample(dk, track.distance, positions,
                            closed=bool(track.is_closed), period=track.lap_length)


def _geometry(track, spline, coefficients, positions, reference=None):
    """Evaluate the authoritative analytic offset trajectory."""
    trajectory = OffsetTrajectory(
        _reference_curve(track) if reference is None else reference, spline, coefficients,
    )
    value = trajectory.evaluate(positions)
    return (np.asarray(value.offset), np.asarray(value.heading_error),
            np.asarray(value.heading_error_s), np.asarray(value.curvature),
            np.asarray(value.fold_factor))


def _spline_geometry(n, first, second, h, h_s, k, k_s):
    """Symbolic twin of :class:`OffsetTrajectory` analytic equations."""
    import casadi as ca
    fold = 1.0 - k * n
    a, b = h * fold, first
    a_s = h_s * fold - h * (k_s * n + k * first)
    metric = ca.sqrt(a * a + b * b)
    psi = ca.atan2(b, a)
    q = (a * second - b * a_s) / (a * a + b * b)
    kappa = (h * k + q) / metric
    return fold, psi, q, kappa, metric


def _is_zero_corridor(*bounds) -> bool:
    """Return true only when every source corridor datum is exactly zero."""
    return all(np.all(np.asarray(values) == 0.0) for values in bounds)


def _capability_interior_margin(settings: QssSettings) -> float:
    """Return a measured row-residual budget, not a continuous certificate."""
    return 8.0 * float(settings.tolerance)


def _validate_capability_margin(surface, settings: QssSettings) -> float:
    """Reject a chart whose vertex bands cannot contain the interior budget."""
    margin = _capability_interior_margin(settings)
    width = (np.asarray(surface.acceleration_max, dtype=float)
             - np.asarray(surface.acceleration_min, dtype=float))
    minimum_width = float(np.min(width))
    if not np.isfinite(minimum_width) or minimum_width <= 2.0 * margin:
        raise ValueError(
            "capability acceleration range must exceed twice the interior margin"
        )
    return margin


class _SplineStateEvaluator:
    """Immutable dense evaluator for the ``energy_hs_v1`` representation."""
    state_representation = "energy_hs_v1"

    def __init__(self, trajectory, mesh, speed, acceleration, midpoint_acceleration,
                 *, control_representation="quadratic_acceleration_v1"):
        self.trajectory = trajectory
        self.mesh = np.array(mesh, dtype=float, copy=True); self.mesh.setflags(write=False)
        self.speed = np.array(speed, dtype=float, copy=True); self.speed.setflags(write=False)
        self.acceleration = np.array(acceleration, dtype=float, copy=True); self.acceleration.setflags(write=False)
        self.midpoint_acceleration = np.array(midpoint_acceleration, dtype=float, copy=True); self.midpoint_acceleration.setflags(write=False)
        self.metric_nodes = np.array(trajectory.evaluate(self.mesh).metric, dtype=float, copy=True); self.metric_nodes.setflags(write=False)
        self.metric_midpoints = np.array(trajectory.evaluate(0.5 * (self.mesh[:-1] + self.mesh[1:])).metric, dtype=float, copy=True); self.metric_midpoints.setflags(write=False)
        if control_representation not in ("quadratic_acceleration_v1",
                                          "quadratic_energy_rate_over_metric_v1"):
            raise ValueError("unknown control representation")
        self.control_representation = control_representation

    def evaluate(self, station):
        s = np.asarray(station, dtype=float); scalar = s.ndim == 0; flat = np.atleast_1d(s)
        if np.any(flat < self.mesh[0]) or np.any(flat > self.mesh[-1]):
            raise ValueError("state station outside mesh")
        i = np.minimum(np.searchsorted(self.mesh, flat, side="right") - 1,
                       self.mesh.size - 2)
        x = (flat - self.mesh[i]) / (self.mesh[i + 1] - self.mesh[i])
        width = self.mesh[i + 1] - self.mesh[i]
        geometry = self.trajectory.evaluate(flat)
        g = np.asarray(geometry.metric)
        f0 = 2.0 * self.acceleration[i] * self.metric_nodes[i]
        f1 = 2.0 * self.acceleration[i + 1] * self.metric_nodes[i + 1]
        w0, w1 = self.speed[i] ** 2, self.speed[i + 1] ** 2
        w = ((2*x**3 - 3*x**2 + 1)*w0
             + (x**3 - 2*x**2 + x)*width*f0
             + (-2*x**3 + 3*x**2)*w1
             + (x**3 - x**2)*width*f1)
        w_s = ((6*x**2 - 6*x)*w0 + (3*x**2 - 4*x + 1)*width*f0
               + (-6*x**2 + 6*x)*w1 + (3*x**2 - 2*x)*width*f1) / width
        # Accepted states must satisfy the explicit Bernstein W floor. Never
        # clamp a failed reconstructed energy into a plausible speed trace.
        if not np.all(np.isfinite(w)) or np.any(w <= 0.0):
            raise ValueError("energy_hs_v1 reconstructed nonpositive kinetic energy")
        v = np.sqrt(w)
        dv = w_s / (2.0 * v)
        control = (2*(x-.5)*(x-1)*self.acceleration[i]
                   + 4*x*(1-x)*self.midpoint_acceleration[i]
                   + 2*x*(x-.5)*self.acceleration[i+1])
        if self.control_representation == "quadratic_energy_rate_over_metric_v1":
            # Physical site variables a are unchanged.  The rate form makes
            # F=W_s quadratic by interpolating F=2*a*G at each site.
            f_mid = 2.0 * self.midpoint_acceleration[i] * self.metric_midpoints[i]
            f = (2*x*(x - 0.5) * f1
                 + 4*x*(1 - x) * f_mid
                 + 2*(x - 0.5)*(x - 1) * f0)
            a = f / (2.0 * g)
        else:
            a = control
            f = 2.0 * a * g
        rhs = a * g / v
        result = {"speed": v, "acceleration": a, "speed_rate": dv,
                  "rhs_speed_rate": rhs, "energy": w, "energy_rate": w_s,
                  "energy_rate_control": f}
        return {key: float(np.asarray(value)[0]) if scalar else value
                for key, value in result.items()}


def _merged_time_geometry_knots(mesh, geometry_knots=()) -> np.ndarray:
    """Merge near-coincident geometry knots with the trajectory-map policy."""
    mesh = np.asarray(mesh, dtype=float)
    if (mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh))
            or np.any(np.diff(mesh) <= 0.0)):
        raise ValueError("time-quadrature mesh must be finite and strictly increasing")
    candidates = np.asarray(geometry_knots, dtype=float).ravel()
    if candidates.size and (not np.all(np.isfinite(candidates))):
        raise ValueError("time-quadrature geometry knots must be finite")
    raw = np.sort(np.r_[mesh[0], candidates, mesh[-1]])
    tolerance = 1.0e-11 * max(1.0, float(mesh[-1] - mesh[0]))
    knots = raw[np.r_[True, np.diff(raw) > tolerance]]
    knots[0], knots[-1] = mesh[0], mesh[-1]
    return knots


def _time_quadrature_panels(mesh, geometry_knots=()) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Split every mesh element at analytic-geometry knots and return GL sites.

    The cubic energy state is smooth within a mesh element, but the source XY
    and offset splines need not be smooth across their respective knots.  Both
    the symbolic objective and replay therefore use this exact panelization.
    """
    mesh = np.asarray(mesh, dtype=float)
    candidates = _merged_time_geometry_knots(mesh, geometry_knots)
    starts: list[float] = []
    ends: list[float] = []
    tolerance = 1.0e-11 * max(1.0, float(mesh[-1] - mesh[0]))
    for left, right in zip(mesh[:-1], mesh[1:]):
        interior = candidates[(candidates > left + tolerance) & (candidates < right - tolerance)]
        boundaries = np.r_[left, np.unique(interior), right]
        starts.extend(boundaries[:-1])
        ends.extend(boundaries[1:])
    left = np.asarray(starts, dtype=float)
    right = np.asarray(ends, dtype=float)
    width = right - left
    if np.any(width <= 0.0):
        raise ValueError("time-quadrature panels must have positive width")
    stations = (left[:, None] + width[:, None] * _TIME_ELEMENT_FRACTIONS).ravel()
    return left, right, stations


def physical_time_quadrature(trajectory, evaluator, mesh) -> float:
    """Integrate represented physical time using the shared knot-aware GL16 rule.

    ``evaluator`` supplies the same cubic energy-Hermite state used by the
    optimizer; ``trajectory`` supplies the authoritative analytic path metric.
    This deliberately does not include the independent open-path regularizer.
    """
    geometry_knots = np.r_[trajectory.reference.source_station,
                           trajectory.spline.control_distance]
    left, right, stations = _time_quadrature_panels(mesh, geometry_knots)
    speed = np.asarray(evaluator.evaluate(stations)["speed"], dtype=float).reshape(
        left.size, _TIME_ELEMENT_FRACTIONS.size,
    )
    metric = np.asarray(trajectory.evaluate(stations).metric, dtype=float).reshape(speed.shape)
    if not (np.all(np.isfinite(speed)) and np.all(np.isfinite(metric))) or np.any(speed <= 0.0):
        raise ValueError("physical-time quadrature requires finite positive represented speed")
    return float(np.sum((right - left)[:, None] * _TIME_ELEMENT_WEIGHTS * metric / speed))


def _validate_spline(track, spline) -> int:
    """Validate that ``spline`` matches the track's distance domain."""
    control_count = getattr(spline, "control_count", None)
    if isinstance(control_count, (bool, np.bool_)) or not isinstance(
        control_count, (int, np.integer)
    ):
        raise ValueError("spline must expose an integer control_count")
    control_count = int(control_count)
    if control_count < 4:
        raise ValueError("spline must expose at least four control points")
    positions = np.asarray(spline.control_distance, dtype=float)
    if positions.shape != (control_count,):
        raise ValueError("spline control distance must match control_count")
    if not np.all(np.isfinite(positions)) or np.any(np.diff(positions) <= 0.0):
        raise ValueError("spline control points must be finite and strictly increasing")
    if bool(getattr(spline, "is_closed", False)) != bool(track.is_closed):
        raise ValueError("spline closure must match the track")
    length = float(track.lap_length)
    if not math.isclose(float(positions[0]), 0.0, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("spline control grid must start at zero")
    if track.is_closed:
        if float(positions[-1]) >= length + 1.0e-9:
            raise ValueError("closed spline must exclude the repeated end point")
    elif not math.isclose(float(positions[-1]), length, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("open spline control grid must end at the lap length")
    return control_count


def _speed_cap_values(speed_cap, node_count: int) -> FloatArray | None:
    """Validate an optional per-node physical speed cap."""
    if speed_cap is None:
        return None
    values = np.asarray(speed_cap, dtype=float).ravel()
    if values.shape != (node_count,):
        raise ValueError("speed_cap must hold one value per mesh node")
    if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("speed_cap must be finite and positive")
    return values


def _sample_corridor(values, corridor, positions, track):
    return track_sample(
        values, corridor.distance, positions,
        closed=bool(track.is_closed), period=float(track.lap_length),
    )



def _canonical_constraint_sites(mesh, fractions, constraint_stations):
    """Union legacy fractional sites with explicit stations, excluding nodes/mids."""
    mesh = np.asarray(mesh, dtype=float)
    fractions = np.asarray(fractions, dtype=float)
    widths = np.diff(mesh)
    legacy = (mesh[:-1, None] + widths[:, None] * fractions).ravel()
    legacy_elements = np.repeat(np.arange(widths.size), fractions.size)
    legacy_site_fractions = np.tile(fractions, widths.size)
    if constraint_stations is None:
        by_element = [np.flatnonzero(legacy_elements == i) for i in range(widths.size)]
        return legacy.copy(), legacy_elements, legacy_site_fractions.copy(), by_element
    explicit = np.asarray(constraint_stations, dtype=float)
    if explicit.ndim != 1:
        raise ValueError("constraint_stations must be a one-dimensional sequence")
    if explicit.size and (not np.all(np.isfinite(explicit))
                          or np.any(explicit <= mesh[0])
                          or np.any(explicit >= mesh[-1])):
        raise ValueError("constraint_stations must be finite interior mesh-domain stations")
    tolerance = 1.0e-11 * max(1.0, float(mesh[-1] - mesh[0]))
    midpoints = 0.5 * (mesh[:-1] + mesh[1:])
    retained = []
    for station in np.sort(explicit):
        element = int(np.searchsorted(mesh, station, side="right") - 1)
        local_tol = min(tolerance, 1.0e-8 * float(widths[element]))
        if np.min(np.abs(mesh - station)) <= local_tol:
            continue
        if np.min(np.abs(midpoints - station)) <= local_tol:
            continue
        if np.min(np.abs(legacy - station)) <= local_tol:
            continue
        if retained and station - retained[-1] <= local_tol:
            continue
        retained.append(float(station))
    stations = np.sort(np.r_[legacy, retained])
    elements = np.searchsorted(mesh, stations, side="right") - 1
    site_fractions = (stations - mesh[elements]) / widths[elements]
    # Sorting can retain nearby historical legacy samples; each belongs to its
    # original containing element and is never discarded by explicit additions.
    by_element = [np.flatnonzero(elements == i) for i in range(widths.size)]
    return stations, elements.astype(int), site_fractions, by_element


def build_spline_ocp(track, corridor, surface, mesh, spline, settings: QssSettings,
                     speed_cap=None, *, constraint_stations=None):
    """Build the spline CasADi NLP and its variable/constraint bounds.

    The return value has the same shape as :func:`qss_line_ocp.build_ocp`:
    ``nlp`` (``x``, ``f``, ``g``), physical variable bounds, physical
    constraint bounds, the mesh, the element count, the spline control count,
    and the spline itself.
    """
    import casadi as ca

    mesh = _validate_inputs(track, corridor, surface, mesh, settings)
    control_count = _validate_spline(track, spline)
    capability_margin = _validate_capability_margin(surface, settings)
    count = mesh.size - 1
    nodes = mesh.size
    midpoints = 0.5 * (mesh[:-1] + mesh[1:])
    widths = np.diff(mesh)
    fractions = np.asarray(settings.off_collocation_fractions, dtype=float)
    off_sites, off_elements, off_fractions, off_by_element = _canonical_constraint_sites(
        mesh, fractions, constraint_stations,
    )

    node_values, node_first, node_second = spline.basis(mesh)
    mid_values, mid_first, mid_second = spline.basis(midpoints)
    off_values, off_first, off_second = spline.basis(off_sites)

    reference = _reference_curve(track)
    time_left, time_right, time_sites = _time_quadrature_panels(
        mesh, np.r_[reference.source_station, spline.control_distance],
    )
    time_values, time_first, time_second = spline.basis(time_sites)
    time_panel_element = np.searchsorted(mesh, time_left, side="right") - 1
    time_panel_element = np.clip(time_panel_element, 0, count - 1)
    time_element = np.repeat(time_panel_element, _TIME_ELEMENT_FRACTIONS.size)
    time_coordinate = (time_sites - mesh[time_element]) / widths[time_element]
    node_reference = reference.evaluate(mesh)
    mid_reference = reference.evaluate(midpoints)
    time_reference = reference.evaluate(time_sites)
    k0_nodes, dk0_nodes = np.asarray(node_reference.curvature), np.asarray(node_reference.curvature_s)
    k0_mids, dk0_mids = np.asarray(mid_reference.curvature), np.asarray(mid_reference.curvature_s)
    k0_time, dk0_time = np.asarray(time_reference.curvature), np.asarray(time_reference.curvature_s)
    h_nodes, hs_nodes = np.asarray(node_reference.metric), np.asarray(node_reference.metric_s)
    h_mids, hs_mids = np.asarray(mid_reference.metric), np.asarray(mid_reference.metric_s)
    h_time, hs_time = np.asarray(time_reference.metric), np.asarray(time_reference.metric_s)
    corridor_lower = _sample_corridor(corridor.minimum, corridor, mesh, track)
    corridor_upper = _sample_corridor(corridor.maximum, corridor, mesh, track)
    mid_lower = _sample_corridor(corridor.minimum, corridor, midpoints, track)
    mid_upper = _sample_corridor(corridor.maximum, corridor, midpoints, track)
    off_lower = _sample_corridor(corridor.minimum, corridor, off_sites, track)
    off_upper = _sample_corridor(corridor.maximum, corridor, off_sites, track)
    zero_corridor = _is_zero_corridor(corridor.minimum, corridor.maximum)
    cap_values = _speed_cap_values(speed_cap, nodes)
    cap_mid = (
        None if cap_values is None
        else np.interp(midpoints, mesh, cap_values)
    )
    cap_off = (
        None if cap_values is None
        else np.interp(off_sites, mesh, cap_values)
    )

    coefficients = ca.MX.sym("c", control_count)
    speed = ca.MX.sym("v", nodes)
    acceleration = ca.MX.sym("a", nodes)
    mid_acceleration = ca.MX.sym("am", count)
    x = ca.vertcat(coefficients, speed, acceleration, mid_acceleration)

    def spline_field(matrix):
        return ca.mtimes(ca.DM(np.asarray(matrix, dtype=float)), coefficients)

    n_nodes = spline_field(node_values)
    n_first_nodes = spline_field(node_first)
    n_second_nodes = spline_field(node_second)
    n_mids = spline_field(mid_values)
    n_first_mids = spline_field(mid_first)
    n_second_mids = spline_field(mid_second)
    n_off = spline_field(off_values)
    n_first_off = spline_field(off_first)
    n_second_off = spline_field(off_second)
    n_time = spline_field(time_values)
    n_first_time = spline_field(time_first)
    n_second_time = spline_field(time_second)

    off_reference = reference.evaluate(off_sites)
    k0_off = np.asarray(off_reference.curvature)
    dk0_off = np.asarray(off_reference.curvature_s)
    h_off = np.asarray(off_reference.metric)
    hs_off = np.asarray(off_reference.metric_s)

    if zero_corridor:
        # The corridor admits only n = 0. Keeping the controls in the decision
        # layout but out of the expressions avoids dense trig terms in every
        # Jacobian evaluation while preserving the declared layout.
        node_geometry = [
            (1.0, 0.0, 0.0, float(k0_nodes[i]), float(h_nodes[i])) for i in range(nodes)
        ]
        mid_geometry = [
            (1.0, 0.0, 0.0, float(k0_mids[i]), float(h_mids[i])) for i in range(count)
        ]
        off_geometry = [
            (1.0, 0.0, 0.0, float(k0_off[i]), float(h_off[i])) for i in range(off_sites.size)
        ]
        time_geometry = [
            (1.0, 0.0, 0.0, float(k0_time[i]), float(h_time[i])) for i in range(time_sites.size)
        ]
    else:
        node_geometry = [
            _spline_geometry(
                n_nodes[i], n_first_nodes[i], n_second_nodes[i],
                float(h_nodes[i]), float(hs_nodes[i]), float(k0_nodes[i]), float(dk0_nodes[i]),
            )
            for i in range(nodes)
        ]
        mid_geometry = [
            _spline_geometry(
                n_mids[i], n_first_mids[i], n_second_mids[i],
                float(h_mids[i]), float(hs_mids[i]), float(k0_mids[i]), float(dk0_mids[i]),
            )
            for i in range(count)
        ]
        off_geometry = [
            _spline_geometry(
                n_off[i], n_first_off[i], n_second_off[i],
                float(h_off[i]), float(hs_off[i]), float(k0_off[i]), float(dk0_off[i]),
            )
            for i in range(off_sites.size)
        ]
        time_geometry = [
            _spline_geometry(
                n_time[i], n_first_time[i], n_second_time[i],
                float(h_time[i]), float(hs_time[i]), float(k0_time[i]), float(dk0_time[i]),
            )
            for i in range(time_sites.size)
        ]

    signed = getattr(surface, "representation", None) == "signed_transient_normalized_v1"
    if signed:
        from simplified_vehicle_models.formulations.signed_capability import symbolic_lookup
        signed_limits = symbolic_lookup(surface, "spline_signed")
        def lookup(speed_value, curvature_value):
            return signed_limits(speed_value, curvature_value)
    else:
        amax = ca.interpolant("spline_amax", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_max.ravel(order="F").tolist())
        amin = ca.interpolant("spline_amin", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_min.ravel(order="F").tolist())
        ceiling = ca.interpolant("spline_vceil", "linear", [surface.curvature.tolist()], surface.speed_ceiling.tolist())
        def lookup(speed_value, curvature_value):
            return (amin(ca.vertcat(speed_value, curvature_value)),
                    amax(ca.vertcat(speed_value, curvature_value)), ceiling(curvature_value))

    constraints: list = []
    lower: list = []
    upper: list = []
    node_acceleration_lower_indices: list[int] = []
    node_acceleration_upper_indices: list[int] = []
    midpoint_acceleration_lower_indices: list[int] = []
    midpoint_acceleration_upper_indices: list[int] = []
    bernstein_middle_indices: list[int] = []
    off_site_acceleration_lower_indices: list[int] = []
    off_site_acceleration_upper_indices: list[int] = []
    node_domain_speed_indices: list[int] = []
    midpoint_domain_speed_indices: list[int] = []
    off_site_domain_speed_indices: list[int] = []
    off_site_corridor_indices: list[int] = []
    off_site_speed_cap_indices: list[int] = []

    def add(expression, low, high):
        constraints.append(expression)
        lower.append(low)
        upper.append(high)
        return len(constraints) - 1

    curvature_lower = float(surface.curvature[0])
    curvature_upper = float(surface.curvature[-1])
    fold_guard = float(settings.fold_guard)
    capability_margin = _capability_interior_margin(settings)

    def limits(offset, psi, q, kappa, car_speed, car_acceleration,
               centreline_curvature):
        add(psi, -float(settings.maximum_heading), float(settings.maximum_heading))
        add(q, -float(settings.maximum_curvature_rate),
            float(settings.maximum_curvature_rate))
        add(kappa, curvature_lower, curvature_upper)
        lower_acceleration, upper_acceleration, domain_speed = lookup(car_speed, kappa)
        # Reserve a multiple of the requested NLP residual for the quadratic
        # control's interpolation against piecewise-linear chart faces.  This
        # changes neither the objective nor independent dense tolerances.
        acceleration_lower_index = add(car_acceleration - lower_acceleration,
                                       capability_margin, math.inf)
        acceleration_upper_index = add(upper_acceleration - car_acceleration,
                                       capability_margin, math.inf)
        # This optional setting explicitly constructs a conservative inner
        # profile, rather than claiming the unrestricted chart-domain optimum.
        domain_speed_index = add(
            domain_speed - car_speed, float(settings.speed_domain_margin_mps), math.inf,
        )
        add(1.0 - centreline_curvature * offset, fold_guard, math.inf)
        return acceleration_lower_index, acceleration_upper_index, domain_speed_index

    rate_control = settings.control_representation == "quadratic_energy_rate_over_metric_v1"

    def energy_rate(site_acceleration, metric):
        """Return F=W_s at a physical acceleration site."""
        return 2.0 * site_acceleration * metric

    def offsite_physical_acceleration(control, metric):
        """Recover a=F/(2G) only for the off-site quadratic rate control."""
        return control / (2.0 * metric) if rate_control else control

    def hermite_energy(index, coordinate):
        """Cubic kinetic energy ``W=v²`` at an interior element coordinate."""
        h = float(widths[index])
        x = float(coordinate)
        metric0, metric1 = node_geometry[index][4], node_geometry[index + 1][4]
        f0 = energy_rate(acceleration[index], metric0)
        f1 = energy_rate(acceleration[index + 1], metric1)
        return ((2*x**3 - 3*x**2 + 1) * speed[index] ** 2
                + (x**3 - 2*x**2 + x) * h * f0
                + (-2*x**3 + 3*x**2) * speed[index + 1] ** 2
                + (x**3 - x**2) * h * f1)

    def energy_speed(energy):
        """Safe optimizer expression; a separate W floor rejects invalid W."""
        return ca.sqrt(ca.fmax(energy, np.finfo(float).tiny))

    def quadratic_control(index, coordinate):
        """Node/midpoint/node Lagrange polynomial for a (legacy) or F."""
        x = float(coordinate)
        if rate_control:
            return (2*x*(x - 0.5) * energy_rate(acceleration[index + 1], node_geometry[index + 1][4])
                    + 4*x*(1 - x) * energy_rate(mid_acceleration[index], mid_geometry[index][4])
                    + 2*(x - 0.5)*(x - 1) * energy_rate(acceleration[index], node_geometry[index][4]))
        return (2*x*(x - 0.5) * acceleration[index + 1]
                + 4*x*(1 - x) * mid_acceleration[index]
                + 2*(x - 0.5)*(x - 1) * acceleration[index])

    # Binding physical-time objective: exactly the same knot-union GL16 rule
    # as ``physical_time_quadrature`` below.  The open-path regularizer remains
    # a separate explicit penalty and is intentionally not physical time.
    objective = 0
    for site in range(time_sites.size):
        panel = site // _TIME_ELEMENT_FRACTIONS.size
        energy = hermite_energy(int(time_element[site]), float(time_coordinate[site]))
        objective += ((time_right[panel] - time_left[panel])
                      * _TIME_ELEMENT_WEIGHTS[site % _TIME_ELEMENT_FRACTIONS.size]
                      * time_geometry[site][4] / energy_speed(energy))
    for index in range(count):
        h = float(widths[index])
        node0 = node_geometry[index]
        node1 = node_geometry[index + 1]
        middle = mid_geometry[index]
        metric0, metric1, metric_mid = node0[4], node1[4], middle[4]
        f0 = energy_rate(acceleration[index], metric0)
        f1 = energy_rate(acceleration[index + 1], metric1)
        midpoint_energy = (0.5 * (speed[index] ** 2 + speed[index + 1] ** 2)
                           + h / 8.0 * (f0 - f1))
        midpoint_speed = energy_speed(midpoint_energy)
        f_mid = energy_rate(mid_acceleration[index], metric_mid)
        # Simpson integrates the quadratic F=W_s control exactly; W is cubic.
        # Normalize this m²/s² row by a fixed 10 m/s reference speed for NLP
        # conditioning. This does not redefine the independently reported
        # physical energy residual or provide a speed-uniform error bound.
        add((speed[index + 1] ** 2 - speed[index] ** 2
             - h / 6.0 * (f0 + 4.0 * f_mid + f1)) / 10.0, 0.0, 0.0)
        if not rate_control:
            # Only the legacy quadratic acceleration control has an acceleration
            # Bernstein guard. In rate mode F/(2G) is not quadratic, so this
            # would be a false continuous-acceleration certificate; dense
            # independent validation remains the physical evidence.
            bernstein_middle = 2.0 * mid_acceleration[index] - 0.5 * (
                acceleration[index] + acceleration[index + 1]
            )
            bernstein_middle_indices.append(add(
                bernstein_middle, float(np.min(surface.acceleration_min)),
                float(np.max(surface.acceleration_max)),
            ))
        # Bernstein coefficients of the cubic W enforce a sufficient global
        # physical speed floor over this element, not merely at the midpoint.
        minimum_energy = float(settings.minimum_speed) ** 2
        for energy_coefficient in (
            speed[index] ** 2,
            speed[index] ** 2 + h * f0 / 3.0,
            speed[index + 1] ** 2 - h * f1 / 3.0,
            speed[index + 1] ** 2,
        ):
            add(energy_coefficient - minimum_energy, 0.0, math.inf)
        if (not track.is_closed and not zero_corridor
                and settings.open_regularisation > 0.0):
            # On a straight open section the lateral offset has no intrinsic
            # cost. A tiny centreline preference removes that null direction
            # without materially changing the minimum time.
            objective += settings.open_regularisation * h / 6.0 * (
                n_nodes[index] ** 2
                + 4.0 * n_mids[index] ** 2
                + n_nodes[index + 1] ** 2
            )
        if not zero_corridor:
            add(n_nodes[index], float(corridor_lower[index]),
                float(corridor_upper[index]))
            add(n_mids[index], float(mid_lower[index]), float(mid_upper[index]))
            for site in off_by_element[index]:
                off_site_corridor_indices.append(
                    add(n_off[site], float(off_lower[site]), float(off_upper[site]))
                )
            add(n_nodes[index + 1], float(corridor_lower[index + 1]),
                float(corridor_upper[index + 1]))
        acceleration_indices = limits(
            n_nodes[index], node0[1], node0[2], node0[3], speed[index],
            acceleration[index], float(k0_nodes[index]),
        )
        node_acceleration_lower_indices.append(acceleration_indices[0])
        node_acceleration_upper_indices.append(acceleration_indices[1])
        node_domain_speed_indices.append(acceleration_indices[2])
        acceleration_indices = limits(
            n_mids[index], middle[1], middle[2], middle[3], midpoint_speed,
            mid_acceleration[index], float(k0_mids[index]),
        )
        midpoint_acceleration_lower_indices.append(acceleration_indices[0])
        midpoint_acceleration_upper_indices.append(acceleration_indices[1])
        midpoint_domain_speed_indices.append(acceleration_indices[2])
        acceleration_indices = limits(
            n_nodes[index + 1], node1[1], node1[2], node1[3],
            speed[index + 1], acceleration[index + 1], float(k0_nodes[index + 1]),
        )
        node_acceleration_lower_indices.append(acceleration_indices[0])
        node_acceleration_upper_indices.append(acceleration_indices[1])
        node_domain_speed_indices.append(acceleration_indices[2])
        for site in off_by_element[index]:
            fraction = float(off_fractions[site])
            off_speed = energy_speed(hermite_energy(index, fraction))
            off_control = quadratic_control(index, fraction)
            off_acceleration = offsite_physical_acceleration(off_control, off_geometry[site][4])
            # These are feasibility rows only: retain the single HS speed
            # defect above rather than enforcing additional polynomial dynamics.
            add(off_speed - float(settings.minimum_speed), 0.0, math.inf)
            acceleration_indices = limits(
                n_off[site], off_geometry[site][1], off_geometry[site][2],
                off_geometry[site][3], off_speed, off_acceleration,
                float(k0_off[site]),
            )
            off_site_acceleration_lower_indices.append(acceleration_indices[0])
            off_site_acceleration_upper_indices.append(acceleration_indices[1])
            off_site_domain_speed_indices.append(acceleration_indices[2])
            if cap_off is not None:
                off_site_speed_cap_indices.append(
                    add(off_speed - float(cap_off[site]), -math.inf, 0.0)
                )
        if cap_values is not None:
            add(speed[index] - float(cap_values[index]), -math.inf, 0.0)
            add(speed[index + 1] - float(cap_values[index + 1]), -math.inf, 0.0)
            add(midpoint_speed - float(cap_mid[index]), -math.inf, 0.0)

    if track.is_closed:
        add(speed[0] - speed[-1], 0.0, 0.0)
    else:
        start_speed = max(float(settings.start_speed), float(settings.minimum_speed))
        add(speed[0] - start_speed, 0.0, 0.0)

    if zero_corridor:
        # A zero corridor fixes the only admissible spline exactly. Bounding the
        # controls at zero is equivalent to the dense corridor rows and removes
        # the otherwise flat lateral direction from the NLP.
        control_lower = np.zeros(control_count)
        control_upper = np.zeros(control_count)
    else:
        limit = max(
            float(np.max(np.abs(corridor_lower))),
            float(np.max(np.abs(corridor_upper))),
        ) + 1.0
        control_lower = np.full(control_count, -limit)
        control_upper = np.full(control_count, limit)

    acceleration_lower = float(np.min(surface.acceleration_min))
    acceleration_upper = float(np.max(surface.acceleration_max))
    maximum_domain_speed = (float(np.max(surface.domain_speed_values)) if signed
                            else float(surface.speed[-1]))
    x_lower = np.concatenate([
        control_lower,
        np.full(nodes, float(settings.minimum_speed)),
        np.full(nodes, acceleration_lower),
        np.full(count, acceleration_lower),
    ])
    x_upper = np.concatenate([
        control_upper,
        np.full(nodes, maximum_domain_speed),
        np.full(nodes, acceleration_upper),
        np.full(count, acceleration_upper),
    ])
    return {
        "nlp": {"x": x, "f": objective, "g": ca.vertcat(*constraints)},
        "x_lower": x_lower,
        "x_upper": x_upper,
        "g_lower": np.asarray(lower, dtype=float),
        "g_upper": np.asarray(upper, dtype=float),
        "mesh": mesh,
        "off_site_stations": np.array(off_sites, dtype=float, copy=True),
        "off_site_elements": np.array(off_elements, dtype=int, copy=True),
        "off_site_fractions": np.array(off_fractions, dtype=float, copy=True),
        "off_site_element_indices": tuple(np.array(indices, dtype=int, copy=True)
                                          for indices in off_by_element),
        "count": count,
        "control_count": control_count,
        "spline": spline,
        "reference": reference,
        "time_quadrature_panel_count": int(time_left.size),
        "time_quadrature_site_count": int(time_sites.size),
        # Diagnostic-only mapping in element-major/fraction-major order. It
        # preserves the NLP layout while allowing regressions to identify a
        # particular physical off-site capability row.
        "node_acceleration_lower_indices": np.asarray(
            node_acceleration_lower_indices, dtype=int,
        ),
        "node_acceleration_upper_indices": np.asarray(
            node_acceleration_upper_indices, dtype=int,
        ),
        "midpoint_acceleration_lower_indices": np.asarray(
            midpoint_acceleration_lower_indices, dtype=int,
        ),
        "midpoint_acceleration_upper_indices": np.asarray(
            midpoint_acceleration_upper_indices, dtype=int,
        ),
        "bernstein_middle_indices": np.asarray(bernstein_middle_indices, dtype=int),
        "off_site_acceleration_lower_indices": np.asarray(
            off_site_acceleration_lower_indices, dtype=int,
        ),
        "off_site_acceleration_upper_indices": np.asarray(
            off_site_acceleration_upper_indices, dtype=int,
        ),
        "node_domain_speed_indices": np.asarray(node_domain_speed_indices, dtype=int),
        "midpoint_domain_speed_indices": np.asarray(
            midpoint_domain_speed_indices, dtype=int,
        ),
        "off_site_domain_speed_indices": np.asarray(
            off_site_domain_speed_indices, dtype=int,
        ),
        "off_site_corridor_indices": np.asarray(off_site_corridor_indices, dtype=int),
        "off_site_speed_cap_indices": np.asarray(off_site_speed_cap_indices, dtype=int),
    }


def unpack(values, count: int, control_count: int) -> dict[str, FloatArray]:
    """Split a spline decision vector into its physical blocks."""
    nodes = count + 1
    controls = int(control_count)
    expected = controls + 3 * nodes - 1
    flat = np.asarray(values, dtype=float).ravel()
    if flat.size != expected:
        raise ValueError(
            f"decision vector has {flat.size} entries, layout needs {expected}"
        )
    return {
        "coefficients": flat[:controls],
        "speed": flat[controls:controls + nodes],
        "acceleration": flat[controls + nodes:controls + 2 * nodes],
        "mid_acceleration": flat[controls + 2 * nodes:],
    }


def _decision_scaling(control_count: int, node_count: int) -> tuple[FloatArray, FloatArray]:
    """Return affine scales for the grouped spline decision vector.

    Coefficients are centred at zero, speed is centred on 20 m/s with a 10 m/s
    scale, and both acceleration blocks use a 5 m/s² scale. Scaling changes
    conditioning only; the NLP and the returned solution stay physical.
    """
    if isinstance(control_count, (bool, np.bool_)) or not isinstance(
        control_count, (int, np.integer)
    ):
        raise ValueError("decision scaling control count must be an integer")
    if isinstance(node_count, (bool, np.bool_)) or not isinstance(
        node_count, (int, np.integer)
    ):
        raise ValueError("decision scaling node count must be an integer")
    controls = int(control_count)
    nodes = int(node_count)
    if controls < 2:
        raise ValueError("decision scaling requires at least two spline controls")
    if nodes < 2:
        raise ValueError("decision scaling requires at least two mesh nodes")
    size = controls + 3 * nodes - 1
    center = np.zeros(size, dtype=float)
    scale = np.ones(size, dtype=float)
    speed_slice = slice(controls, controls + nodes)
    center[speed_slice] = 20.0
    scale[speed_slice] = 10.0
    scale[controls + nodes:controls + 2 * nodes] = 5.0
    scale[controls + 2 * nodes:] = 5.0
    return center, scale


def _envelope_guess(track, surface, mesh, settings: QssSettings, speed_cap=None):
    """Return a forward/backward capability envelope and finite differences.

    The envelope uses the centreline curvature because the initial spline is
    the zero offset that :func:`initial_guess` selects. It is only a seed; the
    OCP decides the actual path and speed profile.
    """
    nodes = mesh.size
    count = nodes - 1
    curvature = np.clip(
        track_sample(
            track.curvature, track.distance, mesh,
            closed=bool(track.is_closed), period=float(track.lap_length),
        ),
        surface.curvature[0], surface.curvature[-1],
    )
    signed = getattr(surface, "representation", None) == "signed_transient_normalized_v1"
    ceiling = (np.asarray([surface.domain_speed(float(k)) for k in curvature]) if signed
               else np.interp(curvature, surface.curvature, surface.speed_ceiling))
    # Match the chart-domain rows while leaving an optional user speed cap
    # independent and unchanged.
    ceiling = ceiling - float(settings.speed_domain_margin_mps)
    cap_values = _speed_cap_values(speed_cap, nodes)
    minimum = float(settings.minimum_speed)
    top = float(np.max(surface.domain_speed_values)) if signed else float(surface.speed[-1])
    widths = np.diff(mesh)

    def amax(car_speed, car_curvature):
        if signed:
            # A forward/backward sweep reuses a speed at a neighbouring
            # curvature; clamp its *seed query* to that local domain only.
            car_speed = min(
                float(car_speed),
                surface.domain_speed(float(car_curvature))
                - float(settings.speed_domain_margin_mps),
            )
            return surface.limits(car_speed, float(car_curvature))[1]
        return _bilinear(
            surface.acceleration_max, surface.speed, surface.curvature,
            car_speed, car_curvature,
        )

    def amin(car_speed, car_curvature):
        if signed:
            car_speed = min(
                float(car_speed),
                surface.domain_speed(float(car_curvature))
                - float(settings.speed_domain_margin_mps),
            )
            return surface.limits(car_speed, float(car_curvature))[0]
        return _bilinear(
            surface.acceleration_min, surface.speed, surface.curvature,
            car_speed, car_curvature,
        )

    speed = np.minimum(ceiling, top)
    if cap_values is not None:
        speed = np.minimum(speed, cap_values)
    speed = np.maximum(speed, minimum)

    if track.is_closed:
        # Enforce the periodic boundary on the ring of distinct nodes, then
        # duplicate the first speed at the lap length.
        ring = np.array(speed[:count], dtype=float)
        ring_curvature = curvature[:count]
        for _ in range(2):
            for index in range(count - 1, -1, -1):
                following = (index + 1) % count
                bound = math.sqrt(max(
                    ring[following] ** 2
                    - 2.0 * amin(ring[following], ring_curvature[following])
                    * float(widths[index]), 0.0,
                ))
                ring[index] = min(ring[index], max(bound, minimum))
            for index in range(count):
                following = (index + 1) % count
                bound = math.sqrt(max(
                    ring[index] ** 2
                    + 2.0 * amax(ring[index], ring_curvature[index])
                    * float(widths[index]), 0.0,
                ))
                ring[following] = min(ring[following], max(bound, minimum))
        speed = np.r_[ring, ring[0]]
    else:
        speed[0] = max(float(settings.start_speed), minimum)
        for index in range(count):
            bound = math.sqrt(max(
                speed[index] ** 2
                + 2.0 * amax(speed[index], curvature[index]) * float(widths[index]),
                0.0,
            ))
            speed[index + 1] = min(speed[index + 1], max(bound, minimum))
        for index in range(count - 1, -1, -1):
            bound = math.sqrt(max(
                speed[index + 1] ** 2
                - 2.0 * amin(speed[index + 1], curvature[index + 1])
                * float(widths[index]), 0.0,
            ))
            speed[index] = min(speed[index], max(bound, minimum))
        speed[0] = max(float(settings.start_speed), minimum)
        for index in range(count):
            bound = math.sqrt(max(
                speed[index] ** 2
                + 2.0 * amax(speed[index], curvature[index]) * float(widths[index]),
                0.0,
            ))
            speed[index + 1] = min(speed[index + 1], max(bound, minimum))

    if cap_values is not None:
        speed = np.minimum(speed, cap_values)
    speed = np.clip(speed, minimum, top)
    acceleration = speed * np.gradient(speed, mesh)
    acceleration = np.clip(
        acceleration,
        float(np.min(surface.acceleration_min)),
        float(np.max(surface.acceleration_max)),
    )
    return speed, acceleration


def initial_guess(track, corridor, surface, mesh, spline, settings: QssSettings,
                  speed_cap=None) -> FloatArray:
    """Return a finite seed in the spline decision layout."""
    mesh = _validate_inputs(track, corridor, surface, mesh, settings)
    control_count = _validate_spline(track, spline)
    basis = spline.basis(mesh)[0]
    coefficients = np.linalg.lstsq(
        basis, np.zeros(mesh.size, dtype=float), rcond=None,
    )[0]
    speed, acceleration = _envelope_guess(
        track, surface, mesh, settings, speed_cap=speed_cap,
    )
    mid_acceleration = 0.5 * (acceleration[:-1] + acceleration[1:])
    return np.concatenate([
        coefficients, speed, acceleration, mid_acceleration,
    ])


def _spline_failure_diagnostics(*, status, iter_count, total_iterations, objective,
                                constraint_violation, bound_violation, stats, built,
                                settings, stage_timings=None,
                                active_decision_indices=None) -> dict[str, object]:
    """Serialize only solver-state facts needed to diagnose strict rejection."""
    diagnostics: dict[str, object] = {
        "return_status": str(status),
        "iter_count": int(iter_count),
        "total_iterations": int(total_iterations),
        "objective_value": float(objective),
        "physical_max_constraint_violation": float(constraint_violation),
        "physical_max_bound_violation": float(bound_violation),
        "layout": {"control_count": int(built["control_count"]),
                   "node_count": int(built["mesh"].size),
                   "decision_size": int(built["nlp"]["x"].numel())},
        "control_representation": settings.control_representation,
        "state_representation": "energy_hs_v1",
        # IPOPT receives affine-scaled x but the constraint residual g is not
        # rescaled.  These labels prevent a failed iterate from implying a
        # common physical coordinate system for both multiplier vectors.
        "dual_x_coordinates": "scaled_decision",
        "dual_g_coordinates": "unscaled_constraint_residuals",
        "time_quadrature": TIME_QUADRATURE_NAME,
        "off_site_stations": np.array(built.get("off_site_stations", []), copy=True).tolist(),
        "off_site_elements": np.array(built.get("off_site_elements", []), copy=True).tolist(),
        "off_site_fractions": np.array(built.get("off_site_fractions", []), copy=True).tolist(),
    }
    if active_decision_indices is not None:
        diagnostics["dual_x_coordinates"] = "scaled_active_decision"
        diagnostics["dual_x_full_decision_indices"] = np.asarray(
            active_decision_indices, dtype=int,
        ).tolist()
        diagnostics["layout"]["active_decision_size"] = len(active_decision_indices)
        diagnostics["layout"]["full_decision_size"] = int(built["nlp"]["x"].numel())
        diagnostics["fixed_coefficients_mode"] = True
    if stage_timings is not None:
        diagnostics["stage_timings"] = dict(stage_timings)
    iterations = stats.get("iterations") if isinstance(stats, dict) else None
    if isinstance(iterations, dict) and "inf_du" in iterations:
        values = np.asarray(iterations["inf_du"], dtype=float).ravel()
        if values.size:
            diagnostics["scaled_dual_infeasibility"] = float(values[-1])
    return diagnostics


def _raise_spline_solver_failure(*, status, values, constraints, objective,
                                 constraint_violation, bound_violation, iter_count,
                                 total_iterations, stats, result, built, settings,
                                 stage_timings=None, active_decision_indices=None):
    """Run the unchanged strict gate and decorate a rejection with raw evidence."""
    try:
        _validate_solver_result(
            status, values, constraints, objective, constraint_violation,
            settings.tolerance, built["x_lower"], built["x_upper"],
        )
    except RuntimeError as error:
        diagnostics = _spline_failure_diagnostics(
            status=status, iter_count=iter_count, total_iterations=total_iterations,
            objective=objective, constraint_violation=constraint_violation,
            bound_violation=bound_violation, stats=stats, built=built, settings=settings,
            stage_timings=stage_timings,
            active_decision_indices=active_decision_indices,
        )
        raise SplineSolveError(
            str(error), diagnostics, failed_values=values,
            constraint_values=constraints, dual_x=result.get("lam_x"),
            dual_g=result.get("lam_g"),
        ) from error


def solve_spline_line(track, corridor, surface, mesh, spline,
                      settings: QssSettings, *, seed: FloatArray | None = None,
                      speed_cap=None, constraint_stations=None,
                       fixed_coefficients=None) -> SplineSolution:
    """Solve the consistent spline minimum-time problem on a fixed mesh.

    The status and tolerance contract matches :func:`solve_line`: a solver
    status other than ``Solve_Succeeded``, a non-finite value, or a constraint
    violation above ``settings.tolerance`` raises ``RuntimeError``.
    """
    total_started = time.perf_counter()
    timings = {name: 0.0 for name in (
        "build_seconds", "initialization_seconds", "symbolic_scaling_seconds",
        "solver_construction_seconds", "solver_execution_seconds",
        "postprocess_seconds", "total_seconds",
    )}
    build_started = time.perf_counter()
    import casadi as ca

    built = build_spline_ocp(
        track, corridor, surface, mesh, spline, settings, speed_cap=speed_cap,
        constraint_stations=constraint_stations,
    )
    timings["build_seconds"] = time.perf_counter() - build_started
    initialization_started = time.perf_counter()
    count = built["count"]
    control_count = built["control_count"]
    size = int(built["nlp"]["x"].numel())
    fixed = None
    if fixed_coefficients is not None:
        raw_fixed = np.asarray(fixed_coefficients)
        if (raw_fixed.ndim != 1 or raw_fixed.dtype.kind not in "iuf"
                or raw_fixed.dtype.kind == "b"):
            raise ValueError("fixed_coefficients must be a one-dimensional real numeric vector")
        fixed = np.array(raw_fixed, dtype=float, copy=True)
        if fixed.shape != (control_count,):
            raise ValueError("fixed_coefficients must match spline.control_count")
        if not np.all(np.isfinite(fixed)):
            raise ValueError("fixed_coefficients must be finite")
        if np.any(fixed < built["x_lower"][:control_count]) or np.any(
                fixed > built["x_upper"][:control_count]):
            raise ValueError("fixed_coefficients must be within coefficient bounds")
    vector = (
        initial_guess(track, corridor, surface, mesh, spline, settings,
                      speed_cap=speed_cap)
        if seed is None else np.asarray(seed, dtype=float).ravel()
    )
    if vector.size != size:
        raise ValueError("seed does not match the spline decision layout")
    if not np.all(np.isfinite(vector)):
        raise ValueError("seed must contain only finite values")
    vector = np.clip(vector, built["x_lower"], built["x_upper"])
    if fixed is not None:
        vector[:control_count] = fixed
    timings["initialization_seconds"] = time.perf_counter() - initialization_started

    scaling_started = time.perf_counter()
    center, scale = _decision_scaling(control_count, built["mesh"].size)
    original_x = built["nlp"]["x"]
    if fixed is None:
        active_indices = np.arange(size, dtype=int)
        scaled_x = ca.MX.sym("spline_scaled_x", size)
        physical_x = ca.DM(center) + ca.DM(scale) * scaled_x
        lower = (built["x_lower"] - center) / scale
        upper = (built["x_upper"] - center) / scale
        scaled_seed = (vector - center) / scale
    else:
        active_indices = np.arange(control_count, size, dtype=int)
        scaled_x = ca.MX.sym("spline_scaled_active_x", size - control_count)
        physical_x = ca.vertcat(ca.DM(fixed), ca.DM(center[control_count:])
                                + ca.DM(scale[control_count:]) * scaled_x)
        lower = (built["x_lower"][control_count:] - center[control_count:]) / scale[control_count:]
        upper = (built["x_upper"][control_count:] - center[control_count:]) / scale[control_count:]
        scaled_seed = (vector[control_count:] - center[control_count:]) / scale[control_count:]
    scaled_nlp = {
        "x": scaled_x,
        "f": ca.substitute(built["nlp"]["f"], original_x, physical_x),
        "g": ca.substitute(built["nlp"]["g"], original_x, physical_x),
    }

    tolerances = _tolerance_schedule(settings)
    timings["symbolic_scaling_seconds"] = time.perf_counter() - scaling_started
    lam_x = lam_g = None
    total_iterations = 0
    iter_count = 0
    result = None
    stats: dict[str, object] = {}
    values = np.array(vector, dtype=float, copy=True)
    constraints = np.full_like(built["g_lower"], np.nan, dtype=float)
    objective = float("nan")
    violation = float("nan")
    status = "Maximum_Iterations_Exceeded"
    for stage, tolerance in enumerate(tolerances):
        remaining_iterations = settings.maximum_iterations - total_iterations
        if remaining_iterations <= 0:
            status = "Maximum_Iterations_Exceeded"
            break
        options = {
            "ipopt.tol": tolerance,
            "ipopt.constr_viol_tol": tolerance,
            # The strict public contract accepts only Solve_Succeeded.  Disable
            # IPOPT's looser acceptable-level exit rather than treating status
            # as numerical feasibility; `_validate_solver_result` remains the
            # final residual and bound gate.
            "ipopt.acceptable_iter": 0,
            "ipopt.max_iter": remaining_iterations,
            "ipopt.hessian_approximation": settings.hessian_approximation,
            "ipopt.nlp_scaling_method": "gradient-based",
            "ipopt.mu_strategy": "adaptive",
            "ipopt.print_level": 0,
            "print_time": False,
        }
        if lam_x is not None:
            options.update({
                "ipopt.warm_start_init_point": "yes",
                "ipopt.warm_start_bound_push": 1.0e-8,
                "ipopt.warm_start_mult_bound_push": 1.0e-8,
                "ipopt.warm_start_slack_bound_push": 1.0e-8,
            })
        construction_started = time.perf_counter()
        solver = ca.nlpsol("qss_spline_line", "ipopt", scaled_nlp, options)
        timings["solver_construction_seconds"] += time.perf_counter() - construction_started
        arguments = {
            "x0": scaled_seed,
            "lbx": lower,
            "ubx": upper,
            "lbg": built["g_lower"],
            "ubg": built["g_upper"],
        }
        if lam_x is not None:
            arguments.update({"lam_x0": lam_x, "lam_g0": lam_g})
        execution_started = time.perf_counter()
        result = solver(**arguments)
        timings["solver_execution_seconds"] += time.perf_counter() - execution_started
        stats = solver.stats()
        status = str(stats["return_status"])
        iter_count = int(stats["iter_count"])
        total_iterations += iter_count
        active_values = np.asarray(result["x"], dtype=float).ravel()
        if fixed is None:
            values = center + scale * active_values
        else:
            values = np.concatenate([fixed, center[control_count:]
                                     + scale[control_count:] * active_values])
        constraints = np.asarray(result["g"], dtype=float).ravel()
        violation = float(np.max(np.maximum(
            np.maximum(built["g_lower"] - constraints, 0.0),
            constraints - built["g_upper"],
        )))
        objective = float(result["f"])
        if status != "Solve_Succeeded":
            break
        if stage + 1 < len(tolerances):
            scaled_seed = np.asarray(result["x"], dtype=float).ravel()
            lam_x = np.asarray(result["lam_x"], dtype=float).ravel()
            lam_g = np.asarray(result["lam_g"], dtype=float).ravel()

    bound_violation = float(np.max(np.maximum(
        np.maximum(built["x_lower"] - values, 0.0), values - built["x_upper"],
    )))
    timings["total_seconds"] = time.perf_counter() - total_started
    _raise_spline_solver_failure(
        status=status, values=values, constraints=constraints, objective=objective,
        constraint_violation=violation, bound_violation=bound_violation,
        iter_count=iter_count, total_iterations=total_iterations, stats=stats,
        result=result or {}, built=built, settings=settings,
        stage_timings=dict(timings),
        active_decision_indices=active_indices if fixed is not None else None,
    )
    violation = max(violation, bound_violation)
    postprocess_started = time.perf_counter()
    blocks = unpack(values, count, control_count)
    trajectory = OffsetTrajectory(built["reference"], spline, blocks["coefficients"])
    evaluated = trajectory.evaluate(built["mesh"])
    # ``result['f']`` includes optional open regularisation; report physical time separately.
    evaluator = _SplineStateEvaluator(
        trajectory, built["mesh"], blocks["speed"], blocks["acceleration"],
        blocks["mid_acceleration"],
        control_representation=settings.control_representation,
    )
    physical_time = physical_time_quadrature(trajectory, evaluator, built["mesh"])
    return SplineSolution(
        distance=np.asarray(built["mesh"], dtype=float),
        offset=np.asarray(evaluated.offset),
        heading_error=np.asarray(evaluated.heading_error),
        curvature=np.asarray(evaluated.curvature),
        speed=blocks["speed"],
        acceleration=blocks["acceleration"],
        lap_time=physical_time,
        status=str(stats["return_status"]),
        iterations=total_iterations,
        maximum_violation=violation,
        coefficients=blocks["coefficients"],
        fixed_coefficients=fixed,
        line_optimized=fixed is None,
        trajectory=trajectory,
        midpoint_acceleration=np.array(blocks["mid_acceleration"], copy=True),
        objective_value=objective,
        state_evaluator=evaluator,
        time_quadrature_metadata=_time_quadrature_metadata() | {
            "panel_count": int(built["time_quadrature_panel_count"]),
            "site_count": int(built["time_quadrature_site_count"]),
            "control_representation": settings.control_representation,
        },
        control_representation=settings.control_representation,
        off_site_stations=np.array(built["off_site_stations"], dtype=float, copy=True),
        off_site_elements=np.array(built["off_site_elements"], dtype=int, copy=True),
        off_site_fractions=np.array(built["off_site_fractions"], dtype=float, copy=True),
        stage_timings={**timings,
                       "postprocess_seconds": time.perf_counter() - postprocess_started,
                       "total_seconds": time.perf_counter() - total_started},
    )
