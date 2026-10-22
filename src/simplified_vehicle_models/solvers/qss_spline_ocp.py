"""Consistent cubic-spline minimum-time line OCP in the distance domain.

The lateral offset is one cubic spline of the track distance, so the offset,
heading error, curvature rate, and path curvature all come from the same
representation at the nodes, the collocation midpoints, and the interior
containment samples.  Speed is the only differential state; acceleration is a
control at the mesh nodes and at the midpoint of every element.

The decision vector is grouped as
``[spline coefficients | speed nodes | acceleration nodes | midpoint
acceleration]``.  A closed track uses a periodic spline and a periodic speed
boundary; an open track keeps the natural spline and fixes only the launch
speed, leaving the terminal speed and the endpoint offsets free.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

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
    return ReferenceCurve(SimpleNamespace(path_coordinates=normalized,
                                          is_closed=bool(track.is_closed)))


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


class _SplineStateEvaluator:
    """Immutable Hermite--Simpson dense state/control evaluator."""
    def __init__(self, trajectory, mesh, speed, acceleration, midpoint_acceleration):
        self.trajectory = trajectory
        self.mesh = np.array(mesh, dtype=float, copy=True); self.mesh.setflags(write=False)
        self.speed = np.array(speed, dtype=float, copy=True); self.speed.setflags(write=False)
        self.acceleration = np.array(acceleration, dtype=float, copy=True); self.acceleration.setflags(write=False)
        self.midpoint_acceleration = np.array(midpoint_acceleration, dtype=float, copy=True); self.midpoint_acceleration.setflags(write=False)
    def evaluate(self, station):
        s=np.asarray(station,dtype=float); scalar=s.ndim==0; flat=np.atleast_1d(s)
        if np.any(flat < self.mesh[0]) or np.any(flat > self.mesh[-1]): raise ValueError("state station outside mesh")
        i=np.minimum(np.searchsorted(self.mesh,flat,side='right')-1,self.mesh.size-2)
        x=(flat-self.mesh[i])/(self.mesh[i+1]-self.mesh[i]); width=self.mesh[i+1]-self.mesh[i]
        g=self.trajectory.evaluate(flat).metric
        f0=self.acceleration[i]*np.asarray(self.trajectory.evaluate(self.mesh[i]).metric)/(self.speed[i])
        f1=self.acceleration[i+1]*np.asarray(self.trajectory.evaluate(self.mesh[i+1]).metric)/(self.speed[i+1])
        v0=self.speed[i]; v1=self.speed[i+1]
        v=(2*x**3-3*x**2+1)*v0+(x**3-2*x**2+x)*width*f0+(-2*x**3+3*x**2)*v1+(x**3-x**2)*width*f1
        dv=((6*x**2-6*x)*v0 + (3*x**2-4*x+1)*width*f0
            + (-6*x**2+6*x)*v1 + (3*x**2-2*x)*width*f1) / width
        # quadratic Lagrange acceleration through node, midpoint, node.
        a=(2*(x-.5)*(x-1)*self.acceleration[i] + 4*x*(1-x)*self.midpoint_acceleration[i] + 2*x*(x-.5)*self.acceleration[i+1])
        rhs = a*g/v
        result = {"speed": v, "acceleration": a, "speed_rate": dv,
                  "rhs_speed_rate": rhs}
        return {key: float(np.asarray(value)[0]) if scalar else value
                for key, value in result.items()}


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



def build_spline_ocp(track, corridor, surface, mesh, spline, settings: QssSettings,
                     speed_cap=None):
    """Build the spline CasADi NLP and its variable/constraint bounds.

    The return value has the same shape as :func:`qss_line_ocp.build_ocp`:
    ``nlp`` (``x``, ``f``, ``g``), physical variable bounds, physical
    constraint bounds, the mesh, the element count, the spline control count,
    and the spline itself.
    """
    import casadi as ca

    mesh = _validate_inputs(track, corridor, surface, mesh, settings)
    control_count = _validate_spline(track, spline)
    count = mesh.size - 1
    nodes = mesh.size
    midpoints = 0.5 * (mesh[:-1] + mesh[1:])
    widths = np.diff(mesh)
    fractions = np.asarray(settings.off_collocation_fractions, dtype=float)
    off_sites = (mesh[:-1, None] + widths[:, None] * fractions).ravel()

    node_values, node_first, node_second = spline.basis(mesh)
    mid_values, mid_first, mid_second = spline.basis(midpoints)
    off_values, off_first, off_second = spline.basis(off_sites)

    reference = _reference_curve(track)
    node_reference = reference.evaluate(mesh)
    mid_reference = reference.evaluate(midpoints)
    k0_nodes, dk0_nodes = np.asarray(node_reference.curvature), np.asarray(node_reference.curvature_s)
    k0_mids, dk0_mids = np.asarray(mid_reference.curvature), np.asarray(mid_reference.curvature_s)
    h_nodes, hs_nodes = np.asarray(node_reference.metric), np.asarray(node_reference.metric_s)
    h_mids, hs_mids = np.asarray(mid_reference.metric), np.asarray(mid_reference.metric_s)
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
    node_acceleration_upper_indices: list[int] = []
    midpoint_acceleration_upper_indices: list[int] = []
    off_site_acceleration_upper_indices: list[int] = []

    def add(expression, low, high):
        constraints.append(expression)
        lower.append(low)
        upper.append(high)
        return len(constraints) - 1

    curvature_lower = float(surface.curvature[0])
    curvature_upper = float(surface.curvature[-1])
    fold_guard = float(settings.fold_guard)

    def limits(offset, psi, q, kappa, car_speed, car_acceleration,
               centreline_curvature):
        add(psi, -float(settings.maximum_heading), float(settings.maximum_heading))
        add(q, -float(settings.maximum_curvature_rate),
            float(settings.maximum_curvature_rate))
        add(kappa, curvature_lower, curvature_upper)
        lower_acceleration, upper_acceleration, domain_speed = lookup(car_speed, kappa)
        add(car_acceleration - lower_acceleration, 0.0, math.inf)
        acceleration_upper_index = add(upper_acceleration - car_acceleration, 0.0, math.inf)
        add(domain_speed - car_speed, 0.0, math.inf)
        add(1.0 - centreline_curvature * offset, fold_guard, math.inf)
        return acceleration_upper_index

    def hermite_speed(index, coordinate):
        """Actual cubic HS speed at an interior element coordinate."""
        h = float(widths[index])
        x = float(coordinate)
        metric0, metric1 = node_geometry[index][4], node_geometry[index + 1][4]
        f0 = acceleration[index] * metric0 / speed[index]
        f1 = acceleration[index + 1] * metric1 / speed[index + 1]
        return ((2*x**3 - 3*x**2 + 1) * speed[index]
                + (x**3 - 2*x**2 + x) * h * f0
                + (-2*x**3 + 3*x**2) * speed[index + 1]
                + (x**3 - x**2) * h * f1)

    def quadratic_acceleration(index, coordinate):
        """Existing node/midpoint/node Lagrange control at an interior site."""
        x = float(coordinate)
        return (2*x*(x - 0.5) * acceleration[index + 1]
                + 4*x*(1 - x) * mid_acceleration[index]
                + 2*(x - 0.5)*(x - 1) * acceleration[index])

    objective = 0
    for index in range(count):
        h = float(widths[index])
        node0 = node_geometry[index]
        node1 = node_geometry[index + 1]
        middle = mid_geometry[index]
        metric0, metric1, metric_mid = node0[4], node1[4], middle[4]
        f0 = acceleration[index] * metric0 / speed[index]
        f1 = acceleration[index + 1] * metric1 / speed[index + 1]
        midpoint_speed = 0.5 * (speed[index] + speed[index + 1]) + h / 8.0 * (f0 - f1)
        f_mid = (
            mid_acceleration[index] * metric_mid / midpoint_speed
        )
        add(speed[index + 1] - speed[index]
            - h / 6.0 * (f0 + 4.0 * f_mid + f1), 0.0, 0.0)
        # The quadratic control is globally boxed through its Bernstein
        # coefficients.  Node and midpoint values alone permit an interior
        # excursion; b1 is the middle Bernstein coefficient.
        bernstein_middle = 2.0 * mid_acceleration[index] - 0.5 * (
            acceleration[index] + acceleration[index + 1]
        )
        add(bernstein_middle, float(np.min(surface.acceleration_min)),
            float(np.max(surface.acceleration_max)))
        # The Hermite midpoint speed is a denominator in the dynamics and the
        # objective; keep it positive even when the node speeds are at the floor.
        add(midpoint_speed - float(settings.minimum_speed), 0.0, math.inf)
        # Time per unit track distance is den / (v cos(psi)); den enters because
        # the offset path is longer than the centreline arc it is measured on.
        objective += h / 6.0 * (
            metric0 / speed[index]
            + 4.0 * metric_mid / midpoint_speed
            + metric1 / speed[index + 1]
        )
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
            for local in range(fractions.size):
                site = index * fractions.size + local
                add(n_off[site], float(off_lower[site]), float(off_upper[site]))
            add(n_nodes[index + 1], float(corridor_lower[index + 1]),
                float(corridor_upper[index + 1]))
        node_acceleration_upper_indices.append(limits(
            n_nodes[index], node0[1], node0[2], node0[3], speed[index],
            acceleration[index], float(k0_nodes[index]),
        ))
        midpoint_acceleration_upper_indices.append(limits(
            n_mids[index], middle[1], middle[2], middle[3], midpoint_speed,
            mid_acceleration[index], float(k0_mids[index]),
        ))
        node_acceleration_upper_indices.append(limits(
            n_nodes[index + 1], node1[1], node1[2], node1[3],
            speed[index + 1], acceleration[index + 1],
            float(k0_nodes[index + 1]),
        ))
        for local in range(fractions.size):
            site = index * fractions.size + local
            off_speed = hermite_speed(index, fractions[local])
            off_acceleration = quadratic_acceleration(index, fractions[local])
            # These are feasibility rows only: retain the single HS speed
            # defect above rather than enforcing additional polynomial dynamics.
            add(off_speed - float(settings.minimum_speed), 0.0, math.inf)
            off_site_acceleration_upper_indices.append(limits(
                n_off[site], off_geometry[site][1], off_geometry[site][2],
                off_geometry[site][3], off_speed, off_acceleration,
                float(k0_off[site]),
            ))
            if cap_off is not None:
                add(off_speed - float(cap_off[site]), -math.inf, 0.0)
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
        "count": count,
        "control_count": control_count,
        "spline": spline,
        "reference": reference,
        # Diagnostic-only mapping in element-major/fraction-major order. It
        # preserves the NLP layout while allowing regressions to identify a
        # particular physical off-site capability row.
        "node_acceleration_upper_indices": np.asarray(
            node_acceleration_upper_indices, dtype=int,
        ),
        "midpoint_acceleration_upper_indices": np.asarray(
            midpoint_acceleration_upper_indices, dtype=int,
        ),
        "off_site_acceleration_upper_indices": np.asarray(
            off_site_acceleration_upper_indices, dtype=int,
        ),
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
    cap_values = _speed_cap_values(speed_cap, nodes)
    minimum = float(settings.minimum_speed)
    top = float(np.max(surface.domain_speed_values)) if signed else float(surface.speed[-1])
    widths = np.diff(mesh)

    def amax(car_speed, car_curvature):
        if signed:
            # A forward/backward sweep reuses a speed at a neighbouring
            # curvature; clamp its *seed query* to that local domain only.
            car_speed = min(float(car_speed), surface.domain_speed(float(car_curvature)))
            return surface.limits(car_speed, float(car_curvature))[1]
        return _bilinear(
            surface.acceleration_max, surface.speed, surface.curvature,
            car_speed, car_curvature,
        )

    def amin(car_speed, car_curvature):
        if signed:
            car_speed = min(float(car_speed), surface.domain_speed(float(car_curvature)))
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


def solve_spline_line(track, corridor, surface, mesh, spline,
                      settings: QssSettings, *, seed: FloatArray | None = None,
                      speed_cap=None) -> SplineSolution:
    """Solve the consistent spline minimum-time problem on a fixed mesh.

    The status and tolerance contract matches :func:`solve_line`: a solver
    status other than ``Solve_Succeeded``, a non-finite value, or a constraint
    violation above ``settings.tolerance`` raises ``RuntimeError``.
    """
    import casadi as ca

    built = build_spline_ocp(
        track, corridor, surface, mesh, spline, settings, speed_cap=speed_cap,
    )
    count = built["count"]
    control_count = built["control_count"]
    size = int(built["nlp"]["x"].numel())
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

    center, scale = _decision_scaling(control_count, built["mesh"].size)
    original_x = built["nlp"]["x"]
    scaled_x = ca.MX.sym("spline_scaled_x", size)
    physical_x = ca.DM(center) + ca.DM(scale) * scaled_x
    scaled_nlp = {
        "x": scaled_x,
        "f": ca.substitute(built["nlp"]["f"], original_x, physical_x),
        "g": ca.substitute(built["nlp"]["g"], original_x, physical_x),
    }
    lower = (built["x_lower"] - center) / scale
    upper = (built["x_upper"] - center) / scale

    tolerances = _tolerance_schedule(settings)
    scaled_seed = (vector - center) / scale
    lam_x = lam_g = None
    total_iterations = 0
    for stage, tolerance in enumerate(tolerances):
        remaining_iterations = settings.maximum_iterations - total_iterations
        if remaining_iterations <= 0:
            status = "Maximum_Iterations_Exceeded"
            break
        options = {
            "ipopt.tol": tolerance,
            "ipopt.constr_viol_tol": tolerance,
            "ipopt.max_iter": remaining_iterations,
            "ipopt.hessian_approximation": "exact",
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
        solver = ca.nlpsol("qss_spline_line", "ipopt", scaled_nlp, options)
        arguments = {
            "x0": scaled_seed,
            "lbx": lower,
            "ubx": upper,
            "lbg": built["g_lower"],
            "ubg": built["g_upper"],
        }
        if lam_x is not None:
            arguments.update({"lam_x0": lam_x, "lam_g0": lam_g})
        result = solver(**arguments)
        stats = solver.stats()
        status = str(stats["return_status"])
        total_iterations += int(stats["iter_count"])
        values = center + scale * np.asarray(result["x"], dtype=float).ravel()
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

    _validate_solver_result(
        status, values, constraints, objective, violation, settings.tolerance,
        built["x_lower"], built["x_upper"],
    )
    violation = max(violation, float(np.max(np.maximum(
        np.maximum(built["x_lower"] - values, 0.0), values - built["x_upper"],
    ))))
    blocks = unpack(values, count, control_count)
    trajectory = OffsetTrajectory(built["reference"], spline, blocks["coefficients"])
    evaluated = trajectory.evaluate(built["mesh"])
    # ``result['f']`` includes optional open regularisation; report physical time separately.
    metric = np.asarray(evaluated.metric, dtype=float)
    evaluator = _SplineStateEvaluator(
        trajectory, built["mesh"], blocks["speed"], blocks["acceleration"],
        blocks["mid_acceleration"],
    )
    midpoint = 0.5 * (built["mesh"][:-1] + built["mesh"][1:])
    midpoint_speed = np.asarray(evaluator.evaluate(midpoint)["speed"], dtype=float)
    midpoint_metric = np.asarray(trajectory.evaluate(midpoint).metric, dtype=float)
    physical_time = float(np.sum(np.diff(built["mesh"]) / 6.0 * (
        metric[:-1] / blocks["speed"][:-1]
        + 4.0 * midpoint_metric / midpoint_speed
        + metric[1:] / blocks["speed"][1:]
    )))
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
        trajectory=trajectory,
        midpoint_acceleration=np.array(blocks["mid_acceleration"], copy=True),
        objective_value=objective,
        state_evaluator=evaluator,
    )
