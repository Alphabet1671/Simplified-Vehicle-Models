"""Minimum-time quasi-steady racing line in the distance domain.

Decision vector is grouped: [n | psi | v | a | dpsi/ds | a_mid | dpsi/ds_mid].
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class QssSettings:
    """Transcription and solver settings."""
    fold_guard: float = 0.2
    maximum_heading: float = 0.6
    maximum_curvature_rate: float = 0.5
    minimum_speed: float = 0.5
    regularisation: float = 1.0e-6
    tolerance: float = 1.0e-6
    maximum_iterations: int = 3000
    tolerance_homotopy: bool = False
    start_speed: float = 0.0
    open_regularisation: float = 1.0e-6
    offset_control_spacing: float | None = None
    # Interior sites for spline-OCP physical constraints.  They supplement the
    # Hermite--Simpson midpoint; they are not extra dynamics defects.
    off_collocation_fractions: tuple[float, ...] = (0.125, 0.25, 0.75, 0.875)


@dataclass(frozen=True, slots=True)
class QssSolution:
    """One accepted transcription solution."""
    distance: FloatArray
    offset: FloatArray
    heading_error: FloatArray
    speed: FloatArray
    acceleration: FloatArray
    curvature_rate: FloatArray
    lap_time: float
    status: str
    iterations: int
    maximum_violation: float
    mid_acceleration: FloatArray | None = None
    mid_curvature_rate: FloatArray | None = None


def _validate_inputs(track, corridor, surface, mesh, settings):
    if not isinstance(settings, QssSettings):
        raise TypeError("settings must be QssSettings")
    if not math.isfinite(settings.fold_guard) or not (0.0 < settings.fold_guard < 1.0):
        raise ValueError("fold_guard must lie strictly between zero and one")
    finite_positive = (settings.maximum_heading, settings.maximum_curvature_rate,
                       settings.minimum_speed, settings.tolerance)
    if not all(math.isfinite(x) and x > 0.0 for x in finite_positive):
        raise ValueError("heading, curvature-rate, speed, and tolerance settings must be finite and positive")
    if settings.maximum_heading >= 0.5 * math.pi:
        raise ValueError("maximum_heading must be less than pi/2")
    if not math.isfinite(settings.regularisation) or settings.regularisation < 0.0:
        raise ValueError("regularisation must be finite and nonnegative")
    if not isinstance(settings.tolerance_homotopy, (bool, np.bool_)):
        raise ValueError("tolerance_homotopy must be a boolean")
    if not math.isfinite(settings.start_speed) or settings.start_speed < 0.0:
        raise ValueError("start_speed must be finite and nonnegative")
    if not math.isfinite(settings.open_regularisation) or settings.open_regularisation < 0.0:
        raise ValueError("open_regularisation must be finite and nonnegative")
    if settings.offset_control_spacing is not None:
        if (not math.isfinite(settings.offset_control_spacing)
                or settings.offset_control_spacing <= 0.0):
            raise ValueError("offset_control_spacing must be finite and positive")
    fractions = settings.off_collocation_fractions
    if (not isinstance(fractions, tuple) or not fractions
            or any(not isinstance(value, (int, float, np.integer, np.floating))
                   or isinstance(value, (bool, np.bool_))
                   or not math.isfinite(value) or not 0.0 < value < 1.0
                   or math.isclose(value, 0.5, rel_tol=0.0, abs_tol=1.0e-12)
                   for value in fractions)
            or len(set(fractions)) != len(fractions)):
        raise ValueError("off_collocation_fractions must be distinct finite interior fractions excluding midpoint")
    if isinstance(settings.maximum_iterations, bool) or settings.maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if int(settings.maximum_iterations) != settings.maximum_iterations:
        raise ValueError("maximum_iterations must be a positive integer")
    closed = bool(getattr(track, "is_closed", False))
    if closed and not getattr(corridor, "is_periodic", False):
        raise ValueError("quasi-steady line OCP requires a periodic corridor for a closed track")
    if not closed and getattr(corridor, "is_closed", True):
        raise ValueError("quasi-steady line OCP requires an open corridor for an open track")
    mesh = np.asarray(mesh, dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0):
        raise ValueError("mesh must be a finite strictly increasing vector")
    length = float(track.lap_length)
    if not np.isclose(mesh[0], 0.0, rtol=0.0, atol=1e-10) or not np.isclose(mesh[-1], length, rtol=1e-10, atol=1e-9):
        raise ValueError("mesh must span exactly zero to track lap length")
    if not np.allclose(corridor.distance, track.distance, rtol=0.0, atol=1e-9):
        raise ValueError("corridor and track distance grids must match")
    if not np.isclose(corridor.distance[-1], length, rtol=1e-10, atol=1e-9):
        raise ValueError("corridor lap length must match track")
    # SignedTransientCapability is opt-in and uses a normalized ``u`` axis;
    # never relabel that axis as physical speed merely to satisfy legacy checks.
    signed = getattr(surface, "representation", None) == "signed_transient_normalized_v1"
    speed_axis = np.asarray(surface.u if signed else surface.speed, dtype=float)
    axes = (speed_axis, np.asarray(surface.curvature, dtype=float))
    if any(axis.ndim != 1 or axis.size < 2 or not np.all(np.isfinite(axis)) or np.any(np.diff(axis) <= 0.0) for axis in axes):
        raise ValueError("capability surface axes must be finite strictly increasing vectors")
    if signed and (not np.isclose(speed_axis[0], 0.0) or not np.isclose(speed_axis[-1], 1.0)):
        raise ValueError("signed capability must use a normalized u=[0,1] axis")
    expected = (axes[0].size, axes[1].size)
    for name in ("acceleration_max", "acceleration_min"):
        values = np.asarray(getattr(surface, name), dtype=float)
        if values.shape != expected or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must be a finite array matching the surface axes")
    if np.any(surface.acceleration_min > surface.acceleration_max):
        raise ValueError("surface acceleration_min must not exceed acceleration_max")
    if signed:
        ceiling_values = np.asarray(surface.domain_speed_values, dtype=float)
        if (ceiling_values.shape != axes[1].shape or not np.all(np.isfinite(ceiling_values))
                or np.any(ceiling_values <= float(surface.speed_floor))):
            raise ValueError("signed domain speeds must exceed the physical speed floor")
        if settings.minimum_speed < float(surface.speed_floor) or settings.minimum_speed >= float(np.max(ceiling_values)):
            raise ValueError("minimum_speed must lie within the signed physical speed domain")
    else:
        ceiling_values = np.asarray(surface.speed_ceiling, dtype=float)
        if ceiling_values.shape != axes[1].shape or not np.all(np.isfinite(ceiling_values)) or np.any(ceiling_values <= 0.0):
            raise ValueError("speed_ceiling must be finite, positive, and match curvature axis")
        if settings.minimum_speed < axes[0][0] or settings.minimum_speed >= axes[0][-1]:
            raise ValueError("minimum_speed must lie within the capability speed axis")
    if np.any(np.asarray(track.curvature, dtype=float) < axes[1][0] - 1e-12) or np.any(np.asarray(track.curvature, dtype=float) > axes[1][-1] + 1e-12):
        raise ValueError("track curvature lies outside capability surface")
    return mesh


def build_mesh(track, spacing: float) -> FloatArray:
    """Uniform mesh ending exactly at lap length."""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be finite and positive")
    length = float(track.lap_length)
    count = max(int(math.ceil(length / spacing)), 4)
    return np.linspace(0.0, length, count + 1)


def periodic_sample(values, distance, positions, *, period: float | None = None) -> FloatArray:
    """Linearly sample periodic track data."""
    values = np.asarray(values, dtype=float)
    distance = np.asarray(distance, dtype=float)
    query = np.asarray(positions, dtype=float)
    if values.ndim != 1 or distance.shape != values.shape or values.size < 2:
        raise ValueError("periodic data must be matching one-dimensional arrays")
    period = float(distance[-1] - distance[0] if period is None else period)
    if not math.isfinite(period) or period <= 0.0 or distance[-1] > distance[0] + period + 1e-9:
        raise ValueError("period must be positive and cover the distance grid")
    nodes = distance
    samples = values
    if distance[-1] < distance[0] + period - 1e-9:
        nodes = np.r_[distance, distance[0] + period]
        samples = np.r_[values, values[0]]
    wrapped = (query - distance[0]) % period + distance[0]
    wrapped = np.where(np.isclose(wrapped, distance[0]) & (query > distance[0]), distance[0] + period, wrapped)
    return np.interp(wrapped, nodes, samples)


def track_sample(values, distance, positions, *, closed: bool, period: float) -> FloatArray:
    """Linearly sample track data, wrapping only for a closed track."""
    if closed:
        return periodic_sample(values, distance, positions, period=period)
    distance = np.asarray(distance, dtype=float)
    query = np.asarray(positions, dtype=float)
    tolerance = 1.0e-9
    if query.size == 0:
        raise ValueError("sample positions must not be empty")
    if np.any(query < distance[0] - tolerance) or np.any(query > distance[-1] + tolerance):
        raise ValueError("sample positions must lie on the track distance range")
    return np.interp(query, distance, np.asarray(values, dtype=float))


def _element_curvature(track, mesh) -> FloatArray:
    return track_sample(
        track.curvature, track.distance, mesh,
        closed=bool(track.is_closed), period=float(track.lap_length),
    )


def _bilinear(grid, x_axis, y_axis, x: float, y: float) -> float:
    """Bilinear interpolation matching CasADi's linear 2-D interpolant."""
    x_axis = np.asarray(x_axis, dtype=float)
    y_axis = np.asarray(y_axis, dtype=float)
    x = float(np.clip(x, x_axis[0], x_axis[-1]))
    y = float(np.clip(y, y_axis[0], y_axis[-1]))
    i = int(np.clip(np.searchsorted(x_axis, x) - 1, 0, x_axis.size - 2))
    j = int(np.clip(np.searchsorted(y_axis, y) - 1, 0, y_axis.size - 2))
    tx = (x - x_axis[i]) / (x_axis[i + 1] - x_axis[i])
    ty = (y - y_axis[j]) / (y_axis[j + 1] - y_axis[j])
    values = np.asarray(grid, dtype=float)
    return float(
        (1.0 - tx) * (1.0 - ty) * values[i, j]
        + tx * (1.0 - ty) * values[i + 1, j]
        + (1.0 - tx) * ty * values[i, j + 1]
        + tx * ty * values[i + 1, j + 1]
    )


def capability_envelope_time(track, surface, settings: QssSettings, *, step: float = 0.05,
                             start_speed: float | None = None) -> float:
    """Integrate the sampled capability along an open centreline.

    This is an independent one-dimensional reference for the open-track
    zero-corridor control. It uses the same sampled surface as the OCP, so it
    checks the transcription rather than validating the surface itself.
    """
    if track.is_closed:
        raise ValueError("capability envelope time is defined for an open track")
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("integration step must be finite and positive")
    length = float(track.lap_length)
    count = max(int(math.ceil(length / step)), 1)
    positions = np.linspace(0.0, length, count + 1)
    curvature = track_sample(
        track.curvature, track.distance, positions,
        closed=False, period=length,
    )
    if start_speed is None:
        speed = max(float(settings.start_speed), float(settings.minimum_speed))
    else:
        if not math.isfinite(start_speed) or start_speed < 0.0:
            raise ValueError("start_speed must be finite and nonnegative")
        speed = float(start_speed)
    elapsed = 0.0
    for index in range(count):
        ds = float(positions[index + 1] - positions[index])
        kappa = 0.5 * (float(curvature[index]) + float(curvature[index + 1]))
        acceleration = _bilinear(
            surface.acceleration_max, surface.speed, surface.curvature,
            speed, kappa,
        )
        next_speed = math.sqrt(max(speed * speed + 2.0 * acceleration * ds, 0.0))
        ceiling = float(np.interp(kappa, surface.curvature, surface.speed_ceiling))
        next_speed = min(next_speed, ceiling)
        mean_speed = 0.5 * (speed + next_speed)
        if mean_speed <= 0.0:
            raise ValueError("capability envelope reached zero speed")
        elapsed += ds / mean_speed
        speed = next_speed
    return elapsed


def launch_time(track, surface, settings: QssSettings, *, samples: int = 200) -> float:
    """Return the time to accelerate from rest to the OCP start speed.

    The distance-domain OCP cannot use zero speed because its dynamics divide by
    speed. For an open standing-start course the reported time must include the
    omitted launch interval.
    """
    target = max(float(settings.start_speed), float(settings.minimum_speed))
    if target <= 0.0:
        return 0.0
    if samples < 1:
        raise ValueError("samples must be positive")
    curvature = float(track_sample(
        track.curvature, track.distance, [float(track.distance[0])],
        closed=bool(track.is_closed), period=float(track.lap_length),
    )[0])
    speeds = np.linspace(0.0, target, samples + 1)
    inverse = []
    for speed in speeds:
        acceleration = _bilinear(
            surface.acceleration_max, surface.speed, surface.curvature,
            speed, curvature,
        )
        if acceleration <= 0.0:
            raise ValueError("capability surface cannot launch the vehicle")
        inverse.append(1.0 / acceleration)
    return float(np.trapezoid(np.asarray(inverse), speeds))


def _offset_basis(track, mesh, spacing: float | None):
    """Return a node-by-control linear interpolation basis for offset."""
    mesh = np.asarray(mesh, dtype=float)
    if spacing is None:
        return None, None
    length = float(track.lap_length)
    controls = max(int(math.ceil(length / spacing)), 2)
    if track.is_closed:
        # A periodic harmonic basis is smooth across the wrap. A piecewise-linear
        # periodic basis has a slope kink there, which the free-lateral NLP does
        # not tolerate.
        harmonics = max(1, int(round(length / (2.0 * spacing))))
        controls = 2 * harmonics + 1
        angle = 2.0 * math.pi * np.asarray(mesh, dtype=float) / length
        columns = [np.ones(mesh.size, dtype=float)]
        for order in range(1, harmonics + 1):
            columns.append(np.cos(order * angle))
            columns.append(np.sin(order * angle))
        basis = np.column_stack(columns)
        return basis, None
    positions = np.linspace(0.0, length, controls)
    scaled = mesh / length * (controls - 1)
    lower = np.clip(np.floor(scaled).astype(int), 0, controls - 2)
    weight = scaled - lower
    upper = lower + 1
    basis = np.zeros((mesh.size, controls), dtype=float)
    rows = np.arange(mesh.size)
    basis[rows, lower] = 1.0 - weight
    basis[rows, upper] += weight
    return basis, positions


def build_ocp(track, corridor, surface, mesh, settings: QssSettings):
    """Build CasADi NLP and its variable/constraint bounds."""
    import casadi as ca

    if getattr(surface, "representation", None) == "signed_transient_normalized_v1":
        raise ValueError("signed transient capability requires the explicit spline adapter")
    mesh = _validate_inputs(track, corridor, surface, mesh, settings)
    count, nodes = mesh.size - 1, mesh.size
    amax = ca.interpolant("qss_amax", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_max.ravel(order="F").tolist())
    amin = ca.interpolant("qss_amin", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_min.ravel(order="F").tolist())
    ceiling = ca.interpolant("qss_vceil", "linear", [surface.curvature.tolist()], surface.speed_ceiling.tolist())
    k0 = _element_curvature(track, mesh)
    sample = lambda values, positions: track_sample(
        values, corridor.distance, positions,
        closed=bool(track.is_closed), period=float(track.lap_length),
    )
    nlo = sample(corridor.minimum, mesh)
    nhi = sample(corridor.maximum, mesh)
    basis, control_distance = _offset_basis(track, mesh, settings.offset_control_spacing)
    if basis is None:
        offset_symbols = [ca.MX.sym(f"n_{i}") for i in range(nodes)]
        n = offset_symbols
        control_count = nodes
        offset_lower, offset_upper = nlo, nhi
    else:
        control_count = basis.shape[1]
        offset_symbols = [ca.MX.sym(f"c_{i}") for i in range(control_count)]
        n = [
            sum(float(basis[row, column]) * offset_symbols[column]
                for column in np.flatnonzero(basis[row]))
            for row in range(nodes)
        ]
        offset_lower = np.full(control_count, float(np.min(nlo)))
        offset_upper = np.full(control_count, float(np.max(nhi)))
        if control_distance is None:
            if np.allclose(nlo, 0.0) and np.allclose(nhi, 0.0):
                offset_lower = np.zeros(control_count)
                offset_upper = np.zeros(control_count)
            else:
                width = max(float(np.max(np.abs(nlo))), float(np.max(np.abs(nhi))))
                limit = 2.0 * width + 1.0
                offset_lower = np.full(control_count, -limit)
                offset_upper = np.full(control_count, limit)
    psi = [ca.MX.sym(f"psi_{i}") for i in range(nodes)]
    v = [ca.MX.sym(f"v_{i}") for i in range(nodes)]
    a = [ca.MX.sym(f"a_{i}") for i in range(nodes)]
    q = [ca.MX.sym(f"q_{i}") for i in range(nodes)]
    am = [ca.MX.sym(f"am_{i}") for i in range(count)]
    qm = [ca.MX.sym(f"qm_{i}") for i in range(count)]
    x = ca.vertcat(*offset_symbols, *psi, *v, *a, *q, *am, *qm)
    acceleration_lower = float(np.min(surface.acceleration_min))
    acceleration_upper = float(np.max(surface.acceleration_max))
    xl = np.concatenate([offset_lower, np.full(nodes,-settings.maximum_heading), np.full(nodes,settings.minimum_speed), np.full(nodes,acceleration_lower), np.full(nodes,-settings.maximum_curvature_rate), np.full(count,acceleration_lower), np.full(count,-settings.maximum_curvature_rate)])
    xu = np.concatenate([offset_upper, np.full(nodes,settings.maximum_heading), np.full(nodes,float(surface.speed[-1])), np.full(nodes,acceleration_upper), np.full(nodes,settings.maximum_curvature_rate), np.full(count,acceleration_upper), np.full(count,settings.maximum_curvature_rate)])

    def rates(state, control, curvature):
        offset, heading, speed = state
        acceleration, rate = control
        den = 1 - curvature * offset
        return ca.vertcat(ca.tan(heading)*den, rate, acceleration*den/(speed*ca.cos(heading)))

    def kap(offset, heading, rate, curvature):
        return (curvature + rate)*ca.cos(heading)/(1-curvature*offset)

    constraints, gl, gu = [], [], []
    def add(expr, low, high):
        constraints.append(expr); gl.append(low); gu.append(high)
    def limits(offset, heading, speed, accel, rate, curvature):
        vehicle_k = kap(offset, heading, rate, curvature)
        add(vehicle_k, float(surface.curvature[0]), float(surface.curvature[-1]))
        add(accel-amin(ca.vertcat(speed,vehicle_k)),0.0,float("inf"))
        add(amax(ca.vertcat(speed,vehicle_k))-accel,0.0,float("inf"))
        add(ceiling(vehicle_k)-speed,0.0,float("inf"))
        add(1-curvature*offset,settings.fold_guard,float("inf"))

    objective = 0
    for i in range(count):
        h = float(mesh[i+1]-mesh[i]); midpos=.5*(mesh[i]+mesh[i+1])
        kmid=float(_element_curvature(track,[midpos])[0])
        interior_positions=mesh[i]+h*np.asarray([0.25,0.5,0.75])
        interior_lower=sample(corridor.minimum, interior_positions)
        interior_upper=sample(corridor.maximum, interior_positions)
        s0=ca.vertcat(n[i],psi[i],v[i]); s1=ca.vertcat(n[i+1],psi[i+1],v[i+1])
        f0=rates((n[i],psi[i],v[i]),(a[i],q[i]),float(k0[i]))
        f1=rates((n[i+1],psi[i+1],v[i+1]),(a[i+1],q[i+1]),float(k0[i+1]))
        sm=.5*(s0+s1)+h/8*(f0-f1)
        hermite_states=[(2*t**3-3*t**2+1)*s0+(-2*t**3+3*t**2)*s1+h*(t**3-2*t**2+t)*f0+h*(t**3-t**2)*f1 for t in (0.25,0.5,0.75)]
        fm=rates((sm[0],sm[1],sm[2]),(am[i],qm[i]),kmid)
        defect=s1-s0-h/6*(f0+4*fm+f1)
        for row in range(3): add(defect[row],0.0,0.0)
        # The Hermite midpoint speed is a denominator in the time rate; keep it
        # positive even when the node speeds sit on the solver floor.
        add(sm[2],float(settings.minimum_speed),float("inf"))
        def time_rate(offset,heading,speed,curvature):
            return (1-curvature*offset)/(speed*ca.cos(heading))
        objective += h/6*(time_rate(n[i],psi[i],v[i],float(k0[i]))+4*time_rate(sm[0],sm[1],sm[2],kmid)+time_rate(n[i+1],psi[i+1],v[i+1],float(k0[i+1])))
        objective += settings.regularisation*h/6*(q[i]**2+4*qm[i]**2+q[i+1]**2)
        if not track.is_closed:
            # On a straight open section neither lateral offset nor heading has
            # an intrinsic cost, so the OCP would have a null direction. A tiny
            # preference for the centreline makes the NLP locally unique without
            # materially changing the minimum time.
            objective += settings.open_regularisation*h/6*(n[i]**2+4*sm[0]**2+n[i+1]**2)
            objective += settings.open_regularisation*h/6*(psi[i]**2+4*sm[1]**2+psi[i+1]**2)
        for sample_index, (dense_state, dense_low, dense_high) in enumerate(zip(hermite_states,interior_lower,interior_upper)):
            if sample_index == 1:
                continue  # Hermite t=1/2 equals the HS collocation midpoint.
            add(dense_state[0],float(dense_low),float(dense_high))
        add(sm[0],float(interior_lower[1]),float(interior_upper[1]))
        if basis is not None and not (np.allclose(nlo, 0.0) and np.allclose(nhi, 0.0)):
            # A reduced basis replaces local offset bounds, so the node values
            # need explicit corridor rows instead of relying on variable bounds.
            # A fully collapsed corridor already fixes every control to zero.
            add(n[i], float(nlo[i]), float(nhi[i]))
            add(n[i+1], float(nlo[i+1]), float(nhi[i+1]))
        limits(n[i],psi[i],v[i],a[i],q[i],float(k0[i]))
        limits(sm[0],sm[1],sm[2],am[i],qm[i],kmid)
        limits(n[i+1],psi[i+1],v[i+1],a[i+1],q[i+1],float(k0[i+1]))
    if track.is_closed:
        if basis is None:
            add(n[0]-n[-1],0.0,0.0)
        add(psi[0]-psi[-1],0.0,0.0); add(v[0]-v[-1],0.0,0.0)
    else:
        # Only heading and launch speed are fixed. Lateral offset stays a free
        # corridor decision at both ends; fixing it as well would duplicate the
        # zero-corridor bounds and leave the open NLP overconstrained.
        start_speed = max(float(settings.start_speed), float(settings.minimum_speed))
        add(psi[0],0.0,0.0); add(psi[-1],0.0,0.0)
        add(v[0]-start_speed,0.0,0.0)
    return {"nlp":{"x":x,"f":objective,"g":ca.vertcat(*constraints)},"x_lower":xl,"x_upper":xu,"g_lower":np.asarray(gl),"g_upper":np.asarray(gu),"mesh":mesh,"count":count,"control_count":control_count,"offset_basis":basis,"control_distance":control_distance}


def unpack(values, count: int, control_count: int | None = None,
           basis: FloatArray | None = None) -> dict[str, FloatArray]:
    nodes=count+1; controls=nodes if control_count is None else int(control_count)
    expected=controls+4*nodes+2*count; flat=np.asarray(values,dtype=float).ravel()
    if flat.size != expected: raise ValueError(f"decision vector has {flat.size} entries, layout needs {expected}")
    if basis is None:
        offset=flat[:controls]
    else:
        basis=np.asarray(basis,dtype=float)
        if basis.shape != (nodes,controls):
            raise ValueError("offset basis must have shape (nodes, controls)")
        offset=basis@flat[:controls]
    heading=flat[controls:controls+nodes]
    speed=flat[controls+nodes:controls+2*nodes]
    acceleration=flat[controls+2*nodes:controls+3*nodes]
    curvature_rate=flat[controls+3*nodes:controls+4*nodes]
    tail=flat[controls+4*nodes:]
    return {
        "offset":offset,"heading":heading,"speed":speed,
        "acceleration":acceleration,"curvature_rate":curvature_rate,
        "mid_acceleration":tail[:count],"mid_curvature_rate":tail[count:],
    }


def initial_guess(track, corridor, surface, mesh, settings: QssSettings) -> FloatArray:
    mesh=_validate_inputs(track,corridor,surface,mesh,settings); count=mesh.size-1
    closed=bool(track.is_closed)
    sample=lambda values: track_sample(values,corridor.distance,mesh,closed=closed,period=float(track.lap_length))
    basis, control_distance = _offset_basis(track, mesh, settings.offset_control_spacing)
    if basis is None:
        offset=.5*(sample(corridor.minimum)+sample(corridor.maximum))
    elif control_distance is None:
        center=.5*(sample(corridor.minimum)+sample(corridor.maximum))
        offset=np.linalg.lstsq(basis,center,rcond=None)[0]
    else:
        offset=.5*(track_sample(corridor.minimum,corridor.distance,control_distance,closed=closed,period=float(track.lap_length))
                   +track_sample(corridor.maximum,corridor.distance,control_distance,closed=closed,period=float(track.lap_length)))
    curvature=np.clip(_element_curvature(track,mesh),surface.curvature[0],surface.curvature[-1])
    ceiling=np.interp(curvature,surface.curvature,surface.speed_ceiling)
    speed=np.full(mesh.size,max(.6*float(np.min(ceiling)),settings.minimum_speed+.1))
    if not closed:
        speed[0]=max(float(settings.start_speed),float(settings.minimum_speed))
    zeros=np.zeros(mesh.size); mids=np.zeros(count)
    return np.concatenate([offset,zeros,speed,zeros,zeros,mids,mids])


def _validate_solver_result(status, values, constraints, objective, violation, tolerance,
                            x_lower=None, x_upper=None):
    """Reject solver outputs that are not converged feasible finite solutions."""
    if status != "Solve_Succeeded":
        raise RuntimeError(f"quasi-steady line solver failed: {status}")
    if not (np.all(np.isfinite(values)) and np.all(np.isfinite(constraints)) and math.isfinite(objective) and math.isfinite(violation)):
        raise RuntimeError("quasi-steady line solver returned non-finite values")
    if x_lower is not None and x_upper is not None:
        if not (np.all(np.isfinite(x_lower)) and np.all(np.isfinite(x_upper))):
            raise RuntimeError("quasi-steady line variable bounds are non-finite")
        x_violation = float(np.max(np.maximum(np.maximum(np.asarray(x_lower)-values,0.0),values-np.asarray(x_upper))))
        violation = max(violation, x_violation)
    if violation > tolerance:
        raise RuntimeError(f"quasi-steady line solution violates constraints by {violation:g}")


def _decision_scaling(node_count: int, control_count: int | None = None) -> tuple[FloatArray, FloatArray]:
    """Return characteristic affine scales for grouped QSS variables.

    Scaled variables are O(1) for typical track geometry and capability data:
    offset in meters, heading in radians, speed around 20 m/s, acceleration in
    m/s², and curvature rate in 1/m. These scales alter conditioning only; the
    NLP objective, constraints, bounds, and returned solution remain physical.
    """
    if isinstance(node_count, (bool, np.bool_)) or not isinstance(node_count, (int, np.integer)):
        raise ValueError("decision scaling node count must be an integer")
    nodes = int(node_count)
    if nodes < 2:
        raise ValueError("decision scaling requires at least two mesh nodes")
    offset_size = nodes if control_count is None else int(control_count)
    if offset_size < 2:
        raise ValueError("decision scaling requires at least two offset controls")
    elements = nodes - 1
    center = np.zeros(offset_size + 4 * nodes + 2 * elements, dtype=float)
    scale = np.ones_like(center)
    starts = (0, offset_size, offset_size + nodes, offset_size + 2 * nodes,
              offset_size + 3 * nodes, offset_size + 4 * nodes,
              offset_size + 4 * nodes + elements)
    sizes = (offset_size, nodes, nodes, nodes, nodes, elements, elements)
    block_scales = (1.0, 0.15, 10.0, 5.0, 0.1, 5.0, 0.1)
    for start, size, block_scale in zip(starts, sizes, block_scales):
        scale[start:start + size] = block_scale
    center[offset_size + nodes:offset_size + 2 * nodes] = 20.0
    return center, scale


def _tolerance_schedule(settings: QssSettings) -> tuple[float, ...]:
    """Return the requested tolerance, optionally preceded by homotopy stages.

    The diagnostic schedule is intentionally opt-in. The iteration cap is shared
    across all stages of one solve, and the last stage always uses the requested
    tolerance.
    """
    if settings.tolerance_homotopy and settings.tolerance < 1.0e-5:
        return (1.0e-4, 1.0e-5, settings.tolerance)
    return (settings.tolerance,)


def solve_line(track, corridor, surface, mesh, settings: QssSettings, *, seed: FloatArray | None = None) -> QssSolution:
    """Solve the free-line minimum-time problem on a fixed mesh."""
    import casadi as ca
    built=build_ocp(track,corridor,surface,mesh,settings); count=built["count"]
    vector=initial_guess(track,corridor,surface,mesh,settings) if seed is None else np.asarray(seed,dtype=float).ravel()
    if vector.size != built["nlp"]["x"].shape[0]: raise ValueError("seed does not match the mesh")
    if not np.all(np.isfinite(vector)):
        raise ValueError("seed must contain only finite values")
    vector=np.clip(vector,built["x_lower"],built["x_upper"])
    center, scale = _decision_scaling(len(mesh), built["control_count"])
    original_x = built["nlp"]["x"]
    scaled_x = ca.MX.sym("qss_scaled_x", vector.size)
    physical_x = ca.DM(center) + ca.DM(scale) * scaled_x
    scaled_nlp = {
        "x": scaled_x,
        "f": ca.substitute(built["nlp"]["f"], original_x, physical_x),
        "g": ca.substitute(built["nlp"]["g"], original_x, physical_x),
    }
    lower = (built["x_lower"] - center) / scale
    upper = (built["x_upper"] - center) / scale
    # Tight solves can destabilize from an otherwise feasible seed. Resolve the
    # same NLP at progressively tighter tolerances and carry IPOPT's multipliers
    # between stages. Multipliers are deliberately not transferred by solve_ladder
    # across meshes, whose variable and constraint layouts differ.
    tolerances = _tolerance_schedule(settings)
    scaled_seed = (vector-center)/scale
    lam_x = lam_g = None
    total_iterations = 0
    for stage, tolerance in enumerate(tolerances):
        remaining_iterations = settings.maximum_iterations - total_iterations
        if remaining_iterations <= 0:
            status = "Maximum_Iterations_Exceeded"
            break
        opts={"ipopt.tol":tolerance,"ipopt.constr_viol_tol":tolerance,"ipopt.max_iter":remaining_iterations,"ipopt.hessian_approximation":"exact","ipopt.nlp_scaling_method":"gradient-based","ipopt.mu_strategy":"adaptive","ipopt.print_level":0,"print_time":False}
        if lam_x is not None:
            opts.update({"ipopt.warm_start_init_point":"yes",
                         "ipopt.warm_start_bound_push":1.0e-8,
                         "ipopt.warm_start_mult_bound_push":1.0e-8,
                         "ipopt.warm_start_slack_bound_push":1.0e-8})
        solver=ca.nlpsol("qss_line","ipopt",scaled_nlp,opts)
        arguments={"x0":scaled_seed,"lbx":lower,"ubx":upper,
                   "lbg":built["g_lower"],"ubg":built["g_upper"]}
        if lam_x is not None:
            arguments.update({"lam_x0":lam_x,"lam_g0":lam_g})
        result=solver(**arguments)
        stats=solver.stats(); status=str(stats["return_status"])
        total_iterations += int(stats["iter_count"])
        values=center + scale * np.asarray(result["x"],dtype=float).ravel()
        g=np.asarray(result["g"],dtype=float).ravel()
        violation=float(np.max(np.maximum(np.maximum(built["g_lower"]-g,0),g-built["g_upper"])))
        objective=float(result["f"])
        if status != "Solve_Succeeded":
            break
        if stage + 1 < len(tolerances):
            scaled_seed=np.asarray(result["x"],dtype=float).ravel()
            lam_x=np.asarray(result["lam_x"],dtype=float).ravel()
            lam_g=np.asarray(result["lam_g"],dtype=float).ravel()
    _validate_solver_result(status, values, g, objective, violation, settings.tolerance,
                            built["x_lower"], built["x_upper"])
    violation=max(violation,float(np.max(np.maximum(np.maximum(built["x_lower"]-values,0.0),values-built["x_upper"]))))
    blocks=unpack(values,count,built["control_count"],built["offset_basis"])
    return QssSolution(np.asarray(mesh,dtype=float),blocks["offset"],blocks["heading"],blocks["speed"],blocks["acceleration"],blocks["curvature_rate"],float(result["f"]),str(stats["return_status"]),total_iterations,violation,blocks["mid_acceleration"],blocks["mid_curvature_rate"])


def project_seed(coarse: QssSolution, mesh, *, is_closed: bool = True,
                 track=None, control_spacing: float | None = None) -> FloatArray:
    """Interpolate prior node and midpoint controls onto a new mesh."""
    mesh=np.asarray(mesh,dtype=float); source=np.asarray(coarse.distance,dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0.0):
        raise ValueError("target mesh must be finite, one-dimensional, and strictly increasing")
    if source.ndim != 1 or source.size < 2 or not np.all(np.isfinite(source)) or np.any(np.diff(source) <= 0.0):
        raise ValueError("solution distance must be finite, one-dimensional, and strictly increasing")
    count=mesh.size-1
    if source.size != coarse.offset.size: raise ValueError("solution distance does not match its node count")
    projected=[np.interp(mesh,source,np.asarray(values,dtype=float)) for values in (coarse.heading_error,coarse.speed,coarse.acceleration,coarse.curvature_rate)]
    if control_spacing is None:
        offset=np.interp(mesh,source,np.asarray(coarse.offset,dtype=float))
    else:
        if track is None:
            raise ValueError("track is required for a reduced offset basis")
        basis, control_distance = _offset_basis(track, mesh, control_spacing)
        if control_distance is None:
            values=track_sample(
                np.asarray(coarse.offset,dtype=float), source, mesh,
                closed=is_closed, period=float(track.lap_length),
            )
            offset=np.linalg.lstsq(basis,values,rcond=None)[0]
        else:
            offset=track_sample(
                np.asarray(coarse.offset,dtype=float), source, control_distance,
                closed=is_closed, period=float(track.lap_length),
            )
    def project_midpoint(values):
        if values is None:
            return np.zeros(count)
        values=np.asarray(values,dtype=float)
        if values.shape != (source.size - 1,) or not np.all(np.isfinite(values)):
            raise ValueError("solution midpoint controls must be finite and match its mesh")
        source_midpoints=.5*(source[:-1]+source[1:])
        target_midpoints=.5*(mesh[:-1]+mesh[1:])
        if not is_closed:
            return np.interp(target_midpoints,source_midpoints,values)
        periodic_midpoints=np.r_[source_midpoints,source_midpoints[0]+float(coarse.distance[-1]-coarse.distance[0])]
        periodic_values=np.r_[values,values[0]]
        wrapped=source_midpoints[0]+np.mod(target_midpoints-source_midpoints[0],float(coarse.distance[-1]-coarse.distance[0]))
        return np.interp(wrapped,periodic_midpoints,periodic_values)
    mids_a=project_midpoint(coarse.mid_acceleration)
    mids_q=project_midpoint(coarse.mid_curvature_rate)
    return np.concatenate([offset,*projected,mids_a,mids_q])


def solve_ladder(track, corridor, surface, settings: QssSettings,
                 spacings: tuple[float, ...] = (5.0, 2.0, 1.0),
                 *, seed: FloatArray | None = None,
                 stage_callback=None) -> list[QssSolution]:
    """Solve coarse to fine, warm starting each stage from the one before.

    For the per-node offset layout, each prior solution is projected onto the
    next stage mesh and used as its seed. The first supplied seed is the seed for
    the first spacing. With a reduced offset basis, the first supplied seed is
    still used, but later stages cold-start because projected basis seeds are
    unreliable. If supplied, ``stage_callback(spacing, solution, wall_seconds)``
    runs after each successful solve; the duration measures the ``solve_line``
    call only.
    """
    if not spacings:
        raise ValueError("at least one mesh spacing is required")
    stages: list[QssSolution] = []
    current = seed
    open_schedule = (1.0e-6, 1.0e-4, 1.0e-3)
    for index, spacing in enumerate(spacings):
        mesh = build_mesh(track, spacing)
        stage_settings = settings
        if not track.is_closed and settings.offset_control_spacing is None:
            stage_settings = replace(
                settings,
                open_regularisation=open_schedule[min(index, len(open_schedule) - 1)],
            )
        started = time.perf_counter()
        stage_seed = current
        try:
            solution = solve_line(track, corridor, surface, mesh, stage_settings, seed=stage_seed)
        except RuntimeError:
            if stage_seed is None:
                raise
            solution = solve_line(track, corridor, surface, mesh, stage_settings, seed=None)
        wall_seconds = time.perf_counter() - started
        if solution.status != "Solve_Succeeded":
            raise RuntimeError(
                f"stage at {spacing} m failed with {solution.status} "
                f"after {solution.iterations} iterations"
            )
        stages.append(solution)
        if stage_callback is not None:
            stage_callback(float(spacing), solution, wall_seconds)
        if index + 1 < len(spacings):
            if settings.offset_control_spacing is None:
                next_mesh = build_mesh(track, spacings[index + 1])
                current = project_seed(
                    solution, next_mesh, is_closed=bool(track.is_closed),
                    track=track, control_spacing=None,
                )
            else:
                # A projected reduced-basis seed is not reliable enough to use;
                # later stages cold-start from the default seed.
                current = None
    return stages
