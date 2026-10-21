"""Minimum-time quasi-steady racing line in the distance domain.

Decision vector is grouped: [n | psi | v | a | dpsi/ds | a_mid | dpsi/ds_mid].
"""
from __future__ import annotations

from dataclasses import dataclass
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
    if isinstance(settings.maximum_iterations, bool) or settings.maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if int(settings.maximum_iterations) != settings.maximum_iterations:
        raise ValueError("maximum_iterations must be a positive integer")
    if not getattr(track, "is_closed", False):
        raise ValueError("quasi-steady line OCP requires a closed track")
    if not getattr(corridor, "is_periodic", False):
        raise ValueError("quasi-steady line OCP requires a periodic corridor")
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
    axes = (np.asarray(surface.speed, dtype=float), np.asarray(surface.curvature, dtype=float))
    if any(axis.ndim != 1 or axis.size < 2 or not np.all(np.isfinite(axis)) or np.any(np.diff(axis) <= 0.0) for axis in axes):
        raise ValueError("capability surface axes must be finite strictly increasing vectors")
    expected = (axes[0].size, axes[1].size)
    for name in ("acceleration_max", "acceleration_min"):
        values = np.asarray(getattr(surface, name), dtype=float)
        if values.shape != expected or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must be a finite array matching the surface axes")
    if np.any(surface.acceleration_min > surface.acceleration_max):
        raise ValueError("surface acceleration_min must not exceed acceleration_max")
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


def _element_curvature(track, mesh) -> FloatArray:
    return periodic_sample(track.curvature, track.distance, mesh, period=float(track.lap_length))


def build_ocp(track, corridor, surface, mesh, settings: QssSettings):
    """Build CasADi NLP and its variable/constraint bounds."""
    import casadi as ca

    mesh = _validate_inputs(track, corridor, surface, mesh, settings)
    count, nodes = mesh.size - 1, mesh.size
    amax = ca.interpolant("qss_amax", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_max.ravel(order="F").tolist())
    amin = ca.interpolant("qss_amin", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_min.ravel(order="F").tolist())
    ceiling = ca.interpolant("qss_vceil", "linear", [surface.curvature.tolist()], surface.speed_ceiling.tolist())
    k0 = _element_curvature(track, mesh)
    nlo = periodic_sample(corridor.minimum, corridor.distance, mesh, period=float(track.lap_length))
    nhi = periodic_sample(corridor.maximum, corridor.distance, mesh, period=float(track.lap_length))
    n = [ca.MX.sym(f"n_{i}") for i in range(nodes)]
    psi = [ca.MX.sym(f"psi_{i}") for i in range(nodes)]
    v = [ca.MX.sym(f"v_{i}") for i in range(nodes)]
    a = [ca.MX.sym(f"a_{i}") for i in range(nodes)]
    q = [ca.MX.sym(f"q_{i}") for i in range(nodes)]
    am = [ca.MX.sym(f"am_{i}") for i in range(count)]
    qm = [ca.MX.sym(f"qm_{i}") for i in range(count)]
    x = ca.vertcat(*n, *psi, *v, *a, *q, *am, *qm)
    acceleration_lower = float(np.min(surface.acceleration_min))
    acceleration_upper = float(np.max(surface.acceleration_max))
    xl = np.concatenate([nlo, np.full(nodes,-settings.maximum_heading), np.full(nodes,settings.minimum_speed), np.full(nodes,acceleration_lower), np.full(nodes,-settings.maximum_curvature_rate), np.full(count,acceleration_lower), np.full(count,-settings.maximum_curvature_rate)])
    xu = np.concatenate([nhi, np.full(nodes,settings.maximum_heading), np.full(nodes,float(surface.speed[-1])), np.full(nodes,acceleration_upper), np.full(nodes,settings.maximum_curvature_rate), np.full(count,acceleration_upper), np.full(count,settings.maximum_curvature_rate)])

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
        interior_lower=periodic_sample(corridor.minimum,corridor.distance,interior_positions,period=float(track.lap_length))
        interior_upper=periodic_sample(corridor.maximum,corridor.distance,interior_positions,period=float(track.lap_length))
        s0=ca.vertcat(n[i],psi[i],v[i]); s1=ca.vertcat(n[i+1],psi[i+1],v[i+1])
        f0=rates((n[i],psi[i],v[i]),(a[i],q[i]),float(k0[i]))
        f1=rates((n[i+1],psi[i+1],v[i+1]),(a[i+1],q[i+1]),float(k0[i+1]))
        sm=.5*(s0+s1)+h/8*(f0-f1)
        hermite_states=[(2*t**3-3*t**2+1)*s0+(-2*t**3+3*t**2)*s1+h*(t**3-2*t**2+t)*f0+h*(t**3-t**2)*f1 for t in (0.25,0.5,0.75)]
        fm=rates((sm[0],sm[1],sm[2]),(am[i],qm[i]),kmid)
        defect=s1-s0-h/6*(f0+4*fm+f1)
        for row in range(3): add(defect[row],0.0,0.0)
        def time_rate(offset,heading,speed,curvature):
            return (1-curvature*offset)/(speed*ca.cos(heading))
        objective += h/6*(time_rate(n[i],psi[i],v[i],float(k0[i]))+4*time_rate(sm[0],sm[1],sm[2],kmid)+time_rate(n[i+1],psi[i+1],v[i+1],float(k0[i+1])))
        objective += settings.regularisation*h/6*(q[i]**2+4*qm[i]**2+q[i+1]**2)
        for sample_index, (dense_state, dense_low, dense_high) in enumerate(zip(hermite_states,interior_lower,interior_upper)):
            if sample_index == 1:
                continue  # Hermite t=1/2 equals the HS collocation midpoint.
            add(dense_state[0],float(dense_low),float(dense_high))
        add(sm[0],float(interior_lower[1]),float(interior_upper[1]))
        limits(n[i],psi[i],v[i],a[i],q[i],float(k0[i]))
        limits(sm[0],sm[1],sm[2],am[i],qm[i],kmid)
        limits(n[i+1],psi[i+1],v[i+1],a[i+1],q[i+1],float(k0[i+1]))
    add(n[0]-n[-1],0.0,0.0); add(psi[0]-psi[-1],0.0,0.0); add(v[0]-v[-1],0.0,0.0)
    return {"nlp":{"x":x,"f":objective,"g":ca.vertcat(*constraints)},"x_lower":xl,"x_upper":xu,"g_lower":np.asarray(gl),"g_upper":np.asarray(gu),"mesh":mesh,"count":count}


def unpack(values, count: int) -> dict[str, FloatArray]:
    nodes=count+1; expected=5*nodes+2*count; flat=np.asarray(values,dtype=float).ravel()
    if flat.size != expected: raise ValueError(f"decision vector has {flat.size} entries, layout needs {expected}")
    names=("offset","heading","speed","acceleration","curvature_rate")
    blocks={name:flat[i*nodes:(i+1)*nodes] for i,name in enumerate(names)}
    tail=flat[5*nodes:]; blocks["mid_acceleration"]=tail[:count]; blocks["mid_curvature_rate"]=tail[count:]
    return blocks


def initial_guess(track, corridor, surface, mesh, settings: QssSettings) -> FloatArray:
    mesh=_validate_inputs(track,corridor,surface,mesh,settings); count=mesh.size-1
    offset=.5*(periodic_sample(corridor.minimum,corridor.distance,mesh)+periodic_sample(corridor.maximum,corridor.distance,mesh))
    curvature=np.clip(_element_curvature(track,mesh),surface.curvature[0],surface.curvature[-1])
    ceiling=np.interp(curvature,surface.curvature,surface.speed_ceiling)
    speed=np.full(mesh.size,max(.6*float(np.min(ceiling)),settings.minimum_speed+.1))
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


def _decision_scaling(node_count: int) -> tuple[FloatArray, FloatArray]:
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
    elements = nodes - 1
    center = np.zeros(5 * nodes + 2 * elements, dtype=float)
    scale = np.ones_like(center)
    starts = (0, nodes, 2 * nodes, 3 * nodes, 4 * nodes, 5 * nodes, 5 * nodes + elements)
    block_scales = (1.0, 0.15, 10.0, 5.0, 0.1, 5.0, 0.1)
    for index, (start, block_scale) in enumerate(zip(starts, block_scales)):
        size = nodes if index < 5 else elements
        scale[start:start + size] = block_scale
    center[2 * nodes:3 * nodes] = 20.0
    return center, scale


def solve_line(track, corridor, surface, mesh, settings: QssSettings, *, seed: FloatArray | None = None) -> QssSolution:
    """Solve the free-line minimum-time problem on a fixed mesh."""
    import casadi as ca
    built=build_ocp(track,corridor,surface,mesh,settings); count=built["count"]
    vector=initial_guess(track,corridor,surface,mesh,settings) if seed is None else np.asarray(seed,dtype=float).ravel()
    if vector.size != built["nlp"]["x"].shape[0]: raise ValueError("seed does not match the mesh")
    if not np.all(np.isfinite(vector)):
        raise ValueError("seed must contain only finite values")
    vector=np.clip(vector,built["x_lower"],built["x_upper"])
    center, scale = _decision_scaling(len(mesh))
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
    opts={"ipopt.tol":settings.tolerance,"ipopt.constr_viol_tol":settings.tolerance,"ipopt.max_iter":settings.maximum_iterations,"ipopt.hessian_approximation":"exact","ipopt.nlp_scaling_method":"gradient-based","ipopt.mu_strategy":"adaptive","ipopt.print_level":0,"print_time":False}
    solver=ca.nlpsol("qss_line","ipopt",scaled_nlp,opts)
    result=solver(x0=(vector-center)/scale,lbx=lower,ubx=upper,lbg=built["g_lower"],ubg=built["g_upper"])
    values=center + scale * np.asarray(result["x"],dtype=float).ravel(); g=np.asarray(result["g"],dtype=float).ravel()
    violation=float(np.max(np.maximum(np.maximum(built["g_lower"]-g,0),g-built["g_upper"])))
    stats=solver.stats(); status=str(stats["return_status"])
    objective=float(result["f"])
    _validate_solver_result(status, values, g, objective, violation, settings.tolerance,
                            built["x_lower"], built["x_upper"])
    violation=max(violation,float(np.max(np.maximum(np.maximum(built["x_lower"]-values,0.0),values-built["x_upper"]))))
    blocks=unpack(values,count)
    return QssSolution(np.asarray(mesh,dtype=float),blocks["offset"],blocks["heading"],blocks["speed"],blocks["acceleration"],blocks["curvature_rate"],float(result["f"]),str(stats["return_status"]),int(stats["iter_count"]),violation,blocks["mid_acceleration"],blocks["mid_curvature_rate"])


def project_seed(coarse: QssSolution, mesh) -> FloatArray:
    """Interpolate prior node and midpoint controls onto a new mesh."""
    mesh=np.asarray(mesh,dtype=float); source=np.asarray(coarse.distance,dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0.0):
        raise ValueError("target mesh must be finite, one-dimensional, and strictly increasing")
    if source.ndim != 1 or source.size < 2 or not np.all(np.isfinite(source)) or np.any(np.diff(source) <= 0.0):
        raise ValueError("solution distance must be finite, one-dimensional, and strictly increasing")
    count=mesh.size-1
    if source.size != coarse.offset.size: raise ValueError("solution distance does not match its node count")
    arrays=[np.interp(mesh,source,np.asarray(values,dtype=float)) for values in (coarse.offset,coarse.heading_error,coarse.speed,coarse.acceleration,coarse.curvature_rate)]
    def project_midpoint(values):
        if values is None:
            return np.zeros(count)
        values=np.asarray(values,dtype=float)
        if values.shape != (source.size - 1,) or not np.all(np.isfinite(values)):
            raise ValueError("solution midpoint controls must be finite and match its mesh")
        source_midpoints=.5*(source[:-1]+source[1:])
        target_midpoints=.5*(mesh[:-1]+mesh[1:])
        periodic_midpoints=np.r_[source_midpoints,source_midpoints[0]+float(coarse.distance[-1]-coarse.distance[0])]
        periodic_values=np.r_[values,values[0]]
        wrapped=source_midpoints[0]+np.mod(target_midpoints-source_midpoints[0],float(coarse.distance[-1]-coarse.distance[0]))
        return np.interp(wrapped,periodic_midpoints,periodic_values)
    mids_a=project_midpoint(coarse.mid_acceleration)
    mids_q=project_midpoint(coarse.mid_curvature_rate)
    return np.concatenate([*arrays,mids_a,mids_q])


def solve_ladder(track, corridor, surface, settings: QssSettings,
                 spacings: tuple[float, ...] = (5.0, 2.0, 1.0),
                 *, seed: FloatArray | None = None,
                 stage_callback=None) -> list[QssSolution]:
    """Solve coarse to fine, warm starting each stage from the one before.

    Each prior solution is projected directly onto the *next* stage mesh before
    it is used as that stage's seed. The first supplied seed is interpreted as
    a seed for the first spacing, consistent with ``solve_line``'s contract.
    If supplied, ``stage_callback(spacing, solution, wall_seconds)`` runs after
    each successful solve; the duration measures the ``solve_line`` call only.
    """
    if not spacings:
        raise ValueError("at least one mesh spacing is required")
    stages: list[QssSolution] = []
    current = seed
    for index, spacing in enumerate(spacings):
        mesh = build_mesh(track, spacing)
        started = time.perf_counter()
        solution = solve_line(track, corridor, surface, mesh, settings, seed=current)
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
            next_mesh = build_mesh(track, spacings[index + 1])
            current = project_seed(solution, next_mesh)
    return stages
