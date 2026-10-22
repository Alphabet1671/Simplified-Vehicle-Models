"""Independent, fail-closed dense validation for minimum-time line results.

This module intentionally has no CasADi dependency.  It checks the exported
analytic trajectory and Hermite--Simpson state evaluator, rather than NLP
constraint values or solver residuals.  Finite sampling is evidence, not a
continuous-feasibility certificate.
"""
from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

from simplified_vehicle_models.formulations.point_mass import evaluate_equilibrium

import numpy as np


REQUIRED_NUMERICAL_CHECKS = (
    "geometry_consistency", "dense_feasibility", "mesh_convergence",
    "basis_convergence", "matched_baseline", "replay_agreement",
)

DEFAULT_TOLERANCES = {
    "geometry_m": 2.0e-5,
    "heading_rad": 2.0e-5,
    "corridor_m": 1.0e-6,
    "fold": 1.0e-8,
    "speed_mps": 1.0e-8,
    "acceleration_mps2": 1.0e-6,
    "dynamics_mps_per_m": 1.0e-6,
    "curvature_per_m": 2.0e-5,
    "time_relative": 1.0e-5,
    "sampling_stability": 1.0e-7,
}


def validation_status(
    solver_succeeded: object, checks: Mapping[str, object] | None,
    required_checks: Sequence[str] = REQUIRED_NUMERICAL_CHECKS,
) -> str:
    """Return the strict tier status without coercing absent evidence to pass."""
    if solver_succeeded is not True:
        return "solver_failed"
    if not required_checks or not checks:
        return "incomplete"
    values = [checks.get(name) for name in required_checks]
    if any(value is False for value in values):
        return "failed"
    if any(value is not True for value in values):
        return "incomplete"
    return "numerically_validated"


def strict_validation(
    solver_succeeded: object, checks: Mapping[str, object] | None,
    required_checks: Sequence[str] = REQUIRED_NUMERICAL_CHECKS,
) -> dict[str, Any]:
    """Build serializable strict-tier fields shared by runner and comparison."""
    status = validation_status(solver_succeeded, checks, required_checks)
    checks = dict(checks or {})
    return {
        "solver_succeeded": solver_succeeded is True,
        "replay_feasible": checks.get("replay_agreement") is True,
        "numerically_validated": status == "numerically_validated",
        # Measurement evidence is deliberately out of scope for this validator.
        "physically_validated": False,
        "accepted": status == "numerically_validated",
        "validation_status": status,
        "required_checks": list(required_checks),
        "failed_checks": [name for name in required_checks if checks.get(name) is False],
        "unavailable_checks": [name for name in required_checks if checks.get(name) is not True and checks.get(name) is not False],
    }


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


def _interpolate(station, axis, values, *, closed: bool) -> np.ndarray:
    station = np.asarray(station, dtype=float)
    axis, values = np.asarray(axis, dtype=float), np.asarray(values, dtype=float)
    if axis.ndim != 1 or values.shape != axis.shape or axis.size < 2:
        raise ValueError("interpolation data must be matching one-dimensional arrays")
    if not np.all(np.isfinite(axis)) or not np.all(np.isfinite(values)) or np.any(np.diff(axis) <= 0):
        raise ValueError("interpolation domain must be finite and strictly increasing")
    if closed:
        period = axis[-1] - axis[0]
        if period <= 0.0:
            raise ValueError("closed interpolation has nonpositive period")
        station = np.mod(station - axis[0], period) + axis[0]
    elif np.any(station < axis[0]) or np.any(station > axis[-1]):
        raise ValueError("station outside interpolation domain")
    return np.interp(station, axis, values)


def _bilinear(surface, speed, curvature, attribute: str) -> np.ndarray:
    v, k = np.asarray(speed, dtype=float), np.asarray(curvature, dtype=float)
    vg, kg = np.asarray(surface.speed, dtype=float), np.asarray(surface.curvature, dtype=float)
    table = np.asarray(getattr(surface, attribute), dtype=float)
    if table.shape != (vg.size, kg.size) or np.any(v < vg[0]) or np.any(v > vg[-1]) or np.any(k < kg[0]) or np.any(k > kg[-1]):
        raise ValueError("capability query outside supported surface domain")
    iv = np.clip(np.searchsorted(vg, v, side="right") - 1, 0, vg.size - 2)
    ik = np.clip(np.searchsorted(kg, k, side="right") - 1, 0, kg.size - 2)
    tv = (v - vg[iv]) / (vg[iv + 1] - vg[iv])
    tk = (k - kg[ik]) / (kg[ik + 1] - kg[ik])
    return ((1-tv)*(1-tk)*table[iv, ik] + tv*(1-tk)*table[iv+1, ik]
            + (1-tv)*tk*table[iv, ik+1] + tv*tk*table[iv+1, ik+1])


def _diagnostic(name: str, violation: np.ndarray, station: np.ndarray, unit: str,
                tolerance: float) -> dict[str, Any]:
    violation = np.asarray(violation, dtype=float)
    if violation.size == 0 or not np.all(np.isfinite(violation)):
        return {"name": name, "passed": False, "maximum_violation": None,
                "worst_station": None, "unit": unit, "tolerance": tolerance}
    index = int(np.argmax(violation))
    maximum = float(violation[index])
    return {"name": name, "passed": maximum <= tolerance,
            "maximum_violation": maximum, "worst_station": float(station[index]),
            "unit": unit, "tolerance": tolerance}


def _sample_stations(solution, trajectory, corridor, samples_per_element: int) -> np.ndarray:
    mesh = np.asarray(getattr(solution, "distance", ()), dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or np.any(np.diff(mesh) <= 0):
        raise ValueError("solution must provide a strictly increasing mesh")
    pieces = [mesh, np.asarray(corridor.distance, dtype=float)]
    reference = trajectory.reference
    pieces.extend((np.asarray(reference.source_station, dtype=float),
                   np.asarray(trajectory.spline.control_distance, dtype=float)))
    # All element interiors, including quarters used by the OCP and additional
    # off-collocation samples.  Sampling grows monotonically with refinement.
    fractions = np.arange(1, samples_per_element, dtype=float) / samples_per_element
    pieces.append((mesh[:-1, None] + np.diff(mesh)[:, None] * fractions).ravel())
    result = np.unique(np.concatenate(pieces))
    start, end = float(mesh[0]), float(mesh[-1])
    return result[(result >= start) & (result <= end)]


def validate_line(solution, corridor, surface, settings=None, *, vehicle=None,
                  equilibrium_callback: Callable[[float, float, float], Any] | None = None,
                  samples_per_element: int = 8, maximum_refinements: int = 4,
                  tolerance: Mapping[str, float] | None = None) -> dict[str, Any]:
    """Independently inspect exported analytic geometry and reconstructed state.

    A callback must return ``False`` or an object with ``is_feasible=False`` to
    fail model feasibility.  With a point-mass vehicle the repository's original
    ``evaluate_equilibrium`` is used directly; no capability-table result is
    relabelled as live-model validation.
    """
    tolerances = dict(DEFAULT_TOLERANCES); tolerances.update(tolerance or {})
    trajectory, evaluator = getattr(solution, "trajectory", None), getattr(solution, "state_evaluator", None)
    missing = (["authoritative trajectory"] if trajectory is None else []) + (["dense state evaluator"] if evaluator is None or not callable(getattr(evaluator, "evaluate", None)) else [])
    if missing:
        return {"checks": {"geometry_consistency": None, "dense_feasibility": None}, "sampling_converged": False, "missing_evidence": missing, "diagnostics": [], "model_feasible": None, "physically_validated": False, "tolerances": tolerances}
    if samples_per_element < 4 or maximum_refinements < 1:
        raise ValueError("samples_per_element must be >= 4 and maximum_refinements >= 1")
    history, final = [], None
    for level in range(maximum_refinements):
        stations = _sample_stations(solution, trajectory, corridor, samples_per_element * 2**level)
        geo, state = trajectory.evaluate(stations), evaluator.evaluate(stations)
        offset, metric, curvature = map(np.asarray, (geo.offset, geo.metric, geo.curvature))
        speed, acceleration = map(np.asarray, (state["speed"], state["acceleration"]))
        diagnostics = []
        lower = _interpolate(stations, corridor.distance, corridor.minimum, closed=bool(corridor.is_closed)); upper = _interpolate(stations, corridor.distance, corridor.maximum, closed=bool(corridor.is_closed))
        diagnostics.extend((_diagnostic("corridor", np.maximum(lower-offset, offset-upper), stations, "m", tolerances["corridor_m"]), _diagnostic("fold", float(getattr(settings, "fold_guard", 0.0))-np.asarray(geo.fold_factor), stations, "1", tolerances["fold"]), _diagnostic("speed_domain", np.where(speed >= max(float(getattr(settings, "minimum_speed", 0.0)), np.finfo(float).eps), 0.0, 1.0), stations, "m/s", tolerances["speed_mps"])))
        if settings is not None:
            diagnostics.extend((_diagnostic("heading_limit", np.abs(np.asarray(geo.heading_error))-settings.maximum_heading, stations, "rad", tolerances["heading_rad"]), _diagnostic("heading_rate_limit", np.abs(np.asarray(geo.heading_error_s))-settings.maximum_curvature_rate, stations, "1/m", tolerances["curvature_per_m"])))
        # Differentiate speed values independently.  Never trust evaluator RHS.
        step = max(1e-6, min(1e-3, float(stations[-1]-stations[0])*1e-5)); interior = stations[(stations > stations[0]+step) & (stations < stations[-1]-step)]
        if interior.size:
            left, right = evaluator.evaluate(interior-step), evaluator.evaluate(interior+step)
            dv = (np.asarray(right["speed"])-np.asarray(left["speed"]))/ (2*step)
            g = np.asarray(trajectory.evaluate(interior).metric); a = np.asarray(evaluator.evaluate(interior)["acceleration"]); v = np.asarray(evaluator.evaluate(interior)["speed"])
            diagnostics.extend((_diagnostic("dynamics_defect", np.abs(dv-a*g/v), interior, "(m/s)/m", tolerances["dynamics_mps_per_m"]), _diagnostic("implied_acceleration", np.abs(v*dv/g-a), interior, "m/s^2", tolerances["acceleration_mps2"])))
            gl, gr = trajectory.evaluate(interior-step), trajectory.evaluate(interior+step)
            dx, dy = (np.asarray(gr.north)-np.asarray(gl.north))/(2*step), (np.asarray(gr.east)-np.asarray(gl.east))/(2*step)
            fd_heading = np.arctan2(dy, dx); heading = np.asarray(trajectory.evaluate(interior).heading)
            fd_curvature = np.gradient(np.unwrap(fd_heading), interior) / np.asarray(trajectory.evaluate(interior).metric)
            diagnostics.extend((_diagnostic("xy_metric", np.abs(np.hypot(dx,dy)-np.asarray(trajectory.evaluate(interior).metric)), interior, "m/m", tolerances["geometry_m"]), _diagnostic("xy_heading", np.abs(np.angle(np.exp(1j*(fd_heading-heading)))), interior, "rad", tolerances["heading_rad"]), _diagnostic("xy_curvature", np.abs(fd_curvature-np.asarray(trajectory.evaluate(interior).curvature)), interior, "1/m", tolerances["curvature_per_m"])))
        else: diagnostics.append({"name":"xy_geometry", "passed":False, "maximum_violation":None, "worst_station":None, "unit":"", "tolerance":None})
        try:
            amax, amin = _bilinear(surface,speed,curvature,"acceleration_max"), _bilinear(surface,speed,curvature,"acceleration_min"); ceiling = _interpolate(curvature,surface.curvature,surface.speed_ceiling,closed=False)
            diagnostics.extend((_diagnostic("acceleration_max", acceleration-amax, stations,"m/s^2",tolerances["acceleration_mps2"]), _diagnostic("acceleration_min",amin-acceleration,stations,"m/s^2",tolerances["acceleration_mps2"]), _diagnostic("speed_ceiling",speed-ceiling,stations,"m/s",tolerances["speed_mps"])))
        except (AttributeError, ValueError) as error: diagnostics.append({"name":"capability_domain","passed":False,"maximum_violation":None,"worst_station":None,"unit":"","tolerance":None,"reason":str(error)})
        # Independent trapezoid integral of physical dt/ds = metric / speed.
        time_value = float(np.sum(np.diff(stations) * ((metric/speed)[:-1] + (metric/speed)[1:]) / 2.0)) if np.all(speed > 0) else np.inf
        diagnostics.append(_diagnostic("lap_time", np.full(stations.shape, abs(time_value-float(solution.lap_time))/max(1.0,abs(float(solution.lap_time))),), stations, "relative", tolerances["time_relative"]))
        final = {"stations":stations, "diagnostics":diagnostics, "time":time_value}; history.append({item["name"]:item["maximum_violation"] for item in diagnostics})
        if len(history) > 1 and all((history[-1].get(name) is not None and history[-2].get(name) is not None and abs(history[-1][name]-history[-2][name]) <= tolerances["sampling_stability"]) for name in history[-1]): break
    assert final is not None
    converged = len(history) > 1 and level < maximum_refinements
    dense_ok = converged and all(item["passed"] for item in final["diagnostics"])
    callback = equilibrium_callback
    if callback is None and vehicle is not None:
        callback = lambda v,k,a: evaluate_equilibrium(vehicle, v, k, a)
    model_feasible = None
    if callback is not None:
        try:
            for v,k,a in zip(speed,curvature,acceleration):
                result = callback(float(v),float(k),float(a))
                if result is False or getattr(result, "is_feasible", True) is False: raise ValueError("live equilibrium infeasible")
            model_feasible = True
        except Exception: model_feasible = False
    if model_feasible is False: dense_ok = False
    geometry_ok = all(item["passed"] for item in final["diagnostics"] if item["name"] in {"xy_metric","xy_heading","xy_curvature","fold"})
    return {"checks":{"geometry_consistency":geometry_ok,"dense_feasibility":dense_ok}, "sampling_converged":converged, "samples_per_element":samples_per_element,"refinements_used":level+1,"sample_count":int(final["stations"].size),"diagnostics":final["diagnostics"],"sampling_history":history,"model_feasible":model_feasible,"surrogate_validated":dense_ok and model_feasible is None,"physically_validated":False,"missing_evidence":[],"tolerances":tolerances}
