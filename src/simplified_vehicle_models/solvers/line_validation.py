"""Independent, fail-closed dense validation for minimum-time line results.

This module intentionally has no CasADi dependency.  It checks the exported
analytic trajectory and Hermite--Simpson state evaluator, rather than NLP
constraint values or solver residuals.  Finite sampling is evidence, not a
continuous-feasibility certificate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

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
                  samples_per_element: int = 8, maximum_refinements: int = 4,
                  tolerance: Mapping[str, float] | None = None) -> dict[str, Any]:
    """Independently inspect a spline solution's dense geometry/state/control.

    ``vehicle`` may expose ``evaluate_equilibrium(speed, curvature, acceleration)``.
    Its successful evaluation establishes only *model* feasibility, never
    empirical physical validation.  Missing evaluator/trajectory returns an
    incomplete report; no nodal reconstruction is substituted.
    """
    tolerances = dict(DEFAULT_TOLERANCES)
    if tolerance:
        tolerances.update(tolerance)
    trajectory = getattr(solution, "trajectory", None)
    evaluator = getattr(solution, "state_evaluator", None)
    missing = []
    if trajectory is None:
        missing.append("authoritative trajectory")
    if evaluator is None or not callable(getattr(evaluator, "evaluate", None)):
        missing.append("dense state evaluator")
    if missing:
        return {"checks": {"geometry_consistency": None, "dense_feasibility": None},
                "sampling_converged": False, "missing_evidence": missing,
                "diagnostics": [], "model_feasible": None,
                "physically_validated": False, "tolerances": tolerances}
    if samples_per_element < 4 or maximum_refinements < 1:
        raise ValueError("samples_per_element must be >= 4 and maximum_refinements >= 1")

    prior = None
    final: dict[str, Any] | None = None
    converged = False
    for level in range(maximum_refinements):
        stations = _sample_stations(solution, trajectory, corridor, samples_per_element * (2 ** level))
        geometry = trajectory.evaluate(stations)
        state = evaluator.evaluate(stations)
        offset, metric = np.asarray(geometry.offset), np.asarray(geometry.metric)
        curvature = np.asarray(geometry.curvature)
        speed, acceleration = np.asarray(state["speed"]), np.asarray(state["acceleration"])
        speed_rate, rhs = np.asarray(state["speed_rate"]), np.asarray(state["rhs_speed_rate"])
        diagnostics: list[dict[str, Any]] = []
        lower = _interpolate(stations, corridor.distance, corridor.minimum, closed=bool(corridor.is_closed))
        upper = _interpolate(stations, corridor.distance, corridor.maximum, closed=bool(corridor.is_closed))
        diagnostics.append(_diagnostic("corridor", np.maximum(lower-offset, offset-upper), stations, "m", tolerances["corridor_m"]))
        diagnostics.append(_diagnostic("fold", -np.asarray(geometry.fold_factor), stations, "1", tolerances["fold"]))
        diagnostics.append(_diagnostic("speed_domain", np.where(speed > 0.0, 0.0, 1.0), stations, "m/s", tolerances["speed_mps"]))
        diagnostics.append(_diagnostic("dynamics_defect", np.abs(speed_rate-rhs), stations, "(m/s)/m", tolerances["dynamics_mps_per_m"]))
        try:
            amax = _bilinear(surface, speed, curvature, "acceleration_max")
            amin = _bilinear(surface, speed, curvature, "acceleration_min")
            ceiling = _interpolate(curvature, surface.curvature, surface.speed_ceiling, closed=False)
            diagnostics.extend((
                _diagnostic("acceleration_max", acceleration-amax, stations, "m/s^2", tolerances["acceleration_mps2"]),
                _diagnostic("acceleration_min", amin-acceleration, stations, "m/s^2", tolerances["acceleration_mps2"]),
                _diagnostic("speed_ceiling", speed-ceiling, stations, "m/s", tolerances["speed_mps"]),
            ))
        except (AttributeError, ValueError) as error:
            diagnostics.append({"name": "capability_domain", "passed": False, "maximum_violation": None,
                                "worst_station": None, "unit": "", "tolerance": None, "reason": str(error)})
        # Finite-difference XY tangent agrees with analytic trajectory heading/metric.
        span = float(stations[-1] - stations[0]); step = max(1e-6, min(1e-3, span * 1e-5))
        interior = stations[(stations > stations[0] + step) & (stations < stations[-1] - step)]
        if interior.size:
            left, right = trajectory.evaluate(interior-step), trajectory.evaluate(interior+step)
            dx = (np.asarray(right.north)-np.asarray(left.north))/(2*step)
            dy = (np.asarray(right.east)-np.asarray(left.east))/(2*step)
            fd_metric = np.hypot(dx, dy)
            fd_heading = np.arctan2(dy, dx)
            analytic_heading = np.asarray(trajectory.evaluate(interior).heading)
            heading_error = np.abs(np.angle(np.exp(1j*(fd_heading-analytic_heading))))
            diagnostics.extend((
                _diagnostic("xy_metric", np.abs(fd_metric-np.asarray(trajectory.evaluate(interior).metric)), interior, "m/m", tolerances["geometry_m"]),
                _diagnostic("xy_heading", heading_error, interior, "rad", tolerances["heading_rad"]),
            ))
        else:
            diagnostics.append({"name": "xy_geometry", "passed": False, "maximum_violation": None,
                                "worst_station": None, "unit": "", "tolerance": None})
        maximum = max((item["maximum_violation"] for item in diagnostics if item["maximum_violation"] is not None), default=np.inf)
        final = {"stations": stations, "diagnostics": diagnostics, "maximum": maximum}
        if prior is not None and abs(maximum-prior) <= tolerances["sampling_stability"]:
            converged = True
            break
        prior = maximum
    assert final is not None
    dense_ok = converged and all(item["passed"] for item in final["diagnostics"])
    model_feasible = None
    if vehicle is not None:
        callback = getattr(vehicle, "evaluate_equilibrium", None)
        if not callable(callback):
            model_feasible = False
        else:
            try:
                for v, k, a in zip(speed, curvature, acceleration):
                    result = callback(float(v), float(k), float(a))
                    if result is False:
                        raise ValueError("evaluate_equilibrium returned false")
                model_feasible = True
            except Exception:
                model_feasible = False
    geometry_ok = all(item["passed"] for item in final["diagnostics"] if item["name"] in {"xy_metric", "xy_heading", "fold"})
    return {"checks": {"geometry_consistency": geometry_ok, "dense_feasibility": dense_ok},
            "sampling_converged": converged, "samples_per_element": samples_per_element,
            "refinements_used": level + 1, "sample_count": int(final["stations"].size),
            "diagnostics": final["diagnostics"], "model_feasible": model_feasible,
            "surrogate_validated": dense_ok and model_feasible is None,
            "physically_validated": False, "missing_evidence": [], "tolerances": tolerances}
