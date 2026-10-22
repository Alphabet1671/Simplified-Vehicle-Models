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


def _richardson_derivative(values: Callable[[float], float], station: float, step: float,
                           side: str | None = None) -> tuple[float, float, bool]:
    """Return an independently sampled derivative and conservative FD error bar.

    ``side=None`` selects central differences; ``left`` and ``right`` use the
    second-order one-sided formula.  The two Richardson levels deliberately
    use only evaluator speed values, never its reported rate or RHS.
    """
    def derivative(h: float) -> float:
        if side is None:
            return (values(station + h) - values(station - h)) / (2.0 * h)
        if side == "right":
            return (-3.0 * values(station) + 4.0 * values(station + h)
                    - values(station + 2.0 * h)) / (2.0 * h)
        return (3.0 * values(station) - 4.0 * values(station - h)
                + values(station - 2.0 * h)) / (2.0 * h)

    try:
        d0, d1, d2 = derivative(step), derivative(step / 2.0), derivative(step / 4.0)
    except (KeyError, TypeError, ValueError, FloatingPointError):
        return np.nan, np.inf, False
    if not np.all(np.isfinite((d0, d1, d2))):
        return np.nan, np.inf, False
    richardson0 = (4.0 * d1 - d0) / 3.0
    richardson1 = (4.0 * d2 - d1) / 3.0
    # Central symmetric differences have an O(h^4) Richardson remainder;
    # the one-sided formula retains O(h^3).  Do not advertise a three-level
    # estimate as a rigorous bound: use a deliberately conservative retained-
    # correction scale plus a convergence and roundoff check.
    roundoff = 64.0 * np.finfo(float).eps * max(1.0, abs(values(station))) / (step / 4.0)
    raw0, raw1 = abs(d1-d0) / 3.0, abs(d2-d1) / 3.0
    divisor = 15.0 if side is None else 7.0
    uncertainty = max(abs(richardson1-richardson0) / divisor, raw1, roundoff)
    stable = raw1 <= 1.5 * max(raw0, roundoff)
    return richardson1, uncertainty, stable


def _independent_speed_derivatives(evaluator, stations: np.ndarray, mesh: np.ndarray,
                                   base_step: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Mesh-aware FD derivatives from independently evaluated speed values.

    All selected adaptive stencil points are deduplicated and evaluated in one
    vector call.  Knots are canonicalized to their mesh coordinate before the
    stencil is selected, so a nearly identical station cannot accidentally use
    a cross-knot central difference.
    """
    mesh = np.asarray(mesh, dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0):
        raise ValueError("solution must provide a finite strictly increasing mesh")
    records: list[tuple[float, float, str | None, bool]] = []
    for requested in np.asarray(stations, dtype=float):
        close = np.flatnonzero(np.isclose(mesh, requested, rtol=0.0,
                                          atol=32*np.finfo(float).eps*max(1.0, abs(requested))))
        station = float(mesh[int(close[0])]) if close.size else float(requested)
        if close.size:
            node = int(close[0])
            sides = (("right", mesh[node+1]-station),) if node == 0 else (("left", station-mesh[node-1]),) if node == mesh.size-1 else (("left", station-mesh[node-1]), ("right", mesh[node+1]-station))
        else:
            element = int(np.searchsorted(mesh, station, side="right") - 1)
            if element < 0 or element >= mesh.size-1:
                records.append((station, np.nan, None, False)); continue
            left, right = station-mesh[element], mesh[element+1]-station
            sides = ((None, min(left, right)),) if min(left, right) >= 2.1*base_step else ((("right", right) if right >= left else ("left", left)),)
        for side, span in sides:
            step = min(base_step, .49*span)
            records.append((station, float(step), side, bool(np.isfinite(step) and step > 0.0)))

    points: list[float] = []
    for station, step, side, valid in records:
        if not valid: continue
        points.append(station)
        for h in (step, step/2.0, step/4.0):
            if side is None: points.extend((station-h, station+h))
            elif side == "left": points.extend((station, station-h, station-2.0*h))
            else: points.extend((station, station+h, station+2.0*h))
    points_array = np.unique(np.asarray(points, dtype=float))
    try:
        values = np.asarray(evaluator.evaluate(points_array)["speed"], dtype=float).reshape(-1)
    except (KeyError, TypeError, ValueError, FloatingPointError) as error:
        raise ValueError("unable to independently sample evaluator speed") from error
    if values.size != points_array.size or not np.all(np.isfinite(values)):
        raise ValueError("evaluator returned non-finite speed samples")
    cache = dict(zip(map(float, points_array), map(float, values)))
    def speed_at(value: float) -> float:
        try: return cache[value]
        except KeyError as error: raise ValueError("missing planned FD sample") from error

    output_s, derivatives, uncertainties, stable = [], [], [], []
    for station, step, side, valid in records:
        if valid: derivative, uncertainty, resolved = _richardson_derivative(speed_at, station, step, side)
        else: derivative, uncertainty, resolved = np.nan, np.inf, False
        output_s.append(station); derivatives.append(derivative)
        uncertainties.append(uncertainty); stable.append(resolved)
    return (np.asarray(output_s), np.asarray(derivatives), np.asarray(uncertainties),
            np.asarray(stable, dtype=bool))


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
    tolerances = dict(DEFAULT_TOLERANCES)
    tolerances.update(tolerance or {})
    trajectory = getattr(solution, "trajectory", None)
    evaluator = getattr(solution, "state_evaluator", None)
    missing = []
    if trajectory is None:
        missing.append("authoritative trajectory")
    if evaluator is None or not callable(getattr(evaluator, "evaluate", None)):
        missing.append("dense state evaluator")
    if missing:
        return {
            "checks": {"geometry_consistency": None, "dense_feasibility": None},
            "sampling_converged": False, "missing_evidence": missing,
            "diagnostics": [], "model_feasible": None,
            "physically_validated": False, "tolerances": tolerances,
        }
    if samples_per_element < 4 or maximum_refinements < 1:
        raise ValueError("samples_per_element must be >= 4 and maximum_refinements >= 1")
    history, final, converged = [], None, False
    try:
        mesh_for_sampling = np.asarray(getattr(solution, "distance", ()), dtype=float)
        if mesh_for_sampling.ndim != 1 or mesh_for_sampling.size < 2 or np.any(~np.isfinite(mesh_for_sampling)) or np.any(np.diff(mesh_for_sampling) <= 0):
            raise ValueError("solution must provide a finite strictly increasing mesh")
    except (TypeError, ValueError) as error:
        failed = {"name": "derivative_uncertainty", "passed": False,
                  "maximum_violation": None, "worst_station": None,
                  "unit": "(m/s)/m", "tolerance": tolerances["dynamics_mps_per_m"],
                  "reason": str(error), "stability_resolved": False}
        return {"checks": {"geometry_consistency": False, "dense_feasibility": False},
                "sampling_converged": False, "samples_per_element": samples_per_element,
                "refinements_used": 0, "sample_count": 0, "diagnostics": [failed],
                "sampling_history": [], "model_feasible": None, "surrogate_validated": False,
                "physically_validated": False, "missing_evidence": [], "tolerances": tolerances}
    for level in range(maximum_refinements):
        stations = _sample_stations(solution, trajectory, corridor,
                                    samples_per_element * 2**level)
        geo = trajectory.evaluate(stations)
        state = evaluator.evaluate(stations)
        offset, metric, curvature = map(np.asarray, (geo.offset, geo.metric, geo.curvature))
        speed, acceleration = map(np.asarray, (state["speed"], state["acceleration"]))
        diagnostics = []
        lower = _interpolate(stations, corridor.distance, corridor.minimum,
                             closed=bool(corridor.is_closed))
        upper = _interpolate(stations, corridor.distance, corridor.maximum,
                             closed=bool(corridor.is_closed))
        diagnostics.append(_diagnostic(
            "corridor", np.maximum(lower-offset, offset-upper), stations, "m",
            tolerances["corridor_m"],
        ))
        diagnostics.append(_diagnostic(
            "fold", float(getattr(settings, "fold_guard", 0.0))-np.asarray(geo.fold_factor),
            stations, "1", tolerances["fold"],
        ))
        diagnostics.append(_diagnostic(
            "speed_domain", np.where(speed >= max(float(getattr(settings, "minimum_speed", 0.0)), np.finfo(float).eps), 0.0, 1.0),
            stations, "m/s", tolerances["speed_mps"],
        ))
        if settings is not None:
            diagnostics.append(_diagnostic(
                "heading_limit", np.abs(np.asarray(geo.heading_error))-settings.maximum_heading,
                stations, "rad", tolerances["heading_rad"],
            ))
            diagnostics.append(_diagnostic(
                "heading_rate_limit", np.abs(np.asarray(geo.heading_error_s))-settings.maximum_curvature_rate,
                stations, "1/m", tolerances["curvature_per_m"],
            ))
        # Differentiate actual speed values independently.  Report FD sampling
        # uncertainty separately: it never relaxes a physical residual.
        step = max(1e-6, min(1e-3, float(stations[-1]-stations[0])*1e-5))
        mesh = np.asarray(getattr(solution, "distance", ()), dtype=float)
        try:
            interior, dv, derivative_uncertainty, derivative_stable = _independent_speed_derivatives(
                evaluator, stations, mesh, step)
        except (KeyError, TypeError, ValueError, FloatingPointError):
            interior = np.asarray(stations, dtype=float)
            dv = np.full(interior.shape, np.nan)
            derivative_uncertainty = np.full(interior.shape, np.inf)
            derivative_stable = np.zeros(interior.shape, dtype=bool)
        if interior.size:
            interior_state = evaluator.evaluate(interior)
            interior_geometry = trajectory.evaluate(interior)
            g = np.asarray(interior_geometry.metric)
            a = np.asarray(interior_state["acceleration"])
            v = np.asarray(interior_state["speed"])
            with np.errstate(divide="ignore", invalid="ignore"):
                dynamics = np.abs(dv-a*g/v)
                implied = np.abs(v*dv/g-a)
            diagnostics.append(_diagnostic("dynamics_defect", dynamics, interior,
                                           "(m/s)/m", tolerances["dynamics_mps_per_m"]))
            diagnostics.append(_diagnostic("implied_acceleration", implied,
                                           interior, "m/s^2", tolerances["acceleration_mps2"]))
            uncertainty_unit = _diagnostic("derivative_uncertainty", derivative_uncertainty,
                                           interior, "(m/s)/m", tolerances["dynamics_mps_per_m"])
            # An unresolved derivative is evidence failure, not a zero residual.
            uncertainty_unit["passed"] = bool(np.all(derivative_stable) and uncertainty_unit["passed"])
            uncertainty_unit["stability_resolved"] = bool(np.all(derivative_stable))
            diagnostics.append(uncertainty_unit)
            implied_uncertainty = derivative_uncertainty * np.abs(v / g)
            implied_uncertainty_unit = _diagnostic(
                "implied_acceleration_derivative_uncertainty", implied_uncertainty,
                interior, "m/s^2", tolerances["acceleration_mps2"],
            )
            implied_uncertainty_unit["passed"] = bool(np.all(derivative_stable) and implied_uncertainty_unit["passed"])
            implied_uncertainty_unit["stability_resolved"] = bool(np.all(derivative_stable))
            diagnostics.append(implied_uncertainty_unit)
            # This is an empirical residual-plus-error indicator, not a
            # certified truncation bound.  It still consumes the evidence
            # budget and never waives a physical residual.
            diagnostics.append(_diagnostic(
                "dynamics_defect_upper_bound", dynamics + derivative_uncertainty,
                interior, "(m/s)/m", tolerances["dynamics_mps_per_m"],
            ))
            diagnostics.append(_diagnostic(
                "implied_acceleration_upper_bound", implied + implied_uncertainty,
                interior, "m/s^2", tolerances["acceleration_mps2"],
            ))
            # Geometry stencils still need two centred spans; endpoint speed
            # derivatives above are valid one-sided evidence but cannot feed
            # these separate geometry checks.
            # Geometry source/control joins can have insufficient smoothness
            # for central curvature/rate stencils.  Only FD-check contained
            # stencils; analytic physical constraints remain sampled everywhere.
            geometry_knots = np.unique(np.concatenate((
                np.asarray(trajectory.reference.source_station, dtype=float),
                np.asarray(trajectory.spline.control_distance, dtype=float),
            )))
            knot_index = np.searchsorted(geometry_knots, interior)
            nearest_left = np.where(knot_index > 0, geometry_knots[np.maximum(knot_index-1, 0)], -np.inf)
            nearest_right = np.where(knot_index < geometry_knots.size, geometry_knots[np.minimum(knot_index, geometry_knots.size-1)], np.inf)
            contained_geometry = np.minimum(interior-nearest_left, nearest_right-interior) > 2.1*step
            geometry_stations = np.unique(interior[(interior > stations[0]+2*step) &
                (interior < stations[-1]-2*step) & contained_geometry])
            if geometry_stations.size == 0:
                raise ValueError("no contained geometry finite-difference stencils")
            geometry = trajectory.evaluate(geometry_stations)
            gl = trajectory.evaluate(geometry_stations-step)
            gr = trajectory.evaluate(geometry_stations+step)
            dx = (np.asarray(gr.north)-np.asarray(gl.north)) / (2*step)
            dy = (np.asarray(gr.east)-np.asarray(gl.east)) / (2*step)
            fd_heading = np.arctan2(dy, dx)
            heading = np.asarray(geometry.heading)
            # Local central tangent-heading derivative avoids nonuniform source/
            # control-knot spacing in a global gradient calculation.
            tangent_left = trajectory.evaluate(geometry_stations-step)
            tangent_right = trajectory.evaluate(geometry_stations+step)
            heading_left = np.arctan2(
                np.asarray(tangent_left.east)-np.asarray(trajectory.evaluate(geometry_stations-2*step).east),
                np.asarray(tangent_left.north)-np.asarray(trajectory.evaluate(geometry_stations-2*step).north),
            )
            heading_right = np.arctan2(
                np.asarray(trajectory.evaluate(geometry_stations+2*step).east)-np.asarray(tangent_right.east),
                np.asarray(trajectory.evaluate(geometry_stations+2*step).north)-np.asarray(tangent_right.north),
            )
            # Each chord heading is centred at s±1.5*step, so their spacing is 3*step.
            g_geo = np.asarray(geometry.metric)
            fd_curvature = np.angle(np.exp(1j*(heading_right-heading_left))) / (3*step*g_geo)
            diagnostics.append(_diagnostic("xy_metric", np.abs(np.hypot(dx,dy)-g_geo),
                                            geometry_stations, "m/m", tolerances["geometry_m"]))
            diagnostics.append(_diagnostic("xy_heading", np.abs(np.angle(np.exp(1j*(fd_heading-heading)))),
                                            geometry_stations, "rad", tolerances["heading_rad"]))
            diagnostics.append(_diagnostic("xy_curvature", np.abs(fd_curvature-np.asarray(geometry.curvature)),
                                            geometry_stations, "1/m", tolerances["curvature_per_m"]))
            # Do not trust analytic heading_error_s: independently differentiate
            # the exported heading-error field at off-grid stations.
            heading_error_left = np.asarray(trajectory.evaluate(geometry_stations-step).heading_error)
            heading_error_right = np.asarray(trajectory.evaluate(geometry_stations+step).heading_error)
            fd_heading_rate = np.angle(np.exp(1j * (heading_error_right-heading_error_left))) / (2*step)
            diagnostics.append(_diagnostic(
                "xy_heading_rate", np.abs(fd_heading_rate-np.asarray(geometry.heading_error_s)),
                geometry_stations, "1/m", tolerances["curvature_per_m"],
            ))
            if settings is not None:
                diagnostics.append(_diagnostic(
                    "xy_heading_rate_limit", np.abs(fd_heading_rate)-settings.maximum_curvature_rate,
                    geometry_stations, "1/m", tolerances["curvature_per_m"],
                ))
        else:
            diagnostics.append({"name": "xy_geometry", "passed": False,
                                "maximum_violation": None, "worst_station": None,
                                "unit": "", "tolerance": None})
        try:
            if getattr(surface, "representation", None) == "signed_transient_normalized_v1":
                bounds = [surface.limits(float(v), float(k)) for v, k in zip(speed, curvature)]
                amin = np.asarray([item[0] for item in bounds])
                amax = np.asarray([item[1] for item in bounds])
                ceiling = np.asarray([surface.domain_speed(float(k)) for k in curvature])
            else:
                amax = _bilinear(surface, speed, curvature, "acceleration_max")
                amin = _bilinear(surface, speed, curvature, "acceleration_min")
                ceiling = _interpolate(curvature, surface.curvature, surface.speed_ceiling,
                                   closed=False)
            diagnostics.append(_diagnostic("acceleration_max", acceleration-amax,
                                           stations, "m/s^2", tolerances["acceleration_mps2"]))
            diagnostics.append(_diagnostic("acceleration_min", amin-acceleration,
                                           stations, "m/s^2", tolerances["acceleration_mps2"]))
            diagnostics.append(_diagnostic("speed_ceiling", speed-ceiling,
                                           stations, "m/s", tolerances["speed_mps"]))
        except (AttributeError, ValueError) as error:
            diagnostics.append({"name": "capability_domain", "passed": False,
                                "maximum_violation": None, "worst_station": None,
                                "unit": "", "tolerance": None, "reason": str(error)})
        # Independent trapezoid integral of physical dt/ds = metric / speed.
        if np.all(speed > 0):
            dt_ds = metric / speed
            time_value = float(np.sum(np.diff(stations) * (dt_ds[:-1] + dt_ds[1:]) / 2.0))
        else:
            time_value = np.inf
        reported_time = float(solution.lap_time)
        relative_time_error = abs(time_value-reported_time) / max(1.0, abs(reported_time))
        diagnostics.append(_diagnostic("lap_time", np.full(stations.shape, relative_time_error),
                                       stations, "relative", tolerances["time_relative"]))
        final = {"stations": stations, "diagnostics": diagnostics, "time": time_value}
        history.append({item["name"]: item["maximum_violation"] for item in diagnostics})
        if len(history) > 1 and all((history[-1].get(name) is not None and history[-2].get(name) is not None and abs(history[-1][name]-history[-2][name]) <= tolerances["sampling_stability"]) for name in history[-1]):
            converged = True
            break
    assert final is not None
    dense_ok = converged and all(item["passed"] for item in final["diagnostics"])
    callback = equilibrium_callback
    if callback is None and vehicle is not None:
        callback = lambda v,k,a: evaluate_equilibrium(vehicle, v, k, a)
    model_feasible = None
    if callback is not None:
        try:
            for v,k,a in zip(speed,curvature,acceleration):
                result = callback(float(v),float(k),float(a))
                if isinstance(result, (bool, np.bool_)):
                    feasible = bool(result)
                elif hasattr(result, "is_feasible") and isinstance(result.is_feasible, (bool, np.bool_)):
                    feasible = bool(result.is_feasible)
                else:
                    raise ValueError("live equilibrium callback returned no boolean feasibility")
                if not feasible:
                    raise ValueError("live equilibrium infeasible")
            model_feasible = True
        except Exception: model_feasible = False
    if model_feasible is False: dense_ok = False
    geometry_ok = all(item["passed"] for item in final["diagnostics"] if item["name"] in {"xy_metric", "xy_heading", "xy_curvature", "xy_heading_rate", "xy_heading_rate_limit", "fold"})
    return {"checks":{"geometry_consistency":geometry_ok,"dense_feasibility":dense_ok}, "sampling_converged":converged, "samples_per_element":samples_per_element,"refinements_used":level+1,"sample_count":int(final["stations"].size),"diagnostics":final["diagnostics"],"sampling_history":history,"model_feasible":model_feasible,"surrogate_validated":dense_ok and model_feasible is None,"physically_validated":False,"missing_evidence":[],"tolerances":tolerances}
