"""Actual fixed-straight, nonnegative-acceleration standing-start OCP.

This restricted transcription neither optimizes a curved line nor certifies
continuous feasibility between its finite live and held-out limit samples.
Wall-time checks are cooperative; callers needing a hard deadline must enforce
an outer timeout.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import numbers
import time
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class StandingStraightSettings:
    intervals: int = 12
    maximum_iterations: int = 120
    maximum_wall_seconds: float = 30.0
    maximum_limit_queries: int = 20000
    tolerance: float = 1e-7
    minimum_interval_seconds: float = 1e-5
    maximum_interval_seconds: float = 10.0
    heldout_fractions: int = 17
    transcription_mode: str = "legacy_normalized_fd_v1"


@dataclass(frozen=True)
class StandingStraightResult:
    time: np.ndarray
    distance: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    total_time: float
    fixed_straight_only: bool = True
    joint_line_optimized: bool = False
    accepted: bool = False
    event_qualified: bool = False
    U3_qualified: bool = False
    numerically_qualified: bool = False
    physical_feasibility_certified: bool = False
    global_minimum_proven: bool = False

    def __post_init__(self):
        for name in ("time", "distance", "speed", "acceleration"):
            arr = np.array(getattr(self, name), dtype=float, copy=True)
            arr.setflags(write=False)
            object.__setattr__(self, name, arr)


def _jsonsafe(x):
    if isinstance(x, np.ndarray): return _jsonsafe(x.tolist())
    if isinstance(x, np.generic): return _jsonsafe(x.item())
    if isinstance(x, float): return x if math.isfinite(x) else None
    if isinstance(x, dict): return {str(k): _jsonsafe(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)): return [_jsonsafe(v) for v in x]
    if x is None or isinstance(x, (str, int, bool)): return x
    return str(x)


def solve_standing_straight(vehicle, distance, *, settings=None, acceleration_limit=None, initial_guess=None):
    """Solve the restricted straight-line problem; success is not event qualification."""
    started = time.monotonic()
    report = {"accepted": False, "optimizer_converged": False, "solver_reported_success": False,
              "sampled_feasible": False, "heldout_sampling_complete": False,
              "heldout_feasible": False, "event_qualified": False, "U3_qualified": False,
              "numerically_qualified": False, "physical_feasibility_certified": False,
              "global_minimum_proven": False, "joint_line_optimized": False,
              "geometry": "fixed straight, nonnegative longitudinal control, rest-to-free-finish",
              "failure": None, "limit_queries": 0, "cache_hits": 0, "derivative_limit_queries": 0, "model_identity_known": False}

    def finish_failure(reason, *, budget=False):
        report["failure"] = str(reason)
        try:
            elapsed = time.monotonic() - started
        except Exception:
            elapsed = None
        report["wall_seconds"] = elapsed
        if budget:
            report["optimizer_converged"] = False
            report["sampled_feasible"] = False
            report["heldout_sampling_complete"] = False
            report["heldout_feasible"] = False
            report["accepted"] = False
        try:
            safe = _jsonsafe(report)
            json.dumps(safe, allow_nan=False)
        except Exception as exc:
            safe = {"accepted": False, "optimizer_converged": False,
                    "failure": "report_serialization_error", "exception_type": type(exc).__name__,
                    "exception_message": str(exc)}
        return None, safe

    if settings is None: settings = StandingStraightSettings()
    if type(settings) is not StandingStraightSettings: return finish_failure("invalid_settings_type")
    mode = settings.transcription_mode
    if type(mode) is not str or mode not in ("legacy_normalized_fd_v1", "physical_units_chain_v1"):
        return finish_failure("invalid_settings_transcription_mode")
    physical_mode = mode == "physical_units_chain_v1"
    for name in ("intervals", "maximum_iterations", "maximum_limit_queries", "heldout_fractions"):
        value = getattr(settings, name)
        minimum = 2 if name == "intervals" else 3 if name == "heldout_fractions" else 1
        if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < minimum:
            return finish_failure("invalid_settings_" + name)
    for name in ("maximum_wall_seconds", "tolerance", "minimum_interval_seconds", "maximum_interval_seconds"):
        value = getattr(settings, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value) or value <= 0:
            return finish_failure("invalid_settings_" + name)
    report["settings"] = {name: getattr(settings, name) for name in settings.__dataclass_fields__}
    if settings.minimum_interval_seconds > settings.maximum_interval_seconds:
        return finish_failure("invalid_interval_bounds")
    if isinstance(distance, bool) or not isinstance(distance, numbers.Real) or not math.isfinite(distance) or distance <= 0:
        return finish_failure("invalid_distance")
    distance = float(distance); n = int(settings.intervals)
    seed = None
    if initial_guess is not None:
        try:
            source = np.asarray(initial_guess, dtype=object)
            raw = np.asarray(initial_guess)
            if (source.shape != (2*n,) or any(isinstance(v, (bool, np.bool_)) for v in source.flat)
                    or raw.shape != (2*n,) or raw.dtype.kind not in "iuf"):
                return finish_failure("invalid_initial_guess")
            seed = np.array(raw, dtype=np.float64, copy=True)
            if not np.all(np.isfinite(seed)) or np.any(seed[:n] < settings.minimum_interval_seconds) or np.any(seed[:n] > settings.maximum_interval_seconds) or np.any(seed[n:] < 0):
                return finish_failure("invalid_initial_guess")
            import hashlib
            report["seed_source"] = "provided"
            report["initial_guess_sha256"] = hashlib.sha256(seed.astype("<f8", copy=False).tobytes()).hexdigest()
            distance_guess = 0.0; speed_guess = 0.0
            for h, a in zip(seed[:n], seed[n:]):
                distance_guess += speed_guess*h + .5*a*h*h
                speed_guess += a*h
            report["initial_guess_finish_distance_residual_m"] = float(distance_guess - float(distance)) if isinstance(distance, numbers.Real) and math.isfinite(distance) else None
        except Exception:
            return finish_failure("invalid_initial_guess")
    else:
        report["seed_source"] = "legacy_default"
    calls = 0; cache = {}; x = None

    def budget_check(where):
        if time.monotonic() - started >= settings.maximum_wall_seconds:
            raise TimeoutError("wall budget " + where)

    try:
        if acceleration_limit is None:
            from simplified_vehicle_models.formulations.point_mass import maximum_acceleration_limit
            callback = maximum_acceleration_limit
        else:
            callback = acceleration_limit
        if not callable(callback): return finish_failure("invalid_callback")

        def cap(speed, *, derivative=False):
            nonlocal calls
            budget_check("before callback/cache")
            key = (float(speed), 0.0)
            if key in cache:
                report["cache_hits"] += 1
                budget_check("after cache")
                return cache[key]
            if calls >= settings.maximum_limit_queries: raise RuntimeError("limit query budget")
            calls += 1; report["limit_queries"] = calls
            if derivative: report["derivative_limit_queries"] += 1
            value = callback(vehicle, float(speed), 0.0)
            budget_check("after callback")
            if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value) or value < 0:
                raise ValueError("invalid acceleration limit")
            cache[key] = float(value)
            return float(value)

        def cap_derivative(speed):
            if not physical_mode: return 0.0
            if not math.isfinite(speed) or speed < 0: raise ValueError("invalid derivative speed")
            h = min(.001, speed / 2.) if speed > 0 else .001
            if speed > 0:
                return (cap(speed + h, derivative=True) - cap(speed - h, derivative=True)) / (2*h)
            return (cap(h, derivative=True) - cap(0.0)) / h

        a0 = cap(0.0)
        if a0 <= 0: return finish_failure("zero_rest_limit")
        if seed is None:
            T = math.sqrt(2 * distance / (.5 * a0)); dt = T / n
            if not settings.minimum_interval_seconds <= dt <= settings.maximum_interval_seconds:
                return finish_failure("seed_out_of_bounds")
        from scipy.optimize import minimize

        def rollout(values):
            dts = values[:n]; acc = values[n:]; positions = [0.0]; speeds = [0.0]; times = [0.0]
            for h, a in zip(dts, acc):
                vn = speeds[-1] + a * h
                sn = positions[-1] + speeds[-1] * h + .5 * a * h * h
                tn = times[-1] + h
                if not all(map(math.isfinite, (vn, sn, tn))): raise ValueError("nonfinite rollout")
                positions.append(sn); speeds.append(vn); times.append(tn)
            return np.asarray(times), np.asarray(positions), np.asarray(speeds)

        def jets(values):
            dts, acc = values[:n], values[n:]
            dv = np.zeros((n+1, 2*n)); ds = np.zeros((n+1, 2*n))
            v = 0.; s = 0.
            for i, (h, a) in enumerate(zip(dts, acc)):
                e_dt = np.zeros(2*n); e_dt[i] = 1.
                e_a = np.zeros(2*n); e_a[n+i] = 1.
                dv[i+1] = dv[i] + a*e_dt + h*e_a
                ds[i+1] = ds[i] + h*dv[i] + (v+a*h)*e_dt + .5*h*h*e_a
                v += a*h; s += v*h - .5*a*h*h
            return dv, ds

        def eq(values):
            terminal = rollout(values)[1][-1] - distance
            return terminal if physical_mode else terminal / max(1.0, distance)
        def eq_jac(values):
            return jets(values)[1][-1] if physical_mode else jets(values)[1][-1] / max(1.0, distance)
        def constraints(values, fractions):
            _, _, speeds = rollout(values); acc = values[n:]; out = []
            for i in range(n):
                for fraction in fractions: out.append(cap(speeds[i] + acc[i] * values[i] * fraction) - acc[i])
            return np.asarray(out)

        def constraints_jac(values, fractions):
            dts, acc = values[:n], values[n:]
            dv, _ = jets(values); _, _, speeds = rollout(values); rows = []
            for i in range(n):
                for fraction in fractions:
                    speed = speeds[i] + acc[i]*dts[i]*fraction
                    velocity_jet = dv[i].copy()
                    velocity_jet[i] += acc[i]*fraction
                    velocity_jet[n+i] += dts[i]*fraction
                    derivative = cap_derivative(float(speed)) if np.any(velocity_jet != 0.) else 0.
                    row = derivative*velocity_jet
                    row[n+i] -= 1.
                    rows.append(row)
            return np.asarray(rows)

        solver_ftol = float(settings.tolerance) if physical_mode else float(settings.tolerance) / (10.0 * max(1.0, distance))
        report["solver_ftol"] = solver_ftol
        report["transcription_mode"] = mode
        report["solver_ftol_note"] = ("terminal-distance equality in metres; ftol uses caller tolerance in metres" if physical_mode else "normalized terminal-distance equality; ftol is one tenth of normalized caller physical tolerance")
        report["jacobian_note"] = ("analytic kinematic chain; cap derivative central difference h=min(0.001,v/2), forward difference at rest" if physical_mode else "SLSQP default finite differences; legacy transcription preserved")
        x0 = seed.copy() if seed is not None else np.r_[np.full(n, dt), np.full(n, .5 * a0)]
        bounds = [(settings.minimum_interval_seconds, settings.maximum_interval_seconds)] * n + [(0, None)] * n
        eq_constraint = {"type": "eq", "fun": eq}
        ineq_constraint = {"type": "ineq", "fun": lambda values: constraints(values, (0., .5, 1.))}
        if physical_mode:
            eq_constraint["jac"] = eq_jac
            ineq_constraint["jac"] = lambda values: constraints_jac(values, (0., .5, 1.))
        solver_kwargs = {"constraints": (eq_constraint, ineq_constraint),
                         "options": {"maxiter": int(settings.maximum_iterations), "ftol": solver_ftol}}
        if physical_mode:
            solver_kwargs["jac"] = lambda values: np.r_[np.ones(n), np.zeros(n)]
        opt = minimize(lambda values: float(np.sum(values[:n])), x0, method="SLSQP", bounds=bounds,
                       **solver_kwargs)
        report["solver_reported_success"] = bool(opt.success and opt.status == 0)
        report["optimizer_status"] = opt.status; report["optimizer_message"] = str(opt.message)
        # Preserve finite solver iterate facts even when SLSQP rejects its status.
        try:
            x = np.asarray(opt.x, dtype=float)
            if x.shape == (2*n,) and np.all(np.isfinite(x)):
                ti, si, vi = rollout(x)
                report.update({"last_finite_iterate": x.tolist(), "last_finite_time_s": ti.tolist(),
                               "last_finite_distance_m": si.tolist(), "last_finite_speed_mps": vi.tolist(),
                               "last_finite_acceleration_mps2": x[n:].tolist(),
                               "last_finite_total_time_s": float(np.sum(x[:n])),
                               "last_finite_finish_distance_error_m": float(si[-1]-distance),
                               "iterations": getattr(opt, "nit", None)})
        except Exception as exc:
            report["last_iterate_exception_type"] = type(exc).__name__
            report["last_iterate_exception_message"] = str(exc)
        if not report["solver_reported_success"]:
            return finish_failure("optimizer_not_converged")
        if x is None or x.shape != (2*n,) or not np.all(np.isfinite(x)):
            return finish_failure("invalid_optimizer_iterate")
        t, s, v = rollout(x); acc = x[n:]; dts = x[:n]
        # Reject optimizer output outside explicit decision bounds before querying
        # the physical callback at any nonphysical speed/control state.
        pre_duration_violation = max(0., float(np.max(settings.minimum_interval_seconds-dts)),
                                     float(np.max(dts-settings.maximum_interval_seconds)))
        pre_negative_accel_violation = max(0., float(-np.min(acc)))
        if pre_duration_violation > 0 or pre_negative_accel_violation > 0:
            report["duration_bound_violation_s"] = pre_duration_violation
            report["negative_acceleration_violation_mps2"] = pre_negative_accel_violation
            return finish_failure("optimizer_iterate_out_of_bounds")
        live = constraints(x, (0., .5, 1.))
        fractions = tuple(np.linspace(0., 1., int(settings.heldout_fractions)))
        held = constraints(x, fractions)
        report["heldout_sampling_complete"] = True
        report["heldout_samples"] = int(len(held))
        report["sampled_minimum_slack_mps2"] = float(np.min(live))
        report["heldout_minimum_slack_mps2"] = float(np.min(held))
        report["finish_distance_error_m"] = float(s[-1] - distance)
        report["initial_speed_error_mps"] = float(v[0])
        report["initial_distance_error_m"] = float(s[0])
        report["minimum_speed_mps"] = float(np.min(v))
        report["minimum_acceleration_mps2"] = float(np.min(acc))
        report["duration_bound_violation_s"] = float(max(0., np.max(settings.minimum_interval_seconds-dts), np.max(dts-settings.maximum_interval_seconds)))
        report["negative_acceleration_violation_mps2"] = float(max(0., -np.min(acc)))
        report["sampled_acceleration_violation_mps2"] = float(max(0., -np.min(live)))
        report["heldout_acceleration_violation_mps2"] = float(max(0., -np.min(held)))
        tol = float(settings.tolerance)
        report["sampled_feasible"] = bool(report["sampled_acceleration_violation_mps2"] <= tol)
        report["heldout_feasible"] = bool(report["heldout_acceleration_violation_mps2"] <= tol)
        report["physical_feasibility"] = bool(
            report["finish_distance_error_m"] <= tol and report["finish_distance_error_m"] >= -tol and
            abs(report["initial_speed_error_mps"]) <= tol and abs(report["initial_distance_error_m"]) <= tol and
            report["duration_bound_violation_s"] <= tol and report["negative_acceleration_violation_mps2"] <= tol and
            report["minimum_speed_mps"] >= -tol and report["sampled_feasible"] and report["heldout_feasible"])
        budget_check("after heldout checks")
        if (not report["sampled_feasible"] or not report["heldout_feasible"] or
                not report["physical_feasibility"]):
            return finish_failure("physical_or_heldout_check_failed")
        report["optimizer_converged"] = True
        result = StandingStraightResult(t, s, v, acc, float(np.sum(dts)))
        report["failure"] = None
        # Serialize/copy the complete payload before final cooperative deadline test.
        report["wall_seconds"] = time.monotonic() - started
        safe = _jsonsafe(report)
        json.dumps(safe, allow_nan=False)
        budget_check("after result copy and report serialization")
        safe["wall_seconds"] = time.monotonic() - started
        json.dumps(safe, allow_nan=False)
        budget_check("after final report assembly")
        return result, safe
    except Exception as exc:
        text = str(exc)
        is_budget = isinstance(exc, TimeoutError) or (isinstance(exc, RuntimeError) and "budget" in text.lower())
        report["exception_type"] = type(exc).__name__
        report["exception_message"] = text[:500]
        return finish_failure("budget_exhausted" if is_budget else "callback_or_solver_error", budget=is_budget)
