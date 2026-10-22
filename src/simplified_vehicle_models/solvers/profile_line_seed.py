"""Optional exact-centreline fixed-path energy-profile seed for spline OCPs.

This module builds a *projection* seed only.  It does not claim that a profile
on an independently sampled path is feasible for every spline-OCP row.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from simplified_vehicle_models.solvers.fixed_path_energy_nlp import fixed_path_energy_profile
from simplified_vehicle_models.solvers.qss_spline_ocp import _reference_curve
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory


def _diagnostics(reason: str, **values: Any) -> dict[str, Any]:
    return {"kind": "energy-nlp-profile-line-seed", "available": False, "reason": reason,
            "projection_only": True, **values}


def _interpolate_energy(profile, ell: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate profile energy in physical arclength and differentiate it.

    The derivative uses deterministic one-sided end points and centred interior
    differences, so node acceleration is exactly ``W_ell / 2``.
    """
    source_ell = np.asarray(profile.distance, dtype=float)
    speed = np.asarray(profile.speed, dtype=float)
    target_ell = np.asarray(ell, dtype=float)
    if (source_ell.ndim != 1 or source_ell.size < 2 or speed.shape != source_ell.shape
            or target_ell.ndim != 1 or target_ell.size < 2
            or not all(np.all(np.isfinite(x)) for x in (source_ell, speed, target_ell))
            or np.any(np.diff(source_ell) <= 0.0) or np.any(np.diff(target_ell) <= 0.0)
            or target_ell[0] < source_ell[0] or target_ell[-1] > source_ell[-1]):
        raise ValueError("profile energy interpolation requires finite covered increasing path stations")
    energy = np.interp(target_ell, source_ell, speed * speed)
    derivative = np.empty_like(energy)
    derivative[0] = (energy[1] - energy[0]) / (target_ell[1] - target_ell[0])
    derivative[-1] = (energy[-1] - energy[-2]) / (target_ell[-1] - target_ell[-2])
    derivative[1:-1] = (energy[2:] - energy[:-2]) / (target_ell[2:] - target_ell[:-2])
    return energy, 0.5 * derivative


def _hs_midpoint_acceleration(energy: np.ndarray, acceleration: np.ndarray,
                              metric: np.ndarray, metric_mid: np.ndarray,
                              mesh: np.ndarray) -> np.ndarray:
    """Repair midpoint controls so every energy Hermite--Simpson defect is zero."""
    width = np.diff(mesh)
    return (6.0 * np.diff(energy) / width - 2.0 * metric[:-1] * acceleration[:-1]
            - 2.0 * metric[1:] * acceleration[1:]) / (8.0 * metric_mid)


def _within_bounds(seed: np.ndarray, surface, spline, mesh: np.ndarray, settings, slack: float) -> tuple[bool, str | None]:
    """Check only cheap global variable bounds, without constructing a second NLP."""
    nodes, count, controls = mesh.size, mesh.size - 1, int(spline.control_count)
    if seed.shape != (controls + 3 * nodes - 1,) or not np.all(np.isfinite(seed)):
        return False, "nonfinite-or-layout-mismatch"
    speed = seed[controls:controls + nodes]
    acceleration = seed[controls + nodes:]
    maximum_speed = (float(np.max(surface.domain_speed_values))
                     if getattr(surface, "representation", None) == "signed_transient_normalized_v1"
                     else float(surface.speed[-1]))
    lower = float(np.min(surface.acceleration_min))
    upper = float(np.max(surface.acceleration_max))
    violation = max(float(settings.minimum_speed - np.min(speed)), float(np.max(speed) - maximum_speed),
                    float(lower - np.min(acceleration)), float(np.max(acceleration) - upper), 0.0)
    if violation > slack:
        return False, f"variable-bound-violation:{violation:g}"
    return True, None


def build_profile_line_seed(track, corridor, surface, mesh, spline, settings, *,
                            profile_step: float = 2., maximum_profile_iterations: int = 200,
                            maximum_profile_seconds: float = 30.) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Return an optional exact-centreline energy-profile projection seed.

    A nonconverged profile, unsupported chart, invalid analytic geometry, or
    profile projection outside OCP variable bounds returns ``None`` with a
    reason.  No clipping or implicit fallback is performed here.
    """
    started = time.monotonic()
    try:
        if not math.isfinite(profile_step) or profile_step <= 0.0:
            raise ValueError("profile_step must be finite and positive")
        if not math.isfinite(maximum_profile_seconds) or maximum_profile_seconds <= 0.0:
            raise ValueError("maximum_profile_seconds must be finite and positive")
        reference = _reference_curve(track)
        coefficients = np.zeros(spline.control_count, dtype=float)
        trajectory = OffsetTrajectory(reference, spline, coefficients)
        path = trajectory.to_path(np.asarray(reference.source_station, dtype=float))
        boundary = "flying" if bool(track.is_closed) else "rolling"
        start_speed = None if boundary == "flying" else max(float(settings.start_speed), float(settings.minimum_speed))
        profile = fixed_path_energy_profile(path, surface, boundary=boundary,
            start_speed=start_speed, trajectory=trajectory, step=profile_step,
            maximum_iterations=maximum_profile_iterations,
            maximum_wall_seconds=maximum_profile_seconds)
        if not profile.converged:
            return None, _diagnostics("profile-not-converged", boundary=boundary,
                profile_status=profile.status, profile_detail=profile.diagnostics.get("detail"),
                profile_iterations=profile.iterations, elapsed_seconds=time.monotonic() - started)
        node_ell = np.asarray(trajectory.station_map.to_path(np.asarray(mesh, dtype=float)), dtype=float)
        energy, acceleration = _interpolate_energy(profile, node_ell)
        node_geometry = trajectory.evaluate(np.asarray(mesh, dtype=float))
        midpoint = .5 * (np.asarray(mesh, dtype=float)[:-1] + np.asarray(mesh, dtype=float)[1:])
        midpoint_metric = np.asarray(trajectory.evaluate(midpoint).metric, dtype=float)
        metric = np.asarray(node_geometry.metric, dtype=float)
        mid_acceleration = _hs_midpoint_acceleration(energy, acceleration, metric, midpoint_metric,
                                                      np.asarray(mesh, dtype=float))
        seed = np.concatenate([coefficients, np.sqrt(energy), acceleration, mid_acceleration])
        valid, reason = _within_bounds(seed, surface, spline, np.asarray(mesh, dtype=float), settings,
                                       slack=8.0 * float(settings.tolerance))
        if not valid:
            return None, _diagnostics(reason, boundary=boundary,
                profile_status=profile.status, elapsed_seconds=time.monotonic() - started)
        return seed, {"kind": "energy-nlp-profile-line-seed", "available": True,
                      "reason": None, "projection_only": True, "boundary": boundary,
                      "profile_status": profile.status, "profile_iterations": profile.iterations,
                      "profile_lap_time": profile.lap_time, "elapsed_seconds": time.monotonic() - started,
                      "node_path_station_m": node_ell.tolist(),
                      "coefficient_sha256": __import__("hashlib").sha256(coefficients.tobytes()).hexdigest(),
                      "ocp_feasible": False}
    except (TypeError, ValueError, ImportError) as error:
        return None, _diagnostics("unsupported-or-invalid", error=f"{type(error).__name__}: {error}",
                                  elapsed_seconds=time.monotonic() - started)


__all__ = ["build_profile_line_seed", "_interpolate_energy", "_hs_midpoint_acceleration"]
