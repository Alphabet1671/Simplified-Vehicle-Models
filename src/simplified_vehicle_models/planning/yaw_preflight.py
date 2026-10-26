"""Track/map preflight for the yaw-aware virtual driver.

Run before planning so an unseen track fails with an explicit cause rather
than a generic "no path". Two families of checks:

* **Map coverage** — for each station of a fixed-geometry path, does the strict
  yaw map support *any* steady (zero longitudinal acceleration) speed, and what
  is the highest such speed? Stations with none are map holes: the solver
  cannot invent support, so the run is blocked. Stations whose steady maximum
  is pinned at the lowest positive speed knot are flagged as near-holes.
* **Geometry conditioning** — curvature-rate noise relative to the grid
  spacing. Yaw demand contains ``v^2 * dkappa/ds``; a sampled path whose
  curvature rate is dominated by station-to-station jitter produces spurious
  yaw demand. This module only *measures and reports*; it never smooths. A
  documented resampling policy belongs to the caller.

The checks are necessary, not sufficient: passing preflight does not prove a
coupled profile exists (acceleration/braking transitions are not examined).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_map import interpolator_for

#: Speed samples per map speed cell for the steady-support scan.
SPEED_SAMPLES_PER_CELL = 8
#: Smoothing window for the curvature-rate high-frequency residual.
NOISE_WINDOW_M = 1.0


@dataclass(frozen=True, slots=True)
class PreflightReport:
    ok: bool
    blocking: tuple[str, ...]
    warnings: tuple[str, ...]
    metrics: Mapping[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"ok": self.ok, "blocking": list(self.blocking), "warnings": list(self.warnings),
                "metrics": dict(self.metrics)}


def _steady_speed_support(path: YawPath, capability: YawCapabilityMap) -> np.ndarray:
    """Highest speed with strict steady support at each station, NaN if none.

    Uses the same strict trilinear query and yaw-demand sign convention as the
    solver's edge oracle, with zero longitudinal acceleration.
    """
    interp = interpolator_for(capability)
    axis = np.asarray(capability.speed, float)
    samples = np.unique(np.concatenate([np.linspace(a, b, SPEED_SAMPLES_PER_CELL + 1)
                                        for a, b in zip(axis[:-1], axis[1:])]))
    samples = samples[samples > 0]
    best = np.full(len(path.distance), np.nan)
    for i, (k, kp) in enumerate(zip(path.curvature, path.curvature_rate)):
        k = float(k); kp = float(kp)
        sign = math.copysign(1., k if abs(k) > 1e-12 else (kp if abs(kp) > 1e-12 else 1.))
        for v in samples[::-1]:
            moments = interp.query(float(v), 0.0, float(v * v * k))
            if moments is None:
                continue
            into, out = moments
            demand = sign * capability.yaw_inertia * (float(v * v) * kp)
            if into - demand >= -1e-9 and out + demand >= -1e-9:
                best[i] = float(v)
                break
    return best


def _curvature_noise(path: YawPath, window_m: float = NOISE_WINDOW_M) -> dict:
    """High-frequency curvature-rate content relative to its own scale.

    ``residual = kappa' - moving_average(kappa', window_m)``; the reported ratio
    is ``rms(residual) / rms(kappa')``. Piecewise-polynomial splines with
    legitimate knot steps give a small ratio; station-to-station jitter gives a
    ratio near one.
    """
    kp = np.asarray(path.curvature_rate, float)
    ds = float(np.median(np.diff(path.distance)))
    width = max(3, int(round(window_m / max(ds, 1e-9))) | 1)
    if kp.size < 2 * width:
        return {"noise_ratio": 0.0, "window_m": window_m, "median_spacing_m": ds}
    kernel = np.ones(width) / width
    mode = "wrap" if path.is_closed else "edge"
    padded = np.pad(kp, width // 2, mode=mode)
    smooth = np.convolve(padded, kernel, mode="valid")
    residual = kp - smooth
    scale = float(np.sqrt(np.mean(kp * kp)))
    ratio = float(np.sqrt(np.mean(residual * residual)) / scale) if scale > 0 else 0.0
    return {"noise_ratio": ratio, "window_m": window_m, "median_spacing_m": ds,
            "rms_curvature_rate": scale, "max_abs_curvature_rate": float(np.max(np.abs(kp)))}


def preflight(path: YawPath, capability: YawCapabilityMap, *, maximum_speed: float | None = None,
              noise_ratio_limit: float = 0.3) -> PreflightReport:
    """Check that ``capability`` can plausibly support ``path``."""
    if not isinstance(path, YawPath) or not isinstance(capability, YawCapabilityMap):
        raise TypeError("path and capability must be YawPath and YawCapabilityMap")
    blocking, warnings = [], []
    ay_lo, ay_hi = float(capability.lateral_acceleration[0]), float(capability.lateral_acceleration[-1])
    v_floor = float(capability.speed[1]) if capability.speed.size > 1 else float(capability.speed[0])

    steady = _steady_speed_support(path, capability)
    holes = np.flatnonzero(~np.isfinite(steady))
    if holes.size:
        blocking.append(f"map_hole: {holes.size} station(s) have no strict steady yaw support at any positive speed")
    finite = steady[np.isfinite(steady)]
    near = np.flatnonzero(np.isfinite(steady) & (steady <= v_floor + 1e-12))
    if near.size:
        warnings.append(f"near_map_hole: {near.size} station(s) only supported at or below the first speed knot ({v_floor:.3g} m/s)")

    # Lateral-axis saturation: fraction of the lap where the steady maximum is
    # set by the map's ay extent rather than by speed or yaw-moment limits.
    k = np.asarray(path.curvature, float)
    ay_limit = np.where(k > 0, ay_hi, np.where(k < 0, ay_lo, np.inf))
    with np.errstate(divide="ignore", invalid="ignore"):
        v_ay = np.sqrt(np.where(k != 0, ay_limit / k, np.inf))
    saturated = np.isfinite(steady) & np.isfinite(v_ay) & (np.abs(steady - v_ay) <= 0.05 * np.maximum(v_ay, 1e-9) + 0.25)
    sat_fraction = float(np.mean(saturated))
    if sat_fraction > 0.25:
        warnings.append(f"lateral_axis_saturation: {sat_fraction:.0%} of stations are limited by the map's ay extent; "
                        "the map, not the vehicle, may be setting cornering speed")

    noise = _curvature_noise(path)
    if noise["noise_ratio"] > noise_ratio_limit:
        warnings.append(f"curvature_rate_noise: high-frequency curvature-rate content is {noise['noise_ratio']:.0%} "
                        f"of its RMS (>{noise_ratio_limit:.0%} over a {noise['window_m']:g} m window); yaw demand may "
                        "be dominated by sampling jitter (resample or use a smoother reference before trusting the profile)")

    if maximum_speed is not None and finite.size and float(np.max(finite)) < 0.5 * float(maximum_speed):
        warnings.append("low_supported_speed: highest steady-supported speed is under half the requested maximum")

    metrics = {
        "stations": int(len(path.distance)), "length_m": float(path.distance[-1]),
        "map_hole_stations": int(holes.size), "map_hole_first_stations": holes[:32].tolist(),
        "map_hole_distances_m": path.distance[holes[:32]].tolist(),
        "near_hole_stations": int(near.size),
        "steady_speed_min_mps": float(finite.min()) if finite.size else None,
        "steady_speed_median_mps": float(np.median(finite)) if finite.size else None,
        "steady_speed_max_mps": float(finite.max()) if finite.size else None,
        "lateral_axis_saturation_fraction": sat_fraction,
        "map_axes": {"speed": [float(capability.speed[0]), float(capability.speed[-1]), int(capability.speed.size)],
                     "ax": [float(capability.longitudinal_acceleration[0]), float(capability.longitudinal_acceleration[-1]),
                            int(capability.longitudinal_acceleration.size)],
                     "ay": [ay_lo, ay_hi, int(capability.lateral_acceleration.size)],
                     "mask_feasible_fraction": float(np.mean(capability.feasible_mask))},
        "curvature": {"max_abs": float(np.max(np.abs(k))), **noise},
        "scope": "steady (ax=0) strict support only; transitions are not examined",
    }
    return PreflightReport(not blocking, tuple(blocking), tuple(warnings), metrics)
