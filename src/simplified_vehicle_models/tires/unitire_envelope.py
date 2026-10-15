"""Reduce simplified UniTire to a pure-force lookup for point-mass use.

The point-mass solver has no slip states, so it cannot call the full combined-
slip tire law directly. This adapter samples the pure longitudinal and lateral
force peaks at each vertical load. It deliberately does not import or reuse the
legacy 2025 Matlab lap-sim friction-ellipse coefficients.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.tire import (
    UniTirePeakEnvelope,
    UniTireSimpleParameters,
)
from simplified_vehicle_models.tires.unitire_simple import (
    evaluate_unitire,
    loaded_radius,
)


def _parabolic_peak(force_samples: np.ndarray) -> np.ndarray:
    """Refine sampled absolute-force maxima with a local quadratic fit."""

    values = np.abs(np.asarray(force_samples, dtype=np.float64))
    columns = np.arange(values.shape[1])
    indices = np.argmax(values, axis=0)
    peak = values[indices, columns].copy()
    interior = (indices > 0) & (indices < values.shape[0] - 1)
    if not np.any(interior):
        return peak

    selected = np.flatnonzero(interior)
    center_rows = indices[selected]
    previous = values[center_rows - 1, selected]
    center = values[center_rows, selected]
    following = values[center_rows + 1, selected]
    denominator = previous - 2.0 * center + following
    offset = np.divide(
        0.5 * (previous - following),
        denominator,
        out=np.zeros_like(center),
        where=denominator < -np.finfo(np.float64).eps,
    )
    offset = np.clip(offset, -1.0, 1.0)
    refined = center - 0.25 * (previous - following) * offset
    peak[selected] = np.maximum(center, refined)
    return peak


def build_unitire_peak_envelope(
    parameters: UniTireSimpleParameters,
    vertical_loads: FloatArray | None = None,
    *,
    reference_speed: float = 15.0,
    slip_angle_max: float = 0.6,
    slip_ratio_max: float = 1.5,
    slip_samples: int = 601,
) -> UniTirePeakEnvelope:
    """Sample UniTire pure-slip peak force capacities over normal load.

    Friction coefficients in simplified UniTire depend on slip speed. The
    peak is therefore evaluated at a stated reference speed rather than
    treating ``mu_0_x``/``mu_0_y`` as a universal constant. Lookup loads
    default to 0--3500 N in 71 evenly spaced samples, spanning the placeholder
    car's wheel loads with downforce. Slip is swept from zero to the stated
    positive maximum; a local parabolic fit refines each sampled peak, and force
    symmetry supplies the opposite sign.
    """

    if not np.isfinite(reference_speed) or reference_speed <= 0.0:
        raise ValueError("reference_speed must be finite and positive")
    if not np.isfinite(slip_angle_max) or slip_angle_max <= 0.0:
        raise ValueError("slip_angle_max must be finite and positive")
    if not np.isfinite(slip_ratio_max) or slip_ratio_max <= 0.0:
        raise ValueError("slip_ratio_max must be finite and positive")
    if slip_samples < 3:
        raise ValueError("slip_samples must be at least three")

    if vertical_loads is None:
        loads = np.linspace(0.0, 3500.0, 71, dtype=np.float64)
    else:
        loads = np.asarray(vertical_loads, dtype=np.float64)
    if (
        loads.ndim != 1
        or loads.size < 2
        or not np.all(np.isfinite(loads))
        or np.any(loads < 0.0)
        or np.any(np.diff(loads) <= 0.0)
    ):
        raise ValueError("vertical_loads must be finite, nonnegative, and increasing")

    radii = np.asarray(loaded_radius(parameters, loads), dtype=np.float64)
    wheel_speeds = reference_speed / np.maximum(radii, np.finfo(float).eps)
    alpha = np.linspace(0.0, slip_angle_max, slip_samples, dtype=np.float64)[:, None]
    kappa = np.linspace(0.0, slip_ratio_max, slip_samples, dtype=np.float64)[:, None]
    load_row = loads[None, :]
    speed_row = wheel_speeds[None, :]

    lateral = evaluate_unitire(
        parameters,
        alpha,
        np.zeros_like(alpha),
        load_row,
        speed_row,
    )
    longitudinal = evaluate_unitire(
        parameters,
        np.zeros_like(kappa),
        kappa,
        load_row,
        speed_row,
    )
    fy_peak = _parabolic_peak(lateral.lateral_force)
    fx_peak = _parabolic_peak(longitudinal.longitudinal_force)

    # Exact zero load is required for a contact wheel in the point-mass model.
    fx_peak = np.where(loads == 0.0, 0.0, fx_peak)
    fy_peak = np.where(loads == 0.0, 0.0, fy_peak)
    reference_radius = float(loaded_radius(parameters, np.asarray(parameters.f_z_rated)))
    return UniTirePeakEnvelope(
        vertical_loads=loads.copy(),
        longitudinal_peak_force=np.asarray(fx_peak, dtype=np.float64),
        lateral_peak_force=np.asarray(fy_peak, dtype=np.float64),
        effective_radius=reference_radius,
        reference_speed=float(reference_speed),
    )
