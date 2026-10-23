"""Bounded mechanical projection for fixed-energy Hermite--Simpson seeds.

The projection changes only acceleration controls.  It keeps supplied positive
energy nodes fixed and imposes the Hermite--Simpson energy equalities exactly.
It is deliberately a small LP utility, not an OCP feasibility check.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class EnergySeedProjectionSettings:
    """Numerical and topology settings for :func:`project_energy_seed_controls`.

    ``periodic_endpoint`` adds ``a_nodes[0] == a_nodes[-1]``.  It does not
    otherwise infer a closed trajectory or certify closed-track feasibility.
    """

    minimum_spacing: float = 1.0e-7
    equality_tolerance: float = 1.0e-9
    periodic_endpoint: bool = False


def _as_finite_vector(value: object, *, name: str, size: int | None = None,
                      positive: bool = False) -> np.ndarray:
    values = np.asarray(value, dtype=float)
    if values.ndim != 1 or (size is not None and values.size != size):
        suffix = " of the required length" if size is not None else ""
        raise ValueError(f"{name} must be a one-dimensional vector{suffix}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain only finite values")
    if positive and np.any(values <= 0.0):
        raise ValueError(f"{name} must contain only positive values")
    return values


def _validate_settings(settings: EnergySeedProjectionSettings) -> None:
    if not isinstance(settings, EnergySeedProjectionSettings):
        raise TypeError("settings must be an EnergySeedProjectionSettings instance")
    if (not np.isfinite(settings.minimum_spacing) or settings.minimum_spacing <= 0.0):
        raise ValueError("settings.minimum_spacing must be finite and positive")
    if (not np.isfinite(settings.equality_tolerance) or settings.equality_tolerance <= 0.0):
        raise ValueError("settings.equality_tolerance must be finite and positive")
    if not isinstance(settings.periodic_endpoint, (bool, np.bool_)):
        raise TypeError("settings.periodic_endpoint must be boolean")


def _diagnostics(*, reason: str | None, available: bool, settings: EnergySeedProjectionSettings,
                 **values: Any) -> dict[str, Any]:
    """Build deliberately conservative projection diagnostics."""
    return {
        "kind": "bounded-energy-seed-control-projection",
        "available": available,
        "reason": reason,
        "projection_only": True,
        "solver_feasibility_claimed": False,
        # Global bounds are deliberately used for midpoint controls: their
        # speed is coupled to unknown nodal acceleration through the equality.
        "capability_checked": False,
        "periodic_endpoint": bool(settings.periodic_endpoint),
        **values,
    }


def project_energy_seed_controls(
    energy: object,
    mesh: object,
    node_metric: object,
    midpoint_metric: object,
    source_node_acceleration: object,
    source_midpoint_acceleration: object,
    acceleration_min: float,
    acceleration_max: float,
    *,
    settings: EnergySeedProjectionSettings | None = None,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Project acceleration controls into global bounds while preserving energy.

    Parameters are node energy ``W``, strictly increasing chart station
    ``mesh``, positive trajectory metrics at nodes and interval midpoints, and
    source acceleration controls.  The returned control vector is
    ``[a_nodes | a_midpoints]``.  It minimizes the unweighted L1 deviation from
    the source controls subject to, for every interval ``i``,

    ``2 G_i a_i + 8 G_mid,i a_mid,i + 2 G_i+1 a_i+1 = 6 (W_i+1-W_i) / h_i``.

    Both node and midpoint controls use the supplied *global* bounds.  An
    infeasible bounded system returns ``(None, diagnostics)`` rather than
    clipping either controls or energy.
    """
    active_settings = EnergySeedProjectionSettings() if settings is None else settings
    _validate_settings(active_settings)
    energy_values = _as_finite_vector(energy, name="energy", positive=True)
    count = energy_values.size
    if count < 2:
        raise ValueError("energy must contain at least two nodes")
    stations = _as_finite_vector(mesh, name="mesh", size=count)
    width = np.diff(stations)
    if np.any(width <= 0.0):
        raise ValueError("mesh must be strictly increasing")
    if np.any(width < active_settings.minimum_spacing):
        raise ValueError("mesh has stations closer than settings.minimum_spacing")
    metric = _as_finite_vector(node_metric, name="node_metric", size=count, positive=True)
    metric_mid = _as_finite_vector(midpoint_metric, name="midpoint_metric", size=count - 1,
                                   positive=True)
    source_node = _as_finite_vector(source_node_acceleration,
                                    name="source_node_acceleration", size=count)
    source_mid = _as_finite_vector(source_midpoint_acceleration,
                                   name="source_midpoint_acceleration", size=count - 1)
    if (not np.isfinite(acceleration_min) or not np.isfinite(acceleration_max)
            or acceleration_min > acceleration_max):
        raise ValueError("acceleration bounds must be finite with min <= max")
    lower, upper = float(acceleration_min), float(acceleration_max)

    control_count = 2 * count - 1
    source = np.concatenate((source_node, source_mid))
    equality = np.zeros((count - 1 + int(active_settings.periodic_endpoint), control_count))
    rhs = np.empty(equality.shape[0])
    for index in range(count - 1):
        equality[index, index] = 2.0 * metric[index]
        equality[index, index + 1] = 2.0 * metric[index + 1]
        equality[index, count + index] = 8.0 * metric_mid[index]
        rhs[index] = 6.0 * (energy_values[index + 1] - energy_values[index]) / width[index]
    if active_settings.periodic_endpoint:
        equality[-1, 0], equality[-1, count - 1] = 1.0, -1.0
        rhs[-1] = 0.0

    # |x - source| <= auxiliary gives the linear L1 objective.  SciPy is
    # intentionally imported here so importing the module has no optimizer cost.
    objective = np.r_[np.zeros(control_count), np.ones(control_count)]
    inequalities = np.zeros((2 * control_count, 2 * control_count))
    inequalities[np.arange(control_count), np.arange(control_count)] = 1.0
    inequalities[np.arange(control_count), control_count + np.arange(control_count)] = -1.0
    inequalities[control_count + np.arange(control_count), np.arange(control_count)] = -1.0
    inequalities[control_count + np.arange(control_count), control_count + np.arange(control_count)] = -1.0
    inequality_rhs = np.r_[source, -source]
    equality_full = np.c_[equality, np.zeros((equality.shape[0], control_count))]
    try:
        from scipy.optimize import linprog
    except ImportError as error:  # pragma: no cover - depends on optional environment
        raise RuntimeError("bounded energy seed projection requires scipy.optimize.linprog") from error
    result = linprog(objective, A_ub=inequalities, b_ub=inequality_rhs,
                     A_eq=equality_full, b_eq=rhs,
                     bounds=[(lower, upper)] * control_count + [(0.0, None)] * control_count,
                     method="highs")
    if not result.success:
        return None, _diagnostics(reason="infeasible", available=False, settings=active_settings,
                                  linprog_status=int(result.status), linprog_message=str(result.message),
                                  energy_unchanged=True)

    controls = np.asarray(result.x[:control_count], dtype=float)
    residual = equality @ controls - rhs
    maximum_defect = float(np.max(np.abs(residual))) if residual.size else 0.0
    bound_violation = max(lower - float(np.min(controls)), float(np.max(controls)) - upper, 0.0)
    tolerance = active_settings.equality_tolerance * max(1.0, float(np.max(np.abs(rhs))))
    if maximum_defect > tolerance or bound_violation > active_settings.equality_tolerance:
        return None, _diagnostics(reason="postsolve-verification-failed", available=False,
                                  settings=active_settings,
                                  maximum_hs_energy_defect=maximum_defect,
                                  maximum_global_bound_violation=bound_violation,
                                  energy_unchanged=True)
    return controls, _diagnostics(
        reason=None, available=True, settings=active_settings,
        finite=bool(np.all(np.isfinite(controls))),
        energy_unchanged=True,
        maximum_hs_energy_defect=maximum_defect,
        maximum_global_bound_violation=bound_violation,
        l1_deviation=float(np.sum(np.abs(controls - source))),
        linprog_status=int(result.status),
    )


__all__ = ["EnergySeedProjectionSettings", "project_energy_seed_controls"]
