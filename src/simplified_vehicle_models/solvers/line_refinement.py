"""Mesh refinement and warm-seed projection for the energy-HS line solver.

This module only transfers an existing ``energy_hs_v1`` representation.  It
neither changes the OCP nor certifies constraint feasibility on the new mesh.
"""
from __future__ import annotations

from typing import Any

import numpy as np


_ENERGY_REPRESENTATION = "energy_hs_v1"
_CONTROL_REPRESENTATIONS = {
    "quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1",
}


def _finite_increasing_mesh(mesh: object, *, name: str, minimum_spacing: float) -> np.ndarray:
    values = np.asarray(mesh, dtype=float)
    if values.ndim != 1 or values.size < 2:
        raise ValueError(f"{name} must be a one-dimensional mesh with at least two stations")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain only finite stations")
    if np.any(np.diff(values) <= 0.0):
        raise ValueError(f"{name} must be strictly increasing")
    if np.any(np.diff(values) < minimum_spacing):
        raise ValueError(f"{name} has stations closer than minimum_spacing")
    return values


def merge_refinement_mesh(mesh: object, extra_stations: object, *, minimum_spacing: float = 1.0e-7) -> np.ndarray:
    """Return a guarded mesh containing all compatible refinement stations.

    Original mesh stations are never moved or removed.  Finite extra stations
    must lie in the original closed domain; ones closer than
    ``minimum_spacing`` to a retained station are deliberately collapsed.  The
    original endpoints are assigned back exactly, which gives callers stable
    domain canonicalization despite mixed floating-point arithmetic.
    """
    if not np.isfinite(minimum_spacing) or minimum_spacing <= 0.0:
        raise ValueError("minimum_spacing must be finite and positive")
    base = _finite_increasing_mesh(mesh, name="mesh", minimum_spacing=float(minimum_spacing))
    extra = np.asarray(extra_stations, dtype=float)
    if extra.ndim != 1:
        raise ValueError("extra_stations must be one-dimensional")
    if not np.all(np.isfinite(extra)):
        raise ValueError("extra_stations must contain only finite stations")
    if np.any(extra < base[0]) or np.any(extra > base[-1]):
        raise ValueError("extra_stations must lie inside the mesh domain")

    # The base grid has strict priority: no extra may displace an existing
    # decision node.  Compatible source knots then win in ascending source
    # order, which gives deterministic collapse of nearly duplicate extras.
    selected: list[float] = []
    for value in np.sort(extra):
        position = int(np.searchsorted(base, value))
        neighbors = base[max(0, position - 1):min(base.size, position + 1)]
        if np.any(np.abs(neighbors - value) < minimum_spacing):
            continue
        if selected and value - selected[-1] < minimum_spacing:
            continue
        selected.append(float(value))
    result = np.sort(np.r_[base, selected])
    result[0], result[-1] = base[0], base[-1]
    if np.any(np.diff(result) < minimum_spacing):
        raise ValueError("merged mesh cannot satisfy minimum_spacing")
    return result


def _control_grid(spline: object, domain: tuple[float, float], *, label: str) -> np.ndarray:
    count = getattr(spline, "control_count", None)
    if isinstance(count, (bool, np.bool_)) or not isinstance(count, (int, np.integer)):
        raise ValueError(f"{label} must expose integer control_count")
    grid = np.asarray(getattr(spline, "control_distance", None), dtype=float)
    if grid.shape != (int(count),) or not np.all(np.isfinite(grid)) or np.any(np.diff(grid) <= 0.0):
        raise ValueError(f"{label} control_distance must be finite and strictly increasing")
    closed = bool(getattr(spline, "is_closed", False))
    start, end = domain
    tolerance = 1.0e-9 * max(1.0, end - start)
    if not np.isclose(grid[0], start, rtol=0.0, atol=tolerance):
        raise ValueError(f"{label} control grid must start at the mesh domain")
    if closed:
        if grid[-1] >= end - tolerance:
            raise ValueError(f"closed {label} control grid must exclude terminal endpoint")
    elif not np.isclose(grid[-1], end, rtol=0.0, atol=tolerance):
        raise ValueError(f"open {label} control grid must end at the mesh domain")
    return grid


def _trajectory_metric(trajectory: object, stations: np.ndarray) -> np.ndarray:
    if trajectory is None or not callable(getattr(trajectory, "evaluate", None)):
        raise ValueError("projection requires an exact source trajectory mapping")
    values = trajectory.evaluate(stations)
    metric = np.asarray(getattr(values, "metric", None), dtype=float)
    if metric.shape != stations.shape or not np.all(np.isfinite(metric)) or np.any(metric <= 0.0):
        raise ValueError("source trajectory must provide finite positive metric values")
    return metric


def _project_coefficients(solution: object, new_spline: object, domain: tuple[float, float]) -> np.ndarray:
    old = np.asarray(getattr(solution, "coefficients", None), dtype=float)
    old_spline = getattr(getattr(solution, "trajectory", None), "spline", None)
    if old_spline is None:
        raise ValueError("projection requires the solution trajectory spline")
    if bool(getattr(old_spline, "is_closed", False)) != bool(getattr(new_spline, "is_closed", False)):
        raise ValueError("source and new spline topology must match")
    old_grid = _control_grid(old_spline, domain, label="source spline")
    new_grid = _control_grid(new_spline, domain, label="new_spline")
    if old.shape != (old_grid.size,):
        raise ValueError("solution coefficients do not match source spline topology")
    if old_grid.shape == new_grid.shape and np.array_equal(old_grid, new_grid):
        return old.copy()

    # Include both bases' knots plus dense, deterministic samples.  This is a
    # least-squares basis transfer, not an interpolation fallback for dynamics.
    dense = np.linspace(domain[0], domain[1], max(65, 8 * (old_grid.size + new_grid.size)))
    stations = np.unique(np.r_[dense, old_grid, new_grid, domain])
    source_offset = np.asarray(getattr(solution, "trajectory").evaluate(stations).offset, dtype=float)
    basis = np.asarray(new_spline.basis(stations)[0], dtype=float)
    if basis.shape != (stations.size, new_grid.size) or not np.all(np.isfinite(source_offset)):
        raise ValueError("spline basis transfer requires finite offset mapping")
    return np.linalg.lstsq(basis, source_offset, rcond=None)[0]


def project_energy_seed(
    solution: object, new_mesh: object, new_spline: object | None = None,
    *, minimum_spacing: float = 1.0e-7,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Project an ``energy_hs_v1`` solution into its standard decision layout.

    Returns ``[coefficients | v_nodes | a_nodes | a_midpoints]`` and diagnostic
    facts.  The diagnostics explicitly describe a projection only: they do not
    assert OCP, bounds, corridor, or capability feasibility.
    """
    if not np.isfinite(minimum_spacing) or minimum_spacing <= 0.0:
        raise ValueError("minimum_spacing must be finite and positive")
    evaluator = getattr(solution, "state_evaluator", None)
    representation = getattr(solution, "state_representation", None)
    control_representation = getattr(solution, "control_representation", None)
    if control_representation is None:
        # Artifacts predating explicit control metadata are legacy by contract.
        control_representation = "quadratic_acceleration_v1"
    if control_representation not in _CONTROL_REPRESENTATIONS:
        raise ValueError("project_energy_seed requires a known control representation")
    evaluator_control = getattr(evaluator, "control_representation", control_representation)
    if evaluator_control != control_representation:
        raise ValueError("state evaluator and solution control representations differ")
    if representation != _ENERGY_REPRESENTATION and getattr(evaluator, "state_representation", None) != _ENERGY_REPRESENTATION:
        raise ValueError("project_energy_seed requires energy_hs_v1 representation")
    if evaluator is None or not callable(getattr(evaluator, "evaluate", None)):
        raise ValueError("project_energy_seed requires an energy state evaluator")
    source_mesh = _finite_increasing_mesh(
        getattr(solution, "distance", None), name="solution distance",
        minimum_spacing=float(minimum_spacing),
    )
    mesh = _finite_increasing_mesh(
        new_mesh, name="new_mesh", minimum_spacing=float(minimum_spacing),
    )
    domain = (float(source_mesh[0]), float(source_mesh[-1]))
    tolerance = 1.0e-9 * max(1.0, domain[1] - domain[0])
    if not (np.isclose(mesh[0], domain[0], rtol=0.0, atol=tolerance) and np.isclose(mesh[-1], domain[1], rtol=0.0, atol=tolerance)):
        raise ValueError("new_mesh domain must match solution distance domain")
    trajectory = getattr(solution, "trajectory", None)
    if getattr(evaluator, "trajectory", trajectory) is not trajectory:
        raise ValueError("state evaluator must use the exact solution trajectory")
    target_spline = getattr(trajectory, "spline", None) if new_spline is None else new_spline
    coefficients = _project_coefficients(solution, target_spline, domain)
    if target_spline is getattr(trajectory, "spline", None):
        projected_trajectory = trajectory
    else:
        reference = getattr(trajectory, "reference", None)
        if reference is None:
            raise ValueError("different new_spline requires an exact trajectory reference")
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory
        projected_trajectory = OffsetTrajectory(reference, target_spline, coefficients)
    metric = _trajectory_metric(projected_trajectory, mesh)
    midpoint_stations = 0.5 * (mesh[:-1] + mesh[1:])
    midpoint_metric = _trajectory_metric(projected_trajectory, midpoint_stations)
    state = evaluator.evaluate(mesh)
    try:
        energy = np.asarray(state["energy"], dtype=float)
        acceleration = np.asarray(state["acceleration"], dtype=float)
    except (KeyError, TypeError) as error:
        raise ValueError("energy state evaluator must provide energy and acceleration fields") from error
    if energy.shape != mesh.shape or acceleration.shape != mesh.shape:
        raise ValueError("energy evaluator fields must match new_mesh")
    if not np.all(np.isfinite(energy)) or np.any(energy <= 0.0) or not np.all(np.isfinite(acceleration)):
        raise ValueError("energy evaluator returned non-finite or nonpositive energy")
    speed = np.sqrt(energy)
    width = np.diff(mesh)
    midpoint = (6.0 * np.diff(energy) / width - 2.0 * metric[:-1] * acceleration[:-1]
                - 2.0 * metric[1:] * acceleration[1:]) / (8.0 * midpoint_metric)
    defect = np.diff(energy) - width / 6.0 * (2.0 * acceleration[:-1] * metric[:-1]
                                               + 8.0 * midpoint * midpoint_metric
                                               + 2.0 * acceleration[1:] * metric[1:])
    sample = np.linspace(domain[0], domain[1], max(65, 8 * coefficients.size))
    offset_error = np.asarray(projected_trajectory.evaluate(sample).offset, dtype=float) - np.asarray(trajectory.evaluate(sample).offset, dtype=float)
    seed = np.concatenate((coefficients, speed, acceleration, midpoint))
    diagnostics: dict[str, Any] = {
        "projection_only": True,
        "solver_feasibility_claimed": False,
        "capability_checked": False,
        "finite": bool(np.all(np.isfinite(seed))),
        "maximum_hs_energy_defect": float(np.max(np.abs(defect))) if defect.size else 0.0,
        "maximum_offset_projection_error": float(np.max(np.abs(offset_error))),
        "source_representation": _ENERGY_REPRESENTATION,
        "control_representation": control_representation,
    }
    return seed, diagnostics
