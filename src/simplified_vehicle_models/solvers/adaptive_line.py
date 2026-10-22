"""Bounded, fail-closed adaptive mesh wrapper for the spline line OCP.

The fixed-mesh solver remains the authority for every round.  This module only
uses independent dense evidence to select local mesh additions and transfers an
``energy_hs_v1`` warm seed; it never changes the spline basis or solver gates.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np

from simplified_vehicle_models.solvers.line_refinement import (
    merge_refinement_mesh,
    project_energy_seed,
)
from simplified_vehicle_models.solvers.line_validation import validate_line
from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line


def _solution_status(solution: object) -> str:
    return str(getattr(solution, "status", "unknown"))


def _finite_mesh(mesh: object, minimum_spacing: float) -> np.ndarray:
    values = np.asarray(mesh, dtype=float)
    if (values.ndim != 1 or values.size < 2 or not np.all(np.isfinite(values))
            or np.any(np.diff(values) < minimum_spacing)):
        raise ValueError("mesh must be finite, increasing, and satisfy minimum_spacing")
    return values.copy()


def _diagnostic_stations(validation: dict[str, Any]) -> np.ndarray:
    """Return every actual station reported by a failed physical diagnostic."""
    stations: list[float] = []
    nonphysical = {"derivative_uncertainty", "implied_acceleration_derivative_uncertainty"}
    for diagnostic in validation.get("diagnostics", ()):
        if diagnostic.get("passed") is False and diagnostic.get("name") not in nonphysical:
            station = diagnostic.get("worst_station")
            if station is not None and np.isfinite(station):
                stations.append(float(station))
            for value in diagnostic.get("refinement_stations", ()):
                if np.isfinite(value):
                    stations.append(float(value))
    return np.unique(np.asarray(stations, dtype=float)) if stations else np.array([], dtype=float)


def _refinement_stations(mesh: np.ndarray, spline: object, solution: object,
                         validation: dict[str, Any]) -> np.ndarray:
    """Bisect elements containing failed-diagnostic stations and retain local knots."""
    failures = _diagnostic_stations(validation)
    if not failures.size:
        return failures
    element_indices = np.unique(np.clip(np.searchsorted(mesh, failures, side="right") - 1,
                                        0, mesh.size - 2))
    extras: list[float] = []
    trajectory = getattr(solution, "trajectory", None)
    reference = getattr(trajectory, "reference", None)
    source_knots = np.asarray(getattr(reference, "source_station", ()), dtype=float)
    offset_knots = np.asarray(getattr(spline, "control_distance", ()), dtype=float)
    for index in element_indices:
        left, right = float(mesh[index]), float(mesh[index + 1])
        extras.append(.5 * (left + right))
        for knots in (source_knots, offset_knots):
            extras.extend(float(value) for value in knots if left < value < right)
    return np.asarray(extras, dtype=float)


def _round_record(round_index: int, mesh: np.ndarray, solution: object | None,
                  validation: dict[str, Any] | None, *, stop: str | None = None,
                  seed_diagnostics: dict[str, Any] | None = None,
                  error: Exception | None = None) -> dict[str, Any]:
    return {
        "round": round_index,
        "mesh": mesh.tolist(),
        "node_count": int(mesh.size),
        "status": _solution_status(solution) if solution is not None else "solver_failed",
        "time": None if solution is None else getattr(solution, "lap_time", None),
        "diagnostics": [] if validation is None else validation.get("diagnostics", []),
        "validation": validation,
        "seed_diagnostics": seed_diagnostics,
        "stop": stop,
        "error": None if error is None else str(error),
    }


def solve_adaptive_line(track, corridor, surface, mesh, spline, settings, *, vehicle=None,
                        seed=None, maximum_rounds=3, maximum_nodes=2000,
                        minimum_spacing=1.e-4, dense_maximum_refinements=6,
                        on_round: Callable[[dict[str, Any]], None] | None = None):
    """Solve, independently validate, and locally refine within explicit budgets.

    Returns ``(solution, report)``.  A solution is returned only when both dense
    evidence and live-model evidence are exactly ``True``.  On every other stop
    the first tuple item is ``None`` and ``accepted`` is false.
    """
    if isinstance(maximum_rounds, bool) or int(maximum_rounds) < 1:
        raise ValueError("maximum_rounds must be at least one")
    if isinstance(maximum_nodes, bool) or int(maximum_nodes) < 2:
        raise ValueError("maximum_nodes must be at least two")
    if (not np.isfinite(minimum_spacing) or minimum_spacing <= 0.0
            or isinstance(dense_maximum_refinements, bool) or int(dense_maximum_refinements) < 1):
        raise ValueError("minimum_spacing must be positive and dense_maximum_refinements at least one")
    current_mesh = _finite_mesh(mesh, float(minimum_spacing))
    if current_mesh.size > int(maximum_nodes):
        raise ValueError("initial mesh exceeds maximum_nodes")
    rounds: list[dict[str, Any]] = []
    current_seed = seed
    final_stop = "maximum_rounds"
    for round_index in range(1, int(maximum_rounds) + 1):
        try:
            solution = solve_spline_line(track, corridor, surface, current_mesh, spline,
                                         settings, seed=current_seed)
        except Exception as error:
            record = _round_record(round_index, current_mesh, None, None,
                                   stop="solver_failed", error=error)
            rounds.append(record)
            if on_round is not None:
                on_round(record)
            final_stop = "solver_failed"
            break
        validation = validate_line(solution, corridor, surface, settings, vehicle=vehicle,
                                   maximum_refinements=int(dense_maximum_refinements))
        dense = validation.get("checks", {}).get("dense_feasibility") is True
        model = validation.get("model_feasible") is True
        record = _round_record(round_index, current_mesh, solution, validation)
        rounds.append(record)
        if on_round is not None:
            on_round(record)
        if dense and model:
            record["stop"] = "validated"
            return solution, {
                "accepted": True, "status": "validated", "stop": "validated",
                "rounds": rounds, "final_mesh": current_mesh.tolist(),
                "final_node_count": int(current_mesh.size),
            }
        # Unresolved FD evidence has no physical station to refine.  Do not turn
        # validation uncertainty into an unbounded mesh-growth policy.
        failed_stations = _diagnostic_stations(validation)
        if not failed_stations.size:
            record["stop"] = "unresolved_no_refinement_station"
            final_stop = record["stop"]
            break
        if round_index == int(maximum_rounds):
            record["stop"] = "maximum_rounds"
            final_stop = record["stop"]
            break
        additions = _refinement_stations(current_mesh, spline, solution, validation)
        proposed = merge_refinement_mesh(current_mesh, additions,
                                         minimum_spacing=float(minimum_spacing))
        if proposed.size == current_mesh.size:
            record["stop"] = "unresolved_no_refinement_station"
            final_stop = record["stop"]
            break
        if proposed.size > int(maximum_nodes):
            record["stop"] = "maximum_nodes"
            final_stop = record["stop"]
            break
        try:
            current_seed, seed_diagnostics = project_energy_seed(
                solution, proposed, spline, minimum_spacing=float(minimum_spacing),
            )
        except (TypeError, ValueError) as error:
            record["stop"] = "seed_projection_failed"
            record["error"] = str(error)
            final_stop = record["stop"]
            break
        record["next_mesh"] = proposed.tolist()
        record["seed_diagnostics"] = seed_diagnostics
        current_mesh = proposed
    return None, {
        "accepted": False, "status": "failed", "stop": final_stop,
        "rounds": rounds, "final_mesh": current_mesh.tolist(),
        "final_node_count": int(current_mesh.size),
    }
