"""Bounded, fail-closed adaptive mesh wrapper for the spline line OCP.

The fixed-mesh solver remains the authority for every round.  This module only
uses independent dense evidence to select local mesh additions and transfers an
``energy_hs_v1`` warm seed; it never changes the spline basis or solver gates.
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable

import numpy as np

from simplified_vehicle_models.solvers.line_refinement import (
    merge_refinement_mesh,
    project_energy_seed,
)
from simplified_vehicle_models.solvers.line_validation import strict_validation, validate_line
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
    nonphysical = {
        "derivative_uncertainty", "implied_acceleration_derivative_uncertainty",
        # These empirical upper bounds contain FD uncertainty.  Their matching
        # raw residual diagnostic remains eligible whenever it really fails.
        "dynamics_defect_upper_bound", "implied_acceleration_upper_bound",
    }
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


def _bounded_integer(value: object, *, name: str, minimum: int) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} must be an integer at least {minimum}")
    value = int(value)
    if value < minimum:
        raise ValueError(f"{name} must be an integer at least {minimum}")
    return value


def solve_adaptive_line(track, corridor, surface, mesh, spline, settings, *, vehicle=None,
                        seed=None, maximum_rounds=3, maximum_nodes=2000,
                        minimum_spacing=1.e-4, dense_maximum_refinements=6,
                        on_round: Callable[[dict[str, Any]], None] | None = None):
    """Solve, independently validate, and locally refine within explicit budgets.

    Returns ``(solution, report)`` only for a locally dense-and-live validated
    candidate.  This is deliberately not strict six-gate numerical acceptance:
    missing mesh/basis/matched/replay evidence remains incomplete and accepted
    is always false here.
    """
    maximum_rounds = _bounded_integer(maximum_rounds, name="maximum_rounds", minimum=1)
    maximum_nodes = _bounded_integer(maximum_nodes, name="maximum_nodes", minimum=2)
    dense_maximum_refinements = _bounded_integer(
        dense_maximum_refinements, name="dense_maximum_refinements", minimum=1)
    if isinstance(minimum_spacing, (bool, np.bool_)):
        raise ValueError("minimum_spacing must be finite and positive")
    try:
        minimum_spacing = float(minimum_spacing)
    except (TypeError, ValueError) as error:
        raise ValueError("minimum_spacing must be finite and positive") from error
    if not np.isfinite(minimum_spacing) or minimum_spacing <= 0.0:
        raise ValueError("minimum_spacing must be finite and positive")
    current_mesh = _finite_mesh(mesh, minimum_spacing)
    if current_mesh.size > maximum_nodes:
        raise ValueError("initial mesh exceeds maximum_nodes")
    rounds: list[dict[str, Any]] = []
    current_seed = seed
    final_stop = "maximum_rounds"

    def notify(record: dict[str, Any]) -> None:
        # An observer gets the final round state, never a mutable future update.
        if on_round is not None:
            on_round(deepcopy(record))

    for round_index in range(1, maximum_rounds + 1):
        try:
            solution = solve_spline_line(track, corridor, surface, current_mesh, spline,
                                         settings, seed=current_seed)
            if _solution_status(solution) != "Solve_Succeeded":
                raise RuntimeError(f"strict solver returned {_solution_status(solution)}")
        except (RuntimeError, ValueError, TypeError, FloatingPointError) as error:
            record = _round_record(round_index, current_mesh, None, None,
                                   stop="solver_failed", error=error)
            rounds.append(record); notify(record)
            final_stop = "solver_failed"
            break
        try:
            validation = validate_line(solution, corridor, surface, settings, vehicle=vehicle,
                                       maximum_refinements=dense_maximum_refinements)
        except (RuntimeError, ValueError, TypeError, FloatingPointError) as error:
            record = _round_record(round_index, current_mesh, solution, None,
                                   stop="validation_failed", error=error)
            rounds.append(record); notify(record)
            final_stop = "validation_failed"
            break
        dense = validation.get("checks", {}).get("dense_feasibility") is True
        model = validation.get("model_feasible") is True
        record = _round_record(round_index, current_mesh, solution, validation)
        rounds.append(record)
        if dense and model:
            record["stop"] = "locally_validated"
            strict = strict_validation(True, validation.get("checks"))
            report = {
                "accepted": False, "validation_status": strict["validation_status"],
                "strict_validation": strict, "validated_solution_available": True,
                "status": "locally_validated", "stop": "locally_validated",
                "rounds": rounds, "final_mesh": current_mesh.tolist(),
                "final_node_count": int(current_mesh.size),
            }
            notify(record)
            return solution, report
        failed_stations = _diagnostic_stations(validation)
        if not failed_stations.size:
            record["stop"] = "unresolved_no_refinement_station"
            final_stop = record["stop"]; notify(record)
            break
        if round_index == maximum_rounds:
            record["stop"] = "maximum_rounds"
            final_stop = record["stop"]; notify(record)
            break
        try:
            additions = _refinement_stations(current_mesh, spline, solution, validation)
            proposed = merge_refinement_mesh(current_mesh, additions,
                                             minimum_spacing=minimum_spacing)
        except (RuntimeError, ValueError, TypeError, FloatingPointError) as error:
            record["stop"] = "refinement_failed"; record["error"] = str(error)
            final_stop = record["stop"]; notify(record)
            break
        if proposed.size == current_mesh.size:
            record["stop"] = "unresolved_no_refinement_station"
            final_stop = record["stop"]; notify(record)
            break
        if proposed.size > maximum_nodes:
            record["stop"] = "maximum_nodes"
            final_stop = record["stop"]; notify(record)
            break
        try:
            current_seed, seed_diagnostics = project_energy_seed(
                solution, proposed, spline, minimum_spacing=minimum_spacing,
            )
        except (RuntimeError, ValueError, TypeError, FloatingPointError) as error:
            record["stop"] = "seed_projection_failed"; record["error"] = str(error)
            final_stop = record["stop"]; notify(record)
            break
        record["next_mesh"] = proposed.tolist()
        record["seed_diagnostics"] = seed_diagnostics
        notify(record)
        current_mesh = proposed
    return None, {
        "accepted": False, "validated_solution_available": False,
        "validation_status": "solver_failed" if final_stop == "solver_failed" else "incomplete",
        "status": "failed", "stop": final_stop,
        "rounds": rounds, "final_mesh": current_mesh.tolist(),
        "final_node_count": int(current_mesh.size),
    }
