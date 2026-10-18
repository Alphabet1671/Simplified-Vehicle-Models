#!/usr/bin/env python3
"""Evaluate the Task 9 vertical 14DoF acceptance gates from finished attempts.

The forward runner records the gates it can decide in-run (``lap_completed``,
``contact_valid``, ``jounce_travel_valid`` for the plan's ``travel_valid``,
``out_of_range_evaluations`` for ``table_domain_valid`` and
``periodicity_valid``). It deliberately records ``refinement_validated=False``
because a second integration has not happened yet, and it makes no cross-model
comparison. This tool is the post-run evidence step for the remaining gates.

It reads four finished attempts and writes one auditable gate table:

* the cold acceptance lap (``--attempt``),
* the warm-started periodicity baseline (``--periodicity``),
* the tighter-step/rtol refinement of the same warm start (``--refinement``),
* the corrected planar 7DoF reference lap (``--planar-reference``).

Each gate is recorded with ``pass``, ``definition``, the attempt it came from,
the runner summary keys it was read from, and a flat ``evidence`` mapping. No
gate passes because a process returned: every gate names its measured evidence.
The plan's own gate names and booleans are also collected in a ``plan_gates``
block, because four of the seven plan names are not runner summary keys.

The table also records the sha256 of the four traces it derived evidence from
(``traces``) and the recorded-versus-working source hashes of both the cold
attempt and the planar reference (``*_source_hash_agreement``). The reference was
produced by the Task 8 runner before its final review fixes, so one of its 14
recorded source hashes differs from this worktree; that is a visible provenance
field, never a gate failure.

Refinement criteria are the plan's Task 9 row, not the planar checker's: lap
time difference <= 5 ms, maximum matched body-speed difference <= 0.05 m/s, and
every matched tire state within its own periodic tolerance. ``tools/gates/
check_iem24_forward_refinement.py`` is *not* used here: its column indices and
limits are the planar 13-column layout, so pointing it at a 39-column vertical
trace would compare ``v_z``/``heave`` as if they were ``yaw_rate``/steering.

Run from the repository root with ``PYTHONPATH=src:.``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

#: The plan's Task 9 acceptance-gate names, in plan order.
GATE_NAMES: tuple[str, ...] = (
    "completed",
    "contact_valid",
    "travel_valid",
    "table_domain_valid",
    "refinement_validated",
    "periodicity_valid",
    "cross_fidelity_reported",
)

#: Task 9 refinement criteria.
REFINEMENT_MAXIMUM_STEP_S = 0.0025
REFINEMENT_RTOL = 1.0e-6
REFINEMENT_LAP_TIME_TOLERANCE_S = 0.005
REFINEMENT_SPEED_TOLERANCE_MPS = 0.05
REFINEMENT_MATCHED_SAMPLES = 1017

#: The LUT metadata's bump-stop onset: a 10x wheel-rate rise from 29.8 mm with a
#: C1 smoothstep blend over 27.8-29.8 mm. A jounce beyond the blend start is a
#: bump-stop entry; the hard acceptance limit is the travel limit (31.8 mm).
BUMP_STOP_BLEND_START_M = 0.0278

#: Cross-fidelity speed-trace sample count written next to the gate table. The
#: tracked CSV uses this grid; the finer grid is recorded in the gate table too so
#: a document can cite a tracked number for either sampling density.
CROSS_FIDELITY_SAMPLES = 200
CROSS_FIDELITY_FINE_SAMPLES = 2001
#: Distance beyond which the two speed traces are reported as agreeing.
CROSS_FIDELITY_TAIL_DISTANCE_M = 508.0

#: Expected qualitative effects of the 14DoF rung, from the plan.
CROSS_FIDELITY_EXPECTED_EFFECTS = (
    "The vertical rung carries the 0.473 rigid-tire front LLTD and the measured roll-centre "
    "geometry, against the planar model's fixed 0.42: more lateral load transfer at the front "
    "axle, earlier front saturation and a looser rear balance. The transient tire layer adds "
    "slip, carcass-deformation and trail memory plus the damper transient, which the planar "
    "quasi-steady tire has no equivalent of. Both effects act on lateral grip, so the 14DoF and "
    "7DoF speed traces are expected to differ even though they share the steady UniTire "
    "coefficients, aero, powertrain and driver."
)


def sha256_file(path) -> str:
    """Return the lowercase SHA-256 hex digest of one file."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path) -> dict:
    """Return one JSON document as a dict."""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def summary_of(directory) -> dict:
    """Return one attempt's ``summary.json``."""
    return load_json(Path(directory) / "summary.json")


def read_trace(path) -> np.ndarray:
    """Return one traced attempt as an ``(n, 1 + state_size)`` array.

    The traces are large per-attempt artifacts and are gitignored; this tool is
    the step that turns them into the committed gate table (and records their
    sha256), so it must run in a worktree that still has them. The pinned-trace
    tests read the same files through this function.
    """
    trace = np.loadtxt(Path(path), delimiter=",", skiprows=1)
    if trace.ndim != 2:
        raise ValueError(f"{path}: trace must be a 2-D array")
    return trace


def trace_of(directory) -> np.ndarray:
    """Return one attempt's ``trace.csv`` as an array; see ``read_trace``."""
    return read_trace(Path(directory) / "trace.csv")


def trace_provenance(directory) -> dict:
    """Return the recorded identity of one attempt's trace artifact.

    The trace is gitignored, so a fresh checkout cannot re-derive the
    trace-based gate evidence; recording the digest here lets the evidence tests
    pin their re-derivation to exactly the trace the gate table was built from
    and skip (or fail) cleanly when it is absent or has changed.
    """
    path = Path(directory) / "trace.csv"
    record = {"trace": relative_attempt(path), "present": path.is_file(),
              "sha256": None, "bytes": None, "rows": None, "columns": None}
    if not record["present"]:
        return record
    trace = read_trace(path)
    record.update(sha256=sha256_file(path), bytes=int(path.stat().st_size),
                  rows=int(trace.shape[0]), columns=int(trace.shape[1]))
    return record


def relative_attempt(directory) -> str:
    """Return one attempt path relative to the repository root when possible."""
    resolved = Path(directory).resolve()
    try:
        return resolved.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def vertical_plant():
    """Return the vertical plant class (the state layout authority)."""
    from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
    return VerticalForwardPlant


def tolerance_by_name(plant_class) -> dict:
    """Return ``state name -> periodic tolerance`` for one plant class."""
    names = tuple(plant_class.state_names)
    indices = tuple(plant_class.periodic_indices)
    tolerances = np.asarray(plant_class.periodic_tolerances, dtype=float)
    if tolerances.shape != (len(indices),):
        raise ValueError("plant periodic tolerances must match its periodic indices")
    return {names[index]: float(tolerance)
            for index, tolerance in zip(indices, tolerances)}


def column(plant_class, name: str) -> int:
    """Return the trace column (time first) of one named plant state."""
    return 1 + tuple(plant_class.state_names).index(name)


def component_table(role: str) -> Path:
    """Return the data table of one assembly component role."""
    from tools.run.run_iem24_forward_driver import (
        VEHICLE, component_data_paths, component_xml,
    )
    paths = component_data_paths(component_xml(VEHICLE, role))
    if len(paths) != 1:
        raise ValueError(f"role {role!r} must reference exactly one data table")
    return REPOSITORY_ROOT / paths[0]


def load_corner_tables() -> dict:
    """Return the compiled corner LUT arrays (spring curves and domains)."""
    return dict(np.load(component_table("suspension/kinematics"), allow_pickle=False))


def load_damper_tables() -> dict:
    """Return the compiled damper LUT arrays (velocity axes)."""
    return dict(np.load(component_table("suspension/damper-front"), allow_pickle=False))


def body_speed(trace: np.ndarray, plant_class) -> np.ndarray:
    """Return the road-plane body speed of every trace sample."""
    return np.hypot(trace[:, column(plant_class, "v_x")],
                    trace[:, column(plant_class, "v_y")])


def matched_metrics(reference_trace, refined_trace, plant_class, samples: int,
                    tolerances: dict) -> dict:
    """Return the Task 9 refinement metrics of two traces on matched distance.

    Both traces must start at the same distance and share an initial state; the
    comparison samples a common ``[0, min(final s)]`` grid, so the two
    integrations are compared where they cover the same physical distance.
    """
    reference_trace = np.asarray(reference_trace, dtype=float)
    refined_trace = np.asarray(refined_trace, dtype=float)
    distance_column = column(plant_class, "s")
    last = min(float(reference_trace[-1, distance_column]),
               float(refined_trace[-1, distance_column]))
    distance = np.linspace(0., last, int(samples))

    def sample(trace, name):
        return np.interp(distance, trace[:, distance_column], trace[:, column(plant_class, name)])

    reference_speed = np.hypot(sample(reference_trace, "v_x"), sample(reference_trace, "v_y"))
    refined_speed = np.hypot(sample(refined_trace, "v_x"), sample(refined_trace, "v_y"))
    tire_prefixes = ("kappa_eff", "lateral_deformation", "trail_angle")
    per_state = {}
    tire_worst_name, tire_worst_ratio = None, -math.inf
    other_worst_name, other_worst_ratio = None, -math.inf
    for name, tolerance in sorted(tolerances.items()):
        difference = float(np.max(np.abs(sample(reference_trace, name)
                                        - sample(refined_trace, name))))
        ratio = difference / float(tolerance) if tolerance > 0. else None
        is_tire = name.startswith(tire_prefixes)
        per_state[name] = {"maximum_absolute_difference": difference,
                           "periodic_tolerance": float(tolerance),
                           "ratio": ratio, "tire_state": bool(is_tire)}
        if ratio is None:
            continue
        if is_tire and ratio > tire_worst_ratio:
            tire_worst_ratio, tire_worst_name = ratio, name
        if not is_tire and ratio > other_worst_ratio:
            other_worst_ratio, other_worst_name = ratio, name
    return {
        "distance": distance,
        "reference_speed": reference_speed,
        "refined_speed": refined_speed,
        "matched_distance_m": last,
        "matched_sample_count": int(samples),
        "maximum_body_speed_difference_mps": float(np.max(np.abs(reference_speed - refined_speed))),
        "body_speed_rmse_mps": float(np.sqrt(np.mean((reference_speed - refined_speed) ** 2))),
        "state_differences": per_state,
        "worst_tire_state": tire_worst_name,
        "worst_tire_state_ratio": (None if tire_worst_name is None else float(tire_worst_ratio)),
        "worst_other_state": other_worst_name,
        "worst_other_state_ratio": (None if other_worst_name is None
                                    else float(other_worst_ratio)),
    }


def gate(pass_value, definition: str, source_attempt: str, runner_keys: dict,
         evidence: dict) -> dict:
    """Assemble one gate record."""
    return {"pass": bool(pass_value), "definition": definition,
            "source_attempt": source_attempt, "runner_keys": dict(runner_keys),
            "evidence": dict(evidence)}


def completed_gate(directory, summary) -> dict:
    """Gate 1: the lap completed inside the chosen 14DoF wall budget."""
    elapsed = summary.get("elapsed_wall_seconds")
    requested = summary.get("requested_wall_seconds")
    within = (isinstance(elapsed, (int, float)) and isinstance(requested, (int, float))
              and float(elapsed) <= float(requested))
    return gate(
        summary.get("lap_completed") is True and summary.get("status") == "Completed" and within,
        "Lap completes within the user-selected 14DoF wall budget; there is no 300 s "
        "acceptance threshold, and elapsed time is recorded.",
        relative_attempt(directory),
        {"lap_completed": summary.get("lap_completed"), "status": summary.get("status")},
        {"lap_time_s": summary.get("lap_time"),
         "simulated_seconds": summary.get("simulated_seconds"),
         "track_length_m": summary.get("track_length"),
         "distance_completed_m": summary.get("distance_completed"),
         "requested_wall_seconds": requested,
         "solver_wall_budget_seconds": summary.get("solver_wall_budget_seconds"),
         "solver_elapsed_seconds": summary.get("solver_elapsed_seconds"),
         "elapsed_wall_seconds": elapsed,
         "peak_memory_bytes": summary.get("peak_memory_bytes"),
         "accepted_steps": summary.get("accepted_steps"),
         "evaluations": summary.get("evaluations"),
         "rejected_domain_trials": summary.get("rejected_domain_trials"),
         "within_requested_wall_seconds": within})


def contact_gate(directory, summary) -> dict:
    """Gate 2: non-negative tire loads on the accepted trajectory."""
    return gate(
        summary.get("contact_valid") is True,
        "Every accepted and midpoint state has tire load >= 0; lifted wheels are counted, "
        "not failed.",
        relative_attempt(directory),
        {"contact_valid": summary.get("contact_valid")},
        {"minimum_contact_load_n": summary.get("minimum_contact_load_n"),
         "contact_load_limit_n": 0.,
         "lifted_wheel_evaluations": summary.get("lifted_wheel_evaluations"),
         "out_of_range_evaluations": summary.get("out_of_range_evaluations"),
         "jounce_travel_valid": summary.get("jounce_travel_valid"),
         "lap_completed": summary.get("lap_completed")})


def bump_stop_evidence(directory, tables: dict) -> dict:
    """Return the bump-stop entry count and peak wheel force from the trace.

    Entry is ``|jounce| > 27.8 mm`` (the C1 blend start of the PROPOSED 10x
    bump-stop rise); the hard travel limit stays 31.8 mm. The peak wheel force is
    the compiled spring curve evaluated at the accepted extreme, which already
    includes the bump stop. When the BDF dense bounds are still present, the
    per-interval bounds are checked too, so a midpoint-only entry is caught.
    """
    from tools.run.run_iem24_forward_driver import jounce_indices
    plant_class = vertical_plant()
    # ``jounce_indices`` returns state indices; a trace row carries ``time`` as
    # column 0, so the CSV columns are one higher. The dense-bound array has one
    # column per state, so it keeps the state indices.
    state_indices = list(jounce_indices(plant_class))
    trace = trace_of(directory)
    jounce = trace[:, [1 + index for index in state_indices]]
    per_wheel_min = np.min(jounce, axis=0)
    per_wheel_max = np.max(jounce, axis=0)
    per_wheel_absmax = np.maximum(np.abs(per_wheel_min), np.abs(per_wheel_max))
    per_wheel_entries = np.count_nonzero(np.abs(jounce) > BUMP_STOP_BLEND_START_M, axis=0)
    bound_extreme = None
    bounds_path = Path(directory) / "dense_polynomials.npz"
    scope = "accepted trace samples only (dense bounds absent or unreadable)"
    if bounds_path.is_file():
        with np.load(bounds_path, allow_pickle=False) as data:
            bounds = np.asarray(data["bounds"], dtype=float)[:, state_indices, :]
        bound_extreme = float(np.max(np.abs(bounds)))
        scope = "accepted trace samples and the per-interval dense bounds"
    front, rear = tables["spring_jounce_front"], tables["spring_jounce_rear"]
    force_at = (
        lambda index, value: float(np.interp(
            value, front if index < 2 else rear,
            tables["spring_force_front"] if index < 2 else tables["spring_force_rear"]))
    )
    peak_compression = max(force_at(index, max(float(per_wheel_max[index]), 0.))
                           for index in range(4))
    minimum_wheel = min(force_at(index, min(float(per_wheel_min[index]), 0.))
                        for index in range(4))
    return {
        "bump_stop_blend_start_m": BUMP_STOP_BLEND_START_M,
        "bump_stop_entry_count": int(np.count_nonzero(per_wheel_entries)),
        "bump_stop_entry_count_per_wheel": [int(value) for value in per_wheel_entries],
        "bump_stop_peak_wheel_force_n": peak_compression,
        "minimum_wheel_force_n": minimum_wheel,
        "jounce_extremes_per_wheel_m": [[float(per_wheel_min[index]), float(per_wheel_max[index])]
                                        for index in range(4)],
        "maximum_absolute_jounce_m": float(np.max(per_wheel_absmax)),
        "maximum_absolute_jounce_dense_bound_m": bound_extreme,
        "bump_stop_scope": scope}


def travel_gate(directory, summary, tables: dict) -> dict:
    """Gate 3: jounce inside the travel limits, with bump-stop reporting."""
    evidence = {"jounce_travel_limits_m": summary.get("jounce_travel_limits_m"),
                "jounce_accepted_minimum_m": summary.get("jounce_accepted_minimum_m"),
                "jounce_accepted_maximum_m": summary.get("jounce_accepted_maximum_m"),
                "jounce_gate_scope": summary.get("jounce_gate_scope"),
                "jounce_evaluation_minimum_m": summary.get("jounce_evaluation_minimum_m"),
                "jounce_evaluation_maximum_m": summary.get("jounce_evaluation_maximum_m"),
                "dense_midpoint_samples": summary.get("dense_midpoint_samples")}
    try:
        evidence.update(bump_stop_evidence(directory, tables))
        evidence["bump_stop_evidence_error"] = None
    except (OSError, ValueError, KeyError) as error:  # trace or bounds unavailable
        evidence["bump_stop_evidence_error"] = f"{type(error).__name__}: {error}"
    return gate(
        summary.get("jounce_travel_valid") is True,
        "All accepted and midpoint states keep jounce inside +/-31.8 mm. Bump-stop entry "
        "before the limit is reported, not failed; a state beyond the limit fails.",
        relative_attempt(directory),
        {"jounce_travel_valid": summary.get("jounce_travel_valid")},
        evidence)


def table_domain_gate(directory, summary, tables: dict, damper: dict) -> dict:
    """Gate 4: no query left a compiled damper, spring or roll-centre table."""
    plant_class = vertical_plant()
    trace = trace_of(directory)
    maximum_roll_deg = float(np.degrees(np.max(np.abs(trace[:, column(plant_class, "roll")]))))
    damper_limit = max(float(np.max(np.abs(damper["velocity_compression"]))),
                       float(np.max(np.abs(damper["velocity_rebound"]))))
    maximum_damper = summary.get("maximum_damper_velocity")
    roll_limit_deg = 5.0
    has_network = "network_state_valid" in summary
    network_valid = summary.get("network_state_valid", True)
    return gate(
        summary.get("out_of_range_evaluations") == 0 and network_valid is True,
        ("out_of_range_evaluations == 0 for dampers, springs and the roll-centre table, and "
         "every suspension-network state (gas volume) inside its physical bounds."
         if has_network else
         "out_of_range_evaluations == 0 for dampers, springs and the roll-centre table."),
        relative_attempt(directory),
        {"out_of_range_evaluations": summary.get("out_of_range_evaluations")},
        {"out_of_range_evaluations": summary.get("out_of_range_evaluations"),
         "out_of_range_scope": ("every plant evaluation, including rejected Newton trials: "
                                "damper-velocity exits plus spring, bump-stop and "
                                "jounce-steer DomainError conversions"),
         "maximum_damper_velocity_mps": maximum_damper,
         "damper_table_velocity_limit_mps": damper_limit,
         "damper_velocity_headroom_fraction": (None if maximum_damper is None
                                               else 1. - float(maximum_damper) / damper_limit),
         "spring_curve_jounce_domain_m": [float(np.min(tables["spring_jounce_front"])),
                                          float(np.max(tables["spring_jounce_front"]))],
         "jounce_steer_jounce_domain_m": [float(np.min(tables["jounce_steer_jounce_front"])),
                                          float(np.max(tables["jounce_steer_jounce_front"]))],
         "jounce_accepted_extremes_m": [summary.get("jounce_accepted_minimum_m"),
                                        summary.get("jounce_accepted_maximum_m")],
         "jounce_steer_baseline_toe_rad": float(np.max(np.abs(
             tables["jounce_steer_toe_front"]))),
         **({"network_state_names": summary.get("network_state_names"),
             "network_state_minimum": summary.get("network_state_minimum"),
             "network_state_maximum": summary.get("network_state_maximum"),
             "network_state_bounds": summary.get("network_state_bounds"),
             "network_state_valid": network_valid} if has_network else {}),
         "roll_centre_table_limit_deg": roll_limit_deg,
         "maximum_absolute_roll_deg": maximum_roll_deg,
         "roll_centre_query_clipped": bool(maximum_roll_deg > roll_limit_deg),
         "definition": ("the evaluator holds the roll-centre height constant above the 1 g "
                        "point and clips its query at the committed 5 deg table maximum; a "
                        "maximum |roll| below 5 deg means the clip never bound")})


def refinement_gate(reference_directory, refined_directory) -> dict:
    """Gate 5: the tighter-step/rtol run reproduces the warm baseline."""
    plant_class = vertical_plant()
    tolerances = tolerance_by_name(plant_class)
    reference_summary = summary_of(reference_directory)
    refined_summary = summary_of(refined_directory)
    reference_trace = trace_of(reference_directory)
    refined_trace = trace_of(refined_directory)
    metrics = matched_metrics(reference_trace, refined_trace, plant_class,
                             REFINEMENT_MATCHED_SAMPLES, tolerances)
    reference_lap = reference_summary.get("lap_time")
    refined_lap = refined_summary.get("lap_time")
    lap_difference = (None if reference_lap is None or refined_lap is None
                      else abs(float(reference_lap) - float(refined_lap)))
    tire_ratio = metrics["worst_tire_state_ratio"]
    same_initial = bool(np.array_equal(reference_trace[0, 1:], refined_trace[0, 1:]))
    same_sources = reference_summary.get("source_hashes") == refined_summary.get("source_hashes")
    tighter = bool(float(refined_summary["settings"]["maximum_step"])
                   == REFINEMENT_MAXIMUM_STEP_S
                   and float(refined_summary["settings"]["rtol"]) == REFINEMENT_RTOL
                   and float(refined_summary["settings"]["rtol"])
                   < float(reference_summary["settings"]["rtol"]))
    complete = (reference_summary.get("lap_completed") is True
                and refined_summary.get("lap_completed") is True)
    passed = bool(complete and same_initial and same_sources and tighter
                  and lap_difference is not None
                  and lap_difference <= REFINEMENT_LAP_TIME_TOLERANCE_S
                  and metrics["maximum_body_speed_difference_mps"]
                  <= REFINEMENT_SPEED_TOLERANCE_MPS
                  and tire_ratio is not None and tire_ratio <= 1.)
    return gate(
        passed,
        "A second run at maximum_step 0.0025 s and rtol 1e-6 from the same warm start: lap "
        "time difference <= 5 ms, max |dv| <= 0.05 m/s at matched s, and every matched tire "
        "state within its own periodic tolerance.",
        relative_attempt(refined_directory),
        {"refinement_validated": refined_summary.get("refinement_validated")},
        {"reference_attempt": relative_attempt(reference_directory),
         "refined_attempt": relative_attempt(refined_directory),
         "reference_lap_time_s": reference_lap,
         "refined_lap_time_s": refined_lap,
         "lap_time_difference_s": lap_difference,
         "lap_time_limit_s": REFINEMENT_LAP_TIME_TOLERANCE_S,
         "maximum_body_speed_difference_mps": metrics["maximum_body_speed_difference_mps"],
         "body_speed_rmse_mps": metrics["body_speed_rmse_mps"],
         "body_speed_limit_mps": REFINEMENT_SPEED_TOLERANCE_MPS,
         "matched_distance_m": metrics["matched_distance_m"],
         "matched_sample_count": metrics["matched_sample_count"],
         "worst_tire_state": metrics["worst_tire_state"],
         "worst_tire_state_ratio": tire_ratio,
         "worst_other_state": metrics["worst_other_state"],
         "worst_other_state_ratio": metrics["worst_other_state_ratio"],
         "state_differences": metrics["state_differences"],
         "refined_maximum_step_s": refined_summary["settings"]["maximum_step"],
         "reference_maximum_step_s": reference_summary["settings"]["maximum_step"],
         "refined_rtol": refined_summary["settings"]["rtol"],
         "reference_rtol": reference_summary["settings"]["rtol"],
         "same_initial_state": same_initial,
         "same_source_hashes": same_sources,
         "both_laps_completed": complete})


def periodicity_gate(directory, summary) -> dict:
    """Gate 6: the warm-started lap closes on the plant's periodic tolerances."""
    plant_class = vertical_plant()
    names = tuple(plant_class.state_names)
    indices = tuple(plant_class.periodic_indices)
    by_name = tolerance_by_name(plant_class)
    delta = np.asarray(summary.get("periodic_state_delta", []), dtype=float)
    tolerances = np.asarray(summary.get("periodic_state_tolerances", []), dtype=float)
    if delta.shape != tolerances.shape or delta.shape != (len(indices),):
        raise ValueError("periodic delta and tolerances must match the plant's periodic layout")
    if np.any(tolerances <= 0.):
        raise ValueError("periodic tolerances must be positive to form a ratio")
    recorded = np.array([by_name[names[index]] for index in indices], dtype=float)
    ratio = np.abs(delta) / tolerances
    worst = int(np.argmax(ratio))
    worst_name = names[indices[worst]]
    vertical = {name: by_name[name]
                for name in ("n", "heave", "roll", "pitch", "roll_rate", "pitch_rate",
                             "jounce_fl", "jounce_rate_fl")}
    tire = {name: by_name[f"{name}_fl"]
            for name in ("kappa_eff", "lateral_deformation", "trail_angle")}
    return gate(
        summary.get("periodicity_valid") is True and float(np.max(ratio)) <= 1.,
        "The same warm-start protocol as 7DoF (final physical state and pedal of the cold lap, "
        "path distance reset) with the plant's own periodic tolerances.",
        relative_attempt(directory),
        {"periodicity_valid": summary.get("periodicity_valid")},
        {"lap_completed": summary.get("lap_completed"),
         "lap_time_s": summary.get("lap_time"),
         "maximum_tolerance_ratio": float(np.max(ratio)),
         "worst_state": worst_name,
         "worst_state_absolute_delta": float(abs(delta[worst])),
         "worst_state_tolerance": float(tolerances[worst]),
         "periodic_state_count": int(len(indices)),
         "verified_state_count": int(np.count_nonzero(ratio <= 1.)),
         "vertical_tolerances": vertical,
         "tire_state_tolerances": tire,
         "recorded_tolerances_match_plant": bool(np.array_equal(recorded, tolerances)),
         "warm_start_source": summary.get("warm_start_source")})


def cross_fidelity_gate(directory, reference_directory, output_directory) -> dict:
    """Gate 7: the lap and speed trace are compared with the corrected 7DoF lap."""
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
    plant_class = vertical_plant()
    summary = summary_of(directory)
    reference_summary = summary_of(reference_directory)
    trace = trace_of(directory)
    reference_trace = trace_of(reference_directory)
    if trace.shape[1] != 1 + plant_class.state_size:
        raise ValueError("the 14DoF attempt trace does not match the vertical plant layout")
    if reference_trace.shape[1] != 1 + PlanarForwardPlant.state_size:
        raise ValueError("the reference trace does not match the planar plant layout")
    last = min(float(trace[-1, column(plant_class, "s")]),
               float(reference_trace[-1, column(PlanarForwardPlant, "s")]))

    def comparison(sample_count: int) -> dict:
        """Return the speed comparison of one matched-distance sampling grid."""
        distance = np.linspace(0., last, int(sample_count))

        def sampled(source_trace, source_plant, name):
            return np.interp(distance, source_trace[:, column(source_plant, "s")],
                             source_trace[:, column(source_plant, name)])

        vertical_speed = np.hypot(sampled(trace, plant_class, "v_x"),
                                  sampled(trace, plant_class, "v_y"))
        planar_speed = np.hypot(sampled(reference_trace, PlanarForwardPlant, "v_x"),
                                sampled(reference_trace, PlanarForwardPlant, "v_y"))
        difference = vertical_speed - planar_speed
        tail = np.abs(difference)[distance > CROSS_FIDELITY_TAIL_DISTANCE_M]
        return {
            "distance": distance, "vertical_speed": vertical_speed,
            "planar_speed": planar_speed, "difference": difference,
            "sample_count": int(sample_count),
            "rmse_mps": float(np.sqrt(np.mean(difference ** 2))),
            "maximum_mps": float(np.max(np.abs(difference))),
            "maximum_distance_m": float(distance[int(np.argmax(np.abs(difference)))]),
            "tail_maximum_mps": (None if tail.size == 0 else float(np.max(tail))),
        }

    tracked = comparison(CROSS_FIDELITY_SAMPLES)
    fine = comparison(CROSS_FIDELITY_FINE_SAMPLES)
    distance = tracked["distance"]
    vertical_speed, planar_speed = tracked["vertical_speed"], tracked["planar_speed"]
    difference = tracked["difference"]
    (output_directory / "cross_fidelity_speed.csv").write_text(
        "distance_m,vertical_14dof_speed_mps,planar_7dof_speed_mps,delta_mps\n"
        + "".join(f"{s:.6f},{v:.9f},{p:.9f},{d:.9f}\n"
                  for s, v, p, d in zip(distance, vertical_speed, planar_speed, difference)),
        encoding="utf-8")
    vertical_lap = summary.get("lap_time")
    planar_lap = reference_summary.get("lap_time")
    return gate(
        True,
        "Report item, not a pass/fail gate: lap time and speed trace compared with the "
        "corrected 7DoF lap. `pass` means the comparison was computed and recorded.",
        relative_attempt(directory),
        {},
        {"reference_attempt": relative_attempt(reference_directory),
         "reference_model": reference_summary.get("model", "planar-7dof"),
         "reference_lap_time_s": planar_lap,
         "reference_usable": reference_summary.get("usable"),
         "reference_path_tracking_valid": reference_summary.get("path_tracking_valid"),
         "reference_maximum_lateral_error_m": reference_summary.get("maximum_lateral_error"),
         "vertical_lap_time_s": vertical_lap,
         "lap_time_delta_s": (None if vertical_lap is None or planar_lap is None
                              else float(vertical_lap) - float(planar_lap)),
         "matched_distance_m": last,
         "matched_sample_count": tracked["sample_count"],
         "speed_difference_rmse_mps": tracked["rmse_mps"],
         "speed_difference_maximum_mps": tracked["maximum_mps"],
         "speed_difference_maximum_distance_m": tracked["maximum_distance_m"],
         "fine_grid_sample_count": fine["sample_count"],
         "fine_grid_speed_difference_rmse_mps": fine["rmse_mps"],
         "fine_grid_speed_difference_maximum_mps": fine["maximum_mps"],
         "fine_grid_speed_difference_maximum_distance_m": fine["maximum_distance_m"],
         "tail_distance_threshold_m": CROSS_FIDELITY_TAIL_DISTANCE_M,
         "tail_maximum_speed_difference_mps": tracked["tail_maximum_mps"],
         "fine_grid_tail_maximum_speed_difference_mps": fine["tail_maximum_mps"],
         "speed_trace_file": "cross_fidelity_speed.csv",
         "expected_effects": CROSS_FIDELITY_EXPECTED_EFFECTS,
         "both_laps_completed": (summary.get("lap_completed") is True
                                 and reference_summary.get("lap_completed") is True)})


def source_hash_agreement(summary) -> dict:
    """Return the recorded-vs-working source hash comparison of one attempt.

    Every recorded source is listed with both digests, so the gate table shows
    exactly which sources differ from the working tree. A difference is a
    provenance fact, not automatically a failure: the corrected planar reference
    was produced before its runner's final review fixes, so one of its 14 sources
    differs while the twelve model/source files it shares with this worktree do
    not.
    """
    entries, mismatches = [], []
    for name, recorded in sorted(summary.get("source_hashes", {}).items()):
        path = Path(name)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        measured = sha256_file(path) if path.is_file() else None
        matches = measured == recorded
        entries.append({"source": name, "recorded_sha256": recorded,
                        "working_sha256": measured, "matches": matches})
        if not matches:
            mismatches.append({"source": name,
                               "reason": "missing" if measured is None else "sha256 differs"})
    return {"sources_checked": len(entries), "all_sources_match": not mismatches,
            "mismatches": mismatches, "sources": entries}


def build(reference_attempt, periodicity_attempt, refinement_attempt,
          planar_reference) -> dict:
    """Return the full Task 9 gate table of one 14DoF acceptance run."""
    reference_attempt = Path(reference_attempt)
    periodicity_attempt = Path(periodicity_attempt)
    refinement_attempt = Path(refinement_attempt)
    planar_reference = Path(planar_reference)
    summary = summary_of(reference_attempt)
    if summary.get("model") != "14dof-simple":
        raise ValueError(f"{reference_attempt}: --attempt must be a 14dof-simple run")
    tables = load_corner_tables()
    damper = load_damper_tables()
    gates = {
        "completed": completed_gate(reference_attempt, summary),
        "contact_valid": contact_gate(reference_attempt, summary),
        "travel_valid": travel_gate(reference_attempt, summary, tables),
        "table_domain_valid": table_domain_gate(reference_attempt, summary, tables, damper),
        "refinement_validated": refinement_gate(periodicity_attempt, refinement_attempt),
        "periodicity_valid": periodicity_gate(periodicity_attempt, summary_of(periodicity_attempt)),
        "cross_fidelity_reported": cross_fidelity_gate(reference_attempt, planar_reference,
                                                       reference_attempt),
    }
    order = list(GATE_NAMES)
    if set(order) != set(gates):
        raise ValueError("gate table does not match GATE_NAMES")
    reference_summary = summary_of(planar_reference)
    return {
        "task": "Task 9 - vertical 14DoF lap, validation gates and report",
        "attempt": relative_attempt(reference_attempt),
        "periodicity_attempt": relative_attempt(periodicity_attempt),
        "refinement_attempt": relative_attempt(refinement_attempt),
        "planar_reference_attempt": relative_attempt(planar_reference),
        # The plan's Task 9 gate table asks for each gate as a boolean in the
        # attempt record. Four of the seven names are not runner summary keys
        # (completed, travel_valid, table_domain_valid, cross_fidelity_reported)
        # and `refinement_validated` needs a second integration the runner cannot
        # run, so the runner's summary alone cannot carry the plan's table. This
        # block is that table, one boolean per plan gate name.
        "plan_gates": {name: gates[name]["pass"] for name in order},
        "gate_source_mapping": {
            "completed": "summary.lap_completed + summary.status + elapsed <= requested wall",
            "contact_valid": "summary.contact_valid",
            "travel_valid": "summary.jounce_travel_valid (the runner's key for the plan's "
                            "travel_valid)",
            "table_domain_valid": "summary.out_of_range_evaluations == 0",
            "refinement_validated": "paired warm baseline vs refined traces (this tool)",
            "periodicity_valid": "summary.periodicity_valid of the warm-started attempt",
            "cross_fidelity_reported": "vertical vs corrected planar 7DoF traces (this tool)"},
        "gate_tool": {"path": relative_attempt(Path(__file__)),
                      "sha256": sha256_file(Path(__file__))},
        # Which runner summary keys the plan's table does not have, and why the
        # table lives here instead.
        "plan_gate_note": (
            "The plan's Task 9 table names seven gates; summary.json carries "
            "contact_valid and periodicity_valid under the same names, "
            "refinement_validated as a hard-coded false, and no key at all for "
            "completed, travel_valid, table_domain_valid or cross_fidelity_reported "
            "(the runner records lap_completed, jounce_travel_valid and "
            "out_of_range_evaluations instead). `plan_gates` is the plan's table "
            "with its own names."),
        "source_hash_agreement": source_hash_agreement(summary),
        "planar_reference_source_hash_agreement": source_hash_agreement(reference_summary),
        # Identity of the four gitignored trace artifacts the trace-derived
        # evidence comes from, so a fresh checkout can tell a missing trace from a
        # changed one and the pinned-trace tests can re-derive the same numbers.
        "traces": {
            "attempt": trace_provenance(reference_attempt),
            "periodicity_attempt": trace_provenance(periodicity_attempt),
            "refinement_attempt": trace_provenance(refinement_attempt),
            "planar_reference_attempt": trace_provenance(planar_reference)},
        "gates": {name: gates[name] for name in order},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", type=Path, required=True,
                        help="cold 14dof-simple acceptance attempt directory")
    parser.add_argument("--periodicity", type=Path, required=True,
                        help="warm-started periodicity baseline attempt directory")
    parser.add_argument("--refinement", type=Path, required=True,
                        help="tighter-step/rtol refinement of the same warm start")
    parser.add_argument("--planar-reference", type=Path, required=True,
                        help="corrected planar 7DoF reference attempt directory")
    parser.add_argument("--output", type=Path,
                        help="gate table path (default: <attempt>/gates.json)")
    args = parser.parse_args(argv)
    destination = args.output or (args.attempt / "gates.json")
    table = build(args.attempt, args.periodicity, args.refinement, args.planar_reference)
    # Record the post-run refinement verdict in the refined attempt's own summary,
    # exactly as tools/gates/check_iem24_forward_refinement.py does for the planar
    # rung: the runner cannot know the outcome of an independent second
    # integration, so it always writes refinement_validated=false. Only
    # refinement_validated is written; the between-step flag is left exactly as the
    # run recorded it, so `usable` stays false unless every other gate on that
    # attempt also passes.
    from tools.run.run_iem24_forward_driver import acceptance_gate
    refined_path = Path(args.refinement) / "summary.json"
    refined = load_json(refined_path)
    refined_gate = table["gates"]["refinement_validated"]
    refined["refinement_validated"] = refined_gate["pass"]
    refined["refinement_evidence"] = destination.name
    refined["refinement_reference_attempt"] = refined_gate["evidence"]["reference_attempt"]
    refined["usable"] = acceptance_gate(refined)
    refined_path.write_text(json.dumps(refined, indent=2, allow_nan=False) + "\n",
                            encoding="utf-8")
    # The gate table is written after that update and repeats the certified value,
    # so the recorded runner key is what the refined summary now holds. Writing the
    # pre-update value would make the table disagree with the file on its first run
    # over fresh attempts (it agreed only because an earlier run had already
    # certified them). Re-running over a certified attempt is idempotent.
    refined_gate["runner_keys"]["refinement_validated"] = refined_gate["pass"]
    refined_gate["runner_key_origin"] = (
        "the runner always writes refinement_validated=false (a second integration has "
        "not happened yet); this tool certifies it in the refined attempt's summary.json")
    destination.write_text(json.dumps(table, indent=2, allow_nan=False) + "\n",
                           encoding="utf-8")
    for name in GATE_NAMES:
        record = table["gates"][name]
        print(f"{name}: {'PASS' if record['pass'] else 'FAIL'}")
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
