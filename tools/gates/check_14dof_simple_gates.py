#!/usr/bin/env python3
"""Evaluate the Task 9 vertical 14DoF acceptance gates from finished attempts.

The forward runner records the gates it can decide in-run (``lap_completed``,
``contact_valid``, ``jounce_travel_valid`` for the plan's ``travel_valid``,
``out_of_range_evaluations`` for ``table_domain_valid`` and
``periodicity_valid``). This tool collects them, with their measured evidence,
into one auditable gate table. The between-step and refinement checks were
removed by user decision on 2026-11-04.

It reads three finished attempts:

* the cold acceptance lap (``--attempt``),
* the warm-started periodicity baseline (``--periodicity``),
* the planar 7DoF reference lap (``--planar-reference``), for a report-only
  speed comparison.

Each gate is recorded with ``pass``, ``definition``, the attempt it came from,
the runner summary keys it was read from, and a flat ``evidence`` mapping. The
cross-fidelity comparison is a report item: it is recorded under ``reports``,
not under ``gates``, so it can never be read as a pass.

Table-derived evidence (spring and damper domains) is read from the attempt's
own ``source_snapshot``, so a later model change cannot re-interpret an old run.
A recorded-versus-working source hash difference is reported as provenance.

Run from the repository root with ``PYTHONPATH=src:.``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

#: The plan's Task 9 acceptance-gate names, in plan order.
GATE_NAMES: tuple[str, ...] = (
    "completed",
    "contact_valid",
    "travel_valid",
    "table_domain_valid",
    "periodicity_valid",
)

#: Historical bump-stop blend onset in the descriptive LUT data; not a
#: mechanical travel or acceptance limit under the current unlimited policy.
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
    "The vertical rung derives tire load transfer from its recorded suspension, roll-centre "
    "geometry and unsprung masses, against the planar model's fixed distribution. A historical "
    "0.473 rigid-tire LLTD is not an active-strut baseline; load-transfer differences must be "
    "measured for each recorded setup, not inferred from elastic roll stiffness alone. The transient tire layer adds "
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


def snapshot_table(directory, name: str) -> Path:
    """Return one data table from the attempt's own ``source_snapshot``.

    The runner copies every hashed source by base name, so the snapshot holds
    the exact tables the run used. Reading them here, not from the working tree,
    keeps the verdict tied to the run.
    """
    path = Path(directory) / "source_snapshot" / name
    if not path.is_file():
        raise FileNotFoundError(f"{path}: table is not in the attempt's source snapshot")
    return path


def _model_input_name(summary, key: str) -> str:
    """Return the base name of one recorded model input."""
    value = summary.get("model_inputs", {}).get(key)
    if value is None:
        raise KeyError(f"summary has no model input {key!r}")
    return Path(value).name


def load_corner_tables(directory, summary) -> dict:
    """Return the corner LUT arrays the run used (spring curves and domains)."""
    name = _model_input_name(summary, "suspension_kinematics_data")
    return dict(np.load(snapshot_table(directory, name), allow_pickle=False))


def load_damper_tables(directory) -> dict:
    """Return the union velocity range of every damper table the run used.

    The active parallel strut suspension has one table per strut family; the
    legacy layout has front and rear corner tables. Both are covered by taking
    every damper ``*.npz`` in the snapshot that holds velocity axes.
    """
    compression, rebound = [], []
    for path in sorted((Path(directory) / "source_snapshot").glob("*.npz")):
        with np.load(path, allow_pickle=False) as data:
            if "velocity_compression" in data and "velocity_rebound" in data:
                compression.append(np.asarray(data["velocity_compression"], dtype=float))
                rebound.append(np.asarray(data["velocity_rebound"], dtype=float))
    if not compression:
        raise FileNotFoundError(f"{directory}: no damper table in the source snapshot")
    return {"velocity_compression": np.concatenate(compression),
            "velocity_rebound": np.concatenate(rebound)}


def body_speed(trace: np.ndarray, plant_class) -> np.ndarray:
    """Return the road-plane body speed of every trace sample."""
    return np.hypot(trace[:, column(plant_class, "v_x")],
                    trace[:, column(plant_class, "v_y")])


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

    Entry is ``|jounce| > 27.8 mm`` (the C1 blend start in the historical LUT's
    PROPOSED 10x spring-rate rise); this is descriptive only, not a travel gate.
    The peak wheel force is
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
    """Gate 3: finite jounce diagnostics and bump-stop reporting.

    Current unlimited-policy summaries pass without a travel test. Historical
    summaries lacking the policy retain their original bounded interpretation.
    """
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
    unlimited_policy = summary.get("suspension_travel_policy") == "unlimited"
    unlimited = (unlimited_policy and summary.get("suspension_travel_enforced") is False)
    finite_jounce = all(isinstance(summary.get(key), (int, float))
                        and np.isfinite(summary[key])
                        for key in ("jounce_accepted_minimum_m", "jounce_accepted_maximum_m"))
    passed = ((unlimited and summary.get("jounce_travel_valid") is True and finite_jounce
               and float(summary["jounce_accepted_minimum_m"]) <= float(summary["jounce_accepted_maximum_m"]))
              if unlimited_policy else summary.get("jounce_travel_valid") is True)
    definition = ("Suspension displacement is unlimited; jounce_travel_valid confirms finite accepted jounce. "
                  "Damper and other physical-domain checks remain separate." if unlimited else
                  "Historical interpretation: accepted and midpoint jounce stays inside recorded travel limits.")
    # Cite only the runner keys the summary records; a historical summary has no
    # travel-policy keys, and citing them as None would claim a value it never held.
    runner_keys = {key: summary[key] for key in
                   ("jounce_travel_valid", "suspension_travel_policy",
                    "suspension_travel_enforced") if key in summary}
    return gate(passed, definition, relative_attempt(directory), runner_keys, evidence)


def table_domain_gate(directory, summary, tables: dict, damper: dict) -> dict:
    """Gate 4: finite diagnostics for unlimited travel; preserve historical bounded policy."""
    plant_class = vertical_plant()
    trace = trace_of(directory)
    maximum_roll_deg = float(np.degrees(np.max(np.abs(trace[:, column(plant_class, "roll")]))))
    damper_limit = max(float(np.max(np.abs(damper["velocity_compression"]))),
                       float(np.max(np.abs(damper["velocity_rebound"]))))
    maximum_damper = summary.get("maximum_damper_velocity")
    roll_limit_deg = 5.0
    has_network = "network_state_valid" in summary
    network_valid = summary.get("network_state_valid", True)
    unlimited = (summary.get("suspension_travel_policy") == "unlimited" and summary.get("suspension_travel_enforced") is False)
    diagnostics = [maximum_roll_deg, maximum_damper,
                   summary.get("jounce_accepted_minimum_m"),
                   summary.get("jounce_accepted_maximum_m")]
    finite_diagnostics = all(isinstance(v, (int, float, np.number))
                             and not isinstance(v, (bool, np.bool_)) and np.isfinite(v)
                             for v in diagnostics)
    ordered_jounce = (finite_diagnostics and diagnostics[2] <= diagnostics[3])
    domain_valid = (bool(ordered_jounce) if unlimited
                    else summary.get("out_of_range_evaluations") == 0)
    domain_text = ("Suspension travel and damper-table extension are descriptive, not physical limits; "
                   "reported roll, damper velocity and ordered jounce extrema must be finite."
                   if unlimited else "Historical interpretation: no query left recorded damper, spring, or roll-centre table domains.")
    if has_network:
        domain_text += " Every suspension-network state remains within physical bounds."
    return gate(
        domain_valid and network_valid is True,
        domain_text,
        relative_attempt(directory),
        {"out_of_range_evaluations": summary.get("out_of_range_evaluations"),
         **({"maximum_damper_velocity": maximum_damper,
             "jounce_accepted_minimum_m": summary.get("jounce_accepted_minimum_m"),
             "jounce_accepted_maximum_m": summary.get("jounce_accepted_maximum_m")}
            if unlimited else {})},
        {"out_of_range_evaluations": summary.get("out_of_range_evaluations"),
          "out_of_range_scope": ("diagnostic table-extension count only; no suspension travel or damper-table limits enforced" if unlimited else "historical table-domain counter interpretation"),
          "jounce_table_ranges_scope": ("design/table knot metadata only, not mechanical travel limits" if unlimited else "historical table-domain metadata"),
          "maximum_damper_velocity_mps": maximum_damper,
          "damper_table_velocity_limit_mps": damper_limit,
          "damper_velocity_headroom_fraction": (None if maximum_damper is None
                                                else 1. - float(maximum_damper) / damper_limit),
          "spring_design_knot_range_m": [float(np.min(tables["spring_jounce_front"])),
                                            float(np.max(tables["spring_jounce_front"]))],
          "spring_curve_jounce_domain_m": [float(np.min(tables["spring_jounce_front"])),
                                           float(np.max(tables["spring_jounce_front"]))],
          "jounce_steer_design_knot_range_m": [float(np.min(tables["jounce_steer_jounce_front"])),
                                                  float(np.max(tables["jounce_steer_jounce_front"]))],
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


def cross_fidelity_report(directory, reference_directory, output_directory) -> dict:
    """Report: the lap and speed trace compared with the planar 7DoF lap. Not a gate."""
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
    return {
        "definition": ("Report item, not a gate: lap time and speed trace compared with the "
                       "planar 7DoF lap. It has no pass value."),
        "source_attempt": relative_attempt(directory),
        "evidence": {"reference_attempt": relative_attempt(reference_directory),
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
                                 and reference_summary.get("lap_completed") is True)}}


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


def build(reference_attempt, periodicity_attempt, planar_reference) -> dict:
    """Return the full vertical 14DoF gate table of one acceptance run."""
    reference_attempt = Path(reference_attempt)
    periodicity_attempt = Path(periodicity_attempt)
    planar_reference = Path(planar_reference)
    summary = summary_of(reference_attempt)
    if summary.get("model") != "14dof-simple":
        raise ValueError(f"{reference_attempt}: --attempt must be a 14dof-simple run")
    tables = load_corner_tables(reference_attempt, summary)
    damper = load_damper_tables(reference_attempt)
    gates = {
        "completed": completed_gate(reference_attempt, summary),
        "contact_valid": contact_gate(reference_attempt, summary),
        "travel_valid": travel_gate(reference_attempt, summary, tables),
        "table_domain_valid": table_domain_gate(reference_attempt, summary, tables, damper),
        "periodicity_valid": periodicity_gate(periodicity_attempt, summary_of(periodicity_attempt)),
    }
    order = list(GATE_NAMES)
    if set(order) != set(gates):
        raise ValueError("gate table does not match GATE_NAMES")
    reference_summary = summary_of(planar_reference)
    return {
        "task": "vertical 14DoF lap acceptance gates",
        "attempt": relative_attempt(reference_attempt),
        "periodicity_attempt": relative_attempt(periodicity_attempt),
        "planar_reference_attempt": relative_attempt(planar_reference),
        "plan_gates": {name: gates[name]["pass"] for name in order},
        "all_gates_pass": all(gates[name]["pass"] for name in order),
        "gate_source_mapping": {
            "completed": "summary.lap_completed + summary.status + elapsed <= requested wall",
            "contact_valid": "summary.contact_valid",
            "travel_valid": "summary.jounce_travel_valid (finite jounce; travel is unlimited)",
            "table_domain_valid": "Unlimited policy: finite ordered diagnostics and valid network state; historical bounded policy: out_of_range_evaluations == 0",
            "periodicity_valid": "summary.periodicity_valid of the warm-started attempt"},
        "removed_gates": {
            "refinement_validated": "removed by user decision 2026-11-04",
            "between_step_validation_complete": "removed by user decision 2026-11-04"},
        "gate_tool": {"path": relative_attempt(Path(__file__)),
                      "sha256": sha256_file(Path(__file__))},
        "source_hash_agreement": source_hash_agreement(summary),
        "planar_reference_source_hash_agreement": source_hash_agreement(reference_summary),
        "traces": {
            "attempt": trace_provenance(reference_attempt),
            "periodicity_attempt": trace_provenance(periodicity_attempt),
            "planar_reference_attempt": trace_provenance(planar_reference)},
        "gates": {name: gates[name] for name in order},
        "reports": {"cross_fidelity": cross_fidelity_report(
            reference_attempt, planar_reference, reference_attempt)},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", type=Path, required=True,
                        help="cold 14dof-simple acceptance attempt directory")
    parser.add_argument("--periodicity", type=Path, required=True,
                        help="warm-started periodicity baseline attempt directory")
    parser.add_argument("--planar-reference", type=Path, required=True,
                        help="planar 7DoF reference attempt directory (report only)")
    parser.add_argument("--output", type=Path,
                        help="gate table path (default: <attempt>/gates.json)")
    args = parser.parse_args(argv)
    destination = args.output or (args.attempt / "gates.json")
    table = build(args.attempt, args.periodicity, args.planar_reference)
    destination.write_text(json.dumps(table, indent=2, allow_nan=False) + "\n",
                           encoding="utf-8")
    for name in GATE_NAMES:
        print(f"{name}: {'PASS' if table['gates'][name]['pass'] else 'FAIL'}")
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
