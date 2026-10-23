"""Bounded, reproducible no-TV minimum-lap experiment matrix.

This is an evidence harness, not an acceptance runner.  It never changes solver,
controller, vehicle, or capability source files.  Every attempted solve executes in
a fresh spawned process and is persisted before the next attempt begins.

Example (a deliberately small smoke matrix)::

    PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m \
      tools.diagnostics.verify_no_tv_minimum_lap --cases acceleration \
      --mesh-spacings 5 --basis-spacings 35 --wall-cap-s 120 \
      --output runs/no_tv_reliability/task5-accel-smoke
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import time

from tools.diagnostics.no_tv_provenance import resolve_base_commit, runtime_provenance
import traceback
import xml.etree.ElementTree as ET

import numpy as np

from tools.diagnostics.compare_qss_track_boundaries import track_view
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import (
    capability_hash, load_capability_surface,
)
from simplified_vehicle_models.solvers.line_validation import (
    REQUIRED_NUMERICAL_CHECKS, strict_validation, validate_line,
)
from simplified_vehicle_models.solvers.capability_profile import capability_profile, matched_profile_replay
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh, track_sample
from simplified_vehicle_models.solvers.qss_spline_ocp import initial_guess, solve_spline_line
from simplified_vehicle_models.solvers.profile_line_seed import build_profile_line_seed
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track

DEFAULT_MESH_SPACINGS = (5.0, 2.5, 1.25)
DIAGNOSTIC_MESH_SPACING = 0.625
DEFAULT_BASIS_SPACINGS = (35.0, 17.5)
DEFAULT_NLP_TOLERANCE = 1.0e-6
DEFAULT_DENSE_MAXIMUM_REFINEMENTS = 4
# This is deliberately the existing solver default, not the evenly-spaced eight
# division grid.  Leaving --off-collocation-divisions unspecified must preserve
# historical solve behavior exactly.
DEFAULT_OFF_COLLOCATION_FRACTIONS = (0.125, 0.25, 0.75, 0.875)
DEFAULT_FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")
DEFAULT_SURFACE = Path("runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz")
SUCCESS_STATUSES = {"Solve_Succeeded", "success", "succeeded"}
REFERENCE_PROFILE_STEPS = (1.0, 0.5, 0.25)
REFERENCE_TIME_CONVERGENCE_RELATIVE = 0.005
REPLAY_TIME_AGREEMENT_RELATIVE = 0.01
# This is the existing strict physical-time diagnostic tolerance, not a new
# baseline-performance allowance.
PHYSICAL_TIME_RELATIVE_TOLERANCE = 1.0e-5


@dataclass(frozen=True)
class MatrixCase:
    """One immutable geometry/start treatment in the Task 5 matrix."""
    name: str
    track: Path
    topology: str
    start_treatment: str
    start_speed_mps: float
    fallback_half_width_m: float | None
    candidate_only_reason: str | None = None


def default_cases() -> dict[str, MatrixCase]:
    """Return preregistered full-geometry cases without writing a cut-open XML."""
    mis = Path("models/tracks/2024_michigan_endurance_closed_edged.xml")
    fsc = Path("tracks_out/fsc_may_2026.xml")
    return {
        "mis_closed": MatrixCase("mis_closed", mis, "closed", "closed_flying", 0.0, None),
        "mis_cut_open": MatrixCase("mis_cut_open", mis, "full_cut_open", "open_rolling_10", 10.0, None),
        "fsc_closed": MatrixCase("fsc_closed", fsc, "closed", "closed_flying", 0.0, 3.0),
        "fsc_cut_open": MatrixCase("fsc_cut_open", fsc, "full_cut_open", "open_rolling_10", 10.0, 3.0),
        "skidpad": MatrixCase("skidpad", Path("models/tracks/fsae_2026_skidpad.xml"), "open", "open_rolling_10", 10.0, 1.5,
                              "NOTstandingstart: launch distance/time accounting is not implemented"),
        "acceleration": MatrixCase("acceleration", Path("models/tracks/fsae_2026_acceleration.xml"), "open", "open_rolling_10", 10.0, 2.45,
                                   "NOTstandingstart: launch distance/time accounting is not implemented"),
    }


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_json_new_or_replace(path: Path, data: dict, *, initial: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "x" if initial else "w"
    with path.open(mode, encoding="utf-8") as handle:
        json.dump(_json_safe(data), handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes(*paths: Path) -> dict[str, str]:
    """Recursively hash direct inputs and XML ``source``/``data`` dependencies.

    References are resolved relative to their declaring XML. Missing targets are a
    setup error, rather than silently omitted provenance. ``visited`` makes cyclic
    XML reference graphs deterministic and finite.
    """
    assets: set[Path] = set()
    visited: set[Path] = set()

    def visit(supplied: Path) -> None:
        path = Path(supplied).resolve()
        if path in visited:
            return
        if not path.is_file():
            raise FileNotFoundError(f"input dependency does not exist: {path}")
        visited.add(path)
        assets.add(path)
        if path.suffix.lower() != ".xml":
            return
        root = ET.parse(path).getroot()
        for node in root.iter():
            if node.tag not in {"source", "data"}:
                continue
            reference = node.get("ref") or node.get("path")
            if reference:
                visit(path.parent / reference)

    for supplied in paths:
        visit(Path(supplied))
    return {str(path): _sha256(path) for path in sorted(assets)}


def load_surface_dispatch(path: Path):
    """Load legacy or explicitly versioned signed caches without conversion."""
    with np.load(path, allow_pickle=False) as archive:
        signed = "u" in archive.files and "domain_speed_values" in archive.files
    if signed:
        from simplified_vehicle_models.formulations.signed_capability import load_signed_transient_capability
        return load_signed_transient_capability(path), "signed_transient_normalized_v1"
    return load_capability_surface(path), "legacy_capability_surface"


def make_track_view(case: MatrixCase):
    """Load a source track and create an in-memory full cut-open counterpart."""
    original = load_track(case.track)
    if case.topology == "closed":
        return track_view(original, is_closed=True)
    if case.topology == "full_cut_open":
        return track_view(original, is_closed=False)
    if case.topology == "open":
        return track_view(original, is_closed=False)
    raise ValueError(f"unknown topology {case.topology!r}")


def _off_collocation_fractions(divisions: int | None) -> tuple[float, ...]:
    """Return explicit dense sites without changing the legacy implicit default."""
    if divisions is None:
        return DEFAULT_OFF_COLLOCATION_FRACTIONS
    if (isinstance(divisions, bool) or not isinstance(divisions, (int, float, np.integer, np.floating))
            or not math.isfinite(divisions) or int(divisions) != divisions or int(divisions) < 3):
        raise ValueError("off_collocation_divisions must be an integer >= 3")
    count = int(divisions)
    return tuple(index / count for index in range(1, count) if 2 * index != count)

def _validate_finite_positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _validate_dense_maximum_refinements(value: int) -> int:
    if (isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating))
            or not math.isfinite(value) or int(value) != value or int(value) < 2):
        raise ValueError("dense_maximum_refinements must be an integer >= 2")
    return int(value)


def _profile_options(profile_step: float, maximum_profile_iterations: int,
                     maximum_profile_seconds: float) -> dict:
    step = _validate_finite_positive("profile_step", profile_step)
    seconds = _validate_finite_positive("maximum_profile_seconds", maximum_profile_seconds)
    if (isinstance(maximum_profile_iterations, bool)
            or not isinstance(maximum_profile_iterations, (int, float, np.integer, np.floating))
            or not math.isfinite(maximum_profile_iterations)
            or int(maximum_profile_iterations) != maximum_profile_iterations
            or int(maximum_profile_iterations) <= 0):
        raise ValueError("maximum_profile_iterations must be a positive integer")
    return {"profile_step_m": step, "maximum_profile_iterations": int(maximum_profile_iterations),
            "maximum_profile_seconds": seconds}


def _convergence_criteria(criteria: dict | None) -> dict | None:
    if criteria is None:
        return None
    required = {"rms_xy_m", "rms_metric", "rms_offset_m", "length_relative", "rms_speed_mps", "rms_curvature_per_m"}
    if not isinstance(criteria, dict) or set(criteria) != required:
        raise ValueError("convergence_criteria must declare exactly the six dimensional metric budgets")
    answer = {}
    for name in sorted(required):
        value = criteria[name]
        if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) or not math.isfinite(value) or value < 0.0:
            raise ValueError(f"convergence_criteria[{name!r}] must be finite and nonnegative")
        answer[name] = float(value)
    return answer


def build_attempt(case: MatrixCase, *, mesh_spacing_m: float, basis_spacing_m: float,
                  maximum_iterations: int, wall_cap_s: float, seed_name: str,
                  formulation: Path, surface: Path, nlp_tolerance: float = DEFAULT_NLP_TOLERANCE,
                  off_collocation_divisions: int | None = None,
                  dense_maximum_refinements: int = DEFAULT_DENSE_MAXIMUM_REFINEMENTS,
                  diagnostic_rungs: bool = False,
                  hessian_approximation: str = "exact",
                  control_representation: str = "quadratic_acceleration_v1",
                  speed_domain_margin_mps: float = 0.0,
                  matched_reference: bool = False, seed_profile: str = "none",
                   profile_control_projection: str = "direct_gradient",
                  strict_profile_seed: bool = False, profile_step: float = 2.0,
                   maximum_profile_iterations: int = 200, maximum_profile_seconds: float = 30.0,
                   convergence_criteria: dict | None = None, provenance: dict | None = None) -> dict:
    """Build a fully serializable solve request; this has no solver side effects."""
    allowed_meshes = DEFAULT_MESH_SPACINGS + ((DIAGNOSTIC_MESH_SPACING,) if diagnostic_rungs else ())
    allowed_basis = diagnostic_rungs or basis_spacing_m in DEFAULT_BASIS_SPACINGS
    if mesh_spacing_m not in allowed_meshes or not allowed_basis:
        raise ValueError("mesh and basis values must be preregistered Task 5 rungs; diagnostic values require --diagnostic-rungs")
    if not math.isfinite(float(basis_spacing_m)) or float(basis_spacing_m) <= 0.0:
        raise ValueError("basis spacing must be finite and positive")
    if (isinstance(maximum_iterations, bool)
            or not isinstance(maximum_iterations, (int, float, np.integer, np.floating))
            or not math.isfinite(maximum_iterations) or int(maximum_iterations) != maximum_iterations
            or int(maximum_iterations) <= 0):
        raise ValueError("maximum_iterations must be a positive integer")
    if not math.isfinite(float(wall_cap_s)) or float(wall_cap_s) <= 0:
        raise ValueError("wall cap must be finite and positive")
    if seed_name not in {"centerline", "low_amplitude_offset"}: 
        raise ValueError("seed must be centerline or low_amplitude_offset")
    if (not isinstance(hessian_approximation, str)
            or hessian_approximation not in {"exact", "limited-memory"}):
        raise ValueError("hessian_approximation must be exact or limited-memory")
    if control_representation not in {"quadratic_acceleration_v1",
                                      "quadratic_energy_rate_over_metric_v1"}:
        raise ValueError("control_representation must be a supported QSS control representation")
    if (isinstance(speed_domain_margin_mps, (bool, np.bool_))
            or not isinstance(speed_domain_margin_mps, (int, float, np.integer, np.floating))
            or not math.isfinite(speed_domain_margin_mps) or speed_domain_margin_mps < 0.0):
        raise ValueError("speed_domain_margin_mps must be a finite nonnegative number")
    if seed_profile not in {"none", "energy-nlp"}:
        raise ValueError("seed_profile must be none or energy-nlp")
    if profile_control_projection not in {"direct_gradient", "bounded_energy_l1"}:
        raise ValueError("profile_control_projection must be direct_gradient or bounded_energy_l1")
    profile_options = _profile_options(profile_step, maximum_profile_iterations, maximum_profile_seconds)
    criteria = _convergence_criteria(convergence_criteria)
    settings = QssSettings(tolerance=_validate_finite_positive("nlp_tolerance", nlp_tolerance),
                           maximum_iterations=int(maximum_iterations),
                           start_speed=float(case.start_speed_mps), open_regularisation=1e-6,
                           off_collocation_fractions=_off_collocation_fractions(off_collocation_divisions),
                           hessian_approximation=hessian_approximation,
                            control_representation=control_representation,
                            speed_domain_margin_mps=float(speed_domain_margin_mps))
    return {
        "case": asdict(case), "mesh_spacing_m": float(mesh_spacing_m),
        "basis_spacing_m": float(basis_spacing_m), "seed": seed_name,
        "wall_cap_s": float(wall_cap_s), "settings": asdict(settings),
        "dense_maximum_refinements": _validate_dense_maximum_refinements(dense_maximum_refinements),
        "mesh_rung_status": "preregistered" if mesh_spacing_m in DEFAULT_MESH_SPACINGS else "diagnostic_not_preregistered",
        "basis_rung_status": "preregistered" if basis_spacing_m in DEFAULT_BASIS_SPACINGS else "diagnostic_not_preregistered",
        "formulation": str(Path(formulation).resolve()), "surface": str(Path(surface).resolve()),
        "matched_reference": bool(matched_reference), "seed_profile": seed_profile,
        "profile_control_projection": profile_control_projection,
        "strict_profile_seed": bool(strict_profile_seed), "profile_options": profile_options,
        "initializer": {"seed": seed_name, "seed_profile": seed_profile,
                         "profile_control_projection": profile_control_projection,
                        "strict_profile_seed": bool(strict_profile_seed), "profile_options": profile_options},
        "convergence_criteria": criteria, "provenance": provenance,
        "no_torque_vectoring": True, "mode_switches": False, "controller_globals": False,
    }


def _seed_for(spline: OffsetSpline, seed_name: str) -> np.ndarray:
    if seed_name == "centerline":
        return np.zeros(spline.control_count)
    # This is deliberately only a deterministic perturbation.  Corridor and
    # physical feasibility are checked and recorded below; never presume them.
    phase = np.linspace(0.0, 2.0 * np.pi, spline.control_count, endpoint=not spline.is_closed)
    return 0.05 * np.sin(phase)


def _seed_domain_report(track, corridor, spline: OffsetSpline, coefficients: np.ndarray) -> dict:
    """Record coefficient and sampled-corridor domain facts without claiming feasibility."""
    stations = np.asarray(spline.control_distance, dtype=float)
    offsets = spline.evaluate(coefficients, stations)[0]
    lower = track_sample(corridor.minimum, corridor.distance, stations,
                         closed=bool(track.is_closed), period=float(track.lap_length))
    upper = track_sample(corridor.maximum, corridor.distance, stations,
                         closed=bool(track.is_closed), period=float(track.lap_length))
    return {"coefficient_sha256": hashlib.sha256(coefficients.tobytes()).hexdigest(),
            "coefficient_count": int(coefficients.size),
            "maximum_abs_coefficient_m": float(np.max(np.abs(coefficients))),
            "control_site_corridor_feasible": bool(np.all(offsets >= lower) and np.all(offsets <= upper)),
            "note": "control-site check only; not a full state/control feasibility claim"}


def _profile_record(profile) -> dict:
    """Serialize a fixed-path profile without presenting it as plant evidence."""
    return {"step_m": None, "converged": bool(profile.converged), "status": profile.status,
            "iterations": int(profile.iterations), "lap_time_s": float(profile.lap_time)
            if math.isfinite(profile.lap_time) else None,
            "max_change": float(profile.max_change) if math.isfinite(profile.max_change) else None}


def _relative_difference(left: float, right: float) -> float | None:
    if not (math.isfinite(left) and math.isfinite(right)):
        return None
    return abs(left - right) / max(1.0, abs(right))


def _profile_boundary(case: MatrixCase) -> tuple[str, float | None]:
    """Map the declared rolling/flying solve treatment to the reference API."""
    if case.start_treatment == "closed_flying":
        return "flying", None
    if case.start_treatment == "open_rolling_10":
        return "rolling", float(case.start_speed_mps)
    raise ValueError(f"unsupported reference start treatment {case.start_treatment!r}")


def _profile_failure_record(step: float, error: Exception) -> dict:
    return {"step_m": step, "converged": False, "status": "exception",
            "iterations": None, "lap_time_s": None, "max_change": None,
            "error": f"{type(error).__name__}: {error}"}


def _fixed_path_profile_evidence(solution, case: MatrixCase, surface) -> dict:
    """Run independent table references on analytic candidate and centerline paths.

    The reference is a prescribed-path capability-table calculation.  It neither
    reuses the OCP's speed state nor claims original-model/empirical validation.
    """
    unavailable = {"available": False, "candidate": {"profiles": []},
                   "baseline": {"profiles": []}, "replay": {"available": False},
                   "matched_baseline": False, "replay_agreement": False,
                   "reason": None, "empirical": False}
    try:
        trajectory = solution.trajectory
        evaluator = solution.state_evaluator
        if trajectory is None or evaluator is None:
            raise ValueError("solution lacks analytic trajectory or dense state evaluator")
        boundary, start_speed = _profile_boundary(case)
        # `to_path` samples the exact analytic geometry while `trajectory=` makes
        # capability_profile query curvature through its authoritative station map.
        stations = np.asarray(trajectory.reference.source_station, dtype=float)
        candidate_path = trajectory.to_path(stations)
        baseline_trajectory = OffsetTrajectory(trajectory.reference, trajectory.spline,
                                               np.zeros_like(trajectory.coefficients))
        baseline_path = baseline_trajectory.to_path(stations)
        candidate_profiles, baseline_profiles = {}, {}
        for step in REFERENCE_PROFILE_STEPS:
            try:
                candidate = capability_profile(candidate_path, surface, boundary=boundary,
                                               start_speed=start_speed, step=step,
                                               trajectory=trajectory)
                record = _profile_record(candidate); record["step_m"] = step
                candidate_profiles[step] = (candidate, record)
            except Exception as error:
                candidate_profiles[step] = (None, _profile_failure_record(step, error))
            try:
                baseline = capability_profile(baseline_path, surface, boundary=boundary,
                                              start_speed=start_speed, step=step,
                                              trajectory=baseline_trajectory)
                record = _profile_record(baseline); record["step_m"] = step
                baseline_profiles[step] = (baseline, record)
            except Exception as error:
                baseline_profiles[step] = (None, _profile_failure_record(step, error))
        # The .5/.25 pair is the declared sampled-time convergence evidence; 1 m
        # remains persisted as a coarse diagnostic, not a substitute endpoint.
        candidate_fine, candidate_half = candidate_profiles[.25][0], candidate_profiles[.5][0]
        baseline_fine, baseline_half = baseline_profiles[.25][0], baseline_profiles[.5][0]
        if any(item is None for item in (candidate_fine, candidate_half, baseline_fine, baseline_half)):
            return {"available": False,
                    "candidate": {"profiles": [candidate_profiles[step][1] for step in REFERENCE_PROFILE_STEPS]},
                    "baseline": {"profiles": [baseline_profiles[step][1] for step in REFERENCE_PROFILE_STEPS]},
                    "replay": {"available": False}, "matched_baseline": False,
                    "replay_agreement": False, "empirical": False,
                    "reason": "one or more declared fixed-path profiles failed"}
        candidate_convergence = _relative_difference(candidate_half.lap_time, candidate_fine.lap_time)
        baseline_convergence = _relative_difference(baseline_half.lap_time, baseline_fine.lap_time)
        candidate_ok = (candidate_half.converged and candidate_fine.converged
                        and candidate_convergence is not None
                        and candidate_convergence <= REFERENCE_TIME_CONVERGENCE_RELATIVE)
        baseline_ok = (baseline_half.converged and baseline_fine.converged
                       and baseline_convergence is not None
                       and baseline_convergence <= REFERENCE_TIME_CONVERGENCE_RELATIVE)
        replay = matched_profile_replay(trajectory, candidate_fine)
        reference_station = np.asarray(replay.reference_distance, dtype=float)
        state = evaluator.evaluate(reference_station)
        rms_speed = float(np.sqrt(np.mean((np.asarray(state["speed"], dtype=float)
                                           - candidate_fine.speed) ** 2)))
        # Compare the sampled fixed-path coordinates with the analytic trajectory
        # at the profile's mapped source stations; do not fabricate a zero from
        # an identity comparison.
        xy = trajectory.evaluate(reference_station)
        sampled_north = np.interp(candidate_fine.distance, candidate_path.distance,
                                  candidate_path.x_north)
        sampled_east = np.interp(candidate_fine.distance, candidate_path.distance,
                                 candidate_path.y_east)
        rms_path = float(np.sqrt(np.mean((np.asarray(xy.north) - sampled_north) ** 2
                                         + (np.asarray(xy.east) - sampled_east) ** 2)))
        replay_time = _relative_difference(float(solution.lap_time), candidate_fine.lap_time)
        replay_ok = candidate_ok and replay_time is not None and replay_time <= REPLAY_TIME_AGREEMENT_RELATIVE
        baseline_margin = PHYSICAL_TIME_RELATIVE_TOLERANCE * max(1.0, abs(baseline_fine.lap_time))
        baseline_ok = baseline_ok and candidate_fine.lap_time <= baseline_fine.lap_time + baseline_margin
        return {"available": True, "candidate": {"profiles": [candidate_profiles[step][1] for step in REFERENCE_PROFILE_STEPS],
                "time_convergence_relative": candidate_convergence, "converged": candidate_ok},
                "baseline": {"profiles": [baseline_profiles[step][1] for step in REFERENCE_PROFILE_STEPS],
                "time_convergence_relative": baseline_convergence,
                "converged": baseline_ok},
                "replay": {"available": True, "lap_time_relative_difference": replay_time,
                "rms_speed_mps": rms_speed, "rms_path_m": rms_path,
                "boundary": boundary, "start_speed_mps": start_speed,
                "reference_kind": "fixed_path_capability_profile_not_empirical"},
                "matched_baseline": bool(baseline_ok), "replay_agreement": bool(replay_ok),
                "reason": None, "empirical": False}
    except Exception as error:
        unavailable["reason"] = f"{type(error).__name__}: {error}"
        return unavailable


def _attempt_worker(request_path: str, result_path: str, artifact_path: str) -> None:
    """Spawn-safe worker entry. All communication is filesystem based."""
    request = json.loads(Path(request_path).read_text(encoding="utf-8"))
    started = time.perf_counter()
    result = {"request": request, "status": "worker_exception", "accepted": False,
              "validation_status": "solver_failed", "wall_seconds": None}
    try:
        case = MatrixCase(**request["case"])
        track = make_track_view(case)
        corridor = corridor_from_track(track, margin=0.1, half_width=case.fallback_half_width_m)
        vehicle = point_mass_view_of_planar(compile_formulation(request["formulation"]).model)
        surface, representation = load_surface_dispatch(Path(request["surface"]))
        expected = capability_hash(vehicle)
        if getattr(surface, "provenance_hash", None) != expected:
            raise ValueError("capability provenance hash does not match formulation")
        settings_values = dict(request["settings"])
        # JSON transports dataclass tuples as lists; restore the solver's strict
        # immutable setting type before passing it to the existing OCP API.
        settings_values["off_collocation_fractions"] = tuple(settings_values["off_collocation_fractions"])
        settings = QssSettings(**settings_values)
        mesh = build_mesh(track, request["mesh_spacing_m"])
        spline = OffsetSpline(track, control_spacing=request["basis_spacing_m"])
        result["actual_basis"] = {"control_count": int(spline.control_count),
                                  "control_station_m": np.asarray(spline.control_distance, dtype=float).tolist()}
        # A profile is an independent centreline initializer. Attempt it before
        # constructing the established envelope seed, so a legacy initializer
        # failure cannot preempt its explicitly requested recovery path.
        result["profile_seed"] = {"requested": request.get("seed_profile", "none"),
                                  "requested_control_projection": request.get("profile_control_projection", "direct_gradient"),
                                  "used": False, "fallback": None}
        use_old_seed = True
        seed = None
        if request.get("seed_profile", "none") == "energy-nlp":
            # Preserve a declared lateral perturbation; a centreline profile must
            # never overwrite it without recomputing its analytic trajectory.
            if request["seed"] != "centerline":
                result["profile_seed"].update({"fallback": "declared-offset-perturbation",
                                                "control_projection_used": None,
                                                "diagnostics": {"available": False, "reason": "profile-centreline-only"}})
                if request.get("strict_profile_seed", False):
                    raise RuntimeError("strict profile seed requires the centerline seed")
            else:
                options = request["profile_options"]
                profile_seed, diagnostics = build_profile_line_seed(
                    track, corridor, surface, mesh, spline, settings,
                    profile_step=options["profile_step_m"],
                    maximum_profile_iterations=options["maximum_profile_iterations"],
                    maximum_profile_seconds=options["maximum_profile_seconds"],
                    control_projection=request.get("profile_control_projection", "direct_gradient"))
                result["profile_seed"]["diagnostics"] = diagnostics
                result["profile_seed"]["control_projection_used"] = (
                    request.get("profile_control_projection", "direct_gradient")
                    if profile_seed is not None else None)
                if profile_seed is not None:
                    seed = profile_seed
                    use_old_seed = False
                    result["profile_seed"]["used"] = True
                else:
                    result["profile_seed"]["fallback"] = "explicit-old-seed"
                    if request.get("strict_profile_seed", False):
                        raise RuntimeError(f"strict profile seed failed: {diagnostics.get('reason')}")
        if use_old_seed:
            seed = initial_guess(track, corridor, surface, mesh, spline, settings)
            coefficients = _seed_for(spline, request["seed"])
            seed[:spline.control_count] = coefficients
            result["initial_seed"] = _seed_domain_report(track, corridor, spline, coefficients)
        else:
            result["initial_seed"] = {"used": "energy-nlp-profile", "note": "projection seed; not OCP feasible"}
        actual_initializer = {"requested": request["initializer"], "used": "energy-nlp-profile" if not use_old_seed else request["seed"],
                              "profile_seed": result["profile_seed"],
                              "seed_sha256": hashlib.sha256(np.asarray(seed, dtype=float).tobytes()).hexdigest()}
        # Audit includes the exact vector digest, but refinement identity excludes
        # mesh/basis-derived bytes and elapsed profile diagnostics.
        actual_initializer["stable_identity"] = _initializer_stable_identity(request, actual_initializer)
        actual_initializer["fingerprint"] = hashlib.sha256(
            json.dumps(_json_safe(actual_initializer["stable_identity"]), sort_keys=True).encode("utf-8")).hexdigest()
        result["actual_initializer"] = actual_initializer
        solution = solve_spline_line(track, corridor, surface, mesh, spline, settings, seed=seed)
        source_grid = np.linspace(float(solution.distance[0]), float(solution.distance[-1]), 256)
        if getattr(solution, "trajectory", None) is not None and getattr(solution, "state_evaluator", None) is not None:
            geometry = solution.trajectory.evaluate(source_grid)
            state = solution.state_evaluator.evaluate(source_grid)
            path_grid = np.asarray(solution.trajectory.station_map.to_path(source_grid), dtype=float)
            common_north, common_east = np.asarray(geometry.north, dtype=float), np.asarray(geometry.east, dtype=float)
            common_metric, common_offset = np.asarray(geometry.metric, dtype=float), np.asarray(geometry.offset, dtype=float)
            common_speed, common_curvature = np.asarray(state["speed"], dtype=float), np.asarray(geometry.curvature, dtype=float)
        else:  # mocked test-only solver fixture; deliberately lacks real convergence evidence
            path_grid = source_grid
            common_north, common_east = source_grid, np.zeros_like(source_grid)
            common_metric, common_offset = np.ones_like(source_grid), np.interp(source_grid, solution.distance, solution.offset)
            common_speed, common_curvature = np.interp(source_grid, solution.distance, solution.speed), np.interp(source_grid, solution.distance, solution.curvature)
        np.savez_compressed(artifact_path, distance=solution.distance, offset=solution.offset,
                            stage_timings=np.array(json.dumps(getattr(solution, "stage_timings", {}), sort_keys=True)),
                              heading_error=solution.heading_error, curvature=solution.curvature,
                              speed=solution.speed, acceleration=solution.acceleration,
                              midpoint_acceleration=solution.midpoint_acceleration,
                              coefficients=solution.coefficients,
                              common_source_s=source_grid, common_path_ell_m=path_grid,
                              common_north_m=common_north,
                              common_east_m=common_east,
                              common_metric=common_metric,
                              common_offset_m=common_offset,
                              common_speed_mps=common_speed,
                              common_curvature_per_m=common_curvature,
                              common_path_length_m=np.array(float(path_grid[-1] - path_grid[0])),
                              control_station_m=np.asarray(spline.control_distance, dtype=float),
                              state_representation=np.array(solution.state_representation),
                              control_representation=np.array(getattr(
                                  solution, "control_representation",
                                  settings.control_representation,
                              )),
                              speed_domain_margin_mps=np.array(
                                  settings.speed_domain_margin_mps,
                              ),
                              time_quadrature=np.array(solution.time_quadrature),
                              time_metadata=np.array(json.dumps(solution.time_quadrature_metadata, sort_keys=True)))
        # Persist exact reference/map metadata separately with the established
        # runner contract, without changing runner behavior or overwriting files.
        from tools.run.solve_minimum_time_line import _write_trajectory_metadata
        trajectory_metadata = Path(artifact_path).with_name("trajectory_metadata.npz")
        _write_trajectory_metadata(trajectory_metadata, solution)
        dense = validate_line(solution, corridor, surface, settings, vehicle=vehicle,
                              maximum_refinements=request["dense_maximum_refinements"])
        checks = dict(dense["checks"])
        # Refinement evidence is aggregation-only.  Reference evidence is
        # optional and fails closed; missing flags remain None for compatibility.
        checks.update({"mesh_convergence": None, "basis_convergence": None,
                       "matched_baseline": None, "replay_agreement": None})
        reference = {"available": False, "reason": "--matched-reference not requested",
                     "matched_baseline": None, "replay_agreement": None,
                     "replay": {"available": False, "reason": "--matched-reference not requested"},
                     "empirical": False}
        if request.get("matched_reference", False):
            reference = _fixed_path_profile_evidence(solution, case, surface)
            checks["matched_baseline"] = reference["matched_baseline"]
            checks["replay_agreement"] = reference["replay_agreement"]
        solver_succeeded = solution.status in SUCCESS_STATUSES
        strict = strict_validation(solver_succeeded, checks, REQUIRED_NUMERICAL_CHECKS)
        result.update({"status": str(solution.status), "solver_succeeded": solver_succeeded,
                       "accepted": False, "objective_s": solution.lap_time,
                       "objective_value": solution.objective_value,
                       "iterations": solution.iterations,
                       "primal_error": solution.maximum_violation,
                       "dual_error": None, "dense_original_model": dense,
                       "checks": checks, "strict": strict,
                       "validation_status": strict["validation_status"],
                       "surface_representation": representation,
                       "state_representation": solution.state_representation,
                       "control_representation": getattr(
                           solution, "control_representation",
                           settings.control_representation,
                       ),
                       "speed_domain_margin_mps": settings.speed_domain_margin_mps,
                       "time_quadrature": solution.time_quadrature,
                       "time_metadata": solution.time_quadrature_metadata,
                       "stage_timings": getattr(solution, "stage_timings", {}),
                       "trajectory_metadata_artifact": str(trajectory_metadata.name),
                       "artifact": str(Path(artifact_path).name),
                       "start_treatment": case.start_treatment,
                       "candidate_only_reason": case.candidate_only_reason,
                       "reference": reference, "replay": reference["replay"]})
    except RuntimeError as error:
        # ``SplineSolveError`` remains a RuntimeError for legacy callers. Its
        # raw iterate is diagnostic-only and never becomes a solution artifact.
        diagnostics = getattr(error, "diagnostics", None)
        failed_values = getattr(error, "failed_values", None)
        constraint_values = getattr(error, "constraint_values", None)
        if isinstance(diagnostics, dict):
            result["failure_diagnostics"] = diagnostics
            if failed_values is not None or constraint_values is not None:
                failed_artifact = Path(artifact_path).with_name("failed_iterate.npz")
                payload = {"metadata": np.array(json.dumps(_json_safe(diagnostics), sort_keys=True))}
                if failed_values is not None:
                    payload["failed_values"] = np.asarray(failed_values, dtype=float)
                if constraint_values is not None:
                    payload["constraint_values"] = np.asarray(constraint_values, dtype=float)
                for name in ("dual_x", "dual_g"):
                    value = getattr(error, name, None)
                    if value is not None:
                        payload[name] = np.asarray(value, dtype=float)
                np.savez_compressed(failed_artifact, **payload)
                result["failed_artifact"] = str(failed_artifact.name)
            result["status"] = diagnostics.get("return_status", "solver_failed")
        result.update({"error_type": type(error).__name__, "error": str(error),
                       "traceback": traceback.format_exc(), "solver_succeeded": False,
                       "accepted": False, "validation_status": "solver_failed"})
    except Exception as error:
        result.update({"error_type": type(error).__name__, "error": str(error),
                       "traceback": traceback.format_exc(), "solver_succeeded": False,
                       "accepted": False})
    finally:
        result["wall_seconds"] = time.perf_counter() - started
        _write_json_new_or_replace(Path(result_path), result, initial=True)


def run_bounded_worker(request: dict, output: Path) -> dict:
    """Execute one request under a hard spawned-process deadline and persist it."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    request_path, result_path, artifact_path = output / "request.json", output / "result.json", output / "solution.npz"
    _write_json_new_or_replace(request_path, request, initial=True)
    process = mp.get_context("spawn").Process(target=_attempt_worker,
        args=(str(request_path), str(result_path), str(artifact_path)))
    process.start(); process.join(float(request["wall_cap_s"]))
    if process.is_alive():
        process.terminate(); process.join()
        result = {"request": request, "status": "wall_timeout", "accepted": False,
                  "solver_succeeded": False, "validation_status": "solver_failed",
                  "error_type": "TimeoutError", "error": "worker exceeded wall cap",
                  "wall_cap_s": request["wall_cap_s"]}
        # Preserve a child result, including a partial one, as worker evidence.
        # The parent verdict is written separately and is authoritative for the
        # matrix row, so a timeout can never inherit a late success result.
        _write_json_new_or_replace(output / "timeout_result.json", result, initial=True)
    elif result_path.exists():
        result = json.loads(result_path.read_text(encoding="utf-8"))
    else:
        result = {"request": request, "status": "worker_no_result", "accepted": False,
                  "solver_succeeded": False, "validation_status": "solver_failed",
                  "error_type": "WorkerExit", "error": f"worker exit code {process.exitcode}"}
        _write_json_new_or_replace(result_path, result, initial=True)
    return result


def _rms_difference(left: Path, right: Path) -> dict:
    """Compare exact common-source artifacts; never synthesize acceptance data."""
    with np.load(left) as a, np.load(right) as b:
        required = ("common_source_s", "common_north_m", "common_east_m", "common_metric",
                    "common_offset_m", "common_speed_mps", "common_curvature_per_m", "common_path_length_m")
        if any(name not in a or name not in b for name in required):
            # Historical/fixture artifacts retain candidate diagnostics only; they
            # cannot pass strict edges because criteria remain absent.
            common = np.linspace(max(a["distance"][0], b["distance"][0]), min(a["distance"][-1], b["distance"][-1]), 256)
            def old(name):
                return np.interp(common, a["distance"], a[name]) - np.interp(common, b["distance"], b[name])
            offset, speed, curvature = old("offset"), old("speed"), old("curvature")
            return {"rms_xy_m": None, "rms_metric": None, "rms_offset_m": float(np.sqrt(np.mean(offset ** 2))),
                    "rms_speed_mps": float(np.sqrt(np.mean(speed ** 2))),
                    "rms_curvature_per_m": float(np.sqrt(np.mean(curvature ** 2))), "length_relative": None,
                    "rms_path": float(np.sqrt(np.mean(offset ** 2))),
                    "rms_speed": float(np.sqrt(np.mean(speed ** 2))),
                    "rms_curvature": float(np.sqrt(np.mean(curvature ** 2)))}
        grid = a["common_source_s"]
        if not np.array_equal(grid, b["common_source_s"]):
            raise ValueError("common-source grids differ")
        def rms(name):
            return float(np.sqrt(np.mean((a[name] - b[name]) ** 2)))
        return {"rms_xy_m": float(np.sqrt(np.mean((a["common_north_m"] - b["common_north_m"]) ** 2
                                                 + (a["common_east_m"] - b["common_east_m"]) ** 2))),
                "rms_metric": rms("common_metric"), "rms_offset_m": rms("common_offset_m"),
                "rms_speed_mps": rms("common_speed_mps"),
                "rms_curvature_per_m": rms("common_curvature_per_m"),
                "length_relative": _relative_difference(float(a["common_path_length_m"]), float(b["common_path_length_m"])),
                # compatibility name, now explicitly offset rather than path XY
                "rms_path": rms("common_offset_m")}


def _criteria_qualified(comparison: dict, criteria: dict | None) -> bool:
    return criteria is not None and all(comparison.get(name) is not None and comparison[name] <= budget
                                        for name, budget in criteria.items())


def _dense_live_qualified(row: dict) -> bool:
    """Require successful solve plus dense geometry, feasibility, and live model."""
    dense = row.get("dense_original_model", {})
    checks = dense.get("checks", {})
    status = row.get("status")
    # Historical records without a status predate this gate and remain diagnostic
    # only; current worker records always carry one and must be success statuses.
    return (status in SUCCESS_STATUSES and row.get("solver_succeeded") is True
            and row.get("artifact") is not None
            and checks.get("geometry_consistency") is True
            and checks.get("dense_feasibility") is True
            and dense.get("model_feasible") is True)


def _attempt_sources_unchanged(row: dict) -> bool:
    """Return whether one attempt retained its exact pre-spawn inputs."""
    request = row.get("request", {})
    return (request.get("source_hashes") is not None
            and request.get("source_hashes") == row.get("source_hashes_after")
            and request.get("provenance") == row.get("provenance_after"))


def _initializer_stable_identity(request: dict, actual_initializer: dict) -> dict:
    """Return refinement-relevant initializer identity, excluding transient audit data."""
    profile_seed = actual_initializer.get("profile_seed", {})
    return {"requested_mode": request.get("initializer"), "requested_seed": request.get("seed"),
            "requested_control_projection": request.get("profile_control_projection", "direct_gradient"),
            "actual_control_projection": profile_seed.get("control_projection_used"),
            "actual_profile_used": profile_seed.get("used"),
            "fallback_category": profile_seed.get("fallback"),
            "used": actual_initializer.get("used"),
            "implementation_runtime": request.get("provenance", {})}


def _same_refinement_conditions(left: dict, right: dict) -> bool:
    """Forbid policy/settings/source changes from masquerading as refinement.

    Exact seed bytes and timing diagnostics stay in artifacts for audit but are
    intentionally absent from the stable identity: both vary with mesh/basis.
    """
    a, b = left["request"], right["request"]
    left_identity = left.get("actual_initializer", {}).get("stable_identity")
    right_identity = right.get("actual_initializer", {}).get("stable_identity")
    # Older diagnostic fixtures have no resolved actual identity and therefore
    # stay eligible only for diagnostic refinement collection, never strict
    # acceptance (which independently requires unavailable geometry/state policy).
    legacy_fingerprint_matches = (left.get("actual_initializer", {}).get("fingerprint")
                                  == right.get("actual_initializer", {}).get("fingerprint"))
    identity_matches = ((left_identity is None and right_identity is None and legacy_fingerprint_matches)
                        or (left_identity is not None and left_identity == right_identity))
    return (a.get("settings") == b.get("settings") and a.get("seed") == b.get("seed")
            and a.get("initializer") == b.get("initializer") and identity_matches
            and a.get("provenance") == b.get("provenance")
            and a.get("convergence_criteria") == b.get("convergence_criteria")
            and _attempt_sources_unchanged(left) and _attempt_sources_unchanged(right)
            and a.get("source_hashes") == b.get("source_hashes")
            and left.get("surface_representation") == right.get("surface_representation"))


def _comparison(case: str, left: dict, right: dict, output: Path, *, kind: str) -> dict | None:
    try:
        metrics = _rms_difference(output / case / left["artifact"], output / case / right["artifact"])
    except (FileNotFoundError, ValueError):
        return None
    return {"case": case, "kind": kind,
            "left": {"attempt_id": left.get("attempt_id"), "mesh_m": left["request"]["mesh_spacing_m"], "basis_m": left["request"]["basis_spacing_m"], "time_s": left.get("objective_s")},
            "right": {"attempt_id": right.get("attempt_id"), "mesh_m": right["request"]["mesh_spacing_m"], "basis_m": right["request"]["basis_spacing_m"], "time_s": right.get("objective_s")},
            "time_difference_s": (left.get("objective_s") - right.get("objective_s")) if left.get("objective_s") is not None and right.get("objective_s") is not None else None,
            "time_relative_difference": _relative_difference(float(left["objective_s"]), float(right["objective_s"]))
            if left.get("objective_s") is not None and right.get("objective_s") is not None else None,
            **metrics, "convergence_criteria": left["request"].get("convergence_criteria"),
            # User-supplied budgets are visible diagnostics only.  No approved,
            # source-hashed geometry/state convergence policy exists yet, so they
            # cannot become strict evidence regardless of their magnitudes.
            "metric_diagnostic_within_user_budget": _criteria_qualified(
                metrics, left["request"].get("convergence_criteria")),
            "metric_diagnostic_nonqualifying": left["request"].get("convergence_criteria") is not None,
            "geometry_state_convergence": None}


def _reference_qualified(row: dict) -> bool:
    """Require actual optional-reference outputs, never a nominal surrogate flag."""
    reference = row.get("reference", {})
    return (reference.get("available") is True
            and row.get("checks", {}).get("matched_baseline") is True
            and row.get("checks", {}).get("replay_agreement") is True
            and reference.get("empirical") is False)


def _qualified_edge(comparisons: list[dict], *, kind: str) -> dict | None:
    """Choose one complete qualified refinement edge; never union edge endpoints."""
    for comparison in comparisons:
        left, right = comparison["left"], comparison["right"]
        rungs = ((left["mesh_m"], right["mesh_m"]) if kind == "mesh_refinement"
                 else (left["basis_m"], right["basis_m"]))
        preregistered = (all(rung in DEFAULT_MESH_SPACINGS for rung in rungs)
                         if kind == "mesh_refinement"
                         else all(rung in DEFAULT_BASIS_SPACINGS for rung in rungs))
        if (comparison.get("reference_qualified") is True and preregistered
                and comparison.get("time_relative_difference") is not None
                and comparison["time_relative_difference"] <= REFERENCE_TIME_CONVERGENCE_RELATIVE
                # Strict geometry/state convergence is deliberately unavailable
                # until a curated, source-hashed policy and data are approved.
                and comparison.get("geometry_state_convergence") is True):
            return {"left_attempt": left.get("attempt_id"), "right_attempt": right.get("attempt_id"),
                    "time_relative_difference": comparison["time_relative_difference"],
                    "maximum_relative_time": REFERENCE_TIME_CONVERGENCE_RELATIVE,
                    "kind": kind}
    return None


def aggregate(results: list[dict], output: Path, *, source_data_unchanged: bool = True,
              expected_cases: tuple[str, ...] | None = None,
              required_mesh_rungs: tuple[float, ...] = DEFAULT_MESH_SPACINGS,
              required_basis_rungs: tuple[float, ...] = DEFAULT_BASIS_SPACINGS) -> dict:
    """Separate candidate differences from qualified one-axis refinement evidence.

    A final matrix source mismatch invalidates every qualified comparison, even if
    individual attempts happened to preserve their own snapshots. Candidate
    diagnostics remain available for investigation.
    """
    candidate_diagnostics, mesh_refinement, basis_refinement = [], [], []
    by_case: dict[str, list[dict]] = {}
    for item in results:
        by_case.setdefault(item["request"]["case"]["name"], []).append(item)
    for name, rows in by_case.items():
        completed = [row for row in rows if row.get("artifact")]
        for index, left in enumerate(completed):
            for right in completed[index + 1:]:
                mesh_equal = left["request"]["mesh_spacing_m"] == right["request"]["mesh_spacing_m"]
                basis_equal = left["request"]["basis_spacing_m"] == right["request"]["basis_spacing_m"]
                if mesh_equal and basis_equal:
                    continue
                candidate = _comparison(name, left, right, output, kind="unqualified_candidate")
                if candidate is not None:
                    candidate_diagnostics.append(candidate)
                if not (source_data_unchanged and _dense_live_qualified(left)
                        and _dense_live_qualified(right)
                        and _same_refinement_conditions(left, right)):
                    continue
                if basis_equal and not mesh_equal:
                    evidence = _comparison(name, left, right, output, kind="mesh_refinement")
                    if evidence is not None:
                        evidence["reference_qualified"] = (_reference_qualified(left)
                                                           and _reference_qualified(right))
                        mesh_refinement.append(evidence)
                elif mesh_equal and not basis_equal:
                    left_grid = left.get("actual_basis", {}).get("control_station_m")
                    right_grid = right.get("actual_basis", {}).get("control_station_m")
                    if left_grid == right_grid:
                        continue
                    evidence = _comparison(name, left, right, output, kind="basis_refinement")
                    if evidence is not None:
                        evidence["reference_qualified"] = (_reference_qualified(left)
                                                           and _reference_qualified(right))
                        basis_refinement.append(evidence)
    expected_cases = tuple(expected_cases) if expected_cases is not None else tuple(default_cases())
    case_status = {}
    for name in expected_cases:
        rows = by_case.get(name, [])
        qualified_mesh = [item for item in mesh_refinement if item["case"] == name]
        qualified_basis = [item for item in basis_refinement if item["case"] == name]
        mesh_pair = _qualified_edge(qualified_mesh, kind="mesh_refinement")
        basis_pair = _qualified_edge(qualified_basis, kind="basis_refinement")
        # Complete declared-rung coverage is distinct from a diagnostic edge:
        # .625 m and custom bases remain visible but cannot qualify this study.
        actual_mesh = {float(row["request"]["mesh_spacing_m"]) for row in rows
                       if _dense_live_qualified(row) and _reference_qualified(row)}
        actual_basis = {float(row["request"]["basis_spacing_m"]) for row in rows
                        if _dense_live_qualified(row) and _reference_qualified(row)
                        and row.get("actual_basis", {}).get("control_station_m") is not None}
        mesh_coverage = set(required_mesh_rungs).issubset(actual_mesh)
        basis_coverage = set(required_basis_rungs).issubset(actual_basis)
        accepted = bool(source_data_unchanged and mesh_pair and basis_pair
                        and mesh_coverage and basis_coverage)
        case_status[name] = {"mesh_evidence": mesh_pair, "basis_evidence": basis_pair,
                             "mesh_rung_coverage": sorted(actual_mesh),
                             "basis_rung_coverage": sorted(actual_basis),
                             "required_mesh_rungs": list(required_mesh_rungs),
                             "required_basis_rungs": list(required_basis_rungs),
                             "accepted": accepted,
                             "status": "numerically_validated" if accepted else "incomplete",
                             "empirical": False}
    all_cases_accepted = bool(expected_cases) and all(value["accepted"] for value in case_status.values())
    return {"attempts": len(results), "candidate_diagnostics": candidate_diagnostics,
            "mesh_refinement": mesh_refinement, "basis_refinement": basis_refinement,
            "case_status": case_status,
            "strict_aggregate": {"accepted": all_cases_accepted,
              "status": "numerically_validated" if all_cases_accepted else "incomplete",
              "empirical": False,
              "reason": None if all_cases_accepted else "every case requires same-condition qualified mesh and distinct actual-basis pairs plus converged matched baseline and replay evidence"}}


def run_matrix(args) -> dict:
    output = Path(args.output).resolve()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output must be a new exclusive directory")
    repo = Path(__file__).resolve().parents[2]
    actual_base_commit = resolve_base_commit(repo, args.base_commit)
    actual_provenance = runtime_provenance(repo)
    cases = default_cases()
    requested_names = args.cases.split(",") if args.cases else list(cases)
    selected = [cases[name] for name in requested_names]
    form, surface = Path(args.formulation).resolve(), Path(args.surface).resolve()
    hashes_before = source_hashes(*(case.track for case in selected), form, surface)
    manifest = {"purpose": "bounded Task5 no-TV reliability matrix; candidate evidence only",
                # Retain the legacy name, but make it an accurate resolved identity.
                "base_commit": actual_base_commit, "git_head": actual_provenance["git_head"],
                "requested_base_commit": args.base_commit,
                "baseline_label": getattr(args, "baseline_label", None),
                "provenance_initial": actual_provenance,
                "cases": [asdict(item) for item in selected], "mesh_spacings_m": args.mesh_spacings,
                "basis_spacings_m": args.basis_spacings, "source_hashes_before": hashes_before,
                "surface": str(surface), "surface_representation": load_surface_dispatch(surface)[1], "results": []}
    output.mkdir(parents=True)
    _write_json_new_or_replace(output / "matrix.json", manifest, initial=True)
    for case in selected:
        case_root = output / case.name; case_root.mkdir()
        for mesh in args.mesh_spacings:
            for basis in args.basis_spacings:
                for seed in args.seeds:
                    attempt_provenance = runtime_provenance(repo)
                    request = build_attempt(case, mesh_spacing_m=mesh, basis_spacing_m=basis,
                        maximum_iterations=args.maximum_iterations, wall_cap_s=args.wall_cap_s,
                        seed_name=seed, formulation=form, surface=surface,
                        nlp_tolerance=getattr(args, "nlp_tolerance", DEFAULT_NLP_TOLERANCE),
                        off_collocation_divisions=getattr(args, "off_collocation_divisions", None),
                        dense_maximum_refinements=getattr(args, "dense_maximum_refinements", DEFAULT_DENSE_MAXIMUM_REFINEMENTS),
                        diagnostic_rungs=getattr(args, "diagnostic_rungs", False),
                        hessian_approximation=getattr(args, "hessian_approximation", "exact"),
                        control_representation=getattr(
                            args, "control_representation", "quadratic_acceleration_v1",
                        ),
                        speed_domain_margin_mps=getattr(args, "speed_domain_margin_mps", 0.0),
                        matched_reference=getattr(args, "matched_reference", False),
                        seed_profile=getattr(args, "seed_profile", "none"),
                        profile_control_projection=getattr(args, "profile_control_projection", "direct_gradient"),
                        strict_profile_seed=getattr(args, "strict_profile_seed", False),
                        profile_step=getattr(args, "profile_step", 2.0),
                        maximum_profile_iterations=getattr(args, "maximum_profile_iterations", 200),
                        maximum_profile_seconds=getattr(args, "maximum_profile_seconds", 30.0),
                        convergence_criteria=getattr(args, "convergence_criteria", None),
                        provenance=attempt_provenance)
                    # Each spawned solve owns a fresh input snapshot. Do not reuse
                    # the matrix-start hashes: sources can change between attempts.
                    request["source_hashes"] = source_hashes(case.track, form, surface)
                    attempt_name = f"mesh-{mesh:g}_basis-{basis:g}_{seed}"
                    result = run_bounded_worker(request, case_root / attempt_name)
                    result["attempt_id"] = f"{case.name}/{attempt_name}"
                    # Hash after child completion, before the next request. Any
                    # in-attempt mutation makes this row candidate-only, while its
                    # result and artifact remain retained for diagnostics.
                    result["source_hashes_after"] = source_hashes(case.track, form, surface)
                    result["provenance_after"] = runtime_provenance(repo)
                    result["source_data_unchanged"] = (
                        request["source_hashes"] == result["source_hashes_after"]
                        and request["provenance"] == result["provenance_after"])
                    # Artifact is relative to its attempt directory until aggregation.
                    if result.get("artifact"):
                        result["artifact"] = f"{attempt_name}/{result['artifact']}"
                    manifest["results"].append(result)
                    _write_json_new_or_replace(output / "matrix.json", manifest)
    manifest["source_hashes_after"] = source_hashes(*(case.track for case in selected), form, surface)
    manifest["provenance_afterfinal"] = runtime_provenance(repo)
    manifest["source_data_unchanged"] = (manifest["source_hashes_before"] == manifest["source_hashes_after"]
                                         and manifest["provenance_initial"] == manifest["provenance_afterfinal"])
    # Artifact paths stay relative to their case directory (attempt/solution.npz).
    # A bounded subset may yield per-case diagnostics, but never a whole-study
    # green result: aggregate against the complete preregistered case matrix.
    manifest["aggregate"] = aggregate(manifest["results"], output,
                                      source_data_unchanged=manifest["source_data_unchanged"],
                                      expected_cases=tuple(cases),
                                      required_mesh_rungs=tuple(DEFAULT_MESH_SPACINGS),
                                      required_basis_rungs=tuple(DEFAULT_BASIS_SPACINGS))
    _write_json_new_or_replace(output / "matrix.json", manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--formulation", type=Path, default=DEFAULT_FORMULATION)
    parser.add_argument("--surface", type=Path, default=DEFAULT_SURFACE)
    parser.add_argument("--cases", default=None, help="comma separated bounded subset of preregistered cases")
    parser.add_argument("--mesh-spacings", type=lambda x: [float(v) for v in x.split(",")], default=DEFAULT_MESH_SPACINGS)
    parser.add_argument("--basis-spacings", type=lambda x: [float(v) for v in x.split(",")], default=DEFAULT_BASIS_SPACINGS)
    parser.add_argument("--seeds", type=lambda x: x.split(","), default=["centerline", "low_amplitude_offset"])
    parser.add_argument("--maximum-iterations", type=int, default=200)
    parser.add_argument("--nlp-tolerance", type=float, default=DEFAULT_NLP_TOLERANCE)
    parser.add_argument("--hessian-approximation", choices=("exact", "limited-memory"),
                        default="exact", help="IPOPT Hessian model for the spline solve")
    parser.add_argument("--control-representation", choices=(
        "quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1",
    ), default="quadratic_acceleration_v1")
    parser.add_argument("--speed-domain-margin-mps", type=float, default=0.0,
                        help="optional conservative inner margin below the speed-domain boundary")
    parser.add_argument("--off-collocation-divisions", type=int, default=None,
                        help="explicit interior i/N sites excluding midpoint; omitted preserves legacy sites")
    parser.add_argument("--dense-maximum-refinements", type=int, default=DEFAULT_DENSE_MAXIMUM_REFINEMENTS)
    parser.add_argument("--diagnostic-rungs", action="store_true",
                        help="allow and label non-preregistered mesh or basis diagnostic rungs")
    parser.add_argument("--matched-reference", action="store_true",
                        help="compute optional analytic-path fixed capability references (.5/.25 convergence)")
    parser.add_argument("--seed-profile", choices=("none", "energy-nlp"), default="none",
                        help="optional fixed-path energy NLP initializer; default preserves old seed")
    parser.add_argument("--profile-control-projection", choices=("direct_gradient", "bounded_energy_l1"),
                        default="direct_gradient", help="energy-NLP profile acceleration-control projection")
    parser.add_argument("--strict-profile-seed", action="store_true",
                        help="fail the worker rather than use the explicit old seed on profile failure")
    parser.add_argument("--wall-cap-s", type=float, default=300.0)
    parser.add_argument("--base-commit", default=None,
                        help="optional revision which must resolve to the actual current HEAD")
    parser.add_argument("--baseline-label", default=None, help="historical baseline label; never provenance")
    parser.add_argument("--profile-step", type=float, default=2.0)
    parser.add_argument("--maximum-profile-iterations", type=int, default=200)
    parser.add_argument("--maximum-profile-seconds", type=float, default=30.0)
    parser.add_argument("--convergence-criteria-json", default=None,
                        help="JSON object declaring six explicit geometry/state convergence budgets")
    args = parser.parse_args(argv)
    _validate_finite_positive("nlp_tolerance", args.nlp_tolerance)
    if not math.isfinite(args.speed_domain_margin_mps) or args.speed_domain_margin_mps < 0.0:
        parser.error("--speed-domain-margin-mps must be finite and nonnegative")
    _off_collocation_fractions(args.off_collocation_divisions)
    _validate_dense_maximum_refinements(args.dense_maximum_refinements)
    args.convergence_criteria = (_convergence_criteria(json.loads(args.convergence_criteria_json))
                                 if args.convergence_criteria_json is not None else None)
    run_matrix(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
