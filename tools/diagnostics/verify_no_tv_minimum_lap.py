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
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh, track_sample
from simplified_vehicle_models.solvers.qss_spline_ocp import initial_guess, solve_spline_line
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track

DEFAULT_MESH_SPACINGS = (5.0, 2.5, 1.25)
DEFAULT_BASIS_SPACINGS = (35.0, 17.5)
DEFAULT_FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")
DEFAULT_SURFACE = Path("runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz")
SUCCESS_STATUSES = {"Solve_Succeeded", "success", "succeeded"}


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


def build_attempt(case: MatrixCase, *, mesh_spacing_m: float, basis_spacing_m: float,
                  maximum_iterations: int, wall_cap_s: float, seed_name: str,
                  formulation: Path, surface: Path) -> dict:
    """Build a fully serializable solve request; this has no solver side effects."""
    if mesh_spacing_m not in DEFAULT_MESH_SPACINGS or basis_spacing_m not in DEFAULT_BASIS_SPACINGS:
        # Bounded subsets may choose only preregistered rungs, never tuned values.
        raise ValueError("mesh and basis values must be preregistered Task 5 rungs")
    if (isinstance(maximum_iterations, bool) or int(maximum_iterations) != maximum_iterations
            or int(maximum_iterations) <= 0):
        raise ValueError("maximum_iterations must be a positive integer")
    if not math.isfinite(float(wall_cap_s)) or float(wall_cap_s) <= 0:
        raise ValueError("wall cap must be finite and positive")
    if seed_name not in {"centerline", "low_amplitude_offset"}: 
        raise ValueError("seed must be centerline or low_amplitude_offset")
    settings = QssSettings(tolerance=1e-6, maximum_iterations=int(maximum_iterations),
                           start_speed=float(case.start_speed_mps), open_regularisation=1e-6)
    return {
        "case": asdict(case), "mesh_spacing_m": float(mesh_spacing_m),
        "basis_spacing_m": float(basis_spacing_m), "seed": seed_name,
        "wall_cap_s": float(wall_cap_s), "settings": asdict(settings),
        "formulation": str(Path(formulation).resolve()), "surface": str(Path(surface).resolve()),
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
        # Project the declared offset coefficients into the production decision
        # layout while retaining the production envelope speed/control initializer.
        seed = initial_guess(track, corridor, surface, mesh, spline, settings)
        coefficients = _seed_for(spline, request["seed"])
        seed[:spline.control_count] = coefficients
        result["initial_seed"] = _seed_domain_report(track, corridor, spline, coefficients)
        solution = solve_spline_line(track, corridor, surface, mesh, spline, settings, seed=seed)
        np.savez_compressed(artifact_path, distance=solution.distance, offset=solution.offset,
                            heading_error=solution.heading_error, curvature=solution.curvature,
                            speed=solution.speed, acceleration=solution.acceleration,
                            midpoint_acceleration=solution.midpoint_acceleration,
                            coefficients=solution.coefficients)
        dense = validate_line(solution, corridor, surface, settings, vehicle=vehicle)
        checks = dict(dense["checks"])
        # These are intentionally unavailable until matrix aggregation can prove
        # their paired evidence.  They must never be inferred from one rung.
        checks.update({"mesh_convergence": None, "basis_convergence": None,
                       "matched_baseline": None, "replay_agreement": None})
        strict = strict_validation(solution.status in SUCCESS_STATUSES, checks, REQUIRED_NUMERICAL_CHECKS)
        result.update({"status": str(solution.status), "solver_succeeded": True,
                       "accepted": False, "objective_s": solution.lap_time,
                       "objective_value": solution.objective_value,
                       "iterations": solution.iterations,
                       "primal_error": solution.maximum_violation,
                       "dual_error": None, "dense_original_model": dense,
                       "checks": checks, "strict": strict,
                       "validation_status": strict["validation_status"],
                       "surface_representation": representation,
                       "artifact": str(Path(artifact_path).name),
                       "start_treatment": case.start_treatment,
                       "candidate_only_reason": case.candidate_only_reason,
                       "replay": {"available": False, "reason": "matched exact-trajectory point-mass replay is not supplied by this harness"}})
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
    with np.load(left) as a, np.load(right) as b:
        common = np.linspace(max(a["distance"][0], b["distance"][0]), min(a["distance"][-1], b["distance"][-1]), 256)
        answer = {}
        for name in ("offset", "speed", "curvature"):
            answer[f"rms_{'path' if name == 'offset' else name}"] = float(np.sqrt(np.mean((np.interp(common, a["distance"], a[name]) - np.interp(common, b["distance"], b[name])) ** 2)))
        return answer


def _dense_live_qualified(row: dict) -> bool:
    """Require successful solve plus dense geometry, feasibility, and live model."""
    dense = row.get("dense_original_model", {})
    checks = dense.get("checks", {})
    return (row.get("solver_succeeded") is True and row.get("artifact") is not None
            and checks.get("geometry_consistency") is True
            and checks.get("dense_feasibility") is True
            and dense.get("model_feasible") is True)


def _same_refinement_conditions(left: dict, right: dict) -> bool:
    """Forbid seed/settings/source changes from masquerading as refinement."""
    a, b = left["request"], right["request"]
    return (a.get("seed") == b.get("seed") and a.get("settings") == b.get("settings")
            and a.get("source_hashes") == b.get("source_hashes")
            and left.get("surface_representation") == right.get("surface_representation"))


def _comparison(case: str, left: dict, right: dict, output: Path, *, kind: str) -> dict | None:
    try:
        metrics = _rms_difference(output / case / left["artifact"], output / case / right["artifact"])
    except (FileNotFoundError, ValueError):
        return None
    return {"case": case, "kind": kind,
            "left": {"mesh_m": left["request"]["mesh_spacing_m"], "basis_m": left["request"]["basis_spacing_m"], "time_s": left.get("objective_s")},
            "right": {"mesh_m": right["request"]["mesh_spacing_m"], "basis_m": right["request"]["basis_spacing_m"], "time_s": right.get("objective_s")},
            "time_difference_s": (left.get("objective_s") - right.get("objective_s")) if left.get("objective_s") is not None and right.get("objective_s") is not None else None,
            **metrics}


def aggregate(results: list[dict], output: Path) -> dict:
    """Separate candidate differences from qualified one-axis refinement evidence."""
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
                if not (_dense_live_qualified(left) and _dense_live_qualified(right)
                        and _same_refinement_conditions(left, right)):
                    continue
                if basis_equal and not mesh_equal:
                    evidence = _comparison(name, left, right, output, kind="mesh_refinement")
                    if evidence is not None:
                        mesh_refinement.append(evidence)
                elif mesh_equal and not basis_equal:
                    evidence = _comparison(name, left, right, output, kind="basis_refinement")
                    if evidence is not None:
                        basis_refinement.append(evidence)
    return {"attempts": len(results), "candidate_diagnostics": candidate_diagnostics,
            "mesh_refinement": mesh_refinement, "basis_refinement": basis_refinement,
            "strict_aggregate": {"accepted": False, "status": "incomplete",
              "reason": "requires qualified independent mesh and basis refinement plus matched baseline and replay; none is supplied automatically"}}


def run_matrix(args) -> dict:
    output = Path(args.output).resolve()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output must be a new exclusive directory")
    cases = default_cases()
    requested_names = args.cases.split(",") if args.cases else list(cases)
    selected = [cases[name] for name in requested_names]
    form, surface = Path(args.formulation).resolve(), Path(args.surface).resolve()
    hashes_before = source_hashes(*(case.track for case in selected), form, surface)
    manifest = {"purpose": "bounded Task5 no-TV reliability matrix; candidate evidence only", "base_commit": args.base_commit,
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
                    request = build_attempt(case, mesh_spacing_m=mesh, basis_spacing_m=basis,
                        maximum_iterations=args.maximum_iterations, wall_cap_s=args.wall_cap_s,
                        seed_name=seed, formulation=form, surface=surface)
                    request["source_hashes"] = hashes_before
                    attempt_name = f"mesh-{mesh:g}_basis-{basis:g}_{seed}"
                    result = run_bounded_worker(request, case_root / attempt_name)
                    result["attempt_id"] = f"{case.name}/{attempt_name}"
                    # Artifact is relative to its attempt directory until aggregation.
                    if result.get("artifact"):
                        result["artifact"] = f"{attempt_name}/{result['artifact']}"
                    manifest["results"].append(result)
                    _write_json_new_or_replace(output / "matrix.json", manifest)
    manifest["source_hashes_after"] = source_hashes(*(case.track for case in selected), form, surface)
    manifest["source_data_unchanged"] = manifest["source_hashes_before"] == manifest["source_hashes_after"]
    # Artifact paths stay relative to their case directory (attempt/solution.npz).
    manifest["aggregate"] = aggregate(manifest["results"], output)
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
    parser.add_argument("--wall-cap-s", type=float, default=300.0)
    parser.add_argument("--base-commit", default="6286c5fbb02217fc21148fdc0be276728f13b99c")
    args = parser.parse_args(argv)
    run_matrix(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
