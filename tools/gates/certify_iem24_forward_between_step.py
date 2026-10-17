#!/usr/bin/env python3
"""Decide whether a refined forward lap earns ``between_step_validation_complete``.

The flag is raised only when every certificate below passes. Nothing here
weakens ``acceptance_gate``; the tool only reports each certificate and sets the
flag on the saved summary when all of them hold.

1. ``dense_polynomial_domain_certified``: the BDF interpolant stays inside the
   explicit plant domain on every interval (from the run summary).
2. ``tube_certificate.json`` passed: three refinement levels shrink and the
   domain margins survive widening by twice the empirical tube.
3. Contact-load analytic conditions hold (run summary).
4. Powertrain monotonicity audit and contact grid audit tests pass.
5. Load-closure interval (Krawczyk) certificate, with adaptive time bisection of
   the dense polynomials, certifies *every* sub-span of *every* interval. Failure here, however small, keeps
   the flag false: a sampled or partial result is reported but never promoted.

The load-closure guesses come from the quasi-static map at each accepted node
(the map does not depend on motor torque).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import unittest
from pathlib import Path

import numpy as np

AUDITS = ("tests.test_powertrain_closure_audit", "tests.test_contact_condition_grid")


def run_audits():
    suite = unittest.defaultTestLoader.loadTestsFromNames(AUDITS)
    outcome = unittest.TextTestRunner(stream=open("/dev/null", "w"), verbosity=0).run(suite)
    return {"tests": list(AUDITS), "run": outcome.testsRun, "passed": outcome.wasSuccessful()}


def _node_guesses(vehicle, trace):
    from simplified_vehicle_models.formulations.planar_7dof import PlanarState, evaluate_planar_dynamics
    guesses = np.empty((trace.shape[0], 2))
    for row, x in enumerate(trace):
        state = PlanarState(x[3], x[4], x[5], x[6:10], x[10])
        evaluation = evaluate_planar_dynamics(vehicle, state, np.zeros(4), x[11])
        guesses[row] = (evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                        evaluation.lateral_acceleration + state.yaw_rate * state.v_x)
    return guesses


def _certify_chunk(arguments):
    """Worker: certify one contiguous run of dense intervals."""
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.solvers.dense_domain import DensePolynomials
    from simplified_vehicle_models.solvers.load_interval import certify_load_closure_adaptive
    formulation, arrays, guesses, depth = arguments
    vehicle = compile_formulation(formulation).model
    poly = DensePolynomials(*arrays)
    return certify_load_closure_adaptive(vehicle, poly, guesses, maximum_depth=depth,
                                         components=slice(0, 12))


def load_closure(vehicle, attempt: Path, workers: int, depth: int = 16):
    from concurrent.futures import ProcessPoolExecutor
    from tools.run.run_iem24_forward_driver import FORMULATION
    data = np.load(attempt / "dense_polynomials.npz")
    trace = np.loadtxt(attempt / "trace.csv", delimiter=",", skiprows=1)[:, 1:]
    n = data["t_end"].size
    if trace.shape[0] != n + 1:
        return {"certified": False, "reason": "trace rows do not match dense intervals"}
    guesses = _node_guesses(vehicle, trace)
    edges = np.linspace(0, n, max(1, workers) * 4 + 1).astype(int)
    jobs = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b > a:
            sl = slice(a, b)
            arrays = (data["times"][a:b + 1], data["t_end"][sl], data["step"][sl],
                      data["s_lower"][sl], data["s_upper"][sl], data["order"][sl],
                      data["coefficients"][sl])
            jobs.append((str(FORMULATION), arrays, guesses[a:b + 1], depth))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        parts = list(pool.map(_certify_chunk, jobs))
    ok = all(part.get("certified") is True for part in parts)
    result = {
        "certified": bool(ok and len(parts) == len(jobs)),
        "dense_intervals": int(n),
        "chunks": len(parts),
        "spans_certified": int(sum(part.get("spans_certified", 0) for part in parts)),
        "spans_failed_at_maximum_depth": int(sum(part.get("spans_failed_at_maximum_depth", 0)
                                                  for part in parts)),
        "deepest_bisection": int(max(part.get("deepest_bisection", 0) for part in parts)),
        "maximum_depth": depth,
        "largest_radius_scale": float(max(part.get("largest_radius_scale", 0.0) or 0.0
                                          for part in parts)),
        "scope": parts[0].get("scope", ""),
    }
    return json.loads(json.dumps(result, default=float, allow_nan=False))


def vertical_dense_domain(vehicle, attempt, summary):
    """Run the narrow 14DoF interpolant state-domain certificate independently."""
    from simplified_vehicle_models.solvers.dense_domain import DensePolynomials
    from simplified_vehicle_models.solvers.vertical_dense_certificate import certify_vertical_dense_domain
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    data = np.load(attempt / "dense_polynomials.npz")
    bounds = data.get("bounds")
    if bounds is None:
        return {"certified": False, "reason": "dense polynomials contain no bounds"}
    settings = summary.get("settings", {})
    track = load_track(settings.get("track"))
    spacing = float(settings.get("spacing", .5))
    distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / spacing)) + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    planner = settings.get("planner", "conservative")
    live_limit = planner == "live"
    lateral_cap = settings.get("lateral_acceleration")
    if lateral_cap in (None, "None") and not live_limit:
        lateral_cap = 2.0
    elif lateral_cap not in (None, "None"):
        lateral_cap = float(lateral_cap)
    control = getattr(vehicle, "planar", vehicle)
    plan = build_speed_plan(
        distance, curvature, float(settings.get("maximum_speed", 8.)),
        lateral_cap, 1., 2., vehicle=control if live_limit else None,
        live_limit=live_limit)
    pedal = bounds.shape[1] == 39
    return certify_vertical_dense_domain(vehicle, plan, bounds, summary, pedal_dynamics=pedal)


def source_hash_check(summary, model=None):
    """Compare the run's recorded source hashes with the files on disk now.

    The certifier reads ``load_interval.py`` and friends from the working tree,
    so a certificate is only meaningful for a run whose snapshot hashes match.
    """
    import hashlib
    recorded = summary.get("source_hashes") or {}
    mismatched, missing = [], []
    for name, digest in recorded.items():
        path = Path(name)
        if not path.exists():
            missing.append(name)
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            mismatched.append(name)
    model = model or "planar-7dof"
    required = (("load_interval.py", "dense_domain.py", "planar_7dof.py")
                if model == "planar-7dof" else
                ("dense_domain.py", "vertical_dense_certificate.py", "vertical_14dof.py"))
    covered = all(any(key.endswith(need) for key in recorded) for need in required)
    return {"passed": bool(recorded) and covered and not mismatched and not missing,
            "mismatched": mismatched, "missing": missing, "covers_required_sources": covered}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fine", type=Path, help="finest attempt directory")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)
    sys.path.insert(0, ".")
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from tools.run.run_iem24_forward_driver import FORMULATION, acceptance_gate, VERTICAL_FORMULATION
    summary_path = args.fine / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    model = summary.get("model", "planar-7dof")
    if model not in ("planar-7dof", "14dof-simple"):
        raise ValueError(f"unsupported forward model {model!r}")
    formulation = VERTICAL_FORMULATION if model == "14dof-simple" else FORMULATION
    vehicle = compile_formulation(str(formulation)).model
    tube_path = args.fine / "tube_certificate.json"
    tube = json.loads(tube_path.read_text()) if tube_path.exists() else {"passed": False}
    model = summary.get("model", "planar-7dof")
    hashes = source_hash_check(summary, model)
    audits = run_audits()
    if model == "14dof-simple":
        dense = vertical_dense_domain(vehicle, args.fine, summary)
        report = {"between_step_validation_complete": False,
                  "checks": {"vertical_dense_domain": bool(dense.get("certified")),
                              "run_source_hashes_match_working_tree": bool(hashes["passed"])},
                  "vertical_dense_domain": dense, "source_hash_check": hashes,
                  "note": "vertical state-domain scope is not a full between-step certificate"}
        (args.fine / "between_step_certificate.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        summary["between_step_validation_complete"] = False
        summary["between_step_checks"] = report["checks"]
        summary["usable"] = acceptance_gate(summary)
        summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(args.fine, report["checks"], "between_step: False", "usable:", summary["usable"])
        return 1
    started = time.perf_counter()
    closure = load_closure(vehicle, args.fine, args.workers)
    closure["wall_seconds"] = time.perf_counter() - started
    checks = {
        "dense_polynomial_domain": bool(summary.get("dense_polynomial_domain_certified")),
        "refinement_tube": bool(tube.get("passed")),
        "contact_conditions": bool(summary.get("contact_nonnegativity_conditions_hold")),
        "powertrain_and_contact_audits": bool(audits["passed"]),
        "load_closure_interval_certificate": bool(closure.get("certified")),
        "run_source_hashes_match_working_tree": bool(hashes["passed"]),
    }
    complete = all(checks.values())
    report = {"between_step_validation_complete": complete, "checks": checks,
              "source_hash_check": hashes, "audits": audits, "load_closure": closure,
              "note": "flag is raised only if every check passed; no check is waived"}
    (args.fine / "between_step_certificate.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    summary["between_step_validation_complete"] = complete
    summary["between_step_checks"] = checks
    summary["load_closure_interval_certificate"] = {
        key: closure.get(key) for key in ("certified", "dense_intervals", "spans_certified",
                                      "spans_failed_at_maximum_depth", "deepest_bisection",
                                      "wall_seconds", "scope")}
    summary["usable"] = acceptance_gate(summary)
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(args.fine, checks, "between_step:", complete, "usable:", summary["usable"])
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
