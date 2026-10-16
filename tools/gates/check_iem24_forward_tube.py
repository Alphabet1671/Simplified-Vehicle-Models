#!/usr/bin/env python3
"""Estimate an empirical error tube from three refinement levels of one lap.

Inputs are three completed attempts of the same lap, ordered coarse, medium and
fine (each tighter than the last), every one holding ``dense_polynomials.npz``.
The BDF dense polynomials are compared *continuously* on the shared time span
(dense sampling inside every interval of the finer run), not only at nodes.

Pre-registered pass criteria (fixed before any run):

* Convergence: the component-wise sup difference of (medium, fine) is smaller
  than that of (coarse, medium), for every state component.
* The tube is the extrapolated remaining error of the fine run,
  ``d_mf * r / (1 - r)`` with ``r = d_mf / d_cm`` capped below one, plus
  ``d_mf`` itself. It is widened to at least ``d_mf``.
* Every dense-domain margin of the fine run, recomputed with its bounds
  widened by *twice* the tube, stays positive.

This is empirical support from three levels, not a proof: the tube estimates
the distance between the fine run and the converged ODE solution under an
assumed geometric convergence. It is saved as ``tube_certificate.json``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SAMPLES_PER_INTERVAL = 9


def load(attempt: Path):
    data = np.load(attempt / "dense_polynomials.npz")
    summary = json.loads((attempt / "summary.json").read_text())
    return data, summary


def evaluate(data, times):
    """Evaluate the stored piecewise polynomials at ``times`` -> (n_states, len)."""
    edges = data["times"]
    index = np.clip(np.searchsorted(edges, times, side="right") - 1, 0, data["t_end"].size - 1)
    s = (times - data["t_end"][index]) / data["step"][index]
    powers = s[:, None] ** np.arange(6)[None, :]
    return np.einsum("tj,tjn->nt", powers, data["coefficients"][index])


def sample_times(data, end):
    edges = data["times"]
    edges = edges[edges <= end]
    fractions = np.linspace(0., 1., SAMPLES_PER_INTERVAL, endpoint=False)
    inside = (edges[:-1, None] + np.diff(edges)[:, None] * fractions[None, :]).ravel()
    return np.append(inside, end)


def sup_difference(first, second):
    end = min(first["times"][-1], second["times"][-1])
    finer = first if first["times"].size >= second["times"].size else second
    times = sample_times(finer, end)
    return np.max(np.abs(evaluate(first, times) - evaluate(second, times)), axis=1), float(end)


def build(coarse: Path, medium: Path, fine: Path, vehicle, plan):
    from simplified_vehicle_models.solvers.planar_dense_certificate import certify_planar_dense_domain
    levels = [load(path) for path in (coarse, medium, fine)]
    for (_, summary), path in zip(levels, (coarse, medium, fine)):
        if not summary.get("lap_completed") or "dense_polynomials_file" not in summary:
            raise ValueError(f"{path} lacks a completed lap with dense polynomials")
    hashes = {json.dumps(summary["source_hashes"], sort_keys=True) for _, summary in levels}
    if len(hashes) != 1:
        raise ValueError("levels must share identical source hashes")
    steps = [float(summary["settings"]["maximum_step"]) for _, summary in levels]
    rtols = [float(summary["settings"]["rtol"]) for _, summary in levels]
    if not (steps[0] > steps[1] > steps[2] and rtols[0] > rtols[1] > rtols[2]):
        raise ValueError("levels must be strictly refined in step and tolerance")
    (cm, _), (mf, end) = sup_difference(levels[0][0], levels[1][0]), sup_difference(levels[1][0], levels[2][0])
    cf, _ = sup_difference(levels[0][0], levels[2][0])
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(cm > 0., mf / cm, 0.)
    shrinks = bool(np.all(mf < cm))
    capped = np.minimum(ratio, 0.9)
    tube = mf + mf * capped / (1. - capped)
    fine_data = levels[2][0]
    bounds = np.asarray(fine_data["bounds"], dtype=float)
    widened = np.stack([bounds[:, :, 0] - 2. * tube[None, :], bounds[:, :, 1] + 2. * tube[None, :]], axis=-1)
    plain = certify_planar_dense_domain(vehicle, plan, bounds, True)
    domain = certify_planar_dense_domain(vehicle, plan, widened, True)
    passed = bool(shrinks and plain["certified"] and domain["certified"] and np.all(np.isfinite(tube)))
    names = ["time_s", "s", "n", "heading", "v_x", "v_y", "yaw_rate", "w_fl", "w_fr", "w_rl", "w_rr",
             "soc", "steering", "pedal"]
    names = (names[1:] if len(tube) == 13 else names)[:len(tube)]
    return {
        "passed": passed,
        "levels": {"coarse": str(coarse), "medium": str(medium), "fine": str(fine)},
        "maximum_step": steps, "rtol": rtols, "shared_time_span_s": end,
        "sup_difference_coarse_medium": dict(zip(names, map(float, cm))),
        "sup_difference_medium_fine": dict(zip(names, map(float, mf))),
        "sup_difference_coarse_fine": dict(zip(names, map(float, cf))),
        "observed_ratio": dict(zip(names, map(float, ratio))),
        "all_components_shrink": shrinks,
        "tube": dict(zip(names, map(float, tube))),
        "domain_margins_plain": plain.get("margins"),
        "domain_margins_widened_by_twice_tube": domain.get("margins"),
        "domain_widened_certified": bool(domain["certified"]),
        "scope": "empirical, from three refinement levels under an assumed geometric "
                 "convergence; not a proof about the true ODE solution",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("coarse", type=Path)
    parser.add_argument("medium", type=Path)
    parser.add_argument("fine", type=Path)
    args = parser.parse_args(argv)
    sys.path.insert(0, ".")
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.tracks.profile import load_track
    from tools.run_iem24_forward_driver import FORMULATION
    settings = json.loads((args.fine / "summary.json").read_text())["settings"]
    vehicle = compile_formulation(str(FORMULATION)).model
    track = load_track(settings["track"])
    count = int(np.ceil(track.lap_length / settings["spacing"]))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live = settings["planner"] == "live"
    cap = settings.get("lateral_acceleration")
    if cap is None and not live:
        cap = 2.
    plan = build_speed_plan(distance, curvature, settings["maximum_speed"], cap, 1., 2.,
                            vehicle=vehicle if live else None, live_limit=live)
    result = build(args.coarse, args.medium, args.fine, vehicle, plan)
    destination = args.fine / "tube_certificate.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(destination, "passed:", result["passed"])
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
