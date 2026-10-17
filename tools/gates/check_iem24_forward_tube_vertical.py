#!/usr/bin/env python3
"""Empirical refinement tube for the vertical 14DoF, from three refinement levels.

The planar tube tool ``check_iem24_forward_tube.py`` is byte-pinned by tests and
hard-wires the planar dense-domain check, so this sibling applies the *same*
pre-registered criteria to the 14DoF:

* Convergence: the component-wise sup difference of (medium, fine) is smaller
  than that of (coarse, medium), for every state component.
* The tube is ``d_mf + d_mf * r / (1 - r)`` with ``r = d_mf / d_cm`` capped at 0.9.
* The vertical evaluation-domain certificate
  (``solvers/vertical_between_step.py``) must hold on the fine run's dense
  enclosure and on that enclosure widened by *twice* the tube.

This is empirical support from three levels under an assumed geometric
convergence, not a proof about the true ODE solution. It writes
``tube_certificate.json`` next to the fine attempt and never touches a gate flag.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from tools.gates.check_iem24_forward_tube import load, sup_difference


def build(coarse: Path, medium: Path, fine: Path, certify):
    levels = [load(path) for path in (coarse, medium, fine)]
    for (_, summary), path in zip(levels, (coarse, medium, fine)):
        if summary.get("model") != "14dof-simple":
            raise ValueError(f"{path} is not a 14DoF attempt")
        if not summary.get("lap_completed") or "dense_polynomials_file" not in summary:
            raise ValueError(f"{path} lacks a completed lap with dense polynomials")
    if len({json.dumps(summary["source_hashes"], sort_keys=True) for _, summary in levels}) != 1:
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
    bounds = np.asarray(levels[2][0]["bounds"], dtype=float)
    if bounds.shape[1] != len(tube):
        raise ValueError("enclosure and polynomial component counts differ")
    widened = np.stack([bounds[:, :, 0] - 2. * tube[None, :], bounds[:, :, 1] + 2. * tube[None, :]], axis=-1)
    plain, domain = certify(bounds), certify(widened)
    passed = bool(shrinks and plain["certified"] and domain["certified"] and np.all(np.isfinite(tube)))
    names = [f"state_{i}" for i in range(len(tube))]
    return {
        "passed": passed, "model": "14dof-simple",
        "levels": {"coarse": str(coarse), "medium": str(medium), "fine": str(fine)},
        "maximum_step": steps, "rtol": rtols, "shared_time_span_s": end,
        "sup_difference_coarse_medium": dict(zip(names, map(float, cm))),
        "sup_difference_medium_fine": dict(zip(names, map(float, mf))),
        "sup_difference_coarse_fine": dict(zip(names, map(float, cf))),
        "observed_ratio": dict(zip(names, map(float, ratio))),
        "all_components_shrink": shrinks,
        "tube": dict(zip(names, map(float, tube))),
        "evaluation_domain_plain": {k: plain.get(k) for k in ("certified", "checks", "margins")},
        "evaluation_domain_widened_by_twice_tube": {k: domain.get(k) for k in ("certified", "checks", "margins")},
        "scope": "empirical, from three refinement levels under an assumed geometric "
                 "convergence; not a proof about the true ODE solution",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("coarse", "medium", "fine"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args(argv)
    sys.path.insert(0, ".")
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan
    from simplified_vehicle_models.solvers.vertical_between_step import certify_vertical_between_step
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.tracks.profile import load_track
    from tools.run.run_iem24_forward_driver import VERTICAL_FORMULATION
    settings = json.loads((args.fine / "summary.json").read_text())["settings"]
    vehicle = compile_formulation(str(VERTICAL_FORMULATION)).model
    track = load_track(settings["track"])
    distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / settings["spacing"])) + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live = settings["planner"] in ("live", "live_live_acceleration_envelope")
    cap = settings.get("lateral_acceleration")
    if cap in (None, "None"):
        cap = None if live else 2.
    plan = build_speed_plan(distance, curvature, settings["maximum_speed"], cap, 1., 2.,
                            vehicle=vehicle.planar if live else None, live_limit=live)
    result = build(args.coarse, args.medium, args.fine,
                   lambda bounds: certify_vertical_between_step(vehicle, plan, bounds, None, bounds.shape[1] == 39))
    destination = args.fine / "tube_certificate.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(destination, "passed:", result["passed"])
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
