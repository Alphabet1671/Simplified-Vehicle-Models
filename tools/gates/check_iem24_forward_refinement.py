#!/usr/bin/env python3
"""Compare two independent forward-driver attempts on a shared physical distance grid.

This is a post-run evidence check, not an in-run claim. The two attempts must
have identical initial plant and pedal states, planner, and vehicle sources.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

# Conservatively bounded maximum differences between independently integrated
# physical trajectories. These are validation criteria, not solver tolerances.
LIMITS = {
    "lap_time_s": .02,
    "body_speed_mps": .03,
    "lateral_error_m": .01,
    "heading_error_rad": .01,
    "yaw_rate_rad_s": .02,
    "wheel_speed_rad_s": .30,
    "steering_rad": .01,
}


def compare(reference: Path, refined: Path) -> dict:
    a = json.loads((reference / "summary.json").read_text())
    b = json.loads((refined / "summary.json").read_text())
    ta = np.loadtxt(reference / "trace.csv", delimiter=",", skiprows=1)
    tb = np.loadtxt(refined / "trace.csv", delimiter=",", skiprows=1)
    pa = np.loadtxt(reference / "pedal.csv", delimiter=",", skiprows=1)
    pb = np.loadtxt(refined / "pedal.csv", delimiter=",", skiprows=1)
    sa, sb = a["settings"], b["settings"]
    if (a.get("lap_completed") is not True or b.get("lap_completed") is not True
            or not np.array_equal(ta[0, 1:], tb[0, 1:])
            or not np.array_equal(pa[0], pb[0])
            or a["source_hashes"] != b["source_hashes"]
            or sa.get("track") != sb.get("track")
            or any(sa[key] != sb[key] for key in
                   ("planner", "maximum_speed", "lateral_acceleration", "pedal_travel_time", "spacing"))
            or not float(sb["rtol"]) < float(sa["rtol"])
            or not float(sb["maximum_step"]) < float(sa["maximum_step"])):
        raise ValueError("refinement requires completed runs, same initial state/model/plan and tighter settings")
    distance = np.linspace(0., min(ta[-1, 1], tb[-1, 1]), 1017)
    def sampled(trace, column):
        return np.interp(distance, trace[:, 1], trace[:, column])
    metrics = {
        "lap_time_s": abs(float(ta[-1, 0] - tb[-1, 0])),
        "body_speed_mps": float(np.max(np.abs(
            np.hypot(sampled(ta, 4), sampled(ta, 5)) -
            np.hypot(sampled(tb, 4), sampled(tb, 5))))),
        "lateral_error_m": float(np.max(np.abs(sampled(ta, 2) - sampled(tb, 2)))),
        "heading_error_rad": float(np.max(np.abs(sampled(ta, 3) - sampled(tb, 3)))),
        "yaw_rate_rad_s": float(np.max(np.abs(sampled(ta, 6) - sampled(tb, 6)))),
        "wheel_speed_rad_s": float(max(np.max(np.abs(sampled(ta, i) - sampled(tb, i)))
                                     for i in range(7, 11))),
        "steering_rad": float(np.max(np.abs(sampled(ta, 12) - sampled(tb, 12)))),
    }
    return {
        "reference": str(reference), "refined": str(refined),
        "reference_rtol": float(sa["rtol"]), "refined_rtol": float(sb["rtol"]),
        "reference_maximum_step_s": float(sa["maximum_step"]),
        "refined_maximum_step_s": float(sb["maximum_step"]),
        "maximum_absolute_differences": metrics, "limits": LIMITS,
        "refinement_validated": bool(all(np.isfinite(value) and value <= LIMITS[key]
                                         for key, value in metrics.items())),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("refined", type=Path)
    args = parser.parse_args()
    result = compare(args.reference, args.refined)
    destination = args.refined / "refinement_comparison.json"
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    # The runner cannot know the outcome of an independent second integration.
    # Certify its saved summary only after the paired comparison is available.
    summary_path = args.refined / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["refinement_validated"] = result["refinement_validated"]
    # One dense midpoint per interval is evidence, not a continuous proof.
    summary["midpoint_sample_validation_complete"] = bool(
        summary.get("midpoint_sample_validation_complete",
                    summary.get("between_step_validation_complete", False)))
    # Only tools/gates/certify_iem24_forward_between_step.py may set this flag, and
    # only when every between-step certificate passes. Never raise it here.
    summary["between_step_validation_complete"] = bool(
        summary.get("between_step_validation_complete", False))
    summary["refinement_evidence"] = destination.name
    from tools.run.run_iem24_forward_driver import acceptance_gate
    summary["usable"] = acceptance_gate(summary)
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n",
                            encoding="utf-8")
    print(destination, result["refinement_validated"], "usable:", summary["usable"])
