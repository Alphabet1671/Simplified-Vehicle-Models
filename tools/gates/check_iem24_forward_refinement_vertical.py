#!/usr/bin/env python3
"""Compare two 14DoF forward-driver attempts on a shared physical distance grid.

The planar checker ``check_iem24_forward_refinement.py`` addresses trace columns
by position (a 13-column layout), so it cannot read the 39-column vertical
trace. This tool applies the *same* pass limits (not looser) to the same
physical quantities, addressed by state name, and adds the vertical rung's own
requirement that every matched tire state stays within its periodic tolerance.

The two attempts must share initial state, pedal, planner, steering settings,
track and vehicle sources, and the refined run must have tighter rtol and step.
Like the planar checker it is a post-run evidence check; it never raises
``between_step_validation_complete``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

#: Same limits as the planar checker, mapped to the vertical state names.
LIMITS = {
    "lap_time_s": .02,
    "body_speed_mps": .03,
    "lateral_error_m": .01,
    "heading_error_rad": .01,
    "yaw_rate_rad_s": .02,
    "wheel_speed_rad_s": .30,
    "steering_rad": .01,
}
TIRE_PREFIXES = ("kappa_eff", "lateral_deformation", "trail_angle")
MATCH_SAMPLES = 1017
SETTING_KEYS = ("planner", "maximum_speed", "lateral_acceleration", "pedal_travel_time", "spacing",
                "preview_seconds", "heading_gain", "lateral_gain")


def _plant():
    from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
    return VerticalForwardPlant


def compare(reference: Path, refined: Path) -> dict:
    plant = _plant()
    names = tuple(plant.state_names)
    col = {name: 1 + i for i, name in enumerate(names)}
    a = json.loads((reference / "summary.json").read_text())
    b = json.loads((refined / "summary.json").read_text())
    ta = np.loadtxt(reference / "trace.csv", delimiter=",", skiprows=1)
    tb = np.loadtxt(refined / "trace.csv", delimiter=",", skiprows=1)
    pa = np.loadtxt(reference / "pedal.csv", delimiter=",", skiprows=1)
    pb = np.loadtxt(refined / "pedal.csv", delimiter=",", skiprows=1)
    sa, sb = a["settings"], b["settings"]
    if ta.shape[1] != 1 + len(names) or tb.shape[1] != 1 + len(names):
        raise ValueError("traces do not have the 14DoF column layout")
    if (a.get("model") != "14dof-simple" or b.get("model") != "14dof-simple"
            or a.get("lap_completed") is not True or b.get("lap_completed") is not True
            or not np.array_equal(ta[0, 1:], tb[0, 1:])
            or not np.array_equal(pa[0], pb[0])
            or a["source_hashes"] != b["source_hashes"]
            or sa.get("track") != sb.get("track")
            or any(sa.get(key) != sb.get(key) for key in SETTING_KEYS)
            or not float(sb["rtol"]) < float(sa["rtol"])
            or not float(sb["maximum_step"]) < float(sa["maximum_step"])):
        raise ValueError("refinement requires completed 14DoF runs, same initial state/model/plan/"
                         "steering settings and tighter settings")
    distance = np.linspace(0., min(ta[-1, col["s"]], tb[-1, col["s"]]), MATCH_SAMPLES)

    def sampled(trace, name):
        return np.interp(distance, trace[:, col["s"]], trace[:, col[name]])

    def diff(name):
        return float(np.max(np.abs(sampled(ta, name) - sampled(tb, name))))

    speed_a = np.hypot(sampled(ta, "v_x"), sampled(ta, "v_y"))
    speed_b = np.hypot(sampled(tb, "v_x"), sampled(tb, "v_y"))
    metrics = {
        "lap_time_s": abs(float(ta[-1, 0] - tb[-1, 0])),
        "body_speed_mps": float(np.max(np.abs(speed_a - speed_b))),
        "lateral_error_m": diff("n"),
        "heading_error_rad": diff("heading_error"),
        "yaw_rate_rad_s": diff("yaw_rate"),
        "wheel_speed_rad_s": float(max(diff(f"wheel_speed_{c}") for c in ("fl", "fr", "rl", "rr"))),
        "steering_rad": diff("steering"),
    }
    tolerance = dict(zip((names[i] for i in plant.periodic_indices),
                         map(float, plant.periodic_tolerances)))
    tire = {name: diff(name) / tolerance[name] for name in names if name.startswith(TIRE_PREFIXES)}
    worst_name = max(tire, key=tire.get) if tire else None
    worst_ratio = float(tire[worst_name]) if worst_name else None
    ok = bool(all(np.isfinite(v) and v <= LIMITS[k] for k, v in metrics.items())
              and worst_ratio is not None and np.isfinite(worst_ratio) and worst_ratio <= 1.)
    return {
        "reference": str(reference), "refined": str(refined),
        "reference_rtol": float(sa["rtol"]), "refined_rtol": float(sb["rtol"]),
        "reference_maximum_step_s": float(sa["maximum_step"]),
        "refined_maximum_step_s": float(sb["maximum_step"]),
        "maximum_absolute_differences": metrics, "limits": LIMITS,
        "worst_tire_state": worst_name, "worst_tire_state_periodic_ratio": worst_ratio,
        "tire_state_ratio_limit": 1.0,
        "refinement_validated": ok,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("refined", type=Path)
    args = parser.parse_args(argv)
    result = compare(args.reference, args.refined)
    destination = args.refined / "refinement_comparison.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    summary_path = args.refined / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["refinement_validated"] = result["refinement_validated"]
    summary["midpoint_sample_validation_complete"] = bool(
        summary.get("midpoint_sample_validation_complete",
                    summary.get("between_step_validation_complete", False)))
    # Only certify_iem24_forward_between_step.py may raise this flag; never here.
    summary["between_step_validation_complete"] = bool(
        summary.get("between_step_validation_complete", False))
    summary["refinement_evidence"] = destination.name
    from tools.run.run_iem24_forward_driver import acceptance_gate
    summary["usable"] = acceptance_gate(summary)
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(destination, result["refinement_validated"], "usable:", summary["usable"])
    return 0 if result["refinement_validated"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
