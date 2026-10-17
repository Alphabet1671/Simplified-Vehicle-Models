#!/usr/bin/env python3
"""Bounded steering-gain sweep on the canonical forward lap.

Every candidate is one isolated cold run of ``run_iem24_forward_driver.py`` with
the canonical report settings (live planner, BDF, rtol 1e-5, speed cap 60 m/s,
step 0.02 s for the 7DoF; step 0.005 s for the 14DoF). Only the three steering
values vary. The sweep is diagnostic: it never edits a gate, and it records the
planner/solver settings the runner actually stored so a mismatch is visible.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

CANONICAL = {
    "planar-7dof": {"maximum_step": 0.02, "timeout": 300.},
    "14dof-simple": {"maximum_step": 0.005, "timeout": 1800.},
}
EXPECTED = {"planner": "live", "method": "BDF", "rtol": 1e-5, "maximum_speed": 60.}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=tuple(CANONICAL), default="planar-7dof")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--preview", type=float, nargs="+", default=[.15])
    parser.add_argument("--heading-gain", type=float, nargs="+", default=[1.5])
    parser.add_argument("--lateral-gain", type=float, nargs="+", default=[1.5])
    parser.add_argument("--maximum-runs", type=int, default=12)
    args = parser.parse_args(argv)
    configs = [(p, h, l) for p in args.preview for h in args.heading_gain for l in args.lateral_gain]
    if len(configs) > args.maximum_runs:
        parser.error(f"{len(configs)} candidates exceed --maximum-runs={args.maximum_runs}")
    canonical = CANONICAL[args.model]
    args.output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, (preview, heading, lateral) in enumerate(configs, 1):
        before = set(args.output_root.glob("attempt_*"))
        command = [sys.executable, "tools/run/run_iem24_forward_driver.py", "--model", args.model,
                   "--output-root", str(args.output_root),
                   "--track", "models/tracks/2024_michigan_endurance_multilap_closed.xml",
                   "--planner", "live", "--maximum-speed", "60.0", "--pedal-travel-time", "0.175",
                   "--maximum-step", str(canonical["maximum_step"]), "--rtol", "1e-05",
                   "--method", "BDF", "--spacing", "0.5", "--duration", "300.0",
                   "--timeout-seconds", str(canonical["timeout"]),
                   "--preview-seconds", str(preview), "--heading-gain", str(heading),
                   "--lateral-gain", str(lateral)]
        try:
            completed = subprocess.run(command, check=False, timeout=canonical["timeout"] + 30.)
            returncode, wrapper_timeout = completed.returncode, False
        except subprocess.TimeoutExpired:
            returncode, wrapper_timeout = 124, True
        new = sorted(set(args.output_root.glob("attempt_*")) - before)
        summary = {}
        if new and (new[-1] / "summary.json").exists():
            summary = json.loads((new[-1] / "summary.json").read_text())
        settings = summary.get("settings", {})
        matches = all(settings.get(key) == value for key, value in EXPECTED.items())
        row = {"candidate": index, "preview_seconds": preview, "heading_gain": heading,
               "lateral_gain": lateral, "returncode": returncode, "wrapper_timeout": wrapper_timeout,
               "attempt": str(new[-1]) if new else None, "status": summary.get("status"),
               "lap_completed": summary.get("lap_completed"),
               "lap_time_s": summary.get("lap_time"),
               "max_lateral_error_m": summary.get("maximum_lateral_error"),
               "path_tracking_valid": summary.get("path_tracking_valid"),
               "canonical_settings_recorded": matches,
               "elapsed_wall_seconds": summary.get("elapsed_wall_seconds")}
        rows.append(row)
        print(json.dumps(row, sort_keys=True), flush=True)
        (args.output_root / "steering_sweep.json").write_text(json.dumps(
            {"model": args.model, "candidates": rows,
             "selection_rule": "diagnostic only: completed lap, canonical settings recorded, "
                               "max lateral error under the existing 0.25 m gate; no gate waived"},
            indent=2, allow_nan=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
