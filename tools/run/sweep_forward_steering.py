#!/usr/bin/env python3
"""Run one bounded steering candidate and stop after its first lap.

A candidate is a complete invocation of the isolated runner. The wrapper uses a
hard parent timeout and the runner's canonical path-end event. It stops on the
first invocation because accepted evidence requires review before proceeding.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("planar-7dof", "14dof-simple"), default="planar-7dof")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=300.)
    parser.add_argument("--preview", type=float, nargs="+", default=[.15])
    parser.add_argument("--heading-gain", type=float, nargs="+", default=[1.5])
    parser.add_argument("--lateral-gain", type=float, nargs="+", default=[1.5])
    parser.add_argument("--planner", choices=("conservative", "live"), default="live")
    parser.add_argument("--maximum-speed", type=float, default=8.)
    parser.add_argument("--lateral-acceleration", type=float, default=None)
    args = parser.parse_args(argv)
    configs = [(p, h, l) for p in args.preview for h in args.heading_gain for l in args.lateral_gain]
    if len(configs) != 1:
        parser.error("exactly one candidate is allowed per invocation; review its outcome before launching another")
    preview, heading, lateral = configs[0]
    args.output_root.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, "tools/run/run_iem24_forward_driver.py", "--model", args.model,
               "--output-root", str(args.output_root), "--timeout-seconds", str(args.timeout_seconds),
               "--planner", args.planner, "--maximum-speed", str(args.maximum_speed),
               "--preview-seconds", str(preview), "--heading-gain", str(heading),
               "--lateral-gain", str(lateral)]
    if args.lateral_acceleration is not None:
        command.extend(["--lateral-acceleration", str(args.lateral_acceleration)])
    try:
        completed = subprocess.run(command, check=False, timeout=args.timeout_seconds + 5.)
        returncode = completed.returncode
        wrapper_timeout = False
    except subprocess.TimeoutExpired:
        returncode = 124
        wrapper_timeout = True
    attempts = sorted(args.output_root.glob("attempt_*/summary.json"), key=lambda p: p.stat().st_mtime)
    summary = json.loads(attempts[-1].read_text()) if attempts else {}
    settings = summary.get("settings", {})
    row = {"model": args.model, "preview_seconds": preview, "heading_gain": heading,
           "lateral_gain": lateral, "requested_planner": args.planner,
           "recorded_planner": settings.get("planner"), "returncode": returncode,
           "wrapper_timeout": wrapper_timeout,
           "attempt": str(attempts[-1].parent) if attempts else None,
           "status": summary.get("status"), "lap_completed": summary.get("lap_completed"),
           "max_lateral_error_m": summary.get("maximum_lateral_error"),
           "path_tracking_valid": summary.get("path_tracking_valid"), "usable": summary.get("usable")}
    result = {"candidate": row,
              "selection_rule": "diagnostic only; require completed lap, settings match, and existing 0.25 m path gate; no other gates waived"}
    output = args.output_root / "steering_candidate.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if not wrapper_timeout and summary.get("settings", {}).get("planner") == args.planner else 1


if __name__ == "__main__":
    raise SystemExit(main())
