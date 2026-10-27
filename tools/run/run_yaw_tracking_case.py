"""Run one identity-bound yaw tracking case from immutable inputs.

The JSON interface is intentionally strict: this front end validates the
case tuple and delegates one replay to the existing suite runner. Invoke via
run_with_deadline.py; its process group is the outer watchdog.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

STATE_FIELDS = {"lateral_error", "heading_error", "speed", "yaw_rate"}
LAG_FIELDS = {"steering", "pedal"}


def apply_state_overrides(initial_state, overrides, *, closed=True, minimum_speed=0.25):
    """Apply named state deltas; reject domain violations without clipping."""
    import numpy as np
    state = np.asarray(initial_state, dtype=float).copy()
    indices = {"lateral_error": 1, "heading_error": 2, "speed": 3, "yaw_rate": 5}
    if state.ndim != 1 or state.size < 6 or not np.all(np.isfinite(state)):
        raise ValueError("initial plant state must be a finite vector with published state layout")
    for key, delta in overrides.items():
        if key not in indices:
            raise ValueError(f"unsupported initial-state field: {key}")
        state[indices[key]] += float(delta)
    if state[3] < minimum_speed or (closed and state[3] <= 1.0):
        raise ValueError("perturbed initial speed is inadmissible")
    return state


def synthetic_path_fixtures(samples=401):
    """Return analytic S-bend, cyclic seam, and width-pinch fixture arrays."""
    import numpy as np
    if samples < 9:
        raise ValueError("at least nine samples are required")
    s = np.linspace(0.0, 100.0, samples, endpoint=False)
    # Smooth curvature reversal, with a closed-form curvature-rate signal.
    k = 0.018 * np.sin(2.0 * np.pi * s / 100.0)
    kp = 0.018 * (2.0 * np.pi / 100.0) * np.cos(2.0 * np.pi * s / 100.0)
    width = 5.0 - 1.5 * np.exp(-0.5 * ((s - 50.0) / 3.0) ** 2)
    seam = int(np.argmax(np.abs(kp)))
    return {"distance": s, "curvature": k, "curvature_rate": kp,
            "half_width": width, "seam_index": seam}


def validate_case(case):
    if not isinstance(case, dict):
        raise ValueError("case must be an object")
    state = case.get("initial_state_override", {})
    controller = case.get("controller", {})
    variation = case.get("plant_variation", {})
    if not isinstance(state, dict) or set(state) - STATE_FIELDS:
        raise ValueError("initial-state override has unknown fields")
    for name, value in state.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"initial-state override {name} must be finite")
    if not isinstance(controller, dict) or set(controller) - {"preview_seconds", "yaw_damping", "heading_gain", "lateral_gain"}:
        raise ValueError("invalid resolved controller dictionary")
    for name, value in controller.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"controller {name} must be finite")
    if not isinstance(variation, dict) or set(variation) - LAG_FIELDS:
        raise ValueError("plant variation may only declare actuator lag multipliers")
    for name, multiplier in variation.items():
        if isinstance(multiplier, bool) or not isinstance(multiplier, (int, float)) or not math.isfinite(multiplier) or multiplier <= 0:
            raise ValueError(f"{name} lag multiplier must be finite and positive")
    return case


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--case", type=Path, required=True)
    args = p.parse_args(argv)
    case = validate_case(json.loads(args.case.read_text()))
    # Until case schema binds an immutable plan digest to a supported map/track
    # identity, no physical replay is launched on a guessed baseline. The
    # runner deliberately launches no partial case that could be mistaken for a
    # completed measurement.
    if not case.get("plan") or not case.get("plan_sha256"):
        p.error("case requires immutable plan path and SHA-256 binding")
    import hashlib
    plan = Path(case["plan"])
    if not plan.is_file() or hashlib.sha256(plan.read_bytes()).hexdigest() != case["plan_sha256"]:
        p.error("immutable plan digest mismatch")
    p.error("physical replay unavailable: this interface does not yet bind plan source provenance, map, and track identity")


if __name__ == "__main__":
    from tools.run.run_with_deadline import run_with_deadline
    import os, sys
    if os.environ.get("YAW_PROCESS_WATCHDOG_ACTIVE") == "1":
        raise SystemExit(main())
    raise SystemExit(run_with_deadline([sys.executable, __file__, *sys.argv[1:]], seconds=1500))
