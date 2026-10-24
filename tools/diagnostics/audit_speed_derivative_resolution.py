"""Analytic evidence study for the independent speed derivative heuristic.

This characterizes (but does not certify) the existing production estimator.
Exact derivatives are supplied by the analytic fixture definitions, never by
another finite-difference estimator.
"""
from __future__ import annotations

import hashlib
import json
import math
import platform
from pathlib import Path
from typing import Callable

import numpy as np

from simplified_vehicle_models.solvers.line_validation import _richardson_derivative


FUNCTIONS = {
    "constant": {"definition": "v(s)=20", "value": lambda s: 20.0, "derivative": lambda s: 0.0},
    "affine": {"definition": "v(s)=20+0.1s", "value": lambda s: 20.0 + 0.1*s, "derivative": lambda s: 0.1},
    "cubic_energy_sqrt": {"definition": "v(s)=sqrt(100+4s+0.3s^3)", "value": lambda s: math.sqrt(100+4*s+0.3*s**3), "derivative": lambda s: (4+0.9*s*s)/(2*math.sqrt(100+4*s+0.3*s**3))},
    "smooth_sine": {"definition": "v(s)=20+0.1sin(s)", "value": lambda s: 20+0.1*math.sin(s), "derivative": lambda s: 0.1*math.cos(s)},
    "rapid_sine": {"definition": "v(s)=20+0.1sin(100s)", "value": lambda s: 20+0.1*math.sin(100*s), "derivative": lambda s: 10*math.cos(100*s)},
}


def _row(function: str, values: Callable[[float], float], exact: float, station: float,
         step: float, side: str | None, *, case: str | None = None, exact_source="analytic_definition",
         speed=None, metric_G=None, physical_control=None, reported_rate=None, domain=(-math.inf, math.inf)) -> dict:
    side_arg = None if side == "central" else side
    queried = []
    def recording_values(s):
        queried.append(float(s))
        return values(s)
    try:
        # Record the estimator's actual sample locations rather than reimplementing
        # its stencil here; this keeps the diagnostic coupled to the characterized code.
        estimate, uncertainty, stable = _richardson_derivative(recording_values, float(station), float(step), side_arg)
        coords = sorted(set(queried))
        expected_distinct = 7 if side == "central" else 5
        representable = (all(math.isfinite(x) for x in coords)
                         and len(coords) == expected_distinct)
        in_domain = all(domain[0] <= x <= domain[1] for x in coords)
        if not in_domain:
            raise ValueError("stencil outside declared domain")
        resolved = bool(math.isfinite(estimate) and math.isfinite(uncertainty) and representable)
    except (ValueError, OverflowError, ZeroDivisionError, TypeError, FloatingPointError):
        coords = sorted(set(queried))
        expected_distinct = 7 if side == "central" else 5
        representable = (all(math.isfinite(x) for x in coords)
                         and len(coords) == expected_distinct)
        in_domain = all(domain[0] <= x <= domain[1] for x in coords)
        estimate, uncertainty, stable = math.nan, math.inf, False
    resolved = bool(math.isfinite(estimate) and math.isfinite(uncertainty)
                    and representable and in_domain)
    actual_error = abs(estimate-exact) if math.isfinite(estimate) and math.isfinite(exact) else math.inf
    row = {"case": case or function, "function": function, "definition": FUNCTIONS.get(function, {}).get("definition", function),
           "station": float(station), "sample_coordinates": coords, "side": side, "step": float(step),
           "exact": float(exact) if math.isfinite(exact) else None, "exact_source": exact_source,
           "estimate": float(estimate) if math.isfinite(estimate) else None,
           "absolute_error": float(actual_error) if math.isfinite(actual_error) else None,
           "uncertainty": float(uncertainty) if math.isfinite(uncertainty) else None,
           "stable": bool(stable), "resolved": resolved, "representable": representable,
           "domain_compatible": bool(in_domain), "false_confidence": bool(stable and actual_error > uncertainty),
           "excess_conservatism_ratio": float(uncertainty/actual_error) if actual_error > 0 and math.isfinite(uncertainty) else None,
           "reported_rate": reported_rate}
    if metric_G is not None:
        physical_speed = float(values(station)) if speed is None else float(speed)
        row.update(speed=physical_speed, metric_G=float(metric_G),
                   propagated_uncertainty=float(abs(physical_speed/metric_G)*uncertainty) if math.isfinite(uncertainty) else None,
                   physical_implied_acceleration=float(physical_speed*estimate/metric_G) if math.isfinite(estimate) else None,
                   physical_implied_acceleration_residual=(float(abs(physical_speed*estimate/metric_G-physical_control))
                       if math.isfinite(estimate) and physical_control is not None else None),
                   exact_physical_residual=(float(abs(physical_speed*exact/metric_G-physical_control))
                       if math.isfinite(exact) and physical_control is not None else None),
                   physical_control=float(physical_control) if physical_control is not None else None)
    return row


def run_analytic_study() -> dict:
    """Return deterministic analytic truth/error evidence (never acceptance)."""
    rows = []
    station = 0.123
    for name, fixture in FUNCTIONS.items():
        for side in ("central", "left", "right"):
            for step in (0.1, 0.01, 0.001, 0.0001):
                rows.append(_row(name, fixture["value"], fixture["derivative"](station), station, step, side))
    # Quantization/cancellation contrasted at two speed scales.
    for offset in (1e8, 1e12):
        f = lambda s, o=offset: o + .25*s
        rows.append(_row(f"cancellation_{offset:.0e}", f, .25, 1.0, .001, "central", case="cancellation"))
    # Coordinates at 2**53 collapse for sub-ULP steps; retain the analytic truth.
    S = float(2**53)
    rows.append(_row("translated_affine", lambda s: 20+(s-S), 1.0, S, .001, "central", case="large_station_collapse"))
    # Tiny mesh cell sided stencils: step obeys the left cell span.
    f = lambda s: math.sqrt(100+4*s)
    exact = 2/math.sqrt(100+4e-12)
    for side, h in (("left", .49e-12), ("right", .001)):
        rows.append(_row("tiny_cell_sqrt_energy", f, exact, 1e-12, h, side, case="tiny_cell"))
    # A true derivative jump: evaluate each branch only on its own side.
    def jump(s): return 10+(.2 if s <= 1 else .8)*(s-1)
    rows.extend((_row("knot_jump", jump, .2, 1., .01, "left", case="knot_jump"),
                 _row("knot_jump", jump, .8, 1., .01, "right", case="knot_jump")))
    # Reported rate is intentionally false; derivative evidence samples speed only.
    lie = _row("lying_rate", FUNCTIONS["affine"]["value"], .1, .123, .001, "central", case="lying_rate", reported_rate=-99., metric_G=2., physical_control=.25)
    rows.append(lie)
    rows.append(_row("nan_speed", lambda s: math.nan if s > .5 else 1., math.nan, .5, .01, "right", case="nonfinite"))
    rows.append(_row("sqrt_domain", math.sqrt, math.inf, 0., .01, "central", case="incompatible_domain", domain=(0., math.inf)))
    return {"schema_version": 1, "acceptance_authority": False,
            "method": "characterization of production _richardson_derivative; analytic derivatives are oracle",
            "method_decision": "retain production estimator unchanged; evidence is heuristic, not a rigorous bound",
            "rows": rows}


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def machine_envelope() -> dict:
    source = Path(__file__).resolve()
    production = Path(__import__("simplified_vehicle_models.solvers.line_validation", fromlist=["__file"]).__file__).resolve()
    return {"study": run_analytic_study(), "runtime": {"python": platform.python_version(), "numpy": np.__version__},
            "code_sha256": {"diagnostic": _sha(source), "line_validation": _sha(production)}}


if __name__ == "__main__":
    print(json.dumps(machine_envelope(), sort_keys=True, allow_nan=False, indent=2))
