"""Analytic evidence study for the independent speed derivative heuristic.

This characterizes (but does not certify) the existing production estimator.
Exact derivatives are supplied by analytic fixture definitions, never by
another finite-difference estimator. No result has acceptance authority.
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
    "constant": {"definition": "v(s)=20", "domain": ["-inf", "inf"], "value": lambda s: 20.0, "derivative": lambda s: 0.0},
    "affine": {"definition": "v(s)=20+0.1s", "domain": ["-inf", "inf"], "value": lambda s: 20.0 + 0.1*s, "derivative": lambda s: 0.1},
    "cubic_energy_sqrt": {"definition": "v(s)=sqrt(100+4s+0.3s^3), domain where radicand >=0", "domain": ["radicand >= 0", "radicand >= 0"], "value": lambda s: math.sqrt(100+4*s+0.3*s**3), "derivative": lambda s: (4+0.9*s*s)/(2*math.sqrt(100+4*s+0.3*s**3))},
    "smooth_sine": {"definition": "v(s)=20+0.1sin(s)", "domain": ["-inf", "inf"], "value": lambda s: 20+0.1*math.sin(s), "derivative": lambda s: 0.1*math.cos(s)},
    "rapid_sine": {"definition": "v(s)=20+0.1sin(100s)", "domain": ["-inf", "inf"], "value": lambda s: 20+0.1*math.sin(100*s), "derivative": lambda s: 10*math.cos(100*s)},
}


def _row(function: str, values: Callable[[float], float], exact: float, station: float,
         step: float, side: str | None, *, case: str | None = None,
         exact_source="analytic_definition", speed=None, metric_G=None,
         physical_control=None, reported_rate=None,
         domain=(-math.inf, math.inf), domain_description="all real stations", definition=None) -> dict:
    """Observe production calls/components; reject illegal samples before evaluation."""
    side_arg = None if side == "central" else side
    queried: list[float] = []
    samples: dict[float, float] = {}
    failure_reason = None

    def recording_values(s):
        nonlocal failure_reason
        s = float(s)
        queried.append(s)
        if not domain[0] <= s <= domain[1]:
            failure_reason = f"out_of_domain_sample:{s!r}"
            raise ValueError(failure_reason)
        value = float(values(s))
        samples[s] = value
        if not math.isfinite(value):
            failure_reason = f"nonfinite_speed_sample:{s!r}"
            raise FloatingPointError(failure_reason)
        return value

    try:
        estimate, uncertainty, stable = _richardson_derivative(
            recording_values, float(station), float(step), side_arg)
        coords = sorted(set(queried))
        expected_distinct = 7 if side == "central" else 5
        representable = all(math.isfinite(x) for x in coords) and len(coords) == expected_distinct
        in_domain = (all(domain[0] <= x <= domain[1] for x in coords)
                     and not (failure_reason or "").startswith("out_of_domain_sample:"))
    except (ValueError, OverflowError, ZeroDivisionError, TypeError, FloatingPointError) as error:
        coords = sorted(set(queried))
        expected_distinct = 7 if side == "central" else 5
        representable = all(math.isfinite(x) for x in coords) and len(coords) == expected_distinct
        in_domain = (all(domain[0] <= x <= domain[1] for x in coords)
                     and not (failure_reason or "").startswith("out_of_domain_sample:"))
        estimate, uncertainty, stable = math.nan, math.inf, False
        if failure_reason is None:
            failure_reason = str(error)
    resolved = bool(math.isfinite(estimate) and math.isfinite(uncertainty)
                    and representable and in_domain)
    actual_error = abs(estimate-exact) if math.isfinite(estimate) and math.isfinite(exact) else math.inf

    components = None
    if len(set(queried)) == expected_distinct and all(x in samples for x in queried):
        h = float(step)
        if side == "central":
            d0 = (samples[station+h]-samples[station-h])/(2*h)
            d1 = (samples[station+h/2]-samples[station-h/2])/h
            d2 = (samples[station+h/4]-samples[station-h/4])/(h/2)
        elif side == "right":
            d0 = (-3*samples[station]+4*samples[station+h]-samples[station+2*h])/(2*h)
            d1 = (-3*samples[station]+4*samples[station+h/2]-samples[station+h])/h
            d2 = (-3*samples[station]+4*samples[station+h/4]-samples[station+h/2])/(h/2)
        else:
            d0 = (3*samples[station]-4*samples[station-h]+samples[station-2*h])/(2*h)
            d1 = (3*samples[station]-4*samples[station-h/2]+samples[station-h])/h
            d2 = (3*samples[station]-4*samples[station-h/4]+samples[station-h/2])/(h/2)
        richardson0, richardson1 = (4*d1-d0)/3, (4*d2-d1)/3
        raw0, raw1 = abs(d1-d0)/3, abs(d2-d1)/3
        level_delta = abs(richardson1-richardson0)
        roundoff = 64*np.finfo(float).eps*max(1.0, abs(samples[station]))/(h/4)
        divisor = 15.0 if side == "central" else 7.0
        components = {
            "d0": d0, "d1": d1, "d2": d2,
            "raw_correction_d0_d1_over3": raw0,
            "raw_correction_d1_d2_over3": raw1,
            "richardson_level0": richardson0,
            "richardson_level1": richardson1,
            "extrapolated_levels_absolute_delta": level_delta,
            "extrapolated_level_remainder_indicator": level_delta/divisor,
            "roundoff_floor": roundoff,
            "aggregate_uncertainty_reconstructed": max(raw1, level_delta/divisor, roundoff),
            "absolute_error_minus_components": {
                "raw_correction_d0_d1_over3": actual_error-raw0,
                "raw_correction_d1_d2_over3": actual_error-raw1,
                "extrapolated_level_remainder_indicator": actual_error-level_delta/divisor,
                "roundoff_floor": actual_error-roundoff},
            "interpretation": "observed production-formula components vs analytic error; indicators are not rigorous bounds"}

    row = {
        "case": case or function, "function": function,
        "definition": definition or FUNCTIONS.get(function, {}).get("definition", function),
        "domain": domain_description, "station": float(station), "sample_coordinates": coords,
        "side": side, "step": float(step), "exact": float(exact) if math.isfinite(exact) else None,
        "exact_source": exact_source, "estimate": float(estimate) if math.isfinite(estimate) else None,
        "absolute_error": float(actual_error) if math.isfinite(actual_error) else None,
        "uncertainty": float(uncertainty) if math.isfinite(uncertainty) else None,
        "error_budget_components": components, "failure_reason": failure_reason,
        "stable": bool(stable), "resolved": resolved, "representable": representable,
        "domain_compatible": bool(in_domain),
        "false_confidence": bool(stable and actual_error > uncertainty),
        "excess_conservatism_ratio": float(uncertainty/actual_error) if actual_error > 0 and math.isfinite(uncertainty) else None,
        "reported_rate": reported_rate}
    if metric_G is not None:
        physical_speed = float(values(station)) if speed is None else float(speed)
        row.update(speed=physical_speed, metric_G=float(metric_G),
                   propagated_uncertainty=float(abs(physical_speed/metric_G)*uncertainty) if math.isfinite(uncertainty) else None,
                   physical_implied_acceleration=float(physical_speed*estimate/metric_G) if math.isfinite(estimate) else None,
                   physical_implied_acceleration_residual=(float(abs(physical_speed*estimate/metric_G-physical_control)) if math.isfinite(estimate) and physical_control is not None else None),
                   exact_physical_residual=(float(abs(physical_speed*exact/metric_G-physical_control)) if math.isfinite(exact) and physical_control is not None else None),
                   physical_control=float(physical_control) if physical_control is not None else None)
    return row


def run_analytic_study() -> dict:
    """Return deterministic analytic truth/error evidence (never acceptance)."""
    rows = []
    station = 0.123
    for name, fixture in FUNCTIONS.items():
        for side in ("central", "left", "right"):
            for step in (0.1, 0.01, 0.001, 0.0001):
                valid_domain = (0.0, 1.0) if name == "cubic_energy_sqrt" else (-math.inf, math.inf)
                domain_text = "0<=s<=1 (radicand positive)" if name == "cubic_energy_sqrt" else "all real stations"
                rows.append(_row(name, fixture["value"], fixture["derivative"](station), station, step, side,
                                 domain=valid_domain, domain_description=domain_text))
    for offset in (1e8, 1e12):
        f = lambda s, o=offset: o + .25*s
        rows.append(_row(f"cancellation_{offset:.0e}", f, .25, 1.0, .001, "central", case="cancellation",
                         definition=f"v(s)={offset:.0e}+0.25s"))
    S = float(2**53)
    rows.append(_row("translated_affine", lambda s: 20+(s-S), 1.0, S, .001, "central", case="large_station_collapse",
                     definition=f"v(s)=20+(s-S), S=2**53={S:.0f}"))
    f = lambda s: math.sqrt(100+4*s)
    exact = 2/math.sqrt(100+4e-12)
    for side, h in (("left", .49e-12), ("right", .001)):
        rows.append(_row("tiny_cell_sqrt_energy", f, exact, 1e-12, h, side, case="tiny_cell",
                         definition="v(s)=sqrt(100+4s), mesh cells [0,1e-12] and [1e-12,1]",
                         domain=(0.0, 1e-12) if side == "left" else (1e-12, 1.0),
                         domain_description="[0,1e-12] left cell" if side == "left" else "[1e-12,1] right cell"))
    def jump(s):
        return 10+(.2 if s <= 1 else .8)*(s-1)
    jump_definition = "v(s)=10+0.2(s-1) for s<=1; 10+0.8(s-1) for s>1"
    rows.extend((_row("knot_jump", jump, .2, 1., .01, "left", case="knot_jump",
                      definition=jump_definition, domain=(0.0, 1.0), domain_description="left branch 0<=s<=1"),
                 _row("knot_jump", jump, .8, 1., .01, "right", case="knot_jump",
                      definition=jump_definition, domain=(1.0, math.inf), domain_description="right branch s>=1")))
    affine_speed = FUNCTIONS["affine"]["value"](station)
    good_acceleration = affine_speed*.1/2
    rows.append(_row("affine_good_control_lie", FUNCTIONS["affine"]["value"], .1, station, .001, "central",
                     case="lying_rate_good_control", definition="v(s)=20+0.1s; reported_rate=-99; a_control=v*0.1/2",
                     reported_rate=-99., metric_G=2., physical_control=good_acceleration))
    rows.append(_row("affine_bad_control_lie", FUNCTIONS["affine"]["value"], .1, station, .001, "central",
                     case="lying_rate_bad_control", definition="v(s)=20+0.1s; reported_rate=-99; a_control=0.25",
                     reported_rate=-99., metric_G=2., physical_control=.25))
    rows.append(_row("nan_speed", lambda s: math.nan if s > .5 else 1., math.nan, .5, .01, "right",
                     case="nonfinite", definition="v(s)=1 for s<=0.5; NaN for s>0.5",
                     domain=(0.0, 1.0), domain_description="0<=s<=1; speed is NaN above 0.5"))
    rows.append(_row("sqrt_domain", math.sqrt, math.inf, 0., .01, "central", case="incompatible_domain",
                     definition="v(s)=sqrt(s), s>=0; derivative singular at s=0",
                     domain=(0.0, math.inf), domain_description="sqrt(s), s>=0; derivative singular at 0"))
    return {"schema_version": 1, "acceptance_authority": False,
            "method": "characterization of production _richardson_derivative; observational components reconstruct its formula from captured samples; analytic derivatives are oracle",
            "method_decision": "retain production estimator unchanged; evidence is heuristic, not a rigorous bound",
            "function_definitions": {name: {"definition": entry["definition"], "domain": entry["domain"]} for name, entry in FUNCTIONS.items()},
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
