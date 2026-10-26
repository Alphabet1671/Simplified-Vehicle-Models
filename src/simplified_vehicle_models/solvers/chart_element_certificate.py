"""Bound explicit rational energy/control polynomials against a stored signed chart.

This is a mathematical candidate check on explicit inputs, never a solver-state,
vehicle, event, or policy certificate. Binary64 chart arithmetic follows the
outward nextafter/RNE convention documented by signed_chart_bounds.
"""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math
import time
from numbers import Real

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.energy_polynomial_bounds import bound_energy_polynomials
from simplified_vehicle_models.solvers.offset_geometry_bounds import bound_offset_geometry
from simplified_vehicle_models.solvers.reference_geometry_bounds import _div_positive, _sqrt_interval
from simplified_vehicle_models.solvers.signed_chart_bounds import bound_signed_chart_rectangle
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory


@dataclass(frozen=True, slots=True)
class ChartElementSettings:
    maximum_panels: int = 128
    maximum_depth: int = 16
    maximum_geometry_panels: int = 256
    geometry_maximum_depth: int = 16
    sqrt_bits: int = 80
    maximum_wall_seconds: float = 30.0


def _ratio(q):
    return {"numerator": q.numerator, "denominator": q.denominator}


def _rat(v, name):
    if isinstance(v, bool) or not isinstance(v, Real):
        raise TypeError(f"{name} must be a finite real non-bool")
    if isinstance(v, Fraction):
        return v
    if isinstance(v, int):
        return Fraction(v)
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite")
    if v != f:
        raise ValueError(f"{name} must be losslessly representable as binary64")
    return Fraction.from_float(f)


def _float_lo(q):
    f = float(q)
    if not math.isfinite(f):
        raise OverflowError("rational interval exceeds binary64")
    if Fraction.from_float(f) > q:
        f = math.nextafter(f, -math.inf)
    if not math.isfinite(f):
        raise OverflowError("outward lower endpoint exceeds binary64")
    return f


def _float_hi(q):
    f = float(q)
    if not math.isfinite(f):
        raise OverflowError("rational interval exceeds binary64")
    if Fraction.from_float(f) < q:
        f = math.nextafter(f, math.inf)
    if not math.isfinite(f):
        raise OverflowError("outward upper endpoint exceeds binary64")
    return f


def _settings(s):
    if type(s) is not ChartElementSettings:
        raise TypeError("settings must be exactly ChartElementSettings")
    for n, v, low in (("maximum_panels", s.maximum_panels, 1), ("maximum_depth", s.maximum_depth, 0),
                      ("maximum_geometry_panels", s.maximum_geometry_panels, 1),
                      ("geometry_maximum_depth", s.geometry_maximum_depth, 0)):
        if isinstance(v, bool) or not isinstance(v, int) or v < low:
            raise ValueError(f"{n} must be an integer >= {low}")
    if isinstance(s.sqrt_bits, bool) or not isinstance(s.sqrt_bits, int) or not 16 <= s.sqrt_bits <= 256:
        raise ValueError("sqrt_bits must be an integer from 16 through 256")
    if isinstance(s.maximum_wall_seconds, bool) or not isinstance(s.maximum_wall_seconds, Real):
        raise TypeError("maximum_wall_seconds must be finite and positive")
    wall = float(s.maximum_wall_seconds)
    if not math.isfinite(wall) or wall <= 0:
        raise ValueError("maximum_wall_seconds must be finite and positive")
    return wall


def _base(status, reason, started, geom_count=0, rows=(), pending=0, evaluated=None, geometry_failures=()):
    return {"status": status, "chart_constraints_proven": status == "chart_constraints_proven",
            "reason": reason, "scope": "explicit_rational_polynomials_on_analytic_offset_path_and_stored_chart",
            "mathematical_semantics": "explicit_rational_Hermite_and_Lagrange_energy_v1",
            "finite_public_eval_not_certified": True, "accepted": False,
            "physical_feasibility_certified": False, "original_vehicle_validated": False,
            "event_qualified": False, "U3_qualified": False, "policy_approved": False,
            "panels_evaluated": len(rows) if evaluated is None else evaluated, "geometry_panels_evaluated": geom_count,
            "elapsed_wall_seconds": float(max(0., time.monotonic()-started)),
            "remaining_pending_panels": pending, "geometry_failures": list(geometry_failures),
            "panels": list(rows)}


def check_chart_element(trajectory, surface, source_interval, energy_endpoints, energy_rate_sites, *,
                        settings=None, speed_domain_margin_mps=0., minimum_fold=.2,
                        maximum_speed_rate_defect=1e-6) -> dict:
    """Sufficiently prove chart inequalities over a whole explicit source span.

    Failure to prove, including an interval that overlaps a constraint, is not
    an infeasibility claim. Fractions in JSON use exact numerator/denominator.
    """
    started = time.monotonic()
    st = ChartElementSettings() if settings is None else settings
    geom_count = 0
    try:
        wall = _settings(st)
        if type(trajectory) is not OffsetTrajectory or type(surface) is not SignedTransientCapability:
            raise TypeError("unsupported trajectory or chart class")
        if not isinstance(source_interval, (tuple, list)) or len(source_interval) != 2:
            raise ValueError("source_interval must contain two endpoints")
        a_input, b_input = source_interval
        if any(isinstance(x, bool) or not isinstance(x, Real) for x in (a_input, b_input)):
            raise TypeError("source endpoints must be finite real non-bool values")
        a, b = float(a_input), float(b_input)
        if not math.isfinite(a) or not math.isfinite(b) or not a < b:
            raise ValueError("source_interval must be finite and increasing")
        if Fraction.from_float(a) != Fraction(a_input) or Fraction.from_float(b) != Fraction(b_input):
            raise ValueError("source endpoints must be binary64 exact")
        if a < trajectory.reference.start or b > trajectory.reference.end:
            raise ValueError("source interval outside trajectory")
        # Freeze iterable inputs once; reject generators whose semantics could change.
        if not isinstance(energy_endpoints, (tuple, list)) or len(energy_endpoints) != 2:
            raise ValueError("energy_endpoints must contain two explicit values")
        if not isinstance(energy_rate_sites, (tuple, list)) or len(energy_rate_sites) != 3:
            raise ValueError("energy_rate_sites must contain three explicit values")
        energy_endpoints = tuple(energy_endpoints)
        energy_rate_sites = tuple(energy_rate_sites)
        # Validate the explicit energy specification before panel work.
        bound_energy_polynomials((a,b), energy_endpoints, energy_rate_sites)
        margin, fold_min, defect_max = (_rat(v,n) for v,n in (
            (speed_domain_margin_mps,"speed_domain_margin_mps"), (minimum_fold,"minimum_fold"),
            (maximum_speed_rate_defect,"maximum_speed_rate_defect")))
        if margin < 0 or fold_min < 0 or defect_max < 0:
            raise ValueError("margins and defect limit must be nonnegative")
    except Exception as exc:
        return _base("not_proven", f"input_error:{type(exc).__name__}:{exc}", started)

    pending = [(a,b,0)]
    rows = []
    evaluated = 0
    geometry_errors = []
    while pending and evaluated < st.maximum_panels:
        if time.monotonic()-started >= wall:
            return _base("not_proven", "wall_budget_exhausted", started, geom_count, rows, len(pending), evaluated, geometry_errors)
        lo, hi, depth = pending.pop(0)
        evaluated += 1
        reason = None
        try:
            if time.monotonic()-started >= wall:
                return _base("not_proven", "wall_budget_exhausted", started, geom_count, rows, len(pending)+1, evaluated, geometry_errors)
            e = bound_energy_polynomials((a,b), energy_endpoints, energy_rate_sites, subinterval=(lo,hi))
            g = bound_offset_geometry(trajectory, (lo,hi), maximum_panels=st.maximum_geometry_panels,
                                      maximum_depth=st.geometry_maximum_depth, sqrt_bits=st.sqrt_bits)
            geom_count += g.panel_count
            if time.monotonic()-started >= wall:
                return _base("not_proven", "wall_budget_exhausted", started, geom_count, rows, len(pending)+1, evaluated, geometry_errors)
            G, K = g.metric_interval, g.curvature_interval
            if G[0] <= 0 or g.fold_interval[0] < fold_min or e.energy_interval[0] <= 0:
                reason = "positive_metric_fold_or_energy_unresolved"
            else:
                sqrtw = _sqrt_interval(e.energy_interval, st.sqrt_bits)
                speed = sqrtw
                # Outward conversion must stay within stored chart support; never clip.
                kf = (_float_lo(K[0]), _float_hi(K[1]))
                if kf[0] < float(surface.curvature[0]) or kf[1] > float(surface.curvature[-1]):
                    reason = "curvature_outside_chart_support"
                else:
                    d = bound_signed_chart_rectangle(surface, (0.,1.), kf)
                    domain_lo = Fraction.from_float(d.domain_speed_lower)
                    domain_hi = Fraction.from_float(d.domain_speed_upper)
                    vmin, vmax = speed
                    if vmin < Fraction.from_float(surface.speed_floor) or vmax > domain_lo-margin:
                        reason = "speed_domain_unresolved"
                    else:
                        den = (domain_lo-Fraction.from_float(surface.speed_floor),
                               domain_hi-Fraction.from_float(surface.speed_floor))
                        u = _div_positive((vmin-Fraction.from_float(surface.speed_floor),
                                           vmax-Fraction.from_float(surface.speed_floor)), den)
                        if u[0] < 0 or u[1] > 1:
                            reason = "normalized_speed_support_unresolved"
                        else:
                            ui = (_float_lo(u[0]), _float_hi(u[1]))
                            if ui[0] < 0 or ui[1] > 1:
                                reason = "normalized_speed_outward_support_unresolved"
                            else:
                                c = bound_signed_chart_rectangle(surface, ui, kf)
                                acc = _div_positive(e.control_energy_rate_interval, (2*G[0],2*G[1]))
                                defect = max(abs(x) for x in e.energy_rate_defect_interval) / (2*sqrtw[0])
                                if (acc[0] < Fraction.from_float(c.acceleration_min_upper) or
                                    acc[1] > Fraction.from_float(c.acceleration_max_lower) or
                                    defect > defect_max):
                                    reason = "acceleration_or_speed_rate_defect_unresolved"
                                else:
                                    rows.append({"source_interval": [lo,hi], "energy": [_ratio(x) for x in e.energy_interval],
                                                 "metric": [_ratio(x) for x in G], "fold": [_ratio(x) for x in g.fold_interval],
                                                 "curvature": [_ratio(x) for x in K], "speed": [_ratio(x) for x in speed],
                                                 "normalized_speed": [_ratio(x) for x in u],
                                                 "acceleration": [_ratio(x) for x in acc],
                                                 "speed_rate_defect_abs_upper": _ratio(defect),
                                                 "chart_acceleration_min_upper": c.acceleration_min_upper,
                                                 "chart_acceleration_max_lower": c.acceleration_max_lower,
                                                 "domain_speed_lower": c.domain_speed_lower})
        except (ValueError, RuntimeError, OverflowError) as exc:
            geometry_errors.append(f"{type(exc).__name__}:{exc}")
            reason = "geometry_unresolved"
        if reason is not None:
            if depth >= st.maximum_depth:
                return _base("not_proven", reason, started, geom_count, rows, len(pending), evaluated, geometry_errors)
            mid = (Fraction.from_float(lo)+Fraction.from_float(hi))/2
            mf = float(mid)
            if not lo < mf < hi:
                return _base("not_proven", "source_refinement_resolution", started, geom_count, rows, len(pending), evaluated, geometry_errors)
            pending[0:0] = [(lo,mf,depth+1),(mf,hi,depth+1)]
    if pending:
        return _base("not_proven", "panel_budget_exhausted", started, geom_count, rows, len(pending), evaluated, geometry_errors)
    if time.monotonic()-started >= wall:
        return _base("not_proven", "wall_budget_exhausted", started, geom_count, rows, 0, evaluated, geometry_errors)
    return _base("chart_constraints_proven", "all_closed_panels_proven", started, geom_count, rows, 0, evaluated, geometry_errors)
