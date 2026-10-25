"""Pure admission evidence for the exact stored fitted geometry.

This narrow API assesses one stored candidate trajectory, when supplied. It does
not assess corridor-wide potential, vehicle footprint clearance, or feasibility.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
import math
from numbers import Real

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.reference_geometry_bounds import bound_reference_geometry
from simplified_vehicle_models.solvers.offset_geometry_bounds import (
    HeadingStatus, LimitStatus, bound_offset_geometry,
)


class AdmissionStatus(str, Enum):
    INVALID = "invalid"
    UNSUPPORTED = "unsupported"
    UNRESOLVED = "unresolved"
    ADMITTED = "admitted"


class EvidenceStatus(str, Enum):
    NOT_REQUESTED = "not_requested"
    ESTABLISHED = "established"
    UNSUPPORTED = "unsupported"
    UNRESOLVED = "unresolved"


@dataclass(frozen=True, slots=True)
class AdmissionSettings:
    fold_guard: Fraction | float
    maximum_heading: Fraction | float
    maximum_curvature_rate: Fraction | float

    def __post_init__(self):
        for name in ("fold_guard", "maximum_heading", "maximum_curvature_rate"):
            value = getattr(self, name)
            if type(value) is Fraction:
                q = value
            elif type(value) is float and math.isfinite(value):
                q = Fraction.from_float(value)
            else:
                raise TypeError(f"{name} must be an exact Fraction or built-in finite float")
            if q <= 0:
                raise ValueError(f"{name} must be positive")
            if name == "fold_guard" and q >= 1:
                raise ValueError("fold_guard must be less than one")
            object.__setattr__(self, name, q)


@dataclass(frozen=True, slots=True)
class OperationBudget:
    maximum_panels: int = 4096
    maximum_depth: int = 24
    sqrt_bits: int = 80
    atan_work_cap: int = 200000
    atan_terms: int = 160
    atan_bit_cap: int = 32768
    atan_reduction_steps: int = 8

    def __post_init__(self):
        bounds = {"maximum_panels": (1, 65536), "maximum_depth": (0, 64),
                  "sqrt_bits": (16, 256), "atan_work_cap": (1, 1000000),
                  "atan_terms": (1, 1024), "atan_bit_cap": (64, 65536),
                  "atan_reduction_steps": (0, 16)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} must be an integer from {low} through {high}")


@dataclass(frozen=True, slots=True)
class Evidence:
    status: EvidenceStatus
    reason: str | None = None
    metric_interval: tuple[Fraction, Fraction] | None = None
    fold_interval: tuple[Fraction, Fraction] | None = None
    heading_interval: tuple[Fraction, Fraction] | None = None
    rate_interval: tuple[Fraction, Fraction] | None = None
    panel_count: int = 0


@dataclass(frozen=True, slots=True)
class GeometryAdmission:
    status: AdmissionStatus
    reason_codes: tuple[str, ...]
    reference: Evidence
    candidate: Evidence | None
    settings: AdmissionSettings
    domain: tuple[float, float]
    operation_budget: OperationBudget
    claim: str = "stored_fit_geometry_only"
    physical_feasibility_certified: bool = False
    sampled_checks_not_continuous_certificate: bool = True


def _bound_args(b: OperationBudget):
    return dict(maximum_panels=b.maximum_panels, maximum_depth=b.maximum_depth, sqrt_bits=b.sqrt_bits)


def offset_basis_domain_supported(reference: ReferenceCurve, spline: OffsetSpline) -> bool:
    """Check source/basis alignment without evaluating, mutating, or extrapolating."""
    if type(reference) is not ReferenceCurve or type(spline) is not OffsetSpline:
        raise TypeError("exact ReferenceCurve and OffsetSpline required")
    knots = spline.control_distance
    if knots.size < 2:
        return False
    return float(knots[0]) == reference.start and float(knots[-1]) == reference.end


def admit_loaded_geometry(*, reference, trajectory_or_none, settings, domain, operation_budget,
                          offset_spline_or_none=None):
    """Admit only when every exact stored-fit enclosure establishes its policy bound."""
    if type(settings) is not AdmissionSettings:
        raise TypeError("settings must be exact AdmissionSettings")
    if type(operation_budget) is not OperationBudget:
        raise TypeError("operation_budget must be exact OperationBudget")
    if type(reference) is not ReferenceCurve:
        raise TypeError("reference must be exact ReferenceCurve")
    if trajectory_or_none is not None and type(trajectory_or_none) is not OffsetTrajectory:
        raise TypeError("trajectory must be exact OffsetTrajectory or None")
    if offset_spline_or_none is not None:
        if type(offset_spline_or_none) is not OffsetSpline:
            raise TypeError("offset_spline_or_none must be exact OffsetSpline or None")
        if trajectory_or_none is not None:
            raise ValueError("preconstruction spline and trajectory are mutually exclusive")
    if not isinstance(domain, (tuple, list)) or len(domain) != 2:
        raise TypeError("domain must be a station pair")
    vals=[]
    for v in domain:
        if isinstance(v, bool) or not isinstance(v, Real):
            raise TypeError("domain endpoints must be real non-bool values")
        f=float(v)
        if not math.isfinite(f) or Fraction.from_float(f) != Fraction(v):
            raise ValueError("domain endpoints must be finite exact binary64")
        vals.append(f)
    lo, hi = vals
    if lo > hi or lo < reference.start or hi > reference.end:
        raise ValueError("domain must be ordered within reference domain")
    domain=(lo,hi)
    if trajectory_or_none is not None:
        t=trajectory_or_none
        if t.reference is not reference:
            raise ValueError("candidate must reference the supplied ReferenceCurve instance")
        if type(t.spline) is not OffsetSpline:
            raise TypeError("candidate spline must be exact OffsetSpline")
    admission_spline = trajectory_or_none.spline if trajectory_or_none is not None else offset_spline_or_none
    if admission_spline is not None and not offset_basis_domain_supported(reference, admission_spline):
        return GeometryAdmission(AdmissionStatus.UNSUPPORTED, ("unsupported_offset_basis_domain",),
            Evidence(EvidenceStatus.NOT_REQUESTED), Evidence(EvidenceStatus.UNSUPPORTED, "unsupported_offset_basis_domain"), settings, domain, operation_budget)
    try:
        rb=bound_reference_geometry(reference, domain, **_bound_args(operation_budget))
    except (RuntimeError, ValueError) as exc:
        ev=Evidence(EvidenceStatus.UNRESOLVED, str(exc))
        return GeometryAdmission(AdmissionStatus.UNRESOLVED, ("unresolved_reference_bound",), ev, None, settings, domain, operation_budget)
    re=Evidence(EvidenceStatus.ESTABLISHED if rb.metric_interval[0] > 0 else EvidenceStatus.UNRESOLVED,
                None if rb.metric_interval[0] > 0 else "reference_metric_not_proven_positive", rb.metric_interval,
                panel_count=rb.panel_count)
    if re.status is not EvidenceStatus.ESTABLISHED:
        return GeometryAdmission(AdmissionStatus.UNRESOLVED, ("unresolved_reference_metric",), re, None, settings, domain, operation_budget)
    if trajectory_or_none is None:
        return GeometryAdmission(AdmissionStatus.ADMITTED, (), re, None, settings, domain, operation_budget)
    b=operation_budget
    try:
        cb=bound_offset_geometry(trajectory_or_none, domain, **_bound_args(b),
            heading_fold_guard=settings.fold_guard, maximum_heading=settings.maximum_heading,
            maximum_heading_rate=settings.maximum_curvature_rate, atan_work_cap=b.atan_work_cap,
            atan_terms=b.atan_terms, atan_bit_cap=b.atan_bit_cap, atan_reduction_steps=b.atan_reduction_steps)
    except (RuntimeError, ValueError) as exc:
        ce=Evidence(EvidenceStatus.UNRESOLVED, str(exc))
        return GeometryAdmission(AdmissionStatus.UNRESOLVED, ("unresolved_candidate_bound",), re, ce, settings, domain, operation_budget)
    reason=None
    if cb.heading_status is HeadingStatus.GUARD_VIOLATION:
        reason="unresolved_candidate_fold_guard_violation"
    elif cb.heading_status is not HeadingStatus.ESTABLISHED:
        reason="unresolved_heading_enclosure"
    elif cb.heading_limit_status is LimitStatus.VIOLATION or cb.heading_rate_limit_status is LimitStatus.VIOLATION:
        reason="unresolved_candidate_heading_or_rate_limit_violation"
    elif cb.heading_limit_status is not LimitStatus.WITHIN_LIMIT or cb.heading_rate_limit_status is not LimitStatus.WITHIN_LIMIT:
        reason="unresolved_heading_or_rate_limit"
    elif cb.fold_interval[0] < settings.fold_guard:
        reason="unresolved_fold_guard"
    elif cb.metric_interval[0] <= 0:
        reason="unresolved_offset_metric"
    status=AdmissionStatus.ADMITTED if reason is None else AdmissionStatus.UNRESOLVED
    ce=Evidence(EvidenceStatus.ESTABLISHED if status is AdmissionStatus.ADMITTED else EvidenceStatus.UNRESOLVED, reason,
        cb.metric_interval, cb.fold_interval, cb.heading_error_interval, cb.heading_error_s_interval, cb.panel_count)
    return GeometryAdmission(status, (() if reason is None else (reason,)), re, ce, settings, domain, operation_budget)
