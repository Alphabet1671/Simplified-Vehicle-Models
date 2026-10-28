"""Resolution-limited discovery of yaw-map lateral-acceleration domains.

This module intentionally accepts an injected direct LP evaluator.  A scan samples
both signed rays completely before it refines *observed* feasible/infeasible
transitions; it does not infer that feasibility is monotone along a ray.  Thus a
reported component is a sampled component, and an island narrower than the scan
spacing can remain undetected.
"""
from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope


class UnsupportedLateralBoundError(TypeError):
    """Raised when a tyre model has no finite, model-derived lateral bound."""


class ProbeStatus(str, Enum):
    """Outcome of one direct LP query.

    ``UNRESOLVED`` covers solver numerical failure, timeout, or any outcome that
    cannot honestly be classified as physical infeasibility.
    """

    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    UNRESOLVED = "unresolved"


@dataclass(frozen=True, slots=True)
class ProbeResult:
    status: ProbeStatus
    detail: str = ""


@dataclass(frozen=True, slots=True)
class YawMapDomainSettings:
    """Immutable finite budgets and resolution declarations for discovery."""

    scan_spacing: float = 0.5
    bracket_tolerance: float = 0.05
    probe_budget: int = 10_000
    speed_ax_audit_subdivisions: int = 1
    lateral_interpolation_loss_tolerance: float = 0.1

    def __post_init__(self) -> None:
        for name in ("scan_spacing", "bracket_tolerance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.bracket_tolerance > self.scan_spacing:
            raise ValueError("bracket_tolerance must not exceed scan_spacing")
        if not isinstance(self.probe_budget, int) or isinstance(self.probe_budget, bool) or self.probe_budget <= 0:
            raise ValueError("probe_budget must be a positive integer")
        if (not isinstance(self.speed_ax_audit_subdivisions, int)
                or isinstance(self.speed_ax_audit_subdivisions, bool)
                or self.speed_ax_audit_subdivisions < 0):
            raise ValueError("speed_ax_audit_subdivisions must be a nonnegative integer")
        if (not math.isfinite(self.lateral_interpolation_loss_tolerance)
                or self.lateral_interpolation_loss_tolerance < 0.0):
            raise ValueError("lateral_interpolation_loss_tolerance must be finite and nonnegative")


@dataclass(frozen=True, slots=True)
class LateralForceBound:
    """A tyre-table-derived finite lateral acceleration enclosure."""

    lateral_acceleration: float
    peak_lateral_force: float
    tyre_load_table_saturates: bool


@dataclass(frozen=True, slots=True)
class TransitionBracket:
    """One locally observed feasible/infeasible transition bracket."""

    feasible_ay: float
    infeasible_ay: float

    @property
    def width(self) -> float:
        return abs(self.infeasible_ay - self.feasible_ay)


@dataclass(frozen=True, slots=True)
class SampledComponent:
    """A contiguous run of feasible scan samples on one signed ray.

    This is not a continuous feasibility proof: islands narrower than
    ``scan_spacing`` can be missed by the declared sampling resolution.
    """

    sign: int
    sampled_lower_ay: float
    sampled_upper_ay: float
    lower_transition: TransitionBracket | None
    upper_transition: TransitionBracket | None


@dataclass(frozen=True, slots=True)
class DomainSlice:
    speed: float
    longitudinal_acceleration: float
    components: tuple[SampledComponent, ...]
    unresolved_ay: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class DiscoveryDiagnostics:
    probe_count: int
    probe_budget: int
    budget_exhausted: bool
    unresolved_probes: tuple[tuple[float, float, float, str], ...]

    @property
    def complete(self) -> bool:
        return not self.budget_exhausted and not self.unresolved_probes


@dataclass(frozen=True, slots=True)
class YawMapDomain:
    """Discovered coverage and all resolution/completeness caveats."""

    lateral_bound: LateralForceBound
    requested_speed_knots: tuple[float, ...]
    requested_longitudinal_acceleration_knots: tuple[float, ...]
    audited_speeds: tuple[float, ...]
    audited_longitudinal_accelerations: tuple[float, ...]
    slices: tuple[DomainSlice, ...]
    diagnostics: DiscoveryDiagnostics
    scan_resolution_limited: bool = True


def unitire_peak_lateral_bound(vehicle) -> LateralForceBound:
    """Return ``4 * max(Fy_peak) / mass`` for a UniTire peak envelope.

    The envelope interpolation clamps loads above its final table knot to that
    knot's force.  ``tyre_load_table_saturates`` records that modelling limit;
    it is deliberately distinct from physical feasibility.
    """

    tire = vehicle.tire
    if not isinstance(tire, UniTirePeakEnvelope):
        raise UnsupportedLateralBoundError(
            "a finite yaw-domain lateral bound is only implemented for UniTirePeakEnvelope"
        )
    mass = float(vehicle.total_mass)
    peaks = np.asarray(tire.lateral_peak_force, dtype=float)
    loads = np.asarray(tire.vertical_loads, dtype=float)
    if (not math.isfinite(mass) or mass <= 0.0 or peaks.ndim != 1 or peaks.size == 0
            or loads.shape != peaks.shape or not np.all(np.isfinite(peaks))
            or not np.all(np.isfinite(loads)) or np.any(loads < 0.)
            or np.any(np.diff(loads) <= 0.) or np.any(peaks < 0.0)):
        raise ValueError("UniTirePeakEnvelope must contain a finite nonnegative lateral peak-force table")
    peak = float(np.max(peaks))
    bound = 4.0 * peak / mass
    if not math.isfinite(bound) or bound <= 0.0:
        raise ValueError("UniTirePeakEnvelope lateral peak-force bound must be finite and positive")
    return LateralForceBound(bound, peak, tyre_load_table_saturates=True)


def _knots(values: Iterable[float], name: str, *, nonnegative: bool = False) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be nonempty and finite")
    if nonnegative and any(value < 0.0 for value in result):
        raise ValueError(f"{name} must be nonnegative")
    if len(set(result)) != len(result):
        raise ValueError(f"{name} must not contain duplicates")
    return tuple(sorted(result))


def _audit_axis(knots: tuple[float, ...], subdivisions: int) -> tuple[float, ...]:
    points = set(knots)
    if subdivisions:
        for left, right in zip(knots, knots[1:]):
            for index in range(1, subdivisions + 1):
                points.add(left + (right - left) * index / (subdivisions + 1))
    return tuple(sorted(points))


def discover_yaw_map_domain(
    vehicle,
    speed_knots: Iterable[float],
    longitudinal_acceleration_knots: Iterable[float],
    evaluator: Callable[[float, float, float], ProbeResult],
    settings: YawMapDomainSettings = YawMapDomainSettings(),
) -> YawMapDomain:
    """Discover resolution-limited signed support using an injected LP evaluator.

    ``evaluator`` must return a :class:`ProbeResult` after checking both yaw LP
    objective directions for zero containment.  Calling it with an LP adapter
    that collapses numerical errors into infeasibility is invalid: unresolved
    status must be preserved.
    """

    if not isinstance(settings, YawMapDomainSettings):
        raise TypeError("settings must be YawMapDomainSettings")
    if not callable(evaluator):
        raise TypeError("evaluator must be callable")
    speeds = _knots(speed_knots, "speed_knots", nonnegative=True)
    accelerations = _knots(longitudinal_acceleration_knots, "longitudinal_acceleration_knots")
    bound = unitire_peak_lateral_bound(vehicle)
    audited_speeds = _audit_axis(speeds, settings.speed_ax_audit_subdivisions)
    audited_accelerations = _audit_axis(accelerations, settings.speed_ax_audit_subdivisions)
    cache: dict[tuple[float, float, float], ProbeResult] = {}
    unresolved: list[tuple[float, float, float, str]] = []
    exhausted = False
    direct_probe_count = 0

    def probe_many(points):
        nonlocal exhausted, direct_probe_count
        points = list(points)
        pending = []
        outputs = {}
        for speed, ax, ay in points:
            key = (speed, ax, ay)
            if key in cache:
                outputs[key] = cache[key]
            elif direct_probe_count + len(pending) < settings.probe_budget:
                if key not in pending: pending.append(key)
            else:
                exhausted = True
                result = ProbeResult(ProbeStatus.UNRESOLVED, 'probe budget exhausted')
                unresolved.append((*key, result.detail)); outputs[key] = result
        if pending:
            direct_probe_count += len(pending)
            batch = getattr(evaluator, 'probe_many', None)
            results = batch(pending) if callable(batch) else [evaluator(*p) for p in pending]
            if len(results) != len(pending): raise ValueError('probe_many returned incorrect result count')
            for key, result in zip(pending, results):
                if not isinstance(result, ProbeResult): raise TypeError('evaluator must return ProbeResult')
                cache[key] = result; outputs[key] = result
                if result.status is ProbeStatus.UNRESOLVED: unresolved.append((*key, result.detail))
        return [outputs[(s,a,y)] for s,a,y in points]

    def probe(speed: float, ax: float, ay: float) -> ProbeResult:
        nonlocal exhausted, direct_probe_count
        key = (speed, ax, ay)
        if key in cache:
            return cache[key]
        if direct_probe_count >= settings.probe_budget:
            exhausted = True
            result = ProbeResult(ProbeStatus.UNRESOLVED, "probe budget exhausted")
            unresolved.append((speed, ax, ay, result.detail))
            return result
        direct_probe_count += 1
        result = evaluator(speed, ax, ay)
        if not isinstance(result, ProbeResult):
            raise TypeError("evaluator must return ProbeResult")
        cache[key] = result
        if result.status is ProbeStatus.UNRESOLVED:
            unresolved.append((speed, ax, ay, result.detail))
        return result

    magnitude_samples = tuple(np.arange(0.0, bound.lateral_acceleration, settings.scan_spacing))
    if not magnitude_samples or magnitude_samples[-1] != bound.lateral_acceleration:
        magnitude_samples = magnitude_samples + (bound.lateral_acceleration,)
    slices: list[DomainSlice] = []
    for speed in audited_speeds:
        for ax in audited_accelerations:
            components: list[SampledComponent] = []
            slice_unresolved: list[float] = []
            for sign in (-1, 1):
                values = tuple(float(sign * magnitude) for magnitude in magnitude_samples)
                outcomes = tuple(probe_many((speed, ax, ay) for ay in values))
                slice_unresolved.extend(ay for ay, result in zip(values, outcomes)
                                        if result.status is ProbeStatus.UNRESOLVED)
                start: int | None = None
                for index, result in enumerate(outcomes + (ProbeResult(ProbeStatus.INFEASIBLE),)):
                    if result.status is ProbeStatus.FEASIBLE and start is None:
                        start = index
                    if result.status is not ProbeStatus.FEASIBLE and start is not None:
                        end = index - 1
                        lower = _refine_transition(probe, speed, ax, values, outcomes, start - 1, start,
                                                   settings.bracket_tolerance)
                        upper = _refine_transition(probe, speed, ax, values, outcomes, end, end + 1,
                                                   settings.bracket_tolerance)
                        sampled_lower = min(values[start], values[end])
                        sampled_upper = max(values[start], values[end])
                        transitions = (lower, upper)
                        lower_transition = min(
                            (item for item in transitions if item is not None),
                            key=lambda item: abs((item.feasible_ay + item.infeasible_ay) * 0.5 - sampled_lower),
                            default=None,
                        )
                        upper_transition = min(
                            (item for item in transitions if item is not None),
                            key=lambda item: abs((item.feasible_ay + item.infeasible_ay) * 0.5 - sampled_upper),
                            default=None,
                        )
                        components.append(SampledComponent(sign, sampled_lower, sampled_upper,
                                                           lower_transition, upper_transition))
                        start = None
            slices.append(DomainSlice(speed, ax, tuple(components), tuple(slice_unresolved)))
    return YawMapDomain(
        bound, speeds, accelerations, audited_speeds, audited_accelerations,
        tuple(slices), DiscoveryDiagnostics(direct_probe_count, settings.probe_budget, exhausted, tuple(unresolved)),
    )


def _refine_transition(probe, speed, ax, values, outcomes, left: int, right: int, tolerance: float) -> TransitionBracket | None:
    """Refine only an adjacent observed transition, never an entire ray."""

    if left < 0 or right >= len(values):
        return None
    left_result, right_result = outcomes[left], outcomes[right]
    if {left_result.status, right_result.status} != {ProbeStatus.FEASIBLE, ProbeStatus.INFEASIBLE}:
        return None
    left_ay, right_ay = values[left], values[right]
    while abs(right_ay - left_ay) > tolerance:
        middle = 0.5 * (left_ay + right_ay)
        result = probe(speed, ax, middle)
        if result.status is ProbeStatus.UNRESOLVED:
            return None
        if result.status is left_result.status:
            left_ay, left_result = middle, result
        else:
            right_ay, right_result = middle, result
    if left_result.status is ProbeStatus.FEASIBLE:
        return TransitionBracket(left_ay, right_ay)
    return TransitionBracket(right_ay, left_ay)
