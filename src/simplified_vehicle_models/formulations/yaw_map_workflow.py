"""Build and finitely audit a strict physical yaw-capability map.

Public interface for the forthcoming CLI
----------------------------------------
``build_yaw_map_workflow(vehicle, speeds, accelerations, evaluator, *,
yaw_inertia, tv_limits=None, settings=..., map_builder=...)`` returns a
:class:`YawMapWorkflowResult`.  The caller supplies a direct LP ``evaluator``
which returns :class:`~simplified_vehicle_models.formulations.yaw_map_domain.ProbeResult`:
``FEASIBLE`` means *zero yaw demand* is feasible after both LP objective
directions were checked; ``UNRESOLVED`` is retained as a numerical failure.
The normal ``map_builder`` is :func:`build_physical_yaw_map`; it may be
injected for deterministic tests.

The result always contains the map, discovery record, and a completion audit.
``complete`` is deliberately false if any finite budget is exhausted, a direct
query is unresolved, an outer face is truncated, or strict interpolation loses
a directly feasible holdout by more than the declared tolerance.  It is a
finite-resolution result, not a continuous feasibility proof.  The map fields
report directional moment authority; zero-yaw feasibility is recorded
separately by the audit.  Testing full yaw-demand feasibility remains the
planner's responsibility.
"""
from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, replace
import math
import time

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import (
    TvLimits, YawCapabilityGrid, YawCapabilityMap, YawCapabilityNumericalError,
    build_physical_yaw_map,
)
from simplified_vehicle_models.formulations.yaw_map_domain import (
    DomainSlice, ProbeResult, ProbeStatus, YawMapDomain, YawMapDomainSettings,
    discover_yaw_map_domain,
)
from simplified_vehicle_models.solvers.yaw_profile_map import MapQueryReason, interpolator_for


@dataclass(frozen=True, slots=True)
class YawMapWorkflowSettings:
    """Finite construction budgets and the maximum lateral map-cell width.

    The 250,000 discovery-call default covers a 12 x 10 domain with a 77 m/s²
    tyre bound and 0.5 m/s² signed scans (about 37,000 ray samples), plus
    transition refinement and independent audits; 10,000/100,000 are too low
    for a realistic full signed scan.
    """

    discovery: YawMapDomainSettings = YawMapDomainSettings(probe_budget=250_000)
    maximum_lateral_cell_width: float = 0.5
    maximum_map_nodes: int = 30_000
    maximum_audit_probes: int = 250_000
    maximum_refinement_rounds: int = 4
    time_budget_seconds: float = 600.0

    def __post_init__(self) -> None:
        if not isinstance(self.discovery, YawMapDomainSettings):
            raise TypeError("discovery must be YawMapDomainSettings")
        if not math.isfinite(self.maximum_lateral_cell_width) or self.maximum_lateral_cell_width <= 0.0:
            raise ValueError("maximum_lateral_cell_width must be finite and positive")
        for name in ("maximum_map_nodes", "maximum_audit_probes", "maximum_refinement_rounds"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not math.isfinite(self.time_budget_seconds) or self.time_budget_seconds <= 0.0:
            raise ValueError("time_budget_seconds must be finite and positive")


@dataclass(frozen=True, slots=True)
class AuditProbe:
    speed: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    status: ProbeStatus
    map_reason: MapQueryReason | None
    # None serializes as JSON null: no finite distance is honest if a full
    # speed/ax holdout slice has no strict map support.
    loss_distance: float | None
    full_slice_unsupported: bool = False
    detail: str = ""


@dataclass(frozen=True, slots=True)
class YawMapCompletionAudit:
    """Evidence from acceptance points not used as map-grid discovery knots."""

    probes: tuple[AuditProbe, ...]
    direct_probe_count: int
    direct_probe_budget: int
    budget_exhausted: bool
    time_exhausted: bool
    outer_truncations: tuple[AuditProbe, ...]
    interpolation_losses: tuple[AuditProbe, ...]
    unresolved: tuple[AuditProbe, ...]
    zero_yaw_feasible: tuple[tuple[float, float, float], ...]
    # None means one or more feasible probes had no strict support anywhere on
    # their speed/ax face; it is deliberately JSON-safe rather than Infinity.
    maximum_interpolation_loss: float | None
    tolerance: float
    resolution_limited: bool = True
    refinement_stalled: bool = False

    @property
    def complete(self) -> bool:
        return (not self.budget_exhausted and not self.time_exhausted and not self.unresolved
                and not self.outer_truncations and not self.refinement_stalled
                and self.maximum_interpolation_loss is not None
                and self.maximum_interpolation_loss <= self.tolerance)


@dataclass(frozen=True, slots=True)
class YawMapWorkflowResult:
    capability: YawCapabilityMap
    domain: YawMapDomain
    # Training audit drives adaptive refinement; validation stays disjoint.
    audit: YawMapCompletionAudit
    validation_audit: YawMapCompletionAudit
    complete: bool
    refinement_rounds: int
    declared_speed_knots: tuple[float, ...]
    declared_longitudinal_acceleration_knots: tuple[float, ...]
    finite_resolution_disclosure: str


def _validated_knots(values: Iterable[float], name: str, *, nonnegative: bool = False) -> tuple[float, ...]:
    result = tuple(sorted(float(value) for value in values))
    if not result or any(not math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be nonempty and finite")
    if nonnegative and any(value < 0.0 for value in result):
        raise ValueError(f"{name} must be nonnegative")
    if len(set(result)) != len(result):
        raise ValueError(f"{name} must not contain duplicates")
    return result


def _fill_lateral_axis(points: Iterable[float], width: float) -> tuple[float, ...]:
    anchors = sorted(set(float(point) for point in points if math.isfinite(point)) | {0.0})
    if len(anchors) == 1:
        anchors = [-width, 0.0, width]
    result = [anchors[0]]
    for left, right in zip(anchors, anchors[1:]):
        pieces = max(1, math.ceil((right - left) / width))
        result.extend(left + (right - left) * index / pieces for index in range(1, pieces + 1))
    # Rounding can coalesce anchors only in pathological subnormal inputs.
    return tuple(sorted(set(result)))


def _domain_axis(domain: YawMapDomain, width: float) -> tuple[float, ...]:
    points: list[float] = [0.0]
    feasible_brackets: list[float] = []
    for slice_ in domain.slices:
        for component in slice_.components:
            # Retain sampled ends and only global bracket extremes.  Adding every
            # per-slice endpoint makes a shared rectangular axis impractical.
            points.extend((component.sampled_lower_ay, component.sampled_upper_ay))
            for transition in (component.lower_transition, component.upper_transition):
                if transition is not None:
                    feasible_brackets.append(transition.feasible_ay)
    if feasible_brackets:
        points.extend((min(feasible_brackets), max(feasible_brackets)))
    return _fill_lateral_axis(points, width)


def _holdout_axis(knots: tuple[float, ...], fractions: tuple[float, float]) -> tuple[float, ...]:
    """Return fixed interior fractions of original declared cells."""
    points = {left + (right - left) * fraction
              for left, right in zip(knots, knots[1:]) for fraction in fractions}
    return tuple(sorted(points)) or knots


def _axis_midpoint(axis: set[float], value: float) -> float | None:
    """Midpoint of the axis cell containing value; never return value itself."""
    values = sorted(axis)
    index = int(np.searchsorted(values, value))
    if index == 0:
        return None
    if index == len(values):
        return None
    return 0.5 * (values[index - 1] + values[index])


def _strict_supported_distance(interpolator, axis: np.ndarray, speed: float, ax: float, ay: float) -> float:
    """Distance to a lateral knot strict interpolation supports at this v/ax.

    This intentionally evaluates every lateral knot at the *same* holdout
    speed/ax.  A narrow ay cell elsewhere cannot conceal a wholly masked
    speed/ax face; no supported knot means infinite loss.
    """
    supported = []
    for candidate in axis:
        evidence = interpolator.query_evidence(speed, ax, float(candidate))
        if evidence["reason"] is MapQueryReason.SUPPORTED:
            supported.append(float(candidate))
    return min((abs(ay - candidate) for candidate in supported), default=None)


def _audit_map(capability: YawCapabilityMap, holdout_domain: YawMapDomain,
               *, settings: YawMapWorkflowSettings, started: float) -> YawMapCompletionAudit:
    """Audit independently discovered direct component boundaries.

    ``holdout_domain`` was scanned at speed/ax coordinates different from map
    discovery.  Its refined feasible transition ends are direct LP evidence of
    a boundary to which strict map support is compared.
    """
    probes: list[AuditProbe] = []
    outer: list[AuditProbe] = []
    losses: list[AuditProbe] = []
    unresolved: list[AuditProbe] = []
    zero: list[tuple[float, float, float]] = []
    # Holdout ray scanning consumes the same audit-call budget; map comparison
    # itself is deterministic and makes no further LP calls.
    count = holdout_domain.diagnostics.probe_count
    budget_exhausted = holdout_domain.diagnostics.budget_exhausted
    time_exhausted = False
    interpolator = interpolator_for(capability)
    ay_axis = np.asarray(capability.lateral_acceleration)
    strict_support_cache: dict[tuple[float, float], tuple[float, ...]] = {}
    for slice_ in holdout_domain.slices:
        for component in slice_.components:
            # Every sampled end and refined feasible transition end is direct
            # finite-resolution boundary evidence.  Deduplicate zero/shared ends.
            boundary_points = {component.sampled_lower_ay, component.sampled_upper_ay}
            for transition in (component.lower_transition, component.upper_transition):
                if transition is not None:
                    boundary_points.add(transition.feasible_ay)
            for ay in sorted(boundary_points):
                if time.monotonic() - started > settings.time_budget_seconds:
                    time_exhausted = True
                    break
                evidence = interpolator.query_evidence(slice_.speed, slice_.longitudinal_acceleration, ay)
                reason = evidence["reason"]
                key = (slice_.speed, slice_.longitudinal_acceleration)
                supported = strict_support_cache.get(key)
                if supported is None:
                    supported = tuple(float(candidate) for candidate in ay_axis
                                      if interpolator.query_evidence(slice_.speed, slice_.longitudinal_acceleration,
                                                                     float(candidate))["reason"]
                                      is MapQueryReason.SUPPORTED)
                    strict_support_cache[key] = supported
                loss = min((abs(ay - candidate) for candidate in supported), default=None)
                full_slice_unsupported = not supported
                probe = AuditProbe(slice_.speed, slice_.longitudinal_acceleration, ay,
                                   ProbeStatus.FEASIBLE, reason,
                                   0.0 if reason is MapQueryReason.SUPPORTED else loss,
                                   full_slice_unsupported)
                probes.append(probe)
                if ay == 0.0:
                    zero.append((slice_.speed, slice_.longitudinal_acceleration, ay))
                if reason is MapQueryReason.OUT_OF_DOMAIN:
                    outer.append(probe)
                elif reason is not MapQueryReason.SUPPORTED:
                    losses.append(probe)
            if budget_exhausted or time_exhausted:
                break
        if budget_exhausted or time_exhausted:
            break
    for speed, ax, ay, detail in holdout_domain.diagnostics.unresolved_probes:
        unresolved.append(AuditProbe(speed, ax, ay, ProbeStatus.UNRESOLVED, None, None, False, detail))
    maximum_loss = (None if any(probe.full_slice_unsupported for probe in losses)
                    else max((probe.loss_distance for probe in losses), default=0.0))
    return YawMapCompletionAudit(tuple(probes), count, settings.maximum_audit_probes,
                                 budget_exhausted, time_exhausted, tuple(outer), tuple(losses),
                                 tuple(unresolved), tuple(zero), maximum_loss,
                                 settings.discovery.lateral_interpolation_loss_tolerance)


def _refine_axes(capability: YawCapabilityMap, audit: YawMapCompletionAudit,
                 settings: YawMapWorkflowSettings, lateral_bound: float | None = None
                 ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Refine one worst responsible holdout, keeping tensor growth bounded.

    Adding every 1/3 and 2/3 holdout to every axis makes one round roughly 27x
    larger.  A rectangular grid cannot locally split just one cell, so select a
    single worst face per round.  A full unsupported face ranks ahead of finite
    losses; outer truncation ranks ahead of masked interior loss.  This is a
    finite adaptive search, not a guarantee it will converge before its budgets.
    """
    speed = set(float(value) for value in capability.speed)
    ax = set(float(value) for value in capability.longitudinal_acceleration)
    ay = set(float(value) for value in capability.lateral_acceleration)
    candidates = (*audit.outer_truncations, *audit.interpolation_losses)
    if not candidates:
        return tuple(sorted(speed)), tuple(sorted(ax)), tuple(sorted(ay))

    def priority(probe: AuditProbe) -> tuple[int, int, float]:
        return (int(probe.full_slice_unsupported),
                int(probe.map_reason is MapQueryReason.OUT_OF_DOMAIN),
                -1.0 if probe.loss_distance is None else probe.loss_distance)

    probe = max(candidates, key=priority)
    # Split enclosing cells rather than inserting the training probe itself.
    # This prevents a pointwise fitted map from acquiring artificial zero loss.
    for axis, value in ((speed, probe.speed), (ax, probe.longitudinal_acceleration)):
        midpoint = _axis_midpoint(axis, value)
        if midpoint is not None:
            axis.add(midpoint)
    ay_midpoint = _axis_midpoint(ay, probe.lateral_acceleration)
    if ay_midpoint is not None:
        ay.add(ay_midpoint)
    elif probe.map_reason is MapQueryReason.OUT_OF_DOMAIN and lateral_bound is not None:
        # Grow only toward directly feasible exterior evidence, never past the
        # model-derived finite tyre bound. `_fill_lateral_axis` preserves width.
        target = max(-lateral_bound, min(lateral_bound, probe.lateral_acceleration))
        if target < min(ay):
            ay.add(target)
        elif target > max(ay):
            ay.add(target)
    return tuple(sorted(speed)), tuple(sorted(ax)), _fill_lateral_axis(ay, settings.maximum_lateral_cell_width)


def build_yaw_map_workflow(
    vehicle,
    speed_knots: Iterable[float],
    longitudinal_acceleration_knots: Iterable[float],
    evaluator: Callable[[float, float, float], ProbeResult],
    *,
    yaw_inertia: float,
    tv_limits: TvLimits | None = None,
    settings: YawMapWorkflowSettings = YawMapWorkflowSettings(),
    map_builder: Callable[..., YawCapabilityMap] = build_physical_yaw_map,
) -> YawMapWorkflowResult:
    """Discover, build, directly audit, and finitely refine a strict map.

    An incomplete result is returned with its evidence instead of raising away
    the audit.  Callers must require ``result.complete`` before advertising a
    qualified map.
    """
    if not isinstance(settings, YawMapWorkflowSettings):
        raise TypeError("settings must be YawMapWorkflowSettings")
    if not callable(evaluator) or not callable(map_builder):
        raise TypeError("evaluator and map_builder must be callable")
    if not math.isfinite(yaw_inertia) or yaw_inertia <= 0.0:
        raise ValueError("yaw_inertia must be finite and positive")
    speeds = _validated_knots(speed_knots, "speed_knots", nonnegative=True)
    accelerations = _validated_knots(longitudinal_acceleration_knots, "longitudinal_acceleration_knots")
    started = time.monotonic()
    # Discovery owns its finite probe budget; this wrapper adds the workflow's
    # deadline without relabelling unrun work as physical infeasibility.
    def deadline_evaluator(speed: float, ax: float, ay: float) -> ProbeResult:
        if time.monotonic() - started > settings.time_budget_seconds:
            return ProbeResult(ProbeStatus.UNRESOLVED, "workflow time budget exhausted")
        return evaluator(speed, ax, ay)
    domain = discover_yaw_map_domain(vehicle, speeds, accelerations, deadline_evaluator, settings.discovery)
    lateral = _domain_axis(domain, settings.maximum_lateral_cell_width)
    map_speed, map_ax = speeds, accelerations
    # Acceptance coordinates are fixed subdivisions of the declared domain;
    # they are not regenerated from adaptive map knots.
    training_speeds = _holdout_axis(speeds, (1.0 / 3.0, 2.0 / 3.0))
    training_ax = _holdout_axis(accelerations, (1.0 / 3.0, 2.0 / 3.0))
    validation_speeds = _holdout_axis(speeds, (0.25, 0.75))
    validation_ax = _holdout_axis(accelerations, (0.25, 0.75))
    independent_holdouts = len(speeds) > 1 and len(accelerations) > 1
    # A separate discovery call makes all audit boundary scans independent of
    # map-axis discovery.  Its probe budget is the audit budget, not the map
    # discovery budget, and all LP calls remain explicitly accounted for.
    audit_settings = replace(settings.discovery, probe_budget=settings.maximum_audit_probes,
                             speed_ax_audit_subdivisions=0)
    training_domain = discover_yaw_map_domain(vehicle, training_speeds, training_ax,
                                               deadline_evaluator, audit_settings)
    rounds = 0
    last_capability: YawCapabilityMap | None = None
    last_audit: YawMapCompletionAudit | None = None
    while True:
        node_count = len(map_speed) * len(map_ax) * len(lateral)
        if node_count > settings.maximum_map_nodes or time.monotonic() - started > settings.time_budget_seconds:
            # Never allocate a map beyond the declared cap.  Refinement retains
            # the last complete-size physical artifact rather than discarding
            # valuable partial coverage for a 1x1x1 placeholder.
            if last_capability is None:
                grid = YawCapabilityGrid((map_speed[0],), (map_ax[0],), (0.0,))
                try:
                    capability = map_builder(vehicle, grid, tv_limits, yaw_inertia=yaw_inertia)
                except YawCapabilityNumericalError:
                    zeros = np.zeros((1, 1, 1))
                    capability = YawCapabilityMap(grid.speed, grid.longitudinal_acceleration,
                        grid.lateral_acceleration, zeros, zeros, np.zeros((1,1,1),bool), yaw_inertia,
                        'physical_no_tv_v1' if tv_limits is None else 'physical_tv_v1',
                        'unresolved-budget-placeholder')
            else:
                capability = last_capability
            if last_audit is None:
                audit = YawMapCompletionAudit((), 0, settings.maximum_audit_probes,
                                               node_count > settings.maximum_map_nodes,
                                               time.monotonic() - started > settings.time_budget_seconds,
                                               (), (), (), (), None,
                                               settings.discovery.lateral_interpolation_loss_tolerance)
            else:
                # Retain the completed-round audit evidence (including boundary
                # losses) and merely mark the next requested refinement budget
                # exhausted, so persisted diagnostics stay actionable.
                audit = replace(last_audit, budget_exhausted=(last_audit.budget_exhausted
                                or node_count > settings.maximum_map_nodes),
                                time_exhausted=(last_audit.time_exhausted
                                or time.monotonic() - started > settings.time_budget_seconds))
            break
        grid = YawCapabilityGrid(map_speed, map_ax, lateral)
        try:
            capability = map_builder(vehicle, grid, tv_limits, yaw_inertia=yaw_inertia)
        except YawCapabilityNumericalError as error:
            # Preserve an inspectable all-masked map when the first map build
            # hits a numerical LP failure; a later refinement retains the last
            # usable map instead.  Either way it is explicit unresolved evidence.
            if last_capability is None:
                shape = (len(grid.speed), len(grid.longitudinal_acceleration), len(grid.lateral_acceleration))
                zeros = np.zeros(shape)
                capability = YawCapabilityMap(grid.speed, grid.longitudinal_acceleration,
                    grid.lateral_acceleration, zeros, zeros, np.zeros(shape, dtype=bool), yaw_inertia,
                    "physical_no_tv_v1" if tv_limits is None else "physical_tv_v1", "unresolved-map-node")
            else:
                capability = last_capability
            audit = YawMapCompletionAudit((), 0, settings.maximum_audit_probes, False, False,
                (), (), (AuditProbe(float(grid.speed[0]), float(grid.longitudinal_acceleration[0]),
                float(grid.lateral_acceleration[0]), ProbeStatus.UNRESOLVED, None, None, False, str(error)),),
                (), None, settings.discovery.lateral_interpolation_loss_tolerance)
            break
        audit = _audit_map(capability, training_domain, settings=settings, started=started)
        last_capability, last_audit = capability, audit
        if (domain.diagnostics.complete and audit.complete) or rounds >= settings.maximum_refinement_rounds:
            break
        refined = _refine_axes(capability, audit, settings, domain.lateral_bound.lateral_acceleration)
        if refined == (map_speed, map_ax, lateral):
            audit = replace(audit, refinement_stalled=True)
            last_audit = audit
            break
        map_speed, map_ax, lateral = refined
        rounds += 1
    validation_domain = discover_yaw_map_domain(vehicle, validation_speeds, validation_ax,
                                                 deadline_evaluator, audit_settings)
    validation_audit = _audit_map(capability, validation_domain, settings=settings, started=started)
    complete = (independent_holdouts and domain.diagnostics.complete
                and training_domain.diagnostics.complete and audit.complete
                and validation_domain.diagnostics.complete and validation_audit.complete)
    disclosure = "Finite scans, transition brackets, training holdouts, and disjoint quarter-position validation holdouts bound only the declared resolution; they are not continuous feasibility proof."
    if not independent_holdouts:
        disclosure += " A singleton declared speed or longitudinal-acceleration axis has no independent holdout coordinate and cannot qualify as complete."
    return YawMapWorkflowResult(capability, domain, audit, validation_audit, complete,
                                rounds, speeds, accelerations, disclosure)
