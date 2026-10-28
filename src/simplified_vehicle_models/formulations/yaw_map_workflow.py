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
    finite_resolution_disclosure: str  # Finite bisection may not reduce every loss before budgets.


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
                 settings: YawMapWorkflowSettings, lateral_bound: float | None = None,
                 axes_to_refine: tuple[str, ...] = ("speed", "ax", "ay")
                 ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Refine one worst responsible holdout, keeping tensor growth bounded.

    Adding every 1/3 and 2/3 holdout to every axis makes one round roughly 27x
    larger.  A rectangular grid cannot locally split just one cell, so select a
    single worst face per round.  A full unsupported face ranks ahead of finite
    losses; outer truncation ranks ahead of masked interior loss.  This is a
    finite adaptive search, not a guarantee it will converge before its budgets.
    """
    if not set(axes_to_refine).issubset({"speed", "ax", "ay"}):
        raise ValueError("axes_to_refine must contain only speed, ax, and ay")
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
    # Split selected enclosing cells rather than inserting a training probe.
    # This prevents a pointwise fitted map from acquiring artificial zero loss.
    for name, axis, value in (("speed", speed, probe.speed), ("ax", ax, probe.longitudinal_acceleration)):
        if name in axes_to_refine:
            midpoint = _axis_midpoint(axis, value)
            if midpoint is not None:
                axis.add(midpoint)
    # Refine the gap toward existing support, not the infeasible side of the
    # direct boundary (especially important for positive ay exactly on a knot).
    supported = [float(value) for value in capability.lateral_acceleration
                 if interpolator_for(capability).query_evidence(probe.speed,
                    probe.longitudinal_acceleration, float(value))['reason'] is MapQueryReason.SUPPORTED]
    if supported:
        nearest = min(supported, key=lambda value: abs(value - probe.lateral_acceleration))
        lo, hi = sorted((nearest, probe.lateral_acceleration))
        gap_knots = sorted({value for value in ay if lo <= value <= hi} | {lo, hi})
        gaps = list(zip(gap_knots, gap_knots[1:]))
        ay_midpoint = (sum(max(gaps, key=lambda pair: pair[1]-pair[0])) / 2
                       if gaps else _axis_midpoint(ay, probe.lateral_acceleration))
    else:
        ay_midpoint = _axis_midpoint(ay, probe.lateral_acceleration)
    if "ay" in axes_to_refine and probe.map_reason is MapQueryReason.OUT_OF_DOMAIN and lateral_bound is not None:
        # Grow only toward directly feasible exterior evidence, never past the
        # model-derived finite tyre bound. `_fill_lateral_axis` preserves width.
        target = max(-lateral_bound, min(lateral_bound, probe.lateral_acceleration))
        if target < min(ay):
            ay.add(target)
        elif target > max(ay):
            ay.add(target)
    elif "ay" in axes_to_refine and ay_midpoint is not None:
        if lateral_bound is not None:
            ay_midpoint = max(-lateral_bound, min(lateral_bound, ay_midpoint))
        ay.add(ay_midpoint)
    return tuple(sorted(speed)), tuple(sorted(ax)), _fill_lateral_axis(ay, settings.maximum_lateral_cell_width)


def build_yaw_map_workflow(vehicle, speed_knots, longitudinal_acceleration_knots, evaluator, *,
                           yaw_inertia, tv_limits=None, settings=YawMapWorkflowSettings(),
                           map_builder=build_physical_yaw_map, validation_evaluator=None,
                           progress_callback=None):
    """Workflow entry point owning any automatically-created validation pool."""
    owned_validation = None
    if validation_evaluator is None:
        try:
            from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator
            from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
            if isinstance(evaluator, ParallelYawEvaluator):
                owned_validation = ParallelYawEvaluator(evaluator.vehicle, evaluator.tv_limits,
                    workers=evaluator.workers, time_budget_seconds=settings.time_budget_seconds,
                    use_reduced=evaluator.use_reduced, batch_size=evaluator.batch_size)
            elif isinstance(evaluator, PhysicalYawEvaluator):
                owned_validation = PhysicalYawEvaluator(evaluator.vehicle, evaluator.tv_limits,
                    time_budget_seconds=settings.time_budget_seconds, use_reduced=evaluator.use_reduced)
        except ImportError:
            pass
    try:
        return _build_yaw_map_workflow_impl(vehicle, speed_knots, longitudinal_acceleration_knots,
            evaluator, yaw_inertia=yaw_inertia, tv_limits=tv_limits, settings=settings,
            map_builder=map_builder, validation_evaluator=validation_evaluator or owned_validation,
            progress_callback=progress_callback)
    finally:
        if owned_validation is not None:
            owned_validation.close()


def _build_yaw_map_workflow_impl(
    vehicle,
    speed_knots: Iterable[float],
    longitudinal_acceleration_knots: Iterable[float],
    evaluator: Callable[[float, float, float], ProbeResult],
    *,
    yaw_inertia: float,
    tv_limits: TvLimits | None = None,
    settings: YawMapWorkflowSettings = YawMapWorkflowSettings(),
    map_builder: Callable[..., YawCapabilityMap] = build_physical_yaw_map,
    validation_evaluator: Callable[[float, float, float], ProbeResult] | None = None,
    progress_callback: Callable[[str, dict], None] | None = None,
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
    absolute_deadline = started + settings.time_budget_seconds
    # Concrete physical evaluators must not silently share their cache with
    # validation. Generic callables remain supported for synthetic tests.
    physical_type = None
    try:
        from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
        from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator
        physical_type = (PhysicalYawEvaluator, ParallelYawEvaluator)
    except ImportError:
        pass
    if physical_type and isinstance(evaluator, physical_type):
        if validation_evaluator is evaluator:
            raise ValueError('validation_evaluator must be independent from the physical training evaluator')
        if validation_evaluator is not None and isinstance(validation_evaluator, physical_type):
            if validation_evaluator is evaluator:
                raise ValueError('physical validation evaluator must be a distinct object')
            if (validation_evaluator.vehicle is not evaluator.vehicle
                    or validation_evaluator.tv_limits != evaluator.tv_limits
                    or validation_evaluator.use_reduced != evaluator.use_reduced):
                raise ValueError('physical validation evaluator inputs must match training evaluator')
        if validation_evaluator is None:
            if isinstance(evaluator, PhysicalYawEvaluator):
                validation_evaluator = PhysicalYawEvaluator(evaluator.vehicle, evaluator.tv_limits,
                    time_budget_seconds=settings.time_budget_seconds, use_reduced=evaluator.use_reduced)
            else:
                validation_evaluator = ParallelYawEvaluator(evaluator.vehicle, evaluator.tv_limits,
                    workers=evaluator.workers, time_budget_seconds=settings.time_budget_seconds,
                    use_reduced=evaluator.use_reduced, batch_size=evaluator.batch_size)
    if physical_type and isinstance(evaluator, ParallelYawEvaluator):
        evaluator.set_deadline(absolute_deadline)
    if physical_type and isinstance(validation_evaluator, ParallelYawEvaluator):
        validation_evaluator.set_deadline(absolute_deadline)
    # Discovery owns its finite probe budget; this wrapper adds the workflow's
    # deadline without relabelling unrun work as physical infeasibility.
    def deadline_evaluator(speed: float, ax: float, ay: float) -> ProbeResult:
        if time.monotonic() >= absolute_deadline:
            return ProbeResult(ProbeStatus.UNRESOLVED, "workflow time budget exhausted")
        return evaluator(speed, ax, ay)
    deadline_evaluator.set_deadline = lambda deadline: evaluator.set_deadline(deadline) if hasattr(evaluator, 'set_deadline') else None
    def deadline_evaluator_many(points):
        points = list(points)
        allowed = []
        results = [None] * len(points)
        for index, point in enumerate(points):
            if time.monotonic() >= absolute_deadline:
                results[index] = ProbeResult(ProbeStatus.UNRESOLVED, 'workflow time budget exhausted')
            else:
                allowed.append((index, point))
        batch = getattr(evaluator, 'probe_many', None)
        if allowed:
            if callable(batch) and hasattr(evaluator, 'set_deadline'):
                evaluator.set_deadline(absolute_deadline)
            values = batch([p for _, p in allowed]) if callable(batch) else [evaluator(*p) for _, p in allowed]
            if len(values) != len(allowed): raise ValueError('evaluator probe_many returned incorrect result count')
            for (index, _), result in zip(allowed, values):
                if not isinstance(result, ProbeResult): raise TypeError('evaluator must return ProbeResult')
                results[index] = result
        return results
    deadline_evaluator.probe_many = deadline_evaluator_many
    def progress(event, **fields):
        if progress_callback is not None:
            progress_callback(event, fields)

    progress('discovery_start', speed_points=len(speeds), ax_points=len(accelerations))
    domain = discover_yaw_map_domain(vehicle, speeds, accelerations, deadline_evaluator, settings.discovery)
    progress('discovery_complete', complete=domain.diagnostics.complete,
             probes=domain.diagnostics.probe_count)
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
    progress('training_audit_discovery_start')
    training_domain = discover_yaw_map_domain(vehicle, training_speeds, training_ax,
                                               deadline_evaluator, audit_settings)
    progress('training_audit_discovery_complete', complete=training_domain.diagnostics.complete,
             probes=training_domain.diagnostics.probe_count)
    rounds = 0
    last_capability: YawCapabilityMap | None = None
    last_audit: YawMapCompletionAudit | None = None
    worst_history: dict[tuple[float, float, float], float | None] = {}
    cross_axis = "speed"
    while True:
        node_count = len(map_speed) * len(map_ax) * len(lateral)
        if node_count > settings.maximum_map_nodes or time.monotonic() - started > settings.time_budget_seconds:
            # Never allocate a map beyond the declared cap.  Refinement retains
            # the last complete-size physical artifact rather than discarding
            # valuable partial coverage for a 1x1x1 placeholder.
            if last_capability is None:
                grid = YawCapabilityGrid((map_speed[0],), (map_ax[0],), (0.0,))
                expired = time.monotonic() >= absolute_deadline
                if expired:
                    # A process evaluator may already have reaped its workers
                    # on deadline. Do not invoke its builder again while
                    # unwinding; preserve a controlled unresolved artifact.
                    zeros = np.zeros((1, 1, 1))
                    capability = YawCapabilityMap(grid.speed, grid.longitudinal_acceleration,
                        grid.lateral_acceleration, zeros, zeros, np.zeros((1,1,1),bool), yaw_inertia,
                        'physical_no_tv_v1' if tv_limits is None else 'physical_tv_v1',
                        'unresolved-budget-placeholder')
                else:
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
        progress('map_build_start', round=rounds, nodes=node_count)
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
        progress('map_build_complete', round=rounds)
        progress('training_map_audit_start', round=rounds)
        audit = _audit_map(capability, training_domain, settings=settings, started=started)
        progress('training_map_audit_complete', round=rounds, complete=audit.complete,
                 probes=audit.direct_probe_count, interpolation_loss=audit.maximum_interpolation_loss)
        last_capability, last_audit = capability, audit
        # Train to a stricter guard band than untouched validation. This is
        # fixed a priori, never selected from the validation residuals.
        training_ready = (audit.complete and audit.maximum_interpolation_loss is not None
                          and audit.maximum_interpolation_loss <= audit.tolerance * 0.5)
        if (domain.diagnostics.complete and training_ready) or rounds >= settings.maximum_refinement_rounds:
            break
        candidates = (*audit.outer_truncations, *audit.interpolation_losses)
        if not candidates:
            # Unresolved/budget evidence is already incomplete but has no
            # physical boundary whose cell can honestly be refined.
            break
        worst = max(candidates, key=lambda probe: (
            int(probe.full_slice_unsupported), int(probe.map_reason is MapQueryReason.OUT_OF_DOMAIN),
            -1.0 if probe.loss_distance is None else probe.loss_distance))
        worst_key = (worst.speed, worst.longitudinal_acceleration, worst.lateral_acceleration)
        prior_loss = worst_history.get(worst_key)
        finite_stalled = (worst.loss_distance is not None and prior_loss is not None
                          and worst.loss_distance >= prior_loss - 1e-12)
        # Finite lateral loss has strict support at its v/ax face: spend the
        # cheap ay slab first.  Escalate only if that same boundary did not
        # improve; full unsupported faces need a cross-axis split immediately.
        if worst.full_slice_unsupported or finite_stalled:
            axes = (cross_axis,)
            cross_axis = "ax" if cross_axis == "speed" else "speed"
        else:
            axes = ("ay",)
        refined = _refine_axes(capability, audit, settings,
                               domain.lateral_bound.lateral_acceleration, axes)
        worst_history[worst_key] = worst.loss_distance
        if refined == (map_speed, map_ax, lateral):
            audit = replace(audit, refinement_stalled=True)
            last_audit = audit
            break
        map_speed, map_ax, lateral = refined
        rounds += 1
    def validation_probe(speed, ax, ay):
        if time.monotonic() >= absolute_deadline:
            return ProbeResult(ProbeStatus.UNRESOLVED, 'workflow time budget exhausted')
        return (validation_evaluator or evaluator)(speed, ax, ay)
    def validation_probe_many(points):
        selected = validation_evaluator or evaluator
        points = list(points); allowed = []; results = [None] * len(points)
        for index, point in enumerate(points):
            if time.monotonic() >= absolute_deadline:
                results[index] = ProbeResult(ProbeStatus.UNRESOLVED, 'workflow time budget exhausted')
            else: allowed.append((index, point))
        batch = getattr(selected, 'probe_many', None)
        if allowed:
            if callable(batch) and hasattr(selected, 'set_deadline'):
                selected.set_deadline(absolute_deadline)
            values = batch([p for _, p in allowed]) if callable(batch) else [selected(*p) for _, p in allowed]
            if len(values) != len(allowed): raise ValueError('validation probe_many returned incorrect result count')
            for (index, _), result in zip(allowed, values):
                if not isinstance(result, ProbeResult): raise TypeError('validation evaluator must return ProbeResult')
                results[index] = result
        return results
    validation_probe.set_deadline = lambda deadline: (validation_evaluator or evaluator).set_deadline(deadline) if hasattr((validation_evaluator or evaluator), 'set_deadline') else None
    validation_probe.probe_many = validation_probe_many
    progress('validation_discovery_start')
    validation_domain = discover_yaw_map_domain(vehicle, validation_speeds, validation_ax,
                                                 validation_probe, audit_settings)
    progress('validation_discovery_complete', complete=validation_domain.diagnostics.complete,
             probes=validation_domain.diagnostics.probe_count)
    progress('validation_map_audit_start')
    validation_audit = _audit_map(capability, validation_domain, settings=settings, started=started)
    progress('validation_map_audit_complete', complete=validation_audit.complete,
             probes=validation_audit.direct_probe_count,
             interpolation_loss=validation_audit.maximum_interpolation_loss)
    complete = (independent_holdouts and domain.diagnostics.complete
                and training_domain.diagnostics.complete and audit.complete
                and validation_domain.diagnostics.complete and validation_audit.complete)
    disclosure = "Finite scans, transition brackets, training holdouts, and disjoint quarter-position validation holdouts bound only the declared resolution; they are not continuous feasibility proof."
    if not independent_holdouts:
        disclosure += " A singleton declared speed or longitudinal-acceleration axis has no independent holdout coordinate and cannot qualify as complete."
    return YawMapWorkflowResult(capability, domain, audit, validation_audit, complete,
                                rounds, speeds, accelerations, disclosure)
