"""Pure evidence ledger for comparing bounded refinement attempts.

This module consumes literal records only: it does not run workers, determine
physical feasibility, or qualify/accept a solution. ``diagnostic_best`` is a
label supplied by independent-validation evidence, not an objective ranking.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import Any


class ComparisonStatus(str, Enum):
    COMPARABLE = "comparable"
    INCOMPARABLE = "incomparable"
    UNRESOLVED = "unresolved"
    NO_INDEPENDENTLY_FEASIBLE = "no_independently_feasible"


class Terminal(str, Enum):
    INDEPENDENTLY_FEASIBLE = "independently_feasible"
    LOCALLY_VALIDATED = "locally_validated"
    SOLVER_COMPLETED = "solver_completed"
    FAILED = "failed"
    UNRESOLVED = "unresolved"
    NO_PROGRESS = "no_progress"


@dataclass(frozen=True)
class IndependentValidation:
    """Explicit external validation evidence associated with an attempt."""
    status: Terminal
    evidence: Any


@dataclass(frozen=True)
class Attempt:
    """One immutable input attempt; evidence mappings are copied on output."""
    mesh_stations: Any
    constraint_stations: Any
    seed_ancestry: Any
    stage: str
    elapsed_s: float
    terminal: Terminal
    outcome: Any
    independent_validation: IndependentValidation | None = None


@dataclass(frozen=True)
class StrategyRecord:
    name: str
    identity: Any
    attempts: tuple[Attempt, ...]


@dataclass(frozen=True)
class ComparisonResult:
    status: ComparisonStatus
    strategies: Any
    diagnostic_best: str | None
    reason: str | None
    diagnostic_candidates: Any = ()
    accepted: bool = False
    physical_feasibility_certified: bool = False
    comparison_is_not_qualification: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status.value, "strategies": _normal(self.strategies),
                "diagnostic_best": self.diagnostic_best,
                "diagnostic_candidates": _normal(self.diagnostic_candidates), "reason": self.reason,
                "accepted": False, "physical_feasibility_certified": False,
                "comparison_is_not_qualification": True}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, allow_nan=False, separators=(",", ":"))


def _normal(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (str, bool)) or value is None:
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("data must contain only finite numbers")
        return value
    if isinstance(value, (list, tuple)):
        return [_normal(v) for v in value]
    if isinstance(value, dict):
        if any(not isinstance(k, str) for k in value):
            raise ValueError("JSON object keys must be strings")
        return {k: _normal(v) for k, v in value.items()}
    raise ValueError(f"data is not strict JSON-safe: {type(value).__name__}")


def _canonical(value: Any) -> str:
    return json.dumps(_normal(value), sort_keys=True, allow_nan=False, separators=(",", ":"))


def _reject_numeric_booleans(value: Any, label: str) -> None:
    """Reject bool where chart/reserve data uses a numeric value slot.

    The identity schema intentionally leaves chart/reserve key names open; any
    boolean nested in that field is ambiguous with a numeric setting and fails
    closed, without imposing types or tolerances on other values.
    """
    if isinstance(value, bool):
        raise ValueError(f"{label} must not contain boolean numeric values")
    if isinstance(value, dict):
        for nested in value.values():
            _reject_numeric_booleans(nested, label)
    elif isinstance(value, (list, tuple)):
        for nested in value:
            _reject_numeric_booleans(nested, label)


def _station_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{label} must be a station sequence")
    for station in value:
        if isinstance(station, bool) or not isinstance(station, (int, float)) or not math.isfinite(station):
            raise ValueError(f"{label} must contain finite non-boolean numbers")
    return _normal(value)


def _attempt_dict(a: Attempt) -> dict[str, Any]:
    return {"mesh_stations": _station_list(a.mesh_stations, "mesh_stations"),
            "constraint_stations": _station_list(a.constraint_stations, "constraint_stations"),
            "seed_ancestry": _normal(a.seed_ancestry), "stage": a.stage,
            "elapsed_s": _normal(a.elapsed_s), "terminal": a.terminal.value,
            "outcome": _normal(a.outcome),
            "independent_validation": None if a.independent_validation is None else {
                "status": a.independent_validation.status.value,
                "evidence": _normal(a.independent_validation.evidence)}}


def compare_strategies(records: Any) -> ComparisonResult:
    """Normalize exactly uniform/localized/exchange records without ranking.

    Exactly one strategy with explicit independent validation is exposed as
    ``diagnostic_best``. Multiple qualifying strategies remain ambiguous and
    are listed as ``diagnostic_candidates`` in input-record order. No solver or
    local status grants feasibility. Inputs are rejected (never coerced) when
    malformed.
    """
    if not isinstance(records, (list, tuple)) or len(records) != 3:
        raise ValueError("exactly three strategy records are required")
    if any(not isinstance(r, StrategyRecord) for r in records):
        raise TypeError("records must be StrategyRecord instances")
    names = [r.name for r in records]
    expected = {"uniform", "localized", "exchange"}
    if len(set(names)) != 3 or set(names) != expected:
        raise ValueError("strategy names must be unique and exactly uniform, localized, exchange")
    required = ("initial_seed", "target_physics", "chart_reserve", "parent_wall_cap_s", "boundary", "representation")
    normalized: dict[str, Any] = {}
    for r in records:
        if not isinstance(r.identity, dict) or set(r.identity) != set(required):
            raise ValueError("identity must contain exactly the required shared fields")
        cap = r.identity["parent_wall_cap_s"]
        if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not math.isfinite(cap) or cap <= 0:
            raise ValueError("parent_wall_cap_s must be finite positive non-boolean numeric data")
        _reject_numeric_booleans(r.identity["chart_reserve"], "chart_reserve")
        _canonical(r.identity)
        if not isinstance(r.attempts, (list, tuple)):
            raise ValueError("attempts must be an ordered sequence")
        attempts = []
        for a in r.attempts:
            if not isinstance(a, Attempt):
                raise TypeError("attempt sequence must contain Attempt records")
            if not isinstance(a.terminal, Terminal):
                raise ValueError("terminal must be a closed Terminal value")
            if not isinstance(a.stage, str) or not a.stage:
                raise ValueError("stage must be a nonempty string")
            if isinstance(a.elapsed_s, bool) or not isinstance(a.elapsed_s, (int, float)) or not math.isfinite(a.elapsed_s) or a.elapsed_s < 0:
                raise ValueError("elapsed_s must be finite nonnegative numeric data")
            if a.independent_validation is not None:
                iv = a.independent_validation
                if not isinstance(iv, IndependentValidation) or not isinstance(iv.status, Terminal):
                    raise ValueError("independent_validation must use typed validation evidence")
            attempts.append(_attempt_dict(a))
        normalized[r.name] = {"identity": _normal(r.identity), "attempts": attempts}
    for field in required:
        values = [_canonical(normalized[name]["identity"][field]) for name in sorted(expected)]
        if values[1:] != values[:-1]:
            return ComparisonResult(ComparisonStatus.INCOMPARABLE, normalized, None, f"identity mismatch: {field}", [])
    feasible = []
    for record in records:
        for attempt in record.attempts:
            iv = attempt.independent_validation
            if attempt.terminal is Terminal.INDEPENDENTLY_FEASIBLE and iv is not None and iv.status is Terminal.INDEPENDENTLY_FEASIBLE:
                feasible.append(record.name)
                break
    candidates = feasible
    if len(candidates) == 1:
        best = candidates[0]
        status = ComparisonStatus.COMPARABLE
        reason = None
    elif len(candidates) > 1:
        best = None
        status = ComparisonStatus.COMPARABLE
        reason = "multiple strategies carry explicit independently_feasible validation evidence; no ranking applied"
    else:
        best = None
        status = ComparisonStatus.NO_INDEPENDENTLY_FEASIBLE
        reason = "no attempt carries explicit independently_feasible validation evidence"
    return ComparisonResult(status, normalized, best, reason, candidates)


__all__ = ["Attempt", "ComparisonResult", "ComparisonStatus", "IndependentValidation", "StrategyRecord", "Terminal", "compare_strategies"]
