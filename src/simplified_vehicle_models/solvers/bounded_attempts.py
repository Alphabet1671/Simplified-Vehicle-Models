"""Pure parent-deadline controller for bounded synchronous attempts.

This coordinates stages only; it does not certify physical feasibility or
qualification, and cannot forcibly cancel a synchronous active call.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from types import MappingProxyType
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class AttemptBudget:
    """Finite positive parent wall-clock allowance in seconds."""
    wall_cap_s: float

    def __post_init__(self) -> None:
        value = self.wall_cap_s
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("wall_cap_s must be a finite positive scalar")
        object.__setattr__(self, "wall_cap_s", float(value))


class AttemptTerminal(Enum):
    LOCALLY_VALIDATED = "locally_validated"
    DEADLINE_EXHAUSTED = "deadline_exhausted"
    CANCELLED = "cancelled"
    STAGE_FAILED = "stage_failed"
    VALIDATION_UNRESOLVED = "validation_unresolved"


@dataclass(frozen=True)
class StageResult:
    """Stage carrier; only validate may set locally_valid, never acceptance."""
    value: Any = None
    locally_valid: bool = False
    resolved: bool = True


@dataclass(frozen=True)
class AttemptContext:
    deadline: float
    remaining_s: float
    previous_results: Mapping[str, Any]


STAGES = ("seed", "build", "solve", "search", "validate", "export")


def _safe(value: Any, depth: int = 0) -> Any:
    if depth > 20:
        raise ValueError("report value nesting too deep")
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_safe(x, depth + 1) for x in value]
    if isinstance(value, Mapping):
        return {str(k): _safe(v, depth + 1) for k, v in value.items()}
    return {"type": type(value).__name__, "repr": str(value)[:500]}


def run_bounded_attempt(*, budget: AttemptBudget, clock: Callable[[], float], stages: Mapping[str, Callable], cancellation=None) -> dict:
    """Run exact named ordered stages under one deadline; acceptance is always false.

    Each callable receives immutable AttemptContext. Validation returns StageResult
    (locally_valid / resolved) or a mapping with those two keys.
    """
    if not isinstance(budget, AttemptBudget):
        raise TypeError("budget must be AttemptBudget")
    if not callable(clock):
        raise TypeError("clock must be callable")
    if not isinstance(stages, Mapping) or tuple(stages.keys()) != STAGES or not all(callable(stages[k]) for k in STAGES):
        raise ValueError("stages must contain exactly seed, build, solve, search, validate, export in order")
    records, state = [], {}
    start_clock = None
    deadline = None
    terminal = AttemptTerminal.STAGE_FAILED
    reason = "invalid_clock"

    def now():
        nonlocal start_clock, deadline
        x = clock()
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            raise ValueError("clock must return finite scalar")
        x = float(x)
        if start_clock is None:
            start_clock, deadline = x, x + budget.wall_cap_s
            if not math.isfinite(deadline):
                raise ValueError("deadline is non-finite")
        return x

    try:
        now()
    except Exception as exc:
        return _report(terminal, reason, records, state, exc)
    for name in STAGES:
        try:
            current = now()
        except Exception as exc:
            terminal, reason = AttemptTerminal.STAGE_FAILED, "invalid_clock"
            break
        if cancellation is not None and (cancellation() if callable(cancellation) else bool(cancellation)):
            terminal, reason = AttemptTerminal.CANCELLED, "cancelled"
            break
        if current >= deadline:
            terminal, reason = AttemptTerminal.DEADLINE_EXHAUSTED, "deadline_exhausted"
            break
        elapsed_start = current - start_clock
        rec = {"stage": name, "start_elapsed_s": elapsed_start, "end_elapsed_s": None, "outcome": "failed"}
        records.append(rec)
        context = AttemptContext(deadline, max(0.0, deadline-current), MappingProxyType(dict(state)))
        try:
            result = stages[name](context)
            after = now()
            rec["end_elapsed_s"] = after-start_clock
            rec["outcome"] = "completed"
            state[name] = result.value if isinstance(result, StageResult) else result
        except Exception as exc:
            try:
                after = now()
                rec["end_elapsed_s"] = after-start_clock
            except Exception:
                rec["end_elapsed_s"] = None
            rec.update(outcome="failed", error_type=type(exc).__name__, error_message=str(exc)[:500])
            terminal, reason = AttemptTerminal.STAGE_FAILED, "stage_failed"
            break
        if after >= deadline:
            terminal, reason = AttemptTerminal.DEADLINE_EXHAUSTED, "deadline_exhausted"
            break
        if name == "validate":
            try:
                valid, resolved = _validation(result)
            except Exception as exc:
                rec.update(outcome="failed", error_type=type(exc).__name__, error_message=str(exc)[:500])
                terminal, reason = AttemptTerminal.STAGE_FAILED, "malformed_stage_return"
                break
            if not resolved:
                terminal, reason = AttemptTerminal.VALIDATION_UNRESOLVED, "validation_unresolved"
                break
            if not valid:
                terminal, reason = AttemptTerminal.VALIDATION_UNRESOLVED, "validation_not_locally_valid"
                break
        if name == "export":
            terminal, reason = AttemptTerminal.LOCALLY_VALIDATED, "locally_validated_export_complete"
    for name in STAGES[len(records):]:
        records.append({"stage": name, "start_elapsed_s": None, "end_elapsed_s": None, "outcome": "skipped"})
    return _report(terminal, reason, records, state)


def _validation(result):
    if isinstance(result, StageResult):
        return result.locally_valid is True, result.resolved is True
    if isinstance(result, Mapping) and set(result) >= {"locally_valid", "resolved"}:
        if type(result["locally_valid"]) is not bool or type(result["resolved"]) is not bool:
            raise ValueError("validation flags must be bool")
        return result["locally_valid"], result["resolved"]
    raise ValueError("validate must return StageResult or mapping with locally_valid and resolved")


def _report(terminal, reason, records, state, error=None):
    data = {"accepted": False, "terminal": terminal.value, "terminal_reason": reason,
            "stages": records, "attempt_state": state,
            "hard_deadline_enforced": False,
            "limitation": "Synchronous active calls cannot be forcibly cancelled; no physical feasibility or qualification inference."}
    if error is not None:
        data["error_type"] = type(error).__name__
        data["error_message"] = str(error)[:500]
    try:
        return _safe(data)
    except Exception:
        return {"accepted": False, "terminal": AttemptTerminal.STAGE_FAILED.value,
                "terminal_reason": "report_serialization_failed", "stages": [], "attempt_state": {},
                "hard_deadline_enforced": False,
                "limitation": "Synchronous active calls cannot be forcibly cancelled; no physical feasibility or qualification inference."}
