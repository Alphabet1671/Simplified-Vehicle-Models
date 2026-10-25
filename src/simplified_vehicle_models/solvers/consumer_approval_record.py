"""Fail-closed, JSON-only consumer approval-record traceability verifier.

This is governance plumbing, not an approval authority: allocation evidence is
opaque, signatures are not cryptographically validated, and no qualification
or physical/consumer approval is conferred.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from types import MappingProxyType
from typing import Any, Mapping

from .convergence_policy import NUMERICAL_POLICY_V1, AccuracyPolicy


class ApprovalStatus(str, Enum):
    UNAPPROVED_POLICY = "UNAPPROVED_POLICY"
    MISSING_CONSUMER_ALLOCATION = "MISSING_CONSUMER_ALLOCATION"
    INVALID_RECORD = "INVALID_RECORD"
    IDENTITY_MISMATCH = "IDENTITY_MISMATCH"
    MISSING_REQUIRED_EVIDENCE = "MISSING_REQUIRED_EVIDENCE"
    QUALIFICATION_UNRESOLVED = "QUALIFICATION_UNRESOLVED"


@dataclass(frozen=True, slots=True)
class ApprovalVerification:
    status: ApprovalStatus
    qualified: bool
    accepted: bool
    geometry_state_convergence: None
    policy_approved: bool
    record: Any
    evidence: Any
    mismatch_field: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status.value, "qualified": False, "accepted": False,
                "geometry_state_convergence": None, "policy_approved": self.policy_approved,
                "record": _thaw(self.record), "evidence": _thaw(self.evidence),
                "mismatch_field": self.mismatch_field}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False)


_IDENTITY_FIELDS = ("source", "runtime", "chart_reserve", "boundary", "representation",
                    "realized_basis_grid", "seed", "reference", "replay")
_ALLOCATION_KEYS = {"time", "xy", "speed", "curvature", "length", "clearance"}


def _normalize(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("nonfinite number")
        return value
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("mapping keys must be strings")
            result[key] = _normalize(item)
        return result
    if isinstance(value, (list, tuple)):
        return [_normalize(item) for item in value]
    raise ValueError("value is not JSON data")


def _canonical(value: Any) -> str:
    return json.dumps(_normalize(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _meaningful_evidence(value: Any) -> bool:
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, Mapping):
        return bool(value)
    if isinstance(value, (list, tuple)):
        return bool(value)
    return False


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return value


def _result(status: ApprovalStatus, policy_approved: bool, record: Any = None,
            evidence: Any = None, mismatch_field: str | None = None) -> ApprovalVerification:
    return ApprovalVerification(status, False, False, None, policy_approved,
                                _freeze(record), _freeze(evidence), mismatch_field)


def verify_consumer_approval_record(record: Any, *, policy: AccuracyPolicy = NUMERICAL_POLICY_V1,
                                    executed_identity: Any, required_evidence: Any) -> ApprovalVerification:
    """Check evidence traceability and fail closed; never grant qualification."""
    approved = getattr(policy, "status", None) == "approved" and getattr(policy, "requirements_status", None) == "approved"
    if not approved:
        # Proposed policy status dominates malformed payloads, while preserving
        # each independently valid JSON payload for traceability.
        try:
            normalized_record = _normalize(record)
        except (TypeError, ValueError, RecursionError):
            normalized_record = None
        try:
            normalized_evidence = _normalize(required_evidence)
        except (TypeError, ValueError, RecursionError):
            normalized_evidence = None
        return _result(ApprovalStatus.UNAPPROVED_POLICY, False,
                       normalized_record, normalized_evidence)
    expected_id, expected_hash = getattr(policy, "identifier", None), getattr(policy, "sha256", None)
    try:
        normalized_record = _normalize(record)
        normalized_evidence = _normalize(required_evidence)
        normalized_identity = _normalize(executed_identity)
    except (TypeError, ValueError):
        return _result(ApprovalStatus.INVALID_RECORD, False)
    if not isinstance(normalized_record, dict):
        return _result(ApprovalStatus.INVALID_RECORD, True, normalized_record, normalized_evidence)
    if normalized_record.get("policy_identifier") != expected_id or normalized_record.get("policy_sha256") != expected_hash:
        return _result(ApprovalStatus.INVALID_RECORD, True, normalized_record, normalized_evidence)

    allocation = normalized_record.get("consumer_allocation")
    if (not isinstance(allocation, dict) or
        not all(isinstance(allocation.get(k), str) and allocation[k].strip() for k in ("consumer_owner", "review_owner", "allocation_id")) or
        not isinstance(allocation.get("dimensional_allocations"), dict) or
        not _ALLOCATION_KEYS.issubset(allocation["dimensional_allocations"]) or
        not all(_meaningful_evidence(allocation["dimensional_allocations"][key]) for key in _ALLOCATION_KEYS) or
        not _meaningful_evidence(allocation.get("uncertainty"))):
        return _result(ApprovalStatus.MISSING_CONSUMER_ALLOCATION, True, normalized_record, normalized_evidence)

    rec_identity = normalized_record.get("executed_identity")
    if not isinstance(rec_identity, dict) or not isinstance(normalized_identity, dict):
        return _result(ApprovalStatus.IDENTITY_MISMATCH, True, normalized_record, normalized_evidence, "executed_identity")
    for field in _IDENTITY_FIELDS:
        if field not in rec_identity or field not in normalized_identity or _canonical(rec_identity[field]) != _canonical(normalized_identity[field]):
            return _result(ApprovalStatus.IDENTITY_MISMATCH, True, normalized_record, normalized_evidence, field)

    if not isinstance(normalized_evidence, dict):
        return _result(ApprovalStatus.MISSING_REQUIRED_EVIDENCE, True, normalized_record, normalized_evidence)
    for name in ("grid", "seed", "reference", "replay", "original_model"):
        item = normalized_evidence.get(name)
        if not isinstance(item, dict) or item.get("passed") is not True or not _meaningful_evidence(item.get("trace")):
            return _result(ApprovalStatus.MISSING_REQUIRED_EVIDENCE, True, normalized_record, normalized_evidence)
    if normalized_evidence.get("maximum_accuracy_certified") is not True or normalized_evidence.get("meets_accuracy_budget") is not True:
        return _result(ApprovalStatus.QUALIFICATION_UNRESOLVED, True, normalized_record, normalized_evidence)
    return _result(ApprovalStatus.QUALIFICATION_UNRESOLVED, True, normalized_record, normalized_evidence)
