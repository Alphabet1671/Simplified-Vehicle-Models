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
                "record": self.record, "evidence": self.evidence,
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


def verify_consumer_approval_record(record: Any, *, policy: AccuracyPolicy = NUMERICAL_POLICY_V1,
                                    executed_identity: Any, required_evidence: Any) -> ApprovalVerification:
    """Check evidence traceability and fail closed; never grant qualification."""
    try:
        normalized_record = _normalize(record)
        normalized_evidence = _normalize(required_evidence)
        normalized_identity = _normalize(executed_identity)
    except (TypeError, ValueError):
        return ApprovalVerification(ApprovalStatus.INVALID_RECORD, False, False, None, False, None, None)

    approved = getattr(policy, "status", None) == "approved" and getattr(policy, "requirements_status", None) == "approved"
    expected_id, expected_hash = getattr(policy, "identifier", None), getattr(policy, "sha256", None)
    if not approved:
        return ApprovalVerification(ApprovalStatus.UNAPPROVED_POLICY, False, False, None, False,
                                   normalized_record, normalized_evidence)
    if not isinstance(normalized_record, dict):
        return ApprovalVerification(ApprovalStatus.INVALID_RECORD, False, False, None, True, normalized_record, normalized_evidence)
    if normalized_record.get("policy_identifier") != expected_id or normalized_record.get("policy_sha256") != expected_hash:
        return ApprovalVerification(ApprovalStatus.INVALID_RECORD, False, False, None, True, normalized_record, normalized_evidence)

    allocation = normalized_record.get("consumer_allocation")
    if (not isinstance(allocation, dict) or
        not all(isinstance(allocation.get(k), str) and allocation[k].strip() for k in ("consumer_owner", "review_owner", "allocation_id")) or
        not isinstance(allocation.get("dimensional_allocations"), dict) or
        not _ALLOCATION_KEYS.issubset(allocation["dimensional_allocations"]) or
        "uncertainty" not in allocation):
        return ApprovalVerification(ApprovalStatus.MISSING_CONSUMER_ALLOCATION, False, False, None, True, normalized_record, normalized_evidence)

    rec_identity = normalized_record.get("executed_identity")
    if not isinstance(rec_identity, dict) or not isinstance(normalized_identity, dict):
        return ApprovalVerification(ApprovalStatus.IDENTITY_MISMATCH, False, False, None, True, normalized_record, normalized_evidence, "executed_identity")
    for field in _IDENTITY_FIELDS:
        if field not in rec_identity or field not in normalized_identity or _canonical(rec_identity[field]) != _canonical(normalized_identity[field]):
            return ApprovalVerification(ApprovalStatus.IDENTITY_MISMATCH, False, False, None, True, normalized_record, normalized_evidence, field)

    if not isinstance(normalized_evidence, dict):
        return ApprovalVerification(ApprovalStatus.MISSING_REQUIRED_EVIDENCE, False, False, None, True, normalized_record, normalized_evidence)
    for name in ("grid", "seed", "reference", "replay", "original_model"):
        item = normalized_evidence.get(name)
        if not isinstance(item, dict) or item.get("passed") is not True:
            return ApprovalVerification(ApprovalStatus.MISSING_REQUIRED_EVIDENCE, False, False, None, True, normalized_record, normalized_evidence)
    if normalized_evidence.get("maximum_accuracy_certified") is not True or normalized_evidence.get("meets_accuracy_budget") is not True:
        return ApprovalVerification(ApprovalStatus.QUALIFICATION_UNRESOLVED, False, False, None, True, normalized_record, normalized_evidence)
    return ApprovalVerification(ApprovalStatus.QUALIFICATION_UNRESOLVED, False, False, None, True, normalized_record, normalized_evidence)
