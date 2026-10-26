"""Pure structural preflight for the versioned track-problem request contract.

A ``supported`` result only means the declared boundary/topology is compatible
with the currently implemented route. It is not geometry, chart, or physics
qualification; those checks require loaded reference data and a solver.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any


_SCHEMA = "track_problem_v1"
_REQUEST_KEYS = {"schema", "topology", "source_stations", "lower_offset", "upper_offset", "boundary"}
_BOUNDARY_KEYS = {"start_mode", "start_speed_mps", "finish_mode", "finish_speed_mps"}


def _invalid(*reasons: str) -> dict[str, Any]:
    return {"status": "invalid", "reasons": sorted(set(reasons)), "normalized_contract": None, "normalized_hash": None}


def _number(value: Any) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value) and math.isfinite(float(value))
    except (OverflowError, TypeError, ValueError):
        return False


def validate_problem_contract(request: dict) -> dict:
    """Validate and normalize a JSON-shaped ``track_problem_v1`` request.

    Malformed requests return ``invalid`` rather than raising. Accepted
    normalized structures use JSON-native values and a canonical SHA-256 hash.
    """
    if not isinstance(request, dict):
        return _invalid("request_must_be_object")
    if set(request) != _REQUEST_KEYS:
        return _invalid("unknown_or_missing_request_keys")
    if not isinstance(request.get("schema"), str) or request.get("schema") != _SCHEMA:
        return _invalid("unsupported_schema")
    topology = request.get("topology")
    if not isinstance(topology, str) or topology not in ("open", "closed"):
        return _invalid("invalid_topology")

    arrays: dict[str, list[float]] = {}
    for name in ("source_stations", "lower_offset", "upper_offset"):
        values = request.get(name)
        if not isinstance(values, list) or len(values) < 2:
            return _invalid(f"invalid_{name}")
        if not all(_number(value) for value in values):
            return _invalid(f"invalid_{name}")
        arrays[name] = [float(value) for value in values]
    stations, lower, upper = (arrays[n] for n in ("source_stations", "lower_offset", "upper_offset"))
    if len(stations) != len(lower) or len(stations) != len(upper):
        return _invalid("array_length_mismatch")
    if any(b <= a for a, b in zip(stations, stations[1:])):
        return _invalid("source_stations_not_strictly_increasing")
    if any(lo > 0.0 or hi < 0.0 or hi <= lo for lo, hi in zip(lower, upper)):
        return _invalid("invalid_corridor_width_or_centreline")

    boundary = request.get("boundary")
    if not isinstance(boundary, dict):
        return _invalid("boundary_must_be_object")
    if set(boundary) - _BOUNDARY_KEYS:
        return _invalid("unknown_boundary_keys")
    if not {"start_mode", "start_speed_mps", "finish_mode"}.issubset(boundary):
        return _invalid("missing_boundary_fields")
    start_mode, start_speed, finish_mode = boundary["start_mode"], boundary["start_speed_mps"], boundary["finish_mode"]
    if not isinstance(start_mode, str) or start_mode not in ("flying", "rolling", "standing"):
        return _invalid("invalid_start_mode")
    if not isinstance(finish_mode, str) or finish_mode not in ("periodic", "free", "fixed_speed"):
        return _invalid("invalid_finish_mode")
    if not _number(start_speed) or start_speed < 0.0:
        return _invalid("invalid_start_speed")
    has_finish_speed = "finish_speed_mps" in boundary
    finish_speed = boundary.get("finish_speed_mps")
    if finish_mode == "fixed_speed":
        if not has_finish_speed or not _number(finish_speed) or finish_speed < 0.0:
            return _invalid("invalid_finish_speed")
    elif has_finish_speed:
        return _invalid("finish_speed_not_applicable")

    if start_mode == "standing":
        if start_speed != 0.0:
            return _invalid("standing_start_requires_zero_speed")
    elif start_speed <= 0.0:
        return _invalid("positive_start_speed_required")
    if topology == "open" and finish_mode == "periodic":
        return _invalid("open_periodic_boundary_contradiction")

    normalized_boundary: dict[str, Any] = {
        "start_mode": start_mode,
        "start_speed_mps": float(start_speed),
        "finish_mode": finish_mode,
    }
    if has_finish_speed:
        normalized_boundary["finish_speed_mps"] = float(finish_speed)
    normalized = {
        "schema": _SCHEMA,
        "topology": topology,
        "source_stations": stations,
        "lower_offset": lower,
        "upper_offset": upper,
        "boundary": normalized_boundary,
    }

    reasons: list[str] = []
    if start_mode == "standing":
        reasons.append("standing_requires_coupled_event_solver")
    if finish_mode == "fixed_speed":
        if finish_speed == 0.0:
            reasons.append("terminal_rest_requires_event_solver")
        else:
            reasons.append("finish_speed_constraint_not_implemented")
    if topology == "closed":
        if finish_mode != "periodic":
            reasons.append("closed_boundary_not_implemented")
        elif start_mode != "flying":
            reasons.append("boundary_solver_not_implemented")
    elif start_mode != "rolling":
        reasons.append("boundary_solver_not_implemented")

    reasons = sorted(set(reasons))
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return {
        "status": "unsupported" if reasons else "supported",
        "reasons": reasons,
        "normalized_contract": normalized,
        "normalized_hash": hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
    }
