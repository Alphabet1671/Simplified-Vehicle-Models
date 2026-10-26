"""Express track edges as a fold-guarded lateral corridor.

The lateral offset ``n`` is positive to the right of travel. With the
repository's right-handed north/east/down frame, positive curvature is a right
turn and the offset-path fold condition is ``1 - curvature * n > 0``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
from types import MappingProxyType
from typing import Mapping
from collections.abc import Mapping as MappingABC

import numpy as np

from simplified_vehicle_models.core.types import FloatArray

LEGACY_POLICY = "legacy-abs-source-curvature-v1"
SIGNED_POLICY = "signed-fitted-reference-v1"


def _thaw(value):
    if isinstance(value, MappingABC):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return value


def _freeze(value):
    """Validate and recursively detach canonical JSON values."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if not np.isfinite(value):
            raise ValueError("policy descriptor numbers must be finite")
        return value
    if isinstance(value, MappingABC):
        if not all(isinstance(key, str) for key in value):
            raise TypeError("policy descriptor keys must be strings")
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    raise TypeError("policy descriptor must contain only canonical JSON values")


@dataclass(frozen=True, slots=True)
class Corridor:
    """Read-only admissible lateral offsets on the track distance grid."""

    distance: FloatArray
    minimum: FloatArray
    maximum: FloatArray
    provenance: str
    is_closed: bool = True
    policy: str = LEGACY_POLICY
    policy_descriptor: Mapping[str, object] = field(default_factory=lambda: MappingProxyType({}))
    policy_hash: str = ""

    def __post_init__(self) -> None:
        for name in ("distance", "minimum", "maximum"):
            values = np.array(getattr(self, name), dtype=float, copy=True)
            values.setflags(write=False)
            object.__setattr__(self, name, values)
        if not (self.distance.shape == self.minimum.shape == self.maximum.shape):
            raise ValueError("corridor arrays must have matching shapes")
        if (not np.all(np.isfinite(self.distance)) or not np.all(np.isfinite(self.minimum)) or not np.all(np.isfinite(self.maximum))):
            raise ValueError("corridor arrays must be finite")
        if np.any(self.minimum > 0.0) or np.any(self.maximum < 0.0):
            raise ValueError("corridor bounds must contain the centreline")
        if np.any(self.minimum > self.maximum):
            raise ValueError("corridor minimum must not exceed maximum")
        if self.policy not in (LEGACY_POLICY, SIGNED_POLICY):
            raise ValueError("unsupported corridor policy")
        if not isinstance(self.policy_descriptor, MappingABC):
            raise TypeError("policy_descriptor must be a mapping")
        descriptor = _freeze(self.policy_descriptor)
        # Hash only ordinary dict/list JSON structures, not MappingProxy/tuples.
        canonical = _thaw(descriptor)
        payload = json.dumps(canonical, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        expected_hash = hashlib.sha256(payload).hexdigest() if canonical else ""
        if self.policy_hash and not expected_hash:
            raise ValueError("policy_hash requires a nonempty descriptor")
        if self.policy_hash and self.policy_hash != expected_hash:
            raise ValueError("policy_hash does not match policy_descriptor")
        object.__setattr__(self, "policy_descriptor", descriptor)
        if not self.policy_hash:
            object.__setattr__(self, "policy_hash", expected_hash)

    @property
    def half_width(self) -> float:
        return float(min(-np.min(self.minimum), np.min(self.maximum)))

    @property
    def is_periodic(self) -> bool:
        return bool(self.is_closed and self.distance.size > 1 and self.distance[-1] > self.distance[0]
                    and np.isclose(self.minimum[0], self.minimum[-1], rtol=0.0, atol=1e-12)
                    and np.isclose(self.maximum[0], self.maximum[-1], rtol=0.0, atol=1e-12))

    @property
    def is_conditional(self) -> bool:
        return self.provenance != "surveyed"


def corridor_from_track(
    track, *, margin: float = 0.0, fold_guard: float = 0.2,
    half_width: float | None = None, policy: str = LEGACY_POLICY, reference=None,
) -> Corridor:
    """Build corridor bounds; signed/fitted behavior requires explicit opt-in.

    ``signed-fitted-reference-v1`` applies the fold guard only at sampled
    corridor stations. It is not a continuous normal-chart certificate.
    """
    if not isinstance(policy, str) or policy not in (LEGACY_POLICY, SIGNED_POLICY):
        raise ValueError("unsupported corridor policy")
    for name, value in (("margin", margin), ("fold_guard", fold_guard)):
        if isinstance(value, (bool, np.bool_)) or not np.isscalar(value):
            raise TypeError(f"{name} must be a real number")
        if not np.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if margin < 0.0:
        raise ValueError("margin must be finite and nonnegative")
    if not 0.0 < fold_guard < 1.0:
        raise ValueError("fold_guard must lie strictly between zero and one")
    if policy == SIGNED_POLICY and reference is None:
        raise ValueError("signed fitted policy requires a ReferenceCurve")
    if policy == LEGACY_POLICY and reference is not None:
        raise ValueError("reference is only valid for signed fitted policy")

    has_edges = bool(getattr(track, "has_edges", False))
    if has_edges and half_width is not None:
        raise ValueError("half_width is only valid for a track without edges")
    if not has_edges and half_width is None:
        raise ValueError("the track carries no lateral corridor; provide half_width")
    distance = np.asarray(track.distance, dtype=float)
    if has_edges:
        left, right = np.asarray(track.half_width_left, dtype=float), np.asarray(track.half_width_right, dtype=float)
        provenance = str(track.width_provenance)
        width_input = None
    else:
        if isinstance(half_width, (bool, np.bool_)) or not np.isfinite(half_width) or half_width <= 0.0:
            raise ValueError("half_width must be finite and positive")
        left = np.full(distance.shape, float(half_width)); right = left.copy()
        provenance, width_input = "assumed", float(half_width)
    source_k = np.asarray(track.curvature, dtype=float)
    if not all(np.all(np.isfinite(a)) for a in (distance, left, right, source_k)):
        raise ValueError("track corridor inputs must be finite")
    if left.shape != distance.shape or right.shape != distance.shape or source_k.shape != distance.shape:
        raise ValueError("track corridor inputs must match the distance grid")

    minimum, maximum = -left + margin, right - margin
    if policy == LEGACY_POLICY:
        magnitude = np.abs(source_k)
        limit = np.full(magnitude.shape, np.inf)
        curved = magnitude > 0.0
        limit[curved] = (1.0 - fold_guard) / magnitude[curved]
        maximum, minimum = np.minimum(maximum, limit), np.maximum(minimum, -limit)
        recipe = None
    else:
        try:
            from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
            if not isinstance(reference, ReferenceCurve):
                raise ValueError("signed fitted policy requires an actual ReferenceCurve")
            reference_station = np.asarray(reference.source_station, dtype=float)
            if reference_station.ndim != 1 or reference_station.size < 2 or np.any(~np.isfinite(reference_station)):
                raise ValueError("invalid reference station domain")
            if reference_station.shape != distance.shape or not np.allclose(reference_station, distance, rtol=0, atol=1e-10):
                raise ValueError("reference domain does not match corridor distance")
            evaluation = reference.evaluate(distance)
            fitted_k = np.asarray(evaluation.curvature, dtype=float)
            if fitted_k.shape != distance.shape or np.any(~np.isfinite(fitted_k)):
                raise ValueError("reference curvature must be finite and match corridor distance")
        except Exception as error:
            if isinstance(error, ValueError): raise
            raise ValueError("reference evaluation failed") from error
        right_turn = fitted_k > 0.0
        left_turn = fitted_k < 0.0
        maximum[right_turn] = np.minimum(maximum[right_turn], (1.0-fold_guard)/fitted_k[right_turn])
        minimum[left_turn] = np.maximum(minimum[left_turn], (1.0-fold_guard)/fitted_k[left_turn])
        fit_digest = hashlib.sha256()
        for label, spline in ((b"x", reference._x), (b"y", reference._y)):
            fit_digest.update(label)
            fit_digest.update(np.asarray(spline.t, dtype="<f8").tobytes())
            fit_digest.update(np.asarray(spline.c, dtype="<f8").tobytes())
            fit_digest.update(str(int(spline.k)).encode("ascii"))
        recipe = {"kind": "ReferenceCurve", "start": float(reference.start), "end": float(reference.end),
                  "is_closed": bool(reference.is_closed), "interpolation_periodic": bool(reference.interpolation_periodic),
                  "fit_sha256": fit_digest.hexdigest()}
    if np.any(minimum > 0.0) or np.any(maximum < 0.0): raise ValueError("the corridor does not contain the centreline")
    if np.any(minimum > maximum): raise ValueError("the corridor is empty")
    descriptor = {
        "schema": "corridor-policy-identity-v1", "policy": policy, "fold_guard": float(fold_guard),
        "margin": float(margin), "half_width_input": width_input, "provenance": provenance,
        "is_conditional": provenance != "surveyed", "distance": distance.tolist(),
        "minimum": minimum.tolist(), "maximum": maximum.tolist(), "is_closed": bool(track.is_closed),
        "reference_recipe": recipe,
    }
    payload = json.dumps(descriptor, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return Corridor(distance, minimum, maximum, provenance, bool(track.is_closed), policy,
                    _freeze(descriptor), hashlib.sha256(payload).hexdigest())
