"""Corner suspension model of the vertical (14DoF) rung.

One ``AxleSuspension`` holds the geometry and force curves of one axle. A
``SuspensionModel`` pairs the front and the rear axle. All arrays use the wheel
order ``(FL, FR, RL, RR)``; jounce is positive in compression.

Sign conventions:

* A positive spring, ARB, or damper force is a compression force. It pushes the
  wheel down and the body up.
* The spring curve already contains the sprung static corner load at jounce 0:
  zero jounce is the static equilibrium at the design ride height.
* Toe angles follow the repository's positive-right road-wheel convention. A
  toe-in on the left wheel is a positive angle. A toe-in on the right wheel is a
  negative angle.

The NPZ file ``suspension-corner-v1`` is written by
``tools/model_data/generate_suspension_lut.py`` and read here by
``load_corner_arrays``.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

SCHEMA = "suspension-corner-v1"
PRELOAD_REFERENCE = "static-equilibrium"

_AXLE_ARRAYS = (
    "spring_jounce",
    "spring_force",
    "motion_ratio_jounce",
    "motion_ratio",
    "rc_roll",
    "rc_height",
    "rc_lateral",
    "jounce_steer_jounce",
    "jounce_steer_toe",
    "jounce_limits",
    "anti_brake",
    "anti_drive",
    "camber_gain",
    "static_sprung_load",
)
_REQUIRED_KEYS = (
    "schema",
    "generator_version",
    "source_sha256",
    "metadata",
    *(f"{name}_{axle}" for name in _AXLE_ARRAYS for axle in ("front", "rear")),
)


@dataclass(frozen=True)
class AxleSuspension:
    """Geometry and force curves of one axle."""

    wheel_rate_spring: MonotoneSpline
    arb_wheel_rate: float
    motion_ratio_damper: MonotoneSpline
    roll_centre_height: MonotoneSpline
    roll_centre_lateral: MonotoneSpline
    anti_brake: float
    anti_drive: float
    camber_gain: float
    static_toe: float
    jounce_steer: MonotoneSpline
    inertance: float
    jounce_limits: tuple[float, float]

    def __post_init__(self) -> None:
        low, high = self.jounce_limits
        if not (np.isfinite(low) and np.isfinite(high) and low < 0.0 < high):
            raise ValueError("jounce_limits must be finite with low < 0 < high")
        for name in ("arb_wheel_rate", "inertance"):
            value = getattr(self, name)
            if not (np.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be finite and nonnegative")
        for name in ("anti_brake", "anti_drive"):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be a fraction from zero through one")
        for name in ("camber_gain", "static_toe"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        for name in ("wheel_rate_spring", "motion_ratio_damper", "jounce_steer"):
            spline = getattr(self, name)
            if spline.x[0] != low or spline.x[-1] != high:
                raise ValueError(f"{name} must span exactly the jounce limits")
        steer = self.jounce_steer
        index = np.flatnonzero(steer.x == 0.0)
        if index.size != 1 or steer.y[index[0]] != 0.0:
            raise ValueError("jounce_steer needs an explicit 0 m knot with 0 rad toe change")
        if np.any(self.motion_ratio_damper.y <= 0.0):
            raise ValueError("damper motion ratio must be positive")
        for name in ("roll_centre_height", "roll_centre_lateral"):
            if getattr(self, name).x[0] != 0.0:
                raise ValueError(f"{name} must start at zero roll")


@dataclass(frozen=True)
class SuspensionModel:
    """Front and rear axle suspension."""

    front: AxleSuspension
    rear: AxleSuspension
    preload_reference: str = PRELOAD_REFERENCE


@dataclass(frozen=True)
class CornerForces:
    """Per-wheel force breakdown, order FL, FR, RL, RR (N, positive in compression)."""

    spring: np.ndarray
    arb: np.ndarray
    damper: np.ndarray
    total: np.ndarray
    damper_velocity: np.ndarray  # m/s at the damper, positive in compression
    out_of_range: np.ndarray     # bool, damper velocity beyond its table


def _four(name: str, value) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (4,):
        raise ValueError(f"{name} must hold four values in FL, FR, RL, RR order")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return array


def corner_forces(model: SuspensionModel, jounce, jounce_rate, dampers) -> CornerForces:
    """Return the wheel-referred spring, ARB, and damper forces of four corners.

    ``dampers`` is ``(front, rear)``, each with ``force(velocity)``. A jounce
    outside the DSS design travel raises ``DomainError``.
    """

    z = _four("jounce", jounce)
    zdot = _four("jounce_rate", jounce_rate)
    if len(dampers) != 2:
        raise ValueError("dampers must be (front, rear)")
    spring = np.empty(4)
    arb = np.empty(4)
    damper = np.empty(4)
    velocity = np.empty(4)
    flag = np.zeros(4, dtype=bool)
    for axle, damper_model, (left, right) in (
        (model.front, dampers[0], (0, 1)),
        (model.rear, dampers[1], (2, 3)),
    ):
        pair = [left, right]
        zz = z[pair]
        spring[pair] = axle.wheel_rate_spring(zz)
        half = 0.5 * axle.arb_wheel_rate * (zz[0] - zz[1])
        arb[left], arb[right] = half, -half
        ratio = axle.motion_ratio_damper(zz)
        v = zdot[pair] / ratio
        result = damper_model.force(v)
        velocity[pair] = v
        damper[pair] = np.asarray(result.force, dtype=float) / ratio
        flag[pair] = np.asarray(result.out_of_range, dtype=bool)
    return CornerForces(
        spring=spring,
        arb=arb,
        damper=damper,
        total=spring + arb + damper,
        damper_velocity=velocity,
        out_of_range=flag,
    )


def corner_toe_angles(model: SuspensionModel, jounce) -> np.ndarray:
    """Return static toe plus jounce steer as four road-wheel angle offsets (rad).

    Positive-right convention: FL/RL = +(toe + steer(z)); FR/RR = -(toe + steer(z)).
    """

    z = _four("jounce", jounce)
    angles = np.empty(4)
    for axle, left, right in ((model.front, 0, 1), (model.rear, 2, 3)):
        angles[left] = axle.static_toe + float(axle.jounce_steer(z[left]))
        angles[right] = -(axle.static_toe + float(axle.jounce_steer(z[right])))
    return angles


def load_corner_arrays(path, expected_sha256: str) -> dict:
    """Load a ``suspension-corner-v1`` NPZ after checking its SHA-256."""

    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != str(expected_sha256).strip().lower():
        raise ValueError(f"suspension corner table sha256 mismatch for {path}: got {digest}")
    with np.load(path, allow_pickle=False) as data:
        missing = [key for key in _REQUIRED_KEYS if key not in data.files]
        if missing:
            raise ValueError(f"suspension corner table {path} is missing keys {missing}")
        arrays = {key: data[key] for key in _REQUIRED_KEYS}
    if str(arrays["schema"]) != SCHEMA:
        raise ValueError(f"unsupported suspension corner schema {arrays['schema']!s}")
    try:
        arrays["metadata"] = json.loads(str(arrays["metadata"]))
    except json.JSONDecodeError as error:
        raise ValueError("suspension corner metadata must be a JSON string") from error
    arrays["file_sha256"] = digest
    return arrays


def axle_from_arrays(
    arrays: dict,
    axle: str,
    *,
    arb_wheel_rate: float,
    inertance: float,
    static_toe: float,
) -> AxleSuspension:
    """Build one axle from loaded NPZ arrays and the XML scalars."""

    if axle not in ("front", "rear"):
        raise ValueError("axle must be 'front' or 'rear'")

    def get(name):
        return arrays[f"{name}_{axle}"]

    limits = np.asarray(get("jounce_limits"), dtype=float)
    if limits.shape != (2,):
        raise ValueError("jounce_limits must hold two values")
    return AxleSuspension(
        wheel_rate_spring=MonotoneSpline(get("spring_jounce"), get("spring_force")),
        arb_wheel_rate=float(arb_wheel_rate),
        motion_ratio_damper=MonotoneSpline(get("motion_ratio_jounce"), get("motion_ratio")),
        roll_centre_height=MonotoneSpline(get("rc_roll"), get("rc_height")),
        roll_centre_lateral=MonotoneSpline(get("rc_roll"), get("rc_lateral")),
        anti_brake=float(get("anti_brake")),
        anti_drive=float(get("anti_drive")),
        camber_gain=float(get("camber_gain")),
        static_toe=float(static_toe),
        jounce_steer=MonotoneSpline(get("jounce_steer_jounce"), get("jounce_steer_toe")),
        inertance=float(inertance),
        jounce_limits=(float(limits[0]), float(limits[1])),
    )
