"""Corner-table loader of the vertical (14DoF) rung.

The ``suspension-corner-v1`` NPZ holds the per-axle spring curve, damper motion
ratio, roll-centre table, anti geometry, camber gain, jounce steer, travel
limits and sprung static load. This module loads it and builds the two parts
that read it: the axle geometry (``kinematics.AxleGeometry``) and the spring
curve of the force network. The force law itself lives in
``suspension/layouts``.

Sign conventions:

* A positive spring, ARB, or damper force is a compression force. It pushes the
  wheel down and the body up.
* The spring curve already contains the sprung static corner load at jounce 0:
  zero jounce is the static equilibrium at the design ride height.
* Toe angles follow the repository's positive-right road-wheel convention. A
  toe-in on the left wheel is a positive angle. A toe-in on the right wheel is a
  negative angle.

The NPZ file is written by ``tools/model_data/generate_suspension_lut.py`` and
read here by ``load_corner_arrays``.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from simplified_vehicle_models.suspension.kinematics import PRELOAD_REFERENCE, AxleGeometry
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

SCHEMA = "suspension-corner-v1"
__all__ = (
    "PRELOAD_REFERENCE", "SCHEMA", "geometry_from_arrays", "load_corner_arrays",
    "spring_from_arrays",
)

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


def _check_axle(axle: str) -> None:
    if axle not in ("front", "rear"):
        raise ValueError("axle must be 'front' or 'rear'")


def geometry_from_arrays(arrays: dict, axle: str, *, static_toe: float) -> AxleGeometry:
    """Build the geometry of one axle from loaded NPZ arrays and the static toe."""

    _check_axle(axle)

    def get(name):
        return arrays[f"{name}_{axle}"]

    limits = np.asarray(get("jounce_limits"), dtype=float)
    if limits.shape != (2,):
        raise ValueError("jounce_limits must hold two values")
    return AxleGeometry(
        motion_ratio=MonotoneSpline(get("motion_ratio_jounce"), get("motion_ratio")),
        roll_centre_height=MonotoneSpline(get("rc_roll"), get("rc_height")),
        roll_centre_lateral=MonotoneSpline(get("rc_roll"), get("rc_lateral")),
        anti_brake=float(get("anti_brake")),
        anti_drive=float(get("anti_drive")),
        camber_gain=float(get("camber_gain")),
        static_toe=float(static_toe),
        jounce_steer=MonotoneSpline(get("jounce_steer_jounce"), get("jounce_steer_toe")),
        jounce_limits=(float(limits[0]), float(limits[1])),
        static_sprung_load=float(get("static_sprung_load")),
    )


def spring_from_arrays(arrays: dict, axle: str) -> MonotoneSpline:
    """Build the wheel-referred spring curve (force against jounce) of one axle."""

    _check_axle(axle)
    return MonotoneSpline(arrays[f"spring_jounce_{axle}"], arrays[f"spring_force_{axle}"])
