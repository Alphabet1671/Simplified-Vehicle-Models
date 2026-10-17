"""Orthographic projection of the vehicle scene (NumPy only, no matplotlib).

Body axes follow SAE J670: ``+x`` forward, ``+y`` right, ``+z`` down. World axes
are ``X`` north, ``Y`` east, ``Z`` down. Yaw is positive clockwise seen from above,
roll positive right side down, pitch positive nose up (the 14-DoF convention).

Every view is the same operation: express the screen ``right`` and ``up`` unit
vectors in the frame of the points and take two dot products. ``right`` is the
camera's forward direction crossed with ``up``, so the picture is never mirrored.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["VIEWS", "VehicleScene", "body_rotation", "transform", "project"]


def _unit(*components: float) -> np.ndarray:
    vector = np.array(components, dtype=float)
    vector.setflags(write=False)
    return vector


#: ``(right, up)`` screen axes of each view, in the frame of the points drawn.
#: ``rear``: camera behind the car looking forward. ``right``: camera on the
#: right side looking left. ``top``: camera above looking down, body ``+x`` up.
#: ``track``: a world map seen from above with north up and east to the right.
VIEWS = {
    "rear": (_unit(0, 1, 0), _unit(0, 0, -1)),
    "right": (_unit(1, 0, 0), _unit(0, 0, -1)),
    "top": (_unit(0, 1, 0), _unit(1, 0, 0)),
    "track": (_unit(0, 1, 0), _unit(1, 0, 0)),
}


@dataclass(frozen=True)
class VehicleScene:
    """One rigid body drawn in body axes, in metres.

    ``triangles`` is the sprung body ``(n, 3, 3)``. ``wheels`` holds, per wheel in
    the order FL, FR, RL, RR, its triangles about the wheel centre and the wheel
    centre in body axes.
    """

    triangles: np.ndarray
    wheels: tuple[tuple[np.ndarray, np.ndarray], ...] = ()


def body_rotation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Return the body-to-parent rotation: yaw about ``+z``, then pitch about ``+y``,
    then roll about ``+x`` (``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``)."""

    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def transform(triangles: np.ndarray, rotation: np.ndarray, translation=(0.0, 0.0, 0.0),
              scale: float = 1.0) -> np.ndarray:
    """Return ``scale * R @ p + t`` for every vertex of an ``(n, 3, 3)`` array."""

    points = np.asarray(triangles, dtype=float)
    return scale * points @ np.asarray(rotation, dtype=float).T + np.asarray(translation, float)


def project(triangles: np.ndarray, right, up) -> np.ndarray:
    """Project ``(..., 3)`` points onto the screen: returns ``(..., 2)`` of
    ``(right, up)`` coordinates. ``right`` and ``up`` must be orthonormal."""

    right, up = np.asarray(right, dtype=float), np.asarray(up, dtype=float)
    if (not np.isclose(np.linalg.norm(right), 1.0) or not np.isclose(np.linalg.norm(up), 1.0)
            or abs(float(right @ up)) > 1.e-12):
        raise ValueError("right and up must be orthonormal unit vectors")
    points = np.asarray(triangles, dtype=float)
    return np.stack((points @ right, points @ up), axis=-1)
