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

from tools.anim.stl import box_triangles, wheel_cylinder_triangles

__all__ = ["VIEWS", "VehicleScene", "VehicleEdges", "body_rotation", "transform", "project",
           "feature_edges", "build_vehicle_edges", "placeholder_scene"]


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


def feature_edges(triangles: np.ndarray, angle_degrees: float = 30.0) -> np.ndarray:
    """Return the edges worth drawing as ``(m, 2, 3)`` segments.

    An edge is kept when it bounds the surface (one adjacent triangle), is shared by
    more than two triangles, or joins two triangles whose normals differ by more
    than ``angle_degrees``. A box therefore draws its 12 edges without the
    triangulation diagonals, and a cylinder draws its two rims.
    """

    tri = np.asarray(triangles, dtype=float)
    if tri.ndim != 3 or tri.shape[1:] != (3, 3):
        raise ValueError("triangles must have shape (n, 3, 3)")
    count = tri.shape[0]
    if count == 0:
        return np.zeros((0, 2, 3))
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(normals, axis=1)
    live = length > 0.0
    normals[live] /= length[live, None]
    points = tri.reshape(-1, 3)
    _, first, inverse = np.unique(np.round(points, 9), axis=0, return_index=True,
                                  return_inverse=True)
    vertices = points[first]
    ids = inverse.reshape(count, 3)
    pairs = np.concatenate([ids[:, [0, 1]], ids[:, [1, 2]], ids[:, [2, 0]]])
    face = np.tile(np.arange(count), 3)
    pairs = np.sort(pairs, axis=1)
    key = pairs[:, 0].astype(np.int64) * vertices.shape[0] + pairs[:, 1]
    order = np.argsort(key, kind="stable")
    key, pairs, face = key[order], pairs[order], face[order]
    starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
    sizes = np.diff(np.r_[starts, key.size])
    keep = sizes != 2
    pair_rows = np.flatnonzero(sizes == 2)
    if pair_rows.size:
        a, b = face[starts[pair_rows]], face[starts[pair_rows] + 1]
        sharp = np.einsum("ij,ij->i", normals[a], normals[b]) < np.cos(np.radians(angle_degrees))
        keep[pair_rows[sharp]] = True
    chosen = pairs[starts[keep]]
    return vertices[chosen]


@dataclass(frozen=True)
class VehicleEdges:
    """Outline segments of a ``VehicleScene``, with the wheels posable.

    ``body`` is ``(m, 2, 3)`` in body axes. Each wheel holds its edges about the
    wheel centre, the centre in body axes, and its radius about the ``y`` axis.
    """

    body: np.ndarray
    wheel_edges: tuple[np.ndarray, ...]
    wheel_centres: np.ndarray
    wheel_radii: np.ndarray

    @property
    def wheel_count(self) -> int:
        return len(self.wheel_edges)

    def extent(self) -> float:
        """Largest body-axis distance of any vertex from the CG, in the ``xy`` plane."""

        pieces = [self.body.reshape(-1, 3)]
        for edges, centre in zip(self.wheel_edges, self.wheel_centres):
            pieces.append(edges.reshape(-1, 3) + centre)
        points = np.concatenate(pieces)
        return float(np.max(np.hypot(points[:, 0], points[:, 1])))

    def posed(self, steering=None, spin=None) -> np.ndarray:
        """Body-axis segments with every wheel steered about ``z`` and spun about ``y``.

        ``steering`` and ``spin`` are per-wheel angles in radians (or ``None``). Each
        wheel adds one spoke from its centre to the top of the rim, so the spin
        is visible in a side view. The segment count does not depend on the angles.
        """

        pieces = [self.body]
        for k, (edges, centre, radius) in enumerate(
                zip(self.wheel_edges, self.wheel_centres, self.wheel_radii)):
            steer = 0.0 if steering is None else float(steering[k])
            turn = 0.0 if spin is None else float(spin[k])
            rotation = body_rotation(steer, turn, 0.0)
            spoke = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, -radius]]])
            local = np.concatenate([edges, spoke]) if edges.size else spoke
            pieces.append(local @ rotation.T + centre)
        return np.concatenate(pieces)


def build_vehicle_edges(scene: VehicleScene, angle_degrees: float = 30.0) -> VehicleEdges:
    body = feature_edges(scene.triangles, angle_degrees)
    edges, centres, radii = [], [], []
    for triangles, centre in scene.wheels:
        triangles = np.asarray(triangles, dtype=float)
        edges.append(feature_edges(triangles, angle_degrees))
        centres.append(np.asarray(centre, dtype=float))
        points = triangles.reshape(-1, 3)
        radii.append(float(np.max(np.hypot(points[:, 0], points[:, 2]))) if points.size else 0.0)
    return VehicleEdges(body, tuple(edges), np.array(centres).reshape(-1, 3), np.array(radii))


def default_body_size(geometry: dict, wheel_width: float = 0.18) -> tuple[float, float, float]:
    """Return the placeholder body block ``(length, width, height)`` for a geometry."""

    positions = np.asarray(geometry["wheel_positions_body_m"], dtype=float)
    wheelbase = float(geometry.get("wheelbase_m", np.ptp(positions[:, 0])))
    track_width = float(geometry.get("track_width_m", np.ptp(positions[:, 1])))
    return (wheelbase + 0.9, max(track_width - wheel_width, 0.2), 0.25)


def placeholder_scene(geometry: dict, *, body_size=None, wheel_radius: float = 0.2,
                      wheel_width: float = 0.18) -> VehicleScene:
    """A box body with four cylinder wheels, from the record's ``vehicle_geometry``.

    Wheel centres come from ``wheel_positions_body_m`` (FL, FR, RL, RR) and sit one
    wheel radius above the ground, which is ``cg_height_m`` below the CG. The body
    box is centred on the CG. ``body_size`` is ``(length, width, height)``; the
    default is a block a little longer than the wheelbase and between the wheels.
    """

    positions = np.asarray(geometry["wheel_positions_body_m"], dtype=float)
    if positions.shape != (4, 2):
        raise ValueError("wheel_positions_body_m must be four (x, y) pairs")
    if body_size is None:
        body_size = default_body_size(geometry, wheel_width)
    z_wheel = float(geometry["cg_height_m"]) - float(wheel_radius)
    cylinder = wheel_cylinder_triangles(wheel_radius, wheel_width)
    wheels = tuple((cylinder, np.array([x, y, z_wheel])) for x, y in positions)
    return VehicleScene(box_triangles(body_size), wheels)
