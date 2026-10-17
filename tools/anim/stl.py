"""STL reading and the placeholder meshes of the vehicle scene (NumPy only).

An STL is a *display asset*. It never enters the physics and never changes a
simulation number. Triangles are returned in the file's own coordinates; the
caller applies one scale and one translation (see ``scene.transform``).
"""
from __future__ import annotations

from pathlib import Path
import re

import numpy as np

__all__ = ["StlError", "read_stl", "bounding_box", "box_triangles", "wheel_cylinder_triangles"]

_BINARY_HEADER = 80
_BINARY_RECORD = np.dtype([("normal", "<f4", (3,)), ("vertex", "<f4", (3, 3)),
                           ("attribute", "<u2")])
_VERTEX = re.compile(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", re.IGNORECASE)
_FACET = re.compile(rb"(?<!end)facet\s+normal", re.IGNORECASE)
_END_FACET = re.compile(rb"endfacet", re.IGNORECASE)
_END_SOLID = re.compile(rb"endsolid", re.IGNORECASE)


class StlError(ValueError):
    """The file is not a readable STL."""


def _parse_ascii(data: bytes) -> np.ndarray | None:
    """Return the triangles of an ASCII STL, or ``None`` when it is not one."""

    if not data.lstrip()[:5].lower() == b"solid":
        return None
    facets = len(_FACET.findall(data))
    vertices = _VERTEX.findall(data)
    complete = (len(_END_FACET.findall(data)) == facets
                and _END_SOLID.search(data) is not None)
    if facets == 0 or len(vertices) != 3 * facets or not complete:
        return None  # a truncated file, or a binary file that begins with "solid"
    try:
        values = np.array([[float(token) for token in row] for row in vertices])
    except ValueError:
        return None
    return values.reshape(facets, 3, 3)


def _parse_binary(data: bytes) -> np.ndarray:
    if len(data) < _BINARY_HEADER + 4:
        raise StlError("binary STL is shorter than its header")
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=_BINARY_HEADER)[0])
    expected = _BINARY_HEADER + 4 + count * _BINARY_RECORD.itemsize
    if len(data) != expected:
        raise StlError(f"binary STL declares {count} triangles ({expected} bytes) "
                       f"but holds {len(data)} bytes")
    records = np.frombuffer(data, dtype=_BINARY_RECORD, count=count, offset=_BINARY_HEADER + 4)
    if not np.all(np.isfinite(records["normal"])):
        raise StlError("binary STL holds a non-finite normal")
    if not np.all(np.isfinite(records["vertex"])):
        raise StlError("binary STL holds a non-finite vertex")
    return records["vertex"].astype(np.float64)


def read_stl(path) -> np.ndarray:
    """Read an ASCII or binary STL into an ``(n, 3, 3)`` float64 triangle array."""

    data = Path(path).read_bytes()
    triangles = _parse_ascii(data)
    if triangles is None:
        triangles = _parse_binary(data)
    if triangles.shape[0] == 0:
        raise StlError("the STL holds no triangles")
    if not np.all(np.isfinite(triangles)):
        raise StlError("the STL holds a non-finite vertex")
    return triangles


def bounding_box(triangles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return the ``(minimum, maximum)`` corner of a triangle array."""

    points = np.asarray(triangles, dtype=float).reshape(-1, 3)
    return points.min(axis=0), points.max(axis=0)


# Corner order of the unit cube: bit 0 is x, bit 1 is y, bit 2 is z.
_CUBE_FACES = (
    (0, 2, 3, 1), (4, 5, 7, 6),  # z = -1, z = +1
    (0, 1, 5, 4), (2, 6, 7, 3),  # y = -1, y = +1
    (0, 4, 6, 2), (1, 3, 7, 5),  # x = -1, x = +1
)


def box_triangles(sizes: tuple[float, float, float]) -> np.ndarray:
    """Return the 12 triangles of an axis-aligned box centred on the origin."""

    sizes = np.asarray(sizes, dtype=float)
    if sizes.shape != (3,) or not np.all(np.isfinite(sizes)) or np.any(sizes <= 0.0):
        raise ValueError("box sizes must be three positive finite lengths")
    corners = np.array([[(index >> axis) & 1 for axis in range(3)] for index in range(8)],
                       dtype=float)
    corners = (corners - 0.5) * sizes
    triangles = []
    for a, b, c, d in _CUBE_FACES:
        triangles.append(corners[[a, b, c]])
        triangles.append(corners[[a, c, d]])
    return np.array(triangles)


def wheel_cylinder_triangles(radius: float, width: float, segments: int = 24) -> np.ndarray:
    """Return a closed cylinder centred on the origin with its axis along ``+y``."""

    if not (np.isfinite(radius) and radius > 0.0 and np.isfinite(width) and width > 0.0):
        raise ValueError("radius and width must be positive and finite")
    if segments < 3:
        raise ValueError("a cylinder needs at least three segments")
    angle = 2.0 * np.pi * np.arange(segments) / segments
    x, z = radius * np.cos(angle), radius * np.sin(angle)
    left = np.column_stack((x, np.full(segments, -width / 2.0), z))
    right = np.column_stack((x, np.full(segments, width / 2.0), z))
    centre_left = np.array([0.0, -width / 2.0, 0.0])
    centre_right = np.array([0.0, width / 2.0, 0.0])
    triangles = []
    for k in range(segments):
        n = (k + 1) % segments
        triangles.append([left[k], right[k], right[n]])
        triangles.append([left[k], right[n], left[n]])
        # Angle grows from +x towards +z, which is right-handed about +y, so the
        # outward cap normals (-y left, +y right) need these windings.
        triangles.append([centre_left, left[k], left[n]])
        triangles.append([centre_right, right[n], right[k]])
    return np.array(triangles)
