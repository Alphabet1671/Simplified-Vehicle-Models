"""Strict reader and exact reference bridge for analytic trajectory artifacts.

Artifacts reproduce an analytic path/state; they do not establish feasibility or
vehicle tracking performance.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from types import MappingProxyType, SimpleNamespace
from typing import Mapping

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline

_SCHEMA = 1
_CONTROLS = {"quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1"}
_TIME = "gauss_legendre_16_energy_hermite_knot_union_v1"


def _ro(value):
    out = np.array(value, dtype=float, copy=True)
    out.setflags(write=False)
    return out


def _readonly(value):
    if isinstance(value, dict):
        return MappingProxyType({k: _readonly(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_readonly(v) for v in value)
    return value


@dataclass(frozen=True, slots=True)
class LoadedTrajectory:
    trajectory: object
    state_evaluator: object
    node_station: np.ndarray
    control_repr: str
    state_repr: str
    time_quadrature: str
    metadata: Mapping
    source_sha256: str
    physical_time: float | None

    @property
    def accepted(self):
        """Whether an independent physical/OCP validation has been performed."""
        return False

    @property
    def physical_validated(self):
        return False

    @property
    def physically_validated(self):
        return False

    def evaluate_reference(self, station):
        """Evaluate analytic path geometry and represented state at reference s."""
        geometry = self.trajectory.evaluate(station)
        state = self.state_evaluator.evaluate(station)
        return {"north": geometry.north, "east": geometry.east,
                "heading": geometry.heading, "curvature": geometry.curvature,
                "source_station": station, **state}

    def _reference_station_for_path(self, ell):
        return self.trajectory.station_map.to_reference(ell)

    def evaluate_path(self, ell):
        """Evaluate geometry and represented state at physical path station."""
        source = self._reference_station_for_path(ell)
        result = self.evaluate_reference(source)
        result["path_station"] = ell
        return result

    def sample_path_reference(self, ell):
        return self.evaluate_path(ell)


class ExactTrajectoryDriverReference:
    """Plan-like preview interface exposing stored physical acceleration exactly.

    This is a reference signal only. Using it with PreviewDriver is not evidence
    of tracking, OCP feasibility, or a validated vehicle model.
    """
    def __init__(self, loaded: LoadedTrajectory):
        self.loaded = loaded
        self.is_closed = loaded.trajectory.reference.is_closed
        node_path = np.asarray(loaded.trajectory.station_map.to_path(loaded.node_station), dtype=float)
        self.length = float(loaded.trajectory.station_map.path_end -
                            loaded.trajectory.station_map.path_start)
        self.distance = _ro(node_path - node_path[0])
        self.speed = _ro(loaded.state_evaluator.speed)
        self.curvature = _ro(loaded.trajectory.evaluate(loaded.node_station).curvature)
        self.acceleration = _ro(loaded.state_evaluator.acceleration)
        if not (self.distance.shape == self.speed.shape == self.curvature.shape == self.acceleration.shape):
            raise ValueError("driver node arrays must have matching shapes")
        if not np.isclose(self.distance[-1], self.length, rtol=0.0, atol=1e-7):
            raise ValueError("driver node grid does not cover physical path length")

    def sample(self, distance):
        d = float(distance)
        if not np.isfinite(d):
            raise ValueError("path distance must be finite")
        if self.is_closed:
            # Preserve the explicitly represented terminal endpoint; other
            # out-of-range stations wrap by one physical lap.
            if d != self.length:
                d %= self.length
        elif d < 0.0 or d > self.length:
            raise ValueError("path distance outside open trajectory")
        source = self.loaded.trajectory.station_map.to_reference(
            d + self.loaded.trajectory.station_map.path_start)
        state = self.loaded.state_evaluator.evaluate(source)
        geometry = self.loaded.trajectory.evaluate(source)
        return float(state["speed"]), float(geometry.curvature), float(state["acceleration"])

    def sample_geometry(self, distance):
        d = float(distance)
        if not np.isfinite(d):
            raise ValueError("path distance must be finite")
        if not self.is_closed and (d < 0.0 or d > self.length):
            raise ValueError("path distance outside open trajectory")
        if self.is_closed and d != self.length:
            d %= self.length
        return self.loaded.sample_path_reference(
            d + self.loaded.trajectory.station_map.path_start)


def load_trajectory_artifact(path) -> LoadedTrajectory:
    path = Path(path)
    raw = path.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    try:
        with np.load(path, allow_pickle=False) as archive:
            required = {"metadata", "source_reference_station", "reference_north", "reference_east",
                        "map_reference_station", "path_station", "control_station", "coefficients",
                        "node_station", "node_speed", "node_acceleration", "midpoint_acceleration"}
            if set(archive.files) != required:
                raise ValueError("trajectory artifact fields do not match schema")
            metadata = json.loads(str(archive["metadata"].item()))
            arrays = {name: np.array(archive[name], dtype=float, copy=True) for name in required - {"metadata"}}
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("trajectory artifact"):
            raise
        raise ValueError(f"invalid trajectory artifact: {exc}") from exc
    if metadata.get("schema_version") != _SCHEMA:
        raise ValueError("unknown trajectory artifact schema")
    state_repr = metadata.get("state_representation")
    if state_repr != "energy_hs_v1":
        raise ValueError("unsupported state representation")
    control_repr = metadata.get("control_representation", "quadratic_acceleration_v1")
    if control_repr not in _CONTROLS:
        raise ValueError("unsupported control representation")
    time_name = metadata.get("time_quadrature")
    if time_name != _TIME:
        raise ValueError("unsupported or unspecified time quadrature")
    if metadata.get("reference_interpolant") != "scipy.make_interp_spline":
        raise ValueError("unsupported reference interpolant")
    if metadata.get("reference_degree") not in (3, 5):
        raise ValueError("unsupported reference degree")
    for name, values in arrays.items():
        if values.ndim != 1 or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must be finite one-dimensional data")
    s, north, east = (arrays[k] for k in ("source_reference_station", "reference_north", "reference_east"))
    mrs, ps = arrays["map_reference_station"], arrays["path_station"]
    cs, coeff = arrays["control_station"], arrays["coefficients"]
    nodes, speed, acc, mid = (arrays[k] for k in ("node_station", "node_speed", "node_acceleration", "midpoint_acceleration"))
    if s.size < 4 or north.shape != s.shape or east.shape != s.shape or np.any(np.diff(s) <= 0):
        raise ValueError("invalid source reference grid")
    if mrs.shape != ps.shape or mrs.size < 2 or np.any(np.diff(mrs) <= 0) or np.any(np.diff(ps) <= 0):
        raise ValueError("invalid stored station map")
    if cs.shape != coeff.shape or cs.size < 4:
        raise ValueError("invalid offset controls")
    if nodes.size < 2 or speed.shape != nodes.shape or acc.shape != nodes.shape or mid.size != nodes.size - 1:
        raise ValueError("invalid state mesh shapes")
    if np.any(np.diff(nodes) <= 0) or np.any(speed <= 0):
        raise ValueError("node station must increase and speed must be positive")
    if not np.isclose(nodes[0], s[0], rtol=0.0, atol=1e-9) or not np.isclose(nodes[-1], s[-1], rtol=0.0, atol=1e-9):
        raise ValueError("state mesh must cover the full reference interval")
    closed = metadata.get("is_closed")
    if not isinstance(closed, bool):
        raise ValueError("is_closed must be boolean")
    track = SimpleNamespace(path_coordinates=SimpleNamespace(distance=s, x_north=north, y_east=east, is_closed=closed),
                            lap_length=float(s[-1] - s[0]), is_closed=closed)
    ref = ReferenceCurve(track)
    if metadata["reference_degree"] != (5 if s.size >= 6 else 3):
        raise ValueError("reference degree does not match source grid")
    spacing = metadata.get("offset_control_spacing_m")
    if not isinstance(spacing, (int, float)) or not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("invalid offset control spacing")
    spline = OffsetSpline(track, float(spacing))
    if not np.allclose(cs, spline.control_distance, rtol=0, atol=1e-9):
        raise ValueError("control grid disagrees with declared spacing")
    trajectory = OffsetTrajectory(ref, spline, coeff, station_accuracy=1e-7)
    if not np.array_equal(mrs, trajectory.station_map.reference_station):
        if mrs.shape != trajectory.station_map.reference_station.shape or not np.allclose(mrs, trajectory.station_map.reference_station, rtol=0, atol=1e-11):
            raise ValueError("stored map reference grid disagrees with reconstructed map")
    if ps.shape != trajectory.station_map.path_station.shape or not np.allclose(ps, trajectory.station_map.path_station, rtol=0, atol=1e-7):
        raise ValueError("stored path map disagrees with reconstructed map")
    from simplified_vehicle_models.solvers.qss_spline_ocp import _SplineStateEvaluator
    evaluator = _SplineStateEvaluator(trajectory, nodes, speed, acc, mid, control_representation=control_repr)
    from simplified_vehicle_models.solvers.qss_spline_ocp import _time_quadrature_metadata
    time_meta = metadata.get("time_quadrature_metadata")
    expected_time_meta = _time_quadrature_metadata()
    if (not isinstance(time_meta, dict)
            or any(time_meta.get(key) != value for key, value in expected_time_meta.items())):
        raise ValueError("time quadrature metadata is inconsistent")
    from simplified_vehicle_models.solvers.qss_spline_ocp import physical_time_quadrature
    physical_time = physical_time_quadrature(trajectory, evaluator, nodes)
    return LoadedTrajectory(trajectory, evaluator, _ro(nodes), control_repr, state_repr,
                            time_name, _readonly(metadata), sha, physical_time)
