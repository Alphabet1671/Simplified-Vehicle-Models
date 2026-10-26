"""Analytic offset trajectories derived from a differentiable XY reference.

The source station remains the independent variable throughout this module.  A
separate :class:`StationMap` records its physical path arclength, rather than
silently treating source station as metres on an offset path.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates

_EPS = 1.0e-12


def _immutable(values: object) -> np.ndarray:
    result = np.array(values, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def _scalar_or_array(values: np.ndarray, scalar: bool) -> Any:
    return float(np.asarray(values).reshape(-1)[0]) if scalar else values


def _check_station(values: object, start: float, end: float, closed: bool) -> tuple[np.ndarray, bool]:
    raw = np.asarray(values, dtype=float)
    scalar = raw.ndim == 0
    if not np.all(np.isfinite(raw)):
        raise ValueError("station must be finite")
    # Periodicity is intentionally accepted only for a closed reference.
    if closed:
        period = end - start
        checked = np.mod(raw - start, period) + start
        checked = np.where(np.isclose(raw, end, rtol=0.0, atol=1e-12), end, checked)
    else:
        if np.any(raw < start) or np.any(raw > end):
            raise ValueError(f"station is outside the open interval [{start}, {end}]")
        checked = raw
    return np.atleast_1d(checked), scalar


@dataclass(frozen=True, slots=True)
class ReferenceEvaluation:
    north: Any
    east: Any
    north_s: Any
    east_s: Any
    north_ss: Any
    east_ss: Any
    north_sss: Any
    east_sss: Any
    metric: Any
    metric_s: Any
    heading: Any
    curvature: Any
    curvature_s: Any

    @property
    def h(self) -> Any:
        return self.metric

    @property
    def h_s(self) -> Any:
        return self.metric_s


@dataclass(frozen=True, slots=True)
class TrajectoryEvaluation:
    north: Any
    east: Any
    heading: Any
    heading_error: Any
    curvature: Any
    heading_error_s: Any
    offset: Any
    metric: Any
    fold_factor: Any

    @property
    def dell_ds(self) -> Any:
        return self.metric


class ReferenceCurve:
    """Quintic (or short-fixture cubic) XY interpolation of a track centerline."""

    def __init__(self, track: object, *, interpolation_periodic: bool | None = None) -> None:
        if interpolation_periodic is not None and not isinstance(interpolation_periodic, (bool, np.bool_)):
            raise TypeError("interpolation_periodic must be bool or None")
        coordinates = getattr(track, "path_coordinates", None)
        if coordinates is None:
            raise ValueError("ReferenceCurve requires TrackProfile.path_coordinates")
        s = np.asarray(coordinates.distance, dtype=float)
        x = np.asarray(coordinates.x_north, dtype=float)
        y = np.asarray(coordinates.y_east, dtype=float)
        if s.ndim != 1 or s.size < 4 or x.shape != s.shape or y.shape != s.shape:
            raise ValueError("path coordinates must be same-length one-dimensional arrays")
        if not all(np.all(np.isfinite(v)) for v in (s, x, y)) or np.any(np.diff(s) <= 0.0):
            raise ValueError("path coordinates must be finite with increasing station")
        self.source_station = _immutable(s)
        self.is_closed = bool(getattr(coordinates, "is_closed", getattr(track, "is_closed", False)))
        metadata = getattr(track, "source_reference_interpolation_periodic", None)
        if metadata is not None and not isinstance(metadata, (bool, np.bool_)):
            raise TypeError("source_reference_interpolation_periodic must be bool or None")
        if interpolation_periodic is not None and metadata is not None and bool(interpolation_periodic) != bool(metadata):
            raise ValueError("interpolation_periodic conflicts with source reference metadata")
        if interpolation_periodic is None and metadata is not None:
            interpolation_periodic = bool(metadata)
        if interpolation_periodic is not None and not bool(interpolation_periodic) and self.is_closed:
            raise ValueError("closed reference cannot use a nonperiodic source interpolation")
        self._interpolation_periodic = (self.is_closed if interpolation_periodic is None
                                        else bool(interpolation_periodic))
        self.start = float(s[0])
        self.end = float(s[-1])
        if self.end <= self.start:
            raise ValueError("reference must have positive source length")
        if self.interpolation_periodic:
            # Periodic source fits require a repeated seam, whether or not the
            # query domain itself is closed. Source round-off is normalized only
            # in the fit copy; caller-owned physical coordinates are untouched.
            # Validate it tightly, then set the terminal fit value exactly equal
            # because SciPy's periodic condition requires bitwise consistency.
            if not np.allclose((x[-1], y[-1]), (x[0], y[0]), rtol=0.0, atol=1e-8):
                raise ValueError("closed reference positions must repeat at the seam")
            x, y = x.copy(), y.copy()
            x[-1], y[-1] = x[0], y[0]
        from scipy.interpolate import make_interp_spline  # lazy optional dependency
        degree = 5 if s.size >= 6 else 3
        boundary = "periodic" if self.interpolation_periodic else None
        try:
            self._x = make_interp_spline(s, x, k=degree, bc_type=boundary)
            self._y = make_interp_spline(s, y, k=degree, bc_type=boundary)
        except ValueError as error:
            raise ValueError(f"cannot fit XY reference interpolation: {error}") from error
        # Reject data that produce a non-regular parameterized curve at nodes.
        dx, dy = self._x(s, 1), self._y(s, 1)
        if np.any(np.hypot(dx, dy) <= _EPS) or not np.all(np.isfinite(np.r_[dx, dy])):
            raise ValueError("reference interpolation has degenerate tangents")
        # Retain auditable fit information.  Curvature is never fitted from the
        # source column; when present it is compared only as a diagnostic.
        fitted_xy_error = np.hypot(np.asarray(self._x(s)) - x, np.asarray(self._y(s)) - y)
        fitted_heading = np.arctan2(dy, dx)
        fitted_k = (dx * self._y(s, 2) - dy * self._x(s, 2)) / np.hypot(dx, dy) ** 3
        stored_k = np.asarray(getattr(coordinates, "curvature", ()), dtype=float)
        source_curvature_valid = stored_k.shape == s.shape and np.all(np.isfinite(stored_k))
        self.fit_discrepancy = {
            "xy_max": float(np.max(fitted_xy_error)),
            "metric_min": float(np.min(np.hypot(dx, dy))),
            "metric_max": float(np.max(np.hypot(dx, dy))),
            "curvature_rms_vs_source": None,
            "curvature_max_vs_source": None,
            "source_curvature_valid": bool(source_curvature_valid),
            "heading_source_available": bool(hasattr(coordinates, "heading")),
            "source_interpolation_periodic": self.interpolation_periodic,
            "source_boundary_condition": "periodic" if self.interpolation_periodic else "not-a-knot",
        }
        if source_curvature_valid:
            curvature_error = fitted_k - stored_k
            self.fit_discrepancy.update({
                "curvature_rms_vs_source": float(np.sqrt(np.mean(curvature_error ** 2))),
                "curvature_max_vs_source": float(np.max(np.abs(curvature_error))),
            })

    @property
    def interpolation_periodic(self) -> bool:
        """Whether the fitted source interpolant uses periodic boundary conditions."""
        return self._interpolation_periodic

    def evaluate(self, station: object) -> ReferenceEvaluation:
        s, scalar = _check_station(station, self.start, self.end, self.is_closed)
        x = np.asarray(self._x(s), dtype=float)
        y = np.asarray(self._y(s), dtype=float)
        xs, ys = np.asarray(self._x(s, 1), dtype=float), np.asarray(self._y(s, 1), dtype=float)
        xss, yss = np.asarray(self._x(s, 2), dtype=float), np.asarray(self._y(s, 2), dtype=float)
        xsss, ysss = np.asarray(self._x(s, 3), dtype=float), np.asarray(self._y(s, 3), dtype=float)
        h = np.hypot(xs, ys)
        if np.any(h <= _EPS):
            raise ValueError("reference interpolation has a degenerate tangent")
        h_s = (xs * xss + ys * yss) / h
        curvature = (xs * yss - ys * xss) / h**3
        curvature_s = (xs * ysss - ys * xsss) / h**3 - 3.0 * curvature * h_s / h
        heading = np.arctan2(ys, xs)
        return ReferenceEvaluation(*(_scalar_or_array(v, scalar) for v in (
            x, y, xs, ys, xss, yss, xsss, ysss, h, h_s, heading, curvature, curvature_s
        )))


class StationMap:
    """Immutable, explicit monotone map from source to physical path station."""

    def __init__(self, reference_station: object, path_station: object, is_closed: bool = False) -> None:
        source = _immutable(reference_station)
        path = _immutable(path_station)
        if source.ndim != 1 or source.size < 2 or path.shape != source.shape:
            raise ValueError("station arrays must be matching one-dimensional arrays")
        if np.any(~np.isfinite(source)) or np.any(~np.isfinite(path)) or np.any(np.diff(source) <= 0.0):
            raise ValueError("station arrays must be finite and strictly increasing")
        if np.any(np.diff(path) <= 0.0):
            raise ValueError("path station map must be strictly monotone (trajectory folds)")
        self.reference_station = source
        self.path_station = path
        self.is_closed = bool(is_closed)
        self.start = float(source[0])
        self.end = float(source[-1])
        self.path_start = float(path[0])
        self.path_end = float(path[-1])
        from scipy.interpolate import PchipInterpolator  # lazy optional dependency
        self._forward = PchipInterpolator(source, path, extrapolate=False)
        self._inverse = PchipInterpolator(path, source, extrapolate=False)

    def to_path(self, station: object) -> Any:
        source, scalar = _check_station(station, self.start, self.end, self.is_closed)
        result = np.asarray(self._forward(source), dtype=float)
        result = np.where(source == self.start, self.path_start, result)
        result = np.where(source == self.end, self.path_end, result)
        if np.any(~np.isfinite(result)):
            raise ValueError("station is not covered by the station map")
        return _scalar_or_array(result, scalar)

    def to_reference(self, station: object) -> Any:
        raw = np.asarray(station, dtype=float)
        scalar = raw.ndim == 0
        if not np.all(np.isfinite(raw)):
            raise ValueError("path station must be finite")
        if self.is_closed:
            length = self.path_end - self.path_start
            checked = np.mod(raw - self.path_start, length) + self.path_start
            checked = np.where(np.isclose(raw, self.path_end, rtol=0.0, atol=1e-12), self.path_end, checked)
        else:
            if np.any(raw < self.path_start) or np.any(raw > self.path_end):
                raise ValueError("path station is outside map coverage")
            checked = raw
        checked = np.atleast_1d(checked)
        result = np.asarray(self._inverse(checked), dtype=float)
        result = np.where(checked == self.path_start, self.start, result)
        result = np.where(checked == self.path_end, self.end, result)
        if np.any(~np.isfinite(result)):
            raise ValueError("path station is not covered by the station map")
        return _scalar_or_array(result, scalar)


class OffsetTrajectory:
    """Offset a reference curve analytically using an existing ``OffsetSpline``."""

    def __init__(self, reference: ReferenceCurve, spline: object, coefficients: object,
                 station_accuracy: float = 1.0e-7, *, build_station_map: bool = True) -> None:
        if bool(getattr(spline, "is_closed", False)) != reference.is_closed:
            raise ValueError("reference and offset spline closure must agree")
        if not np.isfinite(station_accuracy) or station_accuracy <= 0.0:
            raise ValueError("station_accuracy must be positive and finite")
        if not isinstance(build_station_map, (bool, np.bool_)):
            raise TypeError("build_station_map must be boolean")
        values = np.asarray(coefficients, dtype=float)
        count = int(getattr(spline, "control_count"))
        if values.shape != (count,) or not np.all(np.isfinite(values)):
            raise ValueError("offset coefficients must be finite and match the spline")
        self.reference, self.spline = reference, spline
        self.coefficients = _immutable(values)
        self.station_accuracy = float(station_accuracy)
        # Build once.  OffsetSpline.evaluate intentionally constructs an
        # interpolation kernel per public call; a trajectory is an evaluator
        # and must not pay that cost at every geometry or quadrature query.
        self._offset_curve = spline._spline(self.coefficients)
        self.station_map = self._build_station_map() if build_station_map else None

    def _knots(self) -> np.ndarray:
        controls = np.asarray(self.spline.control_distance, dtype=float)
        # A periodic spline excludes the terminal knot; add it for quadrature.
        raw = np.sort(np.r_[self.reference.source_station, controls, self.reference.start, self.reference.end])
        # Arithmetic construction of control and source grids can produce
        # near-identical knots.  Merge those so quadrature never creates a
        # zero-width panel while retaining the exact endpoint values.
        tolerance = 1.0e-11 * max(1.0, self.reference.end - self.reference.start)
        keep = np.r_[True, np.diff(raw) > tolerance]
        knots = raw[keep]
        knots[0], knots[-1] = self.reference.start, self.reference.end
        return knots

    def _build_station_map(self) -> StationMap:
        """Integrate metric with vectorized 8-point Gauss--Legendre panels.

        Each panel is split at both source and offset knots.  Two resolutions
        are compared and doubled until the *integral* convergence target is
        met; map samples are the high-order panel boundaries, never chords.
        """
        knots = self._knots()
        nodes, weights = np.polynomial.legendre.leggauss(8)
        panels_per_interval = 1
        for _ in range(13):
            starts: list[np.ndarray] = []
            ends: list[np.ndarray] = []
            for left, right in zip(knots[:-1], knots[1:]):
                grid = np.linspace(left, right, panels_per_interval + 1)
                starts.append(grid[:-1])
                ends.append(grid[1:])
            lower, upper = np.concatenate(starts), np.concatenate(ends)
            queries = ((lower + upper)[:, None] + (upper - lower)[:, None] * nodes) / 2.0
            metric = np.asarray(self.evaluate(queries.ravel()).metric).reshape(queries.shape)
            increments = (upper - lower) * (metric @ weights) / 2.0
            total = float(np.sum(increments))
            if hasattr(self, "_last_station_total") and abs(total - self._last_station_total) <= self.station_accuracy:
                boundaries = np.r_[lower[0], upper]
                path = np.r_[0.0, np.cumsum(increments)]
                candidate = StationMap(boundaries, path, self.reference.is_closed)
                # Total-length convergence is insufficient: PCHIP's local map
                # can err between samples while the integrated total is exact.
                # Total length does not qualify PCHIP between samples.  Probe
                # three off-grid locations in every panel with independent
                # 16-point Gauss integration, then check the inverse too.
                fractions = np.array([0.211324865405187, 0.5, 0.788675134594813])
                probe = lower[:, None] + (upper - lower)[:, None] * fractions
                probe_lower = np.broadcast_to(lower[:, None], probe.shape)
                fine_nodes, fine_weights = np.polynomial.legendre.leggauss(16)
                local_q = ((probe_lower[..., None] + probe[..., None])
                           + (probe[..., None] - probe_lower[..., None]) * fine_nodes) / 2.0
                local_metric = np.asarray(self.evaluate(local_q.ravel()).metric).reshape(local_q.shape)
                exact_probe_path = (path[:-1, None] + (probe - probe_lower)
                                    * (local_metric @ fine_weights) / 2.0)
                probe_flat, exact_flat = probe.ravel(), exact_probe_path.ravel()
                forward_error = float(np.max(np.abs(np.asarray(candidate.to_path(probe_flat)) - exact_flat)))
                inverse_error = float(np.max(np.abs(np.asarray(candidate.to_reference(exact_flat)) - probe_flat)))
                # Reserve a factor four for off-panel queries; the recorded
                # error remains the measured probe maximum, not this margin.
                if forward_error <= self.station_accuracy / 4.0 and inverse_error <= self.station_accuracy / 4.0:
                    self.station_map_error = {"path_station": forward_error, "reference_station": inverse_error}
                    return candidate
            self._last_station_total = total
            panels_per_interval *= 2
        raise ValueError("station-map quadrature did not reach requested accuracy")

    def evaluate(self, station: object) -> TrajectoryEvaluation:
        s, scalar = _check_station(station, self.reference.start, self.reference.end, self.reference.is_closed)
        ref = self.reference.evaluate(s)
        n = np.asarray(self._offset_curve(s), dtype=float)
        n_s = np.asarray(self._offset_curve(s, 1), dtype=float)
        n_ss = np.asarray(self._offset_curve(s, 2), dtype=float)
        h = np.asarray(ref.metric, dtype=float)
        k = np.asarray(ref.curvature, dtype=float)
        h_s = np.asarray(ref.metric_s, dtype=float)
        k_s = np.asarray(ref.curvature_s, dtype=float)
        fold = 1.0 - k * n
        a, b = h * fold, n_s
        a_s = h_s * fold - h * (k_s * n + k * n_s)
        b_s = n_ss
        metric_sq = a * a + b * b
        metric = np.sqrt(metric_sq)
        if np.any(metric <= _EPS):
            raise ValueError("offset trajectory has a zero metric (fold)")
        psi = np.arctan2(b, a)
        psi_s = (a * b_s - b * a_s) / metric_sq
        heading = np.asarray(ref.heading, dtype=float) + psi
        curvature = (h * k + psi_s) / metric
        north = np.asarray(ref.north, dtype=float) - n * np.asarray(ref.east_s, dtype=float) / h
        east = np.asarray(ref.east, dtype=float) + n * np.asarray(ref.north_s, dtype=float) / h
        return TrajectoryEvaluation(*(_scalar_or_array(v, scalar) for v in (
            north, east, heading, psi, curvature, psi_s, n, metric, fold
        )))

    def sample_path(self, station: object, *, heading_tolerance: float = 1.8e-3,
                    chord_angle_tolerance: float = 1.8e-3,
                    chord_length_tolerance: float = 9.0e-4,
                    maximum_nodes: int = 200000) -> PathCoordinates:
        """Adaptively sample the identical analytic curve for strict CSV loading."""
        s, _ = _check_station(station, self.reference.start, self.reference.end,
                              self.reference.is_closed)
        if s.size < 2 or np.any(np.diff(s) <= 0.0):
            raise ValueError("sample_path station must be a strictly increasing sequence")
        for name, value in (("heading_tolerance", heading_tolerance),
                            ("chord_angle_tolerance", chord_angle_tolerance),
                            ("chord_length_tolerance", chord_length_tolerance)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        # Preserve all original reference samples and offset control knots in
        # addition to caller stations, so adaptive chord checks cannot skip a
        # spline feature hidden within a long requested interval.
        knots = np.r_[s, self.reference.source_station,
                      np.asarray(self.spline.control_distance, dtype=float)]
        knots = knots[(knots >= self.reference.start) & (knots <= self.reference.end)]
        values = list(map(float, np.unique(knots))); index = 0
        while index < len(values) - 1:
            if len(values) > maximum_nodes:
                raise ValueError("adaptive trajectory export exceeded node limit")
            left, right = values[index], values[index + 1]
            item = self.evaluate(np.array([left, right]))
            x, y = np.asarray(item.north), np.asarray(item.east)
            heading = np.unwrap(np.asarray(item.heading)); curvature = np.asarray(item.curvature)
            length = float(self.station_map.to_path(right) - self.station_map.to_path(left))
            chord = float(np.hypot(x[1] - x[0], y[1] - y[0]))
            chord_heading = math.atan2(y[1] - y[0], x[1] - x[0])
            heading_error = abs((chord_heading - .5*(heading[0] + heading[1]) + np.pi) % (2*np.pi) - np.pi)
            trapezoid_error = abs((heading[1] - heading[0]) - .5*(curvature[0] + curvature[1])*length)
            length_error = abs(chord - length) / max(length, 1e-12)
            if (heading_error <= chord_angle_tolerance and trapezoid_error <= heading_tolerance
                    and length_error <= chord_length_tolerance):
                index += 1
            elif right - left <= 1e-12:
                raise ValueError("adaptive trajectory export did not meet loader tolerances")
            else:
                values.insert(index + 1, .5*(left + right))
        return self.to_path(np.asarray(values, dtype=float))

    def to_path(self, station: object) -> PathCoordinates:
        s, _ = _check_station(station, self.reference.start, self.reference.end, self.reference.is_closed)
        if s.size < 2 or np.any(np.diff(s) <= 0.0):
            raise ValueError("to_path station must be a strictly increasing sequence")
        values = self.evaluate(s)
        distance = np.asarray(self.station_map.to_path(s), dtype=float)
        # PathCoordinates defines heading as unwrapped.  Evaluator headings are
        # local principal angles, appropriate for scalar queries, so unwrap only
        # this explicitly ordered exported table.
        heading = np.unwrap(np.asarray(values.heading, dtype=float))
        arrays = [_immutable(v) for v in (distance, values.north, values.east, heading, values.curvature)]
        return PathCoordinates(*arrays, is_closed=self.reference.is_closed,
                               reference_distance=_immutable(s))
