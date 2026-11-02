"""Bounded periodic smoothing of a multi-lap telemetry seam."""
from __future__ import annotations

import numpy as np


def _skip(target, reason):
    return target.copy(), {
        "status": "skipped", "skip_reason": reason, "physical_span_m": 0.0,
        "peak_movement_m": 0.0, "bending_energy_before": 0.0,
        "bending_energy_after": 0.0,
    }


def _energy(y, station):
    h0, h1 = np.diff(station)[:-1], np.diff(station)[1:]
    d2 = 2.0 * ((y[2:] - y[1:-1]) / h1 - (y[1:-1] - y[:-2]) / h0) / (h0 + h1)
    return float(np.sum((h0 + h1) * 0.5 * d2 * d2))


def smooth_seam(mean, overlaps, keep, target):
    """Fit lateral position on a straight crossing node zero.

    The output retains the input orientation and node order. Bounds are the
    per-node envelope of accepted overlap samples after matched-phase lateral
    projection; they are sampled observations, not physical track edges.
    """
    m = np.asarray(mean, dtype=float)
    o = np.asarray(overlaps, dtype=float)
    k = np.asarray(keep)
    t = np.asarray(target, dtype=float)
    if m.ndim != 2 or m.shape[1] != 2 or m.shape[0] < 12:
        raise ValueError("mean must be finite (nodes,2) with at least 12 nodes")
    n = m.shape[0]
    if o.ndim != 3 or o.shape[1:] != m.shape or k.shape != o.shape[:2] or t.shape != m.shape:
        raise ValueError("overlaps, keep, and target arrays must align with mean")
    if k.dtype.kind != "b":
        raise ValueError("keep mask must be boolean")
    if not (np.isfinite(m).all() and np.isfinite(o).all() and np.isfinite(t).all()):
        raise ValueError("input arrays must be finite")
    if o.shape[0] < 2:
        return _skip(t, "insufficient_lap_support")
    if np.any(k.sum(axis=0) < 2):
        return _skip(t, "insufficient_node_support")

    edges = np.linalg.norm(np.roll(m, -1, axis=0) - m, axis=1)
    if np.any(edges <= 1e-9):
        return _skip(t, "degenerate_mean_spacing")
    cumulative = np.r_[0.0, np.cumsum(edges)]
    perimeter = float(cumulative[-1])
    max_side = min(30.0, 0.1 * perimeter)
    # Interpolate positions at fixed physical distance on the periodic polyline.
    def at_distance(distance):
        q = distance % perimeter
        i = int(np.searchsorted(cumulative, q, side="right") - 1)
        fraction = (q - cumulative[i]) / edges[i]
        return m[i] + fraction * (m[(i + 1) % n] - m[i])
    tangent = at_distance(2.0) - at_distance(-2.0)
    norm = float(np.linalg.norm(tangent))
    if norm < 1e-9:
        return _skip(t, "undefined_seam_heading")
    tangent /= norm
    normal = np.array([-tangent[1], tangent[0]])
    heading_tol = np.cos(np.deg2rad(10.0))
    side_counts = []
    for sign in (-1, 1):
        count = 0
        distance = 0.0
        while count < n // 4 and distance < max_side:
            signed_distance = sign * distance
            # Keep the tangent forward-oriented for both incoming and outgoing walks.
            vec = at_distance(signed_distance + 2.0) - at_distance(signed_distance - 2.0)
            vnorm = float(np.linalg.norm(vec))
            if vnorm < 1e-9 or float(np.dot(vec / vnorm, tangent)) < heading_tol:
                break
            next_index = (sign * (count + 1)) % n
            edge_length = edges[next_index] if sign < 0 else edges[(next_index - 1) % n]
            if distance + edge_length > max_side:
                break
            distance += edge_length
            count += 1
        side_counts.append((count, distance))
    if min(side_counts[0][1], side_counts[1][1]) < 5.0:
        return _skip(t, "insufficient_straight_span")

    left, right = side_counts[0][0], side_counts[1][0]
    ids = np.arange(-left, right + 1) % n
    points = m[ids]
    mean_station = (points - m[0]) @ tangent
    if np.any(np.diff(mean_station) <= 1e-8):
        return _skip(t, "folded_longitudinal_station")
    target_relative = t[ids] - m[0]
    station = target_relative @ tangent
    y0 = target_relative @ normal
    if np.any(np.diff(station) <= 1e-8):
        return _skip(t, "nonincreasing_target_station")

    lap_lat = (o[:, ids, :] - m[0][None, None, :]) @ normal
    lower = np.full(len(ids), np.inf)
    upper = np.full(len(ids), -np.inf)
    for j, node in enumerate(ids):
        accepted = lap_lat[k[:, node], j]
        if accepted.size < 2:
            return _skip(t, "insufficient_node_support")
        lower[j], upper[j] = np.min(accepted), np.max(accepted)

    fixed = np.zeros(len(ids), dtype=bool)
    fixed[:2] = True
    fixed[-2:] = True
    if np.any(y0[fixed] < lower[fixed] - 1e-8) or np.any(y0[fixed] > upper[fixed] + 1e-8):
        return _skip(t, "boundary_target_outside_envelope")
    lo, hi = lower.copy(), upper.copy()
    lo[fixed] = hi[fixed] = y0[fixed]

    try:
        from scipy.optimize import Bounds, minimize
        from scipy.sparse import lil_matrix
    except ImportError:
        return _skip(t, "scipy_unavailable")
    ds = np.diff(station)
    if np.any(ds <= 1e-8):
        return _skip(t, "nonincreasing_station")
    # Include the adjacent unchanged target nodes outside the fit in the
    # bending stencil, so transition curvature contributes to the objective.
    outer_ids = np.array([(ids[0] - 1) % n, *ids.tolist(), (ids[-1] + 1) % n])
    outer_station = (t[outer_ids] - m[0]) @ tangent
    outer_y = (t[outer_ids] - m[0]) @ normal
    outer_ds = np.diff(outer_station)
    if np.any(outer_ds <= 1e-8):
        return _skip(t, "nonincreasing_target_station")
    D = lil_matrix((len(outer_ids) - 2, len(outer_ids)))
    for j in range(1, len(outer_ids) - 1):
        h0, h1 = outer_ds[j - 1], outer_ds[j]
        weight = np.sqrt((h0 + h1) / 2.0) * 2.0 / (h0 + h1)
        D[j - 1, j - 1] = weight / h0
        D[j - 1, j] = -weight * (1.0 / h0 + 1.0 / h1)
        D[j - 1, j + 1] = weight / h1
    D = D.tocsr()
    H = (D.T @ D).tocsr()
    # Free variables are in the middle fit window. The exterior target nodes
    # are fixed stencil values and the first/last two fit nodes remain anchored.
    free_mask = (~fixed) & (hi > lo)
    free = np.flatnonzero(free_mask)
    free_outer = free + 1
    held_outer = np.setdiff1d(np.arange(len(outer_ids)), free_outer)
    scale = max(float(np.ptp(y0)), float(np.mean(ds)), 1e-3)
    lam = 1e-3 / (scale * scale)
    fixed_values = y0.copy()
    zero_width = (~fixed) & (hi == lo)
    fixed_values[zero_width] = lo[zero_width]
    full_fixed = np.r_[outer_y[0], fixed_values, outer_y[-1]]
    Hff = H[free_outer][:, free_outer]
    linear = np.asarray(H[free_outer][:, held_outer] @ full_fixed[held_outer]).ravel()

    def fun(x):
        y = fixed_values.copy()
        y[free] = x
        full = np.r_[outer_y[0], y, outer_y[-1]]
        dy = D @ full
        return float(dy @ dy + lam * np.sum((y - y0) ** 2))

    def jac(x):
        y = fixed_values.copy()
        y[free] = x
        return 2.0 * (np.asarray(Hff @ x).ravel() + linear + lam * (x - y0[free]))

    if free.size:
        result = minimize(fun, y0[free], jac=jac, method="L-BFGS-B",
                          bounds=Bounds(lo[free], hi[free]),
                          options={"ftol": 1e-13, "gtol": 1e-9, "maxiter": 2000, "maxls": 50})
    else:
        from types import SimpleNamespace
        result = SimpleNamespace(success=True, x=np.empty(0), message="all variables fixed by bounds", nit=0)
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError(f"seam closure optimizer failed: {result.message}")
    y = fixed_values.copy()
    y[free] = result.x
    # Absolute 1e-7 m tolerance covers optimizer roundoff only; bounds are metres.
    if np.any(y < lo - 1e-7) or np.any(y > hi + 1e-7):
        raise RuntimeError("seam closure optimizer returned a bound violation")

    out = t.copy()
    out[ids] = m[0] + station[:, None] * tangent[None, :] + y[:, None] * normal[None, :]
    before = _energy(np.r_[outer_y[0], y0, outer_y[-1]], outer_station)
    after = _energy(np.r_[outer_y[0], y, outer_y[-1]], outer_station)
    diagnostics = {
        "status": "success", "skip_reason": None,
        "max_bound_violation_m": float(max(np.max(lo - y), np.max(y - hi), 0.0)),
        "longitudinal_station_span_m": float(station[-1] - station[0]),
        "fit_node_indices": ids.tolist(),
        "frame_tangent": tangent.tolist(),
        "physical_span_m": float(station[-1] - station[0]),
        "peak_movement_m": float(np.max(np.abs(y - y0))),
        "corridor_width_m": float(np.max(upper - lower)),
        "bending_energy_before": before, "bending_energy_after": after,
        "solver_status": str(result.message), "solver_iterations": int(result.nit),
    }
    return out, diagnostics
