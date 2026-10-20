"""Reference-free spatial loop detection, registration and robust mean geometry."""
from __future__ import annotations

import math

import numpy as np


def _arc(x, y):
    """Return cumulative polyline length beginning at zero."""
    return np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]


def detect_laps(
    time,
    north,
    east,
    *,
    return_radius=12.0,
    minimum_travel=100.0,
    minimum_duration=15.0,
    maximum_duration=240.0,
    heading_tolerance=math.pi / 3,
):
    """Find moving-start spatial returns with distance, duration and heading guards."""
    from scipy.spatial import cKDTree

    t, x, y = (np.asarray(values, dtype=float) for values in (time, north, east))
    if t.ndim != 1 or len(t) < 8 or len(t) != len(x) or len(t) != len(y):
        raise ValueError("lap detection needs at least eight synchronized samples")
    if (
        not np.all(np.isfinite(t))
        or not np.all(np.isfinite(x))
        or not np.all(np.isfinite(y))
        or np.any(np.diff(t) <= 0)
    ):
        raise ValueError(
            "lap detection requires finite coordinates and strictly increasing time"
        )

    step = np.hypot(np.diff(x), np.diff(y))
    cumulative = np.r_[0.0, np.cumsum(step)]
    # Coordinate samples may be high-rate (MIS ~100 Hz); use a rolling one-second
    # displacement instead of an arbitrary per-sample distance threshold.
    median_dt = float(np.median(np.diff(t)))
    window = max(2, int(round(1.0 / median_dt)))
    movement = (
        np.hypot(x[window:] - x[:-window], y[window:] - y[:-window])
        if len(t) > window
        else np.array([])
    )
    moving = np.flatnonzero(movement > 1.0) + window
    if moving.size < 2:
        raise ValueError(
            "no complete spatial-return lap found: selected window contains no sustained motion"
        )

    stride = max(1, int(np.ceil(len(t) / 30000)))
    anchors = np.unique(np.r_[np.arange(0, len(t), stride), len(t) - 1])
    tree = cKDTree(np.c_[x, y])
    found = []
    rejected = []
    start = 0
    attempts = 0
    while start < len(t) - 2 and attempts < 1000:
        attempts += 1
        # Skip stationary intervals; begin shortly before a sustained moving segment.
        if start + 1 < len(step):
            local_speed = step[start] / (t[start + 1] - t[start])
            if local_speed < 0.5:
                next_moving = moving[moving > start]
                if not len(next_moving):
                    break
                start = max(start, int(next_moving[0]) - 1)

        eligible_anchors = anchors[
            (anchors > start)
            & (t[anchors] - t[start] >= minimum_duration)
            & (t[anchors] - t[start] <= maximum_duration)
            & (cumulative[anchors] - cumulative[start] >= minimum_travel)
        ]
        if not len(eligible_anchors):
            break

        near = np.asarray(
            tree.query_ball_point([x[start], y[start]], return_radius), dtype=int
        )
        near = near[
            (near > start)
            & (t[near] - t[start] >= minimum_duration)
            & (t[near] - t[start] <= maximum_duration)
            & (cumulative[near] - cumulative[start] >= minimum_travel)
        ]
        near = np.intersect1d(near, anchors, assume_unique=False)
        finish = None
        if len(near):
            near = near[np.argsort(near)]
            first = near[0]
            group_end = int(first)
            for candidate in near[1:]:
                if candidate - group_end > stride:
                    break
                group_end = int(candidate)
            encounter = near[near <= group_end]
            # Include dense samples around the full first return encounter.
            low = max(start + 1, int(encounter[0]) - stride)
            high = min(len(t), int(encounter[-1]) + stride + 1)
            segment = np.arange(low, high)
            eligible = segment[
                (t[segment] - t[start] >= minimum_duration)
                & (t[segment] - t[start] <= maximum_duration)
                & (cumulative[segment] - cumulative[start] >= minimum_travel)
            ]
            if len(eligible):
                distances = np.hypot(x[eligible] - x[start], y[eligible] - y[start])
                finish = int(eligible[np.argmin(distances)])

        if finish is None:
            # Advance to the next moving phase and search again, allowing a
            # selected window that starts mid-lap to recover a full loop.
            half_minimum = max(1, int(round(minimum_duration / median_dt)) // 2)
            next_anchor = moving[moving > start + half_minimum]
            if not len(next_anchor):
                break
            start = int(next_anchor[0])
            continue

        heading_span = max(2, int(round(0.5 / median_dt)))
        initial_heading = np.array(
            [
                x[min(start + heading_span, len(t) - 1)] - x[start],
                y[min(start + heading_span, len(t) - 1)] - y[start],
            ]
        )
        final_heading = np.array(
            [
                x[finish] - x[max(start, finish - heading_span)],
                y[finish] - y[max(start, finish - heading_span)],
            ]
        )
        if np.linalg.norm(initial_heading) * np.linalg.norm(final_heading) < 1e-8:
            angle = math.pi
        else:
            cross = (
                initial_heading[0] * final_heading[1]
                - initial_heading[1] * final_heading[0]
            )
            dot = float(np.dot(initial_heading, final_heading))
            angle = abs(math.atan2(cross, dot))
        if angle > heading_tolerance:
            rejected.append(
                {"start_index": start, "end_index": finish, "reason": "heading_mismatch"}
            )
            later = moving[moving > start + 1]
            start = int(later[0]) if len(later) else finish + 1
            continue

        found.append((start, finish + 1))
        # Preserve a common exact seam endpoint while keeping loop interiors
        # disjoint. A repeated full interval cannot be counted as a new lap.
        start = finish
        if start < len(step):
            next_moving = moving[moving > start]
            if len(next_moving) and next_moving[0] - start > 100:
                start = int(next_moving[0]) - 1

    if not found:
        raise ValueError(
            "no complete spatial-return laps found; widen the window or adjust "
            "return radius and duration limits"
        )
    return found, {
        "candidate_count": len(found),
        "rejected": rejected,
        "detection_stride": stride,
    }


def average_laps(
    time,
    north,
    east,
    *,
    spacing=0.5,
    smoothing=2.0,
    return_radius=12.0,
    minimum_travel=100.0,
    minimum_duration=15.0,
    maximum_duration=240.0,
    deviation_limit=8.0,
    minimum_support=2,
):
    """Filter, cyclically register, clip and arithmetically average complete loops."""
    t, x, y = (np.asarray(values, dtype=float) for values in (time, north, east))
    if not isinstance(minimum_support, (int, np.integer)) or minimum_support < 2:
        raise ValueError("minimum_support must be an integer of at least two laps")

    laps, detection = detect_laps(
        t,
        x,
        y,
        return_radius=return_radius,
        minimum_travel=minimum_travel,
        minimum_duration=minimum_duration,
        maximum_duration=maximum_duration,
    )
    rejected_laps = []
    diagnostics = {
        "candidate_count": len(laps),
        "detection_rejections": detection["rejected"],
        "rejected_candidate_count": len(detection["rejected"]),
        "detection_stride": detection["detection_stride"],
    }
    records = []
    for begin, end in laps:
        length = float(_arc(x[begin:end], y[begin:end])[-1])
        duration = float(t[end - 1] - t[begin])
        if (
            duration < minimum_duration
            or duration > maximum_duration
            or length < minimum_travel
        ):
            rejected_laps.append(
                {
                    "start_index": begin,
                    "end_index": end - 1,
                    "reason": "duration_or_travel_limit",
                }
            )
            continue

        # A prolonged near-stop inside a candidate is characteristic of a pit
        # or dwell interval, not a representative driving lap.
        local_window = max(2, int(round(0.5 / float(np.median(np.diff(t[begin:end]))))))
        local_move = np.hypot(
            x[begin + local_window : end] - x[begin : end - local_window],
            y[begin + local_window : end] - y[begin : end - local_window],
        )
        local_speed = local_move / max(
            float(np.median(np.diff(t[begin:end]))) * local_window, 1e-9
        )
        stopped = local_speed < 0.5
        longest = run = 0
        for is_stopped in stopped:
            run = run + 1 if is_stopped else 0
            longest = max(longest, run)
        stopped_duration = (
            longest * float(np.median(np.diff(t[begin:end]))) * local_window
        )
        if stopped_duration > max(4.0, 0.2 * duration):
            rejected_laps.append(
                {
                    "start_index": begin,
                    "end_index": end - 1,
                    "reason": "excessive_stationary_dwell",
                }
            )
            continue
        records.append((begin, end, length, duration))

    if not records:
        raise ValueError("no complete laps pass travel and duration quality checks")
    median_length = float(np.median([record[2] for record in records]))
    length_consistent = []
    for record in records:
        if abs(record[2] - median_length) > max(0.08 * median_length, 3 * deviation_limit):
            rejected_laps.append(
                {
                    "start_index": record[0],
                    "end_index": record[1] - 1,
                    "reason": "lap_length_outlier",
                }
            )
        else:
            length_consistent.append(record)
    if len(length_consistent) < 2:
        raise ValueError("fewer than two geometrically plausible complete laps remain")

    node_count = int(np.clip(round(median_length / spacing), 32, 12000))
    phase = np.arange(node_count) / node_count
    curves = []
    for begin, end, length, _duration in length_consistent:
        north_loop = x[begin:end].copy()
        east_loop = y[begin:end].copy()
        fraction = np.linspace(0.0, 1.0, len(north_loop))
        north_loop -= fraction * (north_loop[-1] - north_loop[0])
        east_loop -= fraction * (east_loop[-1] - east_loop[0])
        distance = _arc(north_loop, east_loop)
        if distance[-1] <= 0:
            continue
        curves.append(
            np.column_stack(
                (
                    np.interp(phase * distance[-1], distance, north_loop),
                    np.interp(phase * distance[-1], distance, east_loop),
                )
            )
        )

    reference = curves[0]
    aligned = []
    shifts = range(0, node_count, max(1, node_count // 512))
    coarse_step = max(1, node_count // 512)
    reference_center = reference.mean(axis=0)
    for curve in curves:
        best_score, best_shift = float("inf"), 0
        for shift in shifts:
            shifted = np.roll(curve, shift, axis=0)
            shifted += reference_center - shifted.mean(axis=0)
            score = float(np.mean(np.sum((shifted - reference) ** 2, axis=1)))
            if score < best_score:
                best_score, best_shift = score, shift
        # Refine around the coarse phase estimate to one-node precision.
        for shift in range(best_shift - coarse_step + 1, best_shift + coarse_step):
            candidate_shift = shift % node_count
            shifted = np.roll(curve, candidate_shift, axis=0)
            shifted += reference_center - shifted.mean(axis=0)
            score = float(np.mean(np.sum((shifted - reference) ** 2, axis=1)))
            if score < best_score:
                best_score, best_shift = score, candidate_shift
        shifted = np.roll(curve, best_shift, axis=0)
        shifted += reference_center - shifted.mean(axis=0)
        aligned.append(shifted)

    stack = np.asarray(aligned)
    centre = np.median(stack, axis=0)
    residual = np.linalg.norm(stack - centre, axis=2)
    lap_score = np.median(residual, axis=1)
    lap_threshold = max(deviation_limit, 3 * float(np.median(lap_score)) + 1.0)
    good_laps = lap_score <= lap_threshold
    for record, accepted, score in zip(length_consistent, good_laps, lap_score):
        if not accepted:
            rejected_laps.append(
                {
                    "start_index": record[0],
                    "end_index": record[1] - 1,
                    "reason": "shape_residual_outlier",
                    "median_residual_m": float(score),
                }
            )
    stack = stack[good_laps]
    length_consistent = [
        record for record, accepted in zip(length_consistent, good_laps) if accepted
    ]
    if len(stack) < 2:
        raise ValueError("fewer than two similar laps remain after shape quality filtering")

    centre = np.median(stack, axis=0)
    residual = np.linalg.norm(stack - centre, axis=2)
    median_residual = np.median(residual, axis=0)
    mad = 1.4826 * np.median(np.abs(residual - median_residual), axis=0)
    cutoff = np.minimum(
        deviation_limit,
        np.maximum(1.0, median_residual + 4 * np.maximum(mad, 0.15)),
    )
    # Keep the configurable physical limit binding even when the robust local
    # scale estimate exceeds it. If that removes too much support, fail closed.
    keep = residual <= deviation_limit
    keep &= residual <= cutoff[None, :]
    if minimum_support > len(stack):
        raise ValueError(
            "minimum_support exceeds the number of accepted laps; widen the "
            "window or reduce the support requirement"
        )
    minimum = int(minimum_support)
    support = keep.sum(axis=0)
    if np.any(support < minimum):
        raise ValueError(
            "too few lap overlaps support the arithmetic mean; widen the window "
            "or relax deviation clipping"
        )
    mean = (stack * keep[:, :, None]).sum(axis=0) / support[:, None]
    overlaps = stack.copy()

    # Smoothing occurs once here; downstream geometry construction must use zero.
    if smoothing > 0:
        from tools.tracks.build_closed_track import periodic_gaussian

        sigma_nodes = min(smoothing / (median_length / node_count), node_count / 16)
        mean[:, 0] = periodic_gaussian(mean[:, 0], sigma_nodes)
        mean[:, 1] = periodic_gaussian(mean[:, 1], sigma_nodes)

    closed_mean = np.vstack((mean, mean[0]))
    distance = _arc(closed_mean[:, 0], closed_mean[:, 1])
    result_length = float(distance[-1])
    result_count = int(np.clip(round(result_length / spacing), 32, 12000))
    grid = np.arange(result_count) * result_length / result_count
    mean = np.column_stack(
        (
            np.interp(grid, distance, closed_mean[:, 0]),
            np.interp(grid, distance, closed_mean[:, 1]),
        )
    )
    mean = np.vstack((mean, mean[0]))

    diagnostics.update(
        {
            "accepted_laps": int(len(stack)),
            "rejected_laps": rejected_laps,
            "lap_lengths_m": [record[2] for record in length_consistent],
            "lap_boundaries": [
                {
                    "start_index": begin,
                    "end_index": end - 1,
                    "start_time_s": float(t[begin]),
                    "end_time_s": float(t[end - 1]),
                }
                for begin, end, _length, _duration in length_consistent
            ],
            "clipped_samples": int((~keep).sum()),
            "total_aligned_samples": int(keep.size),
            "overlap_support": support.tolist(),
            "residuals_m": {
                "median": float(np.median(residual)),
                "p95": float(np.percentile(residual, 95)),
                "maximum": float(np.max(residual)),
            },
            "registration": (
                "normalized arc-length cyclic phase with translation-only alignment; "
                "north/east orientation retained"
            ),
        }
    )
    return mean[:, 0], mean[:, 1], diagnostics, overlaps
