"""Stage 1 acceptance checks for the minimum-time line.

Each check is a pure function of measured values so it can be unit tested. The
run tool computes the values and reports all five.
"""
from __future__ import annotations

import numpy as np


def _valid_times(*values: float) -> bool:
    return all(np.isfinite(value) and value > 0.0 for value in values)


def _valid_tolerance(tolerance: float) -> bool:
    return bool(np.isfinite(tolerance) and tolerance >= 0.0)


def check_zero_corridor_matches_envelope(*, open_loop: float, ocp: float, tolerance: float) -> bool:
    """A collapsed corridor must reproduce the centreline envelope."""
    return bool(_valid_times(open_loop, ocp) and _valid_tolerance(tolerance)
                and abs(ocp - open_loop) <= tolerance * abs(open_loop))


def check_free_line_not_slower(*, centreline: float, free: float) -> bool:
    """The free line must not be slower than the centreline."""
    return bool(_valid_times(centreline, free) and free <= centreline)


def check_corridor_containment(offset, minimum, maximum) -> bool:
    """Every reported offset must lie inside the corridor."""
    values = np.asarray(offset, dtype=float)
    lower = np.asarray(minimum, dtype=float)
    upper = np.asarray(maximum, dtype=float)
    if values.size == 0 or values.shape != lower.shape or values.shape != upper.shape:
        return False
    if not (np.all(np.isfinite(values)) and np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))):
        return False
    if np.any(lower > upper):
        return False
    return bool(np.all(values >= lower - 1.0e-6) and np.all(values <= upper + 1.0e-6))


def check_mesh_convergence(*, coarse: float, fine: float, tolerance: float) -> bool:
    """Two mesh levels must agree to within a relative tolerance."""
    return bool(_valid_times(coarse, fine) and _valid_tolerance(tolerance)
                and abs(fine - coarse) <= tolerance * abs(coarse))


def check_replay_agreement(*, ocp: float, replay: float, tolerance: float) -> bool:
    """The independent solver on the rebuilt path must reproduce the OCP lap."""
    return bool(_valid_times(ocp, replay) and _valid_tolerance(tolerance)
                and abs(replay - ocp) <= tolerance * abs(ocp))
