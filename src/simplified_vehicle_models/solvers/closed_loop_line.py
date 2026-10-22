"""Correct a spline minimum-time line against a stateful controller replay.

The quasi-steady OCP sees a capability surface, so a controller with hidden
state, rate limits, or allocation logic can be slower than the surface predicts.
This module closes that loop: solve the spline OCP, rebuild the path, replay it
with the controller, and lower a per-node speed cap where the replay is slower
or infeasible.  The OCP is then re-solved under the cap.  A correction is
accepted only when the final replay is feasible and no slower than the OCP
within a relative tolerance.  A re-solve that fails (for example because a
floored cap is infeasible) is reported as an unaccepted result rather than
raised out of the correction loop.

The replay adapter owns its integration method and its open/closed start
treatment.  The correction loop therefore uses ``ReplaySample.lap_time``
directly and never integrates a possibly infeasible replay speed trace itself.

``CorrectionResult.speed_caps`` holds one read-only array per solve that
reached a replay: the per-node cap in force for that round.  The first round is
uncapped, so the solved OCP speed profile is recorded as its effective cap;
every later entry is the cap that was passed to :func:`solve_spline_line`.  The
sequence is non-increasing by construction.  A solve that raises adds no entry,
so ``len(speed_caps) <= rounds``.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

import numpy as np

from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
from simplified_vehicle_models.tracks.racing_line import path_from_solution


#: Speed removed from the replay's demonstrated speed before it becomes a cap.
CAP_MARGIN = 0.1


@dataclass(frozen=True, slots=True)
class ReplaySample:
    """One controller replay of a rebuilt path.

    ``distance`` and ``speed`` are the achieved samples on the rebuilt path,
    ``feasible`` reports whether the controller completed the lap without
    violating its own limits, and ``lap_time`` is the replay's own lap time.
    ``saturation`` records the controller's saturation events, if any.
    """

    distance: np.ndarray
    speed: np.ndarray
    feasible: bool
    lap_time: float
    saturation: tuple[str, ...] = ()


class ReplayProtocol(Protocol):
    """Callable replay seam: ``replay(path, controller) -> ReplaySample``."""

    def __call__(self, path, controller) -> ReplaySample: ...


@dataclass(frozen=True, slots=True)
class CorrectionResult:
    """Outcome of one closed-loop correction sequence.

    ``rounds`` is the number of OCP solve attempts, ``replay_time`` is the last
    completed replay's lap time (NaN if no replay completed), and
    ``speed_caps`` holds the cap in force in every round that reached a replay.
    ``accepted`` is true only when the final replay is feasible and no slower
    than the OCP within the requested tolerance.
    """

    rounds: int
    replay_time: float
    speed_caps: tuple[np.ndarray, ...]
    cap_margin: float
    accepted: bool


def _validate_controls(tolerance, maximum_rounds) -> tuple[float, int]:
    """Validate the correction controls before any solver work."""
    if isinstance(maximum_rounds, (bool, np.bool_)) or not isinstance(
        maximum_rounds, (int, np.integer)
    ):
        raise TypeError("maximum_rounds must be an integer")
    rounds = int(maximum_rounds)
    if rounds < 1:
        raise ValueError("maximum_rounds must be at least one")
    if isinstance(tolerance, (bool, np.bool_)):
        raise TypeError("tolerance must be a finite nonnegative number")
    try:
        value = float(tolerance)
    except (TypeError, ValueError) as error:
        raise TypeError("tolerance must be a finite nonnegative number") from error
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("tolerance must be finite and nonnegative")
    return value, rounds


def _validate_replay(sample) -> None:
    """Validate one replay sample before it is used for a cap.

    The replay grid may differ from the OCP mesh; it is interpolated onto the
    mesh later.  The rebuilt path's polyline length can differ slightly from
    the track lap length, so a replay distance grid that is marginally shorter
    than the OCP mesh is allowed and ``np.interp`` clamps its end values.
    """
    if not isinstance(sample, ReplaySample):
        raise TypeError("replay must return ReplaySample")
    distance = np.asarray(sample.distance, dtype=float)
    speed = np.asarray(sample.speed, dtype=float)
    if distance.ndim != 1 or speed.ndim != 1 or distance.shape != speed.shape:
        raise ValueError("replay distance and speed must be one-dimensional and equally sized")
    if distance.size < 2:
        raise ValueError("replay must report at least two distance samples")
    if not np.all(np.isfinite(distance)) or not np.all(np.isfinite(speed)):
        raise ValueError("replay distance and speed must be finite")
    if np.any(np.diff(distance) <= 0.0):
        raise ValueError("replay distance must be strictly increasing")
    if np.any(speed < 0.0):
        raise ValueError("replay speed must be nonnegative")
    lap_time = float(sample.lap_time)
    if not math.isfinite(lap_time) or lap_time < 0.0:
        # A zero time is tolerated for an aborted replay; acceptance still
        # requires a positive time in :func:`_matches`.
        raise ValueError("replay lap_time must be finite and nonnegative")


def _cap_from_replay(mesh, replay, ocp_speed, margin):
    """Return the per-node cap implied by one replay's achieved speed."""
    replay_speed = np.interp(mesh, replay.distance, replay.speed)
    return np.minimum(ocp_speed, replay_speed) - margin


def _next_cap(mesh, replay, ocp_speed, previous, minimum_speed, margin):
    """Return the next non-increasing cap, floored at the solver's speed floor.

    The cap may never exceed the previous cap or the OCP speed.  A cap below
    ``settings.minimum_speed`` is not representable by the transcription, so the
    floor is applied; it is a representability floor, not a physical limit.
    """
    cap = _cap_from_replay(mesh, replay, ocp_speed, margin)
    if previous is not None:
        cap = np.minimum(cap, previous)
    return np.maximum(cap, float(minimum_speed))


def _matches(solution, sample, tolerance: float) -> bool:
    """Return true for a feasible replay that is no slower than the OCP.

    Cap correction can only lower speeds, so a replay faster than the OCP is
    already within the controller's demonstrated capability and must be
    accepted; the relative tolerance is applied only on the slow side.
    """
    if not bool(sample.feasible):
        return False
    ocp_time = float(solution.lap_time)
    replay_time = float(sample.lap_time)
    if not (math.isfinite(ocp_time) and ocp_time > 0.0):
        return False
    if not (math.isfinite(replay_time) and replay_time > 0.0):
        return False
    return replay_time <= ocp_time * (1.0 + tolerance)


def correct_to_replay(track, corridor, surface, mesh, spline, settings, controller,
                      replay: ReplayProtocol, *, tolerance=0.01,
                      maximum_rounds=5) -> CorrectionResult:
    """Re-solve the spline line until its replay is feasible and not slower.

    The first round solves without a speed cap.  A failed round whose replay is
    slower or infeasible lowers the per-node cap to
    ``min(v_ocp, v_replay) - CAP_MARGIN`` (never above the previous cap) and
    re-solves with that cap.  The loop stops after ``maximum_rounds`` solve
    attempts, or as soon as a replay is feasible and

    ``replay_time <= ocp_time * (1 + tolerance)``.

    A re-solve that raises ``RuntimeError`` (for example an infeasible floored
    cap) ends the loop with ``accepted=False``; the result still reports the
    attempts, the caps recorded so far, and the last completed replay time (NaN
    if none completed).  The returned ``accepted`` flag is true only for the
    accepted final replay.
    """
    tolerance, maximum_rounds = _validate_controls(tolerance, maximum_rounds)
    if not callable(replay):
        raise TypeError("replay must be callable as replay(path, controller)")

    minimum_speed = float(settings.minimum_speed)
    cap = None
    caps: list[np.ndarray] = []
    last_replay_time = math.nan
    rounds = 0
    accepted = False

    for _ in range(maximum_rounds):
        rounds += 1
        try:
            solution = solve_spline_line(
                track, corridor, surface, mesh, spline, settings, speed_cap=cap,
            )
        except RuntimeError:
            break
        path = path_from_solution(track, solution)
        sample = replay(path, controller)
        _validate_replay(sample)
        last_replay_time = float(sample.lap_time)

        effective = np.array(solution.speed if cap is None else cap, dtype=float)
        effective.setflags(write=False)
        caps.append(effective)

        if _matches(solution, sample, tolerance):
            accepted = True
            break

        cap = _next_cap(
            mesh, sample, np.asarray(solution.speed, dtype=float), cap,
            minimum_speed, CAP_MARGIN,
        )

    return CorrectionResult(
        rounds=rounds,
        replay_time=last_replay_time,
        speed_caps=tuple(caps),
        cap_margin=CAP_MARGIN,
        accepted=accepted,
    )
