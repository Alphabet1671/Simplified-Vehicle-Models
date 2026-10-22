"""Correct a spline minimum-time line against a stateful controller replay.

The quasi-steady OCP sees a capability surface, so a controller with hidden
state, rate limits, or allocation logic can be slower than the surface predicts.
This module closes that loop: solve the spline OCP, rebuild the path, replay it
with the controller, and lower a per-node speed cap where the replay is slower
or infeasible.  The OCP is then re-solved under the cap.  A correction is
accepted only when the final replay is feasible and within the requested
two-sided relative tolerance of the OCP.  A re-solve that fails (for example
because a floored cap is infeasible) is reported as an unaccepted result rather
than raised out of the correction loop.

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
    """Callable replay seam: ``replay(path, controller) -> ReplaySample``.

    An adapter that drives ``controller`` should declare
    ``consumes_controller = True`` so a caller can reject a policy attached to
    an adapter that only implements a parity seam.
    """

    consumes_controller: bool

    def __call__(self, path, controller) -> ReplaySample: ...


@dataclass(frozen=True, slots=True)
class CorrectionResult:
    """Outcome of one closed-loop correction sequence.

    ``rounds`` is the number of OCP solve attempts, ``replay_time`` is the last
    completed replay's lap time (NaN if no replay completed), and
    ``speed_caps`` holds the cap in force in every round that reached a replay.
    ``feasible`` reports whether the last completed replay was feasible;
    ``accepted`` is true only when the final replay is feasible and within the
    requested two-sided tolerance of the OCP.
    """

    rounds: int
    replay_time: float
    speed_caps: tuple[np.ndarray, ...]
    cap_margin: float
    accepted: bool
    feasible: bool = False


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
    """Return the per-node cap implied by one replay's achieved speed.

    Only a node where the replay is slower than the OCP tightens the cap. A
    faster replay is evidence of inconsistency, not permission to reduce the
    OCP speed, so those nodes keep the OCP speed and rely on the two-sided
    acceptance check instead of a needless cap reduction.
    """
    replay_speed = np.interp(mesh, replay.distance, replay.speed)
    return np.where(
        replay_speed < ocp_speed, replay_speed - margin, ocp_speed,
    )


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
    """Return true for a feasible replay that agrees with the OCP time.

    The comparison is two-sided: the replay must reproduce the OCP lap time
    within the relative tolerance. A replay that is materially faster is
    evidence that the OCP is suboptimal or that the adapter is inconsistent, so
    it is not accepted silently.
    """
    if not bool(sample.feasible):
        return False
    ocp_time = float(solution.lap_time)
    replay_time = float(sample.lap_time)
    if not (math.isfinite(ocp_time) and ocp_time > 0.0):
        return False
    if not (math.isfinite(replay_time) and replay_time > 0.0):
        return False
    return abs(replay_time - ocp_time) <= tolerance * ocp_time


def correct_to_replay(track, corridor, surface, mesh, spline, settings, controller,
                      replay: ReplayProtocol, *, tolerance=0.01,
                      maximum_rounds=5) -> CorrectionResult:
    """Re-solve the spline line until its replay is feasible and matches.

    The first round solves without a speed cap.  A failed round whose replay is
    outside the two-sided tolerance or infeasible lowers the per-node cap at
    each node where the replay speed is below the OCP speed to
    ``v_replay - CAP_MARGIN``; nodes where the replay is faster keep ``v_ocp``.
    The resulting cap is never above the previous cap, and it is floored at the
    solver minimum for representability.  The loop re-solves with that cap and
    stops after ``maximum_rounds`` solve attempts, or as soon as a replay is
    feasible and

    ``abs(replay_time - ocp_time) <= tolerance * ocp_time``.

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
    last_feasible = False

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
        last_feasible = bool(sample.feasible)

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
        feasible=last_feasible,
    )
