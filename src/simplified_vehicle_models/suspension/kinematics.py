"""Suspension kinematics: the per-wheel geometry of a vertical (14DoF) car.

Kinematics holds geometry only and no force law. A force network (see
``network.py``) turns jounces into corner forces; the kinematics says how the
wheel moves relative to the body and where the link forces act.

Wheel order is FL, FR, RL, RR and axle order is front (0), rear (1). Jounce is
positive in compression. Toe follows the repository's positive-right road-wheel
convention: a toe-in on the left wheel is a positive angle and a toe-in on the
right wheel is a negative angle.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

PRELOAD_REFERENCE = "static-equilibrium"
WHEELS_PER_AXLE = 2


def axle_of(wheel: int) -> int:
    """Return the axle index (0 front, 1 rear) of one wheel index."""

    return int(wheel) // WHEELS_PER_AXLE


@dataclass(frozen=True)
class AxleGeometry:
    """Geometry of one axle (both wheels are mirror images)."""

    motion_ratio: MonotoneSpline
    roll_centre_height: MonotoneSpline
    roll_centre_lateral: MonotoneSpline
    anti_brake: float
    anti_drive: float
    camber_gain: float
    static_toe: float
    jounce_steer: MonotoneSpline
    jounce_limits: tuple[float, float]
    static_sprung_load: float
    steering_lift_sine_left: float = 0.0
    steering_lift_cosine_left: float = 0.0
    steering_lift_sine_right: float = 0.0
    steering_lift_cosine_right: float = 0.0

    def __post_init__(self) -> None:
        low, high = self.jounce_limits
        if not (np.isfinite(low) and np.isfinite(high) and low < 0.0 < high):
            raise ValueError("jounce_limits must be finite with low < 0 < high")
        for name in ("anti_brake", "anti_drive"):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be a fraction from zero through one")
        for name in ("camber_gain", "static_toe"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        for name in ("steering_lift_sine_left", "steering_lift_cosine_left",
                     "steering_lift_sine_right", "steering_lift_cosine_right"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if not (np.isfinite(self.static_sprung_load) and self.static_sprung_load > 0.0):
            raise ValueError("static_sprung_load must be finite and positive")
        for name in ("motion_ratio", "jounce_steer"):
            spline = getattr(self, name)
            if spline.x[0] != low or spline.x[-1] != high:
                raise ValueError(f"{name} must span exactly the jounce limits")
        steer = self.jounce_steer
        index = np.flatnonzero(steer.x == 0.0)
        if index.size != 1 or steer.y[index[0]] != 0.0:
            raise ValueError("jounce_steer needs an explicit 0 m knot with 0 rad toe change")
        if np.any(self.motion_ratio.y <= 0.0):
            raise ValueError("damper motion ratio must be positive")
        for name in ("roll_centre_height", "roll_centre_lateral"):
            if getattr(self, name).x[0] != 0.0:
                raise ValueError(f"{name} must start at zero roll")


def _four(name: str, value) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (4,):
        raise ValueError(f"{name} must hold four values in FL, FR, RL, RR order")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return array


@dataclass(frozen=True)
class SuspensionKinematics:
    """Front and rear axle geometry behind wheel- and axle-indexed accessors."""

    front: AxleGeometry
    rear: AxleGeometry
    preload_reference: str = PRELOAD_REFERENCE

    def axle(self, axle: int) -> AxleGeometry:
        """Return the geometry of axle 0 (front) or 1 (rear)."""

        if axle not in (0, 1):
            raise ValueError("axle must be 0 (front) or 1 (rear)")
        return self.front if axle == 0 else self.rear

    def with_axle(self, axle: int, **changes) -> SuspensionKinematics:
        """Return a copy with fields of one axle replaced (validated again)."""

        new = replace(self.axle(axle), **changes)
        return replace(self, front=new) if axle == 0 else replace(self, rear=new)

    # -- wheel-indexed motion ratio ---------------------------------------------
    def motion_ratio(self, wheel: int, jounce: float) -> float:
        """Return the damper motion ratio of one wheel at ``jounce`` (m)."""

        return float(self.axle(axle_of(wheel)).motion_ratio(jounce))

    def motion_ratio_slope(self, wheel: int, jounce: float) -> float:
        """Return the derivative of the motion ratio of one wheel with respect to jounce."""

        return float(self.axle(axle_of(wheel)).motion_ratio.derivative(jounce))

    def motion_ratios(self, jounce) -> np.ndarray:
        """Return the four motion ratios at four jounces."""

        z = _four("jounce", jounce)
        ratios = np.empty(4)
        for axle, pair in ((0, [0, 1]), (1, [2, 3])):
            ratios[pair] = self.axle(axle).motion_ratio(z[pair])
        return ratios

    # -- axle-indexed geometry --------------------------------------------------
    def roll_centre_height(self, axle: int, roll: float) -> float:
        """Return the roll-centre height (m) of one axle at the absolute roll (rad)."""

        return float(self.axle(axle).roll_centre_height(roll))

    def roll_centre_lateral(self, axle: int, roll: float) -> float:
        """Return the roll-centre lateral position (m) of one axle at the absolute roll."""

        return float(self.axle(axle).roll_centre_lateral(roll))

    def anti_brake(self, axle: int) -> float:
        """Return the anti-dive fraction of one axle."""

        return float(self.axle(axle).anti_brake)

    def anti_drive(self, axle: int) -> float:
        """Return the anti-squat fraction of one axle."""

        return float(self.axle(axle).anti_drive)

    def camber_gain(self, axle: int) -> float:
        """Return the camber gain of one axle."""

        return float(self.axle(axle).camber_gain)

    # -- per-wheel arrays -------------------------------------------------------
    def toe(self, jounce) -> np.ndarray:
        """Return static toe plus jounce steer as four road-wheel angle offsets (rad).

        Positive-right convention: FL/RL = +(toe + steer(z)); FR/RR = -(toe + steer(z)).
        """

        z = _four("jounce", jounce)
        angles = np.empty(4)
        for axle, left, right in ((self.front, 0, 1), (self.rear, 2, 3)):
            angles[left] = axle.static_toe + float(axle.jounce_steer(z[left]))
            angles[right] = -(axle.static_toe + float(axle.jounce_steer(z[right])))
        return angles

    def steering_lift(self, angles) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Hub rise and first/second steer derivatives in FL, FR, RL, RR order.

        The signed, measured-geometry coefficients b and a define
        h(delta) = b*sin(delta) + a*(1-cos(delta)). No offset at zero steer.
        The rear may steer but has zero geometry unless configured explicitly.
        """
        delta = _four("steering angles", angles)
        b = np.array([self.front.steering_lift_sine_left,
                      self.front.steering_lift_sine_right,
                      self.rear.steering_lift_sine_left,
                      self.rear.steering_lift_sine_right])
        a = np.array([self.front.steering_lift_cosine_left,
                      self.front.steering_lift_cosine_right,
                      self.rear.steering_lift_cosine_left,
                      self.rear.steering_lift_cosine_right])
        sine, cosine = np.sin(delta), np.cos(delta)
        return b*sine + a*(1.0-cosine), b*cosine + a*sine, -b*sine + a*cosine

    @property
    def jounce_limits(self) -> np.ndarray:
        """Return the ``(low, high)`` jounce limits of each wheel, shape (4, 2)."""

        return np.array([self.front.jounce_limits] * 2 + [self.rear.jounce_limits] * 2,
                        dtype=float)

    @property
    def static_sprung_load(self) -> np.ndarray:
        """Return the sprung static corner loads (N) the design ride height carries."""

        return np.array([self.front.static_sprung_load] * 2
                        + [self.rear.static_sprung_load] * 2, dtype=float)
