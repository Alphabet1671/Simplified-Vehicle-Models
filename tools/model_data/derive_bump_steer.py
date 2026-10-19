"""Derive bump-steer (toe versus jounce) curves from suspension hardpoints.

Each corner is a double wishbone with a tie rod. The upright is one rigid body:
its upper and lower outer ball joints sit at fixed distances from the two
inboard pivots of their wishbones, and its tie-rod joint sits at a fixed distance
from the tie-rod inner joint (the rack or toe link does not move at zero steer).
That is five constraints on the six degrees of freedom of the upright. Prescribing
the vertical travel of the wheel centre closes the system, which this tool
solves by continuation from the design position.

The reported toe is the change in the horizontal heading of the wheel's spin axis,
toe-in positive, which is the repository's per-wheel ``jounce_steer`` curve (a
left wheel adds it, a right wheel subtracts it). The camber change comes out of
the same solve and is reported as a cross-check.

The hardpoints are read from a workbook given on the command line (real car
data). Blocks and rows are found by label: ``Front Right Hardpoints`` and
``Rear Right Hardpoints`` open two blocks whose rows are ``uca front``,
``uca rear``, ``lca front``, ``lca rear``, ``tierod inner``, ``uca outer``,
``lca outer``, ``tierod outer`` and ``wheel center``, each followed by X, Y, Z in
the next three columns. The coordinate system is the CAD one: x forward, y to the
right, z DOWN, in inches (use ``--units-per-metre`` for others). The points are
taken to be the design (static ride height) position, so jounce is measured from
there and zero jounce has exactly zero toe change, whatever static toe the car
carries.

Neglected: compliance, the rack moving with steering, and any rotation of the
wheel about its spin axis. Jounce is the vertical travel of the wheel centre.

Usage::

    PYTHONPATH=src:. python3 tools/model_data/derive_bump_steer.py \\
        --workbook <hardpoint workbook> --sheet <sheet name> \\
        --rebound-mm 31.8 --compression-mm 31.8
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

from tools.model_data.derive_steering_lift import (
    INCH,
    SteeringLiftError,
    read_hardpoint_block,
)

FRONT_HEADER = "Front Right Hardpoints"
REAR_HEADER = "Rear Right Hardpoints"
LABELS = ("uca front", "uca rear", "lca front", "lca rear", "tierod inner",
          "uca outer", "lca outer", "tierod outer", "wheel center")
#: Largest jounce increment (m) between solves; each solve is a Newton polish.
MAX_STEP = 5.0e-4


class BumpSteerError(SteeringLiftError):
    """Raised when the hardpoints are inconsistent or the solve fails."""


@dataclass(frozen=True)
class Corner:
    """Right-hand corner hardpoints in metres: x forward, y right, z DOWN."""

    uca_front: np.ndarray
    uca_rear: np.ndarray
    lca_front: np.ndarray
    lca_rear: np.ndarray
    tierod_inner: np.ndarray
    uca_outer: np.ndarray
    lca_outer: np.ndarray
    tierod_outer: np.ndarray
    wheel_centre: np.ndarray

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise BumpSteerError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, value)

    def mirrored(self) -> "Corner":
        """Return the left-hand corner (``y`` negated)."""

        flip = np.array([1.0, -1.0, 1.0])
        return Corner(**{name: getattr(self, name)*flip for name in self.__dataclass_fields__})

    # The five fixed distances, each as (moving point, fixed point).
    def constraints(self):
        return ((self.uca_outer, self.uca_front), (self.uca_outer, self.uca_rear),
                (self.lca_outer, self.lca_front), (self.lca_outer, self.lca_rear),
                (self.tierod_outer, self.tierod_inner))


def _rotation_vector(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-14:
        return np.eye(3) + _skew(vector)
    k = _skew(vector/angle)
    return np.eye(3) + np.sin(angle)*k + (1.0 - np.cos(angle))*(k @ k)


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _residual(corner: Corner, q: np.ndarray, rise: float) -> np.ndarray:
    """Constraint residuals for upright pose ``q`` = (rotation vector, translation)."""

    rotation, shift = _rotation_vector(q[:3]), q[3:]
    out = np.empty(6)
    for i, (moving, fixed) in enumerate(corner.constraints()):
        length = np.linalg.norm(moving - fixed)
        out[i] = np.linalg.norm(rotation @ moving + shift - fixed) - length
    # z is DOWN: the wheel centre rises by ``rise`` when its z falls by ``rise``.
    out[5] = (rotation @ corner.wheel_centre + shift)[2] - corner.wheel_centre[2] + rise
    return out


def _solve(corner: Corner, q: np.ndarray, rise: float) -> np.ndarray:
    """Newton solve with a central-difference Jacobian, starting from ``q``."""

    q = q.copy()
    for _ in range(40):
        f = _residual(corner, q, rise)
        if np.max(np.abs(f)) < 1.0e-13:
            return q
        jac = np.empty((6, 6))
        for j in range(6):
            step = np.zeros(6)
            step[j] = 1.0e-7
            jac[:, j] = (_residual(corner, q + step, rise) - _residual(corner, q - step, rise))/2.0e-7
        try:
            q = q - np.linalg.solve(jac, f)
        except np.linalg.LinAlgError as error:
            raise BumpSteerError("singular upright constraint system") from error
    if np.max(np.abs(_residual(corner, q, rise))) > 1.0e-10:
        raise BumpSteerError(f"upright solve did not converge at jounce {rise:.4f} m")
    return q


def _angles(q: np.ndarray, side: float = 1.0) -> tuple[float, float]:
    """Return (toe-in, camber) in radians for the right-hand wheel at pose ``q``.

    The spin axis ``s`` points outward (+y). A right turn of the wheel by psi
    turns it to ``(-sin psi, cos psi)`` in the ground plane, so psi is its yaw and
    the right wheel's toe-in is ``-psi``. Camber is positive with the top of the
    wheel outward, which tilts the outward spin axis toward +z (down).
    ``side`` is +1 for a right-hand corner and -1 for a left-hand one, whose
    outward spin axis points along -y; its mirror image gives the same toe-in.
    """

    s = _rotation_vector(q[:3]) @ np.array([0.0, side, 0.0])
    s = s*np.array([1.0, side, 1.0])
    yaw = np.arctan2(-s[0], s[1])
    camber = np.arctan2(s[2], np.hypot(s[0], s[1]))
    return float(-yaw), float(camber)


def bump_steer(corner: Corner, jounce) -> tuple[np.ndarray, np.ndarray]:
    """Return (toe-in [rad], camber change [rad]) at each jounce [m, compression +].

    Zero jounce gives exactly zero for both. Each side of zero is marched from the
    design position in steps no larger than ``MAX_STEP`` so the solution stays on
    the physical branch.
    """

    jounce = np.atleast_1d(np.asarray(jounce, dtype=float))
    if not np.all(np.isfinite(jounce)):
        raise BumpSteerError("jounce must be finite")
    side = 1.0 if corner.wheel_centre[1] >= 0.0 else -1.0
    toe = np.zeros(jounce.shape)
    camber = np.zeros(jounce.shape)
    for sign in (1.0, -1.0):
        picks = np.flatnonzero(sign*jounce > 0.0)
        order = picks[np.argsort(sign*jounce[picks])]
        q, previous = np.zeros(6), 0.0
        for index in order:
            target = jounce[index]
            count = max(1, int(np.ceil(abs(target - previous)/MAX_STEP)))
            for value in np.linspace(previous, target, count + 1)[1:]:
                q = _solve(corner, q, value)
            previous = target
            toe[index], camber[index] = _angles(q, side)
    return toe, camber


def corner_from_block(found: dict[str, np.ndarray]) -> Corner:
    keys = {label: label.replace(" ", "_") for label in LABELS}
    keys["wheel center"] = "wheel_centre"
    return Corner(**{keys[label]: found[label] for label in LABELS})


def read_corners(workbook_path: str, sheet: str, units_per_metre: float):
    """Return the (front, rear) right-hand corners from a hardpoint workbook."""

    return tuple(corner_from_block(read_hardpoint_block(workbook_path, sheet, header, LABELS, units_per_metre))
                 for header in (FRONT_HEADER, REAR_HEADER))


def knot_grid(rebound_mm: float, compression_mm: float, segments: int = 3) -> np.ndarray:
    """Return knots in mm, equal spacing each side of zero, with both travel limits."""

    if not (rebound_mm > 0.0 and compression_mm > 0.0 and segments >= 1):
        raise BumpSteerError("travel limits must be positive")
    below = np.linspace(-rebound_mm, 0.0, segments + 1)
    above = np.linspace(0.0, compression_mm, segments + 1)[1:]
    return np.round(np.concatenate((below, above)), 4)


def format_knots(knots_mm: np.ndarray, toe_rad: np.ndarray) -> str:
    """Return ``mm:deg`` pairs for ``--jounce-steer-*``; the 0 mm knot is exactly 0."""

    toe_deg = np.degrees(toe_rad)
    toe_deg = np.where(knots_mm == 0.0, 0.0, toe_deg)
    return ",".join(f"{k:g}:{t:.6f}" for k, t in zip(knots_mm, toe_deg))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workbook", required=True)
    parser.add_argument("--sheet", required=True)
    parser.add_argument("--rebound-mm", type=float, required=True, help="travel limit in rebound")
    parser.add_argument("--compression-mm", type=float, required=True, help="travel limit in compression")
    parser.add_argument("--segments", type=int, default=3, help="knot segments each side of zero")
    parser.add_argument("--units-per-metre", type=float, default=1.0/INCH)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        front, rear = read_corners(args.workbook, args.sheet, args.units_per_metre)
        knots = knot_grid(args.rebound_mm, args.compression_mm, args.segments)
        dense = np.linspace(-args.rebound_mm, args.compression_mm, 65)*1.0e-3
        lines = []
        for name, corner in (("front", front), ("rear", rear)):
            toe, camber = bump_steer(corner, knots*1.0e-3)
            sweep_toe, _ = bump_steer(corner, dense)
            slope = np.polyfit(dense, sweep_toe, 1)[0]
            print(f"{name}: toe-in range {np.degrees(toe.min()):+.4f} .. {np.degrees(toe.max()):+.4f} deg;"
                  f" mean slope {np.degrees(slope)/1000.0:+.5f} deg/mm;"
                  f" camber gain at 0 {np.degrees(np.gradient(camber, knots*1e-3)[len(knots)//2]):+.2f} deg/m")
            lines.append(f"--jounce-steer-{name} {format_knots(knots, toe)}")
    except (SteeringLiftError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
