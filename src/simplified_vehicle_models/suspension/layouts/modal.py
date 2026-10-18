"""Stateless modal force network and the two mode matrices built on it.

A modal layout does not put a spring on each wheel. It puts springs and dampers
on *modes* of the four wheel jounces, ``m = T z``, with ``T`` an orthonormal
4x4 matrix (``T T^T = I``). Each mode ``k`` carries a linear rate ``K_k`` and a
damper ``D_k``, and the corner forces follow by virtual work:

    F_corner = T^T ( K (T z) + D(T zdot) )  +  preload.

Conventions
-----------
* Wheel order FL, FR, RL, RR; jounce positive in compression; force positive in
  compression.
* Mode rates are **wheel-referred**: a mode rate ``K`` is the force per unit of
  the orthonormal mode coordinate, and the same rate on all four modes is four
  independent wheel springs of rate ``K``. No motion ratio enters.
* A damper on a mode sees the wheel-referred mode velocity ``T zdot`` with no
  extra motion ratio. This changes how a damper table is read compared with the
  independent-corner layout, where the table sees damper-shaft velocity.
* Zero jounce carries the sprung static corner loads as a constant ``preload``
  taken from the kinematics, so the mode springs act about the design ride
  height.

Limitations
-----------
Modal decoupling is exact only for linear elements. The mode springs here are
linear: there is no bump stop or droop stop. The travel limits stay in the
kinematics and are checked by the run gates. A nonlinear mode element would be a
modelling choice, not a decoupling result.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics, _four
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces

#: Tolerance of the orthonormality check ``T T^T = I``.
ORTHONORMAL_ATOL = 1.0e-12

_HALF = 0.5
_ROOT_HALF = float(np.sqrt(0.5))

#: Four-mode matrix, rows heave, roll, pitch, warp; columns FL, FR, RL, RR.
FOUR_MODE_NAMES = ("heave", "roll", "pitch", "warp")
FOUR_MODE_MATRIX = _HALF * np.array(
    [
        [1.0, 1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0, -1.0],
        [1.0, 1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0, 1.0],
    ]
)

#: Per-axle matrix, rows front heave, front roll, rear heave, rear roll.
AXLE_MODE_NAMES = ("heave-front", "roll-front", "heave-rear", "roll-rear")
AXLE_MODE_MATRIX = _ROOT_HALF * np.array(
    [
        [1.0, 1.0, 0.0, 0.0],
        [1.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 1.0],
        [0.0, 0.0, 1.0, -1.0],
    ]
)


def check_orthonormal(matrix) -> np.ndarray:
    """Return ``matrix`` as a float (4, 4) array after checking ``T T^T = I``."""

    array = np.asarray(matrix, dtype=float)
    if array.shape != (4, 4) or not np.all(np.isfinite(array)):
        raise ValueError("the mode matrix must be a finite 4x4 matrix")
    error = float(np.max(np.abs(array @ array.T - np.eye(4))))
    if error > ORTHONORMAL_ATOL:
        raise ValueError(
            f"the mode matrix is not orthonormal: max |T T^T - I| = {error:.3e}"
        )
    return array


@dataclass(frozen=True, eq=False)
class ModalNetwork:
    """Linear springs and table dampers on the modes ``T z`` of four wheels."""

    kinematics: SuspensionKinematics
    transform: np.ndarray  # (4, 4), rows are modes, T T^T = I
    mode_names: tuple[str, ...]
    rates: np.ndarray  # (4,) wheel-referred mode rates [N/m]
    dampers: tuple  # four objects with ``force(velocity)``

    n_states = 0
    has_inertia = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "transform", check_orthonormal(self.transform))
        rates = np.asarray(self.rates, dtype=float)
        if rates.shape != (4,) or not np.all(np.isfinite(rates)):
            raise ValueError("rates must hold four finite mode rates")
        if np.any(rates < 0.0):
            raise ValueError(
                "mode rates must be nonnegative; got "
                + ", ".join(f"{n}={r:g}" for n, r in zip(self.mode_names, rates))
            )
        object.__setattr__(self, "rates", rates)
        if len(self.mode_names) != 4 or len(self.dampers) != 4:
            raise ValueError("a modal network needs four mode names and four dampers")
        object.__setattr__(self, "dampers", tuple(self.dampers))

    # -- configuration ----------------------------------------------------------
    @property
    def preload(self) -> np.ndarray:
        """Return the sprung static corner loads the mode springs act about (N)."""

        return np.asarray(self.kinematics.static_sprung_load, dtype=float)

    @property
    def stiffness_matrix(self) -> np.ndarray:
        """Return the 4x4 corner stiffness ``T^T diag(K) T`` (N/m)."""

        return self.transform.T @ (self.rates[:, None] * self.transform)

    def with_kinematics(self, kinematics: SuspensionKinematics) -> ModalNetwork:
        return replace(self, kinematics=kinematics)

    def with_rates(self, rates) -> ModalNetwork:
        """Return a copy with other mode rates (validated again)."""

        return replace(self, rates=np.asarray(rates, dtype=float))

    def with_dampers(self, dampers) -> ModalNetwork:
        """Return a copy with four other mode dampers."""

        return replace(self, dampers=tuple(dampers))

    def initial_state(self, z_static=None) -> np.ndarray:
        return np.zeros(0)

    # -- forces -----------------------------------------------------------------
    def forces(self, jounce, jounce_rate, state=None) -> NetworkForces:
        """Return the wheel-referred preload, spring and damper forces."""

        z = _four("jounce", jounce)
        zdot = _four("jounce_rate", jounce_rate)
        mode = self.transform @ z
        velocity = self.transform @ zdot
        damper_mode = np.empty(4)
        flag = False
        for index, damper in enumerate(self.dampers):
            result = damper.force(velocity[index:index + 1])
            damper_mode[index] = float(np.asarray(result.force, dtype=float)[0])
            flag = flag or bool(np.any(np.asarray(result.out_of_range, dtype=bool)))
        spring = self.transform.T @ (self.rates * mode)
        damper = self.transform.T @ damper_mode
        preload = self.preload
        # The run gates compare the largest damper velocity with the table limit.
        # The mode velocity can exceed every wheel velocity, so report it on all
        # four corners rather than the (smaller) corner-referred share.
        peak = float(np.max(np.abs(velocity)))
        return NetworkForces(
            total=preload + spring + damper,
            elements={"preload": preload.copy(), "spring": spring, "damper": damper},
            damper_velocity=np.full(4, peak),
            out_of_range=np.full(4, flag, dtype=bool),
        )

    def static_forces(self, jounce, state=None) -> np.ndarray:
        """Return the rate-free corner forces: preload plus the mode springs."""

        z = _four("jounce", jounce)
        return self.preload + self.transform.T @ (self.rates * (self.transform @ z))

    def static_check(self, jounce=None) -> np.ndarray:
        z = np.zeros(4) if jounce is None else jounce
        return self.static_forces(z)

    # -- inertia ----------------------------------------------------------------
    def inertia_block(self, jounce) -> np.ndarray:
        return np.zeros((4, 4))

    def inertial_forcing(self, jounce, jounce_rate) -> np.ndarray:
        return np.zeros(4)


def build_modal(
    *,
    layout: str,
    kinematics: SuspensionKinematics,
    transform,
    mode_names,
    rates,
    dampers,
) -> CompiledSuspension:
    """Assemble a modal suspension and check that zero jounce is static equilibrium."""

    network = ModalNetwork(
        kinematics=kinematics,
        transform=transform,
        mode_names=tuple(mode_names),
        rates=np.asarray(rates, dtype=float),
        dampers=tuple(dampers),
    )
    expected = kinematics.static_sprung_load
    actual = network.static_check()
    if not np.allclose(actual, expected, rtol=1.0e-9, atol=0.0):
        raise ValueError(
            "the modal layout does not carry the sprung static corner loads at zero "
            f"jounce: got {actual.tolist()} N, expected {expected.tolist()} N"
        )
    return CompiledSuspension(kinematics=kinematics, network=network, layout=layout)
