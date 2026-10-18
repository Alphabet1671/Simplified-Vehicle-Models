"""Stateless modal force network and the two mode matrices built on it.

A modal layout places linear springs and table dampers on modes ``m = T z`` of
four wheel jounces, with ``T`` orthonormal.  Modal damper tables may be connected
through a travel Jacobian: shaft velocity is ``J * mode_velocity`` and virtual
work gives generalized mode force ``J * shaft_force``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics, _four
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces

ORTHONORMAL_ATOL = 1.0e-12
_HALF = 0.5
_ROOT_HALF = float(np.sqrt(0.5))
FOUR_MODE_NAMES = ("heave", "roll", "pitch", "warp")
FOUR_MODE_MATRIX = _HALF * np.array([
    [1.0, 1.0, 1.0, 1.0], [1.0, -1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0, -1.0], [1.0, -1.0, -1.0, 1.0],
])
AXLE_MODE_NAMES = ("heave-front", "roll-front", "heave-rear", "roll-rear")
AXLE_MODE_MATRIX = _ROOT_HALF * np.array([
    [1.0, 1.0, 0.0, 0.0], [1.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 1.0], [0.0, 0.0, 1.0, -1.0],
])


def check_orthonormal(matrix) -> np.ndarray:
    array = np.asarray(matrix, dtype=float)
    if array.shape != (4, 4) or not np.all(np.isfinite(array)):
        raise ValueError("the mode matrix must be a finite 4x4 matrix")
    error = float(np.max(np.abs(array @ array.T - np.eye(4))))
    if error > ORTHONORMAL_ATOL:
        raise ValueError(f"the mode matrix is not orthonormal: max |T T^T - I| = {error:.3e}")
    return array


@dataclass(frozen=True, eq=False)
class ModalNetwork:
    """Linear springs and table dampers on the modes ``T z`` of four wheels."""

    kinematics: SuspensionKinematics
    transform: np.ndarray
    mode_names: tuple[str, ...]
    rates: np.ndarray
    dampers: tuple
    damper_velocity_factors: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)

    n_states = 0
    has_inertia = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "transform", check_orthonormal(self.transform))
        rates = np.asarray(self.rates, dtype=float)
        if rates.shape != (4,) or not np.all(np.isfinite(rates)):
            raise ValueError("rates must hold four finite mode rates")
        if np.any(rates < 0.0):
            raise ValueError("mode rates must be nonnegative; got " + ", ".join(
                f"{n}={r:g}" for n, r in zip(self.mode_names, rates)))
        object.__setattr__(self, "rates", rates)
        if len(self.mode_names) != 4 or len(self.dampers) != 4:
            raise ValueError("a modal network needs four mode names and four dampers")
        object.__setattr__(self, "dampers", tuple(self.dampers))
        factors = np.asarray(self.damper_velocity_factors, dtype=float)
        if factors.shape != (4,) or not np.all(np.isfinite(factors)) or np.any(factors <= 0.0):
            raise ValueError("damper_velocity_factors must contain four positive finite values")
        object.__setattr__(self, "damper_velocity_factors", tuple(float(x) for x in factors))

    @property
    def preload(self) -> np.ndarray:
        return np.asarray(self.kinematics.static_sprung_load, dtype=float)

    @property
    def stiffness_matrix(self) -> np.ndarray:
        return self.transform.T @ (self.rates[:, None] * self.transform)

    def with_kinematics(self, kinematics: SuspensionKinematics) -> ModalNetwork:
        return replace(self, kinematics=kinematics)

    def with_rates(self, rates) -> ModalNetwork:
        return replace(self, rates=np.asarray(rates, dtype=float))

    def with_dampers(self, dampers) -> ModalNetwork:
        return replace(self, dampers=tuple(dampers))

    def with_damper_velocity_factors(self, factors) -> ModalNetwork:
        return replace(self, damper_velocity_factors=tuple(factors))

    def initial_state(self, z_static=None) -> np.ndarray:
        return np.zeros(0)

    def forces(self, jounce, jounce_rate, state=None) -> NetworkForces:
        z, zdot = _four("jounce", jounce), _four("jounce_rate", jounce_rate)
        mode, mode_velocity = self.transform @ z, self.transform @ zdot
        shaft_velocity = np.empty(4)
        damper_mode = np.empty(4)
        flag = False
        for i, damper in enumerate(self.dampers):
            factor = self.damper_velocity_factors[i]
            shaft_velocity[i] = factor * mode_velocity[i]
            result = damper.force(shaft_velocity[i:i + 1])
            damper_mode[i] = factor * float(np.asarray(result.force, dtype=float)[0])
            flag = flag or bool(np.any(np.asarray(result.out_of_range, dtype=bool)))
        spring = self.transform.T @ (self.rates * mode)
        damper = self.transform.T @ damper_mode
        preload = self.preload
        peak = float(np.max(np.abs(shaft_velocity)))
        return NetworkForces(
            total=preload + spring + damper,
            elements={"preload": preload.copy(), "spring": spring, "damper": damper},
            damper_velocity=np.full(4, peak), out_of_range=np.full(4, flag, dtype=bool),
        )

    def static_forces(self, jounce, state=None) -> np.ndarray:
        z = _four("jounce", jounce)
        return self.preload + self.transform.T @ (self.rates * (self.transform @ z))

    def static_check(self, jounce=None) -> np.ndarray:
        return self.static_forces(np.zeros(4) if jounce is None else jounce)

    def inertia_block(self, jounce) -> np.ndarray:
        return np.zeros((4, 4))

    def inertial_forcing(self, jounce, jounce_rate) -> np.ndarray:
        return np.zeros(4)


def build_modal(*, layout: str, kinematics: SuspensionKinematics, transform,
                mode_names, rates, dampers) -> CompiledSuspension:
    network = ModalNetwork(kinematics, transform, tuple(mode_names), np.asarray(rates, dtype=float), tuple(dampers))
    expected, actual = kinematics.static_sprung_load, network.static_check()
    if not np.allclose(actual, expected, rtol=1e-9, atol=0.0):
        raise ValueError("the modal layout does not carry the sprung static corner loads at zero "
                         f"jounce: got {actual.tolist()} N, expected {expected.tolist()} N")
    return CompiledSuspension(kinematics=kinematics, network=network, layout=layout)
