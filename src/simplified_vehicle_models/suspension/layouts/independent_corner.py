"""Independent-corner layout: a spring and a damper per wheel and one ARB per axle.

This is the original 14DoF suspension law, moved behind the force-network
interface without a change of arithmetic. Per wheel it adds

* the wheel-referred spring curve at the wheel jounce,
* the damper table at the damper velocity ``zdot / MR`` returned to the wheel
  through ``/ MR``, and
* an anti-roll bar of ``0.5 * k * (zL - zR)`` acting oppositely on the two
  wheels of an axle.

The only coupling between wheels is the ARB. An inerter of ``b`` kg at the
damper adds ``b / MR^2`` to the jounce diagonal of the mass matrix.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.suspension.elements import AxleArbRates, CornerSprings
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics, _four
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces

LAYOUT = "independent-corner"
#: Relative tolerance of the compile-time static check.
STATIC_CHECK_RTOL = 1.0e-9
_AXLES = ((0, (0, 1)), (1, (2, 3)))


@dataclass(frozen=True)
class IndependentCornerNetwork:
    """Spring, ARB, damper and inerter law of independent corners."""

    kinematics: SuspensionKinematics
    springs: CornerSprings
    arb: AxleArbRates
    dampers: tuple  # (front, rear), each with ``force(velocity)``
    inertance: tuple[float, float]  # (front, rear) [kg]

    n_states = 0

    def __post_init__(self) -> None:
        if len(self.dampers) != 2:
            raise ValueError("dampers must be (front, rear)")
        for value in self.inertance:
            if not (np.isfinite(value) and value >= 0.0):
                raise ValueError("inertance must be finite and nonnegative")

    # -- configuration ----------------------------------------------------------
    @property
    def has_inertia(self) -> bool:
        return any(value > 0.0 for value in self.inertance)

    def with_kinematics(self, kinematics: SuspensionKinematics) -> IndependentCornerNetwork:
        return replace(self, kinematics=kinematics)

    def initial_state(self, z_static=None) -> np.ndarray:
        return np.zeros(0)

    # -- forces -----------------------------------------------------------------
    def forces(self, jounce, jounce_rate, state=None) -> NetworkForces:
        """Return the wheel-referred spring, ARB and damper forces of four corners.

        A jounce outside the design travel raises ``DomainError``.
        """

        z = _four("jounce", jounce)
        zdot = _four("jounce_rate", jounce_rate)
        spring = np.empty(4)
        arb = np.empty(4)
        damper = np.empty(4)
        velocity = np.empty(4)
        flag = np.zeros(4, dtype=bool)
        for axle, (left, right) in _AXLES:
            pair = [left, right]
            zz = z[pair]
            spring[pair] = self.springs.axle(axle)(zz)
            half = 0.5 * self.arb.axle(axle) * (zz[0] - zz[1])
            arb[left], arb[right] = half, -half
            ratio = self.kinematics.axle(axle).motion_ratio(zz)
            v = zdot[pair] / ratio
            result = self.dampers[axle].force(v)
            velocity[pair] = v
            damper[pair] = np.asarray(result.force, dtype=float) / ratio
            flag[pair] = np.asarray(result.out_of_range, dtype=bool)
        return NetworkForces(
            total=spring + arb + damper,
            elements={"spring": spring, "arb": arb, "damper": damper},
            damper_velocity=velocity,
            out_of_range=flag,
        )

    def static_forces(self, jounce, state=None) -> np.ndarray:
        """Return the spring and ARB force of four corners (no rate-dependent term)."""

        z = _four("jounce", jounce)
        result = np.empty(4)
        for axle, (left, right) in _AXLES:
            pair = z[[left, right]]
            result[[left, right]] = self.springs.axle(axle)(pair)
            arb = 0.5 * self.arb.axle(axle) * (pair[0] - pair[1])
            result[left] += arb
            result[right] -= arb
        return result

    def static_check(self, jounce=None) -> np.ndarray:
        """Return the rate-free corner forces at zero jounce (or at ``jounce``)."""

        z = np.zeros(4) if jounce is None else jounce
        return self.static_forces(z)

    # -- inertia ----------------------------------------------------------------
    def inertia_block(self, jounce) -> np.ndarray:
        """Return the diagonal ``b / MR^2`` the inerters add to the jounce rows."""

        z = _four("jounce", jounce)
        block = np.zeros((4, 4))
        for i in range(4):
            inertance = self.inertance[i // 2]
            if inertance > 0.0:
                ratio = self.kinematics.motion_ratio(i, z[i])
                block[i, i] = inertance / ratio**2
        return block

    def inertial_forcing(self, jounce, jounce_rate) -> np.ndarray:
        """Return the right-hand-side term of a jounce-dependent inertia.

        The inerter has kinetic energy ``0.5 b_eff(j) jdot^2`` with
        ``b_eff = b / MR(j)^2``. Lagrange gives ``b_eff jddot + 0.5 b_eff' jdot^2 = Q``,
        so ``-0.5 b_eff' jdot^2`` moves to the right-hand side. It is zero for a
        constant motion ratio.
        """

        z = _four("jounce", jounce)
        zdot = _four("jounce_rate", jounce_rate)
        term = np.zeros(4)
        for i in range(4):
            inertance = self.inertance[i // 2]
            if inertance > 0.0:
                ratio = self.kinematics.motion_ratio(i, z[i])
                slope = self.kinematics.motion_ratio_slope(i, z[i])
                b_slope = -2.0 * inertance * slope / ratio**3
                term[i] = -0.5 * b_slope * zdot[i] ** 2
        return term

    def with_inertance(self, front: float, rear: float) -> IndependentCornerNetwork:
        """Return a copy with other inerter values."""

        return replace(self, inertance=(float(front), float(rear)))

    def with_arb(self, front: float, rear: float) -> IndependentCornerNetwork:
        """Return a copy with other ARB wheel rates."""

        return replace(self, arb=AxleArbRates(float(front), float(rear)))

    def with_dampers(self, front, rear) -> IndependentCornerNetwork:
        """Return a copy with other dampers."""

        return replace(self, dampers=(front, rear))


def build_independent_corner(
    *,
    kinematics: SuspensionKinematics,
    springs: CornerSprings,
    arb: AxleArbRates,
    damper_front,
    damper_rear,
    inertance_front: float,
    inertance_rear: float,
) -> CompiledSuspension:
    """Assemble the independent-corner suspension and check its static loads.

    Zero jounce must carry the sprung static corner loads the kinematics names.
    """

    network = IndependentCornerNetwork(
        kinematics=kinematics, springs=springs, arb=arb,
        dampers=(damper_front, damper_rear),
        inertance=(float(inertance_front), float(inertance_rear)),
    )
    expected = kinematics.static_sprung_load
    actual = network.static_check()
    if not np.allclose(actual, expected, rtol=STATIC_CHECK_RTOL, atol=0.0):
        raise ValueError(
            "the spring curves do not carry the sprung static corner loads at zero "
            f"jounce: got {actual.tolist()} N, expected {expected.tolist()} N"
        )
    return CompiledSuspension(kinematics=kinematics, network=network, layout=LAYOUT)
