"""Axle roll/heave layout: a decoupled heave and roll spring and damper per axle.

For each axle the two wheels split into

    heave = (zL + zR) / sqrt(2)
    roll  = (zL - zR) / sqrt(2)

with a wheel-referred rate and a damper on each. The two axles do not couple.
``motion_ratios`` are wheel travel / damper-shaft travel in front heave, front
roll, rear heave, rear roll order. Each modal damper uses the shaft/mode
Jacobian ``1/(sqrt(2)*MR)`` for velocity and virtual-work force.
A roll rate equal to the corner rate gives two independent wheel springs; a roll
rate of ``k_corner + k_arb`` reproduces corner springs plus an anti-roll bar of
wheel rate ``k_arb``, because the bar's ``0.5 k_arb (zL - zR)`` is exactly the
roll mode's ``k_arb`` on the orthonormal roll coordinate.

See ``modal.py`` for the conventions and the linearity limit.
"""

from __future__ import annotations

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.modal import (
    AXLE_MODE_MATRIX,
    AXLE_MODE_NAMES,
    build_modal,
)
from simplified_vehicle_models.suspension.network import CompiledSuspension

LAYOUT = "axle-roll-heave"


def build_axle_roll_heave(
    *,
    kinematics: SuspensionKinematics,
    heave_rate_front: float,
    roll_rate_front: float,
    heave_rate_rear: float,
    roll_rate_rear: float,
    heave_damper_front,
    roll_damper_front,
    heave_damper_rear,
    roll_damper_rear,
    motion_ratios=(1.0, 1.0, 1.0, 1.0),
) -> CompiledSuspension:
    """Assemble the axle roll/heave suspension."""

    if not (heave_rate_front > 0.0 and heave_rate_rear > 0.0):
        raise ValueError("the heave rate of each axle must be positive")
    ratios = tuple(float(value) for value in motion_ratios)
    if len(ratios) != 4 or any(not (value > 0.0) for value in ratios):
        raise ValueError("motion_ratios must contain four positive heave/roll ratios")
    compiled = build_modal(
        layout=LAYOUT,
        kinematics=kinematics,
        transform=AXLE_MODE_MATRIX,
        mode_names=AXLE_MODE_NAMES,
        rates=(heave_rate_front, roll_rate_front, heave_rate_rear, roll_rate_rear),
        dampers=(heave_damper_front, roll_damper_front, heave_damper_rear, roll_damper_rear),
    )
    # DSS ratio means wheel travel / damper-shaft travel, matching the
    # independent-corner convention. The orthonormal mode's wheel travel is
    # q/sqrt(2), so shaft/mode Jacobian is 1/(sqrt(2)*MR).
    network = compiled.network.with_damper_velocity_factors(
        tuple(1.0 / (value * 2**0.5) for value in ratios)
    )
    return CompiledSuspension(kinematics=compiled.kinematics, network=network, layout=compiled.layout)
