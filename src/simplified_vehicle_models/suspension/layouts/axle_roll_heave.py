"""Axle roll/heave layout: a decoupled heave and roll spring and damper per axle.

For each axle the two wheels split into

    heave = (zL + zR) / sqrt(2)
    roll  = (zL - zR) / sqrt(2)

with a wheel-referred rate and a damper on each. The two axles do not couple.
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
) -> CompiledSuspension:
    """Assemble the axle roll/heave suspension."""

    if not (heave_rate_front > 0.0 and heave_rate_rear > 0.0):
        raise ValueError("the heave rate of each axle must be positive")
    return build_modal(
        layout=LAYOUT,
        kinematics=kinematics,
        transform=AXLE_MODE_MATRIX,
        mode_names=AXLE_MODE_NAMES,
        rates=(heave_rate_front, roll_rate_front, heave_rate_rear, roll_rate_rear),
        dampers=(heave_damper_front, roll_damper_front, heave_damper_rear, roll_damper_rear),
    )
