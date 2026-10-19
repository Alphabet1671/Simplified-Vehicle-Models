"""Four-mode layout: heave, roll, pitch and warp springs and dampers.

The four modes of the four wheel jounces are

    heave = (zFL + zFR + zRL + zRR) / 2
    roll  = (zFL - zFR + zRL - zRR) / 2
    pitch = (zFL + zFR - zRL - zRR) / 2
    warp  = (zFL - zFR - zRL + zRR) / 2

with ``T T^T = I``. Each mode carries one wheel-referred rate and one damper;
see ``modal.py`` for the conventions and the linearity limit.
"""

from __future__ import annotations

from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.modal import (
    FOUR_MODE_MATRIX,
    FOUR_MODE_NAMES,
    build_modal,
)
from simplified_vehicle_models.suspension.network import CompiledSuspension

LAYOUT = "four-mode"


def build_four_mode(
    *,
    kinematics: SuspensionKinematics,
    heave_rate: float,
    roll_rate: float,
    pitch_rate: float,
    warp_rate: float,
    heave_damper,
    roll_damper,
    pitch_damper,
    warp_damper,
) -> CompiledSuspension:
    """Assemble the four-mode suspension."""

    if not heave_rate > 0.0:
        raise ValueError("the heave rate must be positive")
    return build_modal(
        layout=LAYOUT,
        kinematics=kinematics,
        transform=FOUR_MODE_MATRIX,
        mode_names=FOUR_MODE_NAMES,
        rates=(heave_rate, roll_rate, pitch_rate, warp_rate),
        dampers=(heave_damper, roll_damper, pitch_damper, warp_damper),
    )
