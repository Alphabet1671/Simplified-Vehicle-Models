from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class AeroParameters:
    """Store the aerodynamic parameters."""

    air_density: float
    downforce_area: float
    drag_area: float
    aero_balance: float
