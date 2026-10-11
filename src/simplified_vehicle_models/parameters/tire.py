from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TireEnvelopeParameters:
    """Store the parameters for a tire force envelope."""

    longitudinal_mu_scale: float
    lateral_mu_scale: float
    load_sensitivity_slope: float
    load_sensitivity_intercept: float
    effective_radius: float
