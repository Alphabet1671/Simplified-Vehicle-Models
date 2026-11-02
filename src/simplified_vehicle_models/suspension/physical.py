"""Physical shaft spring, damper, and inertance elements.

Travel is shaft compression in metres, velocity in metres per second, spring
rate in N/m, preload in N, damper coefficient in N*s/m, and inertance in kg.
Motion ratio is stored as positive dimensionless linkage metadata: conversion
between shaft and wheel coordinates belongs to the suspension network Jacobian.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.suspension.damper_table import DamperForce


def _finite_nonnegative(name: str, value: float) -> None:
    try:
        valid = bool(np.isfinite(value) and value >= 0.0)
    except (TypeError, ValueError):
        valid = False
    if not valid:
        raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class LinearDamper:
    """Linear shaft damper with coefficient in N*s/m."""

    coefficient: float

    def __post_init__(self) -> None:
        _finite_nonnegative("linear damper coefficient", self.coefficient)

    def force(self, velocity) -> DamperForce:
        """Return the linear force, slope, and a false range flag."""
        velocity = np.asarray(velocity)
        force = self.coefficient * velocity
        slope = np.zeros_like(velocity, dtype=np.result_type(velocity, float)) + self.coefficient
        return DamperForce(
            force=np.asarray(force),
            slope=np.asarray(slope),
            out_of_range=np.zeros_like(velocity, dtype=bool),
        )


@dataclass(frozen=True)
class PhysicalStrut:
    """Physical shaft element bundle; MR maps only in the network Jacobian."""

    spring_rate: float
    preload: float
    inertance: float
    damper: object
    damper_scale: float
    motion_ratio: float
    orientation: float

    def __post_init__(self) -> None:
        for name in ("spring_rate", "preload", "inertance", "damper_scale"):
            _finite_nonnegative(name, getattr(self, name))
        try:
            mr_valid = bool(np.isfinite(self.motion_ratio) and self.motion_ratio > 0.0)
        except (TypeError, ValueError):
            mr_valid = False
        if not mr_valid:
            raise ValueError("motion_ratio must be finite and positive")
        if self.orientation not in (-1, 1):
            raise ValueError("orientation must be exactly +1 or -1")
        if self.spring_rate == 0.0 and self.preload != 0.0:
            raise ValueError("nonzero preload requires a nonzero spring rate")
        if self.damper_scale != 0.0 and not callable(getattr(self.damper, "force", None)):
            raise ValueError("enabled damper must provide force(velocity)")

    def force(self, travel, velocity):
        """Return total shaft force (N), damper force (N), and range flags."""
        spring = self.preload + self.spring_rate * np.asarray(travel)
        if self.damper_scale == 0.0:
            return spring, np.zeros_like(spring), np.zeros_like(spring, dtype=bool)
        result = self.damper.force(velocity)
        damping = self.damper_scale * np.asarray(result.force)
        return spring + damping, damping, np.asarray(result.out_of_range)
