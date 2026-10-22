"""Capability backends used by the line OCP; controller behavior is supplied."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface, build_capability_surface,
)


@dataclass(frozen=True, slots=True)
class CapabilityGrid:
    speed: np.ndarray
    curvature: np.ndarray

    def __post_init__(self):
        for name in ('speed', 'curvature'):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.ndim != 1 or value.size < 3 or not np.all(np.isfinite(value)):
                raise ValueError(f'{name} must be a finite one-dimensional grid')
            if np.any(np.diff(value) <= 0.0):
                raise ValueError(f'{name} must increase')
            value.setflags(write=False)
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class SampledCapability:
    surface: CapabilitySurface
    vehicle_hash: str
    controller_hash: str
    mode: str
    version: str = '1'
    metadata: dict = field(default_factory=dict)
    feasible_mask: np.ndarray | None = None

    def __post_init__(self):
        if self.feasible_mask is not None:
            mask = np.asarray(self.feasible_mask, dtype=bool)
            if mask.shape != self.surface.acceleration_max.shape:
                raise ValueError('feasible_mask must match the capability grid')
            mask.setflags(write=False)
            object.__setattr__(self, 'feasible_mask', mask)


def _regrid(surface, grid):
    """Interpolate a builder surface onto the caller's exact axes."""
    from scipy.interpolate import RegularGridInterpolator
    points = (surface.speed, surface.curvature)
    amax = RegularGridInterpolator(points, surface.acceleration_max)(
        (grid.speed[:, None], grid.curvature[None, :]))
    amin = RegularGridInterpolator(points, surface.acceleration_min)(
        (grid.speed[:, None], grid.curvature[None, :]))
    ceiling = np.interp(grid.curvature, surface.curvature, surface.speed_ceiling)
    return CapabilitySurface(grid.speed, grid.curvature, amax, amin, ceiling,
                             surface.provenance_hash)


class IdealCapabilityBackend:
    mode = 'ideal'

    def __init__(self, vehicle):
        self.vehicle = vehicle

    def sample(self, grid: CapabilityGrid, *, curvature_limit: float) -> SampledCapability:
        raw = build_capability_surface(
            self.vehicle, curvature_limit=curvature_limit,
            speed_points=grid.speed.size, curvature_points=grid.curvature.size)
        surface = _regrid(raw, grid)
        mask = np.ones(surface.acceleration_max.shape, dtype=bool)
        return SampledCapability(surface, surface.provenance_hash, '', self.mode,
                                 feasible_mask=mask)
