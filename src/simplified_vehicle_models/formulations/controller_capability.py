"""Controller-aware capability sampling; the policy is an oracle, not a plant."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from simplified_vehicle_models.controls.planning import PathDemand, controller_hash
from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, SampledCapability,
)
from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface, build_capability_surface,
)


@dataclass(frozen=True, slots=True)
class AllocationCapabilityBackend:
    vehicle: object
    policy: object
    scan_points: int = 17
    bisection_steps: int = 12
    mode: str = 'allocation'

    def __post_init__(self):
        if bool(getattr(self.policy, 'is_stateful', False)):
            raise ValueError('a stateful controller must use closed_loop mode, '
                             'not static allocation sampling')

    def _ideal(self, grid, curvature_limit):
        return build_capability_surface(
            self.vehicle, curvature_limit=curvature_limit,
            speed_points=grid.speed.size, curvature_points=grid.curvature.size)

    def _demand(self, speed, curvature, ax):
        # Coerce to Python floats so policy comparisons return bool, not
        # np.bool_, which ControlResponse deliberately rejects.
        ax = float(ax)
        speed = float(speed)
        curvature = float(curvature)
        return PathDemand(speed, curvature, ax, speed * speed * curvature,
                          speed * curvature)

    def feasible_interval(self, speed, curvature, ideal_lo, ideal_hi):
        grid = np.linspace(ideal_lo, ideal_hi, self.scan_points)
        flags = np.array([self.policy.evaluate(self._demand(speed, curvature, ax)).feasible
                          for ax in grid])
        if not flags.any():
            return None
        if flags.all():
            return float(ideal_lo), float(ideal_hi), False
        # Monotone feasibility is assumed for the boundary refinement; verify
        # the sampled set and fall back to an isolated feasible sample if not.
        indices = np.flatnonzero(flags)
        contiguous = np.all(np.diff(indices) == 1)
        if not contiguous:
            return float(grid[indices[-1]]), float(grid[indices[-1]]), True

        def refine(feasible, infeasible):
            lo, hi = float(infeasible), float(feasible)
            for _ in range(self.bisection_steps):
                mid = 0.5 * (lo + hi)
                if self.policy.evaluate(self._demand(speed, curvature, mid)).feasible:
                    hi = mid
                else:
                    lo = mid
            return hi

        lower = float(grid[indices[0]])
        upper = float(grid[indices[-1]])
        if indices[0] > 0:
            lower = refine(lower, grid[indices[0] - 1])
        if indices[-1] + 1 < grid.size:
            upper = refine(upper, grid[indices[-1] + 1])
        return lower, upper, False

    def sample(self, grid, *, curvature_limit, ideal_surface=None, **_):
        if ideal_surface is None:
            ideal_surface = self._ideal(grid, curvature_limit)
        amax = np.empty_like(ideal_surface.acceleration_max)
        amin = np.empty_like(ideal_surface.acceleration_min)
        feasible_mask = np.ones(amax.shape, dtype=bool)
        ceiling = np.empty_like(ideal_surface.speed_ceiling)
        flags = {'non_monotone': False, 'infeasible_cells': 0,
                 'saturation_tags': set()}
        for j, curvature in enumerate(grid.curvature):
            for i, speed in enumerate(grid.speed):
                interval = self.feasible_interval(
                    float(speed), float(curvature),
                    float(ideal_surface.acceleration_min[i, j]),
                    float(ideal_surface.acceleration_max[i, j]))
                if interval is None:
                    feasible_mask[i, j] = False
                    flags['infeasible_cells'] += 1
                    amin[i, j] = 0.0; amax[i, j] = 0.0
                else:
                    amin[i, j], amax[i, j] = interval[0], interval[1]
                    flags['non_monotone'] = flags['non_monotone'] or interval[2]
                    response = self.policy.evaluate(self._demand(
                        float(speed), float(curvature), float(amax[i, j])))
                    flags['saturation_tags'].update(response.saturation)
            ceiling[j] = 0.0
            for i in range(grid.speed.size - 1, -1, -1):
                if feasible_mask[i, j]:
                    ceiling[j] = grid.speed[i]
                    break
            ceiling[j] = min(float(ideal_surface.speed_ceiling[j]), float(ceiling[j]))
        surface = CapabilitySurface(grid.speed, grid.curvature, amax, amin, ceiling,
                                    ideal_surface.provenance_hash)
        flags['saturation_tags'] = sorted(flags['saturation_tags'])
        return SampledCapability(surface, ideal_surface.provenance_hash,
                                 controller_hash(self.policy), self.mode,
                                 metadata=flags, feasible_mask=feasible_mask)
