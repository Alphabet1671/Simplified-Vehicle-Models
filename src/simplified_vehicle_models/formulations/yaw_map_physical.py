"""One direct-cell cache shared by discovery, map nodes and acceptance probes."""
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import (
    direct_yaw_cell, YawCapabilityMap, YawCapabilityNumericalError,
)
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeResult, ProbeStatus
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash


class PhysicalYawEvaluator:
    def __init__(self, vehicle, tv_limits=None):
        self.vehicle = vehicle
        self.tv_limits = tv_limits
        self.cells = {}

    def cell(self, speed, ax, ay):
        key = (float(speed), float(ax), float(ay))
        if key not in self.cells:
            self.cells[key] = direct_yaw_cell(self.vehicle, *key, self.tv_limits)
        return self.cells[key]

    def __call__(self, speed, ax, ay):
        cell = self.cell(speed, ax, ay)
        return ProbeResult(cell.status, cell.detail)

    def build(self, vehicle, grid, tv_limits=None, *, yaw_inertia):
        if vehicle is not self.vehicle or tv_limits != self.tv_limits:
            raise ValueError('physical cache inputs differ from builder inputs')
        shape = tuple(len(x) for x in (grid.speed, grid.longitudinal_acceleration, grid.lateral_acceleration))
        into, out = np.zeros(shape), np.zeros(shape)
        mask = np.zeros(shape, bool)
        for i, v in enumerate(grid.speed):
            for j, ax in enumerate(grid.longitudinal_acceleration):
                for k, ay in enumerate(grid.lateral_acceleration):
                    cell = self.cell(v, ax, ay)
                    if cell.status is ProbeStatus.UNRESOLVED:
                        raise YawCapabilityNumericalError(cell.detail)
                    if cell.status is ProbeStatus.FEASIBLE:
                        mask[i,j,k] = True
                        into[i,j,k], out[i,j,k] = cell.into_turn, cell.out_of_turn
        kind = 'physical_no_tv_v1' if tv_limits is None else 'physical_tv_v1'
        identity = capability_hash((vehicle, grid, tv_limits, yaw_inertia, 'physical'))
        return YawCapabilityMap(grid.speed, grid.longitudinal_acceleration, grid.lateral_acceleration,
                               into, out, mask, yaw_inertia, kind, identity)
