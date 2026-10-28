import time
from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus

def blocking_cell(vehicle, limits, reduced, point):
    time.sleep(30)
    return DirectYawCell(ProbeStatus.FEASIBLE, 1., 1.)
