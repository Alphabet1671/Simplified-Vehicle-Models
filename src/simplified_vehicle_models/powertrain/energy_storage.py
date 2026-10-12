"""Model a battery pack from the exported cell curves."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import csv
import math
import numpy as np
from simplified_vehicle_models.compiler.capabilities import ENERGY_BUFFER
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import closed_parameters, require_no_references

_DATA = Path(__file__).resolve().parents[3] / "models" / "data" / "cosmx_cell_curves.csv"

@dataclass(frozen=True, slots=True)
class storage_state:
    state_of_charge: float

@dataclass(frozen=True, slots=True)
class storage_limits:
    bus_voltage: float
    discharge_power_limit: float
    charge_power_limit: float
    usable_energy: float

@dataclass(frozen=True, slots=True)
class energy_storage_parameters:
    series_count: int
    parallel_count: int
    discharge_current_limit: float
    charge_current_limit: float
    cell_capacity: float
    minimum_cell_voltage: float
    maximum_cell_voltage: float
    initial_state_of_charge: float
    state_of_charge_axis: tuple[float, ...]
    open_circuit_voltage_axis: tuple[float, ...]
    resistance_axis: tuple[float, ...]
    energy_axis: tuple[float, ...]

    @property
    def pack_capacity(self) -> float:
        return self.parallel_count * self.cell_capacity * 3600.0

def _curve(path: Path = _DATA):
    rows=[]
    with path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].startswith("#") or row[0] == "state_of_charge": continue
            rows.append(tuple(float(value) for value in row))
    raw=np.asarray(rows,float)
    # The source is descending. Reverse it for the increasing axis required by interpolation.
    raw=raw[::-1]
    return tuple(tuple(raw[:, i]) for i in range(4))

def _interp(axis, values, soc):
    return float(np.interp(soc, axis, values))

def _validate_state(state):
    soc = state.state_of_charge if isinstance(state, storage_state) else float(state)
    if not 0.0 <= soc <= 1.0: raise ValueError("state_of_charge must be from zero through one")
    return soc

def _pack_values(parameters, soc):
    voltage=parameters.series_count*_interp(parameters.state_of_charge_axis,parameters.open_circuit_voltage_axis,soc)
    resistance=parameters.series_count*_interp(parameters.state_of_charge_axis,parameters.resistance_axis,soc)/parameters.parallel_count
    return voltage,resistance

def evaluate_storage(parameters: energy_storage_parameters, state: storage_state) -> storage_limits:
    soc=_validate_state(state); voltage,resistance=_pack_values(parameters,soc)
    discharge=voltage*parameters.discharge_current_limit-parameters.discharge_current_limit**2*resistance
    charge=-(voltage*parameters.charge_current_limit+parameters.charge_current_limit**2*resistance)
    energy=(parameters.energy_axis[-1]-parameters.energy_axis[0])*3600*parameters.parallel_count
    return storage_limits(voltage,discharge,charge,energy)

def solve_current(parameters: energy_storage_parameters, state: storage_state, terminal_power: float) -> tuple[float,bool]:
    soc=_validate_state(state); voc,resistance=_pack_values(parameters,soc)
    if resistance <= 0: raise ValueError("pack resistance must be positive")
    discriminant=voc*voc-4*resistance*terminal_power
    feasible=discriminant >= 0.0
    if not feasible:
        terminal_power=voc*voc/(4*resistance)
        discriminant=0.0
    root=(voc-math.sqrt(discriminant))/(2*resistance)
    return root,feasible

def integrate_storage(parameters: energy_storage_parameters, state: storage_state, bus_power: float, duration: float) -> storage_state:
    current,_=solve_current(parameters,state,bus_power)
    soc=_validate_state(state)-current*duration/parameters.pack_capacity
    return storage_state(max(0.0,min(1.0,soc)))

_STORAGE_UNITS={"series_count":"1","parallel_count":"1","discharge_current_limit":"A","charge_current_limit":"A","cell_capacity":"A*h","minimum_cell_voltage":"V","maximum_cell_voltage":"V","initial_state_of_charge":"1"}
def compile_battery_pack(model: Model) -> energy_storage_parameters:
    values=closed_parameters(model,_STORAGE_UNITS); require_no_references(model)
    for name in ("series_count","parallel_count"):
        value=values[name]
        if not value.is_integer() or value<1: raise ValueError(f"{name} must be a positive integer")
    for name in ("discharge_current_limit","charge_current_limit","cell_capacity"): 
        if values[name]<=0: raise ValueError(f"{name} must be positive")
    if values["minimum_cell_voltage"]<=0 or values["maximum_cell_voltage"]<=values["minimum_cell_voltage"]: raise ValueError("cell voltage limits are invalid")
    if not 0<=values["initial_state_of_charge"]<=1: raise ValueError("initial_state_of_charge must be from zero through one")
    axis,ocv,resistance,energy=_curve()
    return energy_storage_parameters(int(values.pop("series_count")),int(values.pop("parallel_count")),state_of_charge_axis=axis,open_circuit_voltage_axis=ocv,resistance_axis=resistance,energy_axis=energy,**values)

register_component("energy-storage","battery-pack",capability=ENERGY_BUFFER,compile_function=compile_battery_pack)
