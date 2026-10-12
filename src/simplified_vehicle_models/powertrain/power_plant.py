"""Model the voltage-dependent electric motor.

The fitted parameters use line-to-line peak voltage. This module converts that
basis to per-phase peak voltage before it uses the Section 4.2 equations.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import csv, math
import numpy as np
from simplified_vehicle_models.compiler.capabilities import TORQUE_SOURCE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import closed_parameters, require_no_references

_ROOT=Path(__file__).resolve().parents[3]; _DATA=_ROOT/"models"/"data"
@dataclass(frozen=True, slots=True)
class direct_current_result:
    direct_current: float
    feasible: bool
@dataclass(frozen=True, slots=True)
class shaft_power_result:
    bus_power: float
    loss_power: float
    field_weakening_correction: float
    direct_current: float
    efficiency: float
    active_limit: str
@dataclass(frozen=True, slots=True)
class power_plant_parameters:
    unit_count:int; maximum_motor_speed:float; unit_power_limit:float; inverter_efficiency:float; device_drop:float
    pole_pairs:int; reference_bus_voltage:float; resistance:float; flux_linkage:float; direct_inductance:float; quadrature_inductance:float
    torque_speed:tuple[float,...]; torque_values:tuple[float,...]; loss_speed:tuple[float,...]; loss_torque:tuple[float,...]; loss_values:tuple[float,...]

def _read(name):
    with (_DATA/name).open(newline="") as stream:
        return [row for row in csv.reader(stream) if row and not row[0].startswith("#") and row[0] not in {"name", "speed_rpm"}]
def _parameter_file():
    return {row[0]:float(row[1]) for row in _read("amk_dd5_100c_machine_parameters.csv")}
def _interp(x,y,value):
    if value< x[0] or value>x[-1]: raise ValueError("motor speed is outside the map")
    return float(np.interp(value,x,y))
def _map_values(speed, torque, p):
    s=max(0.0,speed)
    if s>p.maximum_motor_speed: raise ValueError("motor speed exceeds maximum_motor_speed")
    # The map starts at 1 rpm. Use its first row at rest instead of extrapolating.
    s=max(s,p.torque_speed[0]); loss_s=max(s,p.loss_speed[0])
    envelope=_interp(p.torque_speed,p.torque_values,s)
    loss=float(_loss_interpolate(p, loss_s, abs(torque)))
    return envelope,loss

def _loss_interpolate(p, speed, torque):
    speeds=np.asarray(sorted(set(p.loss_speed)))
    values=[]
    for s in speeds:
        pairs=[(t,v) for row_s,t,v in zip(p.loss_speed,p.loss_torque,p.loss_values) if row_s == s]
        pairs.sort()
        values.append(float(np.interp(torque,[x[0] for x in pairs],[x[1] for x in pairs])))
    return float(np.interp(speed,speeds,values))
def _bilinear(xs,ys,z,x,y):
    if x<xs[0] or x>xs[-1] or y<ys[0] or y>ys[-1]: raise ValueError("motor map query is outside the map")
    ix=min(len(xs)-2,max(0,int(np.searchsorted(xs,x)-1))); iy=min(len(ys)-2,max(0,int(np.searchsorted(ys,y)-1)))
    x0,x1=xs[ix],xs[ix+1]; y0,y1=ys[iy],ys[iy+1]; q=(z[ix,iy],z[ix,iy+1],z[ix+1,iy],z[ix+1,iy+1])
    a=(x-x0)/(x1-x0) if x1!=x0 else 0; b=(y-y0)/(y1-y0) if y1!=y0 else 0
    return (1-a)*(1-b)*q[0]+(1-a)*b*q[1]+a*(1-b)*q[2]+a*b*q[3]
def calculate_available_voltage(parameters,bus_voltage,current):
    return bus_voltage/math.sqrt(3)-parameters.resistance*current-parameters.device_drop
def maximum_motor_torque(parameters,motor_speed,available_voltage):
    envelope,_=_map_values(motor_speed,0,parameters)
    if available_voltage<=0: return 0.0
    reference_speed=motor_speed*parameters.reference_bus_voltage/math.sqrt(3)/available_voltage
    if reference_speed<=parameters.torque_speed[-1]:
        reference=_interp(parameters.torque_speed,parameters.torque_values,max(reference_speed,parameters.torque_speed[0]))
    else: reference=0.0
    return reference

def solve_direct_current(parameters,motor_speed,quadrature_current,available_voltage):
    w=parameters.pole_pairs*2*math.pi*max(motor_speed,0.0)/60
    r=parameters.resistance; ld=parameters.direct_inductance; lq=parameters.quadrature_inductance; psi=parameters.flux_linkage; iq=quadrature_current
    # Expand the voltage constraint into one quadratic in direct current.
    a=r*r+(w*ld)**2; b=-2*w*lq*iq*r+2*w*ld*(r*iq+w*psi); c=(r*0-w*lq*iq)**2+(r*iq+w*psi)**2-available_voltage**2
    if abs(a)<1e-30: return direct_current_result(0.0,c<=0)
    disc=b*b-4*a*c
    if disc<0: return direct_current_result(0.0,False)
    roots=((-b-math.sqrt(disc))/(2*a),(-b+math.sqrt(disc))/(2*a)); root=min(roots,key=abs)
    return direct_current_result(root,True)
def evaluate_shaft_power(parameters,motor_speed,motor_torque,bus_voltage):
    avail=calculate_available_voltage(parameters,bus_voltage,0.0)
    # Use the requested torque to estimate q-axis current for both solves.
    quadrature_current=motor_torque/(1.5*parameters.pole_pairs*parameters.flux_linkage)
    current=solve_direct_current(parameters,motor_speed,quadrature_current,avail)
    reference_voltage=calculate_available_voltage(parameters,parameters.reference_bus_voltage,0.0)
    reference=solve_direct_current(parameters,motor_speed,quadrature_current,reference_voltage)
    _,map_loss=_map_values(motor_speed,motor_torque,parameters)
    correction=0.0 if abs(bus_voltage-parameters.reference_bus_voltage)<1e-12 else 1.5*parameters.resistance*(current.direct_current**2-reference.direct_current**2)
    loss=map_loss+correction; shaft=motor_speed*2*math.pi/60*motor_torque
    raw=shaft+loss
    if raw>=0: bus=raw/parameters.inverter_efficiency; efficiency=shaft/bus if bus else 1.0
    else: bus=raw*parameters.inverter_efficiency; efficiency=bus/shaft if shaft else 1.0
    return shaft_power_result(bus,loss,correction,current.direct_current,efficiency,"power" if abs(shaft)>=parameters.unit_power_limit else "torque")

_UNITS={"unit_count":"1","maximum_motor_speed":"rpm","unit_power_limit":"W","inverter_efficiency":"1","device_drop":"V"}
def compile_electric_motor(model:Model)->power_plant_parameters:
    v=closed_parameters(model,_UNITS); require_no_references(model)
    if not v["unit_count"].is_integer() or v["unit_count"]<1: raise ValueError("unit_count must be a positive integer")
    for n in ("maximum_motor_speed","unit_power_limit"): 
        if v[n]<=0: raise ValueError(f"{n} must be positive")
    if not 0<v["inverter_efficiency"]<=1: raise ValueError("inverter_efficiency must be from zero to one")
    m=_parameter_file(); tr=_read("amk_dd5_100c_torque_map.csv"); lo=_read("amk_dd5_100c_loss_map.csv")
    ts=tuple(float(r[0]) for r in tr); tv=tuple(float(r[1]) for r in tr)
    ls=tuple(float(r[0]) for r in lo); lt=tuple(float(r[1]) for r in lo); lv=tuple(float(r[2]) for r in lo)
    return power_plant_parameters(int(v["unit_count"]),v["maximum_motor_speed"],v["unit_power_limit"],v["inverter_efficiency"],v["device_drop"],int(m["pole_pairs"]),m["reference_bus_voltage"],m["resistance"],m["flux_linkage"],m["direct_inductance"],m["quadrature_inductance"],ts,tv,ls,lt,lv)
register_component("power-plant","electric-motor",capability=TORQUE_SOURCE,compile_function=compile_electric_motor)
