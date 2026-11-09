"""Bounded full 38-state short open-loop brake authority probes.

Usage: PYTHONPATH=src python3 tools/run/probe_brake_authority.py --duration .35
Uses fixed-step RK4 and rejects state-domain violations. Reports speed derivative
separately from body-axis acceleration. Trials are transient evidence, not a
universal feasibility claim.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan, DriverCommand
from simplified_vehicle_models.solvers.vertical_forward import (vertical_plant_class, SOC_INDEX, TIRE_OFFSET)
from simplified_vehicle_models.formulations.vertical_14dof import VerticalState
ROOT=Path(__file__).resolve().parents[2]
VX_INDEX, VY_INDEX, YAW_RATE_INDEX = 3, 4, 8
MAX_PROBE_DURATION_S = 2.0

def validate_duration(value):
    value=float(value)
    if not np.isfinite(value) or value <= 0.0 or value > MAX_PROBE_DURATION_S:
        raise argparse.ArgumentTypeError(f"duration must be finite and in (0, {MAX_PROBE_DURATION_S}] seconds")
    return value
FORMULATION=ROOT/'models/formulations/iem24_murphy_real_14dof_simple.xml'
XML=ROOT/'models/vehicles/iem24_murphy_real_planar.xml'
def probe(model,speed,curvature,fraction,duration):
    duration=validate_duration(duration)
    if not np.isfinite(speed) or speed <= 0 or not np.isfinite(curvature) or not np.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('speed, curvature and pedal fraction must be finite; speed positive, fraction in [0,1]')
    plan=SpeedPlan(np.array([0.,1000.]),np.array([speed,speed]),np.array([curvature,curvature]),False)
    plant=vertical_plant_class(model)(model,plan); y=plant.initial_state(speed)
    command=DriverCommand(0.,fraction*model.planar.maximum_pedal_force,0.)
    steps=int(np.ceil(duration/.00025)); h=duration/steps; nfev=0
    def rhs(t,x):
        nonlocal nfev
        if not 0<=x[SOC_INDEX]<=1: raise ValueError('trial leaves SOC state domain')
        nfev+=1; return plant.evaluate(t,x,command).solve()
    for j in range(steps):
        t=j*h; k1=rhs(t,y); k2=rhs(t+h/2,y+h*k1/2); k3=rhs(t+h/2,y+h*k2/2); k4=rhs(t+h,y+h*k3)
        y=y+h*(k1+2*k2+2*k3+k4)/6
    rate=rhs(duration,y); vx,vy=y[VX_INDEX],y[VY_INDEX]; r=y[YAW_RATE_INDEX]
    return {'speed_mps':speed,'curvature_inv_m':curvature,'fraction':fraction,'final_vx_mps':float(vx),
        'vx_rate_mps2':float(rate[VX_INDEX]),'body_ax_mps2':float(rate[VX_INDEX]-r*vy),
        'final_tire_states':y[TIRE_OFFSET:TIRE_OFFSET+12].tolist(),'solver_nfev':nfev,'integration_step_s':h}
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--duration',type=validate_duration,default=.35); ap.add_argument('--output',type=Path); a=ap.parse_args()
    model=compile_formulation(str(FORMULATION)).model; rows=[]
    for f in (0.,.1,.2,.3,.4): rows.append(probe(model,15.,0.,f,a.duration))
    for k in (-1/30,1/30): rows.append(probe(model,12.,k,.15,a.duration))
    data={'model_xml':str(FORMULATION.relative_to(ROOT)),'model_xml_sha256':hashlib.sha256(FORMULATION.read_bytes()).hexdigest(),
          'hydraulic_xml_sha256':hashlib.sha256(XML.read_bytes()).hexdigest(),'scope':'open-loop short transient probes; no general feasibility claim',
          'regen_enabled':model.planar.controller.regen_enabled,'rows':rows}
    text=json.dumps(data,indent=2)+'\n'
    if a.output: a.output.write_text(text)
    else: print(text,end='')
if __name__=='__main__': main()
