"""Full 38-state steady deceleration force-hold diagnostic.

Modes: fixed-force holds the initial physical-inverse force (comparison); state-aware
recomputes the cheap constant-linear-hydraulic inverse from current full state each
RHS evaluation. Neither is controller/path-cap qualification or Task 4 completion.
"""
from __future__ import annotations
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext, BrakeAuthorityPredictor
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan, DriverCommand
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant, SOC_INDEX

ROOT=Path(__file__).resolve().parents[2]
FORMULATION=ROOT/'models/formulations/iem24_murphy_real_14dof_simple.xml'
HYDRAULIC=ROOT/'models/vehicles/iem24_murphy_real_planar.xml'
SETTLE_AFTER_S=.35
MIN_WINDOW_S=.20
MAX_HOLD_S=2.0

def context_from_state(model, plant, x, positions, JOUNCE_INDEX, STEERING_INDEX, wheel_steering_angles, tire_deflection_load, loaded_radius):
    mech=plant.mechanical_state(x)
    z=mech.heave+mech.roll*positions[:,1]-mech.pitch*positions[:,0]
    lift,_,_=model.suspension.kinematics.steering_lift(wheel_steering_angles(float(x[STEERING_INDEX]),4))
    loads=np.maximum(np.asarray(tire_deflection_load(model.planar.tire,z-x[list(JOUNCE_INDEX)]-lift,np.zeros(4),0.).load),1.)
    radii=loaded_radius(model.planar.tire,loads)
    return BrakeAuthorityContext(max(float(x[3]),2.),0.,tuple(radii),tuple(loads),soc=float(np.clip(x[SOC_INDEX],0.,1.)))

def state_aware_force(predictor, context, target):
    """Public predictor-owned fast inverse, for the declared linear hydraulic map."""
    return float(predictor.request_current(context,target).pedal_force_n)

def settled_metrics(time_s, requested, applied, decel, target):
    """Time-weighted metrics in declared post-start, force-tracking window."""
    ts=np.asarray(time_s); req=np.broadcast_to(np.asarray(requested,dtype=float),ts.shape); app=np.asarray(applied)
    eligible=(ts>=SETTLE_AFTER_S)&(np.abs(app-req)<=.01*np.maximum(np.abs(req),1.))
    # Score only one contiguous eligible run; any internal actuator-force gap breaks it.
    indices=np.flatnonzero(eligible)
    splits=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1) if len(indices) else []
    runs=[r for r in splits if len(r)>=2 and ts[r[-1]]-ts[r[0]]>=MIN_WINDOW_S]
    if not runs:
        raise ValueError('no contiguous settled window of at least 0.20 s')
    run=max(runs,key=lambda r:ts[r[-1]]-ts[r[0]])
    lo,hi=ts[run[0]],ts[run[-1]]; keep=np.zeros(ts.shape,dtype=bool); keep[run]=True; tt=ts[keep]
    measured=np.asarray(decel)[keep]; rr=req[keep]; aa=app[keep]
    span=float(tt[-1]-tt[0]); mean=float(np.trapezoid(measured,tt)/span)
    error=abs(mean-target); max_error=float(np.max(np.abs(measured-target))); limit=max(.25,.05*target)
    return {'settled_samples':int(keep.sum()),'settled_duration_s':span,
            'requested_force_mean_N':float(np.trapezoid(rr,tt)/span),
            'applied_force_mean_N':float(np.trapezoid(aa,tt)/span),
            'measured_deceleration_mean_mps2':mean,'measured_deceleration_min_mps2':float(np.min(measured)),
            'measured_deceleration_max_mps2':float(np.max(measured)),'absolute_target_error_mps2':error,
            'maximum_absolute_deceleration_error_mps2':max_error,
            'error_limit_mps2':limit,'mean_tracking_pass':error<=limit,
            'maximum_error_pass':max_error<=limit,'tracking_pass':error<=limit and max_error<=limit}

def run_case(model,target,pedal,*,duration=MAX_HOLD_S,trace=None,control_mode='fixed-force'):
    if control_mode not in ('fixed-force','state-aware'): raise ValueError('unknown control mode')
    v0=20.; plan=SpeedPlan(np.array([0.,1000.]),np.array([v0,v0]),np.zeros(2),False)
    plant=VerticalForwardPlant(model,plan); y0=plant.initial_state(v0)
    predictor=BrakeAuthorityPredictor(model.planar,maximum_abs_curvature_inv_m=.02,minimum_speed_mps=2.,maximum_speed_mps=40.)
    from simplified_vehicle_models.solvers.vertical_forward import JOUNCE_INDEX, STEERING_INDEX, sprung_wheel_positions
    from simplified_vehicle_models.formulations.planar_7dof import wheel_steering_angles
    from simplified_vehicle_models.tires.vertical import tire_deflection_load
    from simplified_vehicle_models.tires.unitire_simple import loaded_radius
    positions=sprung_wheel_positions(model)
    def context(x): return context_from_state(model,plant,x,positions, JOUNCE_INDEX,STEERING_INDEX,wheel_steering_angles,tire_deflection_load,loaded_radius)
    initial_context=context(y0)
    initial_force=(float(predictor.request(initial_context,target).pedal_force_n) if control_mode=='fixed-force'
                   else state_aware_force(predictor,initial_context,target))
    if initial_force<=0.: raise ValueError('inverse produced nonpositive brake force')
    max_force=float(model.planar.maximum_pedal_force); initial_force=min(initial_force,max_force)
    initial=np.r_[y0,float(pedal)]
    def command_force(x):
        return (state_aware_force(predictor,context(x),target) if control_mode=='state-aware' else initial_force)
    def rhs(t,z):
        x=z[:-1]; pos=float(np.clip(z[-1],-1.,1.)); req=command_force(x)
        applied=max(-pos,0.)*max_force
        rate=plant.evaluate(t,x,DriverCommand(max(pos,0.),applied,0.)).solve()
        pdot=float(np.clip((-req/max_force-pos)/.02,-1/.175,1/.175))
        return np.r_[rate,pdot]
    def jac(t,z):
        base=rhs(t,z); J=np.zeros((len(z),len(z)))
        for i,value in enumerate(z):
            h=1.e-5*max(abs(value),1.); trial=z.copy(); trial[i]+=h
            try: rates=rhs(t,trial)
            except (ValueError,FloatingPointError):
                trial[i]=value-h
                try: rates=rhs(t,trial); h=-h
                except Exception: continue
            J[:,i]=(rates-base)/h
        return J
    started=time.perf_counter()
    sol=solve_ivp(rhs,(0.,duration),initial,method='BDF',jac=jac,rtol=2e-5,atol=2e-7,max_step=.02)
    wall=time.perf_counter()-started
    if not sol.success: raise RuntimeError(sol.message)
    ts=sol.t; states=sol.y[:-1].T; pedals=sol.y[-1]; applied=np.maximum(-pedals,0.)*max_force
    requested=np.asarray([command_force(x) for x in states]); decel=[]; loads=[]; radii=[]
    for t,x,pf in zip(ts,states,applied):
        decel.append(-float(plant.evaluate(float(t),x,DriverCommand(0.,float(pf),0.)).solve()[3]))
        c=context(x); loads.append(c.normal_loads_n); radii.append(c.wheel_radii_m)
    decel=np.asarray(decel); speed=states[:,3]
    try: metrics=settled_metrics(ts,requested,applied,decel,target)
    except ValueError as e:
        eligible=(ts>=SETTLE_AFTER_S)&(np.abs(applied-requested)<=.01*np.maximum(np.abs(requested),1.))
        metrics={'settled_samples':int(eligible.sum()),'settled_duration_s':float(np.ptp(ts[eligible])) if np.any(eligible) else 0.,
                 'requested_force_mean_N':None,'applied_force_mean_N':None,'measured_deceleration_mean_mps2':None,
                 'absolute_target_error_mps2':None,'error_limit_mps2':max(.25,.05*target),'tracking_pass':False,'settling_failure':str(e)}
    metrics.update({'control_mode':control_mode,'target_deceleration_mps2':target,'initial_speed_mps':v0,'initial_pedal':pedal,
       'duration_s':duration,'wall_s':wall,'solver_nfev':int(sol.nfev),'accepted_solver_samples':len(ts),
       'terminal_speed_mps':float(speed[-1]),'requested_force_initial_N':initial_force,'requested_force_terminal_N':float(requested[-1]),
       'applied_force_terminal_N':float(applied[-1]),'initial_authority_context_deceleration_mps2':float(predictor.predict(initial_context,initial_force).deceleration_mps2),
       'normal_load_min_N':float(np.min(loads)),'normal_load_max_N':float(np.max(loads)),
       'loaded_radius_min_m':float(np.min(radii)),'loaded_radius_max_m':float(np.max(radii)),
       'all_states_finite':bool(np.all(np.isfinite(sol.y)))})
    metrics['pass']=bool(metrics['tracking_pass'] and metrics['all_states_finite'] and speed[-1]>0.)
    if trace is not None:
        np.savez_compressed(trace,time=ts,state=states,pedal_position=pedals,requested_brake_force_N=requested,
            applied_brake_force_N=applied,measured_deceleration_mps2=decel,normal_load_N=np.asarray(loads),loaded_radius_m=np.asarray(radii))
    return metrics

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True); ap.add_argument('--trace-dir',type=Path,required=True)
    ap.add_argument('--duration',type=float,default=MAX_HOLD_S); ap.add_argument('--control-mode',choices=('fixed-force','state-aware'),default='fixed-force')
    a=ap.parse_args()
    if not 0<a.duration<=MAX_HOLD_S: ap.error('duration must be in (0, 2] seconds')
    model=compile_formulation(str(FORMULATION)).model; cases=[(target,p) for target in (3.,6.) for p in (0.,1.)]; rows=[]
    a.trace_dir.mkdir(parents=True,exist_ok=True)
    for i,(target,pedal) in enumerate(cases,1):
        try: row=run_case(model,target,pedal,duration=a.duration,trace=a.trace_dir/f'brake-decel-{target:g}-{pedal:g}.npz',control_mode=a.control_mode)
        except Exception as e: row={'target_deceleration_mps2':target,'initial_pedal':pedal,'control_mode':a.control_mode,'pass':False,'error':f'{type(e).__name__}: {e}'}
        rows.append(row); print(f'{i}/4 target={target:g} pedal={pedal:g}: {row.get("pass")} {row.get("error","")}',flush=True)
    data={'scope':'full 38-state longitudinal force feedforward diagnostic; not Task 4 completion or path-cap control',
       'control_mode':a.control_mode,'formulation_sha256':hashlib.sha256(FORMULATION.read_bytes()).hexdigest(),
       'hydraulic_sha256':hashlib.sha256(HYDRAULIC.read_bytes()).hexdigest(),
       'settling_rule':{'earliest_s':SETTLE_AFTER_S,'requested_applied_force_tolerance_fraction':.01,'minimum_window_s':MIN_WINDOW_S,'mean':'time-weighted trapezoidal'},
       'rows':rows,'pass':len(rows)==4 and all(r.get('pass',False) for r in rows)}
    a.output.parent.mkdir(parents=True,exist_ok=True); a.trace_dir.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(data,indent=2)+'\n'); print(json.dumps(data,indent=2))
if __name__=='__main__': main()
