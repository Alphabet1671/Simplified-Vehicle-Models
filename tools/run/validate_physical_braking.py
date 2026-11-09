"""Bounded physical-reachable braking matrix on the actual 38-state plant.

This offline validation is not a full-track qualification. Metrics expose the
frozen initial-state planning context and do not claim universal bounds.
"""
from __future__ import annotations
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope, propagate_braking_caps
from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, SpeedPlan, solve_forward
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant

ROOT=Path(__file__).resolve().parents[2]
FORMULATION=ROOT/'models/formulations/iem24_murphy_real_14dof_simple.xml'
HYDRAULIC=ROOT/'models/vehicles/iem24_murphy_real_planar.xml'

def run_case(model, scenario, duration, max_wall, trace_path=None):
    v0,target,pedal,curvature,kind=scenario
    end=max(80.,duration*v0+20.)
    if kind=='stop': end=max(40.,v0*v0/8.+10.)
    knots=np.array([0.,.25*end,.5*end,.75*end,end])
    cap0=v0+5.
    event_target=0.1 if kind=='stop' else target
    cap_target=2.1 if kind=='stop' else target
    caps=np.array([cap0,cap0,cap_target,cap_target,cap_target],float)
    if kind=='seam':
        caps=np.array([cap0,cap0,target,target,cap0],float)
        caps[-1]=caps[0]
    # Stop is represented by a separate terminal event at end-distance;
    # the SpeedPlan remains positive-speed throughout its supported domain.
    curves=np.full(5,curvature)
    decels=np.full(4,2.)
    closed=kind=='seam'
    if closed:
        curves[-1]=curves[0]
        entry,binding,_=propagate_braking_caps(knots,caps,decels,is_closed=True)
    else:
        entry,binding,_=propagate_braking_caps(knots,caps,decels,is_closed=False)
    env=BrakingEnvelope(knots,curves,caps,entry,decels,binding,closed,float(max(cap0,1.)),{})
    speeds=np.maximum(.25,np.minimum(caps,np.linspace(v0,event_target,5)))
    if kind in ('stop','steady'):
        caps=np.full(5,15.)
        entry=np.full(5,15.)
        binding=np.zeros(5,dtype=int)
        speeds=np.array([15.,15.,15.,15.,15.])
        env=BrakingEnvelope(knots,curves,caps,entry,decels,binding,False,15.,{})
    if closed: speeds[-1]=speeds[0]
    plan=SpeedPlan(knots,speeds,curves,closed)
    rates=(1/.175,)*4
    policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode='physical-reachable-v1'))
    driver=PreviewDriver(plan,model.planar.wheelbase,model.planar.steering_maximum_angle,
        model.planar.steering_maximum_rate,model.planar.maximum_pedal_force,
        pedal_travel_time=None,throttle_apply_rate=rates[0],throttle_release_rate=rates[1],
        brake_apply_rate=rates[2],brake_release_rate=rates[3],envelope_policy=policy)
    plant=VerticalForwardPlant(model,plan); state=plant.initial_state(v0)
    predictor=__import__('simplified_vehicle_models.solvers.brake_authority',fromlist=['BrakeAuthorityPredictor']).BrakeAuthorityPredictor(
        model.planar,maximum_abs_curvature_inv_m=max(abs(curvature),.02),minimum_speed_mps=2.,maximum_speed_mps=40.)
    def factory(_plant,_obs,pedal_context,plant_state):
        return _plant.brake_planning_context(plant_state,pedal_context,predictor=predictor,knot_count=4)
    def current_factory(_plant,_obs,_pedal_context,plant_state):
        return _plant.brake_authority_context(plant_state,predictor=predictor)
    object.__setattr__(driver,'brake_planning_context_factory',factory)
    object.__setattr__(driver,'brake_current_context_factory',current_factory)
    object.__setattr__(driver,'brake_current_predictor',predictor)
    samples=[]; started=time.perf_counter()
    try:
        result=solve_forward(plant,driver,state,duration=duration,wall_seconds=max_wall,maximum_wall_seconds=max_wall+5.,
            maximum_step=.02,rtol=2e-5,atol=2e-7,method='BDF',initial_pedal=pedal,step_sink=samples.append)
        if trace_path is not None:
            np.savez_compressed(trace_path,time=np.asarray([x.time for x in samples]),
                state=np.asarray([x.state for x in samples]),pedal=np.asarray([x.pedal for x in samples]),
                requested_brake=np.asarray([x.requested_command.brake_force for x in samples]),
                applied_brake=np.asarray([x.command.brake_force for x in samples]),
                acceleration_request=np.asarray([x.policy_diagnostics.acceleration_request for x in samples]))
    except Exception as e:
        return {'scenario':{'v0_mps':v0,'target_mps':target,'initial_pedal':pedal,'curvature_inv_m':curvature,'kind':kind},
                'error':f'{type(e).__name__}: {e}','partial_samples':len(samples),
                'partial_state_finite':bool(np.all(np.isfinite(samples[-1].state))) if samples else True}
    wall=time.perf_counter()-started
    st=np.asarray([x.state[0] for x in samples]); v=np.asarray([x.state[3] for x in samples])
    if trace_path is not None:
        np.savez_compressed(trace_path,time=np.asarray([x.time for x in samples]),
            state=np.asarray([x.state for x in samples]),pedal=np.asarray([x.pedal for x in samples]),
            requested_brake=np.asarray([x.requested_command.brake_force for x in samples]),
            applied_brake=np.asarray([x.command.brake_force for x in samples]),
            acceleration_request=np.asarray([x.policy_diagnostics.acceleration_request for x in samples]))
    ax=np.asarray([x.rates[3]-x.state[8]*x.state[4] for x in samples])
    requested=np.asarray([x.requested_command.brake_force for x in samples]); applied=np.asarray([x.command.brake_force for x in samples])
    policy_raw=np.asarray([x.policy_diagnostics.acceleration_request for x in samples])
    fallback=np.asarray([bool(x.policy_diagnostics.fallback_reason) for x in samples])
    cap=np.interp(st,knots,caps,left=caps[0],right=caps[-1]); overshoot=np.maximum(0.,v-cap)
    reached=np.flatnonzero(v<=target+.1) if kind not in ('stop','seam','steady') else (np.flatnonzero(v<=.1+1.e-4) if kind=='stop' else (np.flatnonzero(st>=knots[-1]) if kind=='seam' else np.array([],dtype=int)))
    times=np.asarray([x.time for x in samples])
    track=(requested>1.) & (times>.25)
    errors=np.abs(np.maximum(-ax[track],0.)-np.maximum(-policy_raw[track],0.)) if np.any(track) else np.array([])
    # Declared settled window: at least .30 s after first >90% brake pedal,
    # with applied force within 1% of local requested force; score only straight
    # constant-target cases, not cornering or event transients.
    full=np.asarray([x.pedal is not None and x.pedal<=-.90 for x in samples])
    settled=np.zeros(len(samples),dtype=bool)
    if kind=='steady':
        settled=(times>=1.0)&(np.abs(applied-requested)<=.01*max(float(np.max(requested)),1.))&(requested>1.)
    elif np.any(full):
        onset=times[np.flatnonzero(full)[0]]
        settled=(times>=onset+.30)&(np.abs(applied-requested)<=.01*max(float(np.max(requested)),1.))&(requested>1.)
    steady_error=np.abs(np.maximum(-ax[settled],0.)-np.maximum(-policy_raw[settled],0.)) if np.any(settled) else np.array([])
    contact=float(plant.minimum_contact_load); maxov=float(np.max(overshoot))
    return {'scenario':{'v0_mps':v0,'target_mps':target,'initial_pedal':pedal,'curvature_inv_m':curvature,'kind':kind},
        'wall_s':wall,'solver_message':result.message,'solver_nfev':result.evaluations,'samples':len(samples),
        'terminal_speed_mps':float(v[-1]),'terminal_station_m':float(st[-1]),'min_speed_mps':float(np.min(v)),
        'target_reached':bool(len(reached)>0),'station_at_target_m':float(st[reached[0]]) if len(reached) else None,
        'time_at_target_s':float(samples[reached[0]].time) if len(reached) else None,
        'max_cap_overshoot_mps':maxov,'requested_peak_brake_N':float(np.max(requested)),
        'predicted_target_deceleration_mps2':float(np.max([max(0.,-x.policy_diagnostics.acceleration_request) for x in samples])),
        'raw_acceleration_request_mps2':float(np.max([x.policy_diagnostics.acceleration_request for x in samples])),
        'applied_body_ax_min_mps2':float(np.min(ax)),
        'applied_peak_brake_N':float(np.max(applied)),'minimum_contact_N':contact,
        'flagged_condition_fraction':float(np.mean(fallback)),'flagged_condition_counts':{
            str(reason):int(sum(x.policy_diagnostics.fallback_reason==reason for x in samples))
            for reason in sorted({x.policy_diagnostics.fallback_reason for x in samples if x.policy_diagnostics.fallback_reason})},
        'tracking_region_samples':int(np.count_nonzero(track)),
        'settled_tracking_samples':int(np.count_nonzero(settled)),
        'max_transient_mismatch_mps2_not_gated':float(np.max(errors)) if errors.size else None,
        'max_settled_tracking_error_mps2':float(np.max(steady_error)) if steady_error.size else None,
        'settled_deceleration_mean_mps2':float(np.mean(np.maximum(-ax[settled],0.))) if np.any(settled) else None,
        'all_states_finite':bool(np.all(np.isfinite(result.state))),
        'jounce_extrema_m':[float(np.min(result.state[16:20])),float(np.max(result.state[16:20]))],
        'gates':{'finite':bool(np.all(np.isfinite(result.state))),'contact_nonnegative':contact>=0.,
                 'cap_overshoot_le_0.5mps':maxov<=.5,'target_reached':bool(len(reached)>0),
                 'tracking_tolerance':bool(steady_error.size and float(np.max(steady_error))<=max(.25,.05*max(abs(float(np.median(policy_raw[settled]))),.01)))}}

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--duration',type=float,default=5.); p.add_argument('--wall-per-case',type=float,default=100.); p.add_argument('--limit-cases',type=int,default=0); p.add_argument('--trace',type=Path,default=None); a=p.parse_args()
    if not 0<a.duration<=5. or not 1<=a.wall_per_case<=300. or a.limit_cases<0: p.error('duration (0,5], wall-per-case [1,300], limit-cases >=0 required')
    model=compile_formulation(str(FORMULATION)).model
    cases=[(v,t,pedal,0.,'cap') for v,t in ((20.,10.),(15.,5.)) for pedal in (1.,0.,-.5)]
    cases += [(15.,7.,0.,k,'corner') for k in (-1/30,1/30)]
    cases += [(15.,0.,0.,0.,'stop'),(15.,20.,0.,0.,'seam'),(15.,0.,0.,0.,'steady')]
    if a.limit_cases: cases=cases[:a.limit_cases]
    rows=[]
    for i,c in enumerate(cases):
        trace_path=(a.trace.with_name(f'{a.trace.stem}-{i+1:02d}{a.trace.suffix or ".npz"}') if a.trace else None)
        try: row=run_case(model,c,a.duration,a.wall_per_case,trace_path)
        except Exception as e: row={'scenario':c,'error':f'{type(e).__name__}: {e}'}
        rows.append(row); print(f'{i+1}/{len(cases)} {row.get("scenario")} => {row.get("gates",row.get("error"))}',flush=True)
    data={'scope':'actual full 38-state bounded maneuver matrix including true periodic closed-seam path and stop event threshold 0.1 m/s',
        'formulation_sha256':hashlib.sha256(FORMULATION.read_bytes()).hexdigest(),
        'hydraulic_sha256':hashlib.sha256(HYDRAULIC.read_bytes()).hexdigest(),
        'duration_s':a.duration,'rows':rows}
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(data,indent=2)+'\n')
if __name__=='__main__': main()
