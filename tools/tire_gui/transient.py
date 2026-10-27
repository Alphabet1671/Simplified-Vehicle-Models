"""Time-domain replay and gated relaxation parameter identification."""
from __future__ import annotations
from typing import Callable
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
from simplified_vehicle_models.tires.unitire_transient import (TransientTireState,
    steady_rolling_state, evaluate_transient_unitire)
from .contracts import FitCancelled, FitResult, ModelSnapshot
from .events import TransientEvent, validate_event
from .model import with_parameters

_BOUNDS={'longitudinal_length_scale':(.05,10.),'lateral_length_scale':(.05,10.),
         'trail_pole_ratio':(.05,20.),'trail_lead_ratio':(0.,1.)}

def _cancel(cancel):
    if cancel(): raise FitCancelled()

def _inputs(run, ix):
    c=run.channels
    return (float(c['alpha'][ix]),float(c['kappa'][ix]),float(c['fz'][ix]),
            float(c['omega'][ix]),float(c['speed'][ix]))

def simulate_event(snapshot: ModelSnapshot, event: TransientEvent, maximum_step: float, *, _cancel_check: Callable[[], bool] = lambda: False) -> np.ndarray:
    if not np.isfinite(maximum_step) or maximum_step <= 0: raise ValueError('maximum_step must be finite and positive')
    validate_event(event,require_approval=False)
    run=event.run; c=run.channels; start,stop=event.start,event.stop
    t=np.asarray(c['time'][start:stop],float)
    if np.any(c['speed'][start:stop] <= 0): raise ValueError('measured event speed must be positive')
    steady, transient=snapshot.steady,snapshot.transient
    ia0=_inputs(run,event.start); state=steady_rolling_state(steady,transient,ia0[0],ia0[1],ia0[2])
    y0=np.array([state.longitudinal_slip,state.lateral_deformation,state.trail_angle],float)
    output=np.empty((len(t),3),float)
    def rhs(tt,y):
        _cancel(_cancel_check)
        j=min(max(int(np.searchsorted(t,tt,side='right')-1),0),len(t)-2)
        f=(tt-t[j])/(t[j+1]-t[j])
        vals=[(1-f)*c[name][start+j]+f*c[name][start+j+1] for name in ('alpha','kappa','fz','omega','speed')]
        forces,rates=evaluate_transient_unitire(steady,transient,TransientTireState(*map(float,y)),*map(float,vals))
        return [rates.longitudinal_slip,rates.lateral_deformation,rates.trail_angle]
    y=y0
    # Integrate each measured command interval explicitly, preserving all knots.
    for j in range(len(t)-1):
        _cancel(_cancel_check)
        sol=solve_ivp(rhs,(t[j],t[j+1]),y,method='RK45',rtol=1e-6,atol=1e-9,max_step=maximum_step,t_eval=[t[j+1]])
        if not sol.success: raise ValueError(f'transient integration failed: {sol.message}')
        y=sol.y[:,-1]
        # evaluate at observed sample using actual measured V and N
        force,_=evaluate_transient_unitire(steady,transient,TransientTireState(*map(float,y)),*_inputs(run,start+j+1))
        output[j+1]=[force.longitudinal_force,force.lateral_force,force.aligning_moment]
    force,_=evaluate_transient_unitire(steady,transient,state,*_inputs(run,start))
    output[0]=[force.longitudinal_force,force.lateral_force,force.aligning_moment]
    _cancel(_cancel_check)
    if not np.all(np.isfinite(output)): raise ValueError('nonfinite transient replay')
    return output

def fit_transient(snapshot: ModelSnapshot, events: tuple[TransientEvent,...], locked: frozenset[str], cancel: Callable[[],bool]=lambda:False, progress: Callable[[dict],None]=lambda _:None) -> FitResult:
    _cancel(cancel)
    if not events: raise ValueError('at least one approved transient event is required')
    for event in events: validate_event(event)
    lat=[e for e in events if e.axis=='lateral']
    lon=[e for e in events if e.axis=='longitudinal']
    if not lat and not lon: raise ValueError('no excited transient event axes')
    if any(np.ptp(e.run.channels['alpha'][e.start:e.stop]) < np.deg2rad(1) for e in lat): raise ValueError('lateral event lacks sufficient excitation')
    if any(np.ptp(e.run.channels['kappa'][e.start:e.stop]) < .01 for e in lon): raise ValueError('longitudinal event lacks sufficient excitation')
    candidate=snapshot; metrics={}; statuses={}; warnings=[]; converged=True
    def fit_group(group, names, output_col):
        nonlocal candidate,converged
        active=[n for n in names if n not in locked]
        if not active:
            for n in names: statuses[n]='locked'
            return
        base=candidate
        if any(n in locked for n in names):
            for n in names: statuses[n]='locked' if n in locked else 'unchanged (fit group locked)'
            return
        # Save prediction targets and normalized weights; one RMS contribution per event.
        targets=[]
        for e in group:
            pred=simulate_event(base,e,.01,_cancel_check=cancel)
            col={'fx':0,'fy':1,'mz':2}[output_col]
            target=e.run.channels['fx' if col==0 else 'fy' if col==1 else 'mz'][e.start:e.stop]
            load=e.run.channels['fz'][e.start:e.stop]
            radius=max(base.steady.r_1,.001)
            scale=load if col<2 else np.maximum(load*radius,1.)
            targets.append((e,col,target,scale))
        if not targets: return
        bounds=np.array([_BOUNDS[n] for n in active],float)
        x0=np.array([base.values[n] for n in active]); x0=np.clip(x0,bounds[:,0]+1e-9,bounds[:,1]-1e-9)
        def residual(x):
            _cancel(cancel)
            trial=with_parameters(base,dict(zip(active,x)))
            chunks=[]
            for e,col,target,scale in targets:
                _cancel(cancel)
                pred=simulate_event(trial,e,.01,_cancel_check=cancel)
                chunks.append((pred[:,col]-target)/scale/np.sqrt(len(target)*len(targets)))
            return np.concatenate(chunks)
        result=least_squares(residual,x0,bounds=(bounds[:,0],bounds[:,1]),max_nfev=100, diff_step=1e-3)
        _cancel(cancel)
        # Numerical response times below two samples cannot be distinguished;
        # do not accept a nominal full-rank optimizer result in that case.
        dt=float(np.median(np.diff(group[0].run.channels['time'][group[0].start:group[0].stop])))
        from simplified_vehicle_models.tires.unitire_transient import relaxation_lengths
        load0=float(np.median(group[0].run.channels['fz'][group[0].start:group[0].stop]))
        relevant='longitudinal_length_scale' if output_col=='fx' else 'lateral_length_scale'
        sigma=relaxation_lengths(base.steady,base.transient,load0)[0 if output_col=='fx' else 1]
        max_time_constant=(_BOUNDS[relevant][1]/max(base.values[relevant],1e-12))*sigma/max(group[0].run.channels['speed'][group[0].start],.5) if relevant in active else sigma/max(group[0].run.channels['speed'][group[0].start],.5)
        time_resolved=max_time_constant >= 2*dt
        jac=np.asarray(result.jac,float); s=np.linalg.svd(jac,compute_uv=False)
        rank=int(np.sum(s > (s[0]*1e-6 if len(s) and s[0]>0 else np.inf)))
        cond=float(s[0]/s[-1]) if len(s) and s[-1]>0 else float('inf')
        metrics[f'{group[0].axis}_sensitivity_rank']=float(rank); metrics[f'{group[0].axis}_sensitivity_condition']=cond
        identified=result.success and rank==len(active) and cond<=1e6 and time_resolved
        if not time_resolved: warnings.append(f'{group[0].axis} response time is below two sample intervals')
        if identified:
            candidate=with_parameters(base,dict(zip(active,result.x)))
        else:
            warnings.append(f'{group[0].axis} transient sensitivity is rank-deficient or ill-conditioned; parameter changes reverted')
        converged &= bool(identified)
        for n in names: statuses[n]='fitted' if identified and n in active else ('locked' if n in locked else 'unidentified / provisional')
        metrics[f'{group[0].axis}_response_bandwidth_hz']=float(1/(2*np.median(np.diff(group[0].run.channels['time'][group[0].start:group[0].stop]))))
        progress({'stage':group[0].axis,'cost':float(result.cost),'rank':rank,'condition':cond})
    if lat:
        fit_group(lat,['lateral_length_scale'],'fy')
        # Only attempt trail after lateral is identified and Mz is present.
        if any('mz' not in e.run.channels for e in lat):
            raise ValueError('Mz/moment channel required for trail fit')
        if statuses.get('lateral_length_scale')=='fitted' and not any(n in locked for n in ('trail_pole_ratio','trail_lead_ratio')):
            fit_group(lat,['trail_pole_ratio','trail_lead_ratio'],'mz')
        elif statuses.get('lateral_length_scale')=='fitted':
            for n in ('trail_pole_ratio','trail_lead_ratio'):
                statuses[n]='locked' if n in locked else 'unchanged (other trail coefficient locked)'
        else:
            statuses['trail_pole_ratio']=statuses['trail_lead_ratio']='unidentified / provisional'
    else:
        statuses['lateral_length_scale']='unidentified / provisional'; statuses['trail_pole_ratio']=statuses['trail_lead_ratio']='unidentified / provisional'
    if lon: fit_group(lon,['longitudinal_length_scale'],'fx')
    else: statuses['longitudinal_length_scale']='unidentified / provisional'
    # Diagnostics are checked for cancellation; statuses remain conservative unless all active groups identifiable.
    _cancel(cancel)
    metrics['event_count']=float(len(events))
    provenance=dict(candidate.provenance); provenance['transient_fit_status']='fitted' if converged else 'provisional'
    statusmap=dict(candidate.parameter_status); statusmap.update(statuses)
    candidate=type(candidate)(candidate.revision,candidate.steady,candidate.transient,candidate.values,provenance,statusmap,candidate.steady_fingerprint,candidate.load_domain)
    return FitResult(snapshot.revision,candidate,metrics,converged,tuple(warnings),statuses)
