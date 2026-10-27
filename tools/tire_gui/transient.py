"""Time-domain replay and gated relaxation parameter identification."""
from __future__ import annotations
from typing import Callable
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
from simplified_vehicle_models.tires.unitire_transient import (TransientTireState,
    steady_rolling_state, evaluate_transient_unitire, relaxation_lengths)
from .contracts import FitCancelled, FitResult, ModelSnapshot
from .events import TransientEvent, validate_event
from .model import with_parameters

_BOUNDS={'longitudinal_length_scale':(.05,10.),'lateral_length_scale':(.05,10.),
         'trail_pole_ratio':(.05,20.),'trail_lead_ratio':(0.,1.)}

def _cancel(cancel):
    if cancel(): raise FitCancelled()

def _excited_bandwidth(values, dt):
    values=np.asarray(values,dtype=float)
    centered=values-np.mean(values)
    if len(centered)<4 or not np.any(np.abs(centered)>0):
        return None
    power=np.abs(np.fft.rfft(centered))**2
    frequency=np.fft.rfftfreq(len(centered),dt)
    power[0]=0.
    total=float(np.sum(power))
    if total<=0: return None
    cumulative=np.cumsum(power)/total
    ix=int(np.searchsorted(cumulative,.95))
    return float(frequency[min(ix,len(frequency)-1)])


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
        original_names=tuple(names)
        active=[n for n in original_names if n not in locked]
        if not active:
            for n in names: statuses[n]='locked'
            return
        base=candidate
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
        result=least_squares(residual,x0,bounds=(bounds[:,0],bounds[:,1]),max_nfev=100, diff_step=1e-3, callback=lambda _: _cancel(cancel))
        _cancel(cancel)
        # Judge the candidate's actual physical time constants, not the maximum
        # search bound. Check every load/speed pair across every contributing event.
        dt_by_event=[]
        tau_by_parameter={name:[] for name in active}
        tau_trial=with_parameters(base,dict(zip(active,result.x)))
        for e in group:
            ec=e.run.channels
            dt_by_event.append(float(np.median(np.diff(ec['time'][e.start:e.stop]))))
            for fz,vx in zip(ec['fz'][e.start:e.stop],ec['speed'][e.start:e.stop]):
                sx,sy=relaxation_lengths(tau_trial.steady,tau_trial.transient,float(fz))
                tau={'longitudinal_length_scale':sx/max(abs(float(vx)),.5),
                     'lateral_length_scale':sy/max(abs(float(vx)),.5),
                     'trail_pole_ratio':sy/(tau_trial.transient.trail_pole_ratio*max(abs(float(vx)),.5)),
                     'trail_lead_ratio':sy/max(abs(float(vx)),.5)}
                for name in active: tau_by_parameter[name].append(tau[name])
        time_resolved=True
        ratio_by_parameter={name:[] for name in active}
        offset=0
        for event_index,e in enumerate(group):
            count=e.stop-e.start
            for name in active:
                ratio_event=min(tau_by_parameter[name][offset:offset+count])/(2*dt_by_event[event_index])
                metrics[f'{name}_event_{event_index}_tau_over_2dt']=float(ratio_event)
                if ratio_event < 1:
                    warnings.append(f'{name} candidate response time is below two sample intervals for event {event_index}')
                    time_resolved=False
            offset+=count
        for name in active:
            ratio=min(metrics[f'{name}_event_{i}_tau_over_2dt'] for i in range(len(group)))
            metrics[f'{name}_minimum_tau_seconds']=float(min(tau_by_parameter[name]))
            metrics[f'{name}_minimum_tau_over_2dt']=float(ratio)
            if ratio < 1:
                warnings.append(f'{name} candidate response time is below two sample intervals for at least one event')
                time_resolved=False
        jac=np.asarray(result.jac,float); s=np.linalg.svd(jac,compute_uv=False)
        rank=int(np.sum(s > (s[0]*1e-6 if len(s) and s[0]>0 else np.inf)))
        cond=float(s[0]/s[-1]) if len(s) and s[-1]>0 else float('inf')
        metrics[f'{group[0].axis}_sensitivity_rank']=float(rank); metrics[f'{group[0].axis}_sensitivity_condition']=cond
        bound_hit=False
        for name,value,(lower,upper) in zip(active,result.x,bounds):
            hit=bool(value <= lower + 1e-6 * max(1.,abs(lower)) or value >= upper - 1e-6 * max(1.,abs(upper)))
            metrics[f'{name}_bound_hit']=hit
            if hit:
                bound_hit=True
                warnings.append(f'{name} candidate reached a search bound')
        identified=result.success and rank==len(active) and cond<=1e6 and time_resolved and not bound_hit
        if not time_resolved: warnings.append(f'{group[0].axis} response time is below two sample intervals')
        if identified:
            candidate=with_parameters(base,dict(zip(active,result.x)))
        else:
            warnings.append(f'{group[0].axis} transient sensitivity is rank-deficient or ill-conditioned; parameter changes reverted')
        converged &= bool(identified)
        for n in original_names: statuses[n]='fitted' if identified and n in active else ('locked' if n in locked else 'unidentified / provisional')
        dt=float(np.median(np.diff(group[0].run.channels['time'][group[0].start:group[0].stop])))
        metrics[f'{group[0].axis}_nyquist_hz']=float(.5/dt)
        measured_bandwidth=_excited_bandwidth(group[0].run.channels['alpha' if group[0].axis=='lateral' else 'kappa'][group[0].start:group[0].stop],dt)
        metrics[f'{group[0].axis}_excited_input_bandwidth_95_hz']=float(measured_bandwidth) if measured_bandwidth is not None else float('nan')
        progress({'stage':group[0].axis,'cost':float(result.cost),'rank':rank,'condition':cond})
    if lat:
        fit_group(lat,['lateral_length_scale'],'fy')
        # Only attempt trail after lateral is identified and Mz is present.
        # Missing Mz only excludes trail fitting; it does not invalidate Fy.
        # A trusted locked lateral scale can identify trail dynamics even when
        # this run's Fy fit is intentionally disabled.
        lateral_usable = statuses.get('lateral_length_scale') == 'fitted' or 'lateral_length_scale' in locked
        if lateral_usable:
            trail_events=[e for e in lat if 'mz' in e.run.channels]
            if trail_events:
                fit_group(trail_events,['trail_pole_ratio','trail_lead_ratio'],'mz')
            else:
                warnings.append('Mz unavailable; trail coefficients were not fitted')
                for n in ('trail_pole_ratio','trail_lead_ratio'):
                    statuses[n]='locked' if n in locked else 'unidentified / provisional (Mz unavailable)'
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
