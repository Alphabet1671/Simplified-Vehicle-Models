"""Candidate detection and physical screening for transient tire events."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .contracts import RunData


@dataclass(frozen=True)
class TransientEvent:
    run: RunData
    start: int
    stop: int
    baseline_stop: int
    approved: bool = False
    axis: str = 'lateral'
    exclusions: tuple[str, ...] = ()


_REQUIRED = ('time','alpha','kappa','fz','speed','omega','ia','pressure','fx','fy')


def validate_event(event: TransientEvent, *, require_approval=True, check_conditions=True):
    run = event.run
    if require_approval and not event.approved:
        raise ValueError('transient event requires approved status from explicit user approval')
    if event.axis not in ('lateral','longitudinal'):
        raise ValueError('event axis must be lateral or longitudinal')
    if any(name not in run.channels for name in _REQUIRED):
        raise ValueError('event is missing required input or output channels')
    if event.axis == 'lateral' and 'fy' not in run.channels:
        raise ValueError('lateral event is missing Fy')
    if event.axis == 'longitudinal' and 'fx' not in run.channels:
        raise ValueError('longitudinal event is missing Fx')
    n = len(run.raw_indices)
    if any(np.asarray(value).size != n for value in run.channels.values()):
        raise ValueError('event channels and raw indices have mismatched lengths')
    if not (0 <= event.start < event.baseline_stop < event.stop <= n):
        raise ValueError('invalid event or baseline bounds')
    if event.baseline_stop - event.start < 20:
        raise ValueError('baseline must contain at least 0.2 seconds of samples')
    channels = run.channels
    t = channels['time'][event.start:event.stop]
    if len(t) < 4 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('event elapsed time must be strictly increasing')
    if np.any(np.diff(t) > .02 + 1e-12):
        raise ValueError('event crosses a time gap; event must be contiguous')
    baseline = slice(event.start,event.baseline_stop)
    required = _REQUIRED
    for name in required:
        if name not in channels:
            raise ValueError(f'event is missing required channel {name}')
        if not np.all(np.isfinite(channels[name][event.start:event.stop])):
            raise ValueError(f'event channel {name} contains nonfinite values')
    fz = channels['fz'][baseline]
    ia = channels['ia'][baseline]
    pressure = channels['pressure'][baseline]
    speed = channels['speed'][baseline]
    if not check_conditions:
        return True
    def check_relative(values, reference, label):
        reference=np.asarray(reference,dtype=float)
        values=np.asarray(values,dtype=float)
        mean=float(np.mean(reference))
        if mean <= 0 or (np.max(values)-np.min(values))/mean > .10:
            raise ValueError(f'{label} is not stable within 10% across event')
    if np.mean(fz) <= 0 or (np.max(fz)-np.min(fz))/np.mean(fz) > .10:
        raise ValueError('baseline load is not stable within 10%')
    if np.max(np.abs(ia)) > np.deg2rad(.25):
        raise ValueError('baseline inclination is outside +/-0.25 degrees')
    if np.mean(pressure) <= 0 or (np.max(pressure)-np.min(pressure)) > 3000:
        raise ValueError('baseline pressure is not stable within 3 kPa')
    if np.mean(speed) <= .5 or (np.max(speed)-np.min(speed))/np.mean(speed) > .10:
        raise ValueError('baseline speed is not stable within 10% or below runtime floor')
    event_slice=slice(event.start,event.stop)
    check_relative(channels['fz'][event_slice],fz,'event load')
    check_relative(channels['speed'][event_slice],speed,'event speed')
    if np.max(np.abs(channels['ia'][event_slice])) > np.deg2rad(.25):
        raise ValueError('event inclination is outside +/-0.25 degrees')
    if np.max(channels['pressure'][event_slice])-np.min(channels['pressure'][event_slice]) > 3000:
        raise ValueError('event pressure is not stable within 3 kPa')
    return True


def candidate_events(run: RunData) -> tuple[TransientEvent, ...]:
    c = run.channels
    missing = [n for n in _REQUIRED if n not in c]
    if missing:
        return ()
    t = c['time']
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('event detection requires strictly increasing elapsed time')
    # Gaps divide independent continuous traces. Condition changes are screened
    # over the proposed pre-transition baseline and recorded as exclusions rather
    # than used as pointwise cuts (which fragments noisy measured channels).
    cuts = set((np.flatnonzero(np.diff(t) > .02)+1).tolist())
    bounds = [0,*sorted(cuts),len(t)]
    found=[]
    for lo,hi in zip(bounds[:-1],bounds[1:]):
        if hi-lo < 25: continue
        alpha=c['alpha'][lo:hi]
        # Find cumulative one-degree transitions over <=1 s, permitting a
        # measured ramp whose individual samples are each below one degree.
        candidates=[]
        window=max(2,int(round(1.0/np.median(np.diff(t[lo:hi])))))
        for j in range(1,len(alpha)):
            first=max(0,j-window)
            if abs(alpha[j]-alpha[first])>=np.deg2rad(1.):
                candidates.append(lo+j)
                break
        for edge in candidates:
            base=max(lo,edge-20)
            end=min(hi,edge+max(20,int(round(1.0/np.median(np.diff(t[lo:hi]))))))
            if edge-base < 20 or end-edge < 3: continue
            # For a ramp, baseline ends at the onset, not at the later
            # one-degree threshold crossing. Reserve at least 20 prior samples.
            deviations=np.abs(alpha[:j+1]-alpha[0])
            onset_candidates=np.flatnonzero(deviations>=np.deg2rad(.2))
            onset=int(lo+onset_candidates[0]) if onset_candidates.size else edge
            base=max(lo,onset-20)
            event=TransientEvent(run,base,end,onset,False,'lateral',('requires explicit user review/approval',))
            reasons=[]
            try: validate_event(event,require_approval=False)
            except ValueError as exc: reasons.append(str(exc))
            baseline_start=event.baseline_stop-20
            for name,tolerance in (('fz',.10),('speed',.10)):
                baseline_values=c[name][baseline_start:event.baseline_stop]
                mean=float(np.mean(baseline_values))
                if mean<=0 or (np.max(baseline_values)-np.min(baseline_values))/mean>tolerance:
                    reasons.append(f'baseline {name} not stable within {tolerance*100:g}%')
            if np.max(np.abs(c['ia'][baseline_start:edge]))>np.deg2rad(.25): reasons.append('baseline inclination outside +/-0.25 degrees')
            if np.max(c['pressure'][baseline_start:edge])-np.min(c['pressure'][baseline_start:edge])>3000: reasons.append('baseline pressure not stable within 3 kPa')
            # Screen the complete proposed event window, including post-baseline
            # condition excursions; retain each reason for the approval UI.
            event_slice=slice(base,end)
            event_fz=c['fz'][event_slice]; event_speed=c['speed'][event_slice]
            if np.mean(fz:=c['fz'][base:onset]) <= 0 or (np.max(event_fz)-np.min(event_fz))/np.mean(fz) > .10:
                reasons.append('event load is not stable within 10% across event')
            if np.mean(speed:=c['speed'][base:onset]) <= 0 or (np.max(event_speed)-np.min(event_speed))/np.mean(speed) > .10:
                reasons.append('event speed is not stable within 10% across event')
            if np.max(np.abs(c['ia'][event_slice])) > np.deg2rad(.25): reasons.append('event inclination outside +/-0.25 degrees')
            if np.max(c['pressure'][event_slice])-np.min(c['pressure'][event_slice]) > 3000: reasons.append('event pressure not stable within 3 kPa')
            found.append(TransientEvent(run,base,end,onset,False,'lateral',tuple(dict.fromkeys(reasons or event.exclusions))))
            break
    return tuple(found)
