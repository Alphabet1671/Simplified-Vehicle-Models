"""Extract and group measured monotonic tire-input ramps for transient plots."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .contracts import RunData
from .data import contiguous_slices
from .events import TransientEvent


@dataclass(frozen=True)
class RampGroup:
    """Events sharing one measured ramp condition across load levels."""
    events: tuple[TransientEvent, ...]
    belt_speed: float
    ramp_time: float
    steer_rate: float
    direction: int
    axis: str
    tire_id: str = ''
    inclination: float = 0.0
    pressure: float = 0.0
    secondary_slip: float = 0.0

    def aligned_time(self, event: TransientEvent) -> np.ndarray:
        """Return event time relative to its interpolated excitation midpoint."""
        if event not in self.events:
            raise ValueError('event does not belong to this ramp group')
        return aligned_time(event, direction=self.direction)


def aligned_time(event: TransientEvent, *, direction: int | None = None) -> np.ndarray:
    """Align an event using only its measured alpha/kappa midpoint crossing."""
    c = event.run.channels
    signal = c['alpha' if event.axis == 'lateral' else 'kappa']
    values = np.asarray(signal[event.baseline_stop:event.stop], float)
    times = np.asarray(c['time'][event.baseline_stop:event.stop], float)
    if len(values) < 2:
        raise ValueError('ramp event has no measured input interval')
    if direction is None:
        direction = 1 if values[-1] >= values[0] else -1
    sign = 1. if direction == 1 else -1.
    endpoint = int(np.argmax(sign * (values - values[0])))
    values, times = values[:endpoint+1], times[:endpoint+1]
    midpoint = .5 * (float(values[0]) + float(values[-1]))
    crossing = np.flatnonzero(sign * (values-midpoint) >= 0)
    if not crossing.size:
        raise ValueError('ramp does not cross its input midpoint')
    j = int(crossing[0])
    if j == 0:
        crossing_time = float(times[0])
    else:
        v0, v1 = float(values[j-1]), float(values[j])
        frac = 0. if v1 == v0 else (midpoint-v0)/(v1-v0)
        crossing_time = float(times[j-1] + frac*(times[j]-times[j-1]))
    result = np.asarray(c['time'][event.start:event.stop] - crossing_time, dtype=float)
    result.setflags(write=False)
    return result


def _smooth(values: np.ndarray) -> np.ndarray:
    # ~0.1s median rejects isolated encoder glitches and high-frequency jitter.
    padded = np.pad(values, (5, 5), mode='edge')
    return np.array([np.median(padded[i:i+11]) for i in range(len(values))])


def _ramp_intervals(signal: np.ndarray, time: np.ndarray, *, axis: str):
    """Yield ramp spans using an extremum/reversal hysteresis, not derivative zeros."""
    smooth = _smooth(np.asarray(signal, float))
    movement = np.deg2rad(.5) if axis == 'lateral' else .02
    reversal = np.deg2rad(.15) if axis == 'lateral' else .005
    if len(smooth) < 4:
        return
    # Track the current extremum. Tiny counter-motions and held-command jitter do
    # not end a ramp; only a meaningful reversal confirms a turning point.
    start = 0
    sign = 0
    extreme = 0
    for i in range(1, len(smooth)):
        if sign == 0:
            change = smooth[i] - smooth[start]
            if abs(change) >= reversal:
                sign = 1 if change > 0 else -1
                extreme = i
            continue
        if sign * (smooth[i] - smooth[extreme]) > 0:
            extreme = i
            continue
        if sign * (smooth[extreme] - smooth[i]) >= reversal:
            if (sign * (smooth[extreme] - smooth[start]) >= movement
                    and time[extreme] - time[start] >= .05):
                onset_tol = np.deg2rad(.03) if axis == 'lateral' else .001
                candidates = np.flatnonzero(sign * (smooth[start:extreme+1] - smooth[start]) >= onset_tol)
                if candidates.size:
                    yield start + int(candidates[0]), extreme, sign
            start = extreme
            sign = -sign
            extreme = i
    if sign and sign * (smooth[extreme]-smooth[start]) >= movement and time[extreme]-time[start] >= .05:
        onset_tol = np.deg2rad(.03) if axis == 'lateral' else .001
        candidates = np.flatnonzero(sign * (smooth[start:extreme+1]-smooth[start]) >= onset_tol)
        if candidates.size:
            yield start + int(candidates[0]), extreme, sign


def ramp_events(run: RunData) -> tuple[TransientEvent, ...]:
    """Detect all sufficiently large alpha/kappa ramps without crossing gaps."""
    c = run.channels
    if 'time' not in c:
        return ()
    t = np.asarray(c['time'], float)
    if not np.all(np.isfinite(t)):
        raise ValueError('ramp detection requires finite time')
    if any(np.asarray(v).size != t.size for v in c.values()):
        raise ValueError('ramp channels have mismatched lengths')
    # Any clock reset, duplicate timestamp, or >20ms acquisition gap is a hard
    # boundary; event windows and replay remain strictly increasing/contiguous.
    breaks = np.flatnonzero((np.diff(t) <= 0) | (np.diff(t) > .02)) + 1
    bounds = np.r_[0, breaks, len(t)]
    slices = tuple(slice(int(a), int(b)) for a,b in zip(bounds[:-1],bounds[1:]) if b>a)
    found = []
    for sl in slices:
        if sl.stop - sl.start < 24:
            continue
        local_t = t[sl]
        for source, axis in (('alpha', 'lateral'), ('kappa', 'longitudinal')):
            if source not in c:
                continue
            values = np.asarray(c[source][sl], float)
            if not np.all(np.isfinite(values)):
                continue
            for first, last, sign in _ramp_intervals(values, local_t, axis=axis):
                onset = sl.start + first
                finish = sl.start + last
                start = max(sl.start, onset-20)
                # include a short measured tail for post-ramp response, bounded by trace/gap
                stop = min(sl.stop, finish+4)
                baseline_stop = onset
                if baseline_stop-start < 20 or stop-baseline_stop < 4:
                    continue
                found.append(TransientEvent(run, start, stop, baseline_stop, False, axis,
                                            ('ramp display only; requires fit eligibility review',)))
    found.sort(key=lambda e: (e.run.run_id, e.start, e.baseline_stop, e.axis))
    return tuple(found)


def isolated_step_events(run: RunData) -> tuple[TransientEvent, ...]:
    """Detect isolated, zero-baseline lateral command steps with sustained holds."""
    c = run.channels
    if any(name not in c for name in ('time', 'alpha', 'ia')):
        return ()
    t = np.asarray(c['time'], float)
    alpha = np.asarray(c['alpha'], float)
    ia = np.asarray(c['ia'], float)
    if not (len(t) == len(alpha) == len(ia)) or not np.all(np.isfinite(t)):
        return ()
    if not np.all(np.isfinite(alpha)) or not np.all(np.isfinite(ia)):
        return ()
    # TTC datasets encode SA in degrees but normalization is expected upstream.
    # Guard against improperly normalized huge-angle recordings, never reinterpret.
    breaks = np.flatnonzero((np.diff(t) <= 0) | (np.diff(t) > .02)) + 1
    bounds = np.r_[0, breaks, len(t)]
    found = []
    zero_tol = np.deg2rad(.15)
    excursion = np.deg2rad(.5)
    for a, b in zip(bounds[:-1], bounds[1:]):
        if b-a < 55:
            continue
        local = _smooth(alpha[a:b])
        lt = t[a:b]
        # Candidate onset is a threshold crossing from a verified zero hold.
        for direction in (1, -1):
            progress = direction * local
            above = progress >= np.deg2rad(.25)
            edges = np.flatnonzero(above & ~np.r_[False, above[:-1]])
            for crossing in edges:
                onset = int(crossing)
                # Backtrack to the first persistent departure from the zero command.
                while onset > 0 and abs(local[onset-1]) > np.deg2rad(.03):
                    onset -= 1
                if onset < 20 or lt[onset]-lt[onset-20] < .19:
                    continue
                if np.max(np.abs(local[max(0,onset-20):onset])) > zero_tol:
                    continue
                # A step must settle quickly, then remain at least .3s on target.
                # Find the first persistent plateau, not a later command's maximum.
                dt = float(np.median(np.diff(lt)))
                search_stop = min(len(local), onset + int(round(2.0 / dt)))
                settle = None
                level = 0.
                hold_samples = max(5, int(round(.30 / dt)))
                for j in range(onset, max(onset, search_stop-hold_samples)):
                    window = progress[j:j+hold_samples]
                    candidate = float(np.median(window))
                    tolerance = max(np.deg2rad(.05), .01*candidate)
                    single_sample_step = j == onset and abs(progress[onset] - candidate) <= tolerance
                    stable_window = (np.max(window)-np.min(window) <= tolerance
                                     and np.max(np.abs(window-candidate)) <= tolerance)
                    if candidate >= excursion and (single_sample_step or stable_window):
                        settle, level = j, candidate
                        break
                if settle is None:
                    continue
                settle_tol = max(np.deg2rad(.12), .1*level)
                duration = lt[settle]-lt[onset]
                if duration < .02 or duration > 2.0:
                    continue
                tail = np.flatnonzero(np.abs(progress[settle:]-level) > settle_tol)
                hold_stop = (settle + int(tail[0])) if tail.size else len(local)
                if hold_stop-settle < 1 or lt[hold_stop-1]-lt[settle] < .30:
                    continue
                # Keep a bounded response tail; later protocol commands are not part of this event.
                hold_stop = min(hold_stop, settle + int(round(.5 / np.median(np.diff(lt)))) + 1)
                start = onset-20
                stop = hold_stop
                if stop-start < 1 or np.max(np.abs(ia[a+start:a+stop])) > np.deg2rad(.25):
                    continue
                found.append(TransientEvent(run, a+start, a+stop, a+onset, False,
                                            'lateral', ('isolated step display only',)))
    # Collapse duplicate direction/crossing detections, preserving outward events only.
    unique = {}
    for event in found:
        unique[(event.start, event.baseline_stop, event.stop)] = event
    return tuple(sorted(unique.values(), key=lambda e: (e.start, e.baseline_stop)))


def _event_signature(event: TransientEvent, *, require_moving: bool = True):
    c = event.run.channels
    name = 'alpha' if event.axis == 'lateral' else 'kappa'
    x = np.asarray(c[name][event.baseline_stop:event.stop], float)
    t = np.asarray(c['time'][event.baseline_stop:event.stop], float)
    if len(x) < 2:
        return None
    # Identify excursion endpoint before the added response tail; derive onset
    # and endpoint from the actual measured command, retaining all command knots.
    direction = 1 if abs(float(np.max(x)-x[0])) >= abs(float(np.min(x)-x[0])) else -1
    progress = direction*(x-x[0])
    endpoint = int(np.argmax(progress))
    amplitude = float(progress[endpoint])
    threshold = np.deg2rad(.5) if event.axis == 'lateral' else .02
    if amplitude < threshold:
        return None
    # The detector's baseline_stop is the first persistent command movement;
    # retain it as onset rather than trimming a fixed slip threshold.
    onset = 0
    # Plateau after the excursion can be long; estimate endpoint from last
    # 1%-of-excursion crossing, ignoring tiny post-command jitter extrema.
    tail_cut = max(np.deg2rad(.03), amplitude*.01) if event.axis == 'lateral' else max(.001, amplitude*.01)
    moving = np.flatnonzero(progress >= amplitude-tail_cut)
    if moving.size:
        endpoint = int(moving[0])
    duration = float(t[endpoint]-t[onset])
    if duration < .02:
        return None
    x0, x1 = float(x[onset]), float(x[endpoint])
    span = slice(event.baseline_stop+onset, event.baseline_stop+endpoint+1)
    speed = float(np.median(c['speed'][span])) if 'speed' in c else float('nan')
    if require_moving and np.min(c['speed'][span]) < .5:
        return None
    full_ia = np.asarray(c['ia'][event.start:event.stop], float) if 'ia' in c else None
    if full_ia is None or not np.all(np.isfinite(full_ia)) or np.max(np.abs(full_ia)) > np.deg2rad(.25):
        return None
    inclination = float(np.median(c['ia'][span]))
    pressure = float(np.median(c['pressure'][span])) if 'pressure' in c else 0.0
    secondary_name = 'kappa' if event.axis == 'lateral' else 'alpha'
    secondary_slip = float(np.median(c[secondary_name][span])) if secondary_name in c else 0.0
    return x0, x1, duration, speed, inclination, pressure, secondary_slip


def grouped_ramps(events: Iterable[TransientEvent], *, require_moving: bool = True) -> tuple[RampGroup, ...]:
    """Group equivalent ramps across loads, tolerating small sampled timing jitter."""
    ordered = sorted(events, key=lambda e: (e.axis, e.run.run_id, e.baseline_stop, e.start))
    buckets: list[tuple[list[TransientEvent], tuple]] = []
    for event in ordered:
        sig = _event_signature(event, require_moving=require_moving)
        if sig is None:
            continue
        x0, x1, duration, speed, inclination, pressure, secondary_slip = sig
        direction = 1 if x1 >= x0 else -1
        rate = (x1-x0)/duration if duration > 0 else 0.
        display_rate = float(np.rad2deg(rate)) if event.axis == 'lateral' else rate
        identity = (event.run.tire_id, inclination, pressure, secondary_slip)
        chosen = None
        if require_moving and speed < .5:
            continue
        for group, ref in buckets:
            if group[0].axis != event.axis:
                continue
            r0, r1, rd, rs, rr, rdir, ridentity = ref
            rid, ria, rp, rsecondary = ridentity
            eid, eia, ep, esecondary = identity
            secondary_tol = .01 if event.axis == 'lateral' else np.deg2rad(.25)
            if (eid != rid or abs(eia-ria) > np.deg2rad(.25)
                    or abs(ep-rp) > 3000. or abs(esecondary-rsecondary) > secondary_tol):
                continue
            amp_tol = np.deg2rad(1.) if event.axis == 'lateral' else .01
            if (direction == rdir and abs(x0-r0) <= amp_tol and abs(x1-r1) <= amp_tol
                    and abs(duration-rd) <= max(.5, .15*rd)
                    and abs(speed-rs) <= max(.5, .025*rs)
                    and abs(rate-rr) <= max(amp_tol/max(rd,.05), .15*abs(rr))):
                chosen = group
                break
        if chosen is None:
            chosen = [event]
            buckets.append((chosen, (x0, x1, duration, speed, rate, direction, identity)))
        else:
            chosen.append(event)
    result = []
    for group, ref in buckets:
        x0, x1, duration, speed, rate, direction, identity = ref
        display_rate = float(np.rad2deg(rate)) if group[0].axis == 'lateral' else rate
        result.append(RampGroup(tuple(group), speed, duration, display_rate, direction,
                                group[0].axis, identity[0], identity[1], identity[2], identity[3]))
    result.sort(key=lambda g: (g.axis, g.belt_speed, g.ramp_time, g.direction,
                               g.events[0].run.run_id, g.events[0].start))
    return tuple(result)
