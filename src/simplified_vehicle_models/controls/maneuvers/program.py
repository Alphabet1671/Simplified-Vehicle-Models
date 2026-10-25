"""Sampled maneuver controller composition with explicit channel ownership."""
import math
from .pid import PidConfig, SampledPid
from .types import RobotCommand
from .waveforms import SteeringWaveform


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class ManeuverProgram:
    def __init__(self, definition, csv_source=None):
        if not hasattr(definition, 'to_dict'):
            raise TypeError('definition must provide to_dict()')
        self.definition = definition
        self.data = definition.to_dict()
        self.csv_source = csv_source
        if self.data['preset'] == 'csv_playback' and csv_source is None:
            raise ValueError('CSV playback requires a CSV source')
        self.reset()

    def reset(self):
        d = self.data
        s, l = d['steering'], d['longitudinal']
        self._steer_pid = self._pid(s, max_swa=True) if s.get('mode') in ('path_pid','radius_pid') else None
        self._speed_pid = self._pid(l, lower=-1., upper=1.) if l.get('mode') == 'cruise_pid' else None
        self._last_phase = None

    @staticmethod
    def _pid(values, max_swa=False, lower=None, upper=None):
        return SampledPid(PidConfig(float(values.get('kp',0)), float(values.get('ki',0)), float(values.get('kd',0)), float(values.get('derivative_tau_s',0)), float(-values.get('max_swa_rad',1) if max_swa else (-1 if lower is None else lower)), float(values.get('max_swa_rad',1) if max_swa else (1 if upper is None else upper))))

    def _steering(self, o, dt):
        s=self.data['steering']; mode=s['mode']; limit=self.data['limits'].get('max_swa_rad', math.pi)
        if mode == 'locked_swa': return float(s.get('target_heading_rad',0.))
        if mode == 'csv': return None
        if mode == 'path_pid':
            error = _wrap(float(s.get('target_heading_rad',0.))-o.heading_rad) - math.atan2(o.y_m, max(abs(o.speed_mps)*float(s.get('preview_distance_m',1.)),1e-9))
            if abs(o.speed_mps) >= float(s.get('minimum_speed_mps',0.)):
                # Convert desired heading correction to lateral-error-like PID input.
                val=self._steer_pid.update(error,0.,dt)
            else:
                val=max(-limit,min(limit,float(s.get('kp',0))*error))
            return max(-limit,min(limit,val))
        if mode == 'radius_pid':
            direction=float(s.get('direction',1)); radius=float(s.get('target_radius_m',1.))
            radial=math.hypot(o.x_m,o.y_m)
            error=radial-radius
            correction=self._steer_pid.update(0.,error,dt)
            feed=direction*math.atan2(max(0.,o.speed_mps)**2, 9.80665*radius)
            return max(-limit,min(limit,feed-direction*correction))
        # Peak steering waveform: use explicit profile duration/rise as waveform timing.
        preset=self.data['preset']; kind={'step_steer':'step','ramp_steer':'ramp','sine_steer':'sine'}.get(preset)
        if kind:
            p=self.data['profile']; amp=float(s.get('amplitude_rad',0.)); active=float(p.get('ramp_s',p.get('duration_s',1.)))
            frequency=float(s.get('frequency_hz',1.)); lead=float(p.get('lead_in_s',0.)); release=float(p.get('release_s',0.))
            return SteeringWaveform(kind,amp,lead,active,release,frequency).sample(o.time_s)
        if mode == 'peak_ay': return float(s.get('direction',1))*float(s.get('amplitude_mps2',0))*float(s.get('target_radius_m',1))/max(o.speed_mps**2,1e-9)
        return 0.

    def _longitudinal(self, o, dt):
        l=self.data['longitudinal']; mode=l['mode']
        if mode=='csv': return None
        if mode=='constant_accelerator': return max(0.,min(1.,float(l.get('accelerator',0.)))),0.
        if mode=='cruise_pid':
            demand=self._speed_pid.update(float(l.get('target_speed_mps',0)),o.speed_mps,dt)
            return max(0.,demand),max(0.,-demand)
        if mode=='fixed_pedal':
            a=float(l.get('start_accelerator',l.get('accelerator',0))); b=float(l.get('start_brake',0))
            return max(0.,min(1.,a)),max(0.,min(1.,b))
        if mode=='profile':
            p=self.data['profile']; t=o.time_s-float(p.get('lead_in_s',0)); rise=float(p.get('rise_s',1)); ramp=float(p.get('ramp_s',1)); hold=float(p.get('hold_s',1))
            phase='lead' if t<0 else 'rise' if t<rise else 'ramp' if t<rise+ramp else 'hold'
            self._last_phase=phase
            alpha=0. if t<0 else min(1.,max(0.,t/max(rise,1e-9)))
            a=float(l.get('start_accelerator',0))*(1-alpha)+float(l.get('end_accelerator',0))*alpha
            b=float(l.get('start_brake',0))*(1-alpha)+float(l.get('end_brake',0))*alpha
            if a>0 and b>0: return (a,0.) if a>=b else (0.,b)
            return a,b
        return 0.,0.

    def tick(self, observation, dt_s):
        if self.data['preset']=='csv_playback': return self.csv_source.sample(observation.time_s)
        swa=self._steering(observation,dt_s)
        pedals=self._longitudinal(observation,dt_s)
        return RobotCommand(max(-self.data['limits'].get('max_swa_rad',math.pi),min(self.data['limits'].get('max_swa_rad',math.pi),swa or 0.)),pedals[0],pedals[1])

    def breakpoints(self):
        if self.csv_source is not None: return tuple(self.csv_source.breakpoints())
        p=self.data['profile']; vals=[0.]
        total=0.
        for k in ('lead_in_s','rise_s','ramp_s','hold_s','release_s','recovery_s'):
            total+=float(p.get(k,0)); vals.append(total)
        duration=float(p.get('duration_s',total))
        return tuple(sorted(set([x for x in vals if x<=duration]+[duration])))

    def termination_reason(self, observation):
        return termination_reason(observation, self.data)


def termination_reason(observation, definition):
    data = definition.to_dict() if hasattr(definition, 'to_dict') else definition
    term=data.get('termination',{}); limits=data.get('limits',{})
    duration=float(term.get('duration_s', data.get('profile',{}).get('duration_s',math.inf)))
    if observation.time_s>=duration: return 'duration'
    if 'speed_mps' in term and observation.speed_mps>=float(term['speed_mps']): return 'speed'
    distance=float(term.get('distance_m',limits.get('max_distance_m',math.inf)))
    if math.hypot(observation.x_m,observation.y_m)>=distance:return 'distance'
    return None
