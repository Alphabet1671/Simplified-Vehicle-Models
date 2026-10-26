"""Pure analytical steering waveform sources."""
from dataclasses import dataclass
import math

@dataclass(frozen=True, slots=True)
class SteeringWaveform:
    kind: str
    amplitude_rad: float
    lead_in_s: float
    active_s: float
    release_s: float
    frequency_hz: float = 1.

    def __post_init__(self):
        if self.kind not in {'step','ramp','sine'}: raise ValueError('unsupported waveform kind')
        vals=(self.amplitude_rad,self.lead_in_s,self.active_s,self.release_s,self.frequency_hz)
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in vals): raise ValueError('waveform values must be finite numbers')
        if self.lead_in_s<0 or self.active_s<=0 or self.release_s<0 or self.frequency_hz<=0: raise ValueError('invalid waveform timing/frequency')
        if self.kind=='sine' and abs(self.active_s*self.frequency_hz-round(self.active_s*self.frequency_hz))>1e-10: raise ValueError('sine active duration must contain an integral number of cycles')

    def sample(self,time_s: float)->float:
        if isinstance(time_s,bool) or not isinstance(time_s,(int,float)) or not math.isfinite(time_s): raise ValueError('time must be finite')
        t=float(time_s); start=self.lead_in_s; end=start+self.active_s
        if t<start or t>=end+self.release_s:return 0.
        if self.kind=='sine':return self.amplitude_rad*math.sin(2*math.pi*self.frequency_hz*(t-start)) if t<end else 0.
        if self.kind=='ramp': value=self.amplitude_rad*min(1.,max(0.,(t-start)/self.active_s))
        else:value=self.amplitude_rad
        if t>=end:
            if self.release_s<=0:return 0.
            return value*max(0.,1-(t-end)/self.release_s)
        return value

    def breakpoints(self)->tuple[float,...]:
        return (0.,self.lead_in_s,self.lead_in_s+self.active_s,self.lead_in_s+self.active_s+self.release_s)
