"""Qt-free sampled PID controller with filtered measurement derivative."""
from dataclasses import dataclass
import math

@dataclass(frozen=True, slots=True)
class PidConfig:
    kp: float
    ki: float
    kd: float
    derivative_tau_s: float
    lower: float
    upper: float

    def __post_init__(self):
        vals=(self.kp,self.ki,self.kd,self.derivative_tau_s,self.lower,self.upper)
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in vals):
            raise ValueError('PID configuration values must be finite numbers')
        if self.derivative_tau_s<0 or self.lower>self.upper: raise ValueError('invalid PID filter or output bounds')

class SampledPid:
    def __init__(self, config: PidConfig):
        if not isinstance(config,PidConfig): raise TypeError('config must be PidConfig')
        self.config=config; self.reset()
    def reset(self):
        self._integral=0.; self._previous_measurement=None; self._derivative=0.
    def update(self,target:float,measured:float,dt_s:float)->float:
        vals=(target,measured,dt_s)
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in vals) or dt_s<=0:
            raise ValueError('target, measurement, and positive dt must be finite numbers')
        c=self.config; error=float(target)-float(measured)
        if self._previous_measurement is not None:
            raw=(float(measured)-self._previous_measurement)/dt_s
            alpha=dt_s/(c.derivative_tau_s+dt_s) if c.derivative_tau_s else 1.
            self._derivative += alpha*(raw-self._derivative)
        self._previous_measurement=float(measured)
        base=c.kp*error-c.kd*self._derivative
        candidate=self._integral+c.ki*error*dt_s
        raw=base+candidate; clipped=min(c.upper,max(c.lower,raw))
        if raw==clipped or (raw>c.upper and c.ki*error<0) or (raw<c.lower and c.ki*error>0): self._integral=candidate
        return float(min(c.upper,max(c.lower,base+self._integral)))
