"""Pure actual-speed and pedal-aware braking-envelope longitudinal policy."""
from __future__ import annotations
from dataclasses import dataclass
import math
from numbers import Real


def _finite(value, name, *, positive=True):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not math.isfinite(value) or (value <= 0 if positive else value < 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return value

@dataclass(frozen=True, slots=True)
class EnvelopeDriverSettings:
    speed_gain: float = 2.
    throttle_acceleration_scale: float = 2.5
    brake_acceleration_scale: float = 6.
    response_allowance_s: float = .06
    disabled_transition_time_s: float = .4
    positive_acceleration_reserve_mps2: float = 4.
    def __post_init__(self):
        for name in ('speed_gain','throttle_acceleration_scale','brake_acceleration_scale','disabled_transition_time_s'):
            _finite(getattr(self,name),name)
        _finite(self.response_allowance_s,'response_allowance_s',positive=False)
        _finite(self.positive_acceleration_reserve_mps2,'positive_acceleration_reserve_mps2',positive=False)

@dataclass(frozen=True, slots=True)
class PedalPolicyContext:
    throttle_release_rate: float
    brake_apply_rate: float
    pedal_time_constant: float
    maximum_brake_force: float
    governor_margin: float
    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _finite(getattr(self,name),name)

@dataclass(frozen=True, slots=True)
class EnvelopeDecision:
    distance: float
    actual_speed: float
    pedal_position: float | None
    current_corner_cap: float
    preview_station: float
    preview_entry_speed: float
    reference_speed: float
    squared_speed: float
    squared_speed_slope: float
    envelope_deceleration: float
    binding_index: int
    binding_station: float
    binding_distance: float
    speed_domain_max: float
    transition_time_s: float
    reserve_distance_m: float
    speed_squared_slack: float
    acceleration_request: float
    requested_throttle: float
    ungoverned_brake_force: float
    governor_factor: float
    governed_brake_force: float
    authority_domain_exceeded: bool
    governor_shortfall: bool
    def to_dict(self):
        return {name:getattr(self,name) for name in self.__dataclass_fields__}

@dataclass(frozen=True, slots=True)
class EnvelopePolicy:
    envelope: object
    settings: EnvelopeDriverSettings = EnvelopeDriverSettings()
    def __post_init__(self):
        if not callable(getattr(self.envelope,'sample',None)):
            raise TypeError('envelope must provide sample(station)')
        if not isinstance(self.settings,EnvelopeDriverSettings):
            raise TypeError('settings must be EnvelopeDriverSettings')
    @staticmethod
    def _derived(value, name):
        value=float(value)
        if not math.isfinite(value):
            raise ValueError(f"nonfinite derived {name} in envelope policy")
        return value

    def decide(self, observation, pedal_position=None, context=None):
        for name in ('distance','lateral_error','heading_error','yaw_rate','steering','lateral_speed'):
            _finite(getattr(observation,name),f'observation.{name}',positive=False)
        v=_finite(observation.speed,'observation.speed',positive=False)
        margin=getattr(observation,'wheel_speed_margin',float('inf'))
        if isinstance(margin,bool) or not isinstance(margin,Real) or math.isnan(float(margin)):
            raise ValueError('observation.wheel_speed_margin must be numeric and not NaN')
        if margin < 0:
            raise ValueError('observation.wheel_speed_margin must be nonnegative')
        s=float(observation.distance)
        if pedal_position is not None:
            if isinstance(pedal_position,bool) or not isinstance(pedal_position,Real): raise TypeError('pedal_position must be real')
            p=float(pedal_position)
            if not math.isfinite(p) or not -1.<=p<=1.: raise ValueError('pedal_position must be finite and in [-1, 1]')
        else: p=None
        if context is not None and not isinstance(context,PedalPolicyContext): raise TypeError('context must be PedalPolicyContext or None')
        if context is None:
            T=self.settings.disabled_transition_time_s
            maxbrake=1.
            governor_margin=.85
        else:
            p_for_time=1. if p is None else p
            T=max(p_for_time,0.)/context.throttle_release_rate+(1.-max(-p_for_time,0.))/context.brake_apply_rate+self.settings.response_allowance_s+3.*context.pedal_time_constant
            maxbrake=context.maximum_brake_force; governor_margin=context.governor_margin
        T=self._derived(T,'transition time')
        reserve=self._derived(v*T+.5*self.settings.positive_acceleration_reserve_mps2*T*T,'reserve distance')
        preview_station=self._derived(s+reserve,'preview station')
        current=self.envelope.sample(s); preview=self.envelope.sample(preview_station)
        qpreview=self._derived(preview.squared_speed,'preview squared speed')
        qcorner=self._derived(current.corner_speed**2,'local corner cap squared')
        qglobal=self._derived(self.envelope.maximum_speed**2,'global cap squared')
        qref=min(qpreview,qcorner,qglobal)
        binds_local=qcorner<=qpreview and qcorner<=qglobal
        binds_global=qglobal<=qpreview and qglobal<=qcorner
        slope=0. if binds_local or binds_global else preview.squared_speed_slope
        reference=math.sqrt(max(0.,qref))
        reference=self._derived(reference,'reference speed')
        slope=self._derived(slope,'selected squared-speed slope')
        raw=self._derived(.5*slope+self.settings.speed_gain*(reference-v),'acceleration request')
        throttle=0. if v>=self.envelope.maximum_speed or raw<=0 else min(1.,raw/self.settings.throttle_acceleration_scale)
        brake=self._derived(min(1.,max(0.,-raw/self.settings.brake_acceleration_scale))*maxbrake,'ungoverned brake force')
        margin=getattr(observation,'wheel_speed_margin',float('inf'))
        governor=min(1.,max(0.,margin/governor_margin))
        governed=self._derived(brake*governor,'governed brake force')
        slack=self._derived(qref-v*v,'squared-speed slack')
        diagnostics=(s,v,p,current.corner_speed,preview_station,preview.entry_speed,reference,qref,slope,
                     preview.deceleration,preview.binding_index,preview.binding_station,preview.binding_distance,
                     preview.speed_domain_max,T,reserve,slack,raw,throttle,brake,governor,governed)
        if any(isinstance(value,float) and not math.isfinite(value) for value in diagnostics):
            raise ValueError('nonfinite envelope policy diagnostic')
        return EnvelopeDecision(s,v,p,current.corner_speed,preview_station,preview.entry_speed,reference,qref,slope,preview.deceleration,preview.binding_index,preview.binding_station,preview.binding_distance,preview.speed_domain_max,T,reserve,slack,raw,throttle,brake,governor,governed,(v>current.speed_domain_max or v>preview.speed_domain_max),brake>governed+1e-12)
