"""Pure signed longitudinal policy for immutable hybrid speed plans.

This is an opt-in controller, not a maneuver qualification. Rollout authority and
physical signed-pedal response are sampled from immutable capabilities; it performs
no per-slab plant/tire/powertrain solve.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext, advance_signed_pedal

from simplified_vehicle_models.solvers.longitudinal_capability import (
    CapabilityQuery, PathKinematicContext, body_vx_dot_to_path_progress,
    path_progress_to_body_vx_dot, evaluate_pedal_response, pedal_for_acceleration,
)
from simplified_vehicle_models.solvers.hybrid_speed_plan import HybridSpeedPlan
from simplified_vehicle_models.solvers.hybrid_setup import HYBRID_EXECUTION_ALLOCATOR_ID

HYBRID_POLICY_VERSION = "hybrid-longitudinal-policy-v11-verified-midpoint-proposals"
HYBRID_MAX_EXTRA_FORECASTS = 8
HYBRID_BRACKET_RESOLUTION = 1e-8
HYBRID_MAX_TOTAL_FORECASTS = 3 + HYBRID_MAX_EXTRA_FORECASTS


def _coast_referenced_brake_increment(map_partial, map_coast, actual_partial, actual_coast):
    """Envelope brake increments around the validated future-map coast baseline.

    This bounds increments, not total net acceleration against a coast evaluated
    in another physical context. Positive braking-branch increments are unknown.
    """
    values = tuple(float(x) for x in (map_partial, map_coast, actual_partial, actual_coast))
    if not all(math.isfinite(x) for x in values):
        raise ValueError("nonfinite coast-referenced brake response")
    mp, mc, ap, ac = values
    map_increment = mp - mc
    actual_increment = ap - ac
    if not math.isfinite(map_increment) or not math.isfinite(actual_increment):
        raise ValueError("nonfinite coast-referenced brake increment")
    if map_increment > 0. or actual_increment > 0.:
        raise ValueError("positive negative-pedal increment is unsupported")
    result = mc + max(map_increment, actual_increment)
    if not math.isfinite(result):
        raise ValueError("nonfinite coast-referenced brake result")
    return result


def _future_brake_target(capability, target, map_failure):
    try:
        return pedal_for_acceleration(capability, target)
    except ValueError as exc:
        # A requested deceleration weaker than physical coast is represented
        # by neutral pedal, not an invented throttle bias. Beyond full brake,
        # use full pedal operationally; malformed maps still fail closed.
        if str(exc) == "brake target outside physical pedal-map authority":
            coast=float(capability.coast_net_progress_acceleration_mps2)
            full_brake=float(capability.brake_acceleration_grid[-1])
            if target >= coast:
                return 0.
            if target < full_brake:
                return -1.
        map_failure[0]=str(exc)
        return None

@dataclass(frozen=True, slots=True)
class HybridDecision:
    """Decision diagnostics. Constructor is keyword-only to prevent telemetry shifts."""
    target_progress_acceleration_mps2: float
    raw_acceleration_request_mps2: float
    body_vx_dot_target_mps2: float
    actual_pedal_force_n: float
    throttle: float
    brake_force_n: float
    phase: str
    status: str
    event_crossed: bool | None
    event_crossing_time_s: float | None
    event_crossing_speed_mps: float | None
    event_speed_cap_mps: float | None
    lead_time_s: float
    candidate_count: int
    rollout_samples: int
    shortfall_mps2: float
    authority_domain_exceeded: bool
    plan_identity: str
    provider_identity: str
    units: str = "progress/body acceleration:m/s^2; force:N; time:s"
    extra_probe_count: int = 0
    attempted_transitions: int = 0
    selected_mixing_coefficient: float | None = None
    selected_event_residual_mps: float | None = None
    search_termination_reason: str | None = None
    decision_query_count: int | None = None
    original_candidate_statuses: tuple | None = None
    discovery_termination_reason: str | None = None
    def __init__(self, *, target_progress_acceleration_mps2, raw_acceleration_request_mps2,
                 body_vx_dot_target_mps2, actual_pedal_force_n, throttle, brake_force_n,
                 phase, status, event_crossed, event_crossing_time_s, event_crossing_speed_mps, event_speed_cap_mps, lead_time_s,
                 candidate_count, rollout_samples, shortfall_mps2, authority_domain_exceeded,
                 plan_identity, provider_identity, units="progress/body acceleration:m/s^2; force:N; time:s",
                  extra_probe_count=0, attempted_transitions=0, selected_mixing_coefficient=None,
                  selected_event_residual_mps=None, search_termination_reason=None,
                  decision_query_count=None, original_candidate_statuses=None,
                   discovery_termination_reason=None):
        for k,v in locals().copy().items():
            if k not in ("self",): object.__setattr__(self,k,v)
    def to_dict(self):
        return {k:getattr(self,k) for k in self.__dataclass_fields__}

@dataclass(frozen=True, slots=True)
class HybridDriverCommand:
    throttle: float
    brake_force_n: float
    decision: HybridDecision
    first_pedal_target: float | None = None
    predicted_pedal_after_first_slab: float | None = None

@dataclass(frozen=True, slots=True)
class HybridPolicyConfig:
    speed_gain: float = 2.0
    throttle_acceleration_scale: float = 2.5
    maximum_brake_force_n: float = 1.0
    horizon_s: float = 3.0
    slabs: int = 60
    feedback_acceleration_limit_mps2: float = 12.0
    pedal_context: PedalPolicyContext | None = None
    governor_margin: float = .85
    brake_margin: float = .60
    max_extra_forecasts: int = HYBRID_MAX_EXTRA_FORECASTS
    bracket_resolution: float = HYBRID_BRACKET_RESOLUTION
    def __post_init__(self):
        vals=(self.speed_gain,self.throttle_acceleration_scale,self.maximum_brake_force_n,
              self.horizon_s,self.feedback_acceleration_limit_mps2)
        if not all(math.isfinite(float(x)) and x>0 for x in vals): raise ValueError("hybrid config values must be positive finite")
        if self.horizon_s>3 or isinstance(self.slabs,bool) or not isinstance(self.slabs,int) or not 1<=self.slabs<=60: raise ValueError("rollout budget exceeds 3 seconds / 60 slabs")
        if (isinstance(self.max_extra_forecasts,bool) or not isinstance(self.max_extra_forecasts,int) or
                not 0 <= self.max_extra_forecasts <= HYBRID_MAX_EXTRA_FORECASTS):
            raise ValueError("extra forecast budget must be an integer in [0, 8]")
        if not math.isfinite(self.bracket_resolution) or self.bracket_resolution != HYBRID_BRACKET_RESOLUTION:
            raise ValueError("bracket resolution is fixed at 1e-8 normalized first-target units")
        if self.pedal_context is not None and not isinstance(self.pedal_context,PedalPolicyContext): raise TypeError("pedal_context must be PedalPolicyContext")
        if self.pedal_context is not None and not math.isclose(self.maximum_brake_force_n,self.pedal_context.maximum_brake_force,rel_tol=0.,abs_tol=1e-9): raise ValueError("maximum brake force must match pedal context")
        if not 0.<self.brake_margin<=1. or not math.isfinite(self.governor_margin) or self.governor_margin<=0: raise ValueError("invalid brake margin/governor")

def _relay_search_eligible(continue_outcome, release_outcome, brake_outcome, chosen,
                           extra_budget, residuals_consistent):
    """Return true only for a known unsafe→safe original bracket below release priority."""
    return (continue_outcome[0] is False and release_outcome[0] is not True and
            brake_outcome[0] is True and chosen is brake_outcome and
            bool(extra_budget) and residuals_consistent)

def _bounded_verified_relay_search(forecast, continue_outcome, brake_outcome,
                                   event_cap, max_probes, bracket_resolution,
                                   endpoint_span, safe_lambda=1.):
    """Bounded midpoint bracket search; invalid probes never modify verified endpoints."""
    threshold=float(event_cap)+1e-8
    unsafe=float(continue_outcome[4])-threshold
    safe=float(brake_outcome[4])-threshold
    lo,hi=0.,float(safe_lambda); flo,fhi=unsafe,safe
    safe_record=brake_outcome; safe_lambda=float(safe_lambda); selected_residual=None; probes=0
    termination=None
    for _ in range(max_probes):
        if abs((hi-lo)*endpoint_span)<=bracket_resolution:
            termination='bracket_resolution'; break
        # Deterministic strict-interior bisection avoids residual-root sensitivity.
        # If adjacent floats leave no representable interior point, stop without
        # replacing the previously verified safe endpoint.
        proposal=lo+(hi-lo)*.5
        if not math.isfinite(proposal) or not lo<proposal<hi:
            termination='floating_point_exhaustion'; break
        probe=forecast(proposal); probes+=1
        if probe[0] not in (True,False) or probe[2] is None or probe[4] is None or not math.isfinite(probe[4]):
            continue
        residual=float(probe[4])-threshold
        if (probe[0] is True and residual>0.) or (probe[0] is False and residual<=0.):
            continue
        if probe[0] is True:
            hi=proposal; fhi=residual; safe_record=probe; safe_lambda=proposal; selected_residual=residual
        else:
            lo=proposal; flo=residual
        if abs((hi-lo)*endpoint_span)<=bracket_resolution:
            termination='bracket_resolution'; break
    if termination is None: termination='extra_forecast_limit'
    return safe_record,safe_lambda,selected_residual,probes,termination

class HybridLongitudinalPolicy:
    """Stateless path-progress policy with bounded candidate pedal schedules."""
    def __init__(self, plan: HybridSpeedPlan, capability_provider, brake_predictor,
                 config: HybridPolicyConfig, execution_allocator_identity=None):
        if not isinstance(plan,HybridSpeedPlan): raise TypeError("plan must be HybridSpeedPlan")
        if not getattr(capability_provider,"identity",None): raise TypeError("provider identity required")
        if plan.provider_identity != capability_provider.identity: raise ValueError("plan/provider identity mismatch")
        if config.pedal_context is None: raise ValueError("hybrid policy requires complete pedal dynamics context")
        if brake_predictor is None: raise ValueError("hybrid policy requires explicit current brake predictor")
        if getattr(getattr(getattr(brake_predictor,"vehicle",None),"controller",None),"actuator_calibration",None) is not None:
            raise ValueError("hybrid physical rollout does not support nonlinear actuator calibration")
        if hasattr(brake_predictor,"_linear_fast_path") and not brake_predictor._linear_fast_path:
            raise ValueError("hybrid pedal rollout requires validated linear brake force mapping")
        self.plan=plan; self.provider=capability_provider; self.predictor=brake_predictor; self.config=config
        self.provider_identity=capability_provider.identity
        self.execution_allocator_identity=execution_allocator_identity or HYBRID_EXECUTION_ALLOCATOR_ID

    def _cap(self, speed, curvature, soc, yaw=0.):
        if yaw and (self.execution_allocator_identity != self.provider.identity or self.provider.identity != HYBRID_EXECUTION_ALLOCATOR_ID): raise ValueError("unsupported yaw allocator execution path")
        c=self.provider.evaluate(CapabilityQuery(max(0.,speed),curvature,soc,yaw))
        if not c.in_domain: return c
        if c.allocator_identity != self.provider.identity: raise ValueError("allocator identity mismatch")
        return c

    def decide(self, *, observation, pedal_position, kinematics: PathKinematicContext,
               soc: float, steering_rate: float=0., brake_context=None):
        if not isinstance(kinematics,PathKinematicContext): raise TypeError("explicit PathKinematicContext required")
        if not math.isfinite(float(soc)) or not 0.<=float(soc)<=1.: raise ValueError("explicit finite SOC in [0,1] required")
        sample=self.plan.sample(observation.distance)
        B=1-kinematics.curvature_inv_m*kinematics.lateral_error_m
        c=math.cos(kinematics.heading_error_rad)
        progress=(kinematics.vx_mps*c-kinematics.vy_mps*math.sin(kinematics.heading_error_rad))/B if B > 0. else float("nan")
        if not kinematics.vy_dot_valid or not math.isfinite(progress) or B < .5 or abs(c) < 1.e-6 or progress < 0.:
            progress_valid=False
        else: progress_valid=True
        raw=max(-self.config.feedback_acceleration_limit_mps2,min(self.config.feedback_acceleration_limit_mps2,
            sample.acceleration_mps2+self.config.speed_gain*(sample.speed_mps-progress if progress_valid else 0.)))
        cap=self._cap(progress,kinematics.curvature_inv_m,soc)
        if not cap.in_domain:
            d=HybridDecision(target_progress_acceleration_mps2=0.,raw_acceleration_request_mps2=raw,
                body_vx_dot_target_mps2=0.,actual_pedal_force_n=0.,throttle=0.,brake_force_n=0.,phase="fallback",status="capability_domain_exit",event_crossed=None,event_crossing_time_s=None,event_crossing_speed_mps=None,event_speed_cap_mps=None,lead_time_s=0.,candidate_count=0,rollout_samples=0,shortfall_mps2=max(0.,-raw),authority_domain_exceeded=True,plan_identity=self.plan.identity,provider_identity=self.provider.identity)
            return HybridDriverCommand(0.,0.,d,first_pedal_target=0.,predicted_pedal_after_first_slab=None)
        active_bound=(sample.braking_region and progress>=sample.braking_speed_bound_mps-1e-6)
        constraint=max(0.,-sample.acceleration_mps2) if active_bound else 0.
        demand=max(-self.config.feedback_acceleration_limit_mps2,min(0.,raw))
        target=min(demand,-constraint) if constraint>0 else demand
        # Convert path target before any body-axis inverse. Positive demand uses
        # the retained throttle law; no zero-decel inverse is ever queried.
        try:
            if not progress_valid: raise ValueError("invalid path kinematic context")
            body=path_progress_to_body_vx_dot(target if target<0 else raw,kinematics)
        except ValueError: body=float('nan')
        brake=0.; throttle=0.; phase="coast"; status="nominal"; crossed=None; crossing_time=None; crossing_speed=None; event_cap=None; lead=0.; candidates=0; slabs=0; shortfall=0.; first_target=None; predicted_first_state=None
        rollout_diagnostics={'extra_probe_count':0,'attempted_transitions':0,'selected_lambda':None,'selected_residual':None,'termination':None,'discovery_termination':None,'original_candidate_statuses':None,'query_count':None,'selected_progress_acceleration':None}
        if self.config.pedal_context is not None and self.predictor is not None and getattr(getattr(self.predictor,"vehicle",None),"controller",None) is not None:
            if getattr(self.predictor.vehicle.controller,"actuator_calibration",None) is not None:
                status="unsupported_nonlinear_actuator_map"; target=0.; raw_for_command=0.
            else: raw_for_command=raw
        else: raw_for_command=raw
        force=0.
        governor=min(1.,max(0.,float(getattr(observation,'wheel_speed_margin',float('inf')))/self.config.pedal_context.governor_margin)) if self.config.pedal_context else 1.
        current_governor=governor
        terminal_reason=None
        if brake_context is None and self.config.pedal_context is not None:
            status="missing_current_inverse_context"; terminal_reason="missing_current_inverse_context"
        if not math.isfinite(body):
            status="unsupported_kinematic_context"; phase="fallback"; target=0.; body=0.; terminal_reason="unsupported_kinematic_context"
        elif raw_for_command>0 and constraint<=0:
            throttle=min(1.,raw_for_command/self.config.throttle_acceleration_scale); phase="drive"
        elif target<0 or constraint>0:
            physical=max(0.,-body)
            if self.predictor is None or brake_context is None:
                status="missing_current_inverse_context"; phase="fallback"
            elif physical>0:
                inverse=self.predictor.request_current(brake_context,physical)
                brake=min(self.config.maximum_brake_force_n,float(inverse.pedal_force_n))*current_governor; phase="brake" if brake>0 else "coast"
                shortfall=max(0.,physical-float(inverse.deceleration_mps2)*governor)
            force=brake
        # Always forecast a known future braking onset, including positive drive
        # now; select the chosen schedule's actual first target, not just telemetry.
        if math.isfinite(body) and status not in ("unsupported_kinematic_context","missing_current_inverse_context") and not sample.braking_region and sample.distance_to_braking_onset_m not in (None,0.) and sample.distance_to_future_event_m not in (None,0.):
            result=self._rollout(sample,observation,kinematics,soc,pedal_position,brake_context,brake,raw,_diagnostics=rollout_diagnostics)
            terminal_reason=rollout_diagnostics.get('termination')
            if len(result)!=10: raise RuntimeError("hybrid rollout returned malformed candidate outcome")
            crossed,crossing_time,crossing_speed,lead,candidates,slabs,status,selected,first_target,predicted_first_state=result
            event_cap=sample.future_event_speed_cap_mps
            throttle=max(0.,first_target); brake=max(0.,-first_target)*self.config.maximum_brake_force_n; force=brake
            if selected=="release_then_brake": phase="release"; target=0.; body=path_progress_to_body_vx_dot(0.,kinematics)
            elif selected=="brake_now":
                phase="brake"; target=-self._force_deceleration(brake_context,brake)
                body=path_progress_to_body_vx_dot(target,kinematics)
            elif selected=="continue":
                phase="drive" if throttle>0. else "coast"
                target=raw if throttle>0. else 0.; body=path_progress_to_body_vx_dot(target,kinematics)
            elif selected=="mixed":
                phase="drive" if first_target > 0. else "brake" if first_target < 0. else "coast"
                target=rollout_diagnostics.get('selected_progress_acceleration')
                if target is None: raise RuntimeError("verified mixed candidate missing selected progress acceleration")
                body=path_progress_to_body_vx_dot(target,kinematics)
            if crossed is False:
                status="all_candidates_overspeed_fallback_shortfall"; throttle=0.; phase="fallback"
                admissible=self._maximum_admissible_force(brake_context,governor)
                brake=admissible; force=brake
                shortfall=max(shortfall,max(0.,-body-self._force_deceleration(brake_context,brake)))
                target=-max(0.,self._force_deceleration(brake_context,brake)); body=path_progress_to_body_vx_dot(target,kinematics)
                first_target=-brake/self.config.maximum_brake_force_n
                predicted_first_state=advance_signed_pedal(float(pedal_position or 0.),first_target,self.config.horizon_s/self.config.slabs,self.config.pedal_context)
            elif crossed is None:
                status="event_unknown:"+status; phase="coast"; target=0.; body=path_progress_to_body_vx_dot(0.,kinematics)
        desired=target
        d=HybridDecision(target_progress_acceleration_mps2=float(desired),raw_acceleration_request_mps2=float(raw),body_vx_dot_target_mps2=float(body),actual_pedal_force_n=float(force),throttle=float(throttle),brake_force_n=float(brake),phase=phase,status=status,event_crossed=crossed,event_crossing_time_s=crossing_time,event_crossing_speed_mps=crossing_speed,event_speed_cap_mps=event_cap,lead_time_s=lead,candidate_count=candidates,rollout_samples=slabs,shortfall_mps2=float(shortfall),authority_domain_exceeded=False,plan_identity=self.plan.identity,provider_identity=self.provider.identity,
            extra_probe_count=rollout_diagnostics.get('extra_probe_count',0),attempted_transitions=rollout_diagnostics.get('attempted_transitions',slabs),
            selected_mixing_coefficient=rollout_diagnostics.get('selected_lambda'),selected_event_residual_mps=rollout_diagnostics.get('selected_residual'),
            search_termination_reason=terminal_reason,decision_query_count=rollout_diagnostics.get('query_count') if terminal_reason is not None else None,
             original_candidate_statuses=rollout_diagnostics.get('original_candidate_statuses'),
             discovery_termination_reason=rollout_diagnostics.get('discovery_termination'),
            )
        return HybridDriverCommand(throttle,brake,d,first_pedal_target=first_target,predicted_pedal_after_first_slab=predicted_first_state)

    def _rollout(self,sample,observation,kin,soc,pedal,brake_context,brake_force,positive_accel,_diagnostics=None):
        """Compatibility tuple wrapper for the pure schedule forecaster."""
        result,diagnostics=self._rollout_with_diagnostics(sample,observation,kin,soc,pedal,brake_context,brake_force,positive_accel)
        if _diagnostics is not None: _diagnostics.update(diagnostics)
        return result

    def _rollout_with_diagnostics(self,sample,observation,kin,soc,pedal,brake_context,brake_force,positive_accel):
        """Forecast original schedules and, only on the verified relay branch, mixes."""
        dt=self.config.horizon_s/self.config.slabs; start=float(observation.distance)
        future_distance=sample.distance_to_future_event_m
        if future_distance is None:
            return (None,None,None,0.,0,0,"no_future_event",None,0.,0.),{'termination':'no_future_event','query_count':None,'selected_progress_acceleration':None}
        if not math.isfinite(float(future_distance)) or future_distance <= 0.:
            return (None,None,None,0.,0,0,"unsupported_future_event_geometry",None,0.,0.),{'termination':'unsupported_future_event_geometry','query_count':None,'selected_progress_acceleration':None}
        try:
            # Rollout state and capability accelerations are Frenet path-progress
            # quantities. Invert the frozen quasi-steady context for a body-vx
            # brake query only; never substitute body vx for progress speed.
            cos_e=math.cos(kin.heading_error_rad)
            sin_e=math.sin(kin.heading_error_rad)
            B=1.-kin.curvature_inv_m*kin.lateral_error_m
            if (not kin.vy_dot_valid or B < .5 or abs(cos_e) < 1.e-6):
                raise ValueError("kinematic context outside Frenet conversion domain")
            progress=(kin.vx_mps*cos_e-kin.vy_mps*sin_e)/B
            if not math.isfinite(progress) or progress < 0.:
                raise ValueError("negative/nonfinite path-progress speed")
            def brake_query_speed(progress_speed):
                body_vx=(progress_speed*B+kin.vy_mps*sin_e)/cos_e
                if not math.isfinite(body_vx) or body_vx < 0.:
                    raise ValueError("path-progress speed has no nonnegative body-vx brake context")
                return body_vx
        except (ValueError, ZeroDivisionError):
            return (None,None,None,0.,0,0,"unsupported_kinematic_context",None,0.,0.),{'termination':'unsupported_kinematic_context','query_count':None,'selected_progress_acceleration':None}
        query_start=getattr(self.provider,'query_count',None); query_end=query_start
        query_unavailable=(query_start is None)
        event=start+future_distance
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext, BrakeAuthorityPredictor
        physical_predictor=isinstance(self.predictor,BrakeAuthorityPredictor)
        if physical_predictor and not isinstance(brake_context,BrakeAuthorityContext):
            return (None,None,None,0.,0,0,"unsupported_kinematic_context",None,0.,0.),{'termination':'unsupported_kinematic_context','query_count':None,'selected_progress_acceleration':None}
        if physical_predictor:
            try: self.predictor._validate_context(brake_context)
            except (ValueError,TypeError): return (None,None,None,0.,0,0,"unsupported_kinematic_context",None,0.,0.),{'termination':'unsupported_kinematic_context','query_count':None,'selected_progress_acceleration':None}
        schedules=("continue","release_then_brake","brake_now"); outcomes=[]; total=0
        original_candidates=0; extra_probes=0; selected_lambda=None; selected_residual=None; termination=None
        def empty_diagnostics(reason):
            observed=getattr(self.provider,'query_count',None)
            count=max(0,observed-query_start) if query_start is not None and observed is not None and not query_unavailable else None
            return {'extra_probe_count':0,'attempted_transitions':0,'selected_lambda':None,'selected_residual':None,'termination':reason,'query_count':count,'selected_progress_acceleration':None}
        pc=self.config.pedal_context
        if pc is None: return (None,None,None,0.,0,0,"missing_pedal_context",None,0.,0.),empty_diagnostics('missing_pedal_context')
        nominal=-min(1.,brake_force/pc.maximum_brake_force) if brake_force>0 else 0.
        drive_target=min(1.,max(0.,positive_accel/self.config.throttle_acceleration_scale))
        map_failure=[None]
        def target_from_sample(ps):
            cap=self._cap(max(0.,ps.speed_mps),ps.curvature_inv_m,soc)
            if not cap.in_domain: return None
            if ps.acceleration_mps2 < 0.:
                return _future_brake_target(cap,ps.acceleration_mps2,map_failure)
            return min(1.,max(0.,ps.acceleration_mps2/self.config.throttle_acceleration_scale))
        onset_distance=sample.distance_to_braking_onset_m
        onset=start+(onset_distance if onset_distance is not None else future_distance)
        future_brake=target_from_sample(self.plan.sample(onset))
        if future_brake is None:
            status="unsupported_pedal_response_map" if map_failure[0] else "future_capability_domain_exit"
            observed=getattr(self.provider,'query_count',None); count=max(0,observed-query_start) if query_start is not None and observed is not None and not query_unavailable else None
            return (None,None,None,0.,0,0,status,None,0.,0.),{'extra_probe_count':0,'attempted_transitions':0,'selected_lambda':None,'selected_residual':None,'termination':'unknown_onset_'+status,'query_count':count,'selected_progress_acceleration':None}
        governor=min(1.,max(0.,float(getattr(observation,'wheel_speed_margin',float('inf')))/pc.governor_margin))
        current_governor=governor
        # Exact future actuator availability is unknown; hold current governor
        # factor fixed over this pure decision horizon and apply it once to each
        # signed brake target before integrating the exact pedal ODE.
        first_targets={}; first_predicted_states={}
        def forecast(schedule, lam=None, *, record=False):
            nonlocal total, query_end, query_unavailable
            s=start; v=progress; p=float(pedal if pedal is not None else 0.)
            hit=None; reason="event_beyond_horizon"; crossing=None; vc=None; crossing_acceleration=None
            first=None; first_state=None; first_progress_accel=None
            for i in range(1,self.config.slabs+1):
                total+=1
                before=getattr(self.provider,'query_count',None)
                ps=self.plan.sample(s)
                after_plan=getattr(self.provider,'query_count',None)
                if before is None or after_plan is None: query_unavailable=True
                elif after_plan-before not in (0,1): query_unavailable=True
                if schedule in ("continue","mixed"):
                    endpoint=target_from_sample(ps)
                    if endpoint is not None and endpoint>=0.:
                        endpoint=min(1.,max(0.,(ps.acceleration_mps2+2.*(ps.speed_mps-v))/self.config.throttle_acceleration_scale))
                else: endpoint=None
                if schedule=="continue": target=endpoint
                elif schedule=="release_then_brake": target=0. if s<onset else future_brake
                elif schedule=="brake_now": target=future_brake
                else:
                    if endpoint is None: target=None
                    else: target=(1.-lam)*endpoint+lam*future_brake
                if target is None: reason="future_capability_domain_exit"; break
                if target<0. and nominal<0.: target=max(target,nominal)
                target*=current_governor if target<0. else 1.
                if i==1: first=target; first_state=advance_signed_pedal(p,target,dt,pc)
                pnext=advance_signed_pedal(p,target,dt,pc)
                cap=self._cap(v,ps.curvature_inv_m,soc)
                after=getattr(self.provider,'query_count',None)
                if before is None or after is None: query_unavailable=True
                elif after-after_plan not in (0,1): query_unavailable=True
                if not cap.in_domain: reason="capability_domain_exit"; break
                try: a=evaluate_pedal_response(cap,pnext)
                except ValueError: reason="unsupported_pedal_response_map"; break
                if pnext<0. and physical_predictor:
                    try:
                        body_speed=brake_query_speed(v)
                        from dataclasses import replace
                        query=replace(brake_context,speed_mps=body_speed,v_y_mps=kin.vy_mps,yaw_rate_rps=kin.yaw_rate_rps,curvature_inv_m=kin.curvature_inv_m)
                        prediction=self.predictor.predict(query,abs(pnext)*pc.maximum_brake_force)
                        coast_prediction=self.predictor.predict(query,0.)
                        if not prediction.in_domain or not coast_prediction.in_domain: raise ValueError("body-speed brake context outside predictor domain")
                        actual_partial=body_vx_dot_to_path_progress(-float(prediction.deceleration_mps2),kin)
                        actual_coast=body_vx_dot_to_path_progress(-float(coast_prediction.deceleration_mps2),kin)
                        map_partial=float(evaluate_pedal_response(cap,pnext))
                        map_coast=float(evaluate_pedal_response(cap,0.))
                        a=_coast_referenced_brake_increment(map_partial,map_coast,actual_partial,actual_coast)
                    except (ValueError,AttributeError,TypeError): reason="unsupported_kinematic_context"; break
                    if i==1: first_progress_accel=a
                if i==1 and first_progress_accel is None: first_progress_accel=a
                vn=v+a*dt
                if vn<0.: reason="unsupported_future_rest_crossing"; break
                sn=s+.5*(v+vn)*dt
                if sn<=s: reason="unsupported_future_rest_crossing"; break
                if sn>=event:
                    frac=min(1.,max(0.,(event-s)/(sn-s))); crossing=(i-1)*dt+frac*dt; vc=v+frac*(vn-v)
                    hit=vc<=float(sample.future_event_speed_cap_mps)+1e-8; crossing_acceleration=a; reason="event_crossed"; break
                s,v,p=sn,vn,pnext
            query_end=getattr(self.provider,'query_count',query_end)
            if record and first is not None:
                first_targets[schedule]=first; first_predicted_states[schedule]=first_state
            return (hit,reason,crossing,schedule,vc,first,first_state,crossing_acceleration,first_progress_accel)
        for schedule in schedules:
            r=forecast(schedule,record=True); original_candidates+=1
            outcomes.append(r)
        valid=[x for x in outcomes if x[0] is True]
        chosen=next((x for x in valid if x[3]=="continue"),None)
        if chosen is None:
            chosen=next((x for x in valid if x[3]=="release_then_brake"),None)
        if chosen is None:
            chosen=next((x for x in valid if x[3]=="brake_now"),None)
        # Release priority applies only when the release candidate is actually verified feasible.
        # An unknown release is not a search endpoint; relay eligibility is established by the
        # known continue/brake endpoints and the original selected brake_now outcome.
        threshold=float(sample.future_event_speed_cap_mps)+1e-8
        continue_residual=float(outcomes[0][4])-threshold if outcomes[0][4] is not None else float('nan')
        brake_residual=float(outcomes[2][4])-threshold if outcomes[2][4] is not None else float('nan')
        residuals_consistent=(math.isfinite(continue_residual) and math.isfinite(brake_residual) and continue_residual>0. and brake_residual<=0.)
        if _relay_search_eligible(outcomes[0],outcomes[1],outcomes[2],chosen,self.config.max_extra_forecasts,residuals_consistent):
            # Bracket signs follow the actual classification, not a rounded
            # crossing-speed diagnostic (notably at the row-4991 ulp edge).
            endpoint_span=abs(first_targets['brake_now']-first_targets['continue'])
            chosen,selected_lambda,selected_residual,extra_probes,termination=_bounded_verified_relay_search(
                lambda lam: forecast('mixed',lam,record=True),outcomes[0],outcomes[2],
                sample.future_event_speed_cap_mps,self.config.max_extra_forecasts,
                self.config.bracket_resolution,endpoint_span)
            if selected_lambda != 1. or selected_residual is not None:
                first_targets['mixed_verified']=chosen[5]
                first_predicted_states['mixed_verified']=chosen[6]
        discovery_termination=None
        original_candidate_statuses=tuple((x[3],x[0],x[1]) for x in outcomes)
        if (chosen is None and outcomes[0][0] is False and outcomes[1][0] is None and
                outcomes[2][0] is None and self.config.max_extra_forecasts>0):
            discovery_termination='extra_forecast_limit'
            safe=None; safe_lambda=None; used=0
            while used < self.config.max_extra_forecasts:
                proposal=0.5 ** (used+1)
                candidate=forecast('mixed',proposal,record=True)
                used+=1; extra_probes+=1
                if (candidate[0] is True and candidate[2] is not None and candidate[4] is not None and
                        math.isfinite(candidate[2]) and math.isfinite(candidate[4]) and
                        candidate[4] <= threshold):
                    safe=candidate; safe_lambda=proposal
                    discovery_termination='verified_candidate_found'
                    break
            remaining=self.config.max_extra_forecasts-used
            if safe is not None:
                chosen=safe; selected_lambda=safe_lambda
                if remaining:
                    chosen,selected_lambda,selected_residual,refine_probes,termination=_bounded_verified_relay_search(
                        lambda lam: forecast('mixed',lam,record=True),outcomes[0],safe,
                        sample.future_event_speed_cap_mps,remaining,self.config.bracket_resolution,
                        abs(first_targets['brake_now']-first_targets['continue']),safe_lambda=safe_lambda)
                    extra_probes+=refine_probes
                else:
                    termination='extra_forecast_limit'
                first_targets['mixed_verified']=chosen[5]
                first_predicted_states['mixed_verified']=chosen[6]
            else:
                discovery_termination='discovery_exhausted_no_verified_candidate'
        if chosen is not None:
            query_count=max(0,(query_end-query_start)) if query_start is not None and query_end is not None and not query_unavailable else None
            lead=max(0.,(onset-start)/max(progress,1e-9)-float(chosen[2] or 0.))
            target=first_targets.get(chosen[3],chosen[5] if len(chosen)>5 else None)
            predicted=first_predicted_states.get(chosen[3],chosen[6] if len(chosen)>6 else None)
            if selected_lambda is not None:
                target=chosen[5]; predicted=chosen[6]
            result=(True,chosen[2],chosen[4],lead,original_candidates,total,"event_crossed:"+("mixed_verified" if selected_lambda is not None else chosen[3]),chosen[3],target,predicted)
            return result,{'extra_probe_count':extra_probes,'attempted_transitions':total,'selected_lambda':selected_lambda,'selected_residual':selected_residual,'termination':termination or 'original_candidate','query_count':query_count,'selected_progress_acceleration':chosen[8],'original_candidate_statuses':original_candidate_statuses,'discovery_termination':discovery_termination}
        if chosen is None and any(x[0] is None for x in outcomes):
            query_count=max(0,(query_end-query_start)) if query_start is not None and query_end is not None and not query_unavailable else None
            unknown_reason=next(x[1] for x in outcomes if x[0] is None)
            result=(None,None,None,0.,original_candidates,total,unknown_reason,None,0.,0.)
            return result,{'extra_probe_count':extra_probes,'attempted_transitions':total,'selected_lambda':None,'selected_residual':None,'termination':'unknown_original_endpoint:'+unknown_reason,'query_count':query_count,'selected_progress_acceleration':None,'original_candidate_statuses':original_candidate_statuses,'discovery_termination':discovery_termination}
        query_count=max(0,(query_end-query_start)) if query_start is not None and query_end is not None and not query_unavailable else None
        target=first_targets.get('brake_now',future_brake*current_governor); predicted=first_predicted_states.get('brake_now')
        result=(False,None,None,self.config.horizon_s,original_candidates,total,"all_candidates_overspeed","brake_now",target,predicted)
        return result,{'extra_probe_count':extra_probes,'attempted_transitions':total,'selected_lambda':None,'selected_residual':None,'termination':'all_known_candidates_infeasible','query_count':query_count,'selected_progress_acceleration':None}

    def _force_deceleration(self, context, force):
        if context is None or self.predictor is None: return 0.
        pred=self.predictor.predict(context,force)
        return float(pred.deceleration_mps2) if pred.in_domain else 0.

    def _maximum_admissible_force(self, context, governor):
        """Return governed common-pedal prefix from the current physical inverse."""
        if context is None or self.predictor is None: return 0.
        admissible=self.predictor.request_current(context,1.e6)
        physical=min(self.config.maximum_brake_force_n,float(admissible.pedal_force_n))
        return physical*min(1.,max(0.,float(governor)))
