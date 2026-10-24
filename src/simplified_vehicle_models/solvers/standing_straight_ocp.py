"""Actual fixed-straight, nonnegative-acceleration standing-start OCP.

This restricted direct transcription does not optimize a curved line or certify
continuous feasibility between its finite live and held-out limit samples.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import numbers
import time
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class StandingStraightSettings:
    intervals: int = 12
    maximum_iterations: int = 120
    maximum_wall_seconds: float = 30.0
    maximum_limit_queries: int = 20000
    tolerance: float = 1e-7
    minimum_interval_seconds: float = 1e-5
    maximum_interval_seconds: float = 10.0
    heldout_fractions: int = 17


@dataclass(frozen=True)
class StandingStraightResult:
    time: np.ndarray
    distance: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    total_time: float
    fixed_straight_only: bool = True
    joint_line_optimized: bool = False
    accepted: bool = False
    event_qualified: bool = False
    U3_qualified: bool = False
    numerically_qualified: bool = False

    def __post_init__(self):
        for name in ("time", "distance", "speed", "acceleration"):
            arr = np.array(getattr(self, name), dtype=float, copy=True)
            arr.setflags(write=False)
            object.__setattr__(self, name, arr)


def _jsonsafe(x):
    if isinstance(x, (np.integer,)): return int(x)
    if isinstance(x, (np.floating,)): x = float(x)
    if isinstance(x, float): return x if math.isfinite(x) else None
    if isinstance(x, dict): return {str(k): _jsonsafe(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)): return [_jsonsafe(v) for v in x]
    if isinstance(x, np.ndarray): return _jsonsafe(x.tolist())
    return x


def solve_standing_straight(vehicle, distance, *, settings=None, acceleration_limit=None):
    started = time.monotonic()
    report = {"accepted": False, "optimizer_converged": False, "sampled_feasible": False,
              "event_qualified": False, "U3_qualified": False, "numerically_qualified": False,
              "physical_feasibility": False, "geometry": "fixed straight, nonnegative longitudinal control, rest-to-free-finish",
              "failure": None, "limit_queries": 0, "cache_hits": 0, "model_identity_known": False}
    def fail(reason):
        report["failure"] = str(reason); report["wall_seconds"] = time.monotonic()-started
        return None, _jsonsafe(report)
    if settings is None: settings = StandingStraightSettings()
    if type(settings) is not StandingStraightSettings: return fail("invalid_settings_type")
    for name in ("intervals", "maximum_iterations", "maximum_limit_queries", "heldout_fractions"):
        v=getattr(settings,name)
        minimum=2 if name in ("intervals","heldout_fractions") else 1
        if isinstance(v,bool) or not isinstance(v,numbers.Integral) or v<minimum: return fail("invalid_settings_"+name)
    for name in ("maximum_wall_seconds","tolerance","minimum_interval_seconds","maximum_interval_seconds"):
        v=getattr(settings,name)
        if isinstance(v,bool) or not isinstance(v,numbers.Real) or not math.isfinite(v) or v<=0: return fail("invalid_settings_"+name)
    if settings.minimum_interval_seconds>settings.maximum_interval_seconds: return fail("invalid_interval_bounds")
    if isinstance(distance,bool) or not isinstance(distance,numbers.Real) or not math.isfinite(distance) or distance<=0: return fail("invalid_distance")
    distance=float(distance); n=int(settings.intervals)
    try:
        if acceleration_limit is None:
            from simplified_vehicle_models.formulations.point_mass import maximum_acceleration_limit
            callback=maximum_acceleration_limit
        else: callback=acceleration_limit
        if not callable(callback): return fail("invalid_callback")
        cache={}; calls=0
        def cap(v):
            nonlocal calls
            key=(float(v),0.0)
            if key in cache: report["cache_hits"]+=1; return cache[key]
            if time.monotonic()-started>=settings.maximum_wall_seconds: raise TimeoutError("wall budget")
            if calls>=settings.maximum_limit_queries: raise RuntimeError("limit query budget")
            calls+=1; report["limit_queries"]=calls
            value=callback(vehicle,float(v),0.0)
            if time.monotonic()-started>=settings.maximum_wall_seconds: raise TimeoutError("wall budget")
            if isinstance(value,bool) or not isinstance(value,numbers.Real) or not math.isfinite(value) or value<0: raise ValueError("invalid acceleration limit")
            cache[key]=float(value); return float(value)
        a0=cap(0.0)
        if a0<=0: return fail("zero_rest_limit")
        T=math.sqrt(2*distance/(.5*a0)); dt=T/n
        if not settings.minimum_interval_seconds<=dt<=settings.maximum_interval_seconds: return fail("seed_out_of_bounds")
        from scipy.optimize import minimize
        def rollout(x):
            dts=x[:n]; acc=x[n:]; s=[0.0]; v=[0.0]; t=[0.0]
            for h,a in zip(dts,acc):
                vn=v[-1]+a*h; sn=s[-1]+v[-1]*h+.5*a*h*h
                if not all(map(math.isfinite,(vn,sn))): raise ValueError("nonfinite rollout")
                v.append(vn); s.append(sn); t.append(t[-1]+h)
            return np.array(t),np.array(s),np.array(v)
        def eq(x): return (rollout(x)[1][-1]-distance)/max(1.0,distance)
        def ineq(x, fractions=(0.0,.5,1.0)):
            _,_,vs=rollout(x); acc=x[n:]; out=[]
            for i in range(n):
                for f in fractions: out.append(cap(vs[i]+acc[i]*x[i]*f)-acc[i])
            return np.asarray(out)
        x0=np.r_[np.full(n,dt),np.full(n,.5*a0)]
        bounds=[(settings.minimum_interval_seconds,settings.maximum_interval_seconds)]*n+[(0,None)]*n
        opt=minimize(lambda x: float(np.sum(x[:n])),x0,method="SLSQP",bounds=bounds,
                     constraints=({"type":"eq","fun":eq},{"type":"ineq","fun":ineq}),
                     options={"maxiter":int(settings.maximum_iterations),"ftol":1e-12})
        report["optimizer_converged"]=bool(opt.success and opt.status==0)
        report["optimizer_status"]=int(opt.status); report["optimizer_message"]=str(opt.message)
        if not report["optimizer_converged"]: return fail("optimizer_not_converged")
        x=np.asarray(opt.x,dtype=float)
        if x.shape!=(2*n,) or not np.all(np.isfinite(x)): return fail("invalid_optimizer_iterate")
        t,s,v=rollout(x); acc=x[n:]
        sampled=np.asarray(ineq(x)); held=ineq(x,tuple(np.linspace(0,1,int(settings.heldout_fractions))))
        residual=max(abs(float(s[-1]-distance)),abs(float(v[0])),abs(float(s[0])),
                     max(0.0,-float(np.min(sampled))),max(0.0,-float(np.min(held))))
        report.update({"sampled_feasible":bool(np.min(sampled)>=-settings.tolerance),
                       "heldout_feasible":bool(np.min(held)>=-settings.tolerance),
                       "maximum_physical_residual_m":float(residual),"heldout_samples":int(len(held)),
                       "sampled_minimum_slack":float(np.min(sampled)),"heldout_minimum_slack":float(np.min(held))})
        if time.monotonic()-started>=settings.maximum_wall_seconds: return fail("wall_budget_after_optimize")
        if not report["sampled_feasible"] or not report["heldout_feasible"] or residual>settings.tolerance: return fail("physical_or_heldout_check_failed")
        # Deliberately never promote generic/event claims for this restricted model.
        result=StandingStraightResult(t,s,v,acc,float(np.sum(x[:n])))
        report["failure"]="restricted_problem_not_event_qualified"
        report["wall_seconds"]=time.monotonic()-started
        return result,_jsonsafe(report)
    except Exception as exc:
        text=str(exc)
        reason="budget_exhausted" if isinstance(exc,(TimeoutError,)) or "budget" in text else "callback_or_solver_error"
        return fail(reason)
