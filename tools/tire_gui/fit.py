"""Two-stage MATLAB-compatible steady tire fitting."""
from __future__ import annotations

from typing import Callable

import numpy as np
from scipy.optimize import minimize
from scipy.special import expit, logit

from .contracts import FitResult, FitCancelled, ModelSnapshot, SteadySelection
from .model import predict_steady, with_parameters


FORCE_NAMES = ('k_x_2','k_x_1','k_x_0','k_y_2','k_y_1','k_y_0','e_1',
               'mu_0_x','mu_s_x','h_x','v_m_x','mu_0_y','mu_s_y','h_y','v_m_y')
TRAIL_NAMES = ('d_x_0_1','d_x_0_0','d_e_1','d_e_0','d_1_1','d_1_0','d_2_1','d_2_0')
FORCE_BOUNDS = {
    'k_x_2':(0,20000),'k_x_1':(-20000,20000),'k_x_0':(10000,120000),
    'k_y_2':(0,20000),'k_y_1':(-20000,20000),'k_y_0':(10000,120000), 'e_1':(-.5,4),
    **{f'{p}_{a}': b for a in ('x','y') for p,b in [('mu_0',(.5,4)),('mu_s',(.2,3)),('h',(.02,4)),('v_m',(.02,3))]},
}
TRAIL_BOUNDS = {'d_x_0_1':(0,.3),'d_x_0_0':(0,.3),'d_e_1':(-.1,.1),'d_e_0':(-.1,.1),
                **{f'{n}_{d}':(0,8) for n in ('d_1','d_2') for d in (1,0)}}


def force_cost(residuals: tuple[np.ndarray, ...]) -> float:
    if len(residuals) != 4 or any(np.asarray(r).size == 0 for r in residuals):
        raise ValueError('four nonempty residual groups required')
    if any(not np.all(np.isfinite(r)) for r in residuals):
        raise ValueError('residual groups must be finite')
    return sum(w * float(np.sqrt(np.mean(np.asarray(r, dtype=float)**2)))
               for w, r in zip((1.,1.,.75,.75), residuals))


def _groups(snapshot, data, cancel):
    def predict(samples):
        if cancel(): raise FitCancelled()
        pred = predict_steady(snapshot, samples, data.belt_speed)
        return pred
    lat, lon = predict(data.lateral), predict(data.longitudinal)
    cfx, cfy = predict(data.combined_fx), predict(data.combined_fy)
    for p in (lat, lon, cfx, cfy):
        if not (np.all(np.isfinite(p.longitudinal_force)) and np.all(np.isfinite(p.lateral_force))):
            raise ValueError('nonfinite steady prediction')
    return ((lat.lateral_force/data.lateral.fz - data.lateral.target/data.lateral.fz),
            (lon.longitudinal_force/data.longitudinal.fz - data.longitudinal.target/data.longitudinal.fz),
            (cfx.longitudinal_force/data.combined_fx.fz - data.combined_fx.target/data.combined_fx.fz),
            (cfy.lateral_force/data.combined_fy.fz - data.combined_fy.target/data.combined_fy.fz))


def _safe_cost(snapshot, data, cancel):
    groups = _groups(snapshot, data, cancel)
    cost = force_cost(groups)
    if not np.isfinite(cost): raise ValueError('nonfinite steady objective')
    return cost


def fit_steady(snapshot: ModelSnapshot, data: SteadySelection, locked: frozenset[str], cancel: Callable[[], bool] = lambda: False, progress: Callable[[dict], None] = lambda _: None) -> FitResult:
    if any(np.any(s.fz <= 0) for s in (data.lateral,data.longitudinal,data.combined_fx,data.combined_fy,data.mz)):
        raise ValueError('selected samples require positive loads')
    active_force = tuple(n for n in FORCE_NAMES if n not in locked)
    active_trail = tuple(n for n in TRAIL_NAMES if n not in locked)
    if not active_force and not active_trail: raise ValueError('no unlocked fit parameters')
    warnings, status = [], dict(snapshot.parameter_status)
    current = snapshot
    metrics = {}
    converged = True
    def optimize(names, bounds, objective_name):
        nonlocal current, converged
        if not names: return
        limits = [bounds[n] for n in names]
        starts = []
        for n,(lo,hi) in zip(names,limits):
            raw = current.values[n]
            clipped = float(np.clip(raw, lo + (hi-lo)*1e-9, hi - (hi-lo)*1e-9))
            if clipped != raw: warnings.append(f'{n} optimizer start clamped to configured bounds; seed unchanged')
            starts.append(clipped)
        def encode(vals):
            return logit(np.clip((np.asarray(vals)-np.array([b[0] for b in limits]))/np.array([b[1]-b[0] for b in limits]),1e-12,1-1e-12))
        def decode(z):
            return np.array([lo+(hi-lo)*expit(v) for v,(lo,hi) in zip(z,limits)])
        best = [float('inf'), current]
        iterations = [0]
        def objective(z):
            if cancel(): raise FitCancelled()
            try:
                candidate = with_parameters(current, dict(zip(names, decode(z))))
                value = _safe_cost(candidate,data,cancel) if objective_name == 'force' else _trail_cost(candidate,data,cancel)
            except FitCancelled:
                raise
            except (ValueError, FloatingPointError, OverflowError):
                # Invalid load-domain candidates are rejected, not committed;
                # Nelder-Mead can continue exploring the finite feasible region.
                return 1e100
            if not np.isfinite(value): return 1e100
            if value < best[0]: best[:] = [value,candidate]
            return value
        result = minimize(objective, encode(starts), method='Nelder-Mead', options={'maxiter':800,'maxfev':4000,'xatol':1e-5,'fatol':1e-5, 'adaptive':False})
        if best[1] is not current: current = best[1]
        converged = converged and bool(result.success)
        saturated = []
        for n, value, (lo, hi) in zip(names, decode(result.x), limits):
            if min(value-lo, hi-value) <= 1e-5 * (hi-lo):
                status[n] = 'bound-saturated'
                saturated.append(n)
            else:
                status[n] = 'fitted' if result.success else 'not converged'
        if saturated: warnings.extend(f'{n} reached a configured fit bound' for n in saturated)
        metrics[f'{objective_name}_objective'] = float(best[0])
        metrics[f'{objective_name}_iterations'] = float(result.nit)
        progress({'stage':objective_name,'iterations':int(result.nit),'objective':float(best[0])})
    optimize(active_force, FORCE_BOUNDS, 'force')
    optimize(active_trail, TRAIL_BOUNDS, 'trail')
    for name in (*FORCE_NAMES,*TRAIL_NAMES): status.setdefault(name, 'locked' if name in locked else 'unchanged')
    # Dimensional RMSEs are scalar-only metrics.
    for key,samples,component in [('fy_rmse',data.lateral,'lateral_force'),('fx_rmse',data.longitudinal,'longitudinal_force'),('combined_fx_rmse',data.combined_fx,'longitudinal_force'),('combined_fy_rmse',data.combined_fy,'lateral_force'),('mz_rmse',data.mz,'aligning_moment')]:
        pred = predict_steady(current,samples,data.belt_speed)
        metrics[key] = float(np.sqrt(np.mean((getattr(pred,component)-samples.target)**2)))
    for key,samples in [('lateral_count',data.lateral),('longitudinal_count',data.longitudinal),('combined_fx_count',data.combined_fx),('combined_fy_count',data.combined_fy),('mz_count',data.mz)]: metrics[key] = float(samples.target.size)
    # Steady edits invalidate prior transient fit evidence. Preserve the
    # provisional default transient coefficients and make that status explicit.
    provenance = dict(current.provenance)
    provenance['transient_fit_status'] = 'provisional'
    transient_status = dict(current.parameter_status)
    for name in ('longitudinal_length_scale','lateral_length_scale','trail_pole_ratio','trail_lead_ratio'):
        transient_status[name] = 'provisional (steady fit does not identify transient dynamics)'
    current = type(current)(current.revision,current.steady,current.transient,current.values,provenance,
                            transient_status,current.steady_fingerprint,current.load_domain)
    if not converged: warnings.append('one or more optimizer stages did not converge; best finite candidate retained and not applied')
    return FitResult(snapshot.revision,current,metrics,converged,tuple(warnings),status)


def _trail_cost(snapshot,data,cancel):
    if cancel(): raise FitCancelled()
    samples=data.mz
    pred=predict_steady(snapshot,samples,data.belt_speed)
    if not np.all(np.isfinite(pred.aligning_moment)): raise ValueError('nonfinite steady prediction')
    scale=np.maximum(samples.fz*np.maximum(snapshot.steady.r_1,.001),1.)
    return float(np.sqrt(np.mean(((pred.aligning_moment-samples.target)/scale)**2)))
