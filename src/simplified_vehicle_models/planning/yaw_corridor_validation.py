"""Sampled footprint checks; these are evidence, not track certification."""
from __future__ import annotations

import numpy as np

from simplified_vehicle_models.planning.yaw_replay_metrics import project_xy_to_reference


def _result(status, clearance, time, station, model, edge, representation, uncertainty,
            **extra):
    return dict(status=status, minimum_clearance_m=(None if not np.isfinite(clearance) else float(clearance)),
                worst_time_s=time, worst_station_m=station, model_identity=model,
                edge_identity=edge, geometric_representation=representation,
                edge_uncertainty=uncertainty, **extra)


def _boundary_at(corridor, station):
    d = np.asarray(corridor.distance, float)
    q = np.asarray(station, float)
    if corridor.is_closed:
        q = (q - d[0]) % (d[-1] - d[0]) + d[0]
    else:
        q = np.clip(q, d[0], d[-1])
    return np.interp(q, d, corridor.minimum), np.interp(q, d, corridor.maximum)


def check_footprint_samples(time, x, y, heading, station_hint, corridor, reference, *,
                            front_extent=None, rear_extent=None, half_width=None,
                            wheelbase=None, track_width=None, model_identity="unknown",
                            edge_identity=None, edge_uncertainty=None, edge_samples=9):
    """Check sampled wheel contacts and optional rectangular body envelope.

    X/Y use north/east coordinates; heading is the actual body heading. Corridor
    lateral coordinates are positive right. The projection is branch-aware.
    """
    t, x, y, h, hint = np.broadcast_arrays(*map(lambda a: np.asarray(a, float),
                                                 (time, x, y, heading, station_hint)))
    if t.ndim != 1 or len(t) == 0 or not np.all(np.isfinite(np.r_[t,x,y,h,hint])):
        raise ValueError("sample inputs must be finite equal-length vectors")
    if np.any(np.diff(t) < 0):
        raise ValueError("time must be nondecreasing")
    def project(px, py, hs):
        return project_xy_to_reference(reference, np.asarray(px), np.asarray(py), np.asarray(hs),
                                       window=max(30., float(np.ptp(corridor.distance))), samples=241)
    # Four tire contacts use explicitly supplied wheelbase/track geometry only.
    wheel = None
    if wheelbase is not None and track_width is not None:
        wheel = []
        for longitudinal in (-float(wheelbase)/2, float(wheelbase)/2):
            for lateral in (-float(track_width)/2, float(track_width)/2):
                # In north/east frame: forward=(cos h,sin h), right=(-sin h,cos h).
                px=x+longitudinal*np.cos(h)-lateral*np.sin(h)
                py=y+longitudinal*np.sin(h)+lateral*np.cos(h)
                s,n=project(px,py,hint); lo,hi=_boundary_at(corridor,s)
                wheel.append((np.minimum(n-lo,hi-n),s))
    if wheel:
        wc=np.concatenate([v[0] for v in wheel]); ws=np.concatenate([v[1] for v in wheel])
        wi=int(np.argmin(wc)); wheel_result=_result('fail' if wc[wi] < -1e-9 else 'pass',wc[wi],
            float(t[wi % len(t)]),float(ws[wi]),model_identity,'nearest-left/right','four tire contact points',
            edge_uncertainty, sampled=True)
    else:
        wheel_result=_result('unknown',np.nan,None,None,model_identity,None,'wheel contacts unavailable',
                             edge_uncertainty, reason='wheelbase_or_track_width_missing',sampled=True)
    if front_extent is None or rear_extent is None or half_width is None:
        body=_result('unknown',np.nan,None,None,model_identity,None,'whole-body envelope unavailable',
                     edge_uncertainty,reason='explicit_front_rear_extents_and_half_width_required',sampled=True)
        return dict(status='unknown', wheel_contact=wheel_result, body_envelope=body,
                    minimum_clearance_m=None,worst_time_s=None,worst_station_m=None,
                    model_identity=model_identity,edge_identity=None,
                    geometric_representation=body['geometric_representation'],
                    edge_uncertainty=edge_uncertainty,
                    between_step_containment_certified=False, empirical_samples=len(t))
    dims=np.asarray([front_extent,rear_extent,half_width],float)
    if not np.all(np.isfinite(dims)) or min(dims)<=0: raise ValueError('body extents and half-width must be positive')
    clearance=np.full(len(t),np.inf); worst_station=np.zeros(len(t)); worst_edge='none'
    for i in range(len(t)):
        vertices=[]
        for longitudinal,lateral in ((front_extent,-half_width),(front_extent,half_width),
                                     (-rear_extent,half_width),(-rear_extent,-half_width)):
            vertices.append((x[i]+longitudinal*np.cos(h[i])-lateral*np.sin(h[i]),
                             y[i]+longitudinal*np.sin(h[i])+lateral*np.cos(h[i])))
        # Sample each polygon edge, catching crossings between inside vertices.
        edge_min=np.inf; edge_station=None; edge_name='none'
        for j in range(4):
            a=np.asarray(vertices[j]); b=np.asarray(vertices[(j+1)%4])
            q=np.linspace(0.,1.,max(2,int(edge_samples)))
            px=a[0]+q*(b[0]-a[0]); py=a[1]+q*(b[1]-a[1])
            hs=np.full(q.shape,hint[i]); s,n=project(px,py,hs); lo,hi=_boundary_at(corridor,s)
            c=np.minimum(n-lo,hi-n); k=int(np.argmin(c))
            if c[k]<edge_min: edge_min=float(c[k]); edge_station=float(s[k]); edge_name=f'body-edge-{j}'
        clearance[i]=edge_min; worst_station[i]=edge_station
        if edge_min < 0: worst_edge=edge_name
    k=int(np.argmin(clearance)); status='fail' if clearance[k]<-1e-9 else 'pass'
    # Finite edge samples can discover a violation but cannot prove that the
    # continuous polygon edge avoids an inward notch. Preserve detected fails;
    # otherwise report unknown rather than promoting samples to containment.
    body_status = 'fail' if status == 'fail' else 'unknown'
    body=_result(body_status,clearance[k],float(t[k]),float(worst_station[k]),model_identity,worst_edge,
                 'oriented rectangular body polygon with sampled edges (empirical only)',edge_uncertainty,sampled=True,
                 vertex_and_edge_sampling=True, continuous_edge_containment_certified=False,
                 reason=None if status == 'fail' else 'continuous body-edge boundary bound unavailable')
    return dict(status=body_status,wheel_contact=wheel_result,body_envelope=body,
                minimum_clearance_m=body['minimum_clearance_m'],worst_time_s=body['worst_time_s'],
                worst_station_m=body['worst_station_m'],model_identity=model_identity,
                edge_identity=worst_edge,geometric_representation=body['geometric_representation'],
                edge_uncertainty=edge_uncertainty,between_step_containment_certified=False,
                empirical_samples=len(t))


def _motion_clearance_lower_bound(endpoint_clearance, translation_bound, angular_change_bound, body_radius):
    """Conservative clearance arithmetic for fixed straight boundaries."""
    vals = np.asarray([endpoint_clearance, translation_bound, angular_change_bound, body_radius], float)
    if not np.all(np.isfinite(vals)) or translation_bound < 0 or angular_change_bound < 0 or body_radius < 0:
        return None
    return float(endpoint_clearance - translation_bound - body_radius * angular_change_bound)


class IntervalClearanceBoundProvider:
    """Explicit provider contract for conservative swept-clearance enclosures.

    Implementations must bind the returned bound to both ``geometry_model_identity``
    and ``vehicle_motion_model_identity`` and return a proven lower bound in metres,
    or None when unable to enclose the entire interval. Numeric arrays and arbitrary
    callbacks are deliberately not accepted as providers.
    """
    geometry_model_identity = None
    vehicle_motion_model_identity = None

    def lower_bound(self, *, interval_index, start_time_s, end_time_s, start_state,
                    end_state, trajectory, corridor, reference, footprint_geometry):
        raise NotImplementedError


def check_swept_footprint(time, x, y, heading, station_hint, corridor, reference, *,
                          interval_bound_provider=None, interval_bounds=None,
                          subdivision_budget=10000, **kwargs):
    """Check empirical samples and adaptively certify provider-enclosed intervals.

    Only an ``IntervalClearanceBoundProvider`` with nonempty model identities can
    certify. Every interval is bisected until the provider proves a positive bound;
    unavailable bounds, nonpositive terminal bounds, or budget exhaustion remain
    unknown. Sampled clearance and rigorous interval evidence are kept separate.
    """
    result=check_footprint_samples(time,x,y,heading,station_hint,corridor,reference,**kwargs)
    t,x,y,h,hint=np.broadcast_arrays(*map(lambda a: np.asarray(a,float),
                                      (time,x,y,heading,station_hint)))
    valid=(isinstance(interval_bound_provider,IntervalClearanceBoundProvider) and
           bool(interval_bound_provider.geometry_model_identity) and
           bool(interval_bound_provider.vehicle_motion_model_identity))
    rigorous={'available':valid,'certified':False,'provider_identity':type(interval_bound_provider).__name__ if valid else None,
              'geometry_model_identity':getattr(interval_bound_provider,'geometry_model_identity',None) if valid else None,
              'vehicle_motion_model_identity':getattr(interval_bound_provider,'vehicle_motion_model_identity',None) if valid else None,
              'certified_intervals':0,'subdivision_count':0,'budget_exhausted':False,
              'interval_lower_bounds_m':[]}
    result['empirical_dense_sampling']={'samples':int(len(t)),'minimum_clearance_m':result.get('minimum_clearance_m'),
                                         'certified':False}
    result['rigorous_interval_bounds']=rigorous
    result['between_step_containment_certified']=False
    result['swept_model_identity']={'vehicle':result.get('model_identity'),
        'geometry':rigorous['geometry_model_identity'],'motion':rigorous['vehicle_motion_model_identity']}
    if not valid:
        result.update(status='unknown',certification_limitation='sound interval geometry and vehicle-motion provider unavailable')
        return result
    if not isinstance(subdivision_budget,(int,np.integer)) or subdivision_budget<0:
        raise ValueError('subdivision_budget must be a nonnegative integer')
    # Bound calls cover the whole subinterval, not merely endpoint clearance.
    stack=[(i,float(t[i]),float(t[i+1]),np.array([x[i],y[i],h[i],hint[i]]),
            np.array([x[i+1],y[i+1],h[i+1],hint[i+1]]),0) for i in range(len(t)-1)]
    unknown=False; budget=int(subdivision_budget)
    while stack:
        i,a,b,sa,sb,depth=stack.pop()
        try:
            bound=interval_bound_provider.lower_bound(interval_index=i,start_time_s=a,end_time_s=b,
                start_state=sa.copy(),end_state=sb.copy(),trajectory=(t.copy(),x.copy(),y.copy(),h.copy(),hint.copy()),
                corridor=corridor,reference=reference,
                footprint_geometry={key:kwargs.get(key) for key in ('front_extent','rear_extent','half_width')})
            bound=float(bound) if bound is not None else np.nan
        except (NotImplementedError,ValueError,ArithmeticError):
            bound=np.nan
        if np.isfinite(bound) and bound>0:
            rigorous['certified_intervals']+=1; rigorous['interval_lower_bounds_m'].append(bound); continue
        if budget<=0 or b<=a:
            unknown=True; rigorous['interval_lower_bounds_m'].append(None); continue
        mid=(a+b)/2; frac=(mid-a)/(b-a); sm=sa+(sb-sa)*frac
        budget-=1; rigorous['subdivision_count']+=1
        stack.extend([(i,mid,b,sm,sb,depth+1),(i,a,mid,sa,sm,depth+1)])
    rigorous['budget_exhausted']=budget==0 and unknown
    rigorous['certified']=bool(rigorous['certified_intervals']==len(t)-1 and not unknown)
    result['between_step_containment_certified']=rigorous['certified']
    if rigorous['certified']:
        result['status']='pass' if result.get('status')!='fail' else 'fail'
        result['certification_limitation']=None if result['status']=='pass' else 'empirical footprint violation detected'
    else:
        result['status']='fail' if result.get('status')=='fail' else 'unknown'
        result['certification_limitation']='one or more swept intervals lack a proven positive lower bound'
    return result
