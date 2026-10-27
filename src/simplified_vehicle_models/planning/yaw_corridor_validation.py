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


def check_swept_footprint(*args, interval_bounds=None, subdivision_budget=10000, **kwargs):
    """Check sampled evidence plus explicit conservative interval bounds.

    A caller must supply sound lower bounds per interval; absent bounds or an
    exhausted budget produce unknown, never a sampling-based certificate.
    """
    result=check_footprint_samples(*args,**kwargs)
    bounds=interval_bounds
    if bounds is None:
        result.update(status='unknown', rigorous_interval_bounds={'available':False,'certified':False},
                      between_step_containment_certified=False,
                      certification_limitation='sound conservative interval bounds unavailable')
        return result
    # Bare numbers are not a proof: callers cannot establish provenance or
    # soundness by passing an array of purported lower bounds. This interface
    # currently has no interval geometry provider, so never certify from it.
    values=np.asarray(bounds,float)
    if values.ndim!=1 or not np.all(np.isfinite(values)) or np.any(np.diff(values)<0):
        result.update(status='unknown',rigorous_interval_bounds={'available':False,'certified':False},
                      between_step_containment_certified=False,certification_limitation='invalid interval bounds')
        return result
    # Supplied scalar bounds have no binding to an interval's geometry/motion,
    # so retain them as unverified metadata, not certification evidence.
    result['rigorous_interval_bounds']={'available':False,'supplied_unverified_lower_bounds_m':values.tolist(),
        'certified':False,'subdivision_count':0,'budget_exhausted':False}
    result['empirical_dense_sampling']={'samples':result.get('empirical_samples',0),'certified':False}
    result['between_step_containment_certified']=False
    result['status']='unknown'
    result['certification_limitation']='sound interval geometry and vehicle-motion bounds unavailable'
    return result
