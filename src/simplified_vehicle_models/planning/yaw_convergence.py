"""Independent convergence claims for yaw-aware lap studies.

Repeated solves at one resolution are repeatability evidence, not a mesh or
control-basis convergence study. All comparisons use common reference stations.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ConvergenceSample:
    label: str
    geometry_spacing_m: float
    control_spacing_m: float
    lap_time_s: float
    reference_distance: np.ndarray
    speed: np.ndarray
    offset: np.ndarray
    profile_valid: bool
    profile_resolution_converged: bool
    optimizer_converged: bool
    budget_reason: str | None = None

    def __post_init__(self):
        for name in ('geometry_spacing_m','control_spacing_m','lap_time_s'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError(f'{name} must be finite and positive')
        station=np.array(self.reference_distance,float,copy=True)
        speed=np.array(self.speed,float,copy=True); offset=np.array(self.offset,float,copy=True)
        if (station.ndim!=1 or station.size<2 or speed.shape!=station.shape or offset.shape!=station.shape
            or any(not np.all(np.isfinite(a)) for a in (station,speed,offset))
            or np.any(np.diff(station)<=0) or np.any(speed<0)):
            raise ValueError('convergence samples require increasing finite reference stations and matching arrays')
        for name,a in (('reference_distance',station),('speed',speed),('offset',offset)):
            a.setflags(write=False);object.__setattr__(self,name,a)


@dataclass(frozen=True)
class ConvergenceTolerances:
    time_relative: float=.005
    speed_max_mps: float=.1
    offset_max_m: float=.05
    def __post_init__(self):
        if any(not math.isfinite(x) or x<=0 for x in (self.time_relative,self.speed_max_mps,self.offset_max_m)):
            raise ValueError('convergence tolerances must be finite and positive')


def compare_samples(coarse, fine, *, dimension, tolerances=ConvergenceTolerances()):
    """Compare one finer resolution with other resolution dimensions held fixed."""
    if dimension not in ('geometry','control'):
        raise ValueError('dimension must be geometry or control')
    key='geometry_spacing_m' if dimension=='geometry' else 'control_spacing_m'
    other='control_spacing_m' if dimension=='geometry' else 'geometry_spacing_m'
    reasons=[]
    if getattr(fine,key)>=getattr(coarse,key): reasons.append('not_a_strict_refinement')
    if getattr(fine,other)!=getattr(coarse,other): reasons.append('unmatched_other_resolution')
    for row in (coarse,fine):
        if not row.profile_valid: reasons.append(f'{row.label}:profile_invalid')
        if not row.profile_resolution_converged: reasons.append(f'{row.label}:profile_resolution_unproven')
        if not row.optimizer_converged: reasons.append(f'{row.label}:optimizer_unconverged')
        if row.budget_reason: reasons.append(f'{row.label}:budget:{row.budget_reason}')
    # Must cover the same course, not just an overlapping short subsection.
    if not np.allclose(coarse.reference_distance[[0,-1]],fine.reference_distance[[0,-1]],rtol=0,atol=1e-8):
        reasons.append('unmatched_reference_domain')
    start=max(coarse.reference_distance[0],fine.reference_distance[0])
    end=min(coarse.reference_distance[-1],fine.reference_distance[-1])
    if end<=start:
        return {'dimension':dimension,'coarse':coarse.label,'fine':fine.label,'status':'inconclusive',
                'reasons':reasons+['no_common_reference_domain']}
    grid=np.unique(np.r_[start,end,coarse.reference_distance[(coarse.reference_distance>=start)&(coarse.reference_distance<=end)],
                        fine.reference_distance[(fine.reference_distance>=start)&(fine.reference_distance<=end)]])
    dspeed=float(np.max(np.abs(np.interp(grid,coarse.reference_distance,coarse.speed)-np.interp(grid,fine.reference_distance,fine.speed))))
    doffset=float(np.max(np.abs(np.interp(grid,coarse.reference_distance,coarse.offset)-np.interp(grid,fine.reference_distance,fine.offset))))
    dtime=abs(fine.lap_time_s-coarse.lap_time_s)/coarse.lap_time_s
    passed=dtime<=tolerances.time_relative and dspeed<=tolerances.speed_max_mps and doffset<=tolerances.offset_max_m
    return {'dimension':dimension,'coarse':coarse.label,'fine':fine.label,
            'status':'inconclusive' if reasons else ('converged' if passed else 'not_converged'),
            'reasons':reasons,'time_relative_difference':dtime,'speed_max_difference_mps':dspeed,
            'offset_max_difference_m':doffset,'common_reference_samples':len(grid)}


def convergence_study(samples, *, dimension, tolerances=ConvergenceTolerances()):
    """Require an actual refinement pair; report all independent statuses."""
    samples=list(samples)
    key='geometry_spacing_m' if dimension=='geometry' else 'control_spacing_m'
    if dimension not in ('geometry','control'): raise ValueError('unknown convergence dimension')
    ordered=sorted(samples,key=lambda row:getattr(row,key),reverse=True)
    comparisons=[compare_samples(a,b,dimension=dimension,tolerances=tolerances)
                 for a,b in zip(ordered,ordered[1:])]
    statuses=[c['status'] for c in comparisons]
    verdict=('inconclusive' if not statuses or 'inconclusive' in statuses else
             'converged' if all(s=='converged' for s in statuses) else 'not_converged')
    return {'dimension':dimension,'status':verdict,'comparisons':comparisons,
            'profile_resolution_converged':bool(samples) and all(x.profile_resolution_converged for x in samples),
            'optimizer_converged':bool(samples) and all(x.optimizer_converged for x in samples),
            'continuous_feasibility_certified':False,'global_optimality_certified':False}
