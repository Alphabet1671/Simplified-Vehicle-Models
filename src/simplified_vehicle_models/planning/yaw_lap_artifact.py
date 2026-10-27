"""Persist validated yaw-lap plans so controller replays never re-optimize lines."""
from pathlib import Path
import hashlib
import json
import numpy as np
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile

SCHEMA='yaw-lap-plan-v1'
ARRAYS=('distance','curvature','curvature_rate','speed','reference_distance','north','east','heading')


def _digest(arrays):
    digest=hashlib.sha256()
    for key in sorted(arrays):
        value=np.ascontiguousarray(arrays[key],dtype=np.float64)
        digest.update(key.encode());digest.update(str(value.shape).encode());digest.update(value.tobytes())
    return digest.hexdigest()


def save_lap_plan(path, *, yaw_path, speed, geometry, map_identity, track_identity,
                  maximum_speed, profile_settings, provenance=None):
    path=Path(path)
    if path.exists():raise FileExistsError(path)
    arrays=dict(distance=np.asarray(yaw_path.distance),curvature=np.asarray(yaw_path.curvature),
        curvature_rate=np.asarray(yaw_path.curvature_rate),speed=np.asarray(speed))
    arrays.update({key:np.asarray(geometry[key],float) for key in ARRAYS[4:]})
    if any(arrays[key].shape!=arrays['distance'].shape for key in arrays):
        raise ValueError('lap plan geometry and speed must share the physical station grid')
    if any(not np.all(np.isfinite(value)) for value in arrays.values()):raise ValueError('lap plan arrays must be finite')
    if np.any(np.diff(arrays['reference_distance'])<=0):raise ValueError('reference stations must increase')
    if np.any(np.diff(arrays['distance'])<=0):raise ValueError('physical stations must increase')
    if yaw_path.is_closed:
        if not np.isclose(arrays['curvature'][0], arrays['curvature'][-1], rtol=0., atol=1e-8):
            raise ValueError('closed optimized-line curvature must agree at seam')
        if not np.isclose(arrays['curvature_rate'][0], arrays['curvature_rate'][-1], rtol=0., atol=1e-7):
            raise ValueError('closed optimized-line curvature rate must agree at seam')
    metadata=dict(schema=SCHEMA,map_identity=map_identity,track_identity=track_identity,
        maximum_speed=float(maximum_speed),is_closed=yaw_path.is_closed,
        profile_settings={key:getattr(profile_settings,key) for key in profile_settings.__dataclass_fields__},
        array_digest=_digest(arrays),provenance=provenance or {})
    encoded=json.dumps(metadata,allow_nan=False,sort_keys=True)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as stream:np.savez_compressed(stream,**arrays,metadata=np.array(encoded))


def load_lap_plan(path, capability, *, expected_track_identity):
    with np.load(path,allow_pickle=False) as archive:
        metadata=json.loads(str(archive['metadata'].item()))
        if metadata.get('schema')!=SCHEMA:raise ValueError('unsupported lap plan schema')
        if metadata['map_identity']!=capability.source_identity:raise ValueError('lap plan map identity mismatch')
        if metadata['track_identity']!=expected_track_identity:raise ValueError('lap plan track identity mismatch')
        arrays={key:np.array(archive[key],float,copy=True) for key in ARRAYS}
    if _digest(arrays)!=metadata['array_digest']:raise ValueError('lap plan array integrity mismatch')
    path=YawPath(arrays['distance'],arrays['curvature'],arrays['curvature_rate'],metadata['is_closed'])
    settings=CoupledProfileSettings(**metadata['profile_settings'])
    validation=validate_profile(path,capability,arrays['speed'],maximum_speed=metadata['maximum_speed'],
        boundary='flying' if path.is_closed else 'standing',settings=settings)
    if not validation.valid:raise ProfileInvalidError('profile_invalid: saved lap plan fails independent strict validation')
    plan=SpeedPlan(path.distance,arrays['speed'],path.curvature,is_closed=path.is_closed)
    for array in arrays.values():array.setflags(write=False)
    return path,plan,arrays,metadata,validation
