"""Versioned, integrity-checked yaw-map artifacts; incomplete builds fail closed."""
from __future__ import annotations
from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash

SCHEMA = 'vehicle-limit-yaw-map-v1'
FIELDS = ('speed', 'longitudinal_acceleration', 'lateral_acceleration',
          'yaw_moment_into_turn', 'yaw_moment_out_of_turn', 'feasible_mask')


def json_values(value):
    if is_dataclass(value): return json_values(asdict(value))
    if isinstance(value, Enum): return value.value
    if isinstance(value, dict): return {str(k): json_values(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [json_values(v) for v in value]
    if isinstance(value, np.ndarray): return json_values(value.tolist())
    if isinstance(value, np.generic): return json_values(value.item())
    if isinstance(value, Path): return str(value)
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def implementation_identity():
    """Invalidate cached artifacts when the builder or its physics changes."""
    root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    paths = ('formulations/yaw_map_artifact.py', 'formulations/yaw_capability.py', 'formulations/yaw_map_domain.py',
             'formulations/yaw_map_workflow.py', 'formulations/yaw_map_physical.py', 'formulations/yaw_map_parallel.py', 'formulations/yaw_map_reduced.py', 'formulations/point_mass.py',
             'parameters/tire.py', 'suspension/quasi_static.py')
    for name in paths:
        digest.update(name.encode()); digest.update((root / name).read_bytes())
    return digest.hexdigest()


def source_provenance(formulation):
    """Hash the formulation and transitive local XML/data references."""
    import xml.etree.ElementTree as ET
    pending = [Path(formulation).resolve()]
    sources = {}
    while pending:
        path = pending.pop()
        if str(path) in sources: continue
        payload = path.read_bytes()
        sources[str(path)] = hashlib.sha256(payload).hexdigest()
        if path.suffix.lower() == '.xml':
            for node in ET.fromstring(payload).iter():
                for key in ('ref', 'file', 'path', 'href'):
                    value = node.attrib.get(key)
                    if value:
                        referenced = (path.parent / value).resolve()
                        if referenced.is_file(): pending.append(referenced)
    return sources


def build_identity(vehicle, yaw_inertia, settings, speed, ax, tv_limits=None):
    return capability_hash((SCHEMA, implementation_identity(), vehicle, yaw_inertia,
                            settings, np.asarray(speed, float), np.asarray(ax, float), tv_limits))


def map_digest(capability):
    return capability_hash(tuple(getattr(capability, field) for field in FIELDS) +
                           (capability.yaw_inertia, capability.source_kind, capability.source_identity))


def save_yaw_map(path, capability, *, report, identity, provenance=None):
    """Save metadata inside the NPZ so arrays and acceptance evidence travel together."""
    path = Path(path)
    if path.exists(): raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata = dict(schema=SCHEMA, build_identity=identity, map_digest=map_digest(capability),
                    yaw_inertia=capability.yaw_inertia, source_kind=capability.source_kind,
                    source_identity=capability.source_identity, report=json_values(report),
                    provenance=json_values(provenance or {}))
    encoded = json.dumps(metadata, sort_keys=True, allow_nan=False)
    with path.open('xb') as stream:
        np.savez_compressed(stream, **{f: getattr(capability, f) for f in FIELDS}, metadata=np.array(encoded))
    return metadata


def load_yaw_map(path, *, expected_identity, allow_incomplete=False):
    """Reject stale inputs, tampered arrays, or incomplete acceptance by default."""
    with np.load(path, allow_pickle=False) as archive:
        metadata = json.loads(str(archive['metadata'].item()))
        if metadata.get('schema') != SCHEMA: raise ValueError('unsupported yaw-map artifact schema')
        if metadata.get('build_identity') != expected_identity: raise ValueError('stale yaw-map build identity')
        if not allow_incomplete and metadata.get('report', {}).get('complete') is not True:
            raise ValueError('yaw-map build is incomplete')
        capability = YawCapabilityMap(**{f: archive[f].copy() for f in FIELDS},
            yaw_inertia=metadata['yaw_inertia'], source_kind=metadata['source_kind'],
            source_identity=metadata['source_identity'])
    if map_digest(capability) != metadata.get('map_digest'): raise ValueError('yaw-map array integrity mismatch')
    return capability, metadata
