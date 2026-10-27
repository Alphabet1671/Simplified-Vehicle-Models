"""Validated private export bundles for the tire correlation model."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import uuid
import xml.etree.ElementTree as ET
from typing import Mapping

from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
from simplified_vehicle_models.tires.unitire_transient import compile_unitire_transient
from simplified_vehicle_models.xml.loader import load_model
from .contracts import ModelSnapshot
from .model import _make_snapshot
from .parameters import CANONICAL_STEADY_NAMES, TRANSIENT_NAMES, UNITS, VECTOR_FIELDS, unflatten_steady

_STEADY_XML_NAMES = tuple(n for n in CANONICAL_STEADY_NAMES if n not in VECTOR_FIELDS) + tuple(VECTOR_FIELDS)

_VERSION = 1
_STEM = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")


def _safe_root(root: Path) -> Path:
    root = Path(root).expanduser()
    if root.is_symlink():
        raise ValueError('output directory may not be a symlink')
    resolved = root.resolve()
    if not resolved.is_dir():
        raise ValueError('output directory must already exist')
    if any(part.lower() == 'models' for part in resolved.parts):
        raise ValueError('export under models/ is refused')
    return resolved


def _xml(path: Path, model_id: str, subtype: str, values: Mapping[str, float], names):
    root = ET.Element('model', {'schema-version': '1.0', 'id': model_id, 'type': 'tire-model', 'subtype': subtype})
    params = ET.SubElement(root, 'parameters')
    for name in names:
        xml_name = name
        unit = UNITS.get(name)
        if name in VECTOR_FIELDS:
            vector, _ = VECTOR_FIELDS[name]
            degree = name.rsplit('_', 1)[1]
            xml_name = f'{vector}_{degree}'
            unit = UNITS[vector]
        ET.SubElement(params, 'parameter', {'name': xml_name, 'unit': unit, 'value': repr(float(values[name]))})
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def _validate_xml(path: Path, transient: bool):
    model = load_model(path)
    if transient:
        compile_unitire_transient(model)
    else:
        compile_unitire_simple(model)


def _manifest(snapshot: ModelSnapshot, metadata: Mapping | None, include_transient: bool):
    # Metadata is deliberately an allowlisted, caller-supplied private record; no sample arrays accepted.
    combined_metadata = dict(snapshot.session_metadata)
    combined_metadata.update(dict(metadata or {}))
    metadata = combined_metadata
    if 'measurements' in metadata or 'samples' in metadata or 'fitData' in metadata:
        raise ValueError('session metadata may not contain raw measurement arrays')
    values = [{'name': n, 'unit': (UNITS[n] if n not in VECTOR_FIELDS else UNITS[VECTOR_FIELDS[n][0]]), 'value': float(snapshot.values[n])}
              for n in (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES)]
    statuses = dict(snapshot.parameter_status)
    for name in TRANSIENT_NAMES:
        statuses.setdefault(name, 'provisional')
    if not include_transient:
        statuses.update({n: 'provisional' for n in TRANSIENT_NAMES})
    return {
        'version': _VERSION,
        'values': values,
        'steady_fingerprint': snapshot.steady_fingerprint,
        'load_domain': list(snapshot.load_domain),
        'provenance': dict(snapshot.provenance),
        'parameter_status': statuses,
        'selections': metadata.get('selections', {}),
        'locks': metadata.get('locks', {}),
        'bounds': metadata.get('bounds', {}),
        'events': metadata.get('events', []),
        'source_files': metadata.get('source_files', []),
        'session_metadata': {k: v for k, v in metadata.items() if k not in {'selections','locks','bounds','events','source_files'}},
        'transient_included': bool(include_transient),
    }


def save_session(snapshot: ModelSnapshot, output_dir: Path, stem: str, *, include_transient: bool,
                 overwrite: bool = False, metadata: Mapping | None = None) -> tuple[Path, ...]:
    """Write compiler-validated XML and private session in a new atomic bundle directory."""
    if not isinstance(stem, str) or not _STEM.fullmatch(stem) or stem in {'.', '..'}:
        raise ValueError('stem must be a simple filename component')
    root = _safe_root(Path(output_dir))
    dest = root / stem
    known_overwrite = False
    if dest.exists() or dest.is_symlink():
        if not overwrite:
            raise FileExistsError(dest)
        # Never replace an unrecognized or symlinked tree. A known bundle is one with our versioned manifest.
        if dest.is_symlink() or not dest.is_dir() or not (dest / 'session.json').is_file():
            raise FileExistsError('overwrite refused: destination is not a recognized tool-created bundle')
        try:
            previous = json.loads((dest / 'session.json').read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise FileExistsError('overwrite refused: existing session manifest is invalid') from exc
        if previous.get('version') != _VERSION or previous.get('tool') != 'tire-gui':
            raise FileExistsError('overwrite refused: destination ownership is unknown')
        allowed = {'session.json'}
        allowed.add(f'{stem}-unitire-simple.xml')
        if previous.get('transient_included'):
            allowed.add(f'{stem}-unitire-transient-simple.xml')
        entries = list(dest.iterdir())
        if any(item.is_symlink() or not item.is_file() or item.name not in allowed for item in entries) or {item.name for item in entries} != allowed:
            raise FileExistsError('overwrite refused: bundle contains unknown files')
        known_overwrite = True

    temp = Path(tempfile.mkdtemp(prefix=f'.{stem}.tmp-', dir=root)).resolve()
    # All paths are constructed beneath this freshly-created private directory.
    try:
        steady_path = temp / f'{stem}-unitire-simple.xml'
        _xml(steady_path, f'{stem}-steady-{uuid.uuid4().hex}', 'unitire-simple', snapshot.values, _STEADY_XML_NAMES)
        _validate_xml(steady_path, False)
        result = [dest / steady_path.name]
        if include_transient:
            transient_path = temp / f'{stem}-unitire-transient-simple.xml'
            _xml(transient_path, f'{stem}-transient-{uuid.uuid4().hex}', 'unitire-transient-simple', snapshot.values, TRANSIENT_NAMES)
            _validate_xml(transient_path, True)
            result.append(dest / transient_path.name)
        manifest = _manifest(snapshot, metadata, include_transient)
        manifest['tool'] = 'tire-gui'
        (temp / 'session.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if dest.exists() or dest.is_symlink():
            if not overwrite or not known_overwrite or dest.is_symlink() or not dest.is_dir():
                raise FileExistsError(dest)
            backup = root / f'.{stem}.backup-{uuid.uuid4().hex}'
            # Preserve the old known bundle as a sibling backup; never delete it.
            os.rename(dest, backup)
            try:
                os.rename(temp, dest)
            except BaseException:
                # Restore only the exact paths moved above. If another actor occupied dest, keep backup untouched.
                if not dest.exists() and not dest.is_symlink():
                    os.rename(backup, dest)
                raise
            return tuple(result + [dest / 'session.json'])
        os.rename(temp, dest)
        return tuple(result + [dest / 'session.json'])
    except BaseException:
        # temp was created by this call and is not a user tree.
        if temp.exists() and temp.is_dir() and not temp.is_symlink():
            shutil.rmtree(temp)
        raise


def load_session(path: Path) -> ModelSnapshot:
    """Load and strictly validate a private session bundle."""
    bundle = Path(path).expanduser()
    if bundle.is_symlink() or not bundle.is_dir():
        raise ValueError('session path must be a real bundle directory')
    bundle = bundle.resolve()
    manifest_path = bundle / 'session.json'
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError('session manifest missing or unsafe')
    try:
        data = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'malformed session manifest: {exc}') from exc
    if not isinstance(data, dict) or data.get('version') != _VERSION or data.get('tool') != 'tire-gui':
        raise ValueError('unknown session format version or producer')
    rows = data.get('values')
    if not isinstance(rows, list):
        raise ValueError('session values must be rows')
    names = (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES)
    parsed = {}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'name','unit','value'}:
            raise ValueError('malformed canonical value row')
        name = row['name']
        expected_unit = UNITS.get(name) if name not in VECTOR_FIELDS else UNITS[VECTOR_FIELDS[name][0]]
        if name not in names or name in parsed or row['unit'] != expected_unit:
            raise ValueError('unknown, duplicate, or wrong-unit canonical value')
        value = row['value']
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
            raise ValueError(f'invalid canonical value for {name}')
        parsed[name] = float(value)
    if set(parsed) != set(names):
        raise ValueError('session is missing canonical values')
    domain = data.get('load_domain')
    if not isinstance(domain, list) or len(domain) != 2:
        raise ValueError('invalid load domain')
    # Rebuild real compiler-accepted models from the bundle XML; session numbers must agree exactly.
    steady_files = list(bundle.glob('*-unitire-simple.xml'))
    transient_files = list(bundle.glob('*-unitire-transient-simple.xml'))
    if len(steady_files) != 1 or len(transient_files) > 1:
        raise ValueError('session XML file set is malformed')
    steady_model = load_model(steady_files[0]); steady = compile_unitire_simple(steady_model)
    xml_steady = {n: float(v) for n,v in steady_model.parameters.items()}
    canonical_xml = {name: xml_steady[name] for name in CANONICAL_STEADY_NAMES if name not in VECTOR_FIELDS}
    for flat, (vector, _) in VECTOR_FIELDS.items():
        degree = flat.rsplit('_', 1)[1]
        canonical_xml[flat] = xml_steady[f'{vector}_{degree}']
    if canonical_xml != {n: parsed[n] for n in CANONICAL_STEADY_NAMES}:
        raise ValueError('steady XML does not match canonical session values')
    included = bool(data.get('transient_included'))
    if included != (len(transient_files) == 1):
        raise ValueError('transient XML presence disagrees with session')
    from simplified_vehicle_models.tires.unitire_transient import TransientTireParameters
    if included:
        transient_model = load_model(transient_files[0]); transient = compile_unitire_transient(transient_model)
        if {n: float(v) for n,v in transient_model.parameters.items()} != {n: parsed[n] for n in TRANSIENT_NAMES}:
            raise ValueError('transient XML does not match canonical session values')
    else:
        transient = TransientTireParameters(*(parsed[n] for n in TRANSIENT_NAMES))
    domain_tuple = (float(domain[0]), float(domain[1]))
    provenance = data.get('provenance', {})
    statuses = data.get('parameter_status', {})
    if not isinstance(provenance, dict) or not isinstance(statuses, dict):
        raise ValueError('invalid provenance or parameter status')
    if not isinstance(domain[0], (int, float)) or isinstance(domain[0], bool) or not isinstance(domain[1], (int, float)) or isinstance(domain[1], bool):
        raise ValueError('invalid load domain values')
    if not all(math.isfinite(float(v)) for v in domain):
        raise ValueError('load domain values must be finite')
    for name in TRANSIENT_NAMES:
        if statuses.get(name) not in {'validated', 'fitted', 'edited'}:
            statuses[name] = 'provisional'
    snapshot = _make_snapshot(steady, transient, 0, provenance, statuses, domain_tuple)
    if snapshot.steady_fingerprint != data.get('steady_fingerprint'):
        raise ValueError('stale steady fingerprint')
    metadata = {k: data.get(k, {}) for k in ('selections','locks','bounds','events','source_files')}
    metadata.update(data.get('session_metadata', {}))
    return _make_snapshot(steady, transient, 0, provenance, statuses, domain_tuple, metadata)
