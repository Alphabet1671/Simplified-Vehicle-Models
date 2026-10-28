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
from .parameters import CANONICAL_STEADY_NAMES, TRANSIENT_NAMES, UNITS, VECTOR_FIELDS

_STEADY_XML_NAMES = tuple(n for n in CANONICAL_STEADY_NAMES if n not in VECTOR_FIELDS) + tuple(VECTOR_FIELDS)
_VERSION = 2
_STEM = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


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
            xml_name = f'{vector}_{name.rsplit("_", 1)[1]}'
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


def _thaw(value):
    if isinstance(value, Mapping):
        return {k: _thaw(v) for k, v in value.items()}
    if isinstance(value, tuple):
        return [_thaw(v) for v in value]
    return value


def _safe_json(value, location='metadata'):
    """Validate finite scalar trees; arrays/objects and sample payload keys are never persisted."""
    forbidden = {'measurement','measurements','sample','samples','raw','raw_data','raw_samples','fitdata','channels','trace','traces','payload','data','array','arrays','signal','signals','values','vectors','series','points','records'}
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if not isinstance(key, str) or not key or key.lower().replace('-', '_') in forbidden:
                raise ValueError(f'unsafe key in {location}')
            result[key] = _safe_json(item, f'{location}.{key}')
        return result
    if isinstance(value, (list, tuple)):
        return [_safe_json(item, f'{location}[]') for item in value]
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    raise ValueError(f'unsupported or nonfinite value in {location}')


def _validate_metadata(metadata):
    metadata = _safe_json(_thaw(metadata), 'session')
    allowed = {'selections', 'locks', 'bounds', 'events', 'source_files', 'diagnostics'}
    unknown = set(metadata) - allowed
    if unknown:
        raise ValueError(f'unknown session metadata fields: {sorted(unknown)}')
    selections = metadata.get('selections', {})
    locks = metadata.get('locks', {})
    bounds = metadata.get('bounds', {})
    events = metadata.get('events', [])
    source_files = metadata.get('source_files', [])
    diagnostics = metadata.get('diagnostics', {})
    if not isinstance(selections, dict) or not isinstance(locks, dict) or not isinstance(bounds, dict) or not isinstance(events, list) or not isinstance(source_files, list) or not isinstance(diagnostics, dict):
        raise ValueError('session metadata has invalid top-level types')
    if any(not isinstance(k, str) or not isinstance(v, bool) for k, v in locks.items()):
        raise ValueError('locks must map parameter names to booleans')
    for name, pair in bounds.items():
        if not isinstance(name, str) or not isinstance(pair, list) or len(pair) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in pair) or pair[0] > pair[1]:
            raise ValueError('bounds must map parameter names to finite increasing numeric pairs')
    event_keys = {'id', 'start', 'stop', 'run_id', 'axis', 'approved', 'reason', 'source_file', 'segment_id', 'baseline_stop'}
    for event in events:
        if not isinstance(event, dict) or set(event) - event_keys or not isinstance(event.get('start'), (int, float)) or isinstance(event.get('start'), bool) or not isinstance(event.get('stop'), (int, float)) or isinstance(event.get('stop'), bool) or not math.isfinite(event['start']) or not math.isfinite(event['stop']) or event['stop'] <= event['start']:
            raise ValueError('events must contain only documented scalar fields and finite increasing start/stop windows')
        if any(not (item is None or isinstance(item, (str, bool, int)) or isinstance(item, float) and math.isfinite(item)) for item in event.values()):
            raise ValueError('event metadata fields must be scalar')
    def valid_selection(name, value):
        if value is None or isinstance(value, (str, bool, int, float)):
            return True
        if not isinstance(value, list) or len(value) > 1000:
            return False
        if all(x is None or isinstance(x, (str, bool, int, float)) for x in value):
            return True
        # Legacy tire-GUI v1 metadata serialized holdouts as [run_id, segment]
        # rows. Retain read compatibility; new writers use group-qualified IDs.
        return name == 'holdout_ids' and all(
            isinstance(row, list) and len(row) == 2 and all(isinstance(x, (str, int)) and not isinstance(x, bool) for x in row)
            for row in value)
    if any(not isinstance(k, str) or not valid_selection(k, v) for k,v in selections.items()):
        raise ValueError('selections must map names to scalar IDs or short ID lists')
    if any(not isinstance(k, str) or not (v is None or isinstance(v, (str, bool, int, float))) for k,v in diagnostics.items()):
        raise ValueError('diagnostics must contain scalar values only')
    if any(not isinstance(p, str) or not p or '\\x00' in p or len(p) > 4096 for p in source_files):
        raise ValueError('source_files must contain bounded nonempty private path references as strings')
    return {'selections': selections, 'locks': locks, 'bounds': bounds, 'events': events,
            'source_files': source_files, 'diagnostics': diagnostics}


def _owned_bundle(dest: Path, stem: str):
    if dest.is_symlink() or not dest.is_dir():
        return None
    try:
        manifest_path = dest / 'session.json'
        if manifest_path.is_symlink() or not manifest_path.is_file():
            return None
        previous = json.loads(manifest_path.read_text(encoding='utf-8'))
        if not isinstance(previous, dict) or type(previous.get('version')) is not int or previous['version'] not in (1, 2) or previous.get('tool') != 'tire-gui':
            return None
        if type(previous.get('transient_included')) is not bool:
            return None
        if previous.get('version') == 1:
            allowed = {'session.json', f'{stem}-unitire-simple.xml'}
            if previous.get('transient_included') is True:
                allowed.add(f'{stem}-unitire-transient-simple.xml')
        else:
            suffix = '-unitire-transient-simple.xml' if previous.get('transient_included') is True else '-unitire-simple.xml'
            allowed = {'session.json', f'{stem}{suffix}'}
        entries = list(dest.iterdir())
        if any(item.is_symlink() or not item.is_file() or item.name not in allowed for item in entries) or {item.name for item in entries} != allowed:
            return None
        stat = dest.stat()
        digest = __import__('hashlib').sha256(manifest_path.read_bytes()).hexdigest()
        file_signatures = tuple(sorted((item.name, item.stat().st_dev, item.stat().st_ino, item.stat().st_size, item.stat().st_mtime_ns) for item in entries))
        return stat.st_dev, stat.st_ino, allowed, digest, file_signatures
    except (OSError, ValueError, TypeError):
        return None


def _manifest(snapshot: ModelSnapshot, metadata: Mapping | None, include_transient: bool):
    combined = _thaw(snapshot.session_metadata)
    combined.update(_thaw(metadata or {}))
    metadata = _validate_metadata(combined)
    values = [{'name': n, 'unit': UNITS[n] if n not in VECTOR_FIELDS else UNITS[VECTOR_FIELDS[n][0]], 'value': float(snapshot.values[n])}
              for n in (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES)]
    statuses = dict(snapshot.parameter_status)
    for name in TRANSIENT_NAMES:
        statuses.setdefault(name, 'provisional')
    if not include_transient:
        statuses.update({n: 'provisional' for n in TRANSIENT_NAMES})
    return {'version': _VERSION, 'values': values, 'steady_fingerprint': snapshot.steady_fingerprint,
            'load_domain': list(snapshot.load_domain), 'provenance': _safe_json(dict(snapshot.provenance), 'provenance'),
            'parameter_status': _safe_json(statuses, 'parameter_status'), **metadata,
            'transient_included': bool(include_transient)}


def save_session(snapshot: ModelSnapshot, output_dir: Path, stem: str, *, include_transient: bool,
                 overwrite: bool = False, metadata: Mapping | None = None) -> tuple[Path, ...]:
    """Write compiler-validated XML and private session in a new atomic bundle directory."""
    if not isinstance(stem, str) or not _STEM.fullmatch(stem) or stem in {'.', '..'}:
        raise ValueError('stem must be a simple filename component')
    root = _safe_root(Path(output_dir)); dest = root / stem
    original_identity = None
    if dest.exists() or dest.is_symlink():
        if not overwrite:
            raise FileExistsError(dest)
        original_identity = _owned_bundle(dest, stem)
        if original_identity is None:
            raise FileExistsError('overwrite refused: destination is not an exact recognized tool bundle')
    temp = Path(tempfile.mkdtemp(prefix=f'.{stem}.tmp-', dir=root)).resolve()
    try:
        if include_transient:
            xml_path = temp / f'{stem}-unitire-transient-simple.xml'
            model_id = f'{stem}-transient-{uuid.uuid4().hex}'
            _xml(xml_path, model_id, 'unitire-transient-simple', snapshot.values,
                 (*_STEADY_XML_NAMES, *TRANSIENT_NAMES))
            _validate_xml(xml_path, True)
        else:
            xml_path = temp / f'{stem}-unitire-simple.xml'
            model_id = f'{stem}-steady-{uuid.uuid4().hex}'
            _xml(xml_path, model_id, 'unitire-simple', snapshot.values, _STEADY_XML_NAMES)
            _validate_xml(xml_path, False)
        result = [dest / xml_path.name]
        manifest = _manifest(snapshot, metadata, include_transient); manifest['tool'] = 'tire-gui'
        (temp / 'session.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if original_identity is None:
            if dest.exists() or dest.is_symlink():
                raise FileExistsError(dest)
            os.rename(temp, dest)
        else:
            current = _owned_bundle(dest, stem)
            if current != original_identity:
                raise FileExistsError('overwrite refused: destination changed during staging')
            backup = root / f'.{stem}.backup-{uuid.uuid4().hex}'
            os.rename(dest, backup)
            try:
                os.rename(temp, dest)
            except BaseException:
                if not dest.exists() and not dest.is_symlink():
                    os.rename(backup, dest)
                raise
        return tuple(result + [dest / 'session.json'])
    except BaseException:
        if temp.exists() and temp.is_dir() and not temp.is_symlink():
            shutil.rmtree(temp)
        raise


def _compile_legacy_pair(steady_model, transient_model, stem, parsed):
    """Validate and combine an exact v1 coefficient-only transient pair in memory."""
    from simplified_vehicle_models.xml.model import Model
    if transient_model.references:
        raise ValueError('legacy transient XML references are not permitted')
    if transient_model.subtype != 'unitire-transient-simple' or not re.fullmatch(re.escape(f'{stem}-transient-') + r'[0-9a-f]{32}', transient_model.model_id):
        raise ValueError('malformed or duplicate legacy transient XML model ID/subtype')
    if transient_model.model_id == steady_model.model_id:
        raise ValueError('duplicate model IDs in legacy pair')
    if set(transient_model.parameters) != set(TRANSIENT_NAMES) or set(transient_model.units) != set(TRANSIENT_NAMES):
        raise ValueError('legacy transient XML has unknown or missing parameters')
    for name in TRANSIENT_NAMES:
        if transient_model.units[name] != UNITS[name] or float(transient_model.parameters[name]) != parsed[name]:
            raise ValueError(f'legacy transient XML does not match canonical value/unit for {name}')
    complete_parameters = dict(steady_model.parameters)
    complete_units = dict(steady_model.units)
    complete_parameters.update(transient_model.parameters)
    complete_units.update(transient_model.units)
    combined = Model(path=transient_model.path, model_id=transient_model.model_id,
                     model_type=transient_model.model_type, subtype=transient_model.subtype,
                     parameters=complete_parameters, units=complete_units, references=())
    return compile_unitire_transient(combined)


def load_session(path: Path) -> ModelSnapshot:
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
    if (not isinstance(data, dict) or type(data.get('version')) is not int
            or data['version'] not in (1, 2) or data.get('tool') != 'tire-gui'):
        raise ValueError('unknown session format version or producer')
    stem = bundle.name
    if type(data.get('transient_included')) is not bool:
        raise ValueError('invalid transient_included flag')
    if data['version'] == 1:
        expected = {'session.json', f'{stem}-unitire-simple.xml'}
        if data['transient_included']:
            expected.add(f'{stem}-unitire-transient-simple.xml')
    else:
        suffix = '-unitire-transient-simple.xml' if data['transient_included'] else '-unitire-simple.xml'
        expected = {'session.json', f'{stem}{suffix}'}
    entries = list(bundle.iterdir())
    if any(p.is_symlink() or not p.is_file() for p in entries) or {p.name for p in entries} != expected:
        raise ValueError('session bundle contains missing, extra, or unsafe entries')
    rows = data.get('values')
    if not isinstance(rows, list):
        raise ValueError('session values must be rows')
    names = (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES); parsed = {}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'name','unit','value'}:
            raise ValueError('malformed canonical value row')
        name = row['name']; expected_unit = UNITS.get(name) if name not in VECTOR_FIELDS else UNITS[VECTOR_FIELDS[name][0]]
        if name not in names or name in parsed or row['unit'] != expected_unit:
            raise ValueError('unknown, duplicate, or wrong-unit canonical value')
        value = row['value']
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
            raise ValueError(f'invalid canonical value for {name}')
        parsed[name] = float(value)
    if set(parsed) != set(names):
        raise ValueError('session is missing canonical values')
    expected_metadata_keys = {'version', 'tool', 'values', 'steady_fingerprint', 'load_domain', 'provenance', 'parameter_status', 'selections', 'locks', 'bounds', 'events', 'source_files', 'diagnostics', 'transient_included'}
    if set(data) != expected_metadata_keys:
        raise ValueError('session manifest contains unknown or missing fields')
    domain = data.get('load_domain')
    if not isinstance(domain, list) or len(domain) != 2 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in domain):
        raise ValueError('invalid load domain')
    from simplified_vehicle_models.tires.unitire_transient import TransientTireParameters
    if data['version'] == 2:
        xml_path = next(p for p in entries if p.suffix == '.xml')
        model = load_model(xml_path)
        if data['transient_included']:
            transient = compile_unitire_transient(model)
            if model.subtype != 'unitire-transient-simple' or not re.fullmatch(re.escape(f'{stem}-transient-') + r'[0-9a-f]{32}', model.model_id):
                raise ValueError('malformed transient XML model ID/subtype')
            steady = transient.steady
            xml_values = {n: float(model.parameters[n]) for n in (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES) if n not in VECTOR_FIELDS}
            for flat, (vector, _) in VECTOR_FIELDS.items(): xml_values[flat] = float(model.parameters[f'{vector}_{flat.rsplit("_",1)[1]}'])
            if xml_values != parsed:
                raise ValueError('transient XML does not match canonical session values')
        else:
            steady = compile_unitire_simple(model)
            transient = TransientTireParameters(*(parsed[n] for n in TRANSIENT_NAMES))
            if model.subtype != 'unitire-simple' or not re.fullmatch(re.escape(f'{stem}-steady-') + r'[0-9a-f]{32}', model.model_id):
                raise ValueError('malformed steady XML model ID/subtype')
            canonical_xml = {n: float(model.parameters[n]) for n in CANONICAL_STEADY_NAMES if n not in VECTOR_FIELDS}
            for flat, (vector, _) in VECTOR_FIELDS.items(): canonical_xml[flat] = float(model.parameters[f'{vector}_{flat.rsplit("_",1)[1]}'])
            if canonical_xml != {n: parsed[n] for n in CANONICAL_STEADY_NAMES}:
                raise ValueError('steady XML does not match canonical session values')
    else:
        steady_model = load_model(bundle / f'{stem}-unitire-simple.xml'); steady = compile_unitire_simple(steady_model)
        if steady_model.subtype != 'unitire-simple' or not re.fullmatch(re.escape(f'{stem}-steady-') + r'[0-9a-f]{32}', steady_model.model_id):
            raise ValueError('malformed steady XML model ID/subtype')
        canonical_xml = {n: float(steady_model.parameters[n]) for n in CANONICAL_STEADY_NAMES if n not in VECTOR_FIELDS}
        for flat, (vector, _) in VECTOR_FIELDS.items(): canonical_xml[flat] = float(steady_model.parameters[f'{vector}_{flat.rsplit("_",1)[1]}'])
        if canonical_xml != {n: parsed[n] for n in CANONICAL_STEADY_NAMES}:
            raise ValueError('steady XML does not match canonical session values')
        if data['transient_included']:
            transient_model = load_model(bundle / f'{stem}-unitire-transient-simple.xml')
            transient = _compile_legacy_pair(steady_model, transient_model, stem, parsed)
        else:
            transient = TransientTireParameters(*(parsed[n] for n in TRANSIENT_NAMES))
    provenance = data.get('provenance', {}); statuses = data.get('parameter_status', {})
    if not isinstance(provenance, dict) or not isinstance(statuses, dict):
        raise ValueError('invalid provenance or parameter status')
    _safe_json(provenance, 'provenance'); _safe_json(statuses, 'parameter_status')
    if any(not isinstance(k, str) or not isinstance(v, str) for k,v in provenance.items()) or any(not isinstance(k, str) or not isinstance(v, str) for k,v in statuses.items()):
        raise ValueError('provenance and status mappings must contain strings')
    for name in TRANSIENT_NAMES:
        if statuses.get(name) not in {'validated', 'fitted', 'edited'}: statuses[name] = 'provisional'
    metadata = _validate_metadata({k: data.get(k, {}) for k in ('selections','locks','bounds','diagnostics')} | {'events': data.get('events', []), 'source_files': data.get('source_files', [])})
    snapshot = _make_snapshot(steady, transient, 0, provenance, statuses, (float(domain[0]), float(domain[1])), metadata)
    if snapshot.steady_fingerprint != data.get('steady_fingerprint'):
        raise ValueError('stale steady fingerprint')
    return snapshot
