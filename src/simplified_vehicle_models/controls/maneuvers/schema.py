"""Strict serialization and validation for versioned maneuver definitions."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile


class CapabilityError(ValueError):
    """A valid request exceeds available backend/model/calibration capability."""


_UNITS = {
    ('deg', 'rad'): math.pi / 180, ('rad', 'deg'): 180 / math.pi,
    ('km/h', 'm/s'): 1 / 3.6, ('m/s', 'km/h'): 3.6,
    ('g', 'm/s^2'): 9.80665, ('m/s^2', 'g'): 1 / 9.80665,
    ('percent', 'fraction'): .01, ('fraction', 'percent'): 100.,
    ('ms', 's'): .001, ('s', 'ms'): 1000.,
}

def convert_unit(value: float, source: str, target: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('unit value must be a finite number, not boolean')
    if source == target and source in {'deg','rad','km/h','m/s','g','m/s^2','percent','fraction','ms','s'}:
        return float(value)
    try: factor = _UNITS[source, target]
    except KeyError as exc: raise ValueError(f'unsupported unit conversion: {source!r} to {target!r}') from exc
    return float(value) * factor

_TOP = {'schema','name','preset','backend','vehicle','initial','clock','steering','longitudinal','profile','limits','termination','calibration','csv'}
_PRESETS = {'step_steer','ramp_steer','sine_steer','straight_maintain','straight_accel_brake','circular_skidpad','csv_playback'}
_ALLOWED = {
 'vehicle': {'model','components','name'},
 'initial': {'speed_mps','x_m','y_m','heading_rad'},
 'clock': {'controller_tick_s'},
 'steering': {'mode','amplitude_rad','amplitude_mps2','direction','frequency_hz','cycles','target_heading_rad','target_radius_m','kp','ki','kd','preview_distance_m','minimum_speed_mps'},
 'longitudinal': {'mode','accelerator','target_speed_mps','kp','ki','kd','derivative_tau_s','brake_allowed','start_accelerator','end_accelerator','start_brake','end_brake'},
 'profile': {'lead_in_s','rise_s','ramp_s','hold_s','release_s','recovery_s','duration_s','cycles','settle_s','sweep_start_mps','sweep_end_mps','sweep_rate_mps2'},
 'limits': {'max_swa_rad','max_swa_rate_rad_s','max_speed_mps','max_distance_m'},
 'termination': {'duration_s','speed_mps','distance_m','travel_limit_s'},
 'calibration': {'id','identifier','source','revision','units','operating_envelope','expires','expires_at','artifact','kind','maximum_swa_rad','maximum_swa_rate_rad_s','rack_travel_per_revolution_m','steer_arm_length_m','maximum_road_wheel_angle_rad','ackermann_inner_angle_rad','ackermann_outer_offset_rad','accelerator_travel','accelerator_force_n','brake_travel','brake_force_n'}, 'csv': {'mapping','interpolation','time_unit','swa_unit','accelerator_unit','brake_unit','endpoint','sha256','source'},
 'calibration.operating_envelope': {'min_speed_mps','max_speed_mps','max_swa_rad','max_swa_rate_rad_s','max_accelerator','max_brake'},
 'csv.mapping': {'time','swa','accelerator','brake'},
}

def _finite_numbers(value, path=''):
    if isinstance(value, dict):
        for k,v in value.items(): _finite_numbers(v, f'{path}.{k}')
    elif isinstance(value, list):
        for i,v in enumerate(value): _finite_numbers(v, f'{path}[{i}]')
    elif isinstance(value, bool) or value is None or isinstance(value, str): return
    elif isinstance(value, (int,float)):
        if not math.isfinite(value): raise ValueError(f'{path} must be finite')
    else: raise ValueError(f'{path} contains unsupported value')

def _keys(obj, allowed, path):
    if not isinstance(obj, dict): raise ValueError(f'{path} must be an object')
    unknown = set(obj) - allowed
    if unknown: raise ValueError(f'unknown key(s) in {path}: {sorted(unknown)}')


_NESTED_OBJECTS = {
    'calibration.operating_envelope': _ALLOWED['calibration.operating_envelope'],
    'csv.mapping': _ALLOWED['csv.mapping'],
}

def _validate_objects(value, path=''):
    """Validate each object against its owning schema; nested objects are explicit."""
    if not isinstance(value, dict):
        return
    allowed = _ALLOWED.get(path)
    if allowed is None:
        allowed = _NESTED_OBJECTS.get(path)
    if allowed is None:
        raise ValueError(f'unexpected nested object at {path or "definition"}')
    _keys(value, allowed, path)
    for key, child in value.items():
        child_path = f'{path}.{key}' if path else key
        if child_path in _NESTED_OBJECTS and not isinstance(child, dict):
            raise ValueError(f'{child_path} must be an object')
        if isinstance(child, dict):
            _validate_objects(child, child_path)
        elif isinstance(child, list):
            for index, item in enumerate(child):
                if isinstance(item, (dict, list)):
                    raise ValueError(f'unexpected nested structure at {child_path}[{index}]')
    if path == 'csv.mapping':
        for key, column in value.items():
            if not isinstance(column, str) or not column:
                raise ValueError(f'{path}.{key} must be a nonempty column name string')

def validate_definition(payload):
    if not isinstance(payload, dict): raise ValueError('definition must be an object')
    _keys(payload, _TOP, 'definition')
    missing = _TOP - payload.keys()
    if missing: raise ValueError(f'missing definition key(s): {sorted(missing)}')
    data = deepcopy(payload); _finite_numbers(data)
    if data['schema'] != 'maneuver-v1': raise ValueError('schema must be maneuver-v1')
    if not isinstance(data['name'], str) or not data['name'].strip(): raise ValueError('name must be nonempty')
    if data['preset'] not in _PRESETS: raise ValueError('unsupported preset')
    if data['backend'] not in ('simulation','fake','physical'): raise ValueError('unsupported backend')
    for key in _TOP - {'schema', 'name', 'preset', 'backend'}:
        value = data[key]
        if value is None:
            if key not in ('calibration', 'csv'):
                raise ValueError(f'{key} cannot be null')
        else:
            _keys(value, _ALLOWED[key], key)
            _validate_objects(value, key)
    if data['calibration'] is not None and not isinstance(data['calibration'], dict):
        raise ValueError('calibration must be an object or null')
    if data['csv'] is not None and not isinstance(data['csv'], dict):
        raise ValueError('csv must be an object or null')
    positive_time_keys = {'duration_s','controller_tick_s','rise_s','ramp_s','hold_s','release_s','settle_s','travel_limit_s'}
    nonnegative_time_keys = {'lead_in_s','recovery_s','derivative_tau_s'}
    for section in ('initial','clock','steering','longitudinal','profile','limits','termination'):
        for key, value in data[section].items():
            timing = key in positive_time_keys | nonnegative_time_keys or key == 'sweep_rate_mps2'
            if timing:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f'{section}.{key} must be a finite number')
                if key in positive_time_keys or key == 'sweep_rate_mps2':
                    if value <= 0: raise ValueError(f'{section}.{key} must be positive')
                elif key in nonnegative_time_keys:
                    if value < 0: raise ValueError(f'{section}.{key} must be nonnegative')
    if data['clock'].get('controller_tick_s',0) <= 0: raise ValueError('controller_tick_s must be positive')
    direction=data['steering'].get('direction',1)
    if direction not in (-1,1): raise ValueError('direction must be -1 or +1')
    cycles=data['profile'].get('cycles',data['steering'].get('cycles'))
    if cycles is not None and (not isinstance(cycles,(int,float)) or int(cycles)!=cycles or cycles <= 0): raise ValueError('sine cycles must be a positive whole number')
    smode=data['steering'].get('mode'); lmode=data['longitudinal'].get('mode')
    if smode not in {'peak_swa','peak_ay','path_pid','locked_swa','radius_pid','csv'}: raise ValueError('unsupported steering mode')
    if smode == 'locked_swa' and ('amplitude_rad' in data['steering'] or 'amplitude_mps2' in data['steering']): raise ValueError('locked_swa mode cannot also specify an amplitude-design mode')
    if lmode not in {'cruise_pid','constant_accelerator','profile','csv','fixed_pedal'}: raise ValueError('unsupported longitudinal mode')
    if data['preset']=='csv_playback':
        if data['csv'] is None: raise ValueError('CSV preset requires csv configuration')
        if smode != 'csv' or lmode != 'csv': raise ValueError('CSV owns all channels; overlays are forbidden')
    elif smode=='csv' or lmode=='csv': raise ValueError('CSV channel owner requires csv_playback preset')
    if data['backend']=='physical' and not data['calibration']: raise CapabilityError('physical actuation requires calibration')
    if data['backend']=='physical' and data['preset']=='csv_playback' and not data['calibration']: raise CapabilityError('physical CSV playback requires calibration')
    if smode=='peak_ay' and not data['calibration']: raise CapabilityError('peak-ay steering requires calibration')
    return data

def load_definition(path: Path):
    from .types import ManeuverDefinition
    try:
        with Path(path).open('r', encoding='utf-8') as stream: payload=json.load(stream)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc: raise ValueError(f'malformed maneuver JSON: {exc}') from exc
    return ManeuverDefinition.from_dict(payload)

def save_definition(path: Path, definition) -> None:
    if not hasattr(definition, 'to_dict'): raise TypeError('definition must be a ManeuverDefinition')
    path=Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary=tempfile.mkstemp(prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            json.dump(definition.to_dict(),stream,indent=2,sort_keys=True,allow_nan=False); stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary,path)
    except Exception:
        try: os.unlink(temporary)
        except FileNotFoundError: pass
        raise
