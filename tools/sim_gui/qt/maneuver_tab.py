"""Pure maneuver draft/preflight model (independent of Qt widgets)."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

from simplified_vehicle_models.controls.maneuvers.schema import CapabilityError, convert_unit
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition


@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    errors: dict[str, str]


class ManeuverDraft:
    """Editable SI-backed maneuver payload with presentation conversions."""
    def __init__(self, payload: dict):
        self._payload = deepcopy(payload)

    @classmethod
    def from_dict(cls, payload: dict) -> "ManeuverDraft":
        if not isinstance(payload, dict):
            raise TypeError("draft payload must be a dict")
        return cls(payload)

    def to_dict(self) -> dict:
        return deepcopy(self._payload)

    def set(self, dotted_path: str, value) -> None:
        keys = dotted_path.split('.')
        node = self._payload
        for key in keys[:-1]:
            node = node[key]
        node[keys[-1]] = value

    def present(self, dotted_path: str, unit: str) -> float:
        """Return the SI stored scalar rendered in a supported UI unit."""
        si_units = {'deg': 'rad', 'km/h': 'm/s', 'g': 'm/s^2',
                    'percent': 'fraction', 'ms': 's'}
        keys = dotted_path.split('.')
        value = self._payload
        for key in keys:
            value = value[key]
        if unit not in si_units:
            raise ValueError(f"unsupported presentation unit: {unit}")
        return convert_unit(value, si_units[unit], unit)

    def assign_presented(self, dotted_path: str, value: float, unit: str) -> None:
        si_units = {'deg': 'rad', 'km/h': 'm/s', 'g': 'm/s^2',
                    'percent': 'fraction', 'ms': 's'}
        if unit not in si_units:
            raise ValueError(f"unsupported presentation unit: {unit}")
        self.set(dotted_path, convert_unit(value, unit, si_units[unit]))

    def validate(self) -> tuple[ManeuverDefinition | None, dict[str, str]]:
        try:
            return ManeuverDefinition.from_dict(self._payload), {}
        except (ValueError, TypeError) as exc:
            message = str(exc)
            # Schema messages often include a dotted key; preserve it for inline UI.
            field = next((part for part in message.replace(':', ' ').split()
                          if '.' in part and part.split('.', 1)[0] in self._payload), "definition")
            return None, {field.strip("'\" ,") : message}

    def enabled_fields(self) -> dict[str, bool]:
        steering = self._payload.get('steering', {})
        longitudinal = self._payload.get('longitudinal', {})
        csv = self._payload.get('preset') == 'csv_playback'
        smode, lmode = steering.get('mode'), longitudinal.get('mode')
        return {
            'steering.amplitude_rad': not csv and smode == 'peak_swa',
            'steering.amplitude_mps2': not csv and smode == 'peak_ay',
            'steering.target_heading_rad': not csv and smode == 'path_pid',
            'steering.target_radius_m': not csv and smode == 'radius_pid',
            'longitudinal.accelerator': not csv and lmode == 'constant_accelerator',
            'longitudinal.target_speed_mps': not csv and lmode == 'cruise_pid',
            'csv': csv,
        }

    def preview(self, applied: dict | None = None) -> dict:
        return {'requested': self.to_dict(), 'applied': deepcopy(applied) if applied is not None else None}

    def preflight(self, *, calibration_status=None, calibration_fingerprint=None,
                  expected_fingerprint=None, csv_path=None, csv_snapshot=None,
                  backend=None) -> PreflightResult:
        errors = {}
        definition, validation_errors = self.validate()
        errors.update(validation_errors)
        data = self._payload
        calibration = data.get('calibration') or {}
        status = calibration_status if calibration_status is not None else calibration.get('status')
        fingerprint = calibration_fingerprint if calibration_fingerprint is not None else calibration.get('definition_sha256', calibration.get('fingerprint'))
        expected = expected_fingerprint or calibration.get('expected_fingerprint')
        if data.get('steering', {}).get('mode') == 'peak_ay' or (backend or data.get('backend')) == 'physical':
            if status != 'matched':
                errors['calibration'] = f"calibration status must be matched (got {status or 'missing'})"
            if expected and fingerprint != expected:
                errors['calibration'] = 'calibration fingerprint does not match the current definition'
        active_backend = backend or data.get('backend')
        if active_backend == 'physical':
            errors['backend'] = 'physical backend is unavailable in this GUI'
        if data.get('preset') == 'csv_playback':
            path = csv_path or (data.get('csv') or {}).get('source')
            if not path:
                errors['csv.source'] = 'CSV source path is required'
            elif csv_snapshot:
                try:
                    stat = Path(path).stat()
                    digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
                    if (stat.st_mtime_ns != csv_snapshot.get('mtime_ns') or stat.st_size != csv_snapshot.get('size')
                            or digest != csv_snapshot.get('sha256')):
                        errors['csv.source'] = 'CSV changed since it was loaded; reload it before running'
                except OSError as exc:
                    errors['csv.source'] = f'CSV source is unavailable: {exc}'
        return PreflightResult(not errors, errors)


def csv_file_snapshot(path: str | Path) -> dict:
    """Capture content hash and filesystem identity for a loaded CSV."""
    path = Path(path)
    stat = path.stat()
    return {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'mtime_ns': stat.st_mtime_ns, 'size': stat.st_size}
