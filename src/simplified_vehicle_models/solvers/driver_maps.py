"""Immutable, unit-tagged pedal demand and actuator calibration maps."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
from pathlib import Path
from typing import Any


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number (not bool)")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _tuple(values: Any, name: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{name} must be a numeric sequence")
    try:
        result = tuple(_number(v, name) for v in values)
    except TypeError as exc:
        if "must be" in str(exc):
            raise
        raise TypeError(f"{name} must be a numeric sequence") from exc
    return result


@dataclass(frozen=True, slots=True)
class MonotoneTable1D:
    axis: tuple[float, ...]
    value: tuple[float, ...]
    axis_unit: str
    value_unit: str

    def __post_init__(self) -> None:
        x, y = _tuple(self.axis, "axis"), _tuple(self.value, "value")
        if len(x) < 2 or len(x) != len(y):
            raise ValueError("axis and value must have equal lengths of at least two")
        if any(b <= a for a, b in zip(x, x[1:])):
            raise ValueError("axis must be strictly increasing")
        if any(b < a for a, b in zip(y, y[1:])):
            raise ValueError("values must be nondecreasing")
        if not isinstance(self.axis_unit, str) or not self.axis_unit or not isinstance(self.value_unit, str) or not self.value_unit:
            raise ValueError("units must be nonempty strings")
        object.__setattr__(self, "axis", x)
        object.__setattr__(self, "value", y)

    def evaluate(self, x: float) -> float:
        q = _number(x, "query")
        if q <= self.axis[0]: return self.value[0]
        if q >= self.axis[-1]: return self.value[-1]
        for i, right in enumerate(self.axis[1:], 1):
            if q <= right:
                left = i - 1
                f = (q - self.axis[left]) / (right - self.axis[left])
                return self.value[left] + f * (self.value[i] - self.value[left])
        raise AssertionError("unreachable")

    def inverse(self, y: float) -> float:
        q = _number(y, "query")
        if q <= self.value[0]: return self.axis[0]
        if q >= self.value[-1]: return self.axis[-1]
        for i, val in enumerate(self.value):
            if val >= q:
                if val == q:
                    return self.axis[i - 1] if i and self.value[i - 1] == q else self.axis[i]
                left = i - 1
                f = (q - self.value[left]) / (val - self.value[left])
                return self.axis[left] + f * (self.axis[i] - self.axis[left])
        return self.axis[-1]


@dataclass(frozen=True, slots=True)
class LongitudinalDemandMap:
    acceleration_to_throttle: MonotoneTable1D
    deceleration_to_brake: MonotoneTable1D

    def __post_init__(self) -> None:
        for table in (self.acceleration_to_throttle, self.deceleration_to_brake):
            if not isinstance(table, MonotoneTable1D): raise TypeError("demand channels must be MonotoneTable1D")
            if table.axis_unit != "m/s^2" or table.value_unit != "1": raise ValueError("demand units must be m/s^2 to 1")
            if table.axis[0] != 0 or table.value[0] != 0 or table.value[-1] != 1 or table.axis[-1] <= 0:
                raise ValueError("demand channel must start at (0,0) and end at normalized output 1")
            if any(v < 0 or v > 1 for v in table.value): raise ValueError("pedal outputs must be in [0,1]")

    def command(self, acceleration_request_mps2: float) -> tuple[float, float]:
        a = _number(acceleration_request_mps2, "acceleration request")
        if a > 0: return (self.acceleration_to_throttle.evaluate(a), 0.0)
        if a < 0: return (0.0, self.deceleration_to_brake.evaluate(-a))
        return (0.0, 0.0)


@dataclass(frozen=True, slots=True)
class PedalActuatorCalibration:
    throttle_fraction: MonotoneTable1D
    brake_force_fraction: MonotoneTable1D

    def __post_init__(self) -> None:
        for table in (self.throttle_fraction, self.brake_force_fraction):
            if not isinstance(table, MonotoneTable1D): raise TypeError("calibration channels must be MonotoneTable1D")
            if table.axis_unit != "1" or table.value_unit != "1": raise ValueError("calibration units must be 1 to 1")
            if table.axis != (0.0, *table.axis[1:-1], 1.0) or table.value[0] != 0 or table.axis[-1] != 1 or table.value[-1] != 1:
                raise ValueError("calibration channels require exact (0,0) and (1,1) endpoints")
            if any(not 0 <= v <= 1 for v in table.axis + table.value): raise ValueError("calibration values must be in [0,1]")

    def shape_throttle(self, throttle_fraction: float) -> float:
        x = _number(throttle_fraction, "throttle fraction")
        if not 0 <= x <= 1: raise ValueError("throttle fraction must be in [0,1]")
        return self.throttle_fraction.evaluate(x)

    def shape_brake_force(self, brake_force_n: float, maximum_pedal_force_n: float) -> float:
        force = _number(brake_force_n, "brake force")
        maximum = _number(maximum_pedal_force_n, "maximum pedal force")
        if maximum <= 0: raise ValueError("maximum pedal force must be positive")
        if not 0 <= force <= maximum: raise ValueError("brake force must be within [0, maximum]")
        return self.brake_force_fraction.evaluate(force / maximum) * maximum


@dataclass(frozen=True, slots=True)
class PedalMapConfiguration:
    demand_map: LongitudinalDemandMap | None = None
    physical_calibration: PedalActuatorCalibration | None = None

    def __post_init__(self) -> None:
        if self.demand_map is not None and not isinstance(self.demand_map, LongitudinalDemandMap):
            raise TypeError("demand_map must be LongitudinalDemandMap or None")
        if self.physical_calibration is not None and not isinstance(self.physical_calibration, PedalActuatorCalibration):
            raise TypeError("physical_calibration must be PedalActuatorCalibration or None")

    def to_dict(self) -> dict[str, Any]:
        root: dict[str, Any] = {"schema_version": 1, "demand_map": None, "physical_calibration": None}
        if self.demand_map is not None:
            d = self.demand_map
            root["demand_map"] = {"units": {"acceleration_axis": "m/s^2", "deceleration_axis": "m/s^2", "pedal_output": "1"},
                "acceleration_to_throttle": _table_dict(d.acceleration_to_throttle), "deceleration_to_brake": _table_dict(d.deceleration_to_brake)}
        if self.physical_calibration is not None:
            p = self.physical_calibration
            root["physical_calibration"] = {"units": {"throttle_input": "1", "throttle_effective_request": "1", "brake_input": "1", "brake_effective_request": "1"},
                "throttle_fraction": _table_dict(p.throttle_fraction), "brake_force_fraction": _table_dict(p.brake_force_fraction)}
        return root

    @property
    def sha256(self) -> str:
        data = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(data).hexdigest()


def _table_dict(table: MonotoneTable1D) -> dict[str, Any]:
    return {"axis": list(table.axis), "value": list(table.value)}


def _keys(obj: Any, required: set[str], optional: set[str] = set()) -> dict[str, Any]:
    if not isinstance(obj, dict): raise ValueError("configuration objects must be JSON objects")
    if set(obj) - required - optional or required - set(obj): raise ValueError("unknown or missing configuration keys")
    return obj


def _parse_table(obj: Any, axis_unit: str, value_unit: str) -> MonotoneTable1D:
    _keys(obj, {"axis", "value"})
    return MonotoneTable1D(obj["axis"], obj["value"], axis_unit, value_unit)


def parse_pedal_map_config(payload: Any) -> PedalMapConfiguration:
    _keys(payload, {"schema_version"}, {"demand_map", "physical_calibration"})
    version = payload["schema_version"]
    if isinstance(version, bool) or version != 1: raise ValueError("unsupported schema_version")
    demand = physical = None
    if payload.get("demand_map") is not None:
        d = _keys(payload["demand_map"], {"units", "acceleration_to_throttle", "deceleration_to_brake"})
        units = _keys(d["units"], {"acceleration_axis", "deceleration_axis", "pedal_output"})
        if units != {"acceleration_axis": "m/s^2", "deceleration_axis": "m/s^2", "pedal_output": "1"}: raise ValueError("unsupported demand units")
        demand = LongitudinalDemandMap(_parse_table(d["acceleration_to_throttle"], "m/s^2", "1"), _parse_table(d["deceleration_to_brake"], "m/s^2", "1"))
    if payload.get("physical_calibration") is not None:
        p = _keys(payload["physical_calibration"], {"units", "throttle_fraction", "brake_force_fraction"})
        expected = {"throttle_input": "1", "throttle_effective_request": "1", "brake_input": "1", "brake_effective_request": "1"}
        if _keys(p["units"], set(expected)) != expected: raise ValueError("unsupported physical calibration units")
        physical = PedalActuatorCalibration(_parse_table(p["throttle_fraction"], "1", "1"), _parse_table(p["brake_force_fraction"], "1", "1"))
    return PedalMapConfiguration(demand, physical)


def _reject_constant(value: str) -> None:
    raise ValueError(f"nonfinite JSON constant {value} is not allowed")


def load_pedal_map_config(path: str | Path) -> PedalMapConfiguration:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = json.load(stream, parse_constant=_reject_constant)
    return parse_pedal_map_config(payload)
