"""Pedal-input interpretation owned by the drivetrain controller model."""
from __future__ import annotations

from dataclasses import dataclass
import math

from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_positive,
)

CONTROLLER_MODEL_TYPE = "controller-model"
PEDAL_MAP_SUBTYPE = "pedal-map"
_PEDAL_UNITS = {
    "maximum_tps_percent": "%",
    "regen_pedal_force_limit": "N",
    "regen_motor_torque_limit": "N*m",
    "maximum_drive_motor_torque_nm": "N*m",
    "regen_enabled": "1",
}


@dataclass(frozen=True, slots=True)
class BrakePedalCommand:
    """Controller interpretation of a physical brake-pedal force."""

    regen_motor_torque_nm: float  # signed; negative is regenerative braking
    hydraulic_pedal_force_n: float
    load_cell_fraction: float
    regen_torque_limited: bool


@dataclass(frozen=True, slots=True)
class PedalControllerParameters:
    """Map driver pedal inputs into motor and hydraulic requests.

    The 10 N*m load-cell request is per motor at the motor shaft. It is a
    request, not a guarantee; the motor's current speed/voltage envelope caps
    delivered torque.
    """

    maximum_tps_percent: float = 100.0
    regen_pedal_force_limit: float = 30.0 * 9.80665
    regen_motor_torque_limit: float = 10.0
    maximum_drive_motor_torque_nm: float = 1000.0
    regen_enabled: bool = True

    def __post_init__(self) -> None:
        for name in ("maximum_tps_percent", "maximum_drive_motor_torque_nm"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("regen_pedal_force_limit", "regen_motor_torque_limit"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.regen_enabled and (
            self.regen_pedal_force_limit <= 0.0
            or self.regen_motor_torque_limit <= 0.0
        ):
            raise ValueError("enabled regeneration needs positive force and torque ranges")

    @property
    def hydraulic_deadband_n(self) -> float:
        """Return brake-pedal force below which hydraulics stay disengaged."""
        return self.regen_pedal_force_limit if self.regen_enabled else 0.0

    def map_throttle(
        self,
        tps_percent: float,
        constant_torque_nm: float,
        available_motor_torque_nm: float,
    ) -> tuple[float, float]:
        """Return requested and deliverable positive torque per motor (N*m)."""
        if not math.isfinite(tps_percent) or not 0.0 <= tps_percent <= self.maximum_tps_percent:
            raise ValueError("TPS must be finite and within its configured percent range")
        if not math.isfinite(constant_torque_nm) or constant_torque_nm < 0.0:
            raise ValueError("constant torque must be finite and nonnegative")
        if not math.isfinite(available_motor_torque_nm) or available_motor_torque_nm < 0.0:
            raise ValueError("available motor torque must be finite and nonnegative")
        requested = (tps_percent / self.maximum_tps_percent) * min(
            constant_torque_nm, self.maximum_drive_motor_torque_nm
        )
        return requested, min(requested, available_motor_torque_nm)

    def map_brake(
        self, pedal_force_n: float, available_regen_motor_torque_nm: float
    ) -> BrakePedalCommand:
        """Map pedal force to signed per-motor regen request and hydraulic force."""
        if not math.isfinite(pedal_force_n) or pedal_force_n < 0.0:
            raise ValueError("brake pedal force must be finite and nonnegative")
        if not math.isfinite(available_regen_motor_torque_nm) or available_regen_motor_torque_nm < 0.0:
            raise ValueError("available regen torque must be finite and nonnegative")
        if not self.regen_enabled:
            return BrakePedalCommand(0.0, pedal_force_n, 0.0, False)
        fraction = min(pedal_force_n / self.regen_pedal_force_limit, 1.0)
        requested = fraction * self.regen_motor_torque_limit
        delivered = min(requested, available_regen_motor_torque_nm)
        return BrakePedalCommand(
            regen_motor_torque_nm=-delivered,
            hydraulic_pedal_force_n=max(pedal_force_n - self.regen_pedal_force_limit, 0.0),
            load_cell_fraction=fraction,
            regen_torque_limited=delivered + 1.0e-12 < requested,
        )


def compile_pedal_controller(model: Model) -> PedalControllerParameters:
    """Compile the closed-schema drivetrain pedal controller component."""
    require_model_type(model, CONTROLLER_MODEL_TYPE, PEDAL_MAP_SUBTYPE)
    values = closed_parameters(model, _PEDAL_UNITS)
    require_no_references(model)
    require_positive(
        model, values,
        "maximum_tps_percent",
        "maximum_drive_motor_torque_nm",
    )
    if values["regen_pedal_force_limit"] < 0.0 or values["regen_motor_torque_limit"] < 0.0:
        raise ValueError(f"{model.path}: regeneration ranges must be nonnegative")
    enabled = values.pop("regen_enabled")
    if enabled not in (0.0, 1.0):
        raise ValueError(f"{model.path}: regen_enabled must be 0 or 1")
    return PedalControllerParameters(**values, regen_enabled=bool(enabled))


register_component(
    CONTROLLER_MODEL_TYPE, PEDAL_MAP_SUBTYPE,
    compile_function=compile_pedal_controller,
)
