"""Deterministically generate the canonical motor operating map."""
from __future__ import annotations

import argparse
import hashlib
import math
import os
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

from simplified_vehicle_models.powertrain.power_plant import (  # noqa: E402
    compile_analytical_motor_source,
    evaluate_shaft_power_exact,
)
from simplified_vehicle_models.xml.loader import load_model  # noqa: E402

GENERATOR_VERSION = "motor-lut-v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _maximum_feasible(parameters, speed, voltage, sign, envelope):
    if envelope <= 0.0:
        return 0.0
    def feasible(load):
        result = evaluate_shaft_power_exact(parameters, speed, sign * load * envelope, voltage)
        return result.feasible
    if not feasible(0.0):
        return 0.0
    if feasible(envelope):
        return envelope
    lo, hi = 0.0, envelope
    for _ in range(80):
        mid = (lo + hi) * 0.5
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


def generate(path, voltage_points=9, speed_points=129, load_points=65) -> str:
    """Generate the canonical map and return its SHA-256 digest."""
    path = Path(path)
    xml_path = _ROOT / "models" / "power-plants" / "amk_dd5_100c.xml"
    parameters = compile_analytical_motor_source(load_model(xml_path))
    voltage = np.linspace(448.848, 608.688, voltage_points, dtype=np.float64)
    speed = np.linspace(0.0, 18600.0, speed_points, dtype=np.float64)
    load = np.linspace(-1.0, 1.0, load_points, dtype=np.float64)
    drive_torque = np.empty((voltage_points, speed_points), dtype=np.float64)
    regen_torque = np.empty_like(drive_torque)
    bus_power = np.empty((voltage_points, speed_points, load_points), dtype=np.float64)
    loss_power = np.empty_like(bus_power)
    max_repair = 0.0
    for iv, bus_voltage in enumerate(voltage):
        for ix, motor_speed in enumerate(speed):
            envelope = float(np.interp(
                min(motor_speed, parameters.torque_speed[-1]),
                parameters.torque_speed, parameters.torque_values,
            ))
            drive_torque[iv, ix] = _maximum_feasible(parameters, motor_speed, bus_voltage, 1.0, envelope)
            regen_torque[iv, ix] = _maximum_feasible(parameters, motor_speed, bus_voltage, -1.0, envelope)
            for il, normalized_load in enumerate(load):
                torque_limit = drive_torque[iv, ix] if normalized_load >= 0 else regen_torque[iv, ix]
                result = evaluate_shaft_power_exact(parameters, motor_speed, normalized_load * torque_limit, bus_voltage)
                # Persist signed directional power with an exact zero-load anchor;
                # this is the monotone operating coordinate used by inverse queries.
                bus_power[iv, ix, il] = (0.0 if normalized_load == 0.0 else math.copysign(abs(result.bus_power), normalized_load))
                loss_power[iv, ix, il] = result.loss_power
            for indices, extremum in ((slice(0, load_points // 2 + 1), np.maximum.accumulate), (slice(load_points // 2, None), np.maximum.accumulate)):
                row = bus_power[iv, ix, indices]
                repaired = extremum(row)
                delta = float(np.max(np.abs(repaired - row)))
                if delta > max_repair:
                    max_repair = delta
                if delta > 0.0:
                    bus_power[iv, ix, indices] = repaired
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    np.savez_compressed(temporary, voltage=voltage, speed=speed, load=load,
                        drive_torque=drive_torque, regen_torque=regen_torque,
                        bus_power=bus_power, loss_power=loss_power)
    produced = temporary if temporary.exists() else Path(str(temporary) + ".npz")
    os.replace(produced, path)
    digest = _sha256(path)
    print(f"SHA-256: {digest}")
    print(f"axes: voltage={voltage.shape}, speed={speed.shape}, load={load.shape}")
    print(f"array shapes: drive_torque={drive_torque.shape}, regen_torque={regen_torque.shape}, bus_power={bus_power.shape}, loss_power={loss_power.shape}")
    print(f"maximum monotonic repair: {max_repair:.17g}")
    print(f"generator version: {GENERATOR_VERSION}")
    print(f"generation command: {' '.join(sys.argv)}")
    return digest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("xml_path")
    parser.add_argument("output")
    args = parser.parse_args()
    # The XML argument is intentionally accepted as the canonical interface.
    generate(args.output)
