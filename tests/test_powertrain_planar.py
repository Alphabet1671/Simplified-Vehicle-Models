import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.drivetrain.fixed_split import (
    DRIVE_FRONT_FRACTION,
    wheel_shares,
)
from simplified_vehicle_models.powertrain.assembly import (
    compile_powertrain_drivetrain,
    maximum_drive_force,
    maximum_regen_force,
    resolve_braking,
)
from simplified_vehicle_models.powertrain.energy_storage import (
    StorageState,
    evaluate_storage,
)
from simplified_vehicle_models.powertrain.motor_envelope import (
    MotorEnvelopeParameters,
    bus_power,
    compile_motor_envelope,
    motor_speed_limit,
    motor_torque_bounds,
    motor_torque_limit,
    shaft_power,
    total_bus_power,
    wheel_speed_limit,
    wheel_torque_limit,
)
from simplified_vehicle_models.powertrain.wheel_drive import (
    BATTERY_CONSTRAINT,
    MOTOR_CONSTRAINT,
    evaluate_drive_limits,
)
from simplified_vehicle_models.xml.loader import load_model


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
MOTOR_PATH = REPOSITORY_ROOT / "models/power-plants/placeholder_2024_motor.xml"

RPM_TO_RAD = 2.0 * math.pi / 60.0


class MotorEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.motor = compile_motor_envelope(load_model(MOTOR_PATH))

    def test_the_envelope_compiles(self) -> None:
        motor = self.motor

        self.assertIsInstance(motor, MotorEnvelopeParameters)
        self.assertEqual(motor.unit_count, 4)
        self.assertGreater(motor.gear_ratio, 1.0)
        self.assertGreater(motor.peak_power, motor.continuous_power)
        self.assertGreater(motor.peak_motor_torque, motor.continuous_motor_torque)
        # The XML declares revolutions per minute, the object uses radians.
        self.assertGreater(motor.maximum_motor_speed, 1800.0)
        self.assertLess(motor.maximum_motor_speed, 2200.0)

    def test_the_base_speed_follows_the_power_and_torque(self) -> None:
        motor = self.motor

        self.assertAlmostEqual(
            motor.base_motor_speed,
            motor.peak_power / motor.peak_motor_torque,
            places=9,
        )

    def test_the_torque_bound_holds_at_low_speed(self) -> None:
        limit = motor_torque_limit(self.motor, 100.0, 600.0)

        self.assertAlmostEqual(float(limit), self.motor.peak_motor_torque, places=9)

    def test_the_power_bound_holds_at_high_speed(self) -> None:
        # Stay below the speed limit, which is only 1.25 times the base speed.
        speed = 1.2 * self.motor.base_motor_speed

        limit = motor_torque_limit(self.motor, speed, 600.0)

        self.assertAlmostEqual(
            float(limit), self.motor.peak_power / speed, places=6
        )
        # The product stays at the power rating.
        self.assertAlmostEqual(
            float(limit) * speed, self.motor.peak_power, places=3
        )

    def test_a_lower_voltage_reaches_the_power_bound_sooner(self) -> None:
        speed = 1.2 * self.motor.base_motor_speed
        nominal = float(motor_torque_limit(self.motor, speed, 600.0))
        reduced = float(motor_torque_limit(self.motor, speed, 450.0))

        self.assertLess(reduced, nominal)

    def test_the_speed_bound_stops_the_motor(self) -> None:
        limit = motor_speed_limit(self.motor, 600.0)
        above = float(motor_torque_limit(self.motor, limit * 1.05, 600.0))
        below = float(motor_torque_limit(self.motor, limit * 0.95, 600.0))

        self.assertEqual(above, 0.0)
        self.assertGreater(below, 0.0)

    def test_the_loaded_voltage_speed_bound_stops_the_motor(self) -> None:
        voltage = 450.0
        nominal_limit = motor_speed_limit(self.motor, 600.0)
        loaded_limit = motor_speed_limit(self.motor, voltage)

        self.assertEqual(
            float(motor_torque_limit(self.motor, loaded_limit * 1.01, voltage)),
            0.0,
        )
        self.assertGreater(
            float(motor_torque_limit(self.motor, loaded_limit * 0.99, voltage)),
            0.0,
        )
        self.assertGreater(nominal_limit, loaded_limit)

    def test_the_wheel_speed_limit_is_the_motor_limit_over_the_gear(self) -> None:
        self.assertAlmostEqual(
            wheel_speed_limit(self.motor, 600.0),
            motor_speed_limit(self.motor, 600.0) / self.motor.gear_ratio,
            places=9,
        )

    def test_the_bounds_are_returned_separately_for_a_nonlinear_program(self) -> None:
        speed = 0.5 * self.motor.base_motor_speed

        torque, power, speed_bound = motor_torque_bounds(self.motor, speed, 600.0)

        self.assertAlmostEqual(float(torque), self.motor.peak_motor_torque, places=9)
        self.assertGreater(float(power), self.motor.peak_motor_torque)
        self.assertAlmostEqual(
            float(speed_bound), self.motor.maximum_motor_speed, places=9
        )

    def test_a_lower_voltage_lowers_the_speed_limit(self) -> None:
        self.assertLess(
            motor_speed_limit(self.motor, 450.0),
            motor_speed_limit(self.motor, 600.0),
        )

    def test_the_wheel_limit_applies_the_gearing_and_efficiency(self) -> None:
        zero = wheel_torque_limit(self.motor, np.zeros(1), 600.0)

        expected = (
            self.motor.peak_motor_torque
            * self.motor.gear_ratio
            * self.motor.motor_efficiency
        )
        self.assertAlmostEqual(float(zero[0]), expected, places=6)

    def test_the_bus_power_exceeds_the_shaft_power(self) -> None:
        wheel_speed = np.asarray([50.0])
        wheel_torque = np.asarray([100.0])
        mechanical = shaft_power(self.motor, wheel_speed, wheel_torque)
        electrical = bus_power(self.motor, wheel_speed, wheel_torque)

        self.assertGreater(float(electrical[0]), float(mechanical[0]))
        self.assertAlmostEqual(
            float(electrical[0]),
            100.0 * 50.0 / (
                self.motor.motor_efficiency * self.motor.inverter_efficiency
            ),
            places=7,
        )

    def test_regeneration_returns_less_than_it_takes(self) -> None:
        wheel_speed = np.asarray([50.0])
        motoring = bus_power(self.motor, wheel_speed, np.asarray([100.0]))
        regen = bus_power(self.motor, wheel_speed, np.asarray([-100.0]))

        self.assertAlmostEqual(float(regen[0]), -float(motoring[0]) * self.motor.drive_efficiency ** 2, places=6)

    def test_the_total_bus_power_sums_the_units(self) -> None:
        speeds = np.full(4, 50.0)
        torques = np.full(4, 100.0)

        total = total_bus_power(self.motor, speeds, torques)

        self.assertAlmostEqual(
            total, 4.0 * float(bus_power(self.motor, np.asarray([50.0]), np.asarray([100.0]))[0]), places=6
        )

    def test_rejects_bad_parameters(self) -> None:
        model = load_model(MOTOR_PATH)
        for name, value in (
            ("gear_ratio", 0.0),
            ("peak_motor_torque", -1.0),
            ("peak_power", 0.0),
            ("maximum_motor_speed", 0.0),
            ("inverter_efficiency", 1.5),
            ("motor_efficiency", 0.0),
        ):
            parameters = dict(model.parameters)
            parameters[name] = value
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, name):
                compile_motor_envelope(replace(model, parameters=parameters))
        # The continuous rating must not exceed the peak rating.
        parameters = dict(model.parameters)
        parameters["continuous_power"] = parameters["peak_power"] * 2.0
        with self.assertRaisesRegex(ValueError, "continuous power"):
            compile_motor_envelope(replace(model, parameters=parameters))


class DriveLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        compiled = compile_formulation(FORMULATION_PATH)
        cls.vehicle = compiled.model
        cls.motor = cls.vehicle.motor
        cls.storage = cls.vehicle.storage
        cls.state = StorageState(cls.storage.initial_state_of_charge)
        cls.voltage = evaluate_storage(cls.storage, cls.state).bus_voltage

    def _limits(self, speed, state=None, storage=None):
        wheel = np.full(4, speed / 0.192)
        return evaluate_drive_limits(
            self.motor,
            wheel,
            self.voltage,
            self.storage if storage is None else storage,
            self.state if state is None else state,
        )

    def test_low_speed_is_torque_limited(self) -> None:
        result = self._limits(5.0)

        self.assertEqual(result.active_constraint, MOTOR_CONSTRAINT)
        self.assertAlmostEqual(
            float(result.wheel_torque_limits[0]),
            self.motor.peak_motor_torque
            * self.motor.gear_ratio
            * self.motor.motor_efficiency,
            places=6,
        )

    def test_high_speed_is_power_limited(self) -> None:
        motor = replace(self.motor, peak_power=10_000.0)
        result = evaluate_drive_limits(
            motor,
            np.full(4, 30.0 / 0.192),
            self.voltage,
            self.storage,
            self.state,
        )

        self.assertLess(
            float(result.wheel_torque_limits[0]),
            motor.peak_motor_torque * motor.gear_ratio * motor.motor_efficiency,
        )

    def test_above_the_speed_limit_the_drive_is_zero(self) -> None:
        result = self._limits(60.0)

        np.testing.assert_allclose(result.wheel_torque_limits, 0.0)
        self.assertEqual(result.total_torque_limit, 0.0)

    def test_the_motor_limit_is_the_same_on_every_wheel(self) -> None:
        result = self._limits(10.0)

        np.testing.assert_allclose(
            result.wheel_torque_limits, result.wheel_torque_limits[0]
        )

    def test_a_weak_pack_bounds_the_drive(self) -> None:
        weak = replace(self.storage, discharge_current_limit=20.0)

        result = self._limits(20.0, storage=weak)

        self.assertEqual(result.active_constraint, BATTERY_CONSTRAINT)
        self.assertTrue(result.is_battery_limited)
        self.assertLess(result.battery_scale, 1.0)
        self.assertLessEqual(
            result.bus_power_at_limit,
            evaluate_storage(
                weak, StorageState(weak.initial_state_of_charge)
            ).discharge_power_limit + 1.0e-6,
        )

    def test_the_battery_scale_preserves_the_split(self) -> None:
        weak = replace(self.storage, discharge_current_limit=20.0)
        result = self._limits(20.0, storage=weak)

        ratio = result.wheel_torque_limits / result.motor_torque_limits
        np.testing.assert_allclose(ratio, result.battery_scale, rtol=1.0e-12)

    def test_without_a_storage_only_the_motor_bounds(self) -> None:
        result = evaluate_drive_limits(
            self.motor, np.full(4, 50.0), self.voltage
        )

        self.assertEqual(result.active_constraint, MOTOR_CONSTRAINT)
        self.assertEqual(result.bus_power_limit, float("inf"))

    def test_rejects_a_bad_wheel_speed_vector(self) -> None:
        with self.assertRaises(ValueError):
            evaluate_drive_limits(self.motor, np.zeros(3), self.voltage)
        with self.assertRaises(ValueError):
            evaluate_drive_limits(
                self.motor, np.full(4, np.nan), self.voltage
            )

    def test_the_assembly_carries_the_envelope_and_the_storage(self) -> None:
        self.assertIsInstance(self.vehicle.motor, MotorEnvelopeParameters)
        self.assertEqual(self.vehicle.storage.series_count, 144)
        self.assertEqual(self.vehicle.storage.parallel_count, 1)

    def test_point_mass_drive_limit_accepts_the_smooth_motor_envelope(self) -> None:
        vehicle_model = load_model(
            REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
        )
        runtime = compile_powertrain_drivetrain(vehicle_model)
        state = StorageState(
            runtime.powertrain.energy_storage.initial_state_of_charge
        )

        drive = maximum_drive_force(
            runtime.powertrain,
            runtime.drivetrain,
            speed=12.0,
            state=state,
            effective_radius=0.204,
        )

        self.assertGreater(drive.maximum_drive_force, 0.0)
        self.assertTrue(drive.converged)
        self.assertLessEqual(
            drive.bus_power_at_limit,
            evaluate_storage(
                runtime.powertrain.energy_storage, state
            ).discharge_power_limit + 1.0e-6,
        )

    def test_point_mass_drive_limit_uses_rear_motor_shares(self) -> None:
        vehicle_model = load_model(
            REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
        )
        runtime = compile_powertrain_drivetrain(vehicle_model)
        state = StorageState(
            runtime.powertrain.energy_storage.initial_state_of_charge
        )
        unit_count = runtime.powertrain.power_plant.unit_count
        shares = np.full(unit_count, 1.0 / unit_count)
        limit = maximum_drive_force(
            runtime.powertrain,
            runtime.drivetrain,
            speed=5.0,
            state=state,
            effective_radius=0.204,
            wheel_shares=shares,
        )

        self.assertEqual(runtime.drivetrain.transmission.driven_wheels, ("rl", "rr"))
        self.assertAlmostEqual(float(np.sum(shares)), 1.0)
        self.assertGreater(limit.maximum_drive_force, 0.0)

    def test_smooth_motor_braking_blends_regen_and_friction_under_charge_limit(self) -> None:
        runtime = compile_powertrain_drivetrain(
            load_model(REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml")
        )
        state = StorageState(
            runtime.powertrain.energy_storage.initial_state_of_charge
        )
        shares = np.full(self.motor.unit_count, 1.0 / self.motor.unit_count)

        limit = maximum_regen_force(
            runtime.powertrain,
            runtime.drivetrain,
            speed=10.0,
            state=state,
            effective_radius=0.204,
            wheel_shares=shares,
        )
        braking = resolve_braking(
            runtime.powertrain,
            runtime.drivetrain,
            speed=10.0,
            state=state,
            braking_demand=5000.0,
            tire_force_capacity=5000.0,
            effective_radius=0.204,
            wheel_shares=shares,
        )

        self.assertGreater(limit.force_limit, 0.0)
        self.assertAlmostEqual(braking.regen_force_used, limit.force_limit)
        self.assertGreater(braking.friction_force_used, 0.0)
        self.assertLessEqual(
            braking.recovered_bus_power,
            abs(limit.charge_power_limit) + 1.0e-6,
        )

    def test_point_mass_energy_trace_records_smooth_regeneration_as_negative(self) -> None:
        from simplified_vehicle_models.solvers.forward_backward import _energy_trace

        runtime = compile_powertrain_drivetrain(
            load_model(
                REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
            )
        )
        soc = runtime.powertrain.energy_storage.initial_state_of_charge
        vehicle = SimpleNamespace(
            drivetrain=runtime,
            tire=SimpleNamespace(effective_radius=0.204),
        )
        lap = SimpleNamespace(
            speed=np.array([10.0, 10.0]),
            elapsed_time=np.array([0.0, 0.1]),
            longitudinal_forces=np.full((1, 4), -100.0),
        )
        track = SimpleNamespace(segment_length=np.array([1.0]))

        bus, shaft, _voltage, _used, recovered, next_soc = _energy_trace(
            vehicle, track, lap, np.array([soc, soc]), regen_enabled=True
        )

        self.assertLess(bus[0], 0.0)
        self.assertLess(shaft[0], 0.0)
        self.assertGreater(recovered, 0.0)
        self.assertGreater(next_soc[-1], soc)

    def test_point_mass_energy_trace_uses_smooth_motor_efficiency(self) -> None:
        from simplified_vehicle_models.solvers.forward_backward import _energy_trace

        runtime = compile_powertrain_drivetrain(
            load_model(
                REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
            )
        )
        soc = runtime.powertrain.energy_storage.initial_state_of_charge
        vehicle = SimpleNamespace(
            drivetrain=runtime,
            tire=SimpleNamespace(effective_radius=0.204),
        )
        lap = SimpleNamespace(
            speed=np.array([10.0, 10.0]),
            elapsed_time=np.array([0.0, 0.1]),
            longitudinal_forces=np.full((1, 4), 1000.0),
        )
        track = SimpleNamespace(segment_length=np.array([1.0]))

        bus, shaft, voltage, _used, _recovered, next_soc = _energy_trace(
            vehicle, track, lap, np.array([soc, soc]), regen_enabled=False
        )

        self.assertGreater(bus[0], 0.0)
        self.assertGreater(shaft[0], 0.0)
        self.assertTrue(np.isfinite(voltage[0]))
        self.assertLess(next_soc[-1], soc)


if __name__ == "__main__":
    unittest.main()
