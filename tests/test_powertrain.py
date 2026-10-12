from pathlib import Path
from dataclasses import replace
import math
import tempfile
import unittest
import hashlib

import numpy as np

from simplified_vehicle_models.powertrain.assembly import (
    compile_drivetrain_assembly,
    compile_powertrain_assembly,
    compile_powertrain_drivetrain,
    FullDrivetrainRuntime,
    maximum_drive_force,
    maximum_drive_force_lumped,
    maximum_regen_force,
    resolve_braking,
    solve_shaft_torque_fixed_point,
)
from simplified_vehicle_models.drivetrain.lumped import compile_lumped_drivetrain
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.xml.model import DataReference
from simplified_vehicle_models.powertrain.power_plant import (
    compile_electric_motor,
    evaluate_shaft_power,
    _validate_operating_map,
    _load_operating_map,
)
from simplified_vehicle_models.powertrain.energy_storage import (
    StorageState,
    evaluate_storage,
)
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]


class OperatingMapValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with np.load(ROOT / "models/data/amk_dd5_100c_operating_map.npz", allow_pickle=False) as archive:
            cls.data = {name: archive[name] for name in archive.files}

    def bad(self, mutate):
        data = {name: value.copy() for name, value in self.data.items()}
        mutate(data)
        with self.assertRaises(ValueError):
            _validate_operating_map(data)

    def test_rejects_missing_arrays_and_wrong_shapes(self):
        self.bad(lambda d: d.pop("loss_power"))
        self.bad(lambda d: d.__setitem__("drive_torque", d["drive_torque"][:-1]))

    def test_rejects_bad_axes_values_and_negative_loss(self):
        self.bad(lambda d: d["speed"].__setitem__(1, d["speed"][0]))
        self.bad(lambda d: d["loss_power"].__setitem__((0, 0, 0), -1.0))
        self.bad(lambda d: d["bus_power"].__setitem__((0, 0, 32), np.nan))

    def test_rejects_direction_sign_and_monotonicity(self):
        self.bad(lambda d: d["bus_power"].__setitem__((0, 1, 64), -1.0))
        self.bad(lambda d: (d["bus_power"].__setitem__((0, 1, 0), 1.0), d["regen_feasible"].__setitem__((0, 1, 0), True)))

    def test_rejects_raw_negative_row_nonmonotonicity_even_when_infeasible(self):
        def mutate(data):
            row = data["bus_power"][0, 0]
            row[1] = row[0] - 1.0

        self.bad(mutate)

    def test_rejects_regen_feasible_holes_and_reentry(self):
        def mutate(data):
            feasible = data["regen_feasible"][0, 64]
            feasible[10] = False
            feasible[11] = True

        self.bad(mutate)

    def test_rejects_digest_mismatch(self):
        model = load_model(ROOT / "models/power-plants/amk_dd5_100c.xml")
        reference = model.data[0]
        bad_reference = DataReference(reference.role, reference.path, reference.format, "0" * 64)
        bad_model = replace(model, data=(bad_reference,))
        with self.assertRaisesRegex(ValueError, "digest mismatch"):
            _load_operating_map(bad_model)

    def test_canonical_shape_and_digest(self):
        loaded = _validate_operating_map(self.data)
        self.assertEqual(loaded.bus_power.shape, (9, 129, 65))
        digest = hashlib.sha256((ROOT / "models/data/amk_dd5_100c_operating_map.npz").read_bytes()).hexdigest()
        self.assertEqual(digest, "fa670ac4abd4bb81a8be923c493a54cc9ea424021301a750718de05cb057fbee")


class PowertrainAssemblyTests(unittest.TestCase):
    def test_canonical_nested_assemblies_compile(self):
        vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
        self.assertIsInstance(vehicle.drivetrain, FullDrivetrainRuntime)
        self.assertEqual(vehicle.drivetrain.powertrain.power_plant.unit_count, 4)
        self.assertEqual(vehicle.drivetrain.motor_count, 4)
        self.assertEqual(vehicle.drivetrain.gear_ratio, 10.9)
        self.assertEqual(vehicle.drivetrain.maximum_motor_speed, 18600.0)
        self.assertEqual(vehicle.manifest.component("powertrain").subtype, "electric")
        self.assertEqual(vehicle.manifest.component("drivetrain").subtype, "electric-four-wheel")
        self.assertEqual(vehicle.manifest.component("powertrain_drivetrain").subtype, "four-wheel")

    def test_nested_assemblies_are_recorded(self):
        vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
        record = vehicle.manifest.capability("drive-force-limit")
        self.assertEqual(record.source_capability, "powertrain-and-drivetrain")
        self.assertEqual(record.adapter_names, ("reduce_powertrain_and_drivetrain",))
        self.assertEqual(record.component_role, "powertrain_drivetrain")

    def test_regen_force_report_has_both_limits(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 10.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertGreater(result.force_from_torque, 0.0)
        self.assertGreater(result.force_from_charge_power, 0.0)
        # Wheel power is reduced by both directional losses before charging.
        motor = powertrain.power_plant
        transmission = drivetrain.transmission
        battery_power = (
            result.force_limit * 10.0
            * transmission.gear_efficiency * motor.inverter_efficiency
        )
        self.assertLessEqual(battery_power, abs(result.charge_power_limit) + 1.0e-6)
        self.assertGreaterEqual(result.iterations, 1)
        self.assertLessEqual(result.residual, 1.0e-9)
        self.assertTrue(result.converged)

    def test_fixed_point_reports_explicit_nonconvergence(self):
        motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))
        result = solve_shaft_torque_fixed_point(
            motor, 1000.0, 1000.0, 600.0, tolerance=1.0e-30, max_iterations=1
        )
        self.assertFalse(result.converged)
        self.assertEqual(result.iterations, 1)
        self.assertGreater(result.residual, result.torque * 0.0)

    def test_assembly_rejects_motor_wheel_mismatch(self):
        motor_path = ROOT / "models/power-plants/amk_dd5_100c.xml"
        drivetrain_path = ROOT / "models/drivetrains/murphy_2024_drivetrain.xml"
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            motor = motor_path.read_text().replace('value="4"', 'value="3"', 1)
            (folder / "motor.xml").write_text(motor)
            (folder.parent / "data").mkdir(exist_ok=True)
            (folder.parent / "data" / "amk_dd5_100c_operating_map.npz").write_bytes(
                (ROOT / "models/data/amk_dd5_100c_operating_map.npz").read_bytes()
            )
            (folder / "powertrain.xml").write_text(
                '<model schema-version="1.0" id="test-powertrain" '
                'type="powertrain-assembly" subtype="electric">'
                '<components><component role="power-plant" ref="motor.xml"/>'
                f'<component role="energy-storage" ref="{(ROOT / "models/energy-storages/murphy_2024_battery.xml").as_posix()}"/>'
                '</components></model>'
            )
            vehicle = folder / "vehicle.xml"
            vehicle.write_text(
                '<model schema-version="1.0" id="test-vehicle" '
                'type="vehicle-assembly" subtype="four-wheel">'
                f'<components><component role="powertrain" ref="powertrain.xml"/>'
                f'<component role="drivetrain" ref="{drivetrain_path.as_posix()}"/>'
                '</components></model>'
            )
            with self.assertRaisesRegex(ValueError, "does not match driven wheels"):
                compile_powertrain_drivetrain(load_model(vehicle))

    def test_drive_force_has_positive_bus_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        for speed in (0.0, 30.0):
            result = maximum_drive_force(
                powertrain,
                drivetrain,
                speed,
                StorageState(0.8),
                effective_radius=0.193675,
            )
            self.assertGreater(result.maximum_drive_force, 0.0)
            self.assertGreater(result.bus_power_at_limit, 0.0)
            self.assertLess(result.storage_state_rate, 0.0)

    def test_drive_force_operating_point_has_positive_evaluated_bus_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        radius = 0.193675
        result = maximum_drive_force(
            powertrain, drivetrain, 10.0, StorageState(0.8), effective_radius=radius
        )
        motor = powertrain.power_plant
        transmission = drivetrain.transmission
        motor_speed = transmission.motor_speed(10.0 / radius) * 60.0 / (2.0 * math.pi)
        motor_torque = result.maximum_wheel_torque / (
            motor.unit_count * transmission.gear_ratio * transmission.gear_efficiency
        )
        open_limits = evaluate_storage(powertrain.energy_storage, StorageState(0.8))
        self.assertLess(result.bus_voltage, open_limits.bus_voltage)
        evaluated = evaluate_shaft_power(
            motor, motor_speed, motor_torque, result.bus_voltage
        )
        self.assertTrue(evaluated.feasible)
        self.assertGreaterEqual(evaluated.bus_power, 0.0)
        self.assertAlmostEqual(
            evaluated.bus_power * motor.unit_count,
            result.bus_power_at_limit,
            places=6,
        )

    def test_full_and_lumped_assemblies_share_operating_point(self):
        full = compile_powertrain_drivetrain(
            load_model(ROOT / "models/vehicles/murphy_2024.xml")
        )
        lumped = compile_lumped_drivetrain(
            load_model(ROOT / "models/drivetrains/murphy_4wd.xml")
        )
        for name in (
            "motor_count",
            "gear_ratio",
            "peak_motor_torque",
            "power_limit",
            "efficiency",
            "maximum_motor_speed",
        ):
            self.assertEqual(getattr(full, name), getattr(lumped, name))
        speed = 10.0
        state = StorageState(0.8)
        radius = 0.193675
        full_limit = maximum_drive_force(
            full.powertrain, full.drivetrain, speed, state, radius
        )
        legacy_limit = maximum_drive_force_lumped(
            full.powertrain, lumped, speed, state, radius
        )
        # The lumped path omits map losses, so allow its reviewed 2 percent
        # force approximation while requiring matching active behavior.
        self.assertAlmostEqual(
            full_limit.maximum_drive_force,
            legacy_limit.maximum_drive_force,
            delta=250.0,
        )
        self.assertAlmostEqual(
            full_limit.bus_power_at_limit,
            legacy_limit.bus_power_at_limit,
            delta=40000.0,
        )
        self.assertEqual(full_limit.active_constraint, legacy_limit.active_constraint)
        self.assertAlmostEqual(
            full_limit.storage_current, legacy_limit.storage_current, delta=70.0
        )
        self.assertAlmostEqual(
            full_limit.storage_state_rate, legacy_limit.storage_state_rate, delta=0.002
        )
        changed = replace(lumped, peak_motor_torque=lumped.peak_motor_torque * 0.5)
        changed_limit = maximum_drive_force_lumped(
            full.powertrain, changed, speed, state, radius
        )
        self.assertNotEqual(changed_limit.maximum_drive_force, legacy_limit.maximum_drive_force)
        self.assertEqual(
            full_limit.maximum_drive_force,
            maximum_drive_force(full.powertrain, full.drivetrain, speed, state, radius).maximum_drive_force,
        )

    def test_regen_charge_power_constraint_is_active(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 30.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "storage_power")
        self.assertLessEqual(result.force_limit, result.force_from_charge_power)

    def test_resolve_braking_obeys_charge_power(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = resolve_braking(
            powertrain, drivetrain, 30.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "storage_power")
        self.assertLessEqual(result.force_limit, result.force_from_charge_power)

    def test_resolve_braking_allocates_friction_after_regen(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = resolve_braking(
            powertrain,
            drivetrain,
            30.0,
            StorageState(0.8),
            braking_demand=3000.0,
            tire_force_capacity=2500.0,
            effective_radius=0.193675,
        )
        self.assertEqual(result.total_force_limit, 2500.0)
        self.assertGreater(result.regen_force_used, 0.0)
        self.assertGreater(result.friction_force_used, 0.0)
        self.assertAlmostEqual(
            result.regen_force_used + result.friction_force_used,
            result.total_force_limit,
        )
        self.assertGreater(result.recovered_bus_power, 0.0)
        self.assertEqual(result.total_force, result.total_force_limit)
        self.assertEqual(result.recovered_power, result.recovered_bus_power)

    def test_braking_overspeed_and_standstill_have_no_recovery(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        standstill = resolve_braking(
            powertrain,
            drivetrain,
            0.0,
            StorageState(0.8),
            braking_demand=1000.0,
            effective_radius=0.193675,
        )
        self.assertEqual(standstill.recovered_bus_power, 0.0)
        overspeed = resolve_braking(
            powertrain,
            drivetrain,
            400.0,
            StorageState(0.8),
            braking_demand=1000.0,
            effective_radius=0.193675,
        )
        self.assertEqual(overspeed.regen_force_used, 0.0)
        self.assertEqual(overspeed.recovered_bus_power, 0.0)
        self.assertEqual(overspeed.active_constraint, "motor_speed")

    def test_regen_rejects_motor_overspeed(self):
        powertrain = compile_powertrain_assembly(
            load_model(ROOT / "models/powertrains/murphy_2024.xml")
        )
        drivetrain = compile_drivetrain_assembly(
            load_model(ROOT / "models/drivetrains/murphy_2024_drivetrain.xml")
        )
        result = maximum_regen_force(
            powertrain, drivetrain, 400.0, StorageState(0.8), effective_radius=0.193675
        )
        self.assertEqual(result.active_constraint, "motor_speed")
        self.assertEqual(result.force_limit, 0.0)

    def test_fixed_point_reports_convergence_and_residual(self):
        motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))
        result = solve_shaft_torque_fixed_point(motor, 1000.0, 1000.0, 600.0)
        self.assertTrue(result.converged)
        self.assertLessEqual(result.residual, 1.0e-9)
        self.assertGreater(result.iterations, 0)


if __name__ == "__main__":
    unittest.main()
