from pathlib import Path
import tempfile
import unittest

from simplified_vehicle_models.powertrain.assembly import (
    compile_drivetrain_assembly,
    compile_powertrain_assembly,
    compile_powertrain_drivetrain,
    maximum_regen_force,
    solve_shaft_torque_fixed_point,
)
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.powertrain.power_plant import compile_electric_motor
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]


class PowertrainAssemblyTests(unittest.TestCase):
    def test_canonical_nested_assemblies_compile(self):
        vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
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
