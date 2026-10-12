from pathlib import Path
import unittest

from simplified_vehicle_models.powertrain.assembly import (
    compile_drivetrain_assembly,
    compile_powertrain_assembly,
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
        self.assertEqual(vehicle.manifest.capability("drive-force-limit").source_capability, "drive-force-limit")
        self.assertEqual(vehicle.manifest.capability("drive-force-limit").adapter_names, ())

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

    def test_fixed_point_reports_convergence_and_residual(self):
        motor = compile_electric_motor(load_model(ROOT / "models/power-plants/amk_dd5_100c.xml"))
        result = solve_shaft_torque_fixed_point(motor, 1000.0, 1000.0, 600.0)
        self.assertTrue(result.converged)
        self.assertLessEqual(result.residual, 1.0e-9)
        self.assertGreater(result.iterations, 0)


if __name__ == "__main__":
    unittest.main()
