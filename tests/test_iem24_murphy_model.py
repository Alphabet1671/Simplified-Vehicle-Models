import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.point_mass import _point_mass_drive_shares
from simplified_vehicle_models.powertrain.assembly import maximum_drive_force
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_point_mass.xml"
PLANAR_FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


class Iem24MurphyModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_point_mass_vehicle(FORMULATION)

    def test_real_measurements_and_control_configuration_compile(self):
        vehicle = self.vehicle
        self.assertEqual(vehicle.total_mass, 287.0)
        self.assertEqual(vehicle.front_mass_fraction, 0.45)
        self.assertEqual(vehicle.track_width, 1.240)
        self.assertEqual(vehicle.cg_height, 0.3)
        self.assertEqual(vehicle.lateral_load_transfer_distribution, 0.42)
        self.assertEqual(vehicle.aero.downforce_area, 3.5)
        self.assertEqual(vehicle.aero.drag_area, 1.55)
        self.assertEqual(vehicle.aero.aero_balance, 0.5)
        self.assertEqual(vehicle.drivetrain.motor_count, 2)
        controller = vehicle.drivetrain.drivetrain.controller
        self.assertEqual(controller.maximum_drive_motor_torque_nm, 10.5)
        self.assertFalse(controller.regen_enabled)
        self.assertEqual(controller.hydraulic_deadband_n, 0.0)

    def test_planar_formulation_uses_corrected_geometry(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation

        vehicle = compile_formulation(PLANAR_FORMULATION).model
        self.assertEqual(vehicle.front_mass_fraction, 0.45)
        self.assertEqual(vehicle.track_width, 1.240)

    def test_drive_share_is_rear_only_and_motor_request_is_capped(self):
        shares = _point_mass_drive_shares(self.vehicle)
        np.testing.assert_array_equal(shares, [0.0, 0.0, 0.5, 0.5])
        runtime = self.vehicle.drivetrain
        result = maximum_drive_force(
            runtime.powertrain,
            runtime.drivetrain,
            0.0,
            StorageState(runtime.powertrain.energy_storage.initial_state_of_charge),
            effective_radius=self.vehicle.tire.effective_radius,
        )
        maximum_wheel_torque = (
            2.0 * 10.5 * runtime.drivetrain.transmission.gear_ratio
            * runtime.drivetrain.transmission.gear_efficiency
        )
        self.assertLessEqual(result.maximum_wheel_torque, maximum_wheel_torque + 1e-8)


if __name__ == "__main__":
    unittest.main()
