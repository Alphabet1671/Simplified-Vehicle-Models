from __future__ import annotations

import unittest

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.drivetrain.controller import PedalControllerParameters


class PedalControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = PedalControllerParameters()

    def test_throttle_maps_percent_to_constant_torque_request_and_caps_delivery(self):
        self.assertEqual(self.controller.map_throttle(0.0, 100.0, 80.0), (0.0, 0.0))
        self.assertEqual(self.controller.map_throttle(50.0, 100.0, 80.0), (50.0, 50.0))
        self.assertEqual(self.controller.map_throttle(100.0, 100.0, 80.0), (100.0, 80.0))

    def test_brake_load_cell_maps_to_regen_and_hydraulic_deadband(self):
        threshold = self.controller.regen_pedal_force_limit
        zero = self.controller.map_brake(0.0, 10.0)
        half = self.controller.map_brake(threshold / 2.0, 10.0)
        full = self.controller.map_brake(threshold, 10.0)
        beyond = self.controller.map_brake(threshold + 25.0, 10.0)
        self.assertEqual(zero.regen_motor_torque_nm, 0.0)
        self.assertEqual(zero.hydraulic_pedal_force_n, 0.0)
        self.assertAlmostEqual(half.regen_motor_torque_nm, -5.0)
        self.assertEqual(half.hydraulic_pedal_force_n, 0.0)
        self.assertAlmostEqual(full.regen_motor_torque_nm, -10.0)
        self.assertEqual(full.hydraulic_pedal_force_n, 0.0)
        self.assertAlmostEqual(beyond.regen_motor_torque_nm, -10.0)
        self.assertEqual(beyond.hydraulic_pedal_force_n, 25.0)

    def test_apps_controller_torque_cap_limits_the_request(self):
        controller = PedalControllerParameters(maximum_drive_motor_torque_nm=10.5)
        self.assertEqual(controller.map_throttle(100.0, 100.0, 20.0), (10.5, 10.5))
        self.assertEqual(controller.map_throttle(100.0, 100.0, 7.0), (10.5, 7.0))

    def test_regen_disabled_routes_all_brake_force_to_hydraulics(self):
        controller = PedalControllerParameters(
            regen_pedal_force_limit=0.0,
            regen_motor_torque_limit=0.0,
            regen_enabled=False,
        )
        command = controller.map_brake(200.0, 0.0)
        self.assertEqual(command.regen_motor_torque_nm, 0.0)
        self.assertEqual(command.hydraulic_pedal_force_n, 200.0)

    def test_regen_torque_is_capped_by_available_motor_envelope(self):
        command = self.controller.map_brake(
            self.controller.regen_pedal_force_limit, 3.25
        )
        self.assertEqual(command.regen_motor_torque_nm, -3.25)
        self.assertEqual(command.hydraulic_pedal_force_n, 0.0)
        self.assertTrue(command.regen_torque_limited)

    def test_real_iem24_drivetrain_owns_regen_off_controller(self):
        vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml"
        ).model
        self.assertEqual(vehicle.controller.maximum_tps_percent, 100.0)
        self.assertEqual(vehicle.controller.regen_pedal_force_limit, 0.0)
        self.assertEqual(vehicle.controller.regen_motor_torque_limit, 0.0)
        self.assertFalse(vehicle.controller.regen_enabled)
        self.assertEqual(vehicle.controller.maximum_drive_motor_torque_nm, 10.5)
        self.assertEqual(vehicle.drivetrain.drivetrain.controller, vehicle.controller)


if __name__ == "__main__":
    unittest.main()
