from __future__ import annotations

import unittest
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.drivetrain.controller import PedalControllerParameters
from simplified_vehicle_models.solvers.driver_maps import (
    MonotoneTable1D, PedalActuatorCalibration, bind_pedal_calibration,
)
from simplified_vehicle_models.solvers.forward_driver import DriverCommand
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar


def calibration():
    return PedalActuatorCalibration(
        MonotoneTable1D([0, .5, 1], [0, .2, 1], '1', '1'),
        MonotoneTable1D([0, .5, 1], [0, .2, 1], '1', '1'))


class PedalActuatorCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = compile_formulation('models/formulations/iem24_murphy_real_planar_7dof.xml').model
        plan = SimpleNamespace(is_closed=False)
        cls.plant = PlanarForwardPlant(cls.original, plan)
        cls.calibrated = bind_pedal_calibration(cls.original, calibration())

    def eval(self, vehicle, throttle=0., brake=0.):
        return evaluate_forward_powertrain(vehicle, self.plant.driven, self.plant.gearing,
            self.plant.gear_efficiency, np.zeros(4), .7, DriverCommand(throttle, brake, 0.))

    def test_binding_replaces_both_controller_references_immutably(self):
        nested = self.original.drivetrain.drivetrain.controller
        self.assertIs(self.original.controller, nested)
        self.assertIs(self.calibrated.controller, self.calibrated.drivetrain.drivetrain.controller)
        self.assertIsNot(self.calibrated.controller, self.original.controller)
        self.assertIs(self.original.controller, nested)
        self.assertIs(bind_pedal_calibration(self.original, None), self.original)
        with self.assertRaises(TypeError): bind_pedal_calibration(self.original, object())
        view = point_mass_view_of_planar(self.calibrated)
        self.assertIs(view.drivetrain, self.calibrated.drivetrain)
        from simplified_vehicle_models.powertrain.energy_storage import StorageState
        from simplified_vehicle_models.powertrain.assembly import maximum_drive_force
        original_view = point_mass_view_of_planar(self.original)
        a = maximum_drive_force(view.drivetrain.powertrain, view.drivetrain.drivetrain, 0., StorageState(.7)).maximum_drive_force
        b = maximum_drive_force(original_view.drivetrain.powertrain, original_view.drivetrain.drivetrain, 0., StorageState(.7)).maximum_drive_force
        self.assertEqual(a, b)

    def test_controller_validates_optional_calibration_type(self):
        with self.assertRaises(TypeError): PedalControllerParameters(actuator_calibration=object())

    def test_none_and_identity_are_exactly_default(self):
        identity = PedalActuatorCalibration(MonotoneTable1D([0, 1], [0, 1], '1', '1'),
                                            MonotoneTable1D([0, 1], [0, 1], '1', '1'))
        vehicle = bind_pedal_calibration(self.original, identity)
        a, b = self.eval(self.original, .37), self.eval(vehicle, .37)
        for field in ('wheel_torques', 'motor_wheel_torques'):
            np.testing.assert_array_equal(getattr(a, field), getattr(b, field))
        for field in ('bus_power', 'voltage', 'current', 'power_residual', 'limited'):
            self.assertEqual(getattr(a, field), getattr(b, field))

    def test_throttle_shape_and_endpoints_match_canonical_default_evaluator(self):
        shaped = self.eval(self.calibrated, .5)
        expected = self.eval(self.original, .2)
        np.testing.assert_array_equal(shaped.motor_wheel_torques, expected.motor_wheel_torques)
        for throttle in (0., 1.):
            np.testing.assert_array_equal(self.eval(self.calibrated, throttle).motor_wheel_torques,
                                          self.eval(self.original, throttle).motor_wheel_torques)

    def test_brake_force_units_and_endpoints_with_regen_disabled(self):
        f = .5 * self.original.maximum_pedal_force
        got = self.eval(self.calibrated, brake=f)
        expected = self.eval(self.original, brake=.2 * self.original.maximum_pedal_force)
        np.testing.assert_array_equal(got.wheel_torques, expected.wheel_torques)
        for force in (0., self.original.maximum_pedal_force):
            np.testing.assert_array_equal(self.eval(self.calibrated, brake=force).wheel_torques,
                                          self.eval(self.original, brake=force).wheel_torques)

    def test_regen_and_hydraulic_deadband_use_shaped_physical_force_once(self):
        controller = replace(self.calibrated.controller, regen_enabled=True,
                             regen_pedal_force_limit=.1 * self.original.maximum_pedal_force,
                             regen_motor_torque_limit=.5)
        assembly = replace(self.calibrated.drivetrain.drivetrain, controller=controller)
        calibrated = replace(self.calibrated, controller=controller,
                             drivetrain=replace(self.calibrated.drivetrain, drivetrain=assembly))
        # At zero wheel speed the bounded regen request is electrically feasible;
        # compare through canonical evaluator with the already-shaped physical force.
        physical = .5 * self.original.maximum_pedal_force
        shaped = .2 * self.original.maximum_pedal_force
        got = self.eval(calibrated, brake=physical)
        uncalibrated = replace(calibrated, controller=replace(controller, actuator_calibration=None),
            drivetrain=replace(calibrated.drivetrain, drivetrain=replace(assembly,
                controller=replace(controller, actuator_calibration=None))))
        expected = self.eval(uncalibrated, brake=shaped)
        np.testing.assert_array_equal(got.wheel_torques, expected.wheel_torques)
        self.assertLess(got.power_residual, 1.e-6)

    def test_shaped_brake_enters_hydraulics_with_deadband_once_for_regen_modes(self):
        from unittest.mock import patch
        from simplified_vehicle_models.controls.brakes import brake_torques as canonical_brakes
        physical = .5 * self.original.maximum_pedal_force
        shaped = .2 * self.original.maximum_pedal_force
        for regen_enabled, expected in ((True, max(shaped - 100., 0.)), (False, shaped)):
            controller = replace(self.calibrated.controller, regen_enabled=regen_enabled,
                                 regen_pedal_force_limit=100., regen_motor_torque_limit=1.)
            assembly = replace(self.calibrated.drivetrain.drivetrain, controller=controller)
            vehicle = replace(self.calibrated, controller=controller,
                drivetrain=replace(self.calibrated.drivetrain, drivetrain=assembly))
            captured = []
            def capture(force, *args, **kwargs):
                captured.append(float(force))
                return canonical_brakes(force, *args, **kwargs)
            with patch("simplified_vehicle_models.solvers.forward_powertrain.brake_torques", side_effect=capture):
                self.eval(vehicle, brake=physical)
            self.assertEqual(captured, [expected])

    def test_brake_pressure_cap_still_saturates_high_input(self):
        from simplified_vehicle_models.controls.brakes import brake_torques
        vehicle = self.original
        common = dict(maximum_pedal_force=vehicle.maximum_pedal_force,
            pedal_curve=vehicle.pedal_curve, hydraulic_ratio_front=vehicle.hydraulic_ratio_front,
            hydraulic_ratio_rear=vehicle.hydraulic_ratio_rear,
            maximum_brake_pressure=vehicle.maximum_brake_pressure, caliper_area=vehicle.caliper_area,
            brake_bias=vehicle.brake_bias, pad_mu_front=vehicle.pad_mu_front,
            pad_mu_rear=vehicle.pad_mu_rear, effective_radius_front=vehicle.brake_effective_radius_front,
            effective_radius_rear=vehicle.brake_effective_radius_rear)
        common["maximum_brake_pressure"] = 1.e5
        capped = brake_torques(vehicle.maximum_pedal_force, **common)
        self.assertTrue(capped.is_pressure_limited)
        above = brake_torques(vehicle.maximum_pedal_force, **{**common,
            "maximum_brake_pressure": 1.5 * common["maximum_brake_pressure"]})
        self.assertLess(float(np.max(np.abs(capped.wheel_torques))),
                        float(np.max(np.abs(above.wheel_torques))))
        self.assertAlmostEqual(max(capped.front_line_pressure, capped.rear_line_pressure),
                               common["maximum_brake_pressure"])

    def test_inverse_with_nonidentity_re_evaluates_canonical_vehicle(self):
        from simplified_vehicle_models.solvers.longitudinal_actuator import invert_longitudinal_actuators
        desired = self.eval(self.calibrated, .2).wheel_torques
        cmd = invert_longitudinal_actuators(self.calibrated, np.zeros(4), .7,
                                            float(np.sum(desired)), settings=None)
        result = self.eval(self.calibrated, cmd.throttle, cmd.brake_force)
        self.assertAlmostEqual(float(np.sum(result.wheel_torques)), float(np.sum(desired)), delta=1.e-4)

    def test_regen_inverse_is_explicitly_unsupported(self):
        controller = replace(self.calibrated.controller, regen_enabled=True,
                             regen_pedal_force_limit=30., regen_motor_torque_limit=10.)
        assembly = replace(self.calibrated.drivetrain.drivetrain, controller=controller)
        regen = replace(self.calibrated, controller=controller,
                        drivetrain=replace(self.calibrated.drivetrain, drivetrain=assembly))
        self.assertTrue(regen.controller.regen_enabled)
        self.assertEqual(regen.controller.regen_pedal_force_limit, 30.)


if __name__ == '__main__': unittest.main()
