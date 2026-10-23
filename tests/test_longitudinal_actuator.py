import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import DriverCommand
from simplified_vehicle_models.solvers.longitudinal_actuator import (
    ActuatorInversionSettings, invert_longitudinal_actuators,
)


class FakeOutput:
    def __init__(self, values, limited=False):
        self.wheel_torques = np.asarray(values, dtype=float)
        self.power_residual = 0.
        self.limited = limited


def fake_vehicle(regen=False):
    return SimpleNamespace(
        controller=SimpleNamespace(regen_enabled=regen),
        drivetrain=SimpleNamespace(drivetrain=SimpleNamespace(
            transmission=SimpleNamespace(driven_wheels=("rl", "rr"), gear_efficiency=.9))),
        motor=SimpleNamespace(unit_count=2, gear_ratio=4.), maximum_pedal_force=1000.)


def analytic(*args):
    vehicle, driven, gearing, efficiency, speeds, soc, command = args
    del vehicle, driven, gearing, efficiency, speeds, soc
    value = 100. * command.throttle - 2. * max(command.brake_force - 100., 0.)
    return FakeOutput((0., 0., value / 2., value / 2.))


class LongitudinalActuatorTests(unittest.TestCase):
    def test_positive_negative_deadband_and_coast(self):
        vehicle = fake_vehicle()
        for requested in (0., 50., -100., -1800.):
            result = invert_longitudinal_actuators(vehicle, np.ones(4), .8, requested, evaluator=analytic)
            self.assertLessEqual(abs(result.torque_error_nm), 1.e-5)
            self.assertFalse(result.throttle and result.brake_force)
        self.assertAlmostEqual(invert_longitudinal_actuators(vehicle, np.ones(4), .8, -100., evaluator=analytic).brake_force, 150., places=5)

    def test_saturation_and_actual_endpoint_ranges(self):
        vehicle = fake_vehicle()
        result = invert_longitudinal_actuators(vehicle, np.ones(4), .8, 200., evaluator=analytic)
        self.assertTrue(result.saturated)
        self.assertEqual(result.throttle, 1.)
        self.assertEqual(result.achieved_total_wheel_torque_nm, 100.)
        self.assertEqual((result.lower_reachable_torque_nm, result.upper_reachable_torque_nm), (-1800., 100.))
        self.assertEqual(result.requested_total_wheel_torque_nm, 200.)

    def test_plateau_and_nonmonotonic_map(self):
        vehicle = fake_vehicle()
        def plateau(*args):
            command = args[-1]
            return FakeOutput((0., 0., min(command.throttle, .5) * 100., 0.))
        result = invert_longitudinal_actuators(vehicle, np.zeros(4), .5, 50., evaluator=plateau)
        self.assertAlmostEqual(result.achieved_total_wheel_torque_nm, 50.)
        def bad(*args):
            pedal = args[-1].throttle
            return FakeOutput((0., 0., (100. * pedal if pedal < .5 else 60. - 100. * (pedal-.5)), 0.))
        with self.assertRaises(RuntimeError):
            invert_longitudinal_actuators(vehicle, np.zeros(4), .5, 30., evaluator=bad)

    def test_max_iterations_input_validation_and_copy(self):
        vehicle = fake_vehicle()
        speeds = np.ones(4)
        with self.assertRaises(RuntimeError):
            invert_longitudinal_actuators(vehicle, speeds, .5, 20., evaluator=analytic,
                settings=ActuatorInversionSettings(maximum_iterations=1))
        np.testing.assert_array_equal(speeds, np.ones(4))
        for bad in (True, "1", float("inf")):
            with self.assertRaises(ValueError):
                invert_longitudinal_actuators(vehicle, speeds, .5, bad, evaluator=analytic)
        with self.assertRaises(ValueError):
            ActuatorInversionSettings(maximum_iterations=True)

    def test_regen_explicitly_unsupported(self):
        with self.assertRaises(NotImplementedError):
            invert_longitudinal_actuators(fake_vehicle(True), np.ones(4), .5, 0., evaluator=analytic)

    def test_compiled_iem24_forward_closure_roundtrip(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
        plant = PlanarForwardPlant(vehicle, SimpleNamespace(is_closed=False))
        speeds = np.full(4, 30.)
        for requested in (100., -100.):
            result = invert_longitudinal_actuators(vehicle, speeds, vehicle.storage.initial_state_of_charge, requested)
            command = DriverCommand(result.throttle, result.brake_force, 0.)
            actual = evaluate_forward_powertrain(vehicle, plant.driven, plant.gearing,
                plant.gear_efficiency, speeds, vehicle.storage.initial_state_of_charge, command)
            self.assertAlmostEqual(float(np.sum(actual.wheel_torques)), requested, delta=1.e-5)
            self.assertLess(actual.power_residual, 1.e-6)


if __name__ == "__main__":
    unittest.main()
