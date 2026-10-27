import json
import unittest
from unittest import mock
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import DriverCommand, DriverObservation
from simplified_vehicle_models.solvers.physical_longitudinal_mapper import (
    PhysicalLongitudinalMapper, PhysicalMapperSettings,
)


NAMES = ("s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
         "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering")


def fake_vehicle(regen=False):
    return SimpleNamespace(
        controller=SimpleNamespace(regen_enabled=regen),
        wheel_inertia=0.2,
        steering_maximum_rate=0.7,
        maximum_pedal_force=1000.,
        tire=object(), total_mass=500., wheelbase=2.5, track_width=1.5,
        cg_height=.3, front_mass_fraction=.5,
        lateral_load_transfer_distribution=.5, front_brake_share=.6,
        aero=object(), drivetrain=object(),
    )


def equilibrium(vehicle, speed, curvature, acceleration):
    del vehicle, speed, curvature
    # Deliberately infeasible: this is a requested force allocation, not a gate.
    return SimpleNamespace(normal_loads=np.ones(4) * 1000.,
                           longitudinal_forces=np.ones(4) * (2. + acceleration),
                           is_feasible=False)


def inverter(vehicle, wheel_speed, soc, torque, *, settings):
    del vehicle, settings
    inverter.last = (np.array(wheel_speed), soc, torque)
    throttle = min(max(torque / 1000., 0.), 1.)
    brake = min(max(-torque, 0.), 1000.)
    return SimpleNamespace(throttle=throttle, brake_force=brake,
                           torque_error_nm=0.2, saturated=throttle in (0., 1.) or brake in (0., 1000.),
                           evaluations=3)


class PhysicalLongitudinalMapperTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = fake_vehicle()
        self.state = np.array([3., .1, .02, 10., .3, .1, 30., 31., 32., 33., .8, .04])
        self.observation = DriverObservation(3., .1, .02, 10., .1, .04, .3, 1.)
        self.request = DriverCommand(.2, 123., .12)

    def mapper(self, **settings):
        with mock.patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()):
            return PhysicalLongitudinalMapper(
                self.vehicle, SimpleNamespace(sample=lambda s: (12., .01, 1.)),
                state_names=NAMES, settings=PhysicalMapperSettings(**settings),
                equilibrium_evaluator=equilibrium, actuator_inverter=inverter)

    def test_force_radius_inertia_mapping_and_metadata_reordering(self):
        with mock.patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()), mock.patch(
                "simplified_vehicle_models.tires.unitire_simple.loaded_radius",
                return_value=np.array([.3, .31, .32, .33])) as radius:
            mapper = self.mapper()
            command = mapper(0., self.state, self.observation, self.request)
        desired = 1. + 2. * (12. - 10.)
        expected = np.dot(np.ones(4) * (2. + desired), [.3, .31, .32, .33])
        expected += sum(.2 * desired / r for r in [.3, .31, .32, .33])
        self.assertAlmostEqual(inverter.last[2], expected)
        np.testing.assert_array_equal(inverter.last[0], self.state[6:10])
        self.assertEqual(inverter.last[1], .8)
        self.assertEqual(command.steering_rate, self.request.steering_rate)
        self.assertEqual(radius.call_count, 1)
        diag = mapper.snapshot_diagnostics()
        self.assertEqual(diag["infeasible_quasi_steady_demands"], 1)
        self.assertFalse(diag["accepted"])
        self.assertFalse(diag["U3_qualified"])
        json.dumps(diag)
        diag["callback_queries"] = -1
        self.assertEqual(mapper.snapshot_diagnostics()["callback_queries"], 1)

    def test_disabled_inertia_governor_and_permuted_names(self):
        state = self.state.copy()
        permuted = tuple(reversed(NAMES))
        state = state[::-1].copy()
        with mock.patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()), mock.patch(
                "simplified_vehicle_models.tires.unitire_simple.loaded_radius",
                return_value=np.ones(4) * .3):
            mapper = PhysicalLongitudinalMapper(
                self.vehicle, SimpleNamespace(sample=lambda s: (10., .0, 0.)),
                state_names=permuted,
                settings=PhysicalMapperSettings(include_wheel_inertia=False),
                equilibrium_evaluator=equilibrium, actuator_inverter=inverter)
            obs = DriverObservation(3., .1, .02, 10., .1, .04, .3, .2)
            cmd = mapper(1., state, obs, self.request)
        self.assertEqual(cmd.steering_rate, .12)
        self.assertEqual(inverter.last[0].tolist(), [30., 31., 32., 33.])
        self.assertEqual(mapper.snapshot_diagnostics()["brake_governor_limited_queries"], 0)

    def test_governor_counts_reduced_brake_even_when_fully_released(self):
        def brake_inverse(*args, **kwargs):
            return SimpleNamespace(throttle=0., brake_force=100., torque_error_nm=0.,
                                   saturated=False, evaluations=1)
        with mock.patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()), mock.patch(
                "simplified_vehicle_models.tires.unitire_simple.loaded_radius", return_value=np.ones(4) * .3):
            mapper = PhysicalLongitudinalMapper(
                self.vehicle, SimpleNamespace(sample=lambda s: (10., 0., 0.)), state_names=NAMES,
                settings=PhysicalMapperSettings(include_wheel_inertia=False),
                equilibrium_evaluator=equilibrium, actuator_inverter=brake_inverse)
            for margin in (0., .425, .85):
                obs = DriverObservation(3., .1, .02, 10., .1, .04, .3, margin)
                command = mapper(0., self.state, obs, self.request)
                if margin == 0.:
                    self.assertEqual(command.brake_force, 0.)
                elif margin == .425:
                    self.assertEqual(command.brake_force, 50.)
                else:
                    self.assertEqual(command.brake_force, 100.)
        self.assertEqual(mapper.snapshot_diagnostics()["brake_governor_limited_queries"], 2)

    def test_rejects_malformed_inverter_results(self):
        bad_results = (
            SimpleNamespace(throttle=float("nan"), brake_force=0., torque_error_nm=0., saturated=False, evaluations=1),
            SimpleNamespace(throttle=.2, brake_force=10., torque_error_nm=0., saturated=False, evaluations=1),
            SimpleNamespace(throttle=0., brake_force=0., torque_error_nm=float("inf"), saturated=False, evaluations=1),
            SimpleNamespace(throttle=0., brake_force=0., torque_error_nm=0., saturated=1, evaluations=1),
            SimpleNamespace(throttle=0., brake_force=0., torque_error_nm=0., saturated=False, evaluations=True),
        )
        for result in bad_results:
            with self.subTest(result=result), mock.patch(
                    "simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar", return_value=object()), mock.patch(
                    "simplified_vehicle_models.tires.unitire_simple.loaded_radius", return_value=np.ones(4) * .3):
                mapper = PhysicalLongitudinalMapper(
                    self.vehicle, SimpleNamespace(sample=lambda s: (10., 0., 0.)), state_names=NAMES,
                    settings=PhysicalMapperSettings(include_wheel_inertia=False),
                    equilibrium_evaluator=equilibrium,
                    actuator_inverter=lambda *a, **k: result)
                with self.assertRaises(ValueError):
                    mapper(0., self.state, self.observation, self.request)

    def test_compiled_iem24_real_drag_hold_actuator_closure(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan

        vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
        reference = SpeedPlan(np.array([0., 100.]), np.array([10., 10.]),
                              np.array([0., 0.]), is_closed=False)
        plant = PlanarForwardPlant(vehicle, reference)
        state = plant.initial_state(10.)
        obs = plant.observe(state)
        request = DriverCommand(.4, 0., 0.)
        mapper = PhysicalLongitudinalMapper(vehicle, reference, state_names=plant.state_names)
        command = mapper(0., state, obs, request)
        self.assertGreater(command.throttle, 0.)
        self.assertEqual(command.steering_rate, request.steering_rate)
        powertrain = evaluate_forward_powertrain(vehicle, plant.driven, plant.gearing,
            plant.gear_efficiency, state[6:10], state[10], command)
        diag = mapper.snapshot_diagnostics()
        self.assertLessEqual(diag["max_abs_actuator_torque_error_nm"], 1.e-4)
        self.assertLess(powertrain.power_residual, 1.e-6)
        self.assertFalse(diag["accepted"])
        self.assertFalse(diag["U3_qualified"])
        json.dumps(diag)

    def test_validation_guards_and_regen(self):
        for bad in (True, float("nan"), 0., "2"):
            with self.assertRaises(ValueError):
                PhysicalMapperSettings(speed_gain=bad)
        with self.assertRaises(NotImplementedError):
            PhysicalLongitudinalMapper(fake_vehicle(True), object(), state_names=NAMES,
                                       equilibrium_evaluator=equilibrium, actuator_inverter=inverter)
        with self.assertRaises(ValueError):
            self.mapper()(0., self.state[:-1], self.observation, self.request)


if __name__ == "__main__":
    unittest.main()
