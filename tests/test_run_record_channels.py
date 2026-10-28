"""Check the run-record sink: additive hook, same-evaluation channels, availability."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import rotate_tire_forces_to_body
from simplified_vehicle_models.results.plant_channels import RunRecordSink, road_wheel_angles
from simplified_vehicle_models.results.run_record import load_run_record
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, PreviewDriver, SpeedPlan, StepSample, solve_forward,
)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track

PLANAR_XML = "models/formulations/iem24_murphy_real_planar_7dof.xml"
VERTICAL_XML = "models/formulations/iem24_murphy_real_14dof_simple.xml"
TRACK = "models/tracks/mis_ev_endurance_2024_linear.xml"
DURATION = 0.6


def build(plant_class, xml):
    vehicle = compile_formulation(xml).model
    control = getattr(vehicle, "planar", vehicle)
    plan = SpeedPlan(np.array([0., 2000.]), np.array([8., 8.]), np.zeros(2))
    plant = plant_class(vehicle, plan)
    driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
                           control.steering_maximum_rate, control.maximum_pedal_force)
    return plant, driver


class Case(unittest.TestCase):
    plant_class, xml = PlanarForwardPlant, PLANAR_XML

    @classmethod
    def setUpClass(cls):
        cls.path = load_track(TRACK).path_coordinates

    def run_case(self, sink_factory=None, **options):
        plant, driver = build(self.plant_class, self.xml)
        sink = sink_factory(plant, self.path) if sink_factory else None
        result = solve_forward(plant, driver, plant.initial_state(8.), duration=DURATION,
                               maximum_step=0.02, step_sink=sink, **options)
        return plant, result, sink


class StepSinkAdditiveTest(Case):
    def test_sink_sees_exactly_the_accepted_steps_in_order(self):
        samples = []
        _, result, _ = self.run_case(lambda plant, path: samples.append)
        self.assertEqual(len(samples), len(result.time))
        self.assertEqual([s.index for s in samples], list(range(len(result.time))))
        np.testing.assert_array_equal([s.time for s in samples], result.time)
        np.testing.assert_array_equal(np.array([s.state for s in samples]), result.state.T)
        self.assertIsInstance(samples[0], StepSample)
        self.assertEqual(samples[0].rates.shape, samples[0].state.shape)

    def test_recording_does_not_change_the_result_or_add_evaluations(self):
        with tempfile.TemporaryDirectory() as directory:
            plain_plant, plain, _ = self.run_case()
            sink_plant, recorded, sink = self.run_case(
                lambda plant, path: RunRecordSink(plant, path, Path(directory) / "r.npz"))
            sink.close()
        np.testing.assert_array_equal(plain.time, recorded.time)
        np.testing.assert_array_equal(plain.state, recorded.state)
        self.assertEqual(plain_plant.evaluations, sink_plant.evaluations)
        self.assertEqual(plain.evaluations, recorded.evaluations)

    def test_realized_command_matches_the_driver_chain(self):
        samples = []
        self.run_case(lambda plant, path: samples.append)
        for sample in samples:
            self.assertIsInstance(sample.command, DriverCommand)
            self.assertTrue(0. <= sample.command.throttle <= 1.)
            self.assertTrue(sample.command.throttle * sample.command.brake_force < 1.e-9)


class PlanarChannelTest(Case):
    def test_each_channel_equals_the_same_evaluation_field(self):
        checks = []

        def factory(plant, path):
            sink = RunRecordSink(plant, path, "unused.npz")

            def check(sample):
                values = sink.channels(sample)
                evaluation = plant.last_evaluation
                np.testing.assert_array_equal(values["tire-vertical-force"], evaluation.normal_loads)
                np.testing.assert_array_equal(values["tire-longitudinal-force"],
                                              evaluation.longitudinal_forces)
                np.testing.assert_array_equal(values["tire-lateral-force"], evaluation.lateral_forces)
                np.testing.assert_array_equal(values["tire-aligning-moment"],
                                              evaluation.aligning_moments)
                np.testing.assert_array_equal(values["tire-slip-angle"], evaluation.slip_angles)
                # Tire-frame verification: rotating by the road-wheel angle gives the body forces.
                bx, by = rotate_tire_forces_to_body(
                    np.asarray(values["tire-longitudinal-force"]),
                    np.asarray(values["tire-lateral-force"]),
                    np.asarray(values["wheel-steering-angle"]))
                np.testing.assert_allclose(bx, evaluation.body_longitudinal_forces,
                                           rtol=1e-9, atol=1e-9)
                np.testing.assert_allclose(by, evaluation.body_lateral_forces, rtol=1e-9, atol=1e-9)
                np.testing.assert_allclose(values["tire-force-in-body-x"],
                                           evaluation.body_longitudinal_forces, rtol=1e-9, atol=1e-9)
                checks.append(1)
            return check

        _, result, _ = self.run_case(factory)
        self.assertEqual(len(checks), len(result.time))

    def test_availability_planar_has_no_attitude_or_quasi_static(self):
        with tempfile.TemporaryDirectory() as directory:
            _, _, sink = self.run_case(
                lambda plant, path: RunRecordSink(plant, path, Path(directory) / "p.npz"))
            record = load_run_record(sink.close(), verify_sources=False)
        for absent in ("roll", "pitch", "heave", "roll-rate", "acceleration-vertical",
                       "roll-quasi-static", "pitch-quasi-static"):
            self.assertFalse(record.has_channel(absent), absent)
        for present in ("tire-vertical-force", "tire-slip-angle", "position-x", "throttle", "soc"):
            self.assertTrue(record.has_channel(present), present)
        self.assertTrue(np.all(np.isfinite(record.channel)))
        self.assertTrue(record.manifest.coarse_for_120_fps)
        self.assertEqual(record.manifest.state_names, tuple(PlanarForwardPlant.state_names)
                         if isinstance(PlanarForwardPlant.state_names, tuple)
                         else record.manifest.state_names)

    def test_quasi_static_pair_is_opt_in_labelled_and_correctly_signed(self):
        plant, _ = build(self.plant_class, self.xml)
        names = list(plant.state_names)
        command = DriverCommand(0., 0., 0.)
        state = plant.initial_state(8.)
        plant.evaluate(0., state, command).solve()
        off = RunRecordSink(plant, self.path, "unused.npz")
        on = RunRecordSink(plant, self.path, "unused.npz", quasi_static_attitude=True)
        for ax_rate, ay_rate in ((2., 3.), (-2., -3.)):
            rates = np.zeros_like(state)
            rates[names.index("v_x")], rates[names.index("v_y")] = ax_rate, ay_rate
            sample = StepSample(0, 0., state, rates, command, None)
            self.assertNotIn("roll-quasi-static", off.channels(sample))
            values = on.channels(sample)
            # Right-side-down roll is positive, so a right turn (ay > 0) rolls left.
            self.assertEqual(np.sign(values["roll-quasi-static"][0]), -np.sign(ay_rate))
            # Pitch is positive nose up, so acceleration (ax > 0) squats the nose up.
            self.assertEqual(np.sign(values["pitch-quasi-static"][0]), np.sign(ax_rate))
        with tempfile.TemporaryDirectory() as directory:
            _, _, sink = self.run_case(lambda plant, path: RunRecordSink(
                plant, path, Path(directory) / "q.npz", quasi_static_attitude=True))
            record = load_run_record(sink.close(), verify_sources=False)
        self.assertTrue(record.has_channel("pitch-quasi-static"))
        self.assertTrue(any("APPROXIMATE" in note for note in record.manifest.notes))

    def test_body_slip_angle_sign_and_value(self):
        plant, _ = build(self.plant_class, self.xml)
        sink = RunRecordSink(plant, self.path, "unused.npz")
        state = plant.initial_state(8.)
        command = DriverCommand(0., 0., 0.)
        plant.evaluate(0., state, command).solve()
        state[plant.state_names.index("v_x")] = 10.
        state[plant.state_names.index("v_y")] = 1.
        sample = StepSample(0, 0., state, np.zeros_like(state), command, None)
        values = sink.channels(sample)
        self.assertAlmostEqual(float(values["body-slip-angle"][0]), np.arctan2(1., 10.))
        self.assertAlmostEqual(float(values["speed"][0]), np.hypot(10., 1.))

    def test_world_pose_follows_the_path_and_n_is_to_the_right(self):
        plant, _ = build(self.plant_class, self.xml)
        sink = RunRecordSink(plant, self.path, "unused.npz")
        command = DriverCommand(0., 0., 0.)
        state = plant.initial_state(8.)
        names = list(plant.state_names)
        state[names.index("s")], state[names.index("n")] = 100., 0.
        plant.evaluate(0., state, command).solve()
        centre = sink.channels(StepSample(0, 0., state, np.zeros_like(state), command, None))
        x, y, heading, _ = evaluate_path(self.path, 100.)
        np.testing.assert_allclose([centre["position-x"][0], centre["position-y"][0]], [x, y])
        self.assertAlmostEqual(float(centre["heading-world"][0]), float(heading)
                               + state[names.index("heading_error")])
        state[names.index("n")] = 2.
        shifted = sink.channels(StepSample(0, 0., state, np.zeros_like(state), command, None))
        offset = np.array([shifted["position-x"][0] - x, shifted["position-y"][0] - y])
        right = np.array([-np.sin(heading), np.cos(heading)])  # positive n: to the right of travel
        np.testing.assert_allclose(offset, 2. * right, atol=1e-9)
        # Wheel centres surround the CG at the half-track and wheelbase spacing.
        fl, fr = (np.array([shifted["wheel-position-x"][k], shifted["wheel-position-y"][k]])
                  for k in (0, 1))
        self.assertAlmostEqual(float(np.linalg.norm(fr - fl)), plant.vehicle.track_width, places=9)


class VerticalChannelTest(Case):
    plant_class, xml = VerticalForwardPlant, VERTICAL_XML

    def test_road_wheel_angles_include_static_toe_and_asymmetric_jounce_steer(self):
        plant, _ = build(self.plant_class, self.xml)
        state = plant.initial_state(8.)
        names = list(plant.state_names)
        state[names.index("steering")] = 0.12
        jounce = np.array([0.004, -0.003, 0.002, -0.001])
        for wheel, value in zip(("fl", "fr", "rl", "rr"), jounce):
            state[names.index(f"jounce_{wheel}")] = value
        offsets = plant.vehicle.suspension.kinematics.toe(jounce)
        expected = np.array([0.12, 0.12, 0., 0.]) + offsets
        np.testing.assert_allclose(road_wheel_angles(plant, state), expected,
                                   rtol=0., atol=1.e-14)
        self.assertGreater(np.max(np.abs(offsets)), 0.)
        self.assertFalse(np.allclose(offsets,
                                    plant.vehicle.suspension.kinematics.toe(np.zeros(4)),
                                    rtol=0., atol=1.e-14))

    def test_attitude_channels_are_state_values_and_loads_match_the_evaluation(self):
        checks = []

        def factory(plant, path):
            sink = RunRecordSink(plant, path, "unused.npz")

            def check(sample):
                values = sink.channels(sample)
                names = list(plant.state_names)
                evaluation = plant.last_evaluation
                np.testing.assert_array_equal(values["tire-vertical-force"], evaluation.normal_loads)
                np.testing.assert_array_equal(values["tire-lateral-force"],
                                              evaluation.tire_forces[:, 1])
                self.assertEqual(values["roll"][0], sample.state[names.index("roll")])
                self.assertEqual(values["pitch"][0], sample.state[names.index("pitch")])
                self.assertEqual(values["heave"][0], sample.state[names.index("heave")])
                self.assertEqual(values["acceleration-vertical"][0],
                                 sample.rates[names.index("v_z")])
                # The step's own body acceleration is the plant's evaluation value.
                np.testing.assert_allclose(
                    [values["acceleration-longitudinal"][0], values["acceleration-lateral"][0]],
                    evaluation.body_accelerations, rtol=1e-9, atol=1e-9)
                checks.append(1)
            return check

        _, result, _ = self.run_case(factory)
        self.assertEqual(len(checks), len(result.time))

    def test_availability_vertical_has_attitude_and_no_quasi_static(self):
        with tempfile.TemporaryDirectory() as directory:
            _, _, sink = self.run_case(lambda plant, path: RunRecordSink(
                plant, path, Path(directory) / "v.npz", quasi_static_attitude=True))
            record = load_run_record(sink.close(), verify_sources=False)
        for present in ("roll", "pitch", "heave", "roll-rate", "pitch-rate", "acceleration-vertical"):
            self.assertTrue(record.has_channel(present), present)
        # A real attitude state wins; the approximation is never recorded beside it.
        self.assertFalse(record.has_channel("roll-quasi-static"))
        self.assertFalse(record.has_channel("tire-slip-angle"))  # not exposed by this evaluation
        self.assertEqual(record.channel_values("tire-vertical-force").shape[1], 4)


if __name__ == "__main__":
    unittest.main()
