"""Check the animation frame source (and, below it, the panels)."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.results.run_record import RunRecorder, load_run_record
from tools.anim.resample import FrameSource, MissingChannelError

STATES = ("s", "v_x", "omega_fl", "omega_fr", "omega_rl", "omega_rr")
CONTROLS = ("throttle", "brake-force")
PLANAR_CHANNELS = ("speed", "acceleration-longitudinal", "acceleration-lateral", "steering",
                   "tire-vertical-force", "tire-longitudinal-force", "tire-lateral-force",
                   "tire-aligning-moment")


def make_record(directory, *, end=1.0, step=0.004, extra_channels=(), model="planar-7dof",
                invalid_after=None, maximum_step=None, name="record.npz"):
    """A record with an exactly known motion: speed(t) = 10 + 20 t, omega = speed / 0.3."""
    channels = PLANAR_CHANNELS + tuple(extra_channels)
    recorder = RunRecorder(Path(directory) / name, state_names=STATES, control_names=CONTROLS,
                           channel_names=channels, precision="float64")
    times = np.arange(0.0, end + 1e-12, step)
    for index, t in enumerate(times):
        v = 10.0 + 20.0 * t
        omega = v / 0.3
        state = [10.0 * t + 10.0 * t * t, v, omega, omega, omega, omega]
        rate = [v, 20.0, 20.0 / 0.3, 20.0 / 0.3, 20.0 / 0.3, 20.0 / 0.3]
        values = {
            "speed": [v], "acceleration-longitudinal": [20.0],
            "acceleration-lateral": [-3.0 * t], "steering": [0.1 * t],
            "tire-vertical-force": [1000 + t, 1100 + t, 1200 + t, 1300 + t],
            "tire-longitudinal-force": [100.0 * t] * 4, "tire-lateral-force": [400.0 * t] * 4,
            "tire-aligning-moment": [5.0 * t] * 4,
            "roll": [0.01 * t], "pitch": [0.02 * t], "roll-quasi-static": [0.001 * t],
            "pitch-quasi-static": [0.002 * t],
        }
        values = {key: value for key, value in values.items() if key in channels}
        if invalid_after is not None and t > invalid_after:
            values["steering"] = None  # recorded invalid
        recorder.append(t, index, state, rate, [0.5, 10.0 * t], values)
    integrator = {} if maximum_step is None else {"maximum_step": maximum_step}
    return load_run_record(recorder.close(
        {"model": {"model": model}, "integrator": integrator}, created_utc="2026-10-16T00:00:00Z"))


class FrameGridTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def source(self, **keywords):
        options = {key: keywords.pop(key) for key in ("end", "step", "extra_channels", "model",
                                                     "invalid_after", "maximum_step")
                   if key in keywords}
        return FrameSource(make_record(self.directory.name, **options), 120.0, **keywords)

    def test_frame_count_and_last_time_follow_the_record_span(self):
        for end in (1.0, 0.95, 2.37):
            source = self.source(end=end)
            t_end = float(source.record.time[-1])  # the last recorded step, not `end`
            self.assertEqual(source.count, int(np.floor(t_end * 120 + 1e-9)) + 1, end)
            self.assertLessEqual(source.times[-1], t_end + 1e-12)
            self.assertAlmostEqual(source.times[0], 0.0)
            np.testing.assert_allclose(np.diff(source.times)[:-1], 1 / 120, atol=1e-12)

    def test_source_step_is_nondecreasing_and_never_after_the_frame(self):
        source = self.source(step=0.0173)
        self.assertTrue(np.all(np.diff(source.source_step) >= 0))
        recorded = source.record.time[source.source_step]
        self.assertTrue(np.all(recorded <= source.times + 1e-12))

    def test_state_is_interpolated_exactly_for_a_cubic(self):
        source = self.source(step=0.05)
        t = source.times[37]
        np.testing.assert_allclose(source.state_at(37)[:2], [10.0 * t + 10.0 * t * t, 10 + 20 * t],
                                   atol=1e-12)
        self.assertAlmostEqual(source.state_value("v_x", 37), 10 + 20 * t, places=12)
        self.assertIsNone(source.state_value("roll", 37))

    def test_channel_at_holds_the_enclosing_step_and_returns_width(self):
        source = self.source(step=0.1)
        index = 30  # t = 0.25 -> the step recorded at 0.2
        loads = source.channel_at("tire-vertical-force", index)
        np.testing.assert_allclose(loads, [1000.2, 1100.2, 1200.2, 1300.2])
        self.assertEqual(source.scalar_at("speed", index), 10.0 + 20.0 * 0.2)

    def test_absent_channel_is_none_and_invalid_sample_is_masked_not_a_value(self):
        source = self.source(invalid_after=0.5)
        self.assertIsNone(source.channel_at("roll", 3))
        self.assertIsNone(source.scalar_at("roll", 3))
        valid = source.channel_at("steering", 10)
        self.assertFalse(np.ma.is_masked(valid))
        late = source.channel_at("steering", 100)
        self.assertTrue(np.ma.is_masked(late))
        self.assertTrue(late.mask.all())
        self.assertIsNone(source.scalar_at("steering", 100))
        self.assertFalse(source.channel_valid("steering", 100))
        self.assertTrue(source.channel_valid("steering", 10))

    def test_channel_names_limits_what_is_reachable(self):
        source = self.source(channel_names=("speed",))
        self.assertEqual(source.channel_names, ("speed",))
        self.assertIsNone(source.channel_at("steering", 0))
        with self.assertRaises(ValueError):
            self.source(channel_names=("not-a-channel",))


class RequireAndWarningsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_require_raises_for_roll_on_a_planar_record_and_returns_the_name_otherwise(self):
        source = FrameSource(make_record(self.directory.name), 120.0)
        with self.assertRaises(MissingChannelError) as caught:
            source.require("roll")
        self.assertIn("roll", str(caught.exception))
        self.assertIn("planar-7dof", str(caught.exception))
        self.assertEqual(source.require("speed"), "speed")

    def test_quasi_static_pair_warns_once_and_names_the_substitute(self):
        record = make_record(self.directory.name,
                             extra_channels=("roll-quasi-static", "pitch-quasi-static"))
        hidden = FrameSource(record, 120.0)
        self.assertEqual(len(hidden.warnings), 1)
        self.assertIn("roll-quasi-static", hidden.warnings[0])
        self.assertIn("--quasi-static-attitude", hidden.warnings[0])
        self.assertEqual(hidden.attitude_source, "none")
        self.assertIsNone(hidden.attitude_at(5))
        shown = FrameSource(record, 120.0, allow_quasi_static=True)
        self.assertEqual(len(shown.warnings), 1)
        self.assertIn("APPROXIMATE", shown.warnings[0])
        self.assertEqual(shown.attitude_source, "quasi-static")
        t = shown.times[60]
        roll, pitch = shown.attitude_at(60)
        # held from the step at or before the frame (step 0.004 s)
        held = np.floor(t / 0.004 + 1e-9) * 0.004
        self.assertAlmostEqual(roll, 0.001 * held, places=12)
        self.assertAlmostEqual(pitch, 0.002 * held, places=12)

    def test_dynamic_attitude_wins_and_needs_no_warning(self):
        record = make_record(self.directory.name, extra_channels=("roll", "pitch"),
                             model="14dof-simple")
        source = FrameSource(record, 120.0, allow_quasi_static=True)
        self.assertEqual(source.attitude_source, "dynamic")
        self.assertEqual(source.warnings, ())
        self.assertEqual(source.require("roll"), "roll")

    def test_a_record_without_any_attitude_has_no_warning_and_no_attitude(self):
        source = FrameSource(make_record(self.directory.name), 120.0)
        self.assertEqual(source.warnings, ())
        self.assertEqual(source.attitude_source, "none")

    def test_coarse_record_is_flagged_at_120_fps(self):
        record = make_record(self.directory.name, step=0.02, maximum_step=0.02)
        self.assertTrue(record.manifest.coarse_for_120_fps)
        warnings = FrameSource(record, 120.0).warnings
        self.assertEqual(len(warnings), 1)
        self.assertIn("coarse", warnings[0])
        self.assertEqual(FrameSource(record, 30.0).warnings, ())


class ScalesTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.source = FrameSource(make_record(self.directory.name), 120.0)

    def test_scales_are_run_maxima_and_identical_between_calls(self):
        first, second = self.source.scales(), self.source.scales()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["speed"], 30.0)
        self.assertAlmostEqual(first["steering"], 0.1)
        self.assertAlmostEqual(first["tire_vertical_force"], 1300.0 + 1.0)
        self.assertAlmostEqual(first["tire_force"], np.hypot(100.0, 400.0))
        self.assertAlmostEqual(first["tire_moment"], 5.0)
        self.assertAlmostEqual(first["acceleration"], np.hypot(20.0, 3.0), places=12)

    def test_scales_do_not_change_when_the_excerpt_changes(self):
        excerpt = FrameSource(self.source.record, 120.0, t_start=0.0, t_end=0.2)
        self.assertEqual(excerpt.scales(), self.source.scales())
        self.assertLess(excerpt.count, self.source.count)

    def test_returned_scales_are_a_copy(self):
        scales = self.source.scales()
        scales["speed"] = -1.0
        self.assertAlmostEqual(self.source.scales()["speed"], 30.0)

    def test_arrow_at_the_run_maximum_has_the_declared_maximum_length(self):
        peak = self.source.scales()["speed"]
        self.assertAlmostEqual(self.source.arrow_length("speed", peak, 2.5), 2.5, places=12)
        self.assertAlmostEqual(self.source.arrow_length("speed", peak / 2, 2.5), 1.25, places=12)
        with self.assertRaises(MissingChannelError):
            self.source.arrow_length("nothing", 1.0, 1.0)

    def test_scale_is_missing_when_its_channel_is_absent(self):
        limited = FrameSource(self.source.record, 120.0, channel_names=("speed",))
        self.assertEqual(set(limited.scales()), {"speed"})

    def test_an_all_zero_channel_gets_a_safe_unit_scale(self):
        recorder = RunRecorder(Path(self.directory.name) / "zero.npz", state_names=("s",),
                               channel_names=("steering",), precision="float64")
        for index, t in enumerate((0.0, 0.5, 1.0)):
            recorder.append(t, index, [t], [1.0], None, {"steering": [0.0]})
        source = FrameSource(load_run_record(recorder.close({})), 120.0)
        self.assertEqual(source.scales()["steering"], 1.0)
        self.assertEqual(source.arrow_length("steering", 0.0, 2.0), 0.0)


class WheelSpinTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_spin_angle_is_minus_the_integral_of_wheel_speed(self):
        source = FrameSource(make_record(self.directory.name), 120.0)
        angle = source.wheel_spin_angles()
        self.assertEqual(angle.shape, (source.count, 4))
        np.testing.assert_array_equal(angle[0], 0.0)
        t = source.times
        exact = -(10.0 * t + 10.0 * t * t) / 0.3  # integral of (10 + 20 t) / 0.3
        for wheel in range(4):
            np.testing.assert_allclose(angle[:, wheel], exact, atol=1e-9)
        self.assertIs(source.wheel_spin_angles(), angle)  # computed once

    def test_a_model_without_wheel_speed_states_has_no_spin(self):
        recorder = RunRecorder(Path(self.directory.name) / "pm.npz", state_names=("s", "v_x"),
                               channel_names=("speed",), precision="float64")
        for index, t in enumerate((0.0, 1.0)):
            recorder.append(t, index, [t, 1.0], [1.0, 0.0], None, {"speed": [1.0]})
        source = FrameSource(load_run_record(recorder.close({})), 120.0)
        self.assertIsNone(source.wheel_spin_angles())

    def test_vertical_plant_wheel_speed_names_are_found(self):
        names = ("s", "wheel_speed_fl", "wheel_speed_fr", "wheel_speed_rl", "wheel_speed_rr")
        recorder = RunRecorder(Path(self.directory.name) / "v.npz", state_names=names,
                               precision="float64")
        for index, t in enumerate((0.0, 1.0, 2.0)):
            recorder.append(t, index, [t, 30.0, 30.0, 30.0, 30.0], [1.0, 0, 0, 0, 0])
        source = FrameSource(load_run_record(recorder.close({})), 10.0)
        np.testing.assert_allclose(source.wheel_spin_angles()[-1], -60.0)


if __name__ == "__main__":
    unittest.main()
