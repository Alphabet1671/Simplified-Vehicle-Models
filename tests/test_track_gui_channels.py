import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.io import savemat

from tools.track_gui.mat_channels import ChannelSelection, list_channels, load_coordinates


def signal(time, value, units):
    return {"Time": np.asarray(time), "Value": np.asarray(value), "Units": units}


class MatChannelsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "signals.mat"

    def tearDown(self):
        self.temp.cleanup()

    def save(self, data):
        savemat(self.path, data)

    def test_mismatched_clocks_velocity_and_units(self):
        self.save({"VNorth": signal([0, 1, 2], [1, 1, 1], "m/s"),
                   "VEast": signal([0, .5, 1, 1.5, 2], [0] * 5, "km/h")})
        t, n, e = load_coordinates(self.path, ChannelSelection(), 0, 2)
        np.testing.assert_allclose(t, [0, 1, 2])
        np.testing.assert_allclose(n, [0, 1, 2])
        np.testing.assert_allclose(e, 0)
        self.assertEqual(len(list_channels(self.path)), 2)

    def test_position_and_gps(self):
        self.save({"N": signal([0, 1, 2], [10, 11, 12], "m"), "E": signal([0, 1, 2], [4, 6, 8], "m")})
        _, n, e = load_coordinates(self.path, ChannelSelection("N", "E", "position"), 0, 2)
        np.testing.assert_allclose(n, [0, 1, 2]); np.testing.assert_allclose(e, [0, 2, 4])
        self.save({"lat": signal([0, 1, 2], [40, 40.001, 40.002], "deg"),
                   "lon": signal([0, 1, 2], [-88, -87.999, -87.998], "deg")})
        _, n, e = load_coordinates(self.path, ChannelSelection("lat", "lon", "gps"), 0, 2)
        self.assertGreater(n[-1], 200); self.assertGreater(e[-1], 100)

    def test_unknown_units_explicit_scale_and_no_acceleration_override(self):
        self.save({"a": signal([0, 1, 2], [1, 1, 1], ""), "b": signal([0, 1, 2], [1, 1, 1], "")})
        with self.assertRaisesRegex(ValueError, "unknown velocity units"):
            load_coordinates(self.path, ChannelSelection("a", "b"), 0, 2)
        with self.assertRaisesRegex(ValueError, "unknown velocity units"):
            load_coordinates(self.path, ChannelSelection("a", "b", x_scale=2, y_scale=2), 0, 2)
        load_coordinates(self.path, ChannelSelection("a", "b", x_scale=2, y_scale=2,
                                                       x_allow_unknown_units=True, y_allow_unknown_units=True), 0, 2)
        self.save({"a": signal([0, 1, 2], [1, 1, 1], "m/s^2"), "b": signal([0, 1, 2], [1, 1, 1], "m/s")})
        with self.assertRaisesRegex(ValueError, "unsupported velocity units"):
            load_coordinates(self.path, ChannelSelection("a", "b", x_scale=2), 0, 2)

    def test_errors_for_missing_bad_clocks_and_values(self):
        self.save({"x": signal([0, 1, 1], [1, 1, 1], "m/s"), "y": signal([0, 1, 2], [0, 0, 0], "m/s")})
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            load_coordinates(self.path, ChannelSelection("x", "y"), 0, 2)
        with self.assertRaisesRegex(ValueError, "not found"):
            load_coordinates(self.path, ChannelSelection("missing", "y"), 0, 2)
        self.save({"x": signal([0, 1], [1, np.nan], "m/s"), "y": signal([0, 1], [0, 0], "m/s")})
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            load_coordinates(self.path, ChannelSelection("x", "y"), 0, 1)

    def test_legacy_xtime_fallback(self):
        self.save({"x": {"Value": [1, 1, 1]}, "y": {"Value": [0, 0, 0]}, "xtime": {"Value": [0, 1, 2]}})
        _, n, _ = load_coordinates(self.path, ChannelSelection("x", "y", x_scale=1, y_scale=1, x_allow_unknown_units=True, y_allow_unknown_units=True), 0, 2)
        np.testing.assert_allclose(n, [0, 1, 2])


if __name__ == "__main__":
    unittest.main()
