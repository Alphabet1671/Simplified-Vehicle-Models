import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

try:
    from scipy.io import savemat
except ImportError:
    savemat = None

from tools.track_gui.closed import ClosedSettings, build_closed, save_closed, scipy_available, summarize
from tools.track_gui.export import SaveRefused
from simplified_vehicle_models.tracks.profile import load_track


def write_circle(path, radius=30.0, period=60.0, sign=1):
    t = np.linspace(0.0, period, 3001)
    w = 2.0 * np.pi / period
    speed = radius * w
    savemat(path, {"xtime": {"Value": t}, "VNorth": {"Value": speed * np.cos(sign * w * t)},
                   "VEast": {"Value": speed * np.sin(sign * w * t)}})


@unittest.skipIf(savemat is None, "SciPy is not installed")
class ClosedTrackTests(unittest.TestCase):
    def setUp(self):
        self._folder = tempfile.TemporaryDirectory()
        self.addCleanup(self._folder.cleanup)
        self.folder = Path(self._folder.name)
        self.mat = self.folder / "circle.mat"
        write_circle(self.mat)
        self.settings = ClosedSettings(self.mat, 0.0, 60.0)

    def test_circle_geometry_and_summary(self):
        geometry = build_closed(self.settings)
        figures = summarize(geometry)
        self.assertAlmostEqual(figures["minimum_radius_m"], 30.0, delta=0.5)
        self.assertLess(figures["closure_gap_m"], 0.01)
        self.assertLess(figures["winding_error_rad"], 0.01)
        self.assertGreater(figures["minimum_self_distance_m"], 10.0)

    def test_save_uses_the_given_id_and_title(self):
        geometry = build_closed(self.settings)
        out = self.folder / "out"
        saved = save_closed(geometry, self.settings, out, "circle", "my-circle", "My Circle")
        self.assertTrue(saved.is_closed)
        text = saved.xml.read_text()
        self.assertIn('id="my-circle"', text)
        self.assertIn("<title>My Circle</title>", text)
        self.assertNotIn("michigan", text.lower())
        self.assertTrue(load_track(saved.xml).is_closed)

    def test_bad_settings_are_refused_before_scipy_runs(self):
        for bad in (ClosedSettings(self.mat, 10.0, 5.0), ClosedSettings(self.mat, 0.0, 60.0, spacing=0.0),
                    ClosedSettings(self.mat, 0.0, float("nan")), ClosedSettings(self.folder / "no.mat", 0.0, 60.0)):
            with self.assertRaises(ValueError):
                build_closed(bad)

    def test_invalid_settings_do_not_probe_scipy(self):
        bad = ClosedSettings(self.mat, 10.0, 5.0)
        with mock.patch("tools.track_gui.closed.scipy_available") as available:
            with self.assertRaises(ValueError):
                build_closed(bad)
        available.assert_not_called()

    def test_window_with_too_few_samples_is_refused(self):
        with self.assertRaisesRegex(ValueError, "too few samples"):
            build_closed(ClosedSettings(self.mat, 100.0, 200.0))

    def test_models_folder_is_protected(self):
        geometry = build_closed(self.settings)
        from tools.gui_common import REPOSITORY_ROOT
        with self.assertRaisesRegex(SaveRefused, "models/"):
            save_closed(geometry, self.settings, REPOSITORY_ROOT / "models" / "tracks", "zz", "zz", "ZZ")

    def test_scipy_flag(self):
        self.assertTrue(scipy_available())


if __name__ == "__main__":
    unittest.main()
