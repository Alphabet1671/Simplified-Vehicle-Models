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
from tools.track_gui.mat_channels import ChannelSelection
from tools.track_gui.export import SaveRefused
from simplified_vehicle_models.tracks.profile import load_track


def write_circle(path, radius=30.0, period=60.0, sign=1):
    t = np.linspace(0.0, period, 3001)
    w = 2.0 * np.pi / period
    speed = radius * w
    savemat(path, {"xtime": {"Value": t}, "VNorth": {"Value": speed * np.cos(sign * w * t)},
                   "VEast": {"Value": speed * np.sin(sign * w * t)}})


def write_multilap_stadium(path):
    half, radius = 25.0, 20.0
    bottom = np.column_stack((np.linspace(-half, half, 251), np.full(251, -radius)))
    theta = np.linspace(-np.pi/2, np.pi/2, 201)[1:]
    right = np.column_stack((half + radius*np.cos(theta), radius*np.sin(theta)))
    top = np.column_stack((np.linspace(half, -half, 501)[1:], np.full(500, radius)))
    theta2 = np.linspace(np.pi/2, 3*np.pi/2, 201)[1:]
    left = np.column_stack((-half + radius*np.cos(theta2), radius*np.sin(theta2)))
    loop = np.vstack((bottom, right, top, left))
    closed = np.vstack((loop, loop[0]))
    edge = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    distance = np.r_[0.0, np.cumsum(edge)]
    grid = np.arange(0.0, distance[-1], 0.25)
    base = np.column_stack([np.interp(grid, distance, closed[:, axis]) for axis in range(2)])
    base = np.roll(base, -np.argmin(np.linalg.norm(base-[0.0,-radius],axis=1)), axis=0)
    laps = []
    for lateral in (-0.22, 0.0, 0.22):
        p = base.copy()
        p[:, 1] += lateral
        p[:, 1] += 0.5 * np.exp(-((np.minimum(np.arange(len(p)), len(p)-np.arange(len(p)))/3.0)**2))
        laps.append(p)
    points = np.vstack([np.vstack((p, p[0])) for p in laps])
    t = np.arange(len(points)) * 0.04
    savemat(path, {"xtime":{"Value":t}, "North":{"Value":points[:,0],"Units":"m"},
        "East":{"Value":points[:,1],"Units":"m"}})


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

    def test_multi_lap_builder_runs_seam_fit_and_round_trips_provenance(self):
        mat = self.folder / "stadium.mat"
        write_multilap_stadium(mat)
        settings = ClosedSettings(mat, 0.0, 300.0, spacing=0.5, smoothing=0.0,
            maximum_radius=300.0, channel_selection=ChannelSelection("North", "East", "position"),
            multi_lap=True, return_radius=2.0, minimum_travel=150.0,
            minimum_duration=20.0, maximum_duration=150.0, deviation_limit=2.0)
        geometry = build_closed(settings)
        diag = geometry["lap_diagnostics"]
        self.assertEqual(diag["seam_closure"]["status"], "success")
        self.assertGreater(diag["seam_closure"]["physical_span_m"], 5.0)
        self.assertGreater(diag["seam_closure"]["peak_movement_m"], 1e-5)
        self.assertLess(diag["seam_closure"]["bending_energy_after"],
                        diag["seam_closure"]["bending_energy_before"])
        self.assertEqual(len(diag["closure_gaps_m"]), diag["accepted_laps"])
        self.assertGreaterEqual(len(diag["closure_gaps_m"]), 2)
        self.assertTrue(np.isfinite(geometry["north"]).all())
        self.assertTrue(np.isfinite(geometry["east"]).all())
        self.assertTrue(np.isfinite(geometry["curvature"]).all())
        out = self.folder / "out"
        saved = save_closed(geometry, settings, out, "stadium", "test-stadium", "Test Stadium")
        profile = load_track(saved.xml)
        self.assertTrue(profile.is_closed)
        xml = saved.xml.read_text()
        self.assertIn('"seam_closure"', xml)
        self.assertIn('"closure_gaps_m"', xml)
        self.assertIn('"method"', xml)

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
