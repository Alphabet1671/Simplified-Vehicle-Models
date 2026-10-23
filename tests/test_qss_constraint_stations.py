"""Localized physical off-site constraint station regressions."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

try:
    import casadi  # noqa: F401
except ImportError:
    CASADI = False
else:
    CASADI = True

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _surface():
    speed = np.linspace(.5, 40., 81)
    curvature = np.linspace(-.3, .3, 61)
    return CapabilitySurface(speed, curvature, np.full((81, 61), 3.),
                             np.full((81, 61), -8.), np.full(61, 25.), "fixture")


@unittest.skipUnless(CASADI, "CasADi is not installed")
class ConstraintStationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.track = load_track(_write_circle_track(Path(self.tmp.name), radius=40., nodes=72, half_width=3.))
        self.track = replace(self.track, is_closed=False)
        self.mesh = np.array([0., 13., 31., self.track.lap_length])
        self.spline = OffsetSpline(self.track, control_spacing=20.)
        base = corridor_from_track(self.track)
        self.corridor = type(base)(base.distance, np.zeros_like(base.minimum),
                                   np.zeros_like(base.maximum), base.provenance, False)

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, stations=None, mode="quadratic_acceleration_v1"):
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
        settings = QssSettings(start_speed=10., control_representation=mode)
        return build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                self.spline, settings, constraint_stations=stations)

    def test_uneven_sites_localized_and_default_parity(self):
        default = self.build()
        explicit = self.build([18.])
        self.assertEqual(explicit["off_site_stations"].size, default["off_site_stations"].size + 1)
        self.assertEqual(np.count_nonzero(explicit["off_site_elements"] == 1),
                         np.count_nonzero(default["off_site_elements"] == 1) + 1)
        self.assertEqual(np.count_nonzero(explicit["off_site_elements"] == 0),
                         np.count_nonzero(default["off_site_elements"] == 0))
        baseline = self.build(None)
        np.testing.assert_array_equal(default["off_site_stations"], baseline["off_site_stations"])
        np.testing.assert_array_equal(default["off_site_fractions"], baseline["off_site_fractions"])

    def test_validation_and_canonicalization(self):
        for stations in ([np.nan], [-1.], [self.mesh[-1]], [0.]):
            with self.assertRaises(ValueError):
                self.build(stations)
        default = self.build()
        # Existing node/midpoint site and duplicate explicit sites add no rows.
        fractions = np.asarray(QssSettings().off_collocation_fractions)
        existing = self.mesh[1] + np.diff(self.mesh)[1] * fractions[0]
        canonical = self.build([existing, existing, 18., 18.])
        self.assertEqual(canonical["off_site_stations"].size, default["off_site_stations"].size + 1)
        self.assertTrue(np.all(np.diff(canonical["off_site_stations"]) > 0.))

    def test_offsite_acceleration_rows_in_both_control_modes(self):
        for mode in ("quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1"):
            built = self.build([18.], mode)
            self.assertEqual(built["off_site_acceleration_lower_indices"].size,
                             built["off_site_stations"].size)
            self.assertEqual(built["off_site_domain_speed_indices"].size,
                             built["off_site_stations"].size)
            self.assertEqual(len(built["off_site_element_indices"]), self.mesh.size - 1)
            self.assertEqual(sum(x.size for x in built["off_site_element_indices"]),
                             built["off_site_stations"].size)


if __name__ == "__main__":
    unittest.main()
