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
        fractions = np.asarray(QssSettings().off_collocation_fractions)
        expected = (self.mesh[:-1, None] + np.diff(self.mesh)[:, None] * fractions).ravel()
        np.testing.assert_array_equal(default["off_site_stations"], expected)
        np.testing.assert_allclose(
            default["off_site_fractions"],
            np.tile(fractions, self.mesh.size - 1), rtol=0., atol=2e-15,
        )

    def test_validation_and_canonicalization(self):
        for stations in ([np.nan], [-1.], [self.mesh[-1]], [0.], 18.):
            with self.assertRaises(ValueError):
                self.build(stations)
        default = self.build()
        # Existing node/midpoint site and duplicate explicit sites add no rows.
        fractions = np.asarray(QssSettings().off_collocation_fractions)
        existing = self.mesh[1] + np.diff(self.mesh)[1] * fractions[0]
        canonical = self.build([existing, existing, 18., 18.])
        self.assertEqual(canonical["off_site_stations"].size, default["off_site_stations"].size + 1)
        self.assertTrue(np.all(np.diff(canonical["off_site_stations"]) > 0.))

    def test_default_metadata_preserves_legacy_sites_on_mixed_scale_mesh(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import _canonical_constraint_sites
        mesh = np.array([0., 1e-8, 1e8])
        fractions = np.array([.25, .5, .75])
        stations, elements, local, _ = _canonical_constraint_sites(mesh, fractions, None)
        expected = (mesh[:-1, None] + np.diff(mesh)[:, None] * fractions).ravel()
        np.testing.assert_array_equal(stations, expected)
        np.testing.assert_array_equal(elements, np.repeat(np.arange(2), 3))
        np.testing.assert_array_equal(local, np.tile(fractions, 2))

    def test_offsite_rows_use_actual_station_and_physical_acceleration(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import _spline_geometry

        station = 18.
        for mode in ("quadratic_acceleration_v1", "quadratic_energy_rate_over_metric_v1"):
            settings = QssSettings(start_speed=10., control_representation=mode)
            cap = np.full(self.mesh.size, 20.)
            cap[1] = 12.
            # A nonzero corridor ensures the added site gets an analytic corridor row.
            base = corridor_from_track(self.track)
            corridor = type(base)(base.distance, np.full_like(base.minimum, -2.),
                                  np.full_like(base.maximum, 2.), base.provenance, False)
            from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
            built = build_spline_ocp(
                self.track, corridor, _surface(), self.mesh, self.spline, settings,
                speed_cap=cap, constraint_stations=[station],
            )
            site = int(np.flatnonzero(built["off_site_stations"] == station)[0])
            self.assertEqual(built["off_site_elements"][site], 1)
            self.assertAlmostEqual(built["off_site_fractions"][site], 5. / 18.)
            self.assertEqual(built["off_site_acceleration_lower_indices"].size,
                             built["off_site_stations"].size)
            self.assertEqual(built["off_site_domain_speed_indices"].size,
                             built["off_site_stations"].size)
            self.assertEqual(sum(x.size for x in built["off_site_element_indices"]),
                             built["off_site_stations"].size)

            controls, nodes = built["control_count"], self.mesh.size
            values = np.zeros(controls + 3 * nodes - 1)
            values[:controls] = np.linspace(-.7, .8, controls)
            values[controls:controls + nodes] = 10.
            values[controls + nodes:controls + 2 * nodes] = [1., 2., 3., 1.]
            values[controls + 2 * nodes:] = [2., 2., 2.]
            g = np.asarray(ca.Function(f"station_g_{mode[:3]}", [built["nlp"]["x"]],
                                       [built["nlp"]["g"]])(values)).ravel()
            from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory
            trajectory = OffsetTrajectory(built["reference"], built["spline"], values[:controls])
            ref = built["reference"].evaluate(np.array([station]))
            basis = built["spline"].basis(np.array([station]))
            geom = _spline_geometry(
                float((basis[0] @ values[:controls])[0]),
                float((basis[1] @ values[:controls])[0]),
                float((basis[2] @ values[:controls])[0]),
                float(ref.metric[0]), float(ref.metric_s[0]),
                float(ref.curvature[0]), float(ref.curvature_s[0]),
            )
            element, xloc = 1, 5. / 18.
            energy_at_site = (
                (2*xloc**3 - 3*xloc**2 + 1) * values[controls + element]**2
                + (xloc**3 - 2*xloc**2 + xloc) * 18. *
                  2. * values[controls + nodes + element] *
                  float(trajectory.evaluate(np.array([self.mesh[element]])).metric[0])
                + (-2*xloc**3 + 3*xloc**2) * values[controls + element + 1]**2
                + (xloc**3 - xloc**2) * 18. *
                  2. * values[controls + nodes + element + 1] *
                  float(trajectory.evaluate(np.array([self.mesh[element+1]])).metric[0])
            )
            off_speed = np.sqrt(energy_at_site)
            metric_nodes = np.asarray(trajectory.evaluate(self.mesh).metric)
            metric_mid = np.asarray(trajectory.evaluate(.5 * (self.mesh[:-1] + self.mesh[1:])).metric)
            left_f = 2. * values[controls + nodes + element] * metric_nodes[element]
            right_f = 2. * values[controls + nodes + element + 1] * metric_nodes[element + 1]
            middle_f = 2. * values[controls + 2*nodes + element] * metric_mid[element]
            raw_control = (2*xloc*(xloc-.5)*values[controls+nodes+element+1]
                           + 4*xloc*(1-xloc)*values[controls+2*nodes+element]
                           + 2*(xloc-.5)*(xloc-1)*values[controls+nodes+element])
            physical_acceleration = raw_control
            if mode == "quadratic_energy_rate_over_metric_v1":
                f_site = (2*xloc*(xloc-.5)*right_f + 4*xloc*(1-xloc)*middle_f
                          + 2*(xloc-.5)*(xloc-1)*left_f)
                physical_acceleration = f_site / (2. * float(trajectory.evaluate(np.array([station])).metric[0]))
            lower_row = int(built["off_site_acceleration_lower_indices"][site])
            self.assertEqual(built["g_lower"][lower_row], 8. * settings.tolerance)
            np.testing.assert_allclose(g[lower_row], physical_acceleration + 8.,
                                       rtol=1e-10, atol=2e-10)
            if mode == "quadratic_energy_rate_over_metric_v1":
                self.assertGreater(abs(physical_acceleration - raw_control), 1e-5)

            cap_row = int(built["off_site_speed_cap_indices"][site])
            expected_cap = off_speed - np.interp(station, self.mesh, cap)
            self.assertAlmostEqual(g[cap_row], expected_cap, delta=2e-10)
            corridor_row = int(built["off_site_corridor_indices"][site])
            expected_offset = float(trajectory.evaluate(np.array([station])).offset[0])
            self.assertAlmostEqual(g[corridor_row], expected_offset, delta=2e-10)


if __name__ == "__main__":
    unittest.main()
