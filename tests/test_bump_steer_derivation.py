"""Bump-steer derivation from suspension hardpoints and its use in the generator."""

from pathlib import Path
import math
import os
import unittest

import numpy as np

from tools.model_data import derive_bump_steer as bs
from tools.model_data import generate_suspension_lut as gen

ROOT = Path(__file__).resolve().parents[1]


def generic_corner(tie_rod_z=-0.15):
    """A non-degenerate right-hand corner [m; x fwd, y right, z DOWN]."""

    return bs.Corner(
        uca_front=[0.12, 0.30, -0.25], uca_rear=[-0.09, 0.31, -0.26],
        lca_front=[0.11, 0.25, -0.10], lca_rear=[-0.10, 0.26, -0.11],
        tierod_inner=[0.05, 0.28, tie_rod_z],
        uca_outer=[0.01, 0.52, -0.25], lca_outer=[-0.01, 0.57, -0.10],
        tierod_outer=[0.07, 0.55, -0.14], wheel_centre=[0.0, 0.62, -0.2],
    )


class SolverTests(unittest.TestCase):
    def test_zero_jounce_is_exactly_zero(self):
        toe, camber = bs.bump_steer(generic_corner(), [0.0])
        self.assertEqual(float(toe[0]), 0.0)
        self.assertEqual(float(camber[0]), 0.0)

    def test_raising_the_tie_rod_inner_point_changes_the_bump_steer(self):
        z = np.array([-0.025, 0.025])
        low, _ = bs.bump_steer(generic_corner(-0.15), z)
        high, _ = bs.bump_steer(generic_corner(-0.12), z)
        self.assertFalse(np.allclose(low, high, atol=1.0e-5))

    def test_agrees_with_an_independent_least_squares_solve(self):
        from scipy.optimize import least_squares
        corner = generic_corner()
        for rise in (-0.025, 0.02):
            reference = least_squares(lambda q: bs._residual(corner, q, rise), np.zeros(6),
                                      xtol=1.0e-15, ftol=1.0e-15, gtol=1.0e-15).x
            toe, camber = bs.bump_steer(corner, [rise])
            expected = bs._angles(reference)
            self.assertAlmostEqual(float(toe[0]), expected[0], places=9)
            self.assertAlmostEqual(float(camber[0]), expected[1], places=9)

    def test_solution_keeps_every_link_length_and_the_requested_wheel_height(self):
        corner = generic_corner()
        rise = 0.025
        q = bs._solve(corner, np.zeros(6), rise)
        rotation, shift = bs._rotation_vector(q[:3]), q[3:]
        for moving, fixed in corner.constraints():
            self.assertAlmostEqual(np.linalg.norm(rotation @ moving + shift - fixed),
                                   np.linalg.norm(moving - fixed), places=11)
        moved = rotation @ corner.wheel_centre + shift
        self.assertAlmostEqual(-(moved[2] - corner.wheel_centre[2]), rise, places=11)

    def test_toe_in_sign_for_a_right_turn_of_the_right_wheel(self):
        # A positive rotation about the downward z axis yaws the wheel to the right,
        # which turns the right wheel's nose away from the centreline: toe-out.
        toe, camber = bs._angles(np.array([0.0, 0.0, 0.01, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(toe, -0.01, places=12)
        self.assertAlmostEqual(camber, 0.0, places=12)

    def test_mirrored_corner_gives_the_same_toe_in_and_camber(self):
        corner = generic_corner()
        z = np.linspace(-0.03, 0.03, 13)
        right = bs.bump_steer(corner, z)
        left = bs.bump_steer(corner.mirrored(), z)
        np.testing.assert_allclose(left[0], right[0], atol=1.0e-12)
        np.testing.assert_allclose(left[1], right[1], atol=1.0e-12)

    def test_unordered_and_repeated_jounces_are_handled(self):
        corner = generic_corner()
        together, _ = bs.bump_steer(corner, [0.02, -0.02, 0.0, 0.02])
        one_by_one = [bs.bump_steer(corner, [v])[0][0] for v in (0.02, -0.02, 0.0, 0.02)]
        np.testing.assert_allclose(together, one_by_one, atol=1.0e-12)

    def test_nonfinite_jounce_is_rejected(self):
        with self.assertRaises(bs.BumpSteerError):
            bs.bump_steer(generic_corner(), [float("nan")])

    def test_knot_grid_has_both_limits_and_an_exact_zero(self):
        knots = bs.knot_grid(31.8, 30.0, 3)
        self.assertEqual((knots[0], knots[-1]), (-31.8, 30.0))
        self.assertEqual(int(np.sum(knots == 0.0)), 1)
        self.assertTrue(np.all(np.diff(knots) > 0.0))

    def test_formatted_knots_round_trip_through_the_generator_with_an_exact_zero_knot(self):
        knots = bs.knot_grid(31.8, 31.8, 3)
        toe = np.radians(np.linspace(-0.1, 0.1, knots.size) + 0.05)  # nonzero at the 0 knot
        text = bs.format_knots(knots, toe)
        jounce_mm, toe_deg = gen.parse_jounce_steer(text)
        np.testing.assert_allclose(jounce_mm, knots)
        self.assertEqual(toe_deg[list(knots).index(0.0)], 0.0)
        gen.validate_jounce_steer(np.array(jounce_mm)*1e-3, np.radians(toe_deg), (-0.0318, 0.0318))


# The real-car hardpoint workbook is private; point these at it to run the check.
WORKBOOK = Path(os.environ.get("VEHICLE_HARDPOINT_WORKBOOK", ""))
SHEET = os.environ.get("VEHICLE_HARDPOINT_SHEET", "")


@unittest.skipUnless(WORKBOOK.is_file() and SHEET,
                     "set VEHICLE_HARDPOINT_WORKBOOK and VEHICLE_HARDPOINT_SHEET to run")
class RealHardpointTests(unittest.TestCase):
    """The derived camber gain must reproduce the tabulated one (from real car data)."""

    @classmethod
    def setUpClass(cls):
        cls.front, cls.rear = bs.read_corners(str(WORKBOOK), SHEET, 1.0/bs.INCH)

    def test_camber_gain_matches_the_tabulated_values_and_toe_is_small(self):
        z = np.array([-0.001, 0.0, 0.001])
        for corner, tabulated in ((self.front, 25.1), (self.rear, 45.3)):
            _, camber = bs.bump_steer(corner, z)
            gain = math.degrees((camber[2] - camber[0])/0.002)
            self.assertAlmostEqual(abs(gain), tabulated, delta=0.6)
            toe, _ = bs.bump_steer(corner, np.linspace(-0.0318, 0.0318, 41))
            self.assertLess(float(np.max(np.abs(toe))), math.radians(0.3))

    def test_mirrored_hardpoints_give_the_same_curve(self):
        z = np.linspace(-0.0318, 0.0318, 9)
        for corner in (self.front, self.rear):
            np.testing.assert_allclose(bs.bump_steer(corner, z)[0], bs.bump_steer(corner.mirrored(), z)[0],
                                       atol=1.0e-12)


class GeneratorTests(unittest.TestCase):
    cells = gen.read_csv_cells(ROOT / "tests/fixtures/iem24_dss_suspension_rows.csv")

    def build(self, **kwargs):
        return gen.build_corner_arrays(
            self.cells, wheel_rate_front_n_m=450*gen.LBF_IN_TO_N_M, wheel_rate_rear_n_m=350*gen.LBF_IN_TO_N_M,
            static_load_front_n=505.777974, static_load_rear_n=646.503401, **kwargs)

    def test_hardpoint_curves_are_recorded_as_from_real_car_data(self):
        arrays = self.build(jounce_steer_front=([-31.8, 0.0, 31.8], [-0.2, 0.0, -0.1]),
                            jounce_steer_rear=([-31.8, 0.0, 31.8], [0.02, 0.0, -0.1]),
                            jounce_steer_source=gen.HARDPOINT_CURVE_SOURCE)
        text = str(arrays["metadata"])
        self.assertIn("from real car data", text)
        self.assertNotIn(gen.ZERO_CURVE_SOURCE, text)
        np.testing.assert_allclose(arrays["jounce_steer_toe_front"], np.radians([-0.2, 0.0, -0.1]))

    def test_default_remains_the_zero_baseline(self):
        arrays = self.build()
        np.testing.assert_array_equal(arrays["jounce_steer_toe_front"], np.zeros(3))
        self.assertIn(gen.ZERO_CURVE_SOURCE, str(arrays["metadata"]))

    def test_headers_name_the_source_only_when_a_curve_was_supplied(self):
        zero = gen.kinematics_xml("k", "../d.npz", "0"*64, 0.0, 0.0)
        derived = gen.kinematics_xml("k", "../d.npz", "0"*64, 0.0, 0.0, None, gen.HARDPOINT_CURVE_SOURCE)
        self.assertIn(gen.ZERO_CURVE_SOURCE, zero)
        self.assertNotIn(gen.ZERO_CURVE_SOURCE, derived)
        self.assertIn("from real car data", derived)

    def test_malformed_jounce_steer_text_is_rejected(self):
        for bad in ("", "1,2", "a:b", "1:2:3x"):
            with self.assertRaises(gen.SuspensionTableError, msg=bad):
                gen.parse_jounce_steer(bad)


if __name__ == "__main__":
    unittest.main()
