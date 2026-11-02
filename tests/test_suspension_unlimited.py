import unittest

import numpy as np

from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
from simplified_vehicle_models.suspension.elements import CornerSprings
from simplified_vehicle_models.suspension.kinematics import AxleGeometry


class SplineExtrapolationTests(unittest.TestCase):
    def test_linear_tails_use_boundary_tangents_and_default_is_strict(self):
        spline = MonotoneSpline([-1., 0., 1.], [-10., 0., 30.], extrapolation='linear')
        for end, q in ((0, -2.), (-1, 3.)):
            self.assertAlmostEqual(float(spline(q)), spline.y[end] + spline.slopes[end] * (q - spline.x[end]))
            self.assertAlmostEqual(float(spline.derivative(q)), spline.slopes[end])
        with self.assertRaises(DomainError):
            MonotoneSpline([-1., 1.], [-1., 1.])(2.)

    def test_constant_and_nonfinite_queries(self):
        spline = MonotoneSpline([0., 1.], [2., 5.], extrapolation='constant')
        np.testing.assert_allclose(spline(np.array([-10., 0., 1., 10.])), [2., 2., 5., 5.])
        np.testing.assert_allclose(spline.derivative(np.array([-10., 10.])), [0., 0.])
        for q in (np.nan, np.inf, -np.inf):
            with self.assertRaises(DomainError):
                spline(q)

    def test_curve_tangent_spring_and_constant_motion_ratio_contracts(self):
        # Nonlinear, bump-stop-like rate curve: endpoint slopes differ from secants.
        x = np.array([-1., -.25, 0., .4, 1.])
        y = np.array([-100., -20., 0., 60., 260.])
        strict = MonotoneSpline(x, y)
        curves = CornerSprings(strict, strict)
        spring = curves.front
        self.assertGreater(spring.slopes[-1], (y[-1] - y[-2]) / (x[-1] - x[-2]))
        for end, q in ((0, -2.), (-1, 2.5)):
            expected_force = spring.y[end] + spring.slopes[end] * (q - spring.x[end])
            self.assertAlmostEqual(float(spring(q)), expected_force)
            self.assertAlmostEqual(float(spring.derivative(q)), spring.slopes[end])
        in_range = np.array([-.7, -.1, .2, .8])
        np.testing.assert_allclose(spring(in_range), strict(in_range), rtol=0., atol=0.)
        np.testing.assert_allclose(spring.derivative(in_range), strict.derivative(in_range), rtol=0., atol=0.)

    def test_axle_geometry_direct_fixture_opts_geometry_policy(self):
        spline = MonotoneSpline([-1., 0., 1.], [-.1, 0., .2])
        motion = MonotoneSpline([-1., 0., 1.], [.6, .5, .7])
        axle = AxleGeometry(motion, MonotoneSpline([0., 1.], [.5, .5]),
                            MonotoneSpline([0., 1.], [0., 1.]), 0., 0., 0., 0., spline,
                            (-1., 1.), 100.)
        self.assertAlmostEqual(float(axle.jounce_steer(2.)), .2 + spline.slopes[-1])
        self.assertAlmostEqual(float(axle.jounce_steer(-2.)), -.1 + spline.slopes[0] * -1.)
        self.assertEqual(float(axle.motion_ratio(2.)), .7)
        self.assertEqual(float(axle.motion_ratio(-2.)), .6)
        self.assertGreater(float(axle.motion_ratio(2.)), 0.)
        self.assertGreater(float(axle.motion_ratio(-2.)), 0.)
        self.assertEqual(float(axle.motion_ratio.derivative(2.)), 0.)
        self.assertEqual(float(axle.motion_ratio.derivative(-2.)), 0.)
        with self.assertRaises(DomainError):
            axle.roll_centre_height(2.)


if __name__ == '__main__':
    unittest.main()
