import unittest

import numpy as np

from simplified_vehicle_models.suspension.physical import LinearDamper, PhysicalStrut


class PhysicalTests(unittest.TestCase):
    def test_physical_force_and_disabled_damper(self):
        s = PhysicalStrut(spring_rate=1000., preload=20., inertance=3.,
                          damper=LinearDamper(40.), damper_scale=1.,
                          motion_ratio=2., orientation=1.)
        force, damping, flag = s.force(.01, .2)
        self.assertAlmostEqual(float(force), 38.)
        self.assertAlmostEqual(float(damping), 8.)
        self.assertFalse(bool(flag))

        class MustNotRun:
            def force(self, velocity):
                raise AssertionError('disabled damper evaluated')
        zero = PhysicalStrut(0., 0., 0., MustNotRun(), 0., 1., 1.)
        self.assertEqual(float(zero.force(10., 100.)[0]), 0.)

    def test_force_shapes_and_range_flags(self):
        s = PhysicalStrut(100., 2., 0., LinearDamper(5.), 2., 1., 1.)
        force, damping, flag = s.force(np.array([0., .1]), np.array([1., -2.]))
        np.testing.assert_allclose(force, [12., -8.])
        np.testing.assert_allclose(damping, [10., -20.])
        np.testing.assert_array_equal(flag, [False, False])

        disabled = PhysicalStrut(10., 0., 0., object(), 0., 1., 1.)
        f, d, flags = disabled.force(np.array([1., 2.]), np.array([3., 4.]))
        np.testing.assert_array_equal(f, [10., 20.])
        np.testing.assert_array_equal(d, [0., 0.])
        np.testing.assert_array_equal(flags, [False, False])

    def test_rejects_invalid_physical_parameters(self):
        for field in ('spring_rate', 'preload', 'inertance', 'damper_scale'):
            for value in (np.nan, np.inf, -1.):
                with self.subTest(field=field, value=value):
                    kwargs = dict(spring_rate=1., preload=0., inertance=0.,
                                  damper=LinearDamper(1.), damper_scale=1.,
                                  motion_ratio=1., orientation=1.)
                    kwargs[field] = value
                    with self.assertRaises(ValueError):
                        PhysicalStrut(**kwargs)
        for value in (np.nan, np.inf, -1.):
            with self.subTest(linear_coefficient=value):
                with self.assertRaises(ValueError):
                    LinearDamper(value)
        for value in (0., -1., np.nan, np.inf):
            with self.subTest(motion_ratio=value):
                with self.assertRaises(ValueError):
                    PhysicalStrut(1., 0., 0., LinearDamper(1.), 1., value, 1.)
        for value in (0., 2., -2., np.nan):
            with self.subTest(orientation=value):
                with self.assertRaises(ValueError):
                    PhysicalStrut(1., 0., 0., LinearDamper(1.), 1., 1., value)
        with self.assertRaises(ValueError):
            PhysicalStrut(0., 1., 0., LinearDamper(1.), 1., 1., 1.)


if __name__ == '__main__':
    unittest.main()
