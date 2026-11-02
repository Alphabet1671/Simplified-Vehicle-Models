import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.vertical_dense_certificate import certify_vertical_dense_domain


class UnlimitedTravelDomainTests(unittest.TestCase):
    def test_dense_certificate_ignores_legacy_jounce_design_range(self):
        bounds = np.zeros((1, 38, 2), dtype=float)
        bounds[:, :, 0] = -1e-9
        bounds[:, :, 1] = 1e-9
        bounds[:, 3, :] = (10.0, 11.0)
        bounds[:, 12:16, :] = (1.0, 2.0)
        bounds[:, 16:20, :] = (0.08, 0.10)
        bounds[:, 24, :] = (0.5, 0.5)
        bounds[:, 25, :] = (0.0, 0.0)
        class Plan:
            distance = np.array([0.0, 1.0])
            curvature = np.array([0.0, 0.0])
            is_closed = False
        vehicle = SimpleNamespace(
            suspension=SimpleNamespace(kinematics=SimpleNamespace(
                jounce_limits=np.tile(np.array([-0.0318, 0.0318]), (4, 1)))),
            planar=SimpleNamespace(steering_maximum_angle=0.5))
        cert = certify_vertical_dense_domain(vehicle, Plan(), bounds)
        self.assertNotIn('jounce_upper_margin', cert.get('margins', {}))
        self.assertNotIn('jounce_lower_margin', cert.get('margins', {}))
        self.assertNotIn('jounce_upper_margin', cert.get('failed', []))
        self.assertNotIn('jounce_lower_margin', cert.get('failed', []))
        self.assertEqual(cert.get('suspension_travel_policy'), 'unlimited')
        self.assertIs(cert.get('suspension_travel_enforced'), False)

    def test_malformed_design_metadata_is_ignored(self):
        bounds = np.zeros((1, 38, 2), dtype=float)
        bounds[:, :, 0] = -1e-9
        bounds[:, :, 1] = 1e-9
        bounds[:, 3, :] = (10.0, 11.0)
        bounds[:, 12:16, :] = (1.0, 2.0)
        bounds[:, 16:20, :] = (0.001, 0.002)
        bounds[:, 24, :] = (0.5, 0.5)
        bounds[:, 25, :] = (0.0, 0.0)
        vehicle = SimpleNamespace(
            suspension=SimpleNamespace(kinematics=SimpleNamespace(jounce_limits=[["bad"], [1]])),
            planar=SimpleNamespace(steering_maximum_angle=0.5))
        class Plan:
            distance = np.array([0.0, 1.0])
            curvature = np.array([0.0, 0.0])
            is_closed = False
        cert = certify_vertical_dense_domain(vehicle, Plan(), bounds)
        self.assertIsNone(cert["jounce_design_range_m"])

    def test_dense_certificate_still_fails_nonfinite_enclosure(self):
        bounds = np.zeros((1, 38, 2), dtype=float)
        bounds[0, 16, 0] = np.nan
        cert = certify_vertical_dense_domain(object(), object(), bounds)
        self.assertFalse(cert['certified'])
        self.assertIn('non-finite', cert['reason'])


if __name__ == '__main__':
    unittest.main()
