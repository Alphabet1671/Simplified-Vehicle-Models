import unittest

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.planning.yaw_preflight import preflight
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath


def make_map(*, mask=None, into=1e9):
    v = np.linspace(0., 12., 13); ax = np.array([-4., 0., 4.]); ay = np.linspace(-8., 8., 9)
    shape = (v.size, ax.size, ay.size)
    mask = np.ones(shape, bool) if mask is None else mask
    return YawCapabilityMap(v, ax, ay, np.full(shape, into), np.full(shape, into), mask, 1.,
                            "physical_no_tv_v1", "preflight-test")


class PreflightTests(unittest.TestCase):
    def test_supported_path_passes_and_reports_speed_range(self):
        d = np.linspace(0., 20., 201)
        report = preflight(YawPath(d, np.full_like(d, .1), np.zeros_like(d), False), make_map())
        self.assertTrue(report.ok, report)
        # ay limit 8 m/s^2 at kappa .1 -> sqrt(80) ~ 8.94 m/s; sampled below that.
        self.assertLessEqual(report.metrics["steady_speed_max_mps"], 80 ** .5 + 1e-9)
        self.assertGreater(report.metrics["steady_speed_max_mps"], 8.0)
        self.assertGreater(report.metrics["lateral_axis_saturation_fraction"], .9)

    def test_masked_band_is_a_blocking_map_hole_with_location(self):
        mask = np.ones((13, 3, 9), bool)
        mask[:, :, 5:] = False  # no positive lateral acceleration support at all
        d = np.linspace(0., 20., 201)
        k = np.where((d > 8) & (d < 12), .1, 0.)
        report = preflight(YawPath(d, k, np.zeros_like(d), False), make_map(mask=mask))
        self.assertFalse(report.ok)
        self.assertTrue(any(b.startswith("map_hole") for b in report.blocking))
        holes = report.metrics["map_hole_distances_m"]
        self.assertTrue(holes and all(8 < x < 12 for x in holes))

    def test_curvature_rate_jitter_is_reported_not_smoothed(self):
        rng = np.random.default_rng(1)
        d = np.linspace(0., 50., 1001)
        k = .02 * np.sin(d / 8.)
        kp = np.gradient(k, d)
        kp_noisy = kp.copy(); idx = rng.choice(len(d), 120, replace=False); kp_noisy[idx] += .05
        clean = preflight(YawPath(d, k, kp, False), make_map())
        noisy = preflight(YawPath(d, k, kp_noisy, False), make_map())
        self.assertFalse(any(w.startswith("curvature_rate_noise") for w in clean.warnings))
        self.assertTrue(any(w.startswith("curvature_rate_noise") for w in noisy.warnings))


if __name__ == "__main__":
    unittest.main()
