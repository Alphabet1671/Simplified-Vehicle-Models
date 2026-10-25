import unittest
import numpy as np

from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath, yaw_aware_profile
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile, validate_profile_object
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator, SUPPORT_WEIGHT_TOLERANCE
from tests.test_yaw_aware_profile import yaw_map


class ProfileValidationTests(unittest.TestCase):
    def setUp(self):
        self.distance = np.linspace(0., 10., 11)
        self.path = YawPath(self.distance, np.zeros(11), np.zeros(11), False)
        self.cap = yaw_map()
        self.settings = CoupledProfileSettings()
        self.profile = yaw_aware_profile(self.path, self.cap, boundary="standing", maximum_speed=10.)

    def test_recomputes_q_and_rejects_lowered_corrupt_node(self):
        q = self.profile.speed.copy()
        self.assertTrue(validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings).valid)
        q[5] = 0.
        result = validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings)
        self.assertFalse(result.valid)
        self.assertTrue(set((4, 5)) & set(result.invalid_segment_indices))

    def test_zero_zero_nan_and_boundary_are_rejected(self):
        q = self.profile.speed.copy(); q[4:6] = 0.
        self.assertFalse(validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings).valid)
        q = self.profile.speed.copy(); q[4] = np.nan
        result = validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings)
        self.assertFalse(result.valid)
        self.assertIsNone(result.recomputed_time)

    def test_flying_seam_tolerance_and_closed_rate(self):
        closed = YawPath(self.distance, np.zeros(11), np.zeros(11), True)
        q = np.ones(11); q[-1] += 4e-11
        edge = validate_profile(closed, self.cap, q, maximum_speed=10., boundary="flying", settings=self.settings)
        self.assertTrue(edge.valid)
        q[-1] += 7e-11
        result = validate_profile(closed, self.cap, q, maximum_speed=10., boundary="flying", settings=self.settings)
        self.assertFalse(result.valid)

    def test_standing_start_is_exactly_zero(self):
        q = self.profile.speed.copy(); q[0] = 1e-15
        result = validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings)
        self.assertFalse(result.valid)

    def test_object_time_and_elapsed_corruption_fails_validation(self):
        p = self.profile
        valid = validate_profile_object(self.path, self.cap, p, maximum_speed=10., boundary="standing", settings=self.settings)
        self.assertTrue(valid.valid)
        altered_lap = type(p)(p.distance, p.speed, p.longitudinal_acceleration, p.elapsed_time,
                              p.lap_time + 0.1, p.converged, p.iterations, p.maximum_speed_change, p.diagnostics)
        self.assertFalse(validate_profile_object(self.path, self.cap, altered_lap, maximum_speed=10., boundary="standing", settings=self.settings).valid)
        elapsed = p.elapsed_time.copy(); elapsed[-1] += 0.1
        altered_elapsed = type(p)(p.distance, p.speed, p.longitudinal_acceleration, elapsed,
                                  p.lap_time, p.converged, p.iterations, p.maximum_speed_change, p.diagnostics)
        self.assertFalse(validate_profile_object(self.path, self.cap, altered_elapsed, maximum_speed=10., boundary="standing", settings=self.settings).valid)

    def test_acceleration_domain_violation_reported_even_when_support_rejected(self):
        q = self.profile.speed.copy()
        q[1] = np.sqrt(q[0]**2 + 40.)
        result = validate_profile(self.path, self.cap, q, maximum_speed=10., boundary="standing", settings=self.settings)
        self.assertGreater(result.maximum_acceleration_domain_violation, 0.)
        self.assertGreater(result.invalid_support_count, 0)

    def test_dense_audit_is_reported(self):
        result = validate_profile(self.path, self.cap, self.profile.speed, maximum_speed=10., boundary="standing", settings=self.settings, dense=True)
        self.assertIsInstance(result.dense_sample_valid, bool)

    def test_exact_grid_face_ignores_zero_weight_masked_corner_but_positive_weight_fails(self):
        cap0 = yaw_map(ax=(-1., 0., 1.), ay=(-1., 0., 1.))
        mask = np.array(cap0.feasible_mask, copy=True)
        mask[20, 1, 2] = False
        cap = type(cap0)(cap0.speed, cap0.longitudinal_acceleration, cap0.lateral_acceleration,
                         cap0.yaw_moment_into_turn, cap0.yaw_moment_out_of_turn, mask,
                         cap0.yaw_inertia, cap0.source_kind, cap0.source_identity)
        interp = _MapInterpolator(cap)
        exact = interp.query_evidence(float(cap.speed[20]), 0., 0.)
        self.assertEqual(exact["reason"].value, "supported")
        self.assertEqual(exact["support_weight_tolerance"], SUPPORT_WEIGHT_TOLERANCE)
        near = interp.query_evidence(float(cap.speed[20]), 0., 0.1)
        self.assertEqual(near["reason"].value, "masked_support")

    def test_closed_path_requires_curvature_rate_seam_match(self):
        with self.assertRaisesRegex(ValueError, "curvature rate"):
            YawPath(self.distance, np.zeros(11), np.r_[np.zeros(10), 1e-10], True)

    def test_settings_reject_invalid_counts_and_tolerances(self):
        for kw in ({"q_seed_count": True}, {"q_seed_count": 1}, {"max_refinement_rounds": 0},
                   {"max_candidates_per_station": 0}, {"max_total_nodes": 0}, {"max_total_edges": 0},
                   {"max_seam_anchors": 0}, {"max_wall_seconds": np.inf},
                   {"interval_q_tolerance": 0.}, {"acceleration_tolerance": np.nan},
                   {"yaw_moment_tolerance": 0.}, {"refinement_time_relative_tolerance": -1.},
                   {"refinement_speed_tolerance": True}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                CoupledProfileSettings(**kw)

if __name__ == "__main__":
    unittest.main()
