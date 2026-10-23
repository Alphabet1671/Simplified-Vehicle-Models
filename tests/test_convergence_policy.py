"""Fast analytical tests for proposed numerical convergence policy."""
from types import SimpleNamespace
import unittest

import numpy as np

from simplified_vehicle_models.solvers.convergence_policy import (
    NUMERICAL_POLICY_V1, compare_trajectory_states,
)


class Evaluator:
    def __init__(self, trajectory, speed=10.0):
        self.trajectory = trajectory
        self.speed_value = speed

    def evaluate(self, s):
        s = np.asarray(s)
        return {"speed": np.full_like(s, self.speed_value), "acceleration": np.zeros_like(s)}


class SourceReference:
    def __init__(self, knots, frame="world_X_north_Y_east_Z_down_v1", source_delta=0.):
        self.start, self.end, self.is_closed = 0., 1., False
        self.source_station = np.asarray(knots)
        self.coordinate_frame = frame
        self.source_delta = source_delta

    def evaluate(self, s):
        s = np.asarray(s)
        return SimpleNamespace(north=s + self.source_delta, east=np.zeros_like(s))


class Geometry:
    def __init__(self, delta, metric=1.0):
        self.delta = delta
        self.metric_value = metric

    def evaluate(self, s):
        s = np.asarray(s)
        d = np.asarray(self.delta(s), dtype=float)
        return SimpleNamespace(north=d, east=np.zeros_like(s), offset=d,
                               metric=np.full_like(s, self.metric_value),
                               curvature=np.zeros_like(s))


class Map:
    def to_path(self, s):
        return np.asarray(s, dtype=float)


class Trajectory:
    def __init__(self, knots, delta, metric=1., reference=None):
        self.reference = reference if reference is not None else SourceReference(knots)
        self.spline = SimpleNamespace(control_distance=np.asarray(knots))
        self.station_map = Map()
        self.geometry = Geometry(delta, metric)

    def evaluate(self, station):
        return self.geometry.evaluate(station)

    def reference_geometry(self, station):
        return self.reference.evaluate(station)


class Solution:
    def __init__(self, delta, knots=(0., 1.), metric=1., speed=10., reference=None):
        self.trajectory = Trajectory(knots, delta, metric, reference=reference)
        self.state_evaluator = Evaluator(self.trajectory, speed)
        self.distance = np.asarray(knots)
        self.lap_time = 1.


def compare(f, g=lambda s: np.zeros_like(s), **kw):
    return compare_trajectory_states(Solution(f), Solution(g), **kw)


class NumericalConvergencePolicyTests(unittest.TestCase):
    def test_real_qss_evaluator_contract_returns_speed_mapping_and_geometry_separately(self):
        shared = SourceReference((0., 1.))
        left, right = Solution(lambda s: np.full_like(s, .001), reference=shared), Solution(lambda s: np.zeros_like(s), reference=shared)
        result = compare_trajectory_states(left, right, maximum_refinements=2)
        self.assertAlmostEqual(result["metrics"]["xy"]["rms"], .001, places=12)
        self.assertEqual(result["metrics"]["speed"]["max"], 0.0)
        self.assertIsNone(result["meets_accuracy_budget"])
        self.assertFalse(result["maximum_accuracy_certified"])

    def test_policy_hash_is_stable_and_proposed_not_approved(self):
        self.assertEqual(NUMERICAL_POLICY_V1.sha256, NUMERICAL_POLICY_V1.sha256)
        self.assertEqual(NUMERICAL_POLICY_V1.status, "proposed_pending_independent_review")
        self.assertEqual(NUMERICAL_POLICY_V1.requirements_status, "unapproved_assumptions_for_review")
        self.assertAlmostEqual(NUMERICAL_POLICY_V1.derivation["combined_contribution_m_s2"], .02340084001)

    def test_constant_difference_has_analytic_weighted_rms_independent_of_knots(self):
        left = Solution(lambda s: np.full_like(s, .003), knots=(0., .17, .6, 1.))
        right = Solution(lambda s: np.zeros_like(s), knots=(0., .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.))
        result = compare_trajectory_states(left, right, maximum_refinements=2)
        shared = SourceReference((0., .17, .6, 1.))
        left = Solution(lambda s: np.full_like(s, .003), knots=(0., .17, .6, 1.), reference=shared)
        right = Solution(lambda s: np.zeros_like(s), knots=(0., .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.), reference=shared)
        result = compare_trajectory_states(left, right, maximum_refinements=2)
        self.assertAlmostEqual(result["metrics"]["xy"]["rms"], .003, places=12)
        self.assertAlmostEqual(result["metrics"]["xy"]["max"], .003, places=12)
        self.assertIs(result["accepted"], False)

    def test_narrow_peak_fails_max_even_when_rms_is_small(self):
        def peak(s):
            return .03 * np.exp(-((s-.503)/.012)**2)
        shared = SourceReference((0., 1.))
        result = compare_trajectory_states(Solution(peak, reference=shared),
                                           Solution(lambda s: np.zeros_like(s), reference=shared),
                                           maximum_refinements=5, samples_per_interval=5)
        self.assertGreater(result["metrics"]["xy"]["max"], .02)
        self.assertLess(result["metrics"]["xy"]["rms"], .005)
        self.assertIs(result["accepted"], False)

    def test_unsampled_narrow_peak_cannot_be_certified_by_stable_grids(self):
        # All levels sample s=0 and s=1/(5*2^level); place a compact spike
        # halfway between finest-level points and much narrower than a cell.
        center = 1.0 / (5 * 2**5) * .5
        width = 1e-5
        def invisible_peak(s):
            return .03 * np.exp(-((s-center)/width)**2)
        shared = SourceReference((0., 1.))
        result = compare_trajectory_states(Solution(invisible_peak, reference=shared),
                                           Solution(lambda s: np.zeros_like(s), reference=shared),
                                           maximum_refinements=5, samples_per_interval=1)
        self.assertLess(result["metrics"]["xy"]["max"], 1e-6)
        self.assertIsNone(result["meets_accuracy_budget"])
        self.assertFalse(result["maximum_accuracy_certified"])
        self.assertFalse(result["continuous_envelope_certified"])

    def test_unresolved_sampling_is_incomplete_not_pass(self):
        shared = SourceReference((0., 1.))
        result = compare_trajectory_states(Solution(lambda s: .004*np.sin(71*s), reference=shared),
                                           Solution(lambda s: np.zeros_like(s), reference=shared),
                                           maximum_refinements=2, samples_per_interval=1)
        self.assertFalse(result["complete"])
        self.assertIsNone(result["meets_accuracy_budget"])
        self.assertFalse(result["accepted"])

    def test_exact_identical_fields_are_not_certified(self):
        shared = SourceReference((0., 1.))
        result = compare_trajectory_states(Solution(lambda s: np.zeros_like(s), reference=shared),
                                           Solution(lambda s: np.zeros_like(s), reference=shared),
                                           maximum_refinements=2)
        self.assertTrue(result["complete"])
        self.assertIsNone(result["meets_accuracy_budget"])
        self.assertFalse(result["maximum_accuracy_certified"])
        self.assertFalse(result["accepted"])

    def test_out_of_envelope_fails_closed_even_for_identical_paths(self):
        shared = SourceReference((0., 1.))
        left, right = Solution(lambda s: np.zeros_like(s), speed=30.1, reference=shared), Solution(lambda s: np.zeros_like(s), speed=30.1, reference=shared)
        result = compare_trajectory_states(left, right)
        self.assertFalse(result["meets_accuracy_budget"])
        self.assertIn("outside_consumer_operating_envelope", result["reason"])

    def test_same_endpoints_do_not_prove_different_source_identity(self):
        left = Solution(lambda s: np.zeros_like(s))
        left.trajectory.reference = SourceReference((0., 1.))
        right = Solution(lambda s: np.zeros_like(s))
        right.trajectory.reference = SourceReference((0., .5, 1.), source_delta=100.)
        right.trajectory.spline.control_distance = np.asarray([0., .5, 1.])
        result = compare_trajectory_states(left, right)
        self.assertFalse(result["meets_accuracy_budget"])
        self.assertEqual(result["reason"], "source_reference_mismatch")

    def test_unknown_frame_fails_closed_without_matching_source_proof(self):
        left = Solution(lambda s: np.zeros_like(s))
        right = Solution(lambda s: np.zeros_like(s))
        left.trajectory.reference.coordinate_frame = "unknown"
        right.trajectory.reference.coordinate_frame = "unknown"
        self.assertIn("source_identity_unverifiable", compare_trajectory_states(left, right)["reason"])

    def test_bad_domain_and_nonfinite_fail_closed(self):
        left, right = Solution(lambda s: np.zeros_like(s)), Solution(lambda s: np.zeros_like(s))
        right.trajectory.reference.end = .9
        self.assertFalse(compare_trajectory_states(left, right)["meets_accuracy_budget"])
        result = compare(lambda s: np.full_like(s, np.nan))
        self.assertFalse(result["meets_accuracy_budget"])

    def test_refinement_arguments_reject_boolean_and_too_small(self):
        with self.assertRaises(ValueError):
            compare(lambda s: np.zeros_like(s), maximum_refinements=True)
        with self.assertRaises(ValueError):
            compare(lambda s: np.zeros_like(s), maximum_refinements=1)


if __name__ == "__main__":
    unittest.main()
