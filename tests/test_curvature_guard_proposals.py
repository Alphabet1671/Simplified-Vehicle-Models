"""Focused tests for bounded, explicitly non-certified curvature proposals."""
import json
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers import curvature_guard_proposals as proposals
from simplified_vehicle_models.solvers.curvature_guard_proposals import (
    CurvatureProposalSettings, propose_curvature_guards,
)
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def trajectory(straight=False):
    s = np.linspace(0., 18., 31)
    north = s if straight else s + .14*np.sin(.7*s)
    east = .3*s**2/18 if straight else .3*s**2/18 + .4*np.sin(.41*s)
    heading = np.unwrap(np.arctan2(np.gradient(east, s), np.gradient(north, s)))
    k = np.gradient(heading, s)
    coords = PathCoordinates(s, north, east, heading, k, False)
    track = TrackProfile(s, np.diff(s), np.ones_like(s), k, False, "synthetic", coords)
    ref = ReferenceCurve(track)
    spline = OffsetSpline(track, 4.)
    coeff = np.zeros(spline.control_count) if straight else np.linspace(-.3, .45, spline.control_count)
    return OffsetTrajectory(ref, spline, coeff)


class CurvatureGuardProposalTest(unittest.TestCase):
    def assert_all_qualification_flags_false(self, result):
        for key in ("accepted", "all_accepted", "continuous_extrema_certified",
                    "physical_feasibility_certified", "global_minimum_proven",
                    "joint_line_optimized", "event_qualified", "U3_qualified",
                    "policy_approved"):
            self.assertIs(result[key], False, key)

    def test_real_geometry_candidates_and_json(self):
        t = trajectory()
        lo, hi = t.station_map.path_start, t.station_map.path_end
        result = propose_curvature_guards(t, [(lo, hi)], retained_stations=[hi*.4, lo])
        self.assertTrue(result["completed"])
        self.assertTrue(result["finite_public_eval_not_certified"])
        self.assert_all_qualification_flags_false(result)
        self.assertEqual(result["method"], "sampled_local_extremum_proposals_not_certified")
        self.assertTrue(any(p["proposal_kind"] == "local_max" for p in result["proposals"]))
        self.assertEqual([p["physical_station"] for p in result["proposals"]], sorted(p["physical_station"] for p in result["proposals"]))
        json.dumps(result, allow_nan=False)

    def test_flat_straight_endpoints_and_retained(self):
        t = trajectory(True)
        lo, hi = t.station_map.path_start, t.station_map.path_end
        result = propose_curvature_guards(t, [(lo, hi)], retained_stations=[hi*.5, lo])
        self.assertTrue(result["completed"])
        self.assertEqual(result["optimizer_calls"], 0)
        self.assertEqual({p["proposal_kind"] for p in result["proposals"]}, {"endpoint", "retained"})
        self.assertEqual(len(result["proposals"]), 3)

    def test_exact_budget_failures_and_interval_precheck(self):
        t = trajectory(True)
        lo, hi = t.station_map.path_start, t.station_map.path_end
        base = propose_curvature_guards(t, [(lo, hi)])
        exact = base["evaluations"]
        self.assertTrue(propose_curvature_guards(t, [(lo, hi)], settings=CurvatureProposalSettings(maximum_evaluations=exact))["completed"])
        fail = propose_curvature_guards(t, [(lo, hi)], settings=CurvatureProposalSettings(maximum_evaluations=exact-1))
        self.assertFalse(fail["completed"])
        calls = []
        original = t.evaluate
        t.evaluate = lambda x: (calls.append(x), original(x))[1]
        fail = propose_curvature_guards(t, [(lo, hi), (lo, hi)], settings=CurvatureProposalSettings(maximum_intervals=1))
        self.assertFalse(fail["completed"])
        self.assertEqual(calls, [])

    def test_candidate_budget_and_source_mutation(self):
        t = trajectory(True)
        lo, hi = t.station_map.path_start, t.station_map.path_end
        retained = np.linspace(lo, hi, 5)[1:-1]
        exact = CurvatureProposalSettings(maximum_candidates=5)
        result = propose_curvature_guards(t, [(lo, hi)], retained_stations=retained, settings=exact)
        self.assertTrue(result["completed"])
        self.assertEqual(len(result["physical_guard_stations"]), 5)
        self.assertEqual(result["physical_guard_stations"], sorted(result["physical_guard_stations"]))
        short = CurvatureProposalSettings(maximum_candidates=4)
        self.assertFalse(propose_curvature_guards(t, [(lo, hi)], retained_stations=retained, settings=short)["completed"])
        self.assert_all_qualification_flags_false(result)
        # Mutation at optimizer invocation is detected by the before/after source fingerprint.
        t = trajectory()
        lo, hi = t.station_map.path_start, t.station_map.path_end
        real_minimize = proposals.minimize_scalar
        def mutate(*args, **kwargs):
            t.coefficients.setflags(write=True)
            t.coefficients[0] += .001
            t.coefficients.setflags(write=False)
            return real_minimize(*args, **kwargs)
        with patch.object(proposals, "minimize_scalar", side_effect=mutate):
            changed = propose_curvature_guards(t, [(lo, hi)])
        self.assertFalse(changed["completed"])
        self.assertEqual(changed["reason"], "source_fingerprint_mismatch")

    def test_deadline_after_mapping_skips_geometry_evaluation(self):
        t = trajectory(True)
        lo, hi = t.station_map.path_start, t.station_map.path_end
        evaluate = t.evaluate
        t.evaluate = unittest.mock.Mock(wraps=evaluate)
        clock = [0.0]
        def monotonic():
            clock[0] += 0.6
            return clock[0]
        with patch.object(proposals, "_monotonic", side_effect=monotonic):
            with patch.object(t.station_map, "to_reference", wraps=t.station_map.to_reference) as mapped:
                result = propose_curvature_guards(t, [(lo, hi)],
                    settings=CurvatureProposalSettings(maximum_wall_seconds=1.0))
        self.assertFalse(result["completed"])
        self.assertEqual(result["reason"], "wall_budget_exceeded")
        self.assertEqual(mapped.call_count, 1)
        t.evaluate.assert_not_called()
        self.assert_all_qualification_flags_false(result)

    def test_bad_inputs_and_optimizer_failure_are_reports(self):
        t = trajectory(True)
        lo, hi = t.station_map.path_start, t.station_map.path_end
        for intervals in [[(lo, hi, 2)], [(lo, hi+1)], [(True, hi)], [(float('nan'), hi)]]:
            result = propose_curvature_guards(t, intervals)
            self.assertFalse(result["completed"])
            self.assert_all_qualification_flags_false(result)
            json.dumps(result, allow_nan=False)
        settings = CurvatureProposalSettings(maximum_evaluations=2)
        self.assertFalse(propose_curvature_guards(t, [(lo, hi)], settings=settings)["completed"])
        # A non-flat synthetic curve invokes the optimizer; failed optimizer must not certify.
        t = trajectory()
        lo, hi = t.station_map.path_start, t.station_map.path_end
        class Failed:
            success = False
            x = 1.
            fun = 0.
        with patch.object(proposals, "minimize_scalar", return_value=Failed()):
            result = propose_curvature_guards(t, [(lo, hi)])
        self.assertFalse(result["completed"])
        self.assertEqual(result["reason"], "optimizer_failed")
        json.dumps(result, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
