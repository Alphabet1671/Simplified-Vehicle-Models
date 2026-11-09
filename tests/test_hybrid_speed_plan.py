import unittest
import numpy as np

from simplified_vehicle_models.solvers.hybrid_speed_plan import HybridSpeedPlan, build_hybrid_speed_plan
from simplified_vehicle_models.solvers.longitudinal_capability import LongitudinalCapability, CapabilityQuery, CurrentVehicleCapability


class ConstantProvider:
    identity = "constant-provider-v1"

    def evaluate(self, query):
        return LongitudinalCapability(2.0, -0.2, -4.0, True, (), self.identity)


class VariableProvider:
    def __init__(self, identity="variable-v1", drive=None, brake=4., invalid=None, yaw_scale=0.):
        self.identity = identity
        self.allocator_identity = identity
        self.drive = drive or (lambda speed, curvature: 2.)
        self.brake = brake
        self.invalid = invalid
        self.yaw_scale = yaw_scale
        self.queries = []

    def evaluate(self, query):
        self.queries.append(query)
        ok = self.invalid is None or not self.invalid(query)
        return LongitudinalCapability(self.drive(query.speed_mps, query.curvature_inv_m)-abs(query.yaw_moment_request_nm)*self.yaw_scale,
                                      -.2, -self.brake, ok,
                                      () if ok else ("synthetic_domain",), self.identity)


class HybridSpeedPlanTests(unittest.TestCase):
    def test_constant_straight_respects_drive_brake_and_corner_caps(self):
        x = np.array([0., 3., 11., 20., 30.])
        caps = np.array([20., 20., 12., 20., 20.])
        plan = build_hybrid_speed_plan(x, np.zeros_like(x), caps, ConstantProvider(),
                                       initial_speed_mps=5., storage_soc=.5, is_closed=False)
        q = plan.speed_mps ** 2
        ds = np.diff(x)
        self.assertTrue(np.all(np.diff(q) <= 2*2.*ds + 1e-8))
        self.assertTrue(np.all(q[:-1] <= q[1:] + 2*.60*4.*ds + 1e-8))
        self.assertTrue(np.all(plan.speed_mps <= caps + 1e-10))
        for name in ("distance_m", "speed_mps", "acceleration_mps2", "corner_speed_cap_mps",
                     "braking_speed_bound_mps", "binding_index", "curvature_inv_m"):
            self.assertFalse(getattr(plan, name).flags.writeable, name)

    def test_closed_seam_is_canonical_and_periodic(self):
        x = np.array([0., 4., 13., 21., 30.])
        caps = np.array([18., 18., 10., 18., 18.])
        plan = build_hybrid_speed_plan(x, np.array([0., .01, .2, -.01, 0.]), caps,
                                       ConstantProvider(), initial_speed_mps=6., storage_soc=.4,
                                       is_closed=True)
        self.assertEqual(plan.distance_m[0], 0.)
        self.assertAlmostEqual(plan.distance_m[-1], 30.)
        self.assertAlmostEqual(plan.speed_mps[0], plan.speed_mps[-1])
        self.assertAlmostEqual(plan.curvature_inv_m[0], plan.curvature_inv_m[-1])
        self.assertEqual(plan.sample(30.).station_m, 0.)
        self.assertEqual(plan.sample(-1.).station_m, 29.)

    def test_open_initial_speed_is_not_silently_clipped(self):
        with self.assertRaisesRegex(ValueError, "initial.*cap|infeasible.*initial"):
            build_hybrid_speed_plan([0., 1., 20.], [0., .1, 0.], [20., 1., 20.],
                                    ConstantProvider(), initial_speed_mps=8., storage_soc=.5,
                                    is_closed=False)

    def test_end_mid_endpoint_queries_signed_curvature_and_variable_authority(self):
        p = VariableProvider(drive=lambda v, k: 1. + .03*v + 2*k)
        x = [0., 6., 13.]
        k = [-.08, .02, .11]
        plan = build_hybrid_speed_plan(x, k, [20., 20., 20.], p,
                                       initial_speed_mps=4., storage_soc=.5, is_closed=False)
        q0, q1 = plan.speed_mps[:2]**2
        expected = {(-.08), (-.03), .02}
        seen = {round(q.curvature_inv_m, 8) for q in p.queries}
        self.assertTrue(expected.issubset(seen))
        self.assertTrue(all(q.soc == .5 for q in p.queries))
        self.assertLessEqual(q1-q0, 2*min(1.+.03*q.speed_mps+2*q.curvature_inv_m for q in p.queries if q.curvature_inv_m in expected)*6+1e-8)
        self.assertLessEqual(q0-q1, 2*.60*4.*6+1e-8)

    def test_provider_domain_failure_at_sampled_midpoint_fails_closed(self):
        p = VariableProvider(invalid=lambda q: q.curvature_inv_m == .05)
        with self.assertRaisesRegex(ValueError, "outside domain"):
            build_hybrid_speed_plan([0., 10., 20.], [0., .1, .1], [12., 12., 12.], p,
                                    initial_speed_mps=4., storage_soc=.5, is_closed=False)

    def test_out_of_domain_trial_is_skipped_when_accepted_samples_are_valid(self):
        p = VariableProvider(drive=lambda v, k: 1., invalid=lambda q: 3. < q.speed_mps < 4.)
        plan = build_hybrid_speed_plan([0., 10., 20.], [0., 0., 0.], [40., 40., 40.], p,
                                       initial_speed_mps=1., storage_soc=.5, is_closed=False)
        self.assertGreater(plan.speed_mps[1], 1.)
        self.assertLess(plan.speed_mps[1], 8.)
        self.assertTrue(any(q.speed_mps > 8. for q in p.queries))

    def test_negative_drive_is_preserved_not_clamped(self):
        positive = VariableProvider(drive=lambda v,k: -.1)
        plan = build_hybrid_speed_plan([0., 10., 20.], [0., 0., 0.], [20., 20., 20.], positive,
                                       initial_speed_mps=3., storage_soc=.5, is_closed=False)
        self.assertLess(plan.speed_mps[1], plan.speed_mps[0])
        self.assertLess(plan.acceleration_mps2[0], 0.)
        with self.assertRaisesRegex(ValueError, "negative net drive"):
            build_hybrid_speed_plan([0., 100., 200.], [0., 0., 0.], [20., 20., 20.], positive,
                                    initial_speed_mps=3., storage_soc=.5, is_closed=False)
        variable = VariableProvider(drive=lambda v,k: -1. if v < 4. else 1.)
        self.assertIsNotNone(build_hybrid_speed_plan([0., 1., 2.], [0., 0., 0.], [20., 20., 20.], variable,
                                                      initial_speed_mps=4., storage_soc=.5, is_closed=False))

    def test_increasing_authority_above_zero_start_is_examined_but_not_assumed(self):
        # A zero start sample bounds sampled minimum authority at zero, so no
        # positive q can satisfy the declared inequality. Still prove the
        # candidate policy scans the finite cap interval rather than extrapolating
        # an unsupported start-only interval, then fails closed truthfully.
        p = VariableProvider(drive=lambda v, k: 3. if v > 0.05 else 0.)
        with self.assertRaisesRegex(ValueError, "infeasible positive-speed continuation"):
            build_hybrid_speed_plan([0., 10., 20.], [0., 0., 0.], [10., 10., 10.], p,
                                    initial_speed_mps=0., storage_soc=.5, is_closed=False)
        self.assertTrue(any(query.speed_mps > 5. for query in p.queries))

    def test_forward_reachable_edge_avoids_unreachable_cap_query_and_is_budgeted(self):
        # Start state has positive authority, while a far cap-speed query has
        # strongly negative authority. The planner must find local reachability
        # without installing/querying the cap as an attained endpoint first.
        p = VariableProvider(drive=lambda v, k: 3.0 if v < 8.0 else -20.0)
        plan = build_hybrid_speed_plan([0., 10., 20.], [0., 0., 0.], [40., 40., 40.], p,
                                       initial_speed_mps=1., storage_soc=.5, is_closed=False)
        self.assertGreater(plan.speed_mps[1]**2, 1.)
        # Forward candidate scans are performed before backward-ceiling work;
        # no cap-speed query may contaminate the reachable-edge proposal.
        self.assertFalse(any(q.speed_mps == 40. for q in p.queries[:40]))
        # 48 scan + two explicit candidates + eight refinements + one final
        # verification: hard total <=59 per edge/iteration, independent of cache.
        self.assertLessEqual(plan.candidate_edge_evaluations, (4*3+8) * 2 * 59)
        self.assertLessEqual(len(p.queries), 3 * (4*3+8) * 2 * 59 + 3*3*3)

    def test_zero_start_without_positive_continuation_fails_closed(self):
        p = VariableProvider(drive=lambda v, k: 0.)
        with self.assertRaisesRegex(ValueError, "infeasible positive-speed continuation"):
            build_hybrid_speed_plan([0., 1., 2.], [0., 0., 0.], [10., 10., 10.], p,
                                    initial_speed_mps=0., storage_soc=.5, is_closed=False)

    def test_zero_start_with_positive_authority_continues(self):
        p = VariableProvider(drive=lambda v, k: 1.)
        plan = build_hybrid_speed_plan([0., 1., 2.], [0., 0., 0.], [10., 10., 10.], p,
                                       initial_speed_mps=0., storage_soc=.5, is_closed=False)
        self.assertGreater(plan.speed_mps[1]**2, 0.)

    def test_nonmonotone_feasible_island_search_is_bounded_not_maximal(self):
        # Feasibility has a narrow higher-q island between grid probes. The
        # contract intentionally does not promise its recovery or global max.
        p = VariableProvider(drive=lambda v, k: 2. if 4.6 < v < 4.9 else (-5. if v > 4.9 else 1.))
        try:
            plan = build_hybrid_speed_plan([0., 1., 2.], [0., 0., 0.], [10., 10., 10.], p,
                                           initial_speed_mps=1., storage_soc=.5, is_closed=False)
        except ValueError:
            # Fail-closed is an allowed bounded-search outcome.
            self.assertLessEqual(len(p.queries), 2 * (33 + 8) * 3 * (4*3+8))
        else:
            # A returned endpoint is verified feasible, without asserting it is
            # the globally maximal branch.
            q1 = plan.speed_mps[1]**2
            self.assertLessEqual(q1, 1. + 2.*1. + 1e-7)

    def test_backward_ceiling_verified_and_future_event_metadata(self):
        p = ConstantProvider()
        open_plan = build_hybrid_speed_plan([0., 5., 10., 15.], [0., 0., 0., 0.], [12., 12., 6., 12.], p,
                                            initial_speed_mps=4., storage_soc=.5, is_closed=False)
        sample = open_plan.sample(2.)
        self.assertEqual(sample.binding_index, 2)
        self.assertGreater(sample.distance_to_binding_event_m, 0.)
        self.assertLessEqual(open_plan.braking_speed_bound_mps[1]**2,
                             open_plan.braking_speed_bound_mps[2]**2 + 2*.60*4.*5. + 1e-8)
        self.assertTrue(np.any(open_plan.speed_mps < open_plan.braking_speed_bound_mps-1e-8))
        ring = build_hybrid_speed_plan([0., 5., 10., 15., 20.], [0., .01, .02, -.01, 0.],
                                       [6., 15., 15., 15., 6.], p, initial_speed_mps=4.,
                                       storage_soc=.5, is_closed=True)
        self.assertGreater(ring.sample(18.).distance_to_binding_event_m, 0.)

    def test_cached_lookup_metadata_is_immutable_and_per_segment(self):
        p = VariableProvider(drive=lambda v,k: 2., brake=4.)
        plan = build_hybrid_speed_plan([0.,20.,40.,57.,77.,100.], [0.]*6,
            [22.,22.,22.,22.,22.,10.], p, initial_speed_mps=20., storage_soc=.5, is_closed=False)
        self.assertEqual(len(plan._next_braking_onset), len(plan.distance_m))
        self.assertEqual(len(plan._future_candidates), len(plan.distance_m))
        self.assertIsInstance(plan._future_candidates[0], tuple)
        with self.assertRaises((AttributeError, TypeError)):
            plan._future_candidates = ()

    def test_closed_current_braking_segment_is_considered_after_full_wrap(self):
        # The baseline traverses all unique offsets, considering the current
        # braking segment again after one lap when it binds to another node.
        x = [0., 10., 20., 30.]
        q = np.array([100., 80., 100., 100.])
        speed = np.sqrt(q)
        segment_accel = np.diff(q) / (2*np.diff(x))
        accel = np.r_[segment_accel, segment_accel[-1]]
        binding = np.array([1, 1, 2, 0])
        plan = HybridSpeedPlan(np.array(x), speed, accel, np.full(4, 20.),
            np.full(4, 20.), binding, np.zeros(4), True, .5, .6, "p", "h",
            "id", "ok", 1, (), "v", 5, 1e-8)
        sample = plan.sample(5.)
        self.assertEqual(sample.future_event_index, 1)
        self.assertEqual(sample.future_event_station_m, 10.)
        self.assertEqual(sample.distance_to_future_event_m, 5.)

    def test_future_event_is_separate_from_local_self_binding_and_closed_wrap(self):
        p = VariableProvider(drive=lambda v,k: 2., brake=4.)
        plan = build_hybrid_speed_plan([0.,20.,40.,57.,77.,100.], [0.]*6,
            [22.,22.,22.,22.,22.,10.], p, initial_speed_mps=20., storage_soc=.5, is_closed=False)
        sample = plan.sample(0.)
        self.assertEqual(sample.binding_index, 0)
        self.assertEqual(sample.distance_to_binding_event_m, 0.)
        self.assertEqual(sample.future_event_station_m, 100.)
        self.assertAlmostEqual(sample.distance_to_future_event_m, 100.)
        self.assertAlmostEqual(sample.future_event_speed_cap_mps, 10.)
        self.assertIsNotNone(sample.braking_onset_station_m)
        self.assertLess(sample.braking_onset_station_m, sample.future_event_station_m)
        self.assertEqual(plan.sample(100.).future_event_station_m, None)

        ring = build_hybrid_speed_plan([0.,20.,40.,60.,80.], [0.]*5, [22.,22.,10.,22.,22.],
            p, initial_speed_mps=20., storage_soc=.5, is_closed=True)
        wrapped = ring.sample(75.)
        self.assertEqual(wrapped.future_event_station_m, 40.)
        self.assertAlmostEqual(wrapped.distance_to_future_event_m, 45.)
        seam = ring.sample(79.)
        self.assertEqual(seam.future_event_station_m, 40.)
        self.assertAlmostEqual(seam.distance_to_future_event_m, 41.)
        self.assertEqual(ring.algorithm_version, "hybrid-speed-plan-v6-cap-bounded-trial-search")

    def test_identity_sensitivity_and_hard_readonly(self):
        args=dict(distance_m=[0., 5., 10.], curvature_inv_m=[0., .01, 0.], corner_speed_cap_mps=[12., 8., 12.],
                  initial_speed_mps=4., storage_soc=.5, is_closed=False)
        p=VariableProvider()
        base=build_hybrid_speed_plan(**args, capability_provider=p)
        variants=[build_hybrid_speed_plan(**(args|{"storage_soc":.6}), capability_provider=p),
                  build_hybrid_speed_plan(**(args|{"brake_margin":.5}), capability_provider=p),
                  build_hybrid_speed_plan(**(args|{"is_closed":True,"corner_speed_cap_mps":[12.,8.,12.]}), capability_provider=p),
                  build_hybrid_speed_plan(**(args|{"curvature_inv_m":[0.,.02,0.]}), capability_provider=p),
                  build_hybrid_speed_plan(**args, capability_provider=VariableProvider(identity="other"))]
        self.assertTrue(all(v.identity != base.identity for v in variants))
        self.assertEqual(base.algorithm_version, "hybrid-speed-plan-v6-cap-bounded-trial-search")
        for arr in (base.distance_m, base.speed_mps, base.binding_index):
            with self.assertRaises(ValueError): arr.setflags(write=True)

    def test_yaw_provider_must_have_matching_execution_identity_and_affects_plan(self):
        p=VariableProvider(identity="tv-id", yaw_scale=.02)
        kwargs=dict(distance_m=[0., 5., 10.], curvature_inv_m=[0., .01, 0.], corner_speed_cap_mps=[12.,12.,12.],
                    initial_speed_mps=4., storage_soc=.5, is_closed=False)
        with self.assertRaisesRegex(ValueError, "execution allocator"):
            build_hybrid_speed_plan(**kwargs, capability_provider=p, yaw_moment_request_nm=10.)
        plain=build_hybrid_speed_plan(**kwargs, capability_provider=p)
        tv=build_hybrid_speed_plan(**kwargs, capability_provider=p, yaw_moment_request_nm=10., execution_allocator_identity="tv-id")
        self.assertTrue(any(q.yaw_moment_request_nm == 10. for q in p.queries))
        self.assertTrue(np.all(tv.speed_mps <= plain.speed_mps+1e-9))

    def test_max_iterations_one_fails_when_not_converged(self):
        with self.assertRaisesRegex(RuntimeError, "failed to converge"):
            build_hybrid_speed_plan([0., 1., 3., 6.], [0., 0., 0., 0.], [30., 30., 10., 30.], ConstantProvider(),
                                    initial_speed_mps=2., storage_soc=.5, is_closed=True, max_iterations=1)

    def test_invalid_grids_and_seam_rejected(self):
        p = ConstantProvider()
        for x in ([0., 1., 1.], [0., 2., 1.], [0., 1.]):
            with self.subTest(x=x), self.assertRaises(ValueError):
                build_hybrid_speed_plan(x, np.zeros(len(x)), np.full(len(x), 10.), p,
                                        initial_speed_mps=2., storage_soc=.5, is_closed=True)
        with self.assertRaisesRegex(ValueError, "endpoint|seam|duplicate"):
            build_hybrid_speed_plan([0., 5., 10.], [0., .1, .2], [10., 10., 11.], p,
                                    initial_speed_mps=2., storage_soc=.5, is_closed=True)


if __name__ == "__main__":
    unittest.main()
