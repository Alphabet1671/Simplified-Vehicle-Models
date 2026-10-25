"""Yaw-aware fixed-path profile API and reduction tests."""
import unittest

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import (
    YawPath, YawProfileTransitionError, curvature_rate_from_samples, to_speed_plan, yaw_aware_profile,
)


def yaw_map(*, into=1.e9, out=1.e9, ax=(-10., 0., 10.), ay=(-20., 0., 20.),
            feasible=None, inertia=1.):
    speed = np.linspace(0., 30., 61)
    shape = (speed.size, len(ax), len(ay))
    mask = np.ones(shape, dtype=bool) if feasible is None else np.asarray(feasible, bool)
    return YawCapabilityMap(
        speed, np.asarray(ax), np.asarray(ay), np.full(shape, into),
        np.full(shape, out), mask, inertia, "physical_no_tv_v1", "synthetic-test-map",
    )


class YawPathTests(unittest.TestCase):
    def test_requires_explicit_physical_distance_and_valid_arrays(self):
        with self.assertRaisesRegex(ValueError, "start at zero"):
            YawPath(np.array([1., 2., 3.]), np.zeros(3), np.zeros(3), False)
        with self.assertRaisesRegex(ValueError, "finite"):
            YawPath(np.array([0., 1., 2.]), np.array([0., np.nan, 0.]), np.zeros(3), False)

    def test_nonuniform_clothoid_rate_and_periodic_seam(self):
        s = np.linspace(0., 20., 101)
        k = .01 + .002 * s
        rate = curvature_rate_from_samples(s, k)
        np.testing.assert_allclose(rate[2:-2], .002, atol=1e-12)
        closed_s = np.linspace(0., 10., 101)
        closed_k = np.cos(2. * np.pi * closed_s / 10.)
        periodic = curvature_rate_from_samples(closed_s, closed_k, is_closed=True)
        self.assertAlmostEqual(periodic[0], periodic[-1], places=14)


class YawProfileTests(unittest.TestCase):
    def test_transition_failure_has_immutable_numerical_diagnostics(self):
        distance = np.array([0., 1., 2.])
        path = YawPath(distance, np.zeros(3), np.zeros(3), False)
        mask = np.zeros((61, 3, 3), dtype=bool)
        mask[:, 1, :] = True  # station caps exist only on the exact zero-ax slice
        cap = yaw_map(ax=(-1., 0., 1.), feasible=mask)
        with self.assertRaises(YawProfileTransitionError) as caught:
            yaw_aware_profile(path, cap, boundary="standing", maximum_speed=10.)
        error = caught.exception
        diag = error.diagnostics
        self.assertEqual(diag["direction"], "forward")
        self.assertEqual(diag["segment_index"], 0)
        self.assertEqual(diag["fixed_speed"], 0.)
        self.assertGreater(len(diag["candidate_endpoint_speeds"]), 0)
        self.assertEqual(diag["ds"], 1.)
        self.assertEqual(diag["midpoint_curvature"], 0.)
        self.assertEqual(diag["midpoint_curvature_rate"], 0.)
        self.assertTrue(all("required_ax" in row and "reason" in row for row in diag["candidates"]))
        self.assertTrue(any(row["reason"] == "masked_support" for row in diag["candidates"]))
        with self.assertRaises(TypeError):
            diag["direction"] = "backward"
        with self.assertRaises(AttributeError):
            error.diagnostics = {}

    def test_constant_radius_reduces_to_steady_cap_when_yaw_is_nonbinding(self):
        distance = np.linspace(0., 100., 101)
        k = np.full_like(distance, .02)
        path = YawPath(distance, k, np.zeros_like(k), False)
        result = yaw_aware_profile(path, yaw_map(), boundary="standing", maximum_speed=20.)
        self.assertTrue(result.converged)
        self.assertEqual(result.speed[0], 0.)
        self.assertLessEqual(result.speed.max(), 20.)
        self.assertGreater(result.speed[-1], 0.)

    def test_fine_grid_transition_uses_projected_acceleration_candidates(self):
        # A 0.05 m segment with finite +/-1 m/s^2 capability has a launch
        # endpoint of only sqrt(2*ds)=0.316 m/s. That value is absent from
        # the coarse 80-point scan over a 20 m/s range.
        distance = np.arange(0., 2.0001, .05)
        path = YawPath(distance, np.zeros_like(distance), np.zeros_like(distance), False)
        cap = yaw_map(ax=(-1., 0., 1.))
        result = yaw_aware_profile(path, cap, boundary="standing", maximum_speed=20.)
        self.assertEqual(result.speed[0], 0.)
        self.assertGreater(result.speed[1], 0.)
        self.assertGreater(result.speed[-1], result.speed[1])
        self.assertTrue(np.all(np.diff(result.speed) >= -1e-10))
        self.assertLessEqual(np.max(np.abs(result.longitudinal_acceleration)), 1. + 1e-8)

    def test_explicit_lower_speed_witness_passes_strict_oracle_on_fine_grid(self):
        from simplified_vehicle_models.solvers.yaw_aware_profile import _MapInterpolator, _segment_acceleration
        distance = np.arange(0., .5001, .05)
        curvature = np.linspace(.01, .08, distance.size)
        rate = np.gradient(curvature, distance)
        path = YawPath(distance, curvature, rate, False)
        speed = np.linspace(0., 8., 17)
        ax = np.linspace(-2., 2., 9)
        ay = np.linspace(-2., 2., 17)
        shape = (speed.size, ax.size, ay.size)
        # Lateral support shrinks with speed, while a slower explicit profile
        # remains inside the map. This is a witness, not a claim about the old
        # greedy profile's physical failure modes.
        mask = np.ones(shape, dtype=bool)
        for i, v in enumerate(speed):
            allowed = max(.25, 1.25 - .12 * v)
            mask[i, :, np.abs(ay) > allowed] = False
        capability = YawCapabilityMap(speed, ax, ay, np.full(shape, 1.e6),
            np.full(shape, 1.e6), mask, 1., "physical_no_tv_v1", "shrinking-lateral-support")
        witness = np.sqrt(2. * .5 * distance)
        oracle = _MapInterpolator(capability)
        for i, ds in enumerate(np.diff(distance)):
            self.assertTrue(_segment_acceleration(oracle, path, witness[i], witness[i + 1], i, ds), i)

    def test_lateral_axis_bound_precedes_speed_cap_scan_on_fine_grid(self):
        distance = np.arange(0., 2.0001, .05)
        curvature = np.linspace(.10, .15304, distance.size)
        path = YawPath(distance, curvature, np.gradient(curvature, distance), False)
        speed = np.linspace(0., 12., 49)
        ax = np.array([-10., 0., 10.])
        ay = np.linspace(-9.80665, 9.80665, 21)
        shape = (speed.size, ax.size, ay.size)
        cap = YawCapabilityMap(speed, ax, ay, np.full(shape, 1.e9),
                               np.full(shape, 1.e9), np.ones(shape, bool), 1.,
                               "physical_no_tv_v1", "synthetic-fsc-ay-limited")
        result = yaw_aware_profile(path, cap, boundary="standing", maximum_speed=12.)
        station_bound = np.sqrt(ay[-1] / curvature)
        self.assertTrue(np.all(result.speed <= station_bound + 1e-8))
        self.assertTrue(np.all(np.isfinite(result.speed)))
        self.assertGreater(result.speed[-1], 0.)

    def test_clothoid_yaw_moment_caps_speed(self):
        distance = np.linspace(0., 20., 41)
        k = .002 * distance
        kp = np.full_like(k, .002)
        # Closed is false, rolling speed is outside scope; use standing start.
        path = YawPath(distance, k, kp, False)
        # yaw demand for ax=0 at v=10 is I*v^2*k' = .2 N m.
        cap = yaw_map(into=.2, out=.2, inertia=1.)
        result = yaw_aware_profile(path, cap, boundary="standing", maximum_speed=20.)
        self.assertLessEqual(result.speed.max() ** 2 * .002, .2 + .06)
        self.assertGreater(result.speed.max(), 0.)

    def test_standing_start_requires_zero_in_map_speed_grid(self):
        distance = np.linspace(0., 10., 11)
        path = YawPath(distance, np.full(11, .01), np.zeros(11), False)
        source = yaw_map()
        no_rest = YawCapabilityMap(
            source.speed[1:], source.longitudinal_acceleration, source.lateral_acceleration,
            source.yaw_moment_into_turn[1:], source.yaw_moment_out_of_turn[1:],
            source.feasible_mask[1:], source.yaw_inertia, source.source_kind, source.source_identity,
        )
        with self.assertRaisesRegex(ValueError, "speed grid includes zero"):
            yaw_aware_profile(path, no_rest, boundary="standing")

    def test_infeasible_map_is_rejected_and_masked_corners_not_bridged(self):
        distance = np.linspace(0., 10., 11)
        path = YawPath(distance, np.full(11, .01), np.zeros(11), False)
        mask = np.zeros((61, 3, 3), dtype=bool)
        with self.assertRaisesRegex(ValueError, "no feasible speed"):
            yaw_aware_profile(path, yaw_map(feasible=mask), boundary="standing")

    def test_closed_and_standing_boundary_semantics(self):
        distance = np.linspace(0., 20., 21)
        k = np.full_like(distance, .01)
        closed = YawPath(distance, k, np.zeros_like(k), True)
        lap = yaw_aware_profile(closed, yaw_map(), boundary="flying", maximum_speed=10.)
        self.assertAlmostEqual(lap.speed[0], lap.speed[-1], places=12)
        opened = YawPath(distance, k, np.zeros_like(k), False)
        standing = yaw_aware_profile(opened, yaw_map(), boundary="standing", maximum_speed=10.)
        self.assertEqual(standing.speed[0], 0.)
        with self.assertRaises(ValueError):
            yaw_aware_profile(opened, yaw_map(), boundary="flying")
        with self.assertRaises(ValueError):
            yaw_aware_profile(closed, yaw_map(), boundary="standing")

    def test_speed_plan_conversion_preserves_grid_curvature_and_closure(self):
        distance = np.linspace(0., 20., 21)
        k = np.full_like(distance, .01)
        path = YawPath(distance, k, np.zeros_like(k), False)
        profile = yaw_aware_profile(path, yaw_map(), boundary="standing", maximum_speed=10.)
        plan = to_speed_plan(path, profile)
        self.assertFalse(plan.is_closed)
        np.testing.assert_array_equal(plan.distance, path.distance)
        np.testing.assert_array_equal(plan.curvature, path.curvature)
        np.testing.assert_array_equal(plan.speed, profile.speed)
        with self.assertRaises(ValueError):
            to_speed_plan(path, type(profile)(profile.distance + 1, profile.speed,
                            profile.longitudinal_acceleration, profile.elapsed_time,
                            profile.lap_time, True, profile.iterations,
                            profile.maximum_speed_change, profile.diagnostics))


if __name__ == "__main__":
    unittest.main()
