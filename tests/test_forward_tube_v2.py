"""Synthetic unit tests for the NON-BINDING tube diagnostic v2.

No real lap data and no ODE solves: levels are hand-built piecewise
polynomials written in the same ``dense_polynomials.npz`` layout.
"""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools import check_iem24_forward_tube_v2 as v2

N = len(v2.NAMES)


def build(attempts, steering_limit, certify, source_check=lambda hashes: None):
    """v2.build with the working-file source check stubbed (synthetic fixtures carry fake hashes)."""
    return v2.build(attempts, steering_limit, certify, source_check=source_check)

ROOT = Path(__file__).resolve().parents[1]
V1_SHA256 = "57b92964f3cc13057a230f7c72e866e2df6a7742d6c4931e27aaec511ee200ab"


def make_attempt(root, name, step, rtol, offsets=None, slopes=None, intervals=10, horizon=1.,
                 hashes=None, settings_extra=None, completed=True, nan=False, bad_bounds=None,
                 write_dense=True):
    """Piecewise-linear dense run: x_k(t) = offset_k + slope_k * t on every interval."""
    directory = Path(root) / name
    directory.mkdir(parents=True)
    offsets = np.zeros(N) if offsets is None else np.array(offsets, float)
    slopes = np.zeros(N) if slopes is None else np.array(slopes, float)
    # Components left exactly flat at zero would sit on an event threshold (the
    # protocol counts |x - threshold| <= 1e-9 as a crossing), so give untouched
    # wheel speeds and the pedal an off-threshold baseline.
    for k in (*v2.WHEELS, v2.PEDAL):
        if offsets[k] == 0. and slopes[k] == 0.:
            offsets[k] = 10. if k in v2.WHEELS else .5
    times = np.linspace(0., horizon, intervals + 1)
    t_end = times[1:]
    h = np.diff(times)
    coefficients = np.zeros((intervals, 6, N))
    coefficients[:, 0, :] = offsets[None, :] + slopes[None, :] * t_end[:, None]
    coefficients[:, 1, :] = slopes[None, :] * h[:, None]
    if nan:
        coefficients[0, 0, 0] = np.nan
    value_lo = np.minimum(offsets + slopes * 0., offsets + slopes * horizon)
    value_hi = np.maximum(offsets + slopes * 0., offsets + slopes * horizon)
    bounds = np.stack([np.tile(value_lo, (intervals, 1)), np.tile(value_hi, (intervals, 1))], axis=-1)
    if bad_bounds is not None:
        bounds = bad_bounds
    settings = {"maximum_step": step, "rtol": rtol, "track": "t.xml", "planner": "live", "spacing": .5,
                "maximum_speed": 60., "lateral_acceleration": None, "pedal_travel_time": .175,
                "method": "BDF", "warm_start": "cold_attempt"}
    settings.update(settings_extra or {})
    summary = {"lap_completed": completed, "usable": False, "refinement_validated": False,
               "between_step_validation_complete": False,
               "source_hashes": hashes if hashes is not None else {"a": "1", "b": "2"},
               "settings": settings}
    if write_dense:
        summary["dense_polynomials_file"] = "dense_polynomials.npz"
        np.savez_compressed(directory / "dense_polynomials.npz", times=times, t_end=t_end, step=h,
                            coefficients=coefficients, bounds=bounds)
    (directory / "summary.json").write_text(json.dumps(summary))
    return directory


def offsets_at(component, value):
    out = np.zeros(N)
    out[v2.NAMES.index(component)] = value
    return out


def accepting_certify(record=None):
    def certify(bounds):
        if record is not None:
            record.append(np.array(bounds))
        return {"certified": True, "failed": [], "margins": {"m": 1.}, "scope": "stub"}
    return certify


def three_levels(root, errors=(0.4, 0.2, 0.1), component="v_x", **extra):
    """Levels differ from a common reference by a constant ``errors`` offset on one component."""
    names = ("c", "m", "f")
    steps = (.02, .01, .005)
    rtols = (1e-5, 5e-6, 2.5e-6)
    return [make_attempt(root, n, s, r, offsets_at(component, e), **extra)
            for n, s, r, e in zip(names, steps, rtols, errors)]


class TubeScaleTests(unittest.TestCase):
    def test_tube_is_ten_times_componentwise_max_without_ratio_or_cap(self):
        with tempfile.TemporaryDirectory() as root:
            # d_cm = 0.2, d_mf = 0.3 -> ratio 1.5 (>= 1, v1 would cap it): no convergence.
            c, m, f = three_levels(root, errors=(0.0, 0.2, 0.5))
            result = build([c, m, f], 0.5, accepting_certify())
        tube = result["tube_T_diagnostic_scale"]
        self.assertAlmostEqual(tube["v_x"], 10. * 0.3)
        self.assertEqual(tube["s"], 0.)
        self.assertIn("NOT a true ODE error bound", result["tube_T_definition"])
        self.assertIn("v_x", result["empirical_convergence_test"]["observed_ratio_mf_over_cm"])
        self.assertAlmostEqual(result["empirical_convergence_test"]["observed_ratio_mf_over_cm"]["v_x"], 1.5)
        self.assertFalse(result["empirical_convergence_test"]["shrink_cm_mf_all"]["v_x"])
        self.assertIn("s", result["empirical_convergence_test"]["degenerate_zero_components"])

    def test_tube_takes_larger_of_the_two_differences(self):
        with tempfile.TemporaryDirectory() as root:
            c, m, f = three_levels(root, errors=(1.0, 0.2, 0.1))  # d_cm = .8, d_mf = .1
            result = build([c, m, f], 0.5, accepting_certify())
        self.assertAlmostEqual(result["tube_T_diagnostic_scale"]["v_x"], 8.)

    def test_domain_test_receives_bounds_widened_by_twice_tube(self):
        calls = []
        with tempfile.TemporaryDirectory() as root:
            c, m, f = three_levels(root, errors=(1.0, 0.2, 0.1))
            result = build([c, m, f], 0.5, accepting_certify(calls))
        plain, wide = calls
        k = v2.NAMES.index("v_x")
        tube = result["tube_T_diagnostic_scale"]["v_x"]
        np.testing.assert_allclose(wide[:, k, 0], plain[:, k, 0] - 2. * tube)
        np.testing.assert_allclose(wide[:, k, 1], plain[:, k, 1] + 2. * tube)
        j = v2.NAMES.index("n")
        np.testing.assert_allclose(wide[:, j], plain[:, j])
        self.assertTrue(result["domain_margin_test_widened_2T"]["holds"])
        self.assertIn("NOT a convergence test", result["domain_margin_test_widened_2T"]["label"])

    def test_domain_test_failure_is_reported_independently_of_convergence(self):
        def certify(bounds):
            return {"certified": bool(np.max(bounds[:, 3, 1]) < 3.), "failed": ["x"], "margins": {"m": 1.}}
        with tempfile.TemporaryDirectory() as root:
            c, m, f = three_levels(root, errors=(1.0, 0.2, 0.1))  # T = 8 -> widened v_x hi = 16 + .1
            result = build([c, m, f], 0.5, certify)
        self.assertFalse(result["domain_margin_test_widened_2T"]["holds"])
        self.assertTrue(result["empirical_convergence_test"]["shrink_cm_mf_all"]["v_x"])

    def test_no_combined_pass_and_no_acceptance_claim(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, accepting_certify())
        for forbidden in ("passed", "usable", "certified", "accepted"):
            self.assertNotIn(forbidden, result)
        self.assertEqual(result["acceptance_claim"], v2.ACCEPTANCE_CLAIM)
        self.assertTrue(result["non_binding"])
        self.assertEqual(result["version"], "tube_v2")
        self.assertIn("sampled", result["sup_kind"])


class StatisticsTests(unittest.TestCase):
    def test_sup_rms_p99_argmax_per_component(self):
        times = np.linspace(0., 1., 101)
        diff = np.zeros((N, 101))
        diff[3] = np.linspace(0., 1., 101)
        diff[3, 40] = 5.  # a single spike
        block = v2.stats(times, diff, np.ones(101, bool))
        self.assertEqual(block["v_x"]["sup"], 5.)
        self.assertEqual(block["v_x"]["argmax_time_s"], times[40])
        self.assertAlmostEqual(block["v_x"]["rms"], float(np.sqrt(np.mean(diff[3] ** 2))))
        self.assertAlmostEqual(block["v_x"]["p99"], float(np.percentile(diff[3], 99.)))
        self.assertEqual(block["s"]["sup"], 0.)

    def test_empty_selection_is_none_not_zero(self):
        self.assertIsNone(v2.stats(np.zeros(3), np.zeros((N, 3)), np.zeros(3, bool)))

    def test_pair_difference_sup_matches_known_offset_and_sampled_label(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root, errors=(0.4, 0.2, 0.1)), 0.5, accepting_certify())
        cm = result["pair_differences"]["coarse_medium"]["all"]["v_x"]
        self.assertAlmostEqual(cm["sup"], 0.2)
        self.assertAlmostEqual(cm["rms"], 0.2)
        self.assertAlmostEqual(cm["p99"], 0.2)
        self.assertIn("sampled sup", result["pair_differences"]["coarse_medium"]["sampled_sup_label"])

    def test_sampled_sup_sees_interior_extremum_between_knots(self):
        with tempfile.TemporaryDirectory() as root:
            c = make_attempt(root, "c", .02, 1e-5)
            # Quadratic bump inside every interval that vanishes at the knots.
            m = make_attempt(root, "m", .01, 5e-6)
            f = make_attempt(root, "f", .005, 2.5e-6)
            data = np.load(Path(m) / "dense_polynomials.npz")
            coeff = data["coefficients"].copy()
            # s in (-1, 0]: 4*(s+1)*(-s) = -4 s^2 - 4 s peaks 1 at s=-0.5.
            coeff[:, 2, 3] = -4.
            coeff[:, 1, 3] = -4.
            np.savez_compressed(Path(m) / "dense_polynomials.npz", times=data["times"], t_end=data["t_end"],
                                step=data["step"], coefficients=coeff, bounds=data["bounds"])
            result = build([c, m, f], 0.5, accepting_certify())
        sup = result["pair_differences"]["coarse_medium"]["all"]["v_x"]["sup"]
        self.assertGreater(sup, .98)
        self.assertLessEqual(sup, 1. + 1e-12)


class EventWindowTests(unittest.TestCase):
    def pedal_run(self, root, pedal_slope, pedal_offset, **kw):
        names = ("c", "m", "f")
        steps = (.02, .01, .005)
        rtols = (1e-5, 5e-6, 2.5e-6)
        out = []
        for n, s, r in zip(names, steps, rtols):
            offsets = np.zeros(N)
            slopes = np.zeros(N)
            offsets[v2.PEDAL], slopes[v2.PEDAL] = pedal_offset, pedal_slope
            out.append(make_attempt(root, n, s, r, offsets, slopes, **kw))
        return out

    def test_pedal_zero_crossing_creates_window_from_coarse_step(self):
        with tempfile.TemporaryDirectory() as root:
            levels = self.pedal_run(root, 1., -.5)  # crosses 0 at t = 0.5
            result = build(levels, 0.5, accepting_certify())
        self.assertGreaterEqual(result["events_found"], 1)
        components = {e["component"] for e in result["events"]}
        self.assertEqual(components, {"pedal"})
        (low, high), = result["event_windows_s"]
        self.assertAlmostEqual(low, 0.5 - 3 * .02, places=6)
        self.assertAlmostEqual(high, 0.5 + 3 * .02, places=6)
        self.assertAlmostEqual(result["event_window_half_width_s"], .06)
        self.assertTrue(any(e["threshold"] == 0. for e in result["events"]))

    def test_pedal_clip_and_wheel_and_steering_events(self):
        offsets = np.zeros(N)
        slopes = np.zeros(N)
        slopes[v2.PEDAL], offsets[v2.PEDAL] = 2., -0.5   # 0 at .25, +1 at .75
        slopes[6], offsets[6] = -2., 1.2                  # wheel w_fl crosses 0 at .6
        slopes[v2.STEERING], offsets[v2.STEERING] = 1., -.1  # reaches +0.4 limit at .5
        with tempfile.TemporaryDirectory() as root:
            levels = [make_attempt(root, n, s, r, offsets, slopes)
                      for n, s, r in zip("cmf", (.02, .01, .005), (1e-5, 5e-6, 2.5e-6))]
            result = build(levels, 0.4, accepting_certify())
        pairs = {(e["component"], e["threshold"]) for e in result["events"]}
        self.assertIn(("pedal", 0.), pairs)
        self.assertIn(("pedal", 1.), pairs)
        self.assertIn(("w_fl", 0.), pairs)
        self.assertIn(("steering", 0.4), pairs)
        times = {e["component"]: e["time_s"] for e in result["events"]}
        self.assertAlmostEqual(times["w_fl"], .6, places=6)

    def test_windows_ignore_medium_fine_and_held_out_levels(self):
        with tempfile.TemporaryDirectory() as root:
            c = make_attempt(root, "c", .02, 1e-5)  # flat: no events
            offsets = np.zeros(N)
            slopes = np.zeros(N)
            slopes[v2.PEDAL], offsets[v2.PEDAL] = 1., -.5
            m = make_attempt(root, "m", .01, 5e-6, offsets, slopes)
            f = make_attempt(root, "f", .005, 2.5e-6, offsets, slopes)
            g = make_attempt(root, "g", .0025, 1.25e-6, offsets, slopes)
            result = build([c, m, f, g], 0.5, accepting_certify())
        self.assertEqual(result["events_found"], 0)
        self.assertEqual(result["event_windows_s"], [])

    def test_in_and_out_of_window_statistics_are_separated(self):
        with tempfile.TemporaryDirectory() as root:
            levels = self.pedal_run(root, 1., -.5)
            # Put the whole coarse/medium discrepancy on v_x at one interval inside the window.
            c, m, f = levels
            data = np.load(Path(m) / "dense_polynomials.npz")
            coeff = data["coefficients"].copy()
            coeff[4, 0, 3] += 1.   # interval (0.4, 0.5]: t in window [0.44, 0.56]
            np.savez_compressed(Path(m) / "dense_polynomials.npz", times=data["times"], t_end=data["t_end"],
                                step=data["step"], coefficients=coeff, bounds=data["bounds"])
            result = build([c, m, f], 0.5, accepting_certify())
        cm = result["pair_differences"]["coarse_medium"]
        self.assertAlmostEqual(cm["in_window"]["v_x"]["sup"], 1.)
        self.assertEqual(cm["out_of_window"]["v_x"]["sup"], 1.)  # samples at 0.4..0.44 are outside
        self.assertGreater(cm["samples_in_window"], 0)
        self.assertGreater(cm["in_window"]["v_x"]["argmax_time_s"], .43)

    def test_zero_out_of_window_samples_gives_null_never_pass(self):
        with tempfile.TemporaryDirectory() as root:
            # Pedal oscillates around zero everywhere -> windows cover the whole run.
            offsets = np.zeros(N)
            slopes = np.zeros(N)
            names = ("c", "m", "f")
            levels = []
            for n, s, r in zip(names, (.5, .25, .125), (1e-5, 5e-6, 2.5e-6)):
                # half-width 3*.5 = 1.5 s covers the horizon of 1 s around a crossing at .5
                o = offsets.copy(); sl = slopes.copy()
                o[v2.PEDAL], sl[v2.PEDAL] = -.5, 1.
                levels.append(make_attempt(root, n, s, r, o, sl))
            result = build(levels, 0.5, accepting_certify())
        conv = result["empirical_convergence_test"]
        self.assertIsNone(result["pair_differences"]["coarse_medium"]["out_of_window"])
        self.assertIsNone(conv["shrink_cm_mf_out_of_window"])


class HeldOutTests(unittest.TestCase):
    def four(self, root, errors):
        steps = (.02, .01, .005, .0025)
        rtols = (1e-5, 5e-6, 2.5e-6, 1.25e-6)
        return [make_attempt(root, n, s, r, offsets_at("yaw_rate", e))
                for n, s, r, e in zip("cmfg", steps, rtols, errors)]

    def test_tube_never_uses_fourth_level(self):
        with tempfile.TemporaryDirectory() as root:
            three = build(self.four(root, (0.4, 0.2, 0.1, 0.0))[:3], 0.5, accepting_certify())
        with tempfile.TemporaryDirectory() as root:
            four = build(self.four(root, (0.4, 0.2, 0.1, 50.0)), 0.5, accepting_certify())
        self.assertEqual(three["tube_T_diagnostic_scale"], four["tube_T_diagnostic_scale"])
        self.assertEqual(three["event_windows_s"], four["event_windows_s"])
        self.assertIsNone(three["empirical_convergence_test"]["held_out"])

    def test_held_out_containment_and_shrink_flags(self):
        with tempfile.TemporaryDirectory() as root:
            inside = build(self.four(root, (0.4, 0.2, 0.1, 0.05)), 0.5, accepting_certify())
        held = inside["empirical_convergence_test"]["held_out"]
        self.assertTrue(held["d_fg_within_T"]["yaw_rate"])
        self.assertTrue(held["shrink_mf_fg_all"]["yaw_rate"])
        self.assertAlmostEqual(held["observed_ratio_fg_over_mf"]["yaw_rate"], .5)
        with tempfile.TemporaryDirectory() as root:
            outside = build(self.four(root, (0.4, 0.2, 0.1, 5.0)), 0.5, accepting_certify())
        held = outside["empirical_convergence_test"]["held_out"]
        self.assertFalse(held["d_fg_within_T"]["yaw_rate"])   # d_fg = 4.9 > T = 2
        self.assertFalse(held["shrink_mf_fg_all"]["yaw_rate"])
        self.assertAlmostEqual(held["d_fg_over_T"]["yaw_rate"], 4.9 / 2.)
        self.assertIn("fine_g", outside["pair_differences"])


class FailClosedTests(unittest.TestCase):
    def build_with(self, mutate):
        with tempfile.TemporaryDirectory() as root:
            levels = mutate(root)
            return build(levels, 0.5, accepting_certify())

    def test_wrong_number_of_levels(self):
        with tempfile.TemporaryDirectory() as root:
            c, m, f = three_levels(root)
            with self.assertRaises(ValueError):
                build([c, m], 0.5, accepting_certify())

    def test_source_hash_mismatch(self):
        def mutate(root):
            c = make_attempt(root, "c", .02, 1e-5)
            m = make_attempt(root, "m", .01, 5e-6, hashes={"a": "9"})
            f = make_attempt(root, "f", .005, 2.5e-6)
            return [c, m, f]
        with self.assertRaisesRegex(ValueError, "source hashes"):
            self.build_with(mutate)

    def test_missing_source_hashes(self):
        def mutate(root):
            out = []
            for n, s, r in zip("cmf", (.02, .01, .005), (1e-5, 5e-6, 2.5e-6)):
                d = make_attempt(root, n, s, r)
                summary = json.loads((d / "summary.json").read_text())
                del summary["source_hashes"]
                (d / "summary.json").write_text(json.dumps(summary))
                out.append(d)
            return out
        with self.assertRaisesRegex(ValueError, "source hashes"):
            self.build_with(mutate)

    def test_empty_or_malformed_source_hashes(self):
        for bad in ({}, {"a": ""}, {"a": 5}, []):
            def mutate(root, bad=bad):
                return [make_attempt(root, n, st, r, hashes=bad)
                        for n, st, r in zip("cmf", (.02, .01, .005), (1e-5, 5e-6, 2.5e-6))]
            with self.assertRaisesRegex(ValueError, "source hashes"):
                self.build_with(mutate)

    def test_warm_start_mismatch_or_missing(self):
        def mismatch(root):
            c = make_attempt(root, "c", .02, 1e-5)
            m = make_attempt(root, "m", .01, 5e-6, settings_extra={"warm_start": "other_cold"})
            f = make_attempt(root, "f", .005, 2.5e-6)
            return [c, m, f]
        with self.assertRaisesRegex(ValueError, "warm_start"):
            self.build_with(mismatch)
        def missing(root):
            return [make_attempt(root, n, st, r, settings_extra={"warm_start": None})
                    for n, st, r in zip("cmf", (.02, .01, .005), (1e-5, 5e-6, 2.5e-6))]
        with self.assertRaisesRegex(ValueError, "warm_start"):
            self.build_with(missing)

    def test_dense_arrays_must_be_mutually_consistent(self):
        def rewrite(root, **override):
            d = make_attempt(root, "c", .02, 1e-5)
            data = dict(np.load(d / "dense_polynomials.npz"))
            data.update(override)
            np.savez_compressed(d / "dense_polynomials.npz", **data)
            return d
        with tempfile.TemporaryDirectory() as root:
            d = rewrite(root, t_end=np.linspace(.1, 1., 10) - .05)
            with self.assertRaisesRegex(ValueError, "t_end/step"):
                v2.load(d)
        with tempfile.TemporaryDirectory() as root:
            # an event-truncated interval (BDF step longer than the span) is legitimate
            d = rewrite(root, step=np.full(10, .3))
            data, _ = v2.load(d)
            self.assertEqual(data["step"].size, 10)
        with tempfile.TemporaryDirectory() as root:
            d = rewrite(root, step=np.full(9, .1))
            with self.assertRaisesRegex(ValueError, "shape inconsistent"):
                v2.load(d)
        with tempfile.TemporaryDirectory() as root:
            d = rewrite(root, bounds=np.zeros((9, N, 2)))
            with self.assertRaisesRegex(ValueError, "bounds"):
                v2.load(d)
        with tempfile.TemporaryDirectory() as root:
            bounds = np.zeros((10, N, 2)); bounds[:, :, 0] = 1.
            d = rewrite(root, bounds=bounds)
            with self.assertRaisesRegex(ValueError, "bounds"):
                v2.load(d)

    def test_setting_mismatch(self):
        def mutate(root):
            c = make_attempt(root, "c", .02, 1e-5)
            m = make_attempt(root, "m", .01, 5e-6, settings_extra={"pedal_travel_time": .2})
            f = make_attempt(root, "f", .005, 2.5e-6)
            return [c, m, f]
        with self.assertRaisesRegex(ValueError, "pedal_travel_time"):
            self.build_with(mutate)

    def test_not_strictly_refined(self):
        def mutate(root):
            return [make_attempt(root, "c", .02, 1e-5), make_attempt(root, "m", .02, 5e-6),
                    make_attempt(root, "f", .005, 2.5e-6)]
        with self.assertRaisesRegex(ValueError, "strictly refined"):
            self.build_with(mutate)
        def mutate_rtol(root):
            return [make_attempt(root, "c", .02, 1e-5), make_attempt(root, "m", .01, 1e-5),
                    make_attempt(root, "f", .005, 2.5e-6)]
        with self.assertRaisesRegex(ValueError, "strictly refined"):
            self.build_with(mutate_rtol)

    def test_incomplete_lap_or_missing_dense_file(self):
        def incomplete(root):
            return [make_attempt(root, "c", .02, 1e-5, completed=False),
                    make_attempt(root, "m", .01, 5e-6), make_attempt(root, "f", .005, 2.5e-6)]
        with self.assertRaisesRegex(ValueError, "completed lap"):
            self.build_with(incomplete)
        def no_dense(root):
            return [make_attempt(root, "c", .02, 1e-5), make_attempt(root, "m", .01, 5e-6, write_dense=False),
                    make_attempt(root, "f", .005, 2.5e-6)]
        with self.assertRaisesRegex(ValueError, "completed lap"):
            self.build_with(no_dense)

    def test_non_finite_coefficients(self):
        def mutate(root):
            return [make_attempt(root, "c", .02, 1e-5), make_attempt(root, "m", .01, 5e-6, nan=True),
                    make_attempt(root, "f", .005, 2.5e-6)]
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.build_with(mutate)

    def test_non_finite_bounds(self):
        def mutate(root):
            bad = np.full((10, N, 2), np.inf)
            return [make_attempt(root, "c", .02, 1e-5), make_attempt(root, "m", .01, 5e-6),
                    make_attempt(root, "f", .005, 2.5e-6, bad_bounds=bad)]
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.build_with(mutate)

    def test_wrong_state_count(self):
        with tempfile.TemporaryDirectory() as root:
            d = make_attempt(root, "c", .02, 1e-5)
            data = np.load(d / "dense_polynomials.npz")
            np.savez_compressed(d / "dense_polynomials.npz", times=data["times"], t_end=data["t_end"],
                                step=data["step"], coefficients=data["coefficients"][:, :, :5],
                                bounds=data["bounds"][:, :5])
            with self.assertRaisesRegex(ValueError, "unexpected shape"):
                v2.load(d)

    def test_non_finite_steering_limit(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(ValueError, "steering limit"):
                build(three_levels(root), float("nan"), accepting_certify())

    def test_domain_certifier_failure_or_nonfinite_margin_is_not_a_pass(self):
        def nonfinite(bounds):
            return {"certified": True, "margins": {"m": float("nan")}}
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, nonfinite)
        self.assertFalse(result["domain_margin_test_widened_2T"]["holds"])
        def rejected(bounds):
            return {"certified": False, "reason": "enclosure is empty or non-finite"}
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, rejected)
        self.assertFalse(result["domain_margin_test_widened_2T"]["holds"])

    def test_output_is_strict_json(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, accepting_certify())
        json.dumps(result, allow_nan=False)


class WorkingSourceTests(unittest.TestCase):
    """The default check compares recorded hashes with the current working files."""

    def levels_with_sources(self, root, files):
        hashes = {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
        return [make_attempt(root, n, st, r, hashes=hashes)
                for n, st, r in zip("cmf", (.02, .01, .005), (1e-5, 5e-6, 2.5e-6))]

    def test_matching_working_files_pass(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root) / "src.txt"
            source.write_text("model")
            levels = self.levels_with_sources(root, [source])
            result = v2.build(levels, 0.5, accepting_certify())  # default working-file check
        self.assertEqual(result["version"], "tube_v2")

    def test_edited_or_missing_working_file_fails_closed(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root) / "src.txt"
            source.write_text("model")
            levels = self.levels_with_sources(root, [source])
            source.write_text("model edited after the runs")
            with self.assertRaisesRegex(ValueError, "differs from the working file"):
                v2.build(levels, 0.5, accepting_certify())
            source.unlink()
            with self.assertRaisesRegex(ValueError, "missing"):
                v2.build(levels, 0.5, accepting_certify())

    def test_fake_hashes_fail_the_default_check(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(ValueError):
                v2.build(three_levels(root), 0.5, accepting_certify())


class MarginAndStatusTests(unittest.TestCase):
    def test_empty_margins_do_not_pass_domain_test(self):
        for margins in ({}, None):
            def certify(bounds, margins=margins):
                return {"certified": True, "margins": margins}
            with tempfile.TemporaryDirectory() as root:
                result = build(three_levels(root), 0.5, certify)
            self.assertFalse(result["domain_margin_test_widened_2T"]["holds"])

    def test_exploratory_4b_status_preserves_raw_flags(self):
        with tempfile.TemporaryDirectory() as root:
            levels = [make_attempt(root, n, st, r, offsets_at("yaw_rate", e))
                      for n, st, r, e in zip("cmfg", (.02, .01, .005, .0025),
                                             (1e-5, 5e-6, 2.5e-6, 1.25e-6), (0.4, 0.2, 0.1, 0.05))]
            result = build(levels, 0.5, accepting_certify())
        conv = result["empirical_convergence_test"]
        status = conv["exploratory_4b_status"]
        self.assertIn("exploratory", status["label"])
        self.assertIn("shrink_cm_mf_all", conv)
        self.assertIn("shrink_mf_fg_all", conv["held_out"])
        # Constant offsets on one component only: every other component has d = 0, so the
        # strict-shrink flags are False for them and the aggregate must be False, not None.
        self.assertIs(status["all_hold"], False)

    def test_exploratory_4b_status_is_null_without_held_out(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, accepting_certify())
        status = result["empirical_convergence_test"]["exploratory_4b_status"]
        self.assertIsNone(status["all_hold"])
        self.assertIsNone(status["parts"]["shrink_mf_fg_all"])

    def test_exploratory_4b_status_true_when_everything_shrinks(self):
        steps, rtols = (.02, .01, .005, .0025), (1e-5, 5e-6, 2.5e-6, 1.25e-6)
        scale = (8., 4., 2., 1.)
        with tempfile.TemporaryDirectory() as root:
            levels = [make_attempt(root, n, st, r, np.full(N, e) * np.arange(1, N + 1))
                      for n, st, r, e in zip("cmfg", steps, rtols, scale)]
            result = build(levels, 0.5, accepting_certify())
        # offsets_k = scale * (k + 1): d_fg = 1*(k+1) <= T = 10*max(4,2)*(k+1)... all hold
        self.assertIs(result["empirical_convergence_test"]["exploratory_4b_status"]["all_hold"], True)


class IsolationTests(unittest.TestCase):
    def test_build_does_not_modify_summaries_or_gate_flags(self):
        with tempfile.TemporaryDirectory() as root:
            levels = three_levels(root)
            before = [(d / "summary.json").read_bytes() for d in levels]
            build(levels, 0.5, accepting_certify())
            after = [(d / "summary.json").read_bytes() for d in levels]
            self.assertEqual(before, after)
            self.assertFalse(json.loads(after[0])["usable"])
            self.assertEqual(sorted(p.name for p in levels[2].iterdir()),
                             ["dense_polynomials.npz", "summary.json"])

    def test_v1_tool_is_unmodified_and_not_imported(self):
        digest = hashlib.sha256((ROOT / "tools" / "check_iem24_forward_tube.py").read_bytes()).hexdigest()
        self.assertEqual(digest, V1_SHA256)
        source = (ROOT / "tools" / "check_iem24_forward_tube_v2.py").read_text()
        self.assertNotIn("import check_iem24_forward_tube\n", source)
        self.assertNotIn("from tools.check_iem24_forward_tube import", source)
        self.assertNotIn("acceptance_gate", source.replace("never touches", ""))  # no gate use
        self.assertNotIn('"usable"]', source)

    def test_exact_union_knot_sup_is_not_claimed(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(three_levels(root), 0.5, accepting_certify())
        self.assertIn("exact union-knot sup not computed", result["sup_kind"])
        self.assertNotIn("exact_sup_union_knots", json.dumps(result))


if __name__ == "__main__":
    unittest.main()
