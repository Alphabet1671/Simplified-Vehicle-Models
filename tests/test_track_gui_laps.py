"""Regression tests for reference-free closed-loop averaging."""
import unittest
import numpy as np

try:
    import scipy  # noqa: F401
except ImportError:
    scipy = None

from tools.track_gui.lap_average import average_laps, detect_laps


def circle_trace(periods, radius=20.0, samples_per_lap=500, offsets=None):
    period = 8.0
    t = np.arange(periods * samples_per_lap + 1) * period / samples_per_lap
    phase = 2 * np.pi * t / period
    x, y = radius * np.cos(phase), radius * np.sin(phase)
    if offsets:
        lap = np.minimum((t / period).astype(int), periods - 1)
        x += np.asarray(offsets)[lap]
    return t, x, y


@unittest.skipIf(scipy is None, "SciPy is required for spatial return indexing")
class LapAverageTests(unittest.TestCase):
    def test_arithmetic_mean_translation_and_orientation(self):
        # Directly test the averaging core with three already-detected loops;
        # varying radii distinguishes arithmetic mean from median.
        import tools.track_gui.lap_average as lap_module
        original = lap_module.detect_laps
        samples = 401
        phase = np.linspace(0, 2*np.pi, samples, endpoint=False)
        radii = [20.0, 20.2, 21.5]
        stamps=[]; north=[]; east=[]; laps=[]
        for index, radius in enumerate(radii):
            begin=len(stamps)
            stamps.extend(index*8.02 + np.arange(samples)*8/(samples-1))
            north.extend(radius*np.cos(phase)); east.extend(radius*np.sin(phase))
            laps.append((begin, len(stamps)))
        try:
            lap_module.detect_laps = lambda *args, **kwargs: (laps, {"candidate_count":3,"rejected":[],"detection_stride":1})
            n,e,diag,overlap=average_laps(np.asarray(stamps),np.asarray(north),np.asarray(east),
                spacing=.2,smoothing=0,minimum_travel=100,minimum_duration=6,maximum_duration=10,deviation_limit=10)
        finally:
            lap_module.detect_laps=original
        self.assertGreaterEqual(diag["accepted_laps"], 2)
        self.assertIn("lap_boundaries", diag)
        self.assertEqual(overlap.shape[2], 2)
        # The accepted radii [20, 20.2, 21.5] must be averaged arithmetically,
        # giving >20.5 m rather than their median 20.2 m.
        result_radius = float(np.mean(np.hypot(n, e)))
        self.assertAlmostEqual(result_radius, np.mean(radii), delta=.06)
        self.assertGreater(result_radius, 20.4)
        self.assertGreater(np.sum(np.hypot(np.diff(np.r_[n,n[0]]), np.diff(np.r_[e,e[0]]))), 100)
        signed_area = .5 * np.sum(n * np.roll(e,-1) - e * np.roll(n,-1))
        self.assertGreater(signed_area, 0.0)

    def test_clips_local_spike_and_rejects_gross_loop(self):
        import tools.track_gui.lap_average as lap_module
        t,x,y=circle_trace(3)
        x[260] += 5.0
        laps=[(0,501),(500,1001),(1000,1501)]
        original=lap_module.detect_laps
        try:
            lap_module.detect_laps=lambda *args,**kwargs:(laps,{"candidate_count":3,"rejected":[],"detection_stride":1})
            n,e,diag,overlap=average_laps(t,x,y,spacing=.5,smoothing=0,
                minimum_travel=100,minimum_duration=6,maximum_duration=10,deviation_limit=3)
        finally:
            lap_module.detect_laps=original
        self.assertGreater(diag["clipped_samples"],0)
        self.assertEqual(diag["accepted_laps"],3)
        self.assertTrue(np.isfinite(n).all() and np.isfinite(e).all())

    def test_incomplete_tail_and_stationary_window(self):
        t,x,y = circle_trace(2)
        t=np.r_[t, t[-1]+np.arange(1,150)*.02]
        x=np.r_[x, np.full(149,x[-1])]; y=np.r_[y,np.full(149,y[-1])]
        laps,_=detect_laps(t,x,y,return_radius=2,minimum_travel=100,minimum_duration=6,maximum_duration=10)
        self.assertGreaterEqual(len(laps),1)
        for previous, current in zip(laps, laps[1:]):
            # A single shared seam sample is a valid exact endpoint; interiors
            # must not overlap or repeat a full lap.
            self.assertGreaterEqual(current[0], previous[1] - 1)
            self.assertLess(current[0], previous[1])
        self.assertGreaterEqual(laps[0][1] - laps[0][0], 400)
        with self.assertRaisesRegex(ValueError,"no complete spatial-return"):
            detect_laps(np.arange(200)*.1,np.zeros(200),np.zeros(200),minimum_travel=50)

    def test_irregular_clock_and_clipping_support_refusal(self):
        t,x,y=circle_trace(3)
        t=t + .001*np.sin(np.arange(len(t)))
        t=np.maximum.accumulate(t + np.arange(len(t))*1e-7)
        n,e,diag,_=average_laps(t,x,y,spacing=.5,smoothing=0,return_radius=1.5,
            minimum_travel=100,minimum_duration=6,maximum_duration=10)
        self.assertTrue(np.isfinite(n).all() and np.isfinite(e).all())
        self.assertEqual(len(diag["lap_boundaries"]),diag["accepted_laps"])
        self.assertIn("detection_rejections", diag)
        self.assertEqual(
            diag["rejected_candidate_count"], len(diag["detection_rejections"])
        )

    def test_insufficient_support_and_requested_support_refuse(self):
        t, x, y = circle_trace(2)
        # The two loops disagree by more than the clipping limit at each point.
        x[501:] += 10.0
        import tools.track_gui.lap_average as lap_module
        original = lap_module.detect_laps
        try:
            lap_module.detect_laps = lambda *args, **kwargs: (
                [(0, 501), (500, 1001)],
                {"candidate_count": 2, "rejected": [], "detection_stride": 1},
            )
            with self.assertRaisesRegex(ValueError, "too few lap overlaps"):
                average_laps(
                    t, x, y, spacing=.5, smoothing=0, minimum_travel=100,
                    minimum_duration=6, maximum_duration=10, deviation_limit=1.1,
                )
            with self.assertRaisesRegex(ValueError, "minimum_support exceeds"):
                average_laps(
                    t, x, y, spacing=.5, smoothing=0, minimum_travel=100,
                    minimum_duration=6, maximum_duration=10, minimum_support=3,
                )
        finally:
            lap_module.detect_laps = original

    def test_geometry_array_node_cap(self):
        from tools.tracks.build_closed_track import geometry_from_arrays

        phase = np.linspace(0, 2 * np.pi, 1001)
        with self.assertRaisesRegex(ValueError, "more than 12000 geometry nodes"):
            geometry_from_arrays(20 * np.cos(phase), 20 * np.sin(phase), .001, 0)

    def test_no_complete_loop_refused(self):
        t=np.arange(100)*.1
        with self.assertRaisesRegex(ValueError,"no complete spatial-return"):
            detect_laps(t,t,np.zeros_like(t),minimum_travel=20,minimum_duration=2,maximum_duration=5)


if __name__ == "__main__":
    unittest.main()
