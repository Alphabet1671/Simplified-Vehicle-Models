"""Synthetic tests for bounded periodic lap seam closure."""
import unittest

import numpy as np

try:
    import scipy  # noqa: F401
except ImportError:
    scipy = None

from tools.track_gui.lap_closure import smooth_seam


def stadium(n=240, radius=20.0, half_straight=25.0):
    # Uniformly sampled perimeter with node zero halfway along a straight.
    straight = np.linspace(-half_straight, half_straight, n // 2, endpoint=False)
    theta = np.linspace(-np.pi / 2, np.pi / 2, n // 4, endpoint=False)
    right = np.column_stack((half_straight + radius*np.cos(theta), radius*np.sin(theta)))
    top = np.column_stack((np.linspace(half_straight, -half_straight, n // 2, endpoint=False),
                           np.full(n // 2, radius)))
    # Both semicircles follow the same perimeter winding (counterclockwise).
    left_arc = np.column_stack((-half_straight-radius*np.cos(theta[::-1]), radius*np.sin(theta[::-1])))
    points = np.vstack((np.column_stack((straight, np.full_like(straight, -radius))),
                        right, top, left_arc))
    # Resample the closed polyline uniformly by arc distance.
    edge = np.diff(np.vstack((points, points[0])), axis=0)
    s = np.r_[0., np.cumsum(np.linalg.norm(edge, axis=1))]
    sample = np.arange(n) * s[-1] / n
    out = np.column_stack([np.interp(sample, s, np.r_[points[:, k], points[0, k]]) for k in range(2)])
    # put node zero at bottom-straight midpoint
    idx = np.argmin(np.linalg.norm(out - [0., -radius], axis=1))
    return np.roll(out, -idx, axis=0)


@unittest.skipIf(scipy is None, "SciPy is required for bounded seam optimization")
class LapClosureTests(unittest.TestCase):
    @staticmethod
    def _bending(y):
        return float(np.sum((np.roll(y, -1)-2*y+np.roll(y, 1))**2))

    def test_periodic_straight_fit_is_bounded_and_reduces_bending(self):
        mean = stadium()
        n = len(mean)
        offsets = np.array([-0.35, 0.35])
        # Create an endpoint kink in the unsmoothed path, then add lateral
        # variation across driving lines to define the observed envelope.
        mean[:, 1] += 0.08*np.exp(-((np.minimum(np.arange(n), n-np.arange(n))/2.)**2))
        target = mean.copy()
        overlaps = np.repeat(mean[None, :, :], 2, axis=0)
        overlaps[:, :, 1] += offsets[:, None]
        keep = np.ones((2, n), dtype=bool)
        snapshots = [a.copy() for a in (mean, overlaps, keep, target)]
        result, diag = smooth_seam(mean, overlaps, keep, target)
        for actual, before in zip((mean, overlaps, keep, target), snapshots):
            np.testing.assert_array_equal(actual, before)
        self.assertEqual(result.shape, mean.shape)
        self.assertTrue(np.isfinite(result).all())
        self.assertEqual(diag["status"], "success")
        changed = np.linalg.norm(result-target, axis=1) > 1e-10
        self.assertTrue(changed.any())
        self.assertTrue(np.all(np.linalg.norm(result[~changed]-target[~changed], axis=1) == 0))
        self.assertLess(diag["bending_energy_after"], diag["bending_energy_before"])
        normal = np.array([0.0, 1.0])
        lateral_result = (result-mean[0]) @ normal
        lateral_mean = (mean-mean[0]) @ normal
        selected = np.abs(result[:, 1]-target[:, 1]) > 1e-10
        self.assertTrue(np.all(lateral_result[selected] >= lateral_mean[selected]-0.350001))
        self.assertTrue(np.all(lateral_result[selected] <= lateral_mean[selected]+0.350001))
        self.assertLess(self._bending(result[:, 1]), self._bending(mean[:, 1]))

    def test_circle_skips(self):
        t = np.arange(200)*2*np.pi/200
        mean = np.column_stack((20*np.cos(t), 20*np.sin(t)))
        overlaps = np.repeat(mean[None, :, :], 2, axis=0)
        out, diag = smooth_seam(mean, overlaps, np.ones((2, 200), bool), mean)
        np.testing.assert_array_equal(out, mean)
        self.assertEqual(diag["status"], "skipped")
        self.assertTrue(diag["skip_reason"])

    def test_reversed_winding_and_rigid_transform(self):
        base = stadium()
        for source in (base, base[::-1]):
            shifted = source-source[0]
            # rotate and translate without changing the shape or winding
            angle = 0.73
            rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            mean = shifted @ rotation.T + [120., -37.]
            n = len(mean)
            offsets = np.array([-0.3, 0.3])
            normal = np.array([-1., 0.]) @ rotation.T
            overlaps = np.repeat(mean[None], 2, axis=0) + offsets[:, None, None]*normal[None, None, :]
            result, diag = smooth_seam(mean, overlaps, np.ones((2,n),bool), mean)
            self.assertEqual(diag["status"], "success")
            self.assertTrue(np.isfinite(result).all())

    def test_zero_width_envelope_and_excluded_clipped_lap(self):
        mean = stadium()
        n = len(mean)
        overlaps = np.repeat(mean[None], 3, axis=0)
        overlaps[2, :, 1] += 100.0
        keep = np.ones((3,n), bool); keep[2] = False
        result, diag = smooth_seam(mean, overlaps, keep, mean)
        self.assertEqual(diag["status"], "success")
        np.testing.assert_allclose(result, mean, atol=1e-7)
        self.assertEqual(diag["corridor_width_m"], 0.0)

    def test_nonuniform_sampling_and_genuinely_curved_seam_skip(self):
        mean = stadium()
        # A corner at node zero must be refused, never flattened.
        curved = mean.copy()
        curved[:, 1] += 3.0*np.exp(-((np.minimum(np.arange(len(mean)), len(mean)-np.arange(len(mean)))/8.)**2))
        overlaps = np.repeat(curved[None], 2, axis=0)
        out, diag = smooth_seam(curved, overlaps, np.ones((2,len(mean)),bool), curved)
        self.assertEqual(diag["status"], "skipped")
        np.testing.assert_array_equal(out, curved)

        # Construct nonuniform node spacing while retaining a 5 m straight on each side.
        base = stadium(n=240)
        dist = np.r_[0., np.cumsum(np.linalg.norm(np.diff(np.vstack((base, base[0])), axis=0), axis=1))]
        perimeter = dist[-1]
        count = 240
        phase = np.arange(count) * perimeter / count
        offsets = 0.25 * np.sin(2*np.pi*np.arange(count)/count)
        phase = np.sort((phase + offsets) % perimeter)
        nonuniform = np.column_stack([np.interp(phase, dist, np.r_[base[:, j], base[0,j]]) for j in range(2)])
        seam = np.argmin(np.linalg.norm(nonuniform-[0.,-20.],axis=1))
        nonuniform = np.roll(nonuniform, -seam, axis=0)
        overlaps = np.repeat(nonuniform[None],2,axis=0)
        overlaps[:, :, 1] += np.array([-.2,.2])[:,None]
        result, diag = smooth_seam(nonuniform, overlaps, np.ones((2,count),bool), nonuniform)
        self.assertEqual(diag["status"], "success")
        self.assertTrue(np.isfinite(result).all())

    def test_optimizer_failure_is_refused(self):
        from unittest import mock
        from types import SimpleNamespace
        mean = stadium()
        overlaps = np.repeat(mean[None], 2, axis=0)
        overlaps[:, :, 1] += np.array([-0.3, 0.3])[:, None]
        with mock.patch("scipy.optimize.minimize", return_value=SimpleNamespace(success=False, x=np.zeros(len(mean)-4), message="forced failure")): 
            with self.assertRaisesRegex(RuntimeError, "optimizer failed"):
                smooth_seam(mean, overlaps, np.ones((2,len(mean)),bool), mean)

    def test_gaussian_smoothed_target_on_noisy_straight_fits(self):
        mean = stadium()
        n = len(mean)
        overlaps = np.repeat(mean[None], 2, axis=0)
        overlaps[:, :, 1] += np.array([-.5, .5])[:, None]
        target = mean.copy()
        phase = np.arange(n)
        target[:, 1] += .18*np.exp(-((np.minimum(phase,n-phase)/3.)**2))
        target[:, 0] += .02*np.sin(2*np.pi*phase/n)
        result, diag = smooth_seam(mean, overlaps, np.ones((2,n),bool), target)
        self.assertEqual(diag["status"], "success")
        self.assertTrue(np.isfinite(result).all())
        self.assertGreater(np.max(np.linalg.norm(result-target,axis=1)), 1e-5)

    def test_nonuniform_objective_and_bounds(self):
        mean=stadium()
        n=len(mean)
        mean[:,1]+=.09*np.exp(-((np.minimum(np.arange(n),n-np.arange(n))/2.)**2))
        overlaps=np.repeat(mean[None],2,axis=0)
        overlaps[:,:,1]+=np.array([-.25,.25])[:,None]
        target=mean.copy()
        # Nonuniform target station and lateral data emulate speed-independent smoothing.
        target[:,0]+=.03*np.sin(2*np.pi*np.arange(n)/n)
        result,diag=smooth_seam(mean,overlaps,np.ones((2,n),bool),target)
        self.assertEqual(diag["status"],"success")
        selected=np.linalg.norm(result-target,axis=1)>1e-9
        self.assertTrue(selected.any())
        self.assertTrue(np.isfinite(result).all())
        self.assertLess(diag["bending_energy_after"],diag["bending_energy_before"])
        self.assertLessEqual(diag["max_bound_violation_m"],1e-7)

        # Independently recompute expanded energy using reported periodic fit
        # indices plus one exterior target node at each end.
        fit_ids=np.asarray(diag["fit_node_indices"],dtype=int)
        node_ids=np.r_[fit_ids[0]-1,fit_ids,fit_ids[-1]+1] % n
        tangent=np.asarray(diag["frame_tangent"])
        normal=np.array([-tangent[1],tangent[0]])
        stations=(target[node_ids]-mean[0])@tangent
        before_lat=(target[node_ids]-mean[0])@normal
        after_lat=(result[node_ids]-mean[0])@normal
        def physical_energy(values, station_values):
            h0,h1=np.diff(station_values)[:-1],np.diff(station_values)[1:]
            curvature=2*((values[2:]-values[1:-1])/h1-(values[1:-1]-values[:-2])/h0)/(h0+h1)
            return float(np.sum((h0+h1)*.5*curvature**2))
        expected_before=physical_energy(before_lat,stations)
        expected_after=physical_energy(after_lat,stations)
        self.assertAlmostEqual(diag["bending_energy_before"],expected_before,delta=1e-9)
        self.assertAlmostEqual(diag["bending_energy_after"],expected_after,delta=1e-9)

    def test_incoming_bend_outgoing_straight_is_not_fitted_as_straight(self):
        base=stadium()
        n=len(base)
        # A smooth corner only on the incoming (negative arc-length) side.
        d=np.arange(n)
        bump=.65*np.exp(-((np.minimum(d,n-d)/8.)**2))
        incoming=np.where(d>n//2,bump,0.)
        mean=base.copy(); mean[:,1]+=incoming
        overlaps=np.repeat(mean[None],2,axis=0)
        out,diag=smooth_seam(mean,overlaps,np.ones((2,n),bool),mean)
        self.assertEqual(diag["status"],"skipped")
        np.testing.assert_array_equal(out,mean)

    def test_malformed_arrays_refused(self):
        mean = np.zeros((20, 2))
        with self.assertRaises(ValueError):
            smooth_seam(mean, np.zeros((2, 19, 2)), np.ones((2, 19), bool), mean)


if __name__ == "__main__":
    unittest.main()
