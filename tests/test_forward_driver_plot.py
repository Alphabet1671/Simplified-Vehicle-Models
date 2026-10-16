"""Regress the forward-driver trace-to-plot schema mapping."""
import unittest

import numpy as np

from tools.reports.plot_iem24_forward_driver_comparison import trace_channels


class ForwardDriverPlotTests(unittest.TestCase):
    def test_state_columns_are_not_mislabeled_as_acceleration(self):
        trace = np.zeros((2, 13))
        trace[:, 1] = [5., 10.]
        trace[:, 4] = [8., 10.]  # longitudinal velocity
        trace[:, 5] = [3., 4.]   # lateral velocity, NOT acceleration
        trace[:, 6] = [-.2, .4]  # yaw rate, NOT acceleration
        trace[:, 7] = [20., 22.]  # wheel speed, NOT yaw rate
        trace[:, 11] = [.9, .8]
        accelerations = np.array([[1., 2., 3.], [-4., -5., 6.]])
        channels = trace_channels(trace, accelerations)
        np.testing.assert_array_equal(channels["ax"], [1., -4.])
        np.testing.assert_array_equal(channels["ay"], [2., -5.])
        np.testing.assert_array_equal(channels["yaw_rate"], [-.2, .4])
        np.testing.assert_allclose(channels["speed"], [np.hypot(8., 3.), np.hypot(10., 4.)])
        np.testing.assert_array_equal(channels["soc"], [.9, .8])
        with self.assertRaises(ValueError):
            trace_channels(trace[:, :-1], accelerations)


if __name__ == "__main__":
    unittest.main()
