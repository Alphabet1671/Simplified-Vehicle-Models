import unittest

import numpy as np

from tools.iem24_telemetry import (
    LOGGED_CHANNELS,
    SCALAR_CHANNELS,
    NavChannels,
    estimate_nav_skew_deg,
    integrate_position,
    lap_distance_axis,
    rotate_to_body_frame,
    segment_laps,
    travel_distance,
    zero_phase_lowpass,
)


def _circle_channels(
    radius_m: float, speed_mps: float, laps: int, rate_hz: float = 100.0
) -> NavChannels:
    """Return synthetic nav channels for a circle driven at constant speed."""

    period = 2.0 * np.pi * radius_m / speed_mps
    time = np.arange(0.0, laps * period, 1.0 / rate_hz)
    omega = speed_mps / radius_m
    north_velocity = speed_mps * np.cos(omega * time)
    east_velocity = -speed_mps * np.sin(omega * time)
    return NavChannels(
        time=time,
        values={
            "VNorth": north_velocity,
            "VEast": east_velocity,
            "VelX": np.full(time.size, speed_mps),
            "VelY": np.zeros(time.size),
            "GyroZ": np.full(time.size, omega),
        },
    )


class NavChannelConditioningTests(unittest.TestCase):
    def test_lowpass_does_not_overshoot_a_step(self):
        interval = 0.01
        signal = np.zeros(1000)
        signal[500:] = 1.0
        filtered = zero_phase_lowpass(signal, interval, 1.0)
        self.assertLessEqual(float(np.max(filtered)), 1.0 + 1.0e-9)
        self.assertGreaterEqual(float(np.min(filtered)), -1.0e-9)

    def test_lowpass_leaves_a_constant_signal_unchanged(self):
        signal = np.full(500, 3.5)
        filtered = zero_phase_lowpass(signal, 0.01, 1.0)
        np.testing.assert_allclose(filtered, signal, rtol=1.0e-9)

    def test_integrate_position_closes_a_circle(self):
        channels = _circle_channels(100.0, 20.0, 2)
        north, east = integrate_position(
            channels.time, channels["VNorth"], channels["VEast"]
        )
        period = 2.0 * np.pi * 100.0 / 20.0
        index = int(round(period / channels.sample_interval))
        self.assertLess(float(np.hypot(north[index], east[index])), 1.0)

    def test_travel_distance_matches_constant_speed(self):
        time = np.arange(0.0, 10.0, 0.01)
        distance = travel_distance(time, np.full(time.size, 12.0))
        self.assertAlmostEqual(float(distance[-1]), 12.0 * float(time[-1]), places=6)

    def test_segment_laps_finds_one_lap_per_revolution(self):
        channels = _circle_channels(100.0, 20.0, 3)
        period = 2.0 * np.pi * 100.0 / 20.0
        laps = segment_laps(
            channels,
            window=(0.0, float(channels.time[-1])),
            minimum_lap_time_s=20.0,
            maximum_lap_time_s=45.0,
        )
        # The record stops one sample short of the third closure, which the
        # search still accepts because that sample is within the closure radius.
        self.assertEqual(len(laps), 3)
        circumference = 2.0 * np.pi * 100.0
        for lap in laps:
            self.assertAlmostEqual(lap.lap_time_s, period, delta=0.1)
            self.assertAlmostEqual(lap.distance_m, circumference, delta=1.5)
            self.assertFalse(lap.standing_start)

    def test_segment_laps_flags_a_standing_start(self):
        channels = _circle_channels(100.0, 20.0, 2)
        half = channels.time.size // 2
        values = dict(channels.values)
        values["VelX"] = values["VelX"].copy()
        values["VelY"] = values["VelY"].copy()
        values["VelX"][: half] = 0.0
        stationary = NavChannels(time=channels.time, values=values)
        laps = segment_laps(
            stationary,
            window=(0.0, float(stationary.time[-1])),
            minimum_lap_time_s=20.0,
            maximum_lap_time_s=45.0,
        )
        self.assertTrue(laps[0].standing_start)

    def test_lap_distance_axis_starts_at_zero(self):
        channels = _circle_channels(100.0, 20.0, 2)
        laps = segment_laps(
            channels,
            window=(0.0, float(channels.time[-1])),
            minimum_lap_time_s=20.0,
            maximum_lap_time_s=45.0,
        )
        axis = lap_distance_axis(
            channels.time, np.hypot(channels["VelX"], channels["VelY"]), laps[0]
        )
        self.assertEqual(float(axis[0]), 0.0)
        self.assertTrue(np.all(np.diff(axis) >= 0.0))


class PackChannelTests(unittest.TestCase):
    def test_pack_voltage_is_requested_from_the_export(self):
        self.assertIn("VPack", SCALAR_CHANNELS)
        self.assertIn("VPack", LOGGED_CHANNELS)
        for name in ("AccelX", "AccelY", "GyroZ", "VelX", "VelY", "VNorth", "VEast"):
            self.assertIn(name, LOGGED_CHANNELS)

    def test_scalar_channels_survive_a_frame_rotation(self):
        time = np.arange(0.0, 10.0, 0.01)
        channels = NavChannels(
            time=time,
            values={
                "VelX": np.full(time.size, 10.0),
                "VelY": np.full(time.size, 2.0),
                "AccelX": np.zeros(time.size),
                "AccelY": np.zeros(time.size),
                "VPack": np.linspace(575.0, 570.0, time.size),
            },
        )
        body = rotate_to_body_frame(channels, 11.0)
        np.testing.assert_allclose(body["VPack"], channels["VPack"])


class SkewCorrectionTests(unittest.TestCase):
    def _skewed(self, angle_deg: float) -> NavChannels:
        time = np.arange(0.0, 60.0, 0.01)
        speed = 15.0
        radians = np.radians(angle_deg)
        return NavChannels(
            time=time,
            values={
                "VelX": np.full(time.size, speed * np.cos(radians)),
                "VelY": np.full(time.size, speed * np.sin(radians)),
                "AccelX": np.full(time.size, 1.0 * np.cos(radians)),
                "AccelY": np.full(time.size, 1.0 * np.sin(radians)),
                "GyroZ": np.zeros(time.size),
            },
        )

    def test_estimate_recovers_the_mounting_angle(self):
        for angle in (-12.0, -4.5, 8.55, 17.0):
            with self.subTest(angle=angle):
                estimate = estimate_nav_skew_deg(self._skewed(angle))
                self.assertAlmostEqual(estimate.angle_deg, angle, places=3)
                self.assertEqual(estimate.sample_count, 6000)

    def test_rotation_removes_the_cross_axis_velocity(self):
        channels = self._skewed(8.55)
        body = rotate_to_body_frame(channels, 8.55)
        np.testing.assert_allclose(body["VelY"], 0.0, atol=1.0e-9)
        np.testing.assert_allclose(body["VelX"], 15.0, atol=1.0e-9)

    def test_rotation_preserves_speed_and_vertical_yaw_rate(self):
        channels = self._skewed(11.0)
        body = rotate_to_body_frame(channels, 11.0)
        np.testing.assert_allclose(
            np.hypot(body["VelX"], body["VelY"]),
            np.hypot(channels["VelX"], channels["VelY"]),
            atol=1.0e-9,
        )
        np.testing.assert_allclose(body["GyroZ"], channels["GyroZ"], atol=1.0e-12)

    def test_estimate_rejects_a_record_without_straight_running(self):
        channels = self._skewed(8.0)
        values = dict(channels.values)
        values["GyroZ"] = np.full(channels.time.size, 0.5)
        with self.assertRaisesRegex(ValueError, "straight"):
            estimate_nav_skew_deg(
                NavChannels(time=channels.time, values=values)
            )


if __name__ == "__main__":
    unittest.main()
