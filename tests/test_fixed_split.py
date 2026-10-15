import unittest

import numpy as np

from simplified_vehicle_models.drivetrain.fixed_split import (
    DRIVE_FRONT_FRACTION,
    allocate_brake,
    allocate_controls,
    allocate_fixed_split,
    check_limits_are_respected,
    check_split_is_held,
    is_within_limits,
    wheel_shares,
)


class WheelShareTests(unittest.TestCase):
    def test_the_default_split_is_twenty_eighty(self) -> None:
        shares = wheel_shares(0.2)

        np.testing.assert_allclose(shares, [0.1, 0.1, 0.4, 0.4])
        self.assertAlmostEqual(float(np.sum(shares)), 1.0, places=12)

    def test_the_shares_always_sum_to_one(self) -> None:
        for fraction in (0.0, 0.2, 0.5, 0.65, 1.0):
            with self.subTest(fraction=fraction):
                self.assertAlmostEqual(
                    float(np.sum(wheel_shares(fraction))), 1.0, places=12
                )

    def test_rejects_an_invalid_fraction(self) -> None:
        for fraction in (-0.1, 1.1, float("nan"), float("inf")):
            with self.subTest(fraction=fraction), self.assertRaises(ValueError):
                wheel_shares(fraction)


class FixedSplitTests(unittest.TestCase):
    def test_a_free_request_takes_the_whole_share(self) -> None:
        limits = np.full(4, 1000.0)

        result = allocate_fixed_split(400.0, limits)

        np.testing.assert_allclose(
            result.wheel_torques, [40.0, 40.0, 160.0, 160.0]
        )
        self.assertEqual(result.achieved_total, 400.0)
        self.assertEqual(result.unmet_total, 0.0)
        self.assertIsNone(result.limiting_wheel)
        self.assertFalse(result.is_limited)

    def test_each_axle_splits_equally_left_and_right(self) -> None:
        result = allocate_fixed_split(400.0, np.full(4, 1000.0))

        self.assertAlmostEqual(
            float(result.wheel_torques[0]), float(result.wheel_torques[1])
        )
        self.assertAlmostEqual(
            float(result.wheel_torques[2]), float(result.wheel_torques[3])
        )

    def test_a_rear_limit_caps_the_front_too(self) -> None:
        # Each rear wheel accepts 40 N*m, which holds the total to 100 N*m.
        limits = np.asarray([1000.0, 1000.0, 40.0, 40.0])

        result = allocate_fixed_split(400.0, limits)

        self.assertAlmostEqual(result.achieved_total, 100.0, places=9)
        self.assertAlmostEqual(
            float(result.wheel_torques[0]), 10.0, places=9
        )
        self.assertAlmostEqual(
            float(result.wheel_torques[2]), 40.0, places=9
        )
        self.assertTrue(result.is_limited)
        self.assertIn(result.limiting_wheel, (2, 3))

    def test_one_tight_rear_wheel_caps_the_whole_request(self) -> None:
        limits = np.asarray([1000.0, 1000.0, 20.0, 40.0])

        result = allocate_fixed_split(400.0, limits)

        # The tight wheel allows 20/0.4 = 50 N*m total.
        self.assertAlmostEqual(result.achieved_total, 50.0, places=9)
        self.assertAlmostEqual(
            float(result.wheel_torques[0]), 5.0, places=9
        )
        self.assertAlmostEqual(
            float(result.wheel_torques[2]), 20.0, places=9
        )
        self.assertEqual(result.limiting_wheel, 2)

    def test_a_front_limit_caps_the_request(self) -> None:
        limits = np.asarray([10.0, 10.0, 1000.0, 1000.0])

        result = allocate_fixed_split(400.0, limits)

        self.assertAlmostEqual(result.achieved_total, 100.0, places=9)
        self.assertAlmostEqual(
            float(result.wheel_torques[2]), 40.0, places=9
        )
        self.assertIn(result.limiting_wheel, (0, 1))

    def test_the_split_holds_at_the_limit(self) -> None:
        limits = np.asarray([1000.0, 1000.0, 37.5, 37.5])

        result = allocate_fixed_split(500.0, limits)

        self.assertTrue(result.is_limited)
        self.assertLessEqual(check_split_is_held(
            result.wheel_torques, result.achieved_total, 0.2), 1.0e-12
        )
        self.assertFalse(check_limits_are_respected(
            result.wheel_torques, limits) > 1.0e-9)
        self.assertTrue(is_within_limits(result.wheel_torques, limits))

    def test_a_zero_share_wheel_never_caps_the_request(self) -> None:
        limits = np.asarray([0.0, 0.0, 100.0, 100.0])

        result = allocate_fixed_split(100.0, limits, front_fraction=0.0)

        self.assertAlmostEqual(result.achieved_total, 100.0, places=9)
        np.testing.assert_allclose(result.wheel_torques[:2], 0.0)

    def test_a_zero_request_returns_zero(self) -> None:
        result = allocate_fixed_split(0.0, np.full(4, 100.0))

        np.testing.assert_allclose(result.wheel_torques, 0.0)
        self.assertFalse(result.is_limited)
        self.assertIsNone(result.limiting_wheel)

    def test_rejects_invalid_inputs(self) -> None:
        with self.assertRaises(ValueError):
            allocate_fixed_split(-1.0, np.full(4, 100.0))
        with self.assertRaises(ValueError):
            allocate_fixed_split(float("nan"), np.full(4, 100.0))
        with self.assertRaises(ValueError):
            allocate_fixed_split(100.0, np.full(3, 100.0))
        with self.assertRaises(ValueError):
            allocate_fixed_split(100.0, np.full(4, -1.0))
        with self.assertRaises(ValueError):
            allocate_fixed_split(100.0, np.full(4, np.inf))
        with self.assertRaises(ValueError):
            allocate_fixed_split(100.0, np.full(4, 100.0), front_fraction=1.5)


class BrakeAllocationTests(unittest.TestCase):
    def test_braking_torques_are_negative(self) -> None:
        result = allocate_brake(1000.0, np.full(4, 1000.0), front_fraction=0.65)

        self.assertTrue(np.all(result.wheel_torques <= 0.0))
        self.assertAlmostEqual(result.achieved_total, 1000.0, places=9)

    def test_the_bias_splits_front_and_rear(self) -> None:
        result = allocate_brake(1000.0, np.full(4, 2000.0), front_fraction=0.65)

        front = abs(float(result.wheel_torques[0] + result.wheel_torques[1]))
        rear = abs(float(result.wheel_torques[2] + result.wheel_torques[3]))
        self.assertAlmostEqual(front / (front + rear), 0.65, places=9)

    def test_a_rear_brake_limit_caps_the_total(self) -> None:
        limits = np.asarray([1000.0, 1000.0, 100.0, 100.0])

        result = allocate_brake(1000.0, limits, front_fraction=0.65)

        # Each rear wheel allows 100/0.175 = 571.43 N*m total.
        self.assertLess(result.achieved_total, 1000.0)
        self.assertTrue(result.is_limited)
        self.assertTrue(is_within_limits(result.wheel_torques, limits))


class ControlAllocationTests(unittest.TestCase):
    def test_throttle_scales_the_drive(self) -> None:
        limits = np.full(4, 1000.0)
        torques, drive, braking = allocate_controls(
            0.5, 0.0, 400.0, 1000.0, limits, limits
        )

        np.testing.assert_allclose(torques, [20.0, 20.0, 80.0, 80.0])
        np.testing.assert_allclose(braking.wheel_torques, 0.0)
        self.assertEqual(drive.achieved_total, 200.0)

    def test_brake_scales_the_braking(self) -> None:
        limits = np.full(4, 1000.0)
        torques, _, braking = allocate_controls(
            0.0, 0.5, 400.0, 1000.0, limits, limits
        )

        self.assertTrue(np.all(torques <= 0.0))
        self.assertAlmostEqual(braking.achieved_total, 500.0, places=9)

    def test_drive_and_brake_add(self) -> None:
        limits = np.full(4, 1000.0)
        torques, drive, braking = allocate_controls(
            1.0, 1.0, 400.0, 200.0, limits, limits
        )

        np.testing.assert_allclose(
            torques, drive.wheel_torques + braking.wheel_torques
        )

    def test_rejects_invalid_controls(self) -> None:
        limits = np.full(4, 1000.0)
        with self.assertRaises(ValueError):
            allocate_controls(1.5, 0.0, 400.0, 100.0, limits, limits)
        with self.assertRaises(ValueError):
            allocate_controls(0.0, -0.1, 400.0, 100.0, limits, limits)
        with self.assertRaises(ValueError):
            allocate_controls(0.5, 0.0, -1.0, 100.0, limits, limits)

    def test_the_default_drive_fraction_is_twenty_percent(self) -> None:
        self.assertEqual(DRIVE_FRONT_FRACTION, 0.2)


if __name__ == "__main__":
    unittest.main()
