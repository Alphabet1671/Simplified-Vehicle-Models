import math
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.controls.brakes import (
    PedalRatioCurve,
    brake_torques,
    front_brake_torque_share,
    front_master_cylinder_share,
    maximum_brake_torques,
)
from simplified_vehicle_models.controls.steering import (
    AckermannTable,
    rack_travel,
    steering_angles,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"

RACK_TRAVEL_PER_REVOLUTION = 3.5 * 0.0254
STEER_ARM = 0.065
MAXIMUM_WHEEL_ANGLE = math.radians(30.0)


def _parallel() -> AckermannTable:
    return AckermannTable(
        inner_angle=np.asarray([0.2, 0.4, 0.6, 0.8, 1.0]),
        outer_offset=np.zeros(5),
    )


class AckermannTableTests(unittest.TestCase):
    def test_a_zero_table_is_parallel(self) -> None:
        table = _parallel()

        self.assertTrue(table.is_parallel)
        for angle in (0.0, 0.3, 0.9):
            with self.subTest(angle=angle):
                self.assertEqual(table.offset(angle), 0.0)

    def test_an_empty_table_is_parallel(self) -> None:
        table = AckermannTable(
            inner_angle=np.asarray([]), outer_offset=np.asarray([])
        )

        self.assertTrue(table.is_parallel)
        self.assertEqual(table.offset(0.5), 0.0)

    def test_the_offset_interpolates_and_is_symmetric(self) -> None:
        table = AckermannTable(
            inner_angle=np.asarray([0.0, 0.5, 1.0]),
            outer_offset=np.asarray([0.0, 0.05, 0.08]),
        )

        self.assertAlmostEqual(table.offset(0.25), 0.025, places=12)
        self.assertAlmostEqual(table.offset(-0.25), 0.025, places=12)
        self.assertAlmostEqual(table.offset(1.5), 0.08, places=12)

    def test_rejects_a_bad_table(self) -> None:
        with self.assertRaises(ValueError):
            AckermannTable(
                inner_angle=np.asarray([0.5, 0.2]), outer_offset=np.zeros(2)
            )
        with self.assertRaises(ValueError):
            AckermannTable(
                inner_angle=np.asarray([0.2, 0.4]), outer_offset=np.asarray([0.0, -0.1])
            )
        with self.assertRaises(ValueError):
            AckermannTable(
                inner_angle=np.asarray([0.2, 0.4]), outer_offset=np.zeros(3)
            )


class RackTravelTests(unittest.TestCase):
    def test_one_revolution_moves_the_rack_by_the_ratio(self) -> None:
        travel = rack_travel(2.0 * math.pi, RACK_TRAVEL_PER_REVOLUTION)

        self.assertAlmostEqual(travel, 3.5 * 0.0254, places=12)

    def test_a_quarter_turn_moves_a_quarter_of_the_ratio(self) -> None:
        travel = rack_travel(math.pi / 2.0, RACK_TRAVEL_PER_REVOLUTION)

        self.assertAlmostEqual(travel, 3.5 * 0.0254 / 4.0, places=12)

    def test_the_travel_is_odd_in_the_angle(self) -> None:
        forward = rack_travel(1.0, RACK_TRAVEL_PER_REVOLUTION)
        backward = rack_travel(-1.0, RACK_TRAVEL_PER_REVOLUTION)

        self.assertAlmostEqual(forward, -backward, places=12)

    def test_rejects_a_bad_ratio(self) -> None:
        for ratio in (0.0, -1.0, float("nan")):
            with self.subTest(ratio=ratio), self.assertRaises(ValueError):
                rack_travel(1.0, ratio)


class SteeringAngleTests(unittest.TestCase):
    def _steer(self, wheel_degrees, maximum=None, table=None):
        return steering_angles(
            math.radians(wheel_degrees),
            RACK_TRAVEL_PER_REVOLUTION,
            STEER_ARM,
            MAXIMUM_WHEEL_ANGLE if maximum is None else maximum,
            _parallel() if table is None else table,
        )

    def test_zero_steering_gives_straight_wheels(self) -> None:
        result = self._steer(0.0)

        np.testing.assert_allclose(result.wheel_angles, 0.0)
        self.assertEqual(result.rack_travel, 0.0)
        self.assertFalse(result.is_limited)

    def test_a_positive_input_turns_right(self) -> None:
        result = self._steer(90.0)

        self.assertGreater(result.wheel_angles[0], 0.0)
        self.assertGreater(result.wheel_angles[1], 0.0)
        self.assertAlmostEqual(
            float(result.wheel_angles[0]), float(result.wheel_angles[1]), places=12
        )

    def test_a_negative_input_turns_left(self) -> None:
        result = self._steer(-90.0)

        self.assertLess(result.wheel_angles[0], 0.0)
        self.assertLess(result.wheel_angles[1], 0.0)

    def test_the_steering_is_odd_in_the_input(self) -> None:
        right = self._steer(90.0)
        left = self._steer(-90.0)

        np.testing.assert_allclose(
            right.wheel_angles, -left.wheel_angles, atol=1.0e-12
        )

    def test_the_rear_wheels_stay_straight(self) -> None:
        result = self._steer(120.0)

        self.assertEqual(float(result.wheel_angles[2]), 0.0)
        self.assertEqual(float(result.wheel_angles[3]), 0.0)

    def test_the_maximum_wheel_angle_is_enforced(self) -> None:
        for wheel_degrees in (180.0, 360.0, 1080.0):
            result = self._steer(wheel_degrees)
            with self.subTest(wheel_degrees=wheel_degrees):
                self.assertTrue(result.is_limited)
                self.assertLessEqual(
                    float(np.max(np.abs(result.wheel_angles))),
                    MAXIMUM_WHEEL_ANGLE + 1.0e-12,
                )
                self.assertAlmostEqual(
                    float(result.wheel_angles[0]), MAXIMUM_WHEEL_ANGLE, places=12
                )

    def test_the_limit_does_not_apply_below_the_maximum(self) -> None:
        result = self._steer(90.0)

        self.assertFalse(result.is_limited)

    def test_the_reference_angle_follows_the_arm_geometry(self) -> None:
        result = self._steer(90.0)
        expected = math.atan2(result.rack_travel, STEER_ARM)

        self.assertAlmostEqual(result.reference_angle, expected, places=12)

    def test_ackermann_reduces_the_outer_wheel_only(self) -> None:
        table = AckermannTable(
            inner_angle=np.asarray([0.0, 1.0]),
            outer_offset=np.asarray([0.0, 0.05]),
        )
        right = self._steer(90.0, table=table)

        # A right turn makes the right wheel inner, so it turns further.
        self.assertGreater(
            float(right.wheel_angles[1]), float(right.wheel_angles[0])
        )
        self.assertAlmostEqual(
            float(right.wheel_angles[1] - right.wheel_angles[0]),
            table.offset(right.wheel_angles[1]),
            places=12,
        )

    def test_ackermann_mirrors_for_a_left_turn(self) -> None:
        table = AckermannTable(
            inner_angle=np.asarray([0.0, 1.0]),
            outer_offset=np.asarray([0.0, 0.05]),
        )
        right = self._steer(90.0, table=table)
        left = self._steer(-90.0, table=table)

        np.testing.assert_allclose(
            right.wheel_angles, -left.wheel_angles[[1, 0, 2, 3]], atol=1.0e-12
        )

    def test_rejects_bad_geometry(self) -> None:
        with self.assertRaises(ValueError):
            self._steer(90.0, maximum=0.0)
        with self.assertRaises(ValueError):
            steering_angles(
                1.0, RACK_TRAVEL_PER_REVOLUTION, 0.0, MAXIMUM_WHEEL_ANGLE, _parallel()
            )


class PedalRatioCurveTests(unittest.TestCase):
    def test_an_empty_curve_is_constant(self) -> None:
        curve = PedalRatioCurve(
            pedal_force=np.asarray([]), pedal_ratio=np.asarray([]), constant_ratio=5.5
        )

        self.assertTrue(curve.is_constant)
        for force in (0.0, 500.0, 1200.0):
            with self.subTest(force=force):
                self.assertEqual(curve.ratio(force), 5.5)

    def test_a_curve_interpolates_and_clamps(self) -> None:
        curve = PedalRatioCurve(
            pedal_force=np.asarray([0.0, 600.0, 1200.0]),
            pedal_ratio=np.asarray([4.0, 5.0, 6.0]),
            constant_ratio=5.0,
        )

        self.assertAlmostEqual(curve.ratio(300.0), 4.5, places=12)
        self.assertAlmostEqual(curve.ratio(-10.0), 4.0, places=12)
        self.assertAlmostEqual(curve.ratio(5000.0), 6.0, places=12)

    def test_rejects_a_bad_curve(self) -> None:
        with self.assertRaises(ValueError):
            PedalRatioCurve(
                pedal_force=np.asarray([600.0, 0.0]), pedal_ratio=np.ones(2)
            )
        with self.assertRaises(ValueError):
            PedalRatioCurve(
                pedal_force=np.asarray([0.0, 600.0]), pedal_ratio=np.asarray([4.0, -1.0])
            )
        with self.assertRaises(ValueError):
            PedalRatioCurve(
                pedal_force=np.asarray([]),
                pedal_ratio=np.asarray([]),
                constant_ratio=0.0,
            )


class BrakeTorqueTests(unittest.TestCase):
    """Check the force-based hydraulic pedal chain against a hand calculation.

    The chain input is the driver's pedal force in newtons; the 0-100 % brake
    position belongs to the driver robot, not to the vehicle.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model

    def _common(self, **overrides):
        vehicle = self.vehicle
        values = dict(
            maximum_pedal_force=vehicle.maximum_pedal_force,
            pedal_curve=vehicle.pedal_curve,
            hydraulic_ratio_front=vehicle.hydraulic_ratio_front,
            hydraulic_ratio_rear=vehicle.hydraulic_ratio_rear,
            maximum_brake_pressure=vehicle.maximum_brake_pressure,
            caliper_area=vehicle.caliper_area,
            brake_bias=vehicle.brake_bias,
            pad_mu_front=vehicle.pad_mu_front,
            pad_mu_rear=vehicle.pad_mu_rear,
            effective_radius_front=vehicle.brake_effective_radius_front,
            effective_radius_rear=vehicle.brake_effective_radius_rear,
        )
        values.update(overrides)
        return values

    def _brake(self, pedal_force, **overrides):
        return brake_torques(pedal_force, **self._common(**overrides))

    def test_murphy_hydraulic_parameters(self) -> None:
        vehicle = self.vehicle
        self.assertEqual(vehicle.pad_mu_front, 0.40)
        self.assertEqual(vehicle.pad_mu_rear, 0.35)
        self.assertEqual(vehicle.brake_bias, 0.0)
        self.assertTrue(vehicle.pedal_curve.is_constant)
        self.assertEqual(vehicle.pedal_curve.ratio(0.0), 1.55)
        self.assertEqual(vehicle.pedal_curve.ratio(1000.0), 1.55)

    def test_murphy_front_rear_torque_split_is_62_38(self) -> None:
        self.assertAlmostEqual(self.vehicle.front_brake_share, 0.62, places=6)
        for force in (50.0, 450.0, self.vehicle.maximum_pedal_force):
            torque = self._brake(force).wheel_torque_magnitudes
            with self.subTest(force=force):
                self.assertAlmostEqual(
                    float(torque[0] / (torque[0] + torque[2])), 0.62, places=6
                )

    def test_zero_pedal_gives_no_torque(self) -> None:
        result = self._brake(0.0)

        np.testing.assert_allclose(result.wheel_torques, 0.0)
        self.assertEqual(result.clamp_force, 0.0)

    def test_the_chain_matches_a_hand_calculation(self) -> None:
        vehicle = self.vehicle
        pedal = 400.0
        result = self._brake(pedal)

        pushrod = pedal * 1.55
        front_master = pushrod * 0.5 * (1.0 + vehicle.brake_bias)
        rear_master = pushrod - front_master
        front_clamp = front_master * vehicle.hydraulic_ratio_front
        rear_clamp = rear_master * vehicle.hydraulic_ratio_rear
        front = 2.0 * 0.40 * front_clamp * vehicle.brake_effective_radius_front
        rear = 2.0 * 0.35 * rear_clamp * vehicle.brake_effective_radius_rear
        self.assertAlmostEqual(result.pedal_force, pedal, places=12)
        self.assertAlmostEqual(result.pushrod_force, pushrod, places=9)
        self.assertAlmostEqual(result.front_master_cylinder_force, front_master, places=9)
        self.assertAlmostEqual(result.front_line_pressure,
                               front_clamp / vehicle.caliper_area, places=3)
        self.assertAlmostEqual(result.rear_line_pressure,
                               rear_clamp / vehicle.caliper_area, places=3)
        np.testing.assert_allclose(result.wheel_torque_magnitudes,
                                   [front, front, rear, rear], rtol=1.0e-12)

    def test_the_torque_is_linear_in_pedal_force_for_a_constant_ratio(self) -> None:
        light = self._brake(200.0).wheel_torque_magnitudes
        heavy = self._brake(600.0).wheel_torque_magnitudes
        np.testing.assert_allclose(heavy, 3.0 * light, rtol=1.0e-12)

    def test_the_signed_bias_moves_force_between_master_cylinders(self) -> None:
        for bias, front_share in ((0.0, 0.5), (0.5, 0.75), (-0.5, 0.25),
                                  (1.0, 1.0), (-1.0, 0.0)):
            result = self._brake(400.0, brake_bias=bias)
            with self.subTest(bias=bias):
                self.assertAlmostEqual(
                    result.front_master_cylinder_force / result.pushrod_force,
                    front_share, places=12,
                )
        self.assertEqual(float(self._brake(400.0, brake_bias=1.0)
                               .wheel_torque_magnitudes[2]), 0.0)
        self.assertEqual(float(self._brake(400.0, brake_bias=-1.0)
                               .wheel_torque_magnitudes[0]), 0.0)

    def test_the_share_helper_matches_the_chain(self) -> None:
        for bias in (-0.6, 0.0, 0.3):
            torque = self._brake(300.0, brake_bias=bias).wheel_torque_magnitudes
            common = self._common(brake_bias=bias)
            share = front_brake_torque_share(
                bias, common["hydraulic_ratio_front"], common["hydraulic_ratio_rear"],
                common["pad_mu_front"], common["pad_mu_rear"],
                common["effective_radius_front"], common["effective_radius_rear"],
            )
            with self.subTest(bias=bias):
                self.assertAlmostEqual(float(torque[0] / (torque[0] + torque[2])),
                                       share, places=12)
        self.assertEqual(front_master_cylinder_share(0.0), 0.5)

    def test_the_two_wheels_of_an_axle_match(self) -> None:
        result = self._brake(500.0)

        self.assertEqual(float(result.wheel_torque_magnitudes[0]),
                         float(result.wheel_torque_magnitudes[1]))
        self.assertEqual(float(result.wheel_torque_magnitudes[2]),
                         float(result.wheel_torque_magnitudes[3]))

    def test_the_full_pedal_force_is_driver_limited(self) -> None:
        vehicle = self.vehicle
        result = self._brake(vehicle.maximum_pedal_force)

        self.assertAlmostEqual(result.pedal_force, vehicle.maximum_pedal_force, places=9)
        self.assertFalse(result.is_pressure_limited)
        self.assertTrue(result.is_driver_limited)

    def test_450_newtons_is_about_one_g(self) -> None:
        vehicle = self.vehicle
        result = self._brake(450.0)

        tire_radius = 0.19
        force = 2.0 * float(np.sum(result.wheel_torque_magnitudes[[0, 2]])) / tire_radius
        deceleration = force / (vehicle.total_mass * 9.81)
        self.assertAlmostEqual(deceleration, 1.0, delta=0.12)

    def test_full_pedal_brakes_harder_than_the_tire_can_hold(self) -> None:
        vehicle = self.vehicle
        result = self._brake(vehicle.maximum_pedal_force)

        tire_radius = 0.19
        force = 2.0 * float(np.sum(result.wheel_torque_magnitudes[[0, 2]])) / tire_radius
        self.assertGreater(force / (vehicle.total_mass * 9.81), 1.5)

    def test_a_low_pressure_limit_binds_and_keeps_the_split(self) -> None:
        vehicle = self.vehicle
        result = self._brake(vehicle.maximum_pedal_force, maximum_brake_pressure=2.0e6)

        self.assertTrue(result.is_pressure_limited)
        self.assertLessEqual(result.line_pressure, 2.0e6 + 1.0e-6)
        self.assertFalse(result.is_driver_limited)
        torque = result.wheel_torque_magnitudes
        self.assertAlmostEqual(float(torque[0] / (torque[0] + torque[2])),
                               vehicle.front_brake_share, places=12)
        self.assertLess(float(torque[0]), float(maximum_brake_torques(**self._common())[0]))

    def test_rejects_bad_inputs(self) -> None:
        vehicle = self.vehicle
        common = self._common()
        for force in (-0.1, vehicle.maximum_pedal_force + 1.0, float("nan")):
            with self.subTest(force=force), self.assertRaises(ValueError):
                brake_torques(force, **common)
        for name, value in (
            ("brake_bias", 1.01),
            ("brake_bias", -1.01),
            ("caliper_area", 0.0),
            ("hydraulic_ratio_front", -1.0),
            ("hydraulic_ratio_rear", 0.0),
            ("maximum_pedal_force", 0.0),
            ("pad_mu_rear", 0.0),
        ):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                brake_torques(100.0, **{**common, name: value})


if __name__ == "__main__":
    unittest.main()
