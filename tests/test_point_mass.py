from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import unittest

import numpy as np

from simplified_vehicle_models.core.conventions import (
    AXIS_DOWN,
    AXIS_FORWARD,
    AXIS_RIGHT,
    GRAVITY,
    LEFT_TURN_CURVATURE_SIGN,
    RIGHT_TURN_CURVATURE_SIGN,
    VEHICLE_AXIS_CONVENTION,
    WHEEL_ORDER,
)
from simplified_vehicle_models.powertrain.assembly import FullDrivetrainRuntime, maximum_drive_force
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope
from simplified_vehicle_models.formulations.point_mass import (
    acceleration_limits,
    aerodynamic_forces,
    evaluate_equilibrium,
    lateral_transfer_loads,
    maximum_acceleration_limit,
    maximum_deceleration_limit,
    _equilibrium_limits,
    maximum_corner_speed,
    maximum_vehicle_speed,
    normal_loads,
    tire_force_capacities,
    _available_drive_force,
    _positive_root,
    DriveCache,
    DriveCachePolicy,
)
from simplified_vehicle_models.drivetrain.lumped import compile_lumped_drivetrain
from simplified_vehicle_models.solvers import forward_backward
from simplified_vehicle_models.solvers.forward_backward import _build_result
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.xml.loader import load_model, load_point_mass_vehicle


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"


class PointMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = load_point_mass_vehicle(FORMULATION_PATH)

    def test_unitire_peak_lookup_drives_point_mass_tire_capacity(self) -> None:
        tire = UniTirePeakEnvelope(
            vertical_loads=np.array([0.0, 1000.0, 2000.0]),
            longitudinal_peak_force=np.array([0.0, 2100.0, 4000.0]),
            lateral_peak_force=np.array([0.0, 2200.0, 4200.0]),
            effective_radius=0.2,
            reference_speed=15.0,
        )
        vehicle = replace(self.vehicle, tire=tire)
        loads = np.array([500.0, 1000.0, 1500.0])

        longitudinal, lateral = tire_force_capacities(vehicle, loads)

        np.testing.assert_allclose(longitudinal, [1050.0, 2100.0, 3050.0])
        np.testing.assert_allclose(lateral, [1100.0, 2200.0, 3200.0])

    def test_static_loads_sum_to_vehicle_weight(self) -> None:
        loads = normal_loads(self.vehicle, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(float(np.sum(loads)), self.vehicle.total_mass * GRAVITY)
        self.assertAlmostEqual(float(loads[0]), float(loads[1]))
        self.assertAlmostEqual(float(loads[2]), float(loads[3]))

    def test_aero_loads_scale_with_speed_squared(self) -> None:
        downforce_10, drag_10 = aerodynamic_forces(self.vehicle, 10.0)
        downforce_20, drag_20 = aerodynamic_forces(self.vehicle, 20.0)

        self.assertAlmostEqual(downforce_20, 4.0 * downforce_10)
        self.assertAlmostEqual(drag_20, 4.0 * drag_10)

    def test_longitudinal_load_transfer_has_correct_direction(self) -> None:
        static = normal_loads(self.vehicle, 0.0, 0.0, 0.0)
        accelerating = normal_loads(self.vehicle, 0.0, 5.0, 0.0)
        braking = normal_loads(self.vehicle, 0.0, -5.0, 0.0)

        self.assertLess(float(np.sum(accelerating[:2])), float(np.sum(static[:2])))
        self.assertGreater(float(np.sum(braking[:2])), float(np.sum(static[:2])))

    def test_lateral_transfer_conserves_total_load(self) -> None:
        straight = normal_loads(self.vehicle, 12.0, 0.0, 0.0)
        turning = normal_loads(self.vehicle, 12.0, 0.0, 6.0)

        self.assertAlmostEqual(float(np.sum(straight)), float(np.sum(turning)))
        self.assertGreater(float(turning[0]), float(turning[1]))
        self.assertGreater(float(turning[2]), float(turning[3]))

    def test_axis_convention_is_sae(self) -> None:
        self.assertEqual(VEHICLE_AXIS_CONVENTION, "sae-j670")
        self.assertEqual((AXIS_FORWARD, AXIS_RIGHT, AXIS_DOWN), ("x", "y", "z"))
        self.assertEqual(WHEEL_ORDER, ("fl", "fr", "rl", "rr"))
        self.assertEqual(RIGHT_TURN_CURVATURE_SIGN, 1.0)
        self.assertEqual(LEFT_TURN_CURVATURE_SIGN, -1.0)

    def test_positive_lateral_acceleration_is_a_right_turn(self) -> None:
        right_turn = normal_loads(self.vehicle, 12.0, 0.0, 6.0)

        self.assertGreater(float(right_turn[0]), float(right_turn[1]))
        self.assertGreater(float(right_turn[2]), float(right_turn[3]))

    def test_negative_lateral_acceleration_is_a_left_turn(self) -> None:
        left_turn = normal_loads(self.vehicle, 12.0, 0.0, -6.0)

        self.assertLess(float(left_turn[0]), float(left_turn[1]))
        self.assertLess(float(left_turn[2]), float(left_turn[3]))

    def test_turn_direction_mirrors_the_wheel_loads(self) -> None:
        right_turn = normal_loads(self.vehicle, 12.0, 0.0, 6.0)
        left_turn = normal_loads(self.vehicle, 12.0, 0.0, -6.0)

        np.testing.assert_allclose(right_turn, left_turn[[1, 0, 3, 2]])

    def test_right_turn_transfer_matches_the_hand_calculation(self) -> None:
        lateral_acceleration = 6.0
        total_transfer = (
            self.vehicle.total_mass
            * lateral_acceleration
            * self.vehicle.cg_height
            / self.vehicle.track_width
        )
        front_transfer = (
            total_transfer * self.vehicle.lateral_load_transfer_distribution
        )
        loads = normal_loads(self.vehicle, 0.0, 0.0, lateral_acceleration)

        # The left wheel gains the front transfer and the right wheel loses it.
        self.assertAlmostEqual(
            float(loads[0] - loads[1]),
            2.0 * front_transfer,
        )
        self.assertAlmostEqual(
            float(loads[2] - loads[3]),
            2.0 * (total_transfer - front_transfer),
        )

    def test_lateral_transfer_helper_conserves_each_axle(self) -> None:
        loads = lateral_transfer_loads(self.vehicle, 1000.0, 1200.0, 6.0)

        self.assertAlmostEqual(loads[0] + loads[1], 1000.0)
        self.assertAlmostEqual(loads[2] + loads[3], 1200.0)
        self.assertGreater(loads[0], loads[1])
        self.assertGreater(loads[2], loads[3])

    def test_positive_curvature_is_a_right_turn(self) -> None:
        right_turn = evaluate_equilibrium(self.vehicle, 12.0, 1.0 / 20.0, 0.0)
        left_turn = evaluate_equilibrium(self.vehicle, 12.0, -1.0 / 20.0, 0.0)

        self.assertGreater(right_turn.lateral_acceleration, 0.0)
        self.assertLess(left_turn.lateral_acceleration, 0.0)
        self.assertGreater(
            float(right_turn.normal_loads[0]),
            float(right_turn.normal_loads[1]),
        )
        self.assertLess(
            float(left_turn.normal_loads[0]),
            float(left_turn.normal_loads[1]),
        )

    def test_turn_sign_does_not_change_the_scalar_limits(self) -> None:
        storage = StorageState(0.8)
        for speed, magnitude in ((15.0, 0.01), (25.0, 1.0 / 30.0)):
            self.assertAlmostEqual(
                maximum_acceleration_limit(self.vehicle, speed, magnitude, storage),
                maximum_acceleration_limit(self.vehicle, speed, -magnitude, storage),
                places=10,
            )
            self.assertAlmostEqual(
                maximum_deceleration_limit(self.vehicle, speed, magnitude, storage),
                maximum_deceleration_limit(self.vehicle, speed, -magnitude, storage),
                places=10,
            )

    def test_load_sensitivity_reduces_effective_friction(self) -> None:
        loads = np.asarray([300.0, 600.0, 900.0, 1200.0])
        longitudinal, lateral = tire_force_capacities(self.vehicle, loads)

        self.assertTrue(np.all(np.diff(longitudinal / loads) < 0.0))
        self.assertTrue(np.all(np.diff(lateral / loads) < 0.0))

    def test_measured_friction_is_physically_plausible(self) -> None:
        loads = np.asarray([700.0, 850.0, 1000.0, 1200.0])
        longitudinal, lateral = tire_force_capacities(self.vehicle, loads)
        lateral_friction = lateral / loads
        longitudinal_friction = longitudinal / loads

        self.assertTrue(np.all(lateral_friction >= 1.0))
        self.assertTrue(np.all(lateral_friction <= 1.4))
        self.assertTrue(np.all(longitudinal_friction >= 0.5))
        self.assertTrue(np.all(longitudinal_friction <= 0.8))

    def test_unloaded_tire_has_zero_capacity(self) -> None:
        longitudinal, lateral = tire_force_capacities(
            self.vehicle,
            np.asarray([0.0, 0.0, 0.0, 0.0]),
        )

        np.testing.assert_array_equal(longitudinal, 0.0)
        np.testing.assert_array_equal(lateral, 0.0)

    def test_live_limits_are_finite(self) -> None:
        launch = acceleration_limits(self.vehicle, 0.0, 0.0)
        moving = acceleration_limits(self.vehicle, 20.0, 0.0)

        self.assertGreater(launch.maximum_acceleration, 0.0)
        self.assertGreater(launch.maximum_deceleration, 0.0)
        self.assertGreater(moving.maximum_acceleration, 0.0)
        self.assertGreater(moving.maximum_deceleration, 0.0)
        self.assertTrue(np.isfinite(moving.maximum_acceleration))

    def test_exact_drive_cache_is_solver_local_and_reuses_identical_points(self) -> None:
        cache = DriveCache(DriveCachePolicy())
        limits = np.full(4, 10_000.0)
        first = _available_drive_force(self.vehicle, 10.0, limits, drive_cache=cache)
        second = _available_drive_force(self.vehicle, 10.0, limits, drive_cache=cache)
        other = DriveCache(DriveCachePolicy())
        fresh = _available_drive_force(self.vehicle, 10.0, limits, drive_cache=other)
        self.assertEqual(first, second)
        self.assertEqual(first, fresh)
        self.assertEqual(cache.hits, 1)
        self.assertEqual(len(cache.values), 1)
        self.assertEqual(len(other.values), 1)
        self.assertEqual(cache.max_speed_displacement, 0.0)
        self.assertEqual(cache.max_soc_displacement, 0.0)

    def test_final_equilibrium_evaluations_use_solver_cache(self) -> None:
        track = TrackProfile(
            distance=np.asarray([0.0, 1.0, 2.0]),
            segment_length=np.asarray([1.0, 1.0]),
            radius=np.asarray([np.inf, np.inf, np.inf]),
            curvature=np.zeros(3),
            is_closed=False,
            title="cache regression",
        )
        speed = np.full(3, 10.0)
        cache = DriveCache(DriveCachePolicy(speed_resolution=1.0))
        with patch.object(
            forward_backward,
            "evaluate_equilibrium",
            wraps=forward_backward.evaluate_equilibrium,
        ) as evaluate:
            _build_result(self.vehicle, track, speed, True, 1, 0.0, drive_cache=cache)

        self.assertEqual(evaluate.call_count, 2)
        for call in evaluate.call_args_list:
            self.assertIs(call.args[5], cache)

    def test_quantized_drive_cache_reports_bucket_displacement(self) -> None:
        cache = DriveCache(DriveCachePolicy(speed_resolution=1.0, soc_resolution=0.1))
        limits = np.full(4, 10_000.0)
        state = StorageState(0.94)
        _available_drive_force(self.vehicle, 10.0, limits, state, cache)
        _available_drive_force(self.vehicle, 10.4, limits, StorageState(0.95), cache)
        self.assertEqual(cache.hits, 1)
        self.assertGreater(cache.max_speed_displacement, 0.0)
        self.assertGreater(cache.max_soc_displacement, 0.0)

    def test_drive_cache_policy_is_immutable_and_validated(self) -> None:
        policy = DriveCachePolicy()
        with self.assertRaises(Exception):
            policy.speed_resolution = 1.0
        with self.assertRaises(ValueError):
            DriveCachePolicy(speed_resolution=-1.0)

    def test_assembled_dispatch_matches_compiled_drive_capability(self) -> None:
        self.assertIsInstance(self.vehicle.drivetrain, FullDrivetrainRuntime)
        tire_limits = np.full(4, 10_000.0)
        force, constraint = _available_drive_force(self.vehicle, 10.0, tire_limits)
        initial_soc = (
            self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge
        )
        expected = maximum_drive_force(
            self.vehicle.drivetrain.powertrain,
            self.vehicle.drivetrain.drivetrain,
            10.0,
            StorageState(initial_soc),
            effective_radius=self.vehicle.tire.effective_radius,
        )
        self.assertEqual(constraint, expected.active_constraint)
        self.assertAlmostEqual(force, expected.maximum_drive_force, places=9)

    def test_neighboring_speeds_dispatch_independently(self) -> None:
        self.assertIsInstance(self.vehicle.drivetrain, FullDrivetrainRuntime)
        tire_limits = np.full(4, 10_000.0)
        speeds = (10.0, 10.1)
        initial_soc = (
            self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge
        )
        actual = [
            _available_drive_force(self.vehicle, speed, tire_limits)[0]
            for speed in speeds
        ]
        expected = [
            maximum_drive_force(
                self.vehicle.drivetrain.powertrain,
                self.vehicle.drivetrain.drivetrain,
                speed,
                StorageState(initial_soc),
                effective_radius=self.vehicle.tire.effective_radius,
            ).maximum_drive_force
            for speed in speeds
        ]

        self.assertNotEqual(expected[0], expected[1])
        for actual_force, expected_force in zip(actual, expected):
            self.assertAlmostEqual(actual_force, expected_force, places=9)

    def test_legacy_dispatch_keeps_scalar_force_reduction(self) -> None:
        legacy = compile_lumped_drivetrain(
            load_model(REPOSITORY_ROOT / "models/drivetrains/murphy_4wd.xml")
        )
        vehicle = replace(self.vehicle, drivetrain=legacy)
        force, constraint = _available_drive_force(vehicle, 10.0, np.full(4, 10_000.0))
        expected = min(
            legacy.motor_count
            * legacy.peak_motor_torque
            * legacy.gear_ratio
            * legacy.efficiency
            / vehicle.tire.effective_radius,
            legacy.power_limit * legacy.efficiency / 10.0,
        )
        self.assertAlmostEqual(force, expected, places=9)
        self.assertIn(constraint, {"motor_torque", "power"})

    def test_drive_force_does_not_exceed_power_limit(self) -> None:
        speed = 30.0
        limits = acceleration_limits(self.vehicle, speed, 0.0)
        if isinstance(self.vehicle.drivetrain, FullDrivetrainRuntime):
            assembled = maximum_drive_force(
                self.vehicle.drivetrain.powertrain,
                self.vehicle.drivetrain.drivetrain,
                speed,
                StorageState(
                    self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge
                ),
                effective_radius=self.vehicle.tire.effective_radius,
            ).maximum_drive_force
            self.assertLessEqual(limits.acceleration_state.drive_force_limit, assembled)
        else:
            power_force = (
                self.vehicle.drivetrain.power_limit
                * self.vehicle.drivetrain.efficiency
                / speed
            )
            self.assertLessEqual(limits.acceleration_state.drive_force_limit, power_force)

    def test_lateral_forces_close_yaw_moment(self) -> None:
        for curvature in (1.0 / 20.0, -1.0 / 20.0):
            state = evaluate_equilibrium(self.vehicle, 10.0, curvature, -2.0)

            self.assertTrue(state.is_feasible)
            self.assertAlmostEqual(state.yaw_moment_residual, 0.0, places=10)

    def test_wheel_lift_is_infeasible_without_added_load(self) -> None:
        state = evaluate_equilibrium(self.vehicle, 25.0, 1.0 / 3.0, 0.0)
        expected_load = (
            self.vehicle.total_mass * GRAVITY
            + aerodynamic_forces(self.vehicle, 25.0)[0]
        )

        self.assertFalse(state.contact_feasible)
        self.assertAlmostEqual(float(np.sum(state.normal_loads)), expected_load)

    def test_limit_diagnostics_match_solved_accelerations(self) -> None:
        limits = acceleration_limits(self.vehicle, 15.0, 1.0 / 30.0)

        self.assertAlmostEqual(
            limits.acceleration_state.longitudinal_acceleration,
            limits.maximum_acceleration,
        )
        self.assertAlmostEqual(
            limits.braking_state.longitudinal_acceleration,
            -limits.maximum_deceleration,
        )

    def test_directional_limits_match_full_limits(self) -> None:
        storage = StorageState(0.8)
        limits = acceleration_limits(self.vehicle, 15.0, 0.01, storage)
        self.assertAlmostEqual(
            maximum_acceleration_limit(self.vehicle, 15.0, 0.01, storage),
            limits.maximum_acceleration,
            places=8,
        )
        self.assertAlmostEqual(
            maximum_deceleration_limit(self.vehicle, 15.0, 0.01, storage),
            limits.maximum_deceleration,
            places=8,
        )

    def test_scalar_equilibrium_kernel_matches_diagnostics(self) -> None:
        for speed, curvature, acceleration, soc in (
            (15.0, 0.01, 4.0, 0.8),
            (15.0, 0.01, -4.0, 0.8),
            (25.0, 1.0 / 30.0, 0.0, 0.2),
            (10.0, 0.0, 3.0, 1.0),
        ):
            storage = StorageState(soc)
            state = evaluate_equilibrium(
                self.vehicle, speed, curvature, acceleration, storage
            )
            contact, lateral, drag, drive, brake = _equilibrium_limits(
                self.vehicle, speed, curvature, acceleration, storage
            )
            self.assertEqual(contact, state.contact_feasible)
            self.assertEqual(lateral, state.lateral_feasible)
            self.assertAlmostEqual(drag, state.drag_force, places=12)
            self.assertAlmostEqual(drive, state.drive_force_limit, places=12)
            self.assertAlmostEqual(brake, state.brake_force_limit, places=12)

    def test_tighter_corner_has_lower_speed(self) -> None:
        tight = maximum_corner_speed(self.vehicle, 8.0)
        open_corner = maximum_corner_speed(self.vehicle, 40.0)

        self.assertGreater(tight, 0.0)
        self.assertLess(tight, open_corner)
        self.assertLessEqual(open_corner, maximum_vehicle_speed(self.vehicle))

    def test_positive_root_hint_preserves_root_and_reduces_evaluations(self) -> None:
        tolerance = 1.0e-8
        calls = {"cold": 0, "warm": 0}

        def cold(value: float) -> float:
            calls["cold"] += 1
            return 4.0 - value

        def warm(value: float) -> float:
            calls["warm"] += 1
            return 4.0 - value

        unhinted = _positive_root(cold, 16.0, tolerance=tolerance)
        hinted = _positive_root(warm, 16.0, tolerance=tolerance, hint=4.1)

        self.assertLessEqual(abs(unhinted - hinted), 2.0 * tolerance)
        self.assertLess(calls["warm"], calls["cold"])

    def test_corner_speed_hint_is_equivalent_from_both_sides(self) -> None:
        tolerance = 1.0e-8
        unhinted = maximum_corner_speed(self.vehicle, 8.0, tolerance=tolerance)
        lower_hint = maximum_corner_speed(
            self.vehicle, 8.0, tolerance=tolerance, hint=unhinted * 0.9
        )
        upper_hint = maximum_corner_speed(
            self.vehicle, 8.0, tolerance=tolerance, hint=unhinted * 1.1
        )

        self.assertLessEqual(abs(unhinted - lower_hint), 2.0 * tolerance)
        self.assertLessEqual(abs(unhinted - upper_hint), 2.0 * tolerance)


if __name__ == "__main__":
    unittest.main()
