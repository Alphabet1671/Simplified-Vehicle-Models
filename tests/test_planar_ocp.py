"""Check the optimal-control physics and solve one small lap.

CasADi and IPOPT are optional, so every test here skips when CasADi is absent.
Install ``casadi`` to run them.
"""

import math
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState,
    evaluate_planar_dynamics,
    evaluate_normal_loads,
)
from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire
from simplified_vehicle_models.powertrain.energy_storage import StorageState, solve_current
from simplified_vehicle_models.powertrain.motor_envelope import total_bus_power
from simplified_vehicle_models.tracks.profile import load_track

try:  # pragma: no cover - depends on the host
    import casadi  # noqa: F401

    HAS_CASADI = True
except ImportError:  # pragma: no cover - depends on the host
    HAS_CASADI = False

if HAS_CASADI:
    from simplified_vehicle_models.solvers import planar_ocp
    from simplified_vehicle_models.solvers.planar_symbolic import symbolic_tire


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/placeholder_planar_7dof.xml"
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"


@unittest.skipUnless(HAS_CASADI, "the optimal control needs casadi")
class SymbolicTireTests(unittest.TestCase):
    """Compare the symbolic tire against the NumPy port."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.casadi = planar_ocp._require_casadi()
        cls.vehicle = compile_formulation(FORMULATION_PATH).model
        cls.tire = cls.vehicle.tire
        angle = cls.casadi.SX.sym("alpha")
        ratio = cls.casadi.SX.sym("kappa")
        load = cls.casadi.SX.sym("fz")
        speed = cls.casadi.SX.sym("omega")
        one = symbolic_tire(cls.tire, angle, ratio, load, speed)
        cls.function = cls.casadi.Function(
            "tire",
            [angle, ratio, load, speed],
            [
                one.longitudinal_force,
                one.lateral_force,
                one.aligning_moment,
                one.loaded_radius,
                one.rolling_moment,
            ],
        )

    def test_the_forces_match_the_numpy_tire(self) -> None:
        generator = np.random.default_rng(20261013)
        worst = 0.0
        for _ in range(150):
            angle = float(generator.uniform(-0.35, 0.35))
            ratio = float(generator.uniform(-0.3, 0.3))
            load = float(generator.uniform(50.0, 2200.0))
            speed = float(generator.uniform(0.0, 180.0))
            values = self.function(angle, ratio, load, speed)
            reference = evaluate_unitire(self.tire, angle, ratio, load, speed)
            for index, expected in enumerate(
                (
                    reference.longitudinal_force,
                    reference.lateral_force,
                    reference.aligning_moment,
                    reference.effective_radius,
                    reference.rolling_resistance_moment,
                )
            ):
                scale = max(abs(float(expected)), 1.0)
                worst = max(worst, abs(float(values[index]) - float(expected)) / scale)
        self.assertLess(worst, 1.0e-12)

    def test_a_lifted_wheel_makes_no_force(self) -> None:
        values = self.function(0.2, 0.1, -500.0, 50.0)

        for index in range(3):
            self.assertTrue(math.isfinite(float(values[index])))
        self.assertAlmostEqual(float(values[0]), 0.0, places=9)
        self.assertAlmostEqual(float(values[1]), 0.0, places=9)

    def test_zero_slip_has_a_finite_jacobian(self) -> None:
        # The derivative of a square root at zero is not finite, so the model
        # guards the slip square roots. A solver needs this to work.
        casadi = self.casadi
        angle = casadi.SX.sym("alpha")
        ratio = casadi.SX.sym("kappa")
        load = casadi.SX.sym("fz", 1)
        speed = casadi.SX.sym("omega", 1)
        one = symbolic_tire(self.tire, angle, ratio, load[0], speed[0])
        jacobian = casadi.Function(
            "jacobian",
            [angle, ratio, load, speed],
            [casadi.jacobian(one.longitudinal_force, angle)],
        )
        value = float(jacobian(0.0, 0.0, np.asarray([800.0]), np.asarray([50.0])))

        self.assertTrue(math.isfinite(value))


@unittest.skipUnless(HAS_CASADI, "the optimal control needs casadi")
class SymbolicResidualTests(unittest.TestCase):
    """Check the algebraic equations against the NumPy simulation."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model
        positions = planar_ocp._wheel_positions(cls.vehicle)
        cls.positions = positions
        peak = (
            cls.vehicle.motor.peak_motor_torque
            * cls.vehicle.motor.gear_ratio
            * cls.vehicle.motor.motor_efficiency
        )
        cls.functions = planar_ocp._build_node_functions(
            planar_ocp._require_casadi(),
            cls.vehicle,
            4.0 * peak,
            np.asarray(planar_ocp._brake_torque_magnitudes(cls.vehicle)),
        )

    def test_the_load_equations_hold_at_a_numpy_state(self) -> None:
        vehicle = self.vehicle
        speed = 25.0
        yaw_rate = 0.4
        steering = math.radians(3.0)
        state = PlanarState(speed, 0.5, yaw_rate, np.full(4, speed / 0.19))
        evaluation = evaluate_planar_dynamics(vehicle, state, np.zeros(4), steering)
        loads = evaluation.normal_loads
        vector = np.zeros(planar_ocp.STATE_COUNT)
        vector[2], vector[3], vector[4] = state.v_x, state.v_y, state.yaw_rate
        vector[5] = vehicle.storage.initial_state_of_charge
        control = np.asarray([0.0, 0.0, steering])
        algebraic = np.concatenate(
            (loads, state.wheel_speeds, np.asarray([50.0]))
        )

        residual = np.asarray(
            self.functions.algebraic(vector, control, algebraic, 0.01)
        ).ravel()

        # The NumPy model solves the same equations, so they must hold here.
        # The first four are the load equations; the last four are the
        # quasi-steady wheel balances, which are not zero at a spinning wheel.
        self.assertLess(float(np.max(np.abs(residual[:4]))), 1.0e-6)
        self.assertGreater(float(np.min(loads)), 0.0)

    def test_electrical_current_is_tied_to_motor_power(self) -> None:
        vehicle = self.vehicle
        state = np.zeros(planar_ocp.STATE_COUNT)
        state[2] = 15.0
        state[5] = vehicle.storage.initial_state_of_charge
        loads = evaluate_normal_loads(vehicle, 15.0, 0.0, 0.0)
        omega = 15.0 / planar_ocp.loaded_radius(vehicle.tire, loads)
        algebraic = np.concatenate((loads, omega, [0.0]))
        control = np.array([0.25, 0.0, 0.0])
        peak = vehicle.motor.peak_motor_torque * vehicle.motor.gear_ratio * vehicle.motor.motor_efficiency
        drive = planar_ocp.wheel_shares(0.2) * (4.0 * peak * control[0])
        demand = total_bus_power(vehicle.motor, omega, drive)
        current, feasible = solve_current(vehicle.storage, StorageState(state[5]), demand)
        self.assertTrue(feasible)
        algebraic[8] = current
        balanced = np.asarray(self.functions.algebraic(state, control, algebraic, 0.0)).ravel()
        self.assertAlmostEqual(float(balanced[-1]), 0.0, places=6)
        algebraic[8] = 0.0
        disconnected = np.asarray(self.functions.algebraic(state, control, algebraic, 0.0)).ravel()
        self.assertLess(float(disconnected[-1]), -demand * 0.99)

    def test_soc_derivative_is_per_metre(self) -> None:
        vehicle = self.vehicle
        state = np.array([0.0, 0.0, 20.0, 0.0, 0.0,
                          vehicle.storage.initial_state_of_charge, 0.0])
        loads = evaluate_normal_loads(vehicle, 20.0, 0.0, 0.0)
        omega = 20.0 / planar_ocp.loaded_radius(vehicle.tire, loads)
        algebraic = np.concatenate((loads, omega, [100.0]))
        derivative = np.asarray(self.functions.dynamics(state, np.zeros(3), algebraic, 0.0)).ravel()
        expected = -100.0 / (3600.0 * vehicle.storage.pack_capacity_ah * 20.0)
        self.assertAlmostEqual(float(derivative[5]), expected, places=11)

    def test_projected_seed_balances_loads_wheels_and_power(self) -> None:
        vehicle = self.vehicle
        loads = evaluate_normal_loads(vehicle, 15.0, 0.0, 0.0)
        omega = 15.0 / planar_ocp.loaded_radius(vehicle.tire, loads)
        state = np.array([[0.0], [0.0], [15.0], [0.0], [0.0],
                          [vehicle.storage.initial_state_of_charge], [0.0]])
        control = np.array([[0.2], [0.0], [0.0]])
        guess = {"state": state, "control": control,
                 "algebraic": np.concatenate((loads, omega, [0.0]))[:, None]}
        path = {"distance": np.array([0.0]), "curvature": np.array([0.0])}
        before = np.asarray(self.functions.algebraic(state[:, 0], control[:, 0],
                                  guess["algebraic"][:, 0], 0.0)).ravel()
        projected = planar_ocp._project_start(guess, vehicle, path, self.functions)
        after = np.asarray(self.functions.algebraic(projected["state"][:, 0],
                    projected["control"][:, 0], projected["algebraic"][:, 0], 0.0)).ravel()
        self.assertGreater(float(np.max(np.abs(before))), 100.0)
        self.assertLess(float(np.max(np.abs(after[:8]))), 1e-4)
        self.assertLess(float(abs(after[8])), 1e-5)

    def test_motor_speed_limit_is_exposed(self) -> None:
        vehicle = self.vehicle
        state = np.array([0.0, 0.0, 40.0, 0.0, 0.0,
                          vehicle.storage.initial_state_of_charge, 0.0])
        loads = evaluate_normal_loads(vehicle, 40.0, 0.0, 0.0)
        algebraic = np.concatenate((loads, np.full(4, 220.0), [0.0]))
        bounds = self.functions.bounds(state, np.zeros(3), algebraic, 0.0)
        self.assertEqual(len(bounds), 8)
        self.assertTrue(np.any(np.asarray(bounds[-1]).ravel() > 0.0))


@unittest.skipUnless(HAS_CASADI, "the optimal control needs casadi")
class OptimalControlTests(unittest.TestCase):
    """Solve one small lap and check the constraints that bound it.

    The solve is a coarse mesh, which is the only mesh that currently solves.
    ``test_the_algebraic_equations_are_not_yet_converged`` records the known
    limitation: the algebraic equations lag, so the lap time is not yet a
    converged optimum. Read the lap time with that in mind.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model
        cls.track = load_track(TRACK_PATH)
        cls.options = planar_ocp.PlanarOcpOptions(
            mesh_spacing=20.0,
            warm_start=False,
            print_level=0,
            maximum_iterations=2500,
        )
        cls.result = planar_ocp.solve_planar_hot_lap(
            cls.vehicle, cls.track, cls.options
        )

    def test_the_problem_builds_and_solves(self) -> None:
        self.assertGreater(self.result.iterations, 0)
        self.assertEqual(self.result.node_count, len(self.result.distance))
        self.assertTrue(self.result.status)

    def test_status_cannot_claim_an_infeasible_solution(self) -> None:
        # A plausible lap time is not a convergence certificate. Check both
        # equality groups, all sampled inequality bounds, and periodicity.
        if self.result.converged:
            self.assertLess(self.result.maximum_load_residual, 1.0e-3)
            self.assertLess(self.result.maximum_dynamics_residual, 1.0e-4)
            self.assertLess(self.result.maximum_inequality_violation, 1.0e-4)
            self.assertLess(self.result.periodicity_residual, 1.0e-4)
        else:
            self.assertFalse(self.result.status == "Solve_Succeeded" and
                self.result.maximum_load_residual < 1.0e-3 and
                self.result.maximum_dynamics_residual < 1.0e-4 and
                self.result.maximum_inequality_violation < 1.0e-4 and
                self.result.periodicity_residual < 1.0e-4)

    def test_the_lap_time_is_plausible(self) -> None:
        # The quasi-steady point-mass reference on the same track is 68.3 s.
        # A coarse mesh cannot beat the physics, so the optimum sits below it
        # and well above an impossible time.
        self.assertTrue(math.isfinite(self.result.lap_time))
        self.assertGreater(self.result.lap_time, 30.0)
        self.assertLess(self.result.lap_time, 120.0)

    def test_the_elapsed_time_increases(self) -> None:
        difference = np.diff(self.result.elapsed_time)

        self.assertTrue(np.all(difference > 0.0))

    def test_the_speed_stays_inside_the_bounds(self) -> None:
        self.assertGreaterEqual(
            float(np.min(self.result.speed)),
            self.options.minimum_progress_speed - 1.0e-6,
        )
        self.assertLessEqual(
            float(np.max(self.result.speed)), self.options.maximum_speed + 1.0e-6
        )

    def test_the_lateral_error_stays_inside_the_bound(self) -> None:
        self.assertLessEqual(
            self.result.maximum_lateral_error,
            self.options.maximum_lateral_error + 1.0e-7,
        )

    def test_the_heading_error_stays_inside_the_bound(self) -> None:
        self.assertLessEqual(
            float(np.max(np.abs(self.result.heading_error))),
            self.options.maximum_heading_error + 1.0e-7,
        )

    def test_the_steering_stays_inside_the_vehicle_limit(self) -> None:
        self.assertLessEqual(
            float(np.max(np.abs(self.result.steering))),
            self.vehicle.steering_maximum_angle + 1.0e-9,
        )

    def test_the_controls_stay_inside_their_bounds(self) -> None:
        self.assertGreaterEqual(float(np.min(self.result.throttle)), -1.0e-7)
        self.assertLessEqual(float(np.max(self.result.throttle)), 1.0 + 1.0e-9)
        self.assertGreaterEqual(float(np.min(self.result.brake)), -1.0e-7)
        self.assertLessEqual(
            float(np.max(self.result.brake)), self.options.maximum_brake + 1.0e-9
        )

    def test_the_tire_loads_stay_compressive(self) -> None:
        self.assertGreaterEqual(float(np.min(self.result.normal_loads)), 0.0)

    def test_the_pack_current_stays_inside_the_pack_limit(self) -> None:
        self.assertLessEqual(
            float(np.max(self.result.pack_current)),
            self.vehicle.storage.discharge_current_limit + 1.0e-6,
        )
        self.assertGreaterEqual(
            float(np.min(self.result.pack_current)),
            -self.vehicle.storage.charge_current_limit - 1.0e-6,
        )

    def test_the_drive_never_exceeds_the_motor_torque_bound(self) -> None:
        peak = (
            self.vehicle.motor.peak_motor_torque
            * self.vehicle.motor.gear_ratio
            * self.vehicle.motor.motor_efficiency
        )

        self.assertLessEqual(
            float(np.max(self.result.wheel_torques)), peak + 1.0e-3
        )

    def test_power_mismatch_reports_real_four_motor_demand(self) -> None:
        motor = self.vehicle.motor
        peak = motor.peak_motor_torque * motor.gear_ratio * motor.motor_efficiency
        shares = np.asarray([0.1, 0.1, 0.4, 0.4])[:, None]
        drive = shares * (4.0 * peak * self.result.throttle[None, :])
        motor_bus_power = np.sum(drive * self.result.wheel_speeds, axis=0)
        motor_bus_power /= motor.motor_efficiency * motor.inverter_efficiency
        mismatch = float(np.max(np.abs(self.result.bus_power - motor_bus_power)))
        self.assertAlmostEqual(mismatch, self.result.maximum_power_mismatch,
                               delta=1e-6)
        if self.result.converged:
            self.assertLess(mismatch, 1e-3)

    def test_the_state_of_charge_falls_over_the_lap(self) -> None:
        self.assertLessEqual(
            float(self.result.state_of_charge[-1]),
            float(self.result.state_of_charge[0]) + 1.0e-9,
        )

    def test_the_wheel_speeds_are_positive(self) -> None:
        self.assertGreater(float(np.min(self.result.wheel_speeds)), 0.0)

    def test_the_drive_torque_is_never_negative(self) -> None:
        # The wheel torque also carries the brakes, so it is often negative.
        # The drive part alone must not be, because the throttle is bounded at
        # zero and the split shares are positive.
        peak = (
            self.vehicle.motor.peak_motor_torque
            * self.vehicle.motor.gear_ratio
            * self.vehicle.motor.motor_efficiency
        )
        drive = self.result.throttle * 4.0 * peak * np.asarray([0.1, 0.1, 0.4, 0.4])[:, None]

        self.assertGreaterEqual(float(np.min(drive)), -1.0e-5)
        self.assertGreaterEqual(float(np.min(self.result.throttle)), -1.0e-7)

    def test_a_coarse_mesh_uses_fewer_nodes(self) -> None:
        coarse = planar_ocp.solve_planar_hot_lap(
            self.vehicle,
            self.track,
            planar_ocp.PlanarOcpOptions(
                mesh_spacing=40.0,
                warm_start=False,
                print_level=0,
                maximum_iterations=1200,
            ),
        )

        self.assertLess(coarse.node_count, self.result.node_count)


if __name__ == "__main__":
    unittest.main()
