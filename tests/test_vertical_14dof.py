"""Focused contract tests for vertical mechanics primitives."""
import unittest
import numpy as np
from pathlib import Path
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState, evaluate_vertical_dynamics, sprung_wheel_positions, unsprung_masses,
)
from simplified_vehicle_models.formulations.planar_7dof import wheel_positions
from simplified_vehicle_models.tires.unitire_transient import TransientTireState, steady_rolling_state

ROOT = Path(__file__).resolve().parents[1]

#: Vertical position coordinates of ``VerticalState`` in ``as_vector`` order.
_VERTICAL_POSITION_INDEX = [6, 7, 8, 13, 14, 15, 16]


def solve_vertical_steady_state(vehicle, state, tires, iterations=40):
    """Return the vertical coordinates that make the evaluator's rates vanish.

    Newton solve on ``generalized_forces``, which is the residual of the seven
    vertical equations (the mass block is invertible, so zero generalized force
    is zero acceleration). The static equilibrium solver has no longitudinal
    tire force, so at speed drag leaves a residual pitch and jounce moment;
    this gives the true steady state of the evaluated equations instead. As it
    zeroes the evaluator's own residual, this route is a consistency check of the
    equations with themselves; ``relax_vertical`` is the independent route.
    """
    q = state.as_vector()

    def residual(x):
        trial = q.copy()
        trial[_VERTICAL_POSITION_INDEX] = x
        return evaluate_vertical_dynamics(vehicle, VerticalState.from_vector(trial),
                                          np.zeros(4), 0.0, tires).generalized_forces

    x = q[_VERTICAL_POSITION_INDEX].copy()
    for _ in range(iterations):
        r = residual(x)
        if np.linalg.norm(r, ord=np.inf) < 1e-11:
            break
        jac = np.empty((7, 7))
        for k in range(7):
            h = 1e-7
            xp, xm = x.copy(), x.copy()
            xp[k] += h
            xm[k] -= h
            jac[:, k] = (residual(xp) - residual(xm)) / (2.0 * h)
        x = x - np.linalg.solve(jac, r)
    trial = q.copy()
    trial[_VERTICAL_POSITION_INDEX] = x
    return VerticalState.from_vector(trial)


def anti_forces(vehicle, bfx):
    """Re-implement the documented anti-dive/anti-squat rule for assembly checks.

    ``k_axle = anti*(h_cg/L)/share`` with the brake bias under braking and the
    driven-axle share under drive; the body-side force is ``-k*bfx`` at the front
    and ``+k*bfx`` at the rear. The dedicated sign, scale and invariance tests in
    ``LongitudinalLoadTransferTests`` check this rule against physics.
    """
    from simplified_vehicle_models.formulations.vertical_14dof import axle_drive_shares
    p = vehicle.planar
    bias = float(p.brake_bias)
    drive_front, drive_rear = axle_drive_shares(vehicle)
    result = np.zeros(4)
    for axle, ix, sign, brake_share, drive_share in (
        (vehicle.suspension.front, (0, 1), -1.0, bias, drive_front),
        (vehicle.suspension.rear, (2, 3), 1.0, 1.0 - bias, drive_rear),
    ):
        for i in ix:
            driving = bfx[i] >= 0.0
            share = drive_share if driving else brake_share
            if share <= 0.0:
                continue
            percentage = axle.anti_drive if driving else axle.anti_brake
            result[i] = sign * percentage * (float(p.cg_height) / float(p.wheelbase)) * bfx[i] / share
    return result


def longitudinal_jacking_pitch(vehicle, ax):
    """Return the hub-only unsprung longitudinal transfer (front -, rear +)."""
    total_unsprung = float(np.sum(unsprung_masses(vehicle)))
    scale = 0.5 * total_unsprung * ax * vehicle.unsprung_cg_height / float(vehicle.planar.wheelbase)
    return np.array([-1.0, -1.0, 1.0, 1.0]) * scale


def relax_vertical(vehicle, q, tires, steps=1500, dt=2.0e-3):
    """Integrate the 14 vertical states from ``q`` to a fixed point with RK4.

    An independent route to the steady state: no Jacobian and no generalized
    force solve, just the raw rates.
    """
    from simplified_vehicle_models.formulations.vertical_14dof import VERTICAL_ACCELERATION_INDEX
    vel, pos = list(VERTICAL_ACCELERATION_INDEX), _VERTICAL_POSITION_INDEX

    def derivative(y):
        trial = q.copy()
        trial[vel], trial[pos] = y[:7], y[7:]
        out = evaluate_vertical_dynamics(vehicle, VerticalState.from_vector(trial),
                                         np.zeros(4), 0.0, tires)
        return np.r_[out.rates[vel], out.rates[pos]]

    y = np.r_[q[vel], q[pos]]
    for _ in range(steps):
        k1 = derivative(y)
        k2 = derivative(y + 0.5 * dt * k1)
        k3 = derivative(y + 0.5 * dt * k2)
        k4 = derivative(y + dt * k3)
        y = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    relaxed = q.copy()
    relaxed[vel], relaxed[pos] = y[:7], y[7:]
    return VerticalState.from_vector(relaxed)


def roll_stiffness_front_share(vehicle, static_state):
    """Return the front share of the axle roll stiffness, tire compliance included.

    Each axle's roll stiffness is ``0.5*track^2`` times the wheel rate (spring
    plus anti-roll bar) in series with the tire vertical stiffness, both at the
    static design position. ``wheel_rate_spring`` is the preloaded spring FORCE,
    so its derivative is the rate.
    """
    from simplified_vehicle_models.formulations.vertical_14dof import sprung_wheel_positions
    from simplified_vehicle_models.tires.vertical import tire_deflection_load
    p = vehicle.planar
    positions = sprung_wheel_positions(vehicle)
    q = static_state.as_vector()
    deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] - q[13:17]
    tire_rate = np.asarray(tire_deflection_load(p.tire, deflection, np.zeros(4), 0.0).stiffness)
    track = 2.0 * p.half_track
    stiffness = []
    for axle, jounce, index in ((vehicle.suspension.front, 0.0, 0),
                                (vehicle.suspension.rear, 0.0, 2)):
        rate = float(axle.wheel_rate_spring.derivative(jounce)) + float(axle.arb_wheel_rate)
        series = rate * float(tire_rate[index]) / (rate + float(tire_rate[index]))
        stiffness.append(0.5 * track**2 * series)
    return stiffness[0] / sum(stiffness)


def braking_state(vehicle, kappa, speed=30.0, loads=None, anti=True):
    """Return ``(state, tires, out)`` for a steady braking or driving slip ratio.

    The wheel speeds follow ``(1 + kappa)`` so the commanded slip ratio matches
    the steady tire states, and the vertical coordinates are solved to a steady
    state after each load update. ``anti=False`` zeroes the anti-dive/squat
    fractions so the load transfer can be checked without that force path.
    """
    from dataclasses import replace
    from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
    from simplified_vehicle_models.tires.unitire_simple import loaded_radius
    if not anti:
        s = vehicle.suspension

        def zeroed(axle):
            return replace(axle, anti_brake=0.0, anti_drive=0.0)
        vehicle = replace(vehicle, suspension=replace(s, front=zeroed(s.front), rear=zeroed(s.rear)))
    p = vehicle.planar
    q = solve_static_equilibrium(vehicle).as_vector()
    q[0] = speed
    if loads is None:
        zero = (TransientTireState(0.0, 0.0, 0.0),) * 4
        loads = evaluate_vertical_dynamics(vehicle, VerticalState.from_vector(q), np.zeros(4),
                                           0.0, zero).normal_loads
    for _ in range(5):
        q[9:13] = (1.0 + kappa) * speed / loaded_radius(p.tire, loads)
        tires = tuple(steady_rolling_state(p.tire, vehicle.transient_tire, 0.0, kappa, float(n))
                      for n in loads)
        state = solve_vertical_steady_state(vehicle, VerticalState.from_vector(q), tires)
        out = evaluate_vertical_dynamics(vehicle, state, np.zeros(4), 0.0, tires)
        loads = out.normal_loads
        q = state.as_vector()
    return vehicle, state, tires, out



class VerticalStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml").model

    def test_evaluator_smoke_has_documented_shapes(self):
        state = VerticalState()
        tire = TransientTireState(0.0, 0.0, 0.0)
        out = evaluate_vertical_dynamics(self.vehicle, state, np.zeros(4), 0.0, (tire,) * 4)
        self.assertEqual(out.rates.shape, (21,))
        self.assertEqual(out.tire_state_rates.shape, (12,))
        self.assertEqual(out.normal_loads.shape, (4,))
        self.assertEqual(out.tire_forces.shape, (4, 3))
    def test_positive_front_toe_rotates_lateral_force_to_positive_body_longitudinal(self):
        """A perturbed FL toe must rotate its tire force using the same angle.

        The toe-curve vehicle and the baseline vehicle are evaluated at one
        shared state, steering input and tire-state set, so the FL jounce-steer
        curve is the only input that differs. The FL road-wheel angle is then
        derived by hand from the documented contact-velocity convention, which
        gives the expected FL slip angle, and the evaluator's reported FL output
        must follow it: the relaxation rates are driven by that slip angle and
        the FL tire force is re-rotated into the body axes by that angle. FR, RL
        and RR must be bitwise unchanged, which is what pins the perturbation to
        FL: an index swap, a dropped toe term or a toe applied to every wheel
        breaks these equalities.
        """
        from dataclasses import replace
        from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

        suspension = self.vehicle.suspension
        front = suspension.front
        # Synthetic, isolated FL toe-in at the current 20 mm jounce; FR and
        # both rear wheels retain zero incremental toe.
        low, high = front.jounce_limits
        toe = 0.5*np.pi/180.0
        steer_curve = MonotoneSpline(np.array([low, 0.0, 0.02, high]),
                                     np.array([0.0, 0.0, toe, toe]))
        front = replace(front, jounce_steer=steer_curve)
        vehicle = replace(self.vehicle, suspension=replace(suspension, front=front))
        # Jounce is positive in compression (hub up relative to body), so the hub
        # is at heave - jounce below the body point; add 0.03 m heave to keep FL loaded.
        state = VerticalState(v_x=8.0, v_y=0.2, heave=0.03, wheel_speed_fl=8.0 / 0.32,
                              jounce_fl=0.02)
        from simplified_vehicle_models.tires.unitire_transient import (
            relaxation_lengths, steady_rolling_state)
        tire = steady_rolling_state(vehicle.planar.tire, vehicle.transient_tire,
                                    -0.5*np.pi/180.0, 0.0, 1000.0)
        # Distinct nonzero torques so the wheel-spin accelerations are not
        # trivially zero on either vehicle.
        torques = np.array([25.0, -15.0, 60.0, 40.0])
        baseline = evaluate_vertical_dynamics(self.vehicle, state, torques, 0.0, (tire,) * 4)
        out = evaluate_vertical_dynamics(vehicle, state, torques, 0.0, (tire,) * 4)

        # The FL tire carries load and a nonzero lateral force, so the toe
        # rotation matters; the body-axis total below is asserted.
        fy_tire = baseline.tire_forces[0, 1]
        self.assertGreater(baseline.normal_loads[0], 0.0)
        self.assertNotEqual(fy_tire, 0.0)

        # Road-wheel angles straight from the suspension model inputs, by the
        # documented static-toe-plus-jounce-steer rule (left positive).
        jounce = state.as_vector()[13:17]

        def wheel_angles(model):
            angles = np.empty(4)
            for index, axle in enumerate((model.front, model.front,
                                          model.rear, model.rear)):
                sign = 1.0 if index in (0, 2) else -1.0
                angles[index] = sign*(axle.static_toe + float(axle.jounce_steer(jounce[index])))
            return angles

        angles_base = wheel_angles(self.vehicle.suspension)
        angles_toe = wheel_angles(vehicle.suspension)
        self.assertAlmostEqual(float(angles_toe[0] - angles_base[0]), toe, delta=1e-12)
        np.testing.assert_array_equal(angles_toe[1:], angles_base[1:])
        self.assertGreater(abs(float(angles_toe[0])), 1e-3)

        # FL contact kinematics from the planar_7dof convention: the contact
        # velocity of a rigid body at (x, y) is (v_x - r*y, v_y + r*x), rotated
        # into the wheel frame, and the slip angle is -atan2(v_lat, v_long).
        positions = wheel_positions(vehicle.planar)

        def fl_kinematics(angle):
            body_longitudinal = state.v_x - state.yaw_rate*positions[0, 1]
            body_lateral = state.v_y + state.yaw_rate*positions[0, 0]
            vx = body_longitudinal*np.cos(angle) + body_lateral*np.sin(angle)
            vy = -body_longitudinal*np.sin(angle) + body_lateral*np.cos(angle)
            return vx, vy, -np.arctan2(vy, np.maximum(np.abs(vx), 1.0))

        vx_base, _, alpha_base = fl_kinematics(float(angles_base[0]))
        vx_toe, _, alpha_toe = fl_kinematics(float(angles_toe[0]))
        self.assertGreater(abs(vx_base), 1.0)  # speed floor inactive: the slip is exact
        delta_alpha = alpha_toe - alpha_base
        self.assertAlmostEqual(delta_alpha, toe, delta=1e-12)

        # The commanded slip angle enters the transient relaxation law
        # d(lateral_deformation)/dt = speed*(alpha_command - alpha_effective) and
        # d(trail)/dt = speed*(alpha_effective - trail)/sigma_trail, with speed =
        # max(|v_x_contact|, 0.1) and alpha_effective = deformation/sigma_y. The
        # reported FL tire-state rates must move by exactly that much.
        load = float(baseline.normal_loads[0])
        self.assertGreater(load, 2.0)  # window fully open, no contact blending
        transient = vehicle.transient_tire
        _, sigma_y = relaxation_lengths(vehicle.planar.tire, transient, load)
        alpha_effective = tire.lateral_deformation/sigma_y
        sigma_trail = sigma_y/transient.trail_pole_ratio
        speed_base, speed_toe = max(abs(vx_base), 0.1), max(abs(vx_toe), 0.1)
        predicted_deformation_rate = (speed_toe*(alpha_toe - alpha_effective)
                                      - speed_base*(alpha_base - alpha_effective))
        predicted_trail_rate = ((speed_toe - speed_base)
                                * (alpha_effective - tire.trail_angle)/sigma_trail)
        reported = (np.asarray(out.tire_state_rates)
                    - np.asarray(baseline.tire_state_rates)).reshape(4, 3)
        self.assertGreater(abs(predicted_deformation_rate), 1e-3)
        self.assertAlmostEqual(float(reported[0, 1]), predicted_deformation_rate, delta=1e-12)
        self.assertAlmostEqual(float(reported[0, 2]), predicted_trail_rate, delta=1e-12)

        # FL tire axes -> body axes: Fx_b = Fx*cos(delta) - Fy*sin(delta) and
        # Fy_b = Fx*sin(delta) + Fy*cos(delta). Only FL's angle moved, so the
        # whole change in body acceleration is that re-rotation of FL's force.
        force_x, force_y = float(baseline.tire_forces[0, 0]), fy_tire

        def body_force(angle):
            return (force_x*np.cos(angle) - force_y*np.sin(angle),
                    force_x*np.sin(angle) + force_y*np.cos(angle))

        before, after = body_force(float(angles_base[0])), body_force(float(angles_toe[0]))
        delta_acceleration = np.asarray(out.body_accelerations) - np.asarray(
            baseline.body_accelerations)
        self.assertAlmostEqual(float(delta_acceleration[0]),
                               (after[0] - before[0])/vehicle.planar.total_mass, delta=1e-12)
        # yaw_rate is zero in this state, so the lateral rate is the body lateral
        # force over the total mass.
        self.assertAlmostEqual(float(delta_acceleration[1]),
                               (after[1] - before[1])/vehicle.planar.total_mass, delta=1e-12)

        # The other three corners and the delayed FL force response must be
        # bitwise unchanged: the transient force comes from the tire state, not
        # from the command, so the toe shows up in the rates and the body axes.
        others = np.array([1, 2, 3])
        with self.subTest("untouched corners are bitwise unchanged"):
            np.testing.assert_array_equal(out.normal_loads[others], baseline.normal_loads[others])
            np.testing.assert_array_equal(out.tire_forces[others], baseline.tire_forces[others])
            np.testing.assert_array_equal(
                out.tire_state_rates.reshape(4, 3)[others],
                baseline.tire_state_rates.reshape(4, 3)[others])
            np.testing.assert_array_equal(out.rates[9:13], baseline.rates[9:13])
            np.testing.assert_array_equal(out.lifted, baseline.lifted)
            np.testing.assert_array_equal(out.corner_forces.total[others],
                                          baseline.corner_forces.total[others])
            np.testing.assert_array_equal(out.tire_forces[0], baseline.tire_forces[0])
            self.assertEqual(out.out_of_range_count, baseline.out_of_range_count)
        self.assertGreater(float(np.max(np.abs(out.rates[9:13]))), 1.0)

        # Tire axes -> body axes, summed over all wheels with steering plus the
        # jounce-steer toe angles; this pins the body longitudinal force to the
        # total wheel angles the toe-curve vehicle defines.
        from simplified_vehicle_models.formulations.planar_7dof import (
            aerodynamic_forces, wheel_steering_angles)
        from simplified_vehicle_models.suspension.corner import corner_toe_angles
        total_angles = wheel_steering_angles(0.0, 4) + corner_toe_angles(vehicle.suspension,
                                                                        jounce)
        tire_fx, tire_fy = out.tire_forces[:, 0], out.tire_forces[:, 1]
        expected_total_fx = np.sum(tire_fx*np.cos(total_angles) - tire_fy*np.sin(total_angles))
        _, drag = aerodynamic_forces(vehicle.planar, float(np.hypot(state.v_x, state.v_y)))
        self.assertAlmostEqual(out.body_accelerations[0] * vehicle.planar.total_mass + drag,
                               expected_total_fx, delta=1e-9)

    @staticmethod
    def _mirror_state(state):
        """Reflect a state through the vehicle x-z plane (y -> -y).

        Even under reflection: v_x, v_z, pitch_rate, heave, pitch. Odd: v_y,
        roll_rate, yaw_rate, roll. Per-wheel quantities swap FL<->FR, RL<->RR.
        """
        q = state.as_vector()
        swap = [1, 0, 3, 2]
        mirrored = q.copy()
        mirrored[[1, 3, 5, 7]] *= -1.0
        mirrored[9:13] = q[9:13][swap]
        mirrored[13:17] = q[13:17][swap]
        mirrored[17:21] = q[17:21][swap]
        return VerticalState.from_vector(mirrored)

    @staticmethod
    def _mirror_tire_states(tire_states):
        """Swap sides; lateral deformation and trail angle change sign."""
        return tuple(TransientTireState(tire_states[i].longitudinal_slip,
                                        -tire_states[i].lateral_deformation,
                                        -tire_states[i].trail_angle)
                     for i in (1, 0, 3, 2))

    def test_evaluator_is_left_right_mirror_symmetric(self):
        """Reflecting y -> -y must reflect every per-wheel and body output.

        The state is deliberately asymmetric (nonzero roll, heave, pitch, body
        rates, different jounces and jounce rates at all four corners,
        distinct wheel speeds/torques, steering, and nonzero transient tire
        states) so that no output is trivially zero or symmetric.
        """
        swap = np.array([1, 0, 3, 2])
        state = VerticalState(
            v_x=15.0, v_y=0.4, v_z=0.05, roll_rate=0.03, pitch_rate=-0.02,
            yaw_rate=0.25, heave=0.0095, roll=-0.003, pitch=0.0014,
            wheel_speed_fl=15.3/0.32, wheel_speed_fr=14.6/0.32,
            wheel_speed_rl=15.1/0.32, wheel_speed_rr=14.8/0.32,
            jounce_fl=0.0012, jounce_fr=-0.0008, jounce_rl=0.0015, jounce_rr=-0.0005,
            jounce_rate_fl=0.11, jounce_rate_fr=-0.07,
            jounce_rate_rl=0.05, jounce_rate_rr=-0.13)
        torques = np.array([30.0, -10.0, 60.0, 20.0])
        steering = 0.04
        # Nonzero, unequal tire states from steady rolling at distinct slips
        # and loads, then perturbed so lateral deformation and trail angle differ
        # from the steady target (all three states are exercised on every wheel).
        slip_angles = (0.03, -0.02, 0.04, 0.01)
        slip_ratios = (0.02, -0.01, 0.03, 0.005)
        loads_guess = (800.0, 550.0, 950.0, 650.0)
        tire_states = []
        for alpha, kappa, fz in zip(slip_angles, slip_ratios, loads_guess):
            steady = steady_rolling_state(self.vehicle.planar.tire, self.vehicle.transient_tire,
                                          alpha, kappa, fz)
            tire_states.append(TransientTireState(steady.longitudinal_slip*1.2,
                                                  steady.lateral_deformation*0.8,
                                                  steady.trail_angle*1.1))
        tire_states = tuple(tire_states)

        mirror_state = self._mirror_state(state)
        mirror_tires = self._mirror_tire_states(tire_states)
        out = evaluate_vertical_dynamics(self.vehicle, state, torques, steering, tire_states)
        mir = evaluate_vertical_dynamics(self.vehicle, mirror_state, torques[swap], -steering,
                                         mirror_tires)

        # Guard against a vacuous test: the scenario must actually break
        # left/right symmetry and carry load.
        self.assertTrue(np.all(out.normal_loads > 0.0), out.normal_loads)
        self.assertFalse(np.any(out.lifted))
        self.assertGreater(abs(out.normal_loads[0] - out.normal_loads[1]), 10.0)
        self.assertGreater(abs(out.normal_loads[2] - out.normal_loads[3]), 10.0)
        self.assertTrue(np.all(np.abs(out.tire_forces[:, :2]) > 1e-3), out.tire_forces)
        self.assertTrue(np.all(np.abs(out.tire_state_rates) > 0.0), out.tire_state_rates)
        self.assertGreater(abs(out.rates[3]), 1e-6)  # roll acceleration
        self.assertGreater(abs(out.rates[5]), 1e-6)  # yaw acceleration

        tol = dict(rtol=1e-12, atol=1e-10)
        # Per-wheel loads, accelerations (wheel spin and jounce), and forces.
        with self.subTest("normal loads"):
            np.testing.assert_allclose(mir.normal_loads, out.normal_loads[swap], **tol)
        with self.subTest("lifted flags and out-of-range count"):
            np.testing.assert_array_equal(mir.lifted, out.lifted[swap])
            self.assertEqual(mir.out_of_range_count, out.out_of_range_count)
        with self.subTest("wheel spin accelerations"):
            np.testing.assert_allclose(mir.rates[9:13], out.rates[9:13][swap], **tol)
        with self.subTest("jounce rates (d jounce/dt)"):
            np.testing.assert_allclose(mir.rates[13:17], out.rates[13:17][swap], **tol)
        with self.subTest("jounce accelerations"):
            np.testing.assert_allclose(mir.rates[17:21], out.rates[17:21][swap], **tol)
        with self.subTest("tire forces and aligning moment (Fx even, Fy/Mz odd)"):
            np.testing.assert_allclose(mir.tire_forces,
                                       out.tire_forces[swap]*np.array([1.0, -1.0, -1.0]), **tol)
        with self.subTest("tire state rates (slip even, deformation/trail odd)"):
            np.testing.assert_allclose(
                mir.tire_state_rates.reshape(4, 3),
                out.tire_state_rates.reshape(4, 3)[swap]*np.array([1.0, -1.0, -1.0]), **tol)
        with self.subTest("suspension corner forces"):
            for name in ("spring", "arb", "damper", "total", "damper_velocity"):
                np.testing.assert_allclose(getattr(mir.corner_forces, name),
                                           getattr(out.corner_forces, name)[swap], err_msg=name, **tol)
        # Body parity: even v_x_dot, v_z_dot, pitch accel, heave_dot, pitch_dot;
        # odd v_y_dot, roll accel, yaw accel, roll_dot (roll rate).
        parity = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
        for k, name in enumerate(("v_x_dot", "v_y_dot", "v_z_dot", "roll_accel",
                                  "pitch_accel", "yaw_accel", "heave_dot", "roll_dot",
                                  "pitch_dot")):
            with self.subTest(f"body rate {name}"):
                self.assertAlmostEqual(mir.rates[k], parity[k]*out.rates[k], delta=1e-10 + 1e-12*abs(out.rates[k]))
        with self.subTest("body_accelerations (ax even, ay odd)"):
            np.testing.assert_allclose(mir.body_accelerations,
                                       out.body_accelerations*np.array([1.0, -1.0]), **tol)
        with self.subTest("mass matrix"):
            if out.mass_matrix is None:
                self.assertIsNone(mir.mass_matrix)
            else:
                sign = np.ones(21)
                sign[[1, 3, 5, 7]] = -1.0
                perm = np.arange(21)
                perm[9:13], perm[13:17], perm[17:21] = (np.array([9, 10, 11, 12])[swap],
                                                        np.array([13, 14, 15, 16])[swap],
                                                        np.array([17, 18, 19, 20])[swap])
                expected = (sign[:, None]*sign[None, :]) * out.mass_matrix[np.ix_(perm, perm)]
                np.testing.assert_allclose(mir.mass_matrix, expected, **tol)

    def test_vector_round_trip_is_21_values(self):
        vector = np.arange(21, dtype=float)
        self.assertTrue(np.array_equal(VerticalState.from_vector(vector).as_vector(), vector))

    def test_evaluator_at_static_equilibrium_has_zero_vertical_rates(self):
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        from simplified_vehicle_models.tires.unitire_transient import steady_rolling_state
        state = solve_static_equilibrium(self.vehicle)
        tire_states = tuple(steady_rolling_state(self.vehicle.planar.tire, self.vehicle.transient_tire,
                                                  0.0, 0.0,
                                                  float(evaluate_vertical_dynamics(self.vehicle, state, np.zeros(4), 0.0,
                                                      (TransientTireState(0,0,0),)*4).normal_loads[i])) for i in range(4))
        out = evaluate_vertical_dynamics(self.vehicle, state, np.zeros(4), 0.0, tire_states)
        self.assertTrue(np.allclose(out.rates[:9], 0.0, atol=1e-5), out.rates[:9])

    # ------------------------------------------------------------------
    # Sign convention and linear-stability oracles at static equilibrium.
    #
    # heave positive DOWN; roll positive right side DOWN (y right); pitch
    # positive nose UP (x forward); jounce positive COMPRESSION (hub moves UP
    # relative to the body). corner_forces positive pushes the body UP and the
    # hub DOWN.
    # ------------------------------------------------------------------
    @classmethod
    def _static(cls):
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        state = solve_static_equilibrium(cls.vehicle)
        base = evaluate_vertical_dynamics(cls.vehicle, state, np.zeros(4), 0.0,
                                          (TransientTireState(0.0, 0.0, 0.0),) * 4)
        tires = tuple(steady_rolling_state(cls.vehicle.planar.tire, cls.vehicle.transient_tire,
                                           0.0, 0.0, float(load)) for load in base.normal_loads)
        return state.as_vector(), tires

    def _perturbed(self, **changes):
        """Evaluate at static equilibrium plus absolute increments by field name."""
        q, tires = self._static()
        names = [f.name for f in __import__("dataclasses").fields(VerticalState)]
        for name, delta in changes.items():
            q[names.index(name)] += delta
        return evaluate_vertical_dynamics(self.vehicle, VerticalState.from_vector(q), np.zeros(4),
                                          0.0, tires)

    def test_static_equilibrium_is_stationary_in_all_vertical_coordinates(self):
        q, tires = self._static()
        out = evaluate_vertical_dynamics(self.vehicle, VerticalState.from_vector(q), np.zeros(4),
                                         0.0, tires)
        np.testing.assert_allclose(out.rates[[2, 3, 4, 17, 18, 19, 20]], 0.0, atol=1e-6)

    def _hub_fixed(self, heave=0.0, roll=0.0, pitch=0.0):
        """Body displacement with the four hubs held fixed in space.

        Jounce is positive compression (hub up relative to body), so a body point
        that moves down by z_body with a fixed hub gives jounce = +z_body.
        """
        pos = sprung_wheel_positions(self.vehicle)
        z_body = heave + roll * pos[:, 1] - pitch * pos[:, 0]
        changes = dict(heave=heave, roll=roll, pitch=pitch)
        for name, value in zip(("jounce_fl", "jounce_fr", "jounce_rl", "jounce_rr"), z_body):
            changes[name] = float(value)
        return self._perturbed(**changes)

    def test_heave_down_with_hubs_fixed_compresses_springs_and_lifts_body(self):
        base = self._perturbed()
        out = self._hub_fixed(heave=1e-3)
        np.testing.assert_allclose(out.normal_loads, base.normal_loads, rtol=0.0, atol=1e-9)
        self.assertTrue(np.all(out.corner_forces.total > base.corner_forces.total))
        self.assertLess(out.rates[2], -1e-3)  # compression pushes the body UP
        self.assertAlmostEqual(out.rates[3], 0.0, delta=1e-9)
        # Front/rear stiffness differ, so the heave push has a pitch moment; check sign
        # against the force-weighted lever arms directly.
        force = out.corner_forces.total - base.corner_forces.total
        sprung_x = sprung_wheel_positions(self.vehicle)[:, 0]
        self.assertAlmostEqual(out.rates[4] * self.vehicle.pitch_inertia,
                               float(np.sum(sprung_x * force)), delta=1e-9)

    def test_heave_down_with_body_only_loads_tires_and_hub_goes_down(self):
        base = self._perturbed()
        out = self._perturbed(heave=1e-3)
        self.assertTrue(np.all(out.normal_loads > base.normal_loads))
        self.assertTrue(np.all(out.rates[17:21] > base.rates[17:21] + 1.0))  # hub-relative jounce accel
        np.testing.assert_allclose(out.corner_forces.total, base.corner_forces.total, atol=1e-12)

    def test_heave_rate_down_with_hubs_fixed_damper_pushes_body_up(self):
        base = self._perturbed()
        out = self._perturbed(v_z=0.1, jounce_rate_fl=0.1, jounce_rate_fr=0.1,
                              jounce_rate_rl=0.1, jounce_rate_rr=0.1)
        np.testing.assert_allclose(out.normal_loads, base.normal_loads, rtol=0.0, atol=1e-9)
        self.assertTrue(np.all(out.corner_forces.damper > base.corner_forces.damper))
        self.assertLess(out.rates[2], -1e-3)

    def test_heave_rate_down_alone_increases_tire_load(self):
        base = self._perturbed()
        out = self._perturbed(v_z=0.1)
        self.assertTrue(np.all(out.normal_loads > base.normal_loads))

    def test_positive_roll_lowers_right_side_and_is_restored(self):
        base = self._perturbed()
        body_only = self._perturbed(roll=1e-3)
        delta = body_only.normal_loads - base.normal_loads
        self.assertTrue(np.all(delta[[1, 3]] > 0.0), delta)   # right tires loaded
        self.assertTrue(np.all(delta[[0, 2]] < 0.0), delta)   # left tires unloaded
        hub_fixed = self._hub_fixed(roll=1e-3)
        np.testing.assert_allclose(hub_fixed.normal_loads, base.normal_loads, atol=1e-9)
        # Right springs compress and push the right side UP: negative roll accel.
        self.assertLess(hub_fixed.rates[3], -1e-3)
        self.assertAlmostEqual(hub_fixed.rates[2], 0.0, delta=1e-9)

    def test_positive_pitch_raises_nose_and_is_restored(self):
        base = self._perturbed()
        body_only = self._perturbed(pitch=1e-3)
        delta = body_only.normal_loads - base.normal_loads
        self.assertTrue(np.all(delta[:2] < 0.0), delta)   # nose up: front unloaded
        self.assertTrue(np.all(delta[2:] > 0.0), delta)   # rear loaded
        hub_fixed = self._hub_fixed(pitch=1e-3)
        np.testing.assert_allclose(hub_fixed.normal_loads, base.normal_loads, atol=1e-9)
        # Rear springs compress (push rear up), front extend: nose-down, negative.
        self.assertLess(hub_fixed.rates[4], -1e-3)

    def test_jounce_is_compression_hub_up_relative_to_body(self):
        base = self._perturbed()
        out = self._perturbed(jounce_fl=1e-3)
        # Body fixed, hub moves up: the tire unloads while the spring compresses.
        self.assertLess(out.normal_loads[0], base.normal_loads[0])
        self.assertGreater(out.corner_forces.total[0], base.corner_forces.total[0])
        # Compression force pushes the body UP (heave accel negative), the left
        # side up (positive roll accel) and the nose up (positive pitch accel).
        self.assertLess(out.rates[2], base.rates[2])
        self.assertGreater(out.rates[3], base.rates[3])
        self.assertGreater(out.rates[4], base.rates[4])
        # Compression force pushes the hub DOWN: reduces jounce acceleration.
        self.assertLess(out.rates[17], base.rates[17])

    def test_jounce_rate_is_hub_up_rate_and_damper_pushes_body_up(self):
        base = self._perturbed()
        out = self._perturbed(jounce_rate_rr=0.1)
        self.assertLess(out.normal_loads[3], base.normal_loads[3])
        self.assertGreater(out.corner_forces.damper[3], base.corner_forces.damper[3])
        self.assertLess(out.rates[2], base.rates[2])
        self.assertLess(out.rates[3], base.rates[3])  # right side pushed up: negative roll
        self.assertLess(out.rates[4], base.rates[4])  # rear pushed up: nose down (negative pitch)

    def test_hub_acceleration_is_body_point_minus_jounce_kinematics(self):
        """A uniform body down-acceleration appears as hub-relative jounce acceleration."""
        q, tires = self._static()
        names = [f.name for f in __import__("dataclasses").fields(VerticalState)]
        q[names.index("jounce_fl")] += 1e-3
        out = evaluate_vertical_dynamics(self.vehicle, VerticalState.from_vector(q), np.zeros(4),
                                         0.0, tires)
        pos = sprung_wheel_positions(self.vehicle)
        um = self.vehicle.unsprung_mass_front
        body_down = out.rates[2] + pos[0, 1] * out.rates[3] - pos[0, 0] * out.rates[4]
        # hub down-acceleration = g + (spring_force - tire_load)/m (anti-dive is zero here)
        hub_down_acc = 9.80665 + (out.corner_forces.total[0] - out.normal_loads[0]) / um
        self.assertAlmostEqual(out.rates[17], body_down - hub_down_acc, delta=1e-9)

    def _linearisation(self):
        """Central-difference Jacobian of the 14 vertical states.

        Order: [v_z, roll_rate, pitch_rate, jounce_rate x4 | heave, roll, pitch,
        jounce x4]; rows are the time derivatives in the same order.
        """
        q0, tires = self._static()
        index = [2, 3, 4, 17, 18, 19, 20, 6, 7, 8, 13, 14, 15, 16]

        def f(x):
            q = q0.copy()
            q[index] = x
            r = evaluate_vertical_dynamics(self.vehicle, VerticalState.from_vector(q), np.zeros(4),
                                           0.0, tires).rates
            return np.r_[r[2:5], r[17:21], r[6:9], r[13:17]]

        x0 = q0[index]
        jac = np.empty((14, 14))
        for k in range(14):
            h = 1e-6
            xp, xm = x0.copy(), x0.copy()
            xp[k] += h
            xm[k] -= h
            jac[:, k] = (f(xp) - f(xm)) / (2.0 * h)
        return jac

    def test_static_equilibrium_linearisation_is_asymptotically_stable(self):
        """Dynamic FD oracle over heave/roll/pitch/jounce rates and positions."""
        jac = self._linearisation()
        eigenvalues = np.linalg.eigvals(jac)
        self.assertLess(float(np.max(eigenvalues.real)), -1.0, eigenvalues)
        # Stiffness (position -> acceleration) block: no negative stiffness.
        stiffness = -jac[:7, 7:]
        stiffness_eigenvalues = np.linalg.eigvals(stiffness)
        self.assertLess(float(np.max(np.abs(stiffness_eigenvalues.imag))), 1e-6)
        self.assertGreater(float(np.min(stiffness_eigenvalues.real)), 0.0, stiffness_eigenvalues)
        # Diagonal stiffness and damping of the sprung coordinates are restoring.
        # Each hub: d(jounce accel)/d(own jounce) and d(own jounce rate) restoring.
        for k in range(4):
            self.assertLess(jac[3 + k, 10 + k], 0.0)
            self.assertLess(jac[3 + k, 3 + k], 0.0)
        # Body acceleration reacts to the jounce rates through the dampers: a hub
        # moving up (compression) pushes the body up (heave accel decreases), the
        # right side up (roll accel decreases for right corners), and the nose
        # up for front corners.
        self.assertTrue(np.all(jac[0, 3:7] < 0.0))
        self.assertTrue(np.all(jac[1, [4, 6]] < 0.0) and np.all(jac[1, [3, 5]] > 0.0))
        self.assertTrue(np.all(jac[2, [3, 4]] > 0.0) and np.all(jac[2, [5, 6]] < 0.0))
        # Kinematic block: d(position rate)/d(rate) is the identity.
        np.testing.assert_allclose(jac[7:, :7], np.eye(7), atol=1e-8)
        np.testing.assert_allclose(jac[7:, 7:], 0.0, atol=1e-8)

    def test_rejects_wrong_shape(self):
        with self.assertRaises(ValueError):
            VerticalState.from_vector(np.zeros(20))

    def test_rejects_non_finite(self):
        vector = np.zeros(21); vector[2] = np.nan
        with self.assertRaises(ValueError):
            VerticalState.from_vector(vector)

class VerticalMassMatrixTests(unittest.TestCase):
    """Task 6 mass block: physical M0, grounded-to-chassis-point inerter, RateSystem."""

    G = 9.80665

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml").model

    @staticmethod
    def _with_inertance(vehicle, front, rear):
        from dataclasses import replace
        s = vehicle.suspension
        return replace(vehicle, suspension=replace(
            s, front=replace(s.front, inertance=front), rear=replace(s.rear, inertance=rear)))

    def _static_tires(self, vehicle, speed=0.0):
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        state = solve_static_equilibrium(vehicle, speed=speed)
        zero = (TransientTireState(0.0, 0.0, 0.0),) * 4
        base = evaluate_vertical_dynamics(vehicle, state, np.zeros(4), 0.0, zero)
        tires = tuple(steady_rolling_state(vehicle.planar.tire, vehicle.transient_tire,
                                           0.0, 0.0, float(n)) for n in base.normal_loads)
        return state, tires

    def _perturbed_state(self, vehicle):
        """Straight-line state with nonzero accelerations in all seven coordinates."""
        state, tires = self._static_tires(vehicle, speed=10.0)
        q = state.as_vector()
        q[6] += 1.5e-3      # heave
        q[7] += 4.0e-4      # roll
        q[8] -= 6.0e-4      # pitch
        q[13:17] += np.array([1.0e-3, -0.6e-3, 0.8e-3, -0.3e-3])
        q[17:21] = np.array([0.05, -0.03, 0.04, 0.02])
        q[2], q[3], q[4] = 0.02, 0.01, -0.01
        return VerticalState.from_vector(q), tires

    def test_physical_block_is_spd_with_expected_structure(self):
        from simplified_vehicle_models.formulations.vertical_14dof import (
            vertical_mass_block, unsprung_masses)
        v = self.vehicle
        pos = sprung_wheel_positions(v)
        um = unsprung_masses(v)
        expected = np.zeros((7, 7))
        expected[0, 0], expected[1, 1], expected[2, 2] = v.sprung_mass, v.roll_inertia, v.pitch_inertia
        for i in range(4):
            c = np.array([1.0, pos[i, 1], -pos[i, 0], 0, 0, 0, 0])
            c[3 + i] = -1.0
            expected += um[i] * np.outer(c, c)
        block = vertical_mass_block(v, np.zeros(4))
        np.testing.assert_allclose(block, expected, rtol=1e-13, atol=1e-12)
        np.testing.assert_allclose(block, block.T, atol=1e-12)
        # Hub-jounce diagonal is exactly m_u with no inertance; body corner term is c c^T.
        np.testing.assert_allclose(np.diag(block)[3:], um, rtol=1e-13)
        self.assertGreater(float(np.linalg.eigvalsh(block).min()), 0.0)

    def test_positive_inertance_is_spd_and_adds_b_over_mr2_only_on_jounce_diagonals(self):
        from simplified_vehicle_models.formulations.vertical_14dof import vertical_mass_block
        b_f, b_r = 2.5, 4.0
        v0 = self._with_inertance(self.vehicle, 0.0, 0.0)
        vb = self._with_inertance(self.vehicle, b_f, b_r)
        z = np.array([0.004, -0.002, 0.006, -0.001])
        m0, mb = vertical_mass_block(v0, z), vertical_mass_block(vb, z)
        delta = mb - m0
        s = vb.suspension
        mr = [float(s.front.motion_ratio_damper(z[0])), float(s.front.motion_ratio_damper(z[1])),
              float(s.rear.motion_ratio_damper(z[2])), float(s.rear.motion_ratio_damper(z[3]))]
        expected = np.zeros((7, 7))
        for i, (b, ratio) in enumerate(zip((b_f, b_f, b_r, b_r), mr)):
            expected[3 + i, 3 + i] = b / ratio**2
        np.testing.assert_allclose(delta, expected, rtol=1e-12, atol=1e-12)
        # Nothing couples to the body and nothing is added to the body diagonals.
        self.assertEqual(float(np.abs(delta[:3, :]).max()), 0.0)
        self.assertGreater(float(np.linalg.eigvalsh(mb).min()), float(np.linalg.eigvalsh(m0).min()))
        np.testing.assert_allclose(mb, mb.T, atol=1e-12)

    def test_zero_inertance_baseline_has_no_matrix_and_forcing_equals_rates(self):
        v = self._with_inertance(self.vehicle, 0.0, 0.0)
        state, tires = self._perturbed_state(v)
        out = evaluate_vertical_dynamics(v, state, np.array([40.0, 40.0, 80.0, 80.0]), 0.0, tires)
        self.assertIsNone(out.mass_matrix)
        np.testing.assert_array_equal(out.forcing, out.rates)
        from simplified_vehicle_models.solvers.forward_driver import RateSystem
        np.testing.assert_allclose(RateSystem(out.forcing, out.mass_matrix).solve(), out.rates, rtol=0, atol=0)

    def test_m0_times_acceleration_equals_independent_newton_euler_generalized_force(self):
        for slip in (0.03, -0.03):  # drive and brake anti fractions
            with self.subTest(slip_ratio=slip):
                self._check_m0_a_equals_q(slip)

    def _check_m0_a_equals_q(self, slip):
        """Oracle: separate body/hub Newton-Euler equations imply M0 a = Q.

        The test writes the sprung-body equations and four hub equations
        directly from the reported forces (straight line, so lateral forces and
        RC jacking vanish), builds Q from virtual work, and compares with
        ``M0 @ a`` from the evaluator's solved rates.
        """
        from simplified_vehicle_models.formulations.vertical_14dof import (
            vertical_mass_block, unsprung_masses)
        from simplified_vehicle_models.formulations.planar_7dof import aerodynamic_forces
        v = self._with_inertance(self.vehicle, 0.0, 0.0)
        state, tires = self._perturbed_state(v)
        tires = tuple(steady_rolling_state(v.planar.tire, v.transient_tire, 0.0,
                                           slip * (1.0 + 0.2 * i), 800.0) for i in range(4))
        out = evaluate_vertical_dynamics(v, state, np.array([40.0, 40.0, 80.0, 80.0]), 0.0, tires)
        self.assertLess(float(np.abs(out.tire_forces[:, 1]).max()), 1e-6)  # no lateral force
        self.assertGreater(float(np.abs(out.tire_forces[:, 0]).min()), 1.0)  # anti force is exercised
        pos = sprung_wheel_positions(v)
        um = unsprung_masses(v)
        a = out.rates[[2, 3, 4, 17, 18, 19, 20]]
        # Anti-dive/squat and the longitudinal force path (see the Task 6 roll
        # report ruling); this is an assembly check, not a scale check.
        bfx = out.tire_forces[:, 0]
        ax = float(out.body_accelerations[0])
        anti = anti_forces(v, bfx)
        jacking_pitch = longitudinal_jacking_pitch(v, ax)
        link_longitudinal = bfx - um * ax
        upward_on_body = out.corner_forces.total + anti
        down_force, _ = aerodynamic_forces(v.planar, float(np.hypot(state.v_x, state.v_y)))
        balance = v.planar.aero.aero_balance
        arm = (balance * 0.5 * (pos[0, 0] + pos[1, 0]) + (1 - balance) * 0.5 * (pos[2, 0] + pos[3, 0]))
        # Sprung body.
        heave_eq = v.sprung_mass * self.G + down_force - upward_on_body.sum()
        roll_eq = -np.sum(pos[:, 1] * upward_on_body)
        pitch_eq = (np.sum(pos[:, 0] * upward_on_body) - down_force * arm
                    + v.sprung_cg_height * np.sum(link_longitudinal))
        # Hubs: down-acceleration is the body-point acceleration minus jounce acceleration.
        body_down = a[0] + pos[:, 1] * a[1] - pos[:, 0] * a[2]
        hub_down = body_down - a[3:]
        hub_force = um * self.G + upward_on_body + jacking_pitch - out.normal_loads
        np.testing.assert_allclose(um * hub_down, hub_force, rtol=1e-9, atol=1e-7)
        np.testing.assert_allclose(v.sprung_mass * a[0], heave_eq, rtol=1e-9, atol=1e-7)
        np.testing.assert_allclose(v.roll_inertia * a[1], roll_eq, rtol=1e-9, atol=1e-7)
        np.testing.assert_allclose(v.pitch_inertia * a[2], pitch_eq, rtol=1e-9, atol=1e-7)
        # Self-consistency/regression check, not independent validation: q is
        # assembled with the same body/hub decomposition the evaluator uses, so
        # these two equalities only confirm the mass-block and virtual-work
        # bookkeeping. The independent content of this test is the body and hub
        # equations above, written from reported forces.
        q = np.zeros(7)
        q[0] = heave_eq + hub_force.sum()
        q[1] = roll_eq + np.sum(pos[:, 1] * hub_force)
        q[2] = pitch_eq - np.sum(pos[:, 0] * hub_force)
        q[3:] = -hub_force
        np.testing.assert_allclose(vertical_mass_block(v, state.as_vector()[13:17]) @ a, q, rtol=1e-9, atol=1e-6)
        np.testing.assert_allclose(out.generalized_forces, q, rtol=1e-9, atol=1e-6)
        # The motion must be nontrivial in every coordinate.
        self.assertTrue(np.all(np.abs(a) > 1e-3), a)

    def test_rate_system_recovers_rates_with_inertance(self):
        from simplified_vehicle_models.solvers.forward_driver import RateSystem
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        state, tires = self._perturbed_state(v)
        torques = np.array([40.0, 40.0, 80.0, 80.0])
        out = evaluate_vertical_dynamics(v, state, torques, 0.0, tires)
        self.assertEqual(out.mass_matrix.shape, (21, 21))
        self.assertEqual(out.forcing.shape, (21,))
        np.testing.assert_allclose(RateSystem(out.forcing, out.mass_matrix).solve(), out.rates,
                                   rtol=1e-10, atol=1e-9)
        # Rows outside the vertical block are unit-mass and untouched.
        others = [k for k in range(21) if k not in (2, 3, 4, 17, 18, 19, 20)]
        np.testing.assert_array_equal(out.mass_matrix[np.ix_(others, others)], np.eye(len(others)))
        np.testing.assert_array_equal(out.forcing[others], out.rates[others])
        # Positive inertance slows the jounce response relative to b = 0.
        base = evaluate_vertical_dynamics(self._with_inertance(self.vehicle, 0.0, 0.0), state, torques, 0.0, tires)
        self.assertFalse(np.allclose(base.rates[17:21], out.rates[17:21]))
        # b > 0 solve vs independent physical block solve with the generalized force.
        from simplified_vehicle_models.formulations.vertical_14dof import vertical_mass_block
        block = vertical_mass_block(v, state.as_vector()[13:17])
        np.testing.assert_allclose(block @ out.rates[[2, 3, 4, 17, 18, 19, 20]], out.generalized_forces,
                                   rtol=1e-9, atol=1e-6)

    def test_b_positive_leaves_static_equilibrium_stationary(self):
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        state, tires = self._static_tires(v)
        out = evaluate_vertical_dynamics(v, state, np.zeros(4), 0.0, tires)
        self.assertIsNotNone(out.mass_matrix)
        np.testing.assert_allclose(out.rates[[2, 3, 4, 17, 18, 19, 20]], 0.0, atol=1e-6)
        np.testing.assert_allclose(out.generalized_forces, 0.0, atol=1e-6)

    def test_b_positive_speed_equilibrium_is_stationary_in_all_seven_coordinates(self):
        """The evaluator's own steady state is stationary in all seven coordinates.

        The static equilibrium solver has no longitudinal tire force, so at speed
        drag leaves a residual pitch and jounce moment; the state below is refined
        on the evaluator's generalized forces instead. Wheel speeds are aligned
        with the loaded radii (zero slip, zero tire force), so only aero downforce
        and drag act.
        """
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        from simplified_vehicle_models.tires.unitire_simple import loaded_radius
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        speed = 12.0
        coarse = solve_static_equilibrium(v, speed=speed)
        zero = (TransientTireState(0.0, 0.0, 0.0),) * 4
        loads = evaluate_vertical_dynamics(v, coarse, np.zeros(4), 0.0, zero).normal_loads
        q = coarse.as_vector()
        q[9:13] = speed / loaded_radius(v.planar.tire, loads)
        tires = tuple(steady_rolling_state(v.planar.tire, v.transient_tire, 0.0, 0.0, float(n))
                      for n in loads)
        state = solve_vertical_steady_state(v, VerticalState.from_vector(q), tires)
        out = evaluate_vertical_dynamics(v, state, np.zeros(4), 0.0, tires)
        self.assertLess(float(np.abs(out.tire_forces[:, :2]).max()), 1e-6)
        np.testing.assert_allclose(out.generalized_forces, 0.0, atol=1e-6)
        np.testing.assert_allclose(out.rates[[2, 3, 4, 17, 18, 19, 20]], 0.0, atol=1e-6)
        # The refinement is a real pitch change from the drag transfer, not noise.
        self.assertGreater(abs(state.pitch - coarse.pitch), 1e-6)

    def test_mass_block_is_left_right_mirror_symmetric(self):
        from simplified_vehicle_models.formulations.vertical_14dof import vertical_mass_block
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        z = np.array([0.004, -0.002, 0.006, -0.001])
        m = vertical_mass_block(v, z)
        mirror = vertical_mass_block(v, z[[1, 0, 3, 2]])
        sign = np.array([1, -1, 1, 1, 1, 1, 1.0])
        perm = np.array([0, 1, 2, 4, 3, 6, 5])
        np.testing.assert_allclose(mirror, (sign[:, None] * sign[None, :]) * m[np.ix_(perm, perm)],
                                   rtol=1e-12, atol=1e-10)

    def test_mirrored_states_give_mirrored_matrix_and_forcing_with_inertance(self):
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        state, tires = self._perturbed_state(v)
        swap = np.array([1, 0, 3, 2])
        mirror_state = VerticalStateTests._mirror_state(state)
        mirror_tires = VerticalStateTests._mirror_tire_states(tires)
        torques = np.array([40.0, 30.0, 80.0, 60.0])
        out = evaluate_vertical_dynamics(v, state, torques, 0.02, tires)
        mir = evaluate_vertical_dynamics(v, mirror_state, torques[swap], -0.02, mirror_tires)
        sign = np.ones(21)
        sign[[1, 3, 5, 7]] = -1.0
        perm = np.arange(21)
        perm[9:13], perm[13:17], perm[17:21] = (np.array([9, 10, 11, 12])[swap],
                                                np.array([13, 14, 15, 16])[swap],
                                                np.array([17, 18, 19, 20])[swap])
        np.testing.assert_allclose(mir.mass_matrix,
                                   (sign[:, None] * sign[None, :]) * out.mass_matrix[np.ix_(perm, perm)],
                                   rtol=1e-12, atol=1e-10)
        np.testing.assert_allclose(mir.rates[[2, 4, 17, 18, 19, 20]],
                                   np.r_[out.rates[2], out.rates[4], out.rates[17:21][swap]],
                                   rtol=1e-10, atol=1e-9)
        np.testing.assert_allclose(mir.rates[3], -out.rates[3], rtol=1e-10, atol=1e-9)

    def test_variable_motion_ratio_adds_inertance_velocity_term(self):
        """Lagrange term -0.5 b'(j) jdot^2 appears in forcing; constant MR gives none."""
        from dataclasses import replace
        from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
        v = self._with_inertance(self.vehicle, 3.0, 5.0)
        front = v.suspension.front
        low, high = front.jounce_limits
        knots = np.array([low, 0.0, high])
        # Strictly varying but smooth motion ratio around the constant baseline.
        varying = MonotoneSpline(knots, 1.05 * np.array([0.94, 1.0, 1.07]))
        vv = replace(v, suspension=replace(v.suspension, front=replace(front, motion_ratio_damper=varying)))
        state, tires = self._perturbed_state(vv)
        q = state.as_vector()
        out = evaluate_vertical_dynamics(vv, state, np.zeros(4), 0.0, tires)
        for i in (0, 1):
            j, jd, b = q[13 + i], q[17 + i], front.inertance
            h = 1e-7
            b_eff = lambda x: b / float(varying(x)) ** 2
            slope = (b_eff(j + h) - b_eff(j - h)) / (2 * h)
            self.assertNotEqual(slope, 0.0)
            expected = out.generalized_forces[3 + i] - 0.5 * slope * jd**2
            self.assertAlmostEqual(out.forcing[17 + i], expected, delta=1e-5 * (1 + abs(expected)))
        # Rear MR is constant: no velocity term.
        self.assertAlmostEqual(out.forcing[19], out.generalized_forces[5], delta=1e-9)
        self.assertAlmostEqual(out.forcing[20], out.generalized_forces[6], delta=1e-9)
        from simplified_vehicle_models.solvers.forward_driver import RateSystem
        np.testing.assert_allclose(RateSystem(out.forcing, out.mass_matrix).solve(), out.rates,
                                   rtol=1e-10, atol=1e-9)


class LateralLoadTransferTests(unittest.TestCase):
    """Form B roll: the RC jacking pair D, the unsprung pair Du, and the sign.

    Every lateral scenario is built from a sideslip, so the lateral tire forces
    come from the tire model rather than being imposed.
    """

    G = 9.80665
    _relaxed = None

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml").model

    def _lateral_setup(self, speed=30.0, sideslip=0.36):
        """Return (state, tire states) for a sideslip at the static equilibrium.

        ``sideslip`` is the lateral contact velocity in m/s. Wheel speeds are
        aligned with the loaded radii so the slip ratio is zero, and the tire
        states are steady at the slip angles the sideslip produces.
        """
        from simplified_vehicle_models.formulations.planar_7dof import (
            PlanarState, contact_velocities, slip_quantities)
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        from simplified_vehicle_models.suspension.corner import corner_toe_angles
        from simplified_vehicle_models.tires.unitire_simple import loaded_radius
        from simplified_vehicle_models.tires.vertical import tire_deflection_load

        vehicle = self.vehicle
        p = vehicle.planar
        q = solve_static_equilibrium(vehicle).as_vector()
        q[0], q[1] = speed, sideslip
        pos_s = sprung_wheel_positions(vehicle)
        z_body = q[6] + q[7] * pos_s[:, 1] - q[8] * pos_s[:, 0]
        deflection = z_body - q[13:17]
        loads = tire_deflection_load(p.tire, deflection, np.zeros(4), 0.0).load
        q[9:13] = speed / loaded_radius(p.tire, loads)
        angles = corner_toe_angles(vehicle.suspension, q[13:17])
        vx, vy = contact_velocities(PlanarState(q[0], q[1], 0.0, q[9:13]), wheel_positions(p), angles)
        _, sa = slip_quantities(q[9:13], loaded_radius(p.tire, loads), vx, vy, 1.0)
        tires = tuple(steady_rolling_state(p.tire, vehicle.transient_tire, float(alpha), 0.0,
                                           float(load)) for alpha, load in zip(sa, loads))
        return VerticalState.from_vector(q), tires

    def _rightward(self, speed=30.0, sideslip=0.36):
        """Return the sideslip sign that gives rightward (+ay) lateral force."""
        for sign in (-1.0, 1.0):
            state, tires = self._lateral_setup(speed, sign * sideslip)
            out = evaluate_vertical_dynamics(self.vehicle, state, np.zeros(4), 0.0, tires)
            if float(out.body_accelerations[1]) > 2.0:
                return state, tires, out
        raise AssertionError("no sideslip sign produced rightward lateral force")

    def _body_forces(self, out, steering=0.0, jounce=None):
        """Rebuild body-frame tire forces and the Form B vertical force arrays."""
        from simplified_vehicle_models.formulations.planar_7dof import (
            rotate_tire_forces_to_body, wheel_steering_angles)
        from simplified_vehicle_models.suspension.corner import corner_toe_angles
        vehicle = self.vehicle
        j = jounce if jounce is not None else np.zeros(4)
        total = wheel_steering_angles(steering, 4) + corner_toe_angles(vehicle.suspension, j)
        bfx, bfy = rotate_tire_forces_to_body(out.tire_forces[:, 0], out.tire_forces[:, 1], total)
        return bfx, bfy

    def _jacking(self, out, ay, jounce=None, steering=0.0):
        """Recompute Y, D and Du from reported outputs (independent of the source)."""
        from simplified_vehicle_models.formulations.vertical_14dof import unsprung_masses
        vehicle = self.vehicle
        p = vehicle.planar
        bfx, bfy = self._body_forces(out, steering, jounce)
        um = unsprung_masses(vehicle)
        y_wheel = sprung_wheel_positions(vehicle)[:, 1]
        roll = 0.0
        rc = np.array([float(vehicle.suspension.front.roll_centre_height(roll))] * 2
                      + [float(vehicle.suspension.rear.roll_centre_height(roll))] * 2)
        sign_y = np.where(y_wheel > 0.0, 1.0, -1.0)
        track = 2.0 * p.half_track
        link = bfy - um * ay
        return link, -sign_y * 2.0 * rc * link / track, \
            -sign_y * 2.0 * um * ay * vehicle.unsprung_cg_height / track

    def test_rightward_lateral_force_gives_negative_roll_acceleration_at_static_state(self):
        """+ay must roll the body left side down, and the moment is the whole-vehicle one."""
        state, tires, out = self._rightward()
        vehicle = self.vehicle
        ay = float(out.body_accelerations[1])
        self.assertGreater(ay, 2.0)
        self.assertAlmostEqual(state.roll, 0.0, delta=1e-12)
        np.testing.assert_allclose(out.normal_loads[:2], out.normal_loads[[1, 0]], atol=1e-9)
        self.assertLess(out.rates[3], -1e-3, out.rates[3])
        # With the body level every corner load is symmetric, so the roll moment
        # is exactly the whole-vehicle -m_total*h_cg*ay (the pre-Form-B code gave
        # +h_rc*Sigma Fy here, the wrong sign). Self-consistency/regression
        # check, not independent validation: at the level body the RC pairs D
        # cancel and the equality follows from the documented link sum
        # Sigma Y = m_s*ay, the pair Du = -sign(y)*2*mu*ay*r_1/track and
        # m_s*h_s + m_u*r_1 = m_total*h_cg. The independent evidence for the sign
        # is the negative roll acceleration and the relaxation identities below.
        self.assertAlmostEqual(float(out.generalized_forces[1]),
                               -vehicle.planar.total_mass * vehicle.planar.cg_height * ay,
                               delta=1e-6 * vehicle.planar.total_mass * abs(ay))

    def _steady_lateral(self):
        """Return ``(relaxed state, out, tires)`` at +ay, relaxed once per class."""
        cached = LateralLoadTransferTests._relaxed
        if cached is None:
            vehicle = self.vehicle
            state, tires, _ = self._rightward()
            relaxed = relax_vertical(vehicle, state.as_vector(), tires)
            out = evaluate_vertical_dynamics(vehicle, relaxed, np.zeros(4), 0.0, tires)
            cached = (relaxed, out, tires)
            LateralLoadTransferTests._relaxed = cached
        return cached

    def test_lltd_matches_the_roll_stiffness_distribution(self):
        """The steady front share of the transfer equals the roll stiffness share.

        The classic linear reference: the lateral load transfer splits between
        the axles in proportion to their roll stiffness, each axles' wheel rate
        (spring plus bar) in series with the tire. The model's split comes from
        an independent dynamic relaxation, so the two are separate routes.

        This is a *loose* check on the total split, not on the force path: the
        RC-jacking terms move the measured share by only ~0.009, and the
        predicted reference itself sits 0.0086 from the measurement, so no
        tolerance below ~0.015 is supportable. The absolute oracle for the
        jacking path is
        ``test_axle_load_transfer_matches_the_reported_forces_and_link_geometry``.
        """
        from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
        vehicle = self.vehicle
        relaxed, out, _ = self._steady_lateral()
        loads = out.normal_loads
        front = float(loads[0] - loads[1])
        rear = float(loads[2] - loads[3])
        self.assertGreater(front, 0.0)
        self.assertGreater(rear, 0.0)
        measured = front / (front + rear)
        predicted = roll_stiffness_front_share(vehicle, solve_static_equilibrium(vehicle))
        # Measured 0.4856 vs predicted 0.4942: the reference itself is 0.0086
        # away, so 0.02 is the tightest honest tolerance (2.3x that gap).
        self.assertAlmostEqual(measured, predicted, delta=0.02)
        # The design parameter is a different quantity and is not the oracle.
        self.assertNotAlmostEqual(predicted, vehicle.planar.lateral_load_transfer_distribution,
                                  delta=0.02)
        self.assertLess(relaxed.roll, 0.0)

    def test_axle_load_transfer_matches_the_reported_forces_and_link_geometry(self):
        """Absolute oracle for the jacking path, from reported forces only.

        Each corner's steady hub balance is Newton's law with the link force
        paths written geometrically rather than read out of the evaluator:

            N_i = mu_i*g + S_i + D_i + Du_i + JP_i,
            Y_i  = bfy_i - mu_i*ay,
            D_i  = -sign(y_i)*2*h_rc,i*Y_i/track,
            Du_i = -sign(y_i)*2*mu_i*ay*r_1/track,
            JP_i = [-1,-1,+1,+1]*0.5*m_u*ax*r_1/L.

        ``S_i`` is the corner force (spring, ARB, damper) that the evaluator
        reports, so the inputs are the reported tire and suspension forces plus
        the geometry constants. Subtracting the left/right pair, and using that
        the static state is left/right symmetric (so the ``delta`` form of the
        reviewer's transfer identity reduces to the absolute one here), gives
        the axle form

            N_left - N_right = (S_left - S_right)
                + (2/track)*[h_rc*bfy_axle + (r_1 - h_rc)*mu_axle*ay].

        Nothing is fitted and the tolerances are tight: the residuals are ~2e-5 N
        per corner and ~6e-7 N per axle, while every jacking term is 19-21 N at
        this state. Dropping ``Du``, moving ``D`` to the body only, or flipping
        ``JP`` breaks this check by tens of newtons; the LLTD share above cannot
        see any of them.
        """
        vehicle = self.vehicle
        p = vehicle.planar
        relaxed, out, _ = self._steady_lateral()
        jounce = relaxed.as_vector()[13:17]
        bfx, bfy = self._body_forces(out, 0.0, jounce)
        # Pure cornering, so the longitudinal tire force and the anti pair vanish;
        # the hub balance below needs no anti term. This also documents the limit
        # of the oracle: it says nothing about the anti rule (checked in
        # LongitudinalLoadTransferTests).
        np.testing.assert_allclose(bfx, 0.0, rtol=0.0, atol=1e-9)
        ay = float(out.body_accelerations[1])
        ax = float(out.body_accelerations[0])
        self.assertGreater(ay, 2.0)
        um = unsprung_masses(vehicle)
        loads = out.normal_loads
        corner = out.corner_forces.total
        y_wheel = sprung_wheel_positions(vehicle)[:, 1]
        track = 2.0 * p.half_track
        roll_query = float(np.clip(abs(relaxed.roll), 0.0, 5.0 * np.pi / 180.0))
        rc = np.array([float(vehicle.suspension.front.roll_centre_height(roll_query))] * 2
                      + [float(vehicle.suspension.rear.roll_centre_height(roll_query))] * 2)
        sign_y = np.where(y_wheel > 0.0, 1.0, -1.0)
        link = bfy - um * ay
        d = -sign_y * 2.0 * rc * link / track
        du = -sign_y * 2.0 * um * ay * vehicle.unsprung_cg_height / track
        jp = (np.array([-1.0, -1.0, 1.0, 1.0]) * 0.5 * float(np.sum(um)) * ax
              * vehicle.unsprung_cg_height / float(p.wheelbase))
        # Every term must be substantial, or the oracle would prove nothing.
        self.assertTrue(np.all(np.abs(d) > 1.0), d)
        self.assertTrue(np.all(np.abs(du) > 1.0), du)
        self.assertTrue(np.all(np.abs(jp) > 1.0), jp)
        predicted = um * self.G + corner + d + du + jp
        np.testing.assert_allclose(loads, predicted, rtol=0.0, atol=1e-3)
        for axle, ix in (("front", (0, 1)), ("rear", (2, 3))):
            transfer = float(loads[ix[0]] - loads[ix[1]])
            reacted = float(corner[ix[0]] - corner[ix[1]])
            geometric = (2.0 / track) * (
                float(rc[ix[0]]) * float(bfy[ix[0]] + bfy[ix[1]])
                + (vehicle.unsprung_cg_height - float(rc[ix[0]]))
                * float(np.sum(um[list(ix)])) * ay)
            self.assertGreater(abs(geometric), 1.0, geometric)
            with self.subTest(axle=axle):
                self.assertAlmostEqual(transfer, reacted + geometric, delta=1e-3)

    def test_steady_state_has_left_load_greater_and_the_load_transfer_identity(self):
        """Dynamic relaxation gives an independent steady state at +ay."""
        vehicle = self.vehicle
        relaxed, out, _ = self._steady_lateral()

        # The relaxation must have reached a genuine fixed point.
        from simplified_vehicle_models.formulations.vertical_14dof import VERTICAL_ACCELERATION_INDEX
        np.testing.assert_allclose(out.rates[list(VERTICAL_ACCELERATION_INDEX)], 0.0, atol=2e-5)
        ay = float(out.body_accelerations[1])
        self.assertGreater(ay, 2.0)
        self.assertLess(out.rates[3], 0.0)
        self.assertLess(float(relaxed.roll), 0.0)  # roll angle is negative
        loads = out.normal_loads
        self.assertGreater(loads[0], loads[1])
        self.assertGreater(loads[2], loads[3])
        y_wheel = sprung_wheel_positions(vehicle)[:, 1]
        expected = -vehicle.planar.total_mass * vehicle.planar.cg_height * ay
        self.assertAlmostEqual(float(np.sum(y_wheel * loads)), expected,
                               delta=1e-5 * abs(expected))

    def test_newton_euler_force_path_includes_unsprung_jacking(self):
        """M0 a = Q with B and H assembled independently, Du included."""
        from simplified_vehicle_models.formulations.vertical_14dof import (
            VERTICAL_ACCELERATION_INDEX, unsprung_masses, vertical_mass_block)
        vehicle = self.vehicle
        state, tires, out = self._rightward()
        ay = float(out.body_accelerations[1])
        um = unsprung_masses(vehicle)
        y_wheel = sprung_wheel_positions(vehicle)[:, 1]
        link, jacking, jacking_unsprung = self._jacking(out, ay, state.as_vector()[13:17])
        # The unsprung pair is nonzero and is what makes Sigma y*N exact.
        self.assertTrue(np.any(np.abs(jacking_unsprung) > 1.0), jacking_unsprung)
        self.assertLess(float(np.sum(y_wheel * jacking_unsprung)),
                        -0.5 * vehicle.unsprung_cg_height * np.sum(um) * ay)
        from simplified_vehicle_models.formulations.planar_7dof import aerodynamic_forces
        downforce, _ = aerodynamic_forces(vehicle.planar, float(np.hypot(state.v_x, state.v_y)))
        x_wheel = sprung_wheel_positions(vehicle)[:, 0]
        balance = vehicle.planar.aero.aero_balance
        aero_arm = (balance * 0.5 * (x_wheel[0] + x_wheel[1])
                    + (1.0 - balance) * 0.5 * (x_wheel[2] + x_wheel[3]))
        bfx, _ = self._body_forces(out, 0.0, state.as_vector()[13:17])
        ax = float(out.body_accelerations[0])
        anti = anti_forces(vehicle, bfx)
        jacking_pitch = longitudinal_jacking_pitch(vehicle, ax)
        link_longitudinal = bfx - um * ax
        body = out.corner_forces.total + anti + jacking
        hub = (um * self.G + body + jacking_unsprung + jacking_pitch - out.normal_loads)
        gen = np.zeros(7)
        gen[0] = vehicle.sprung_mass * self.G + downforce - np.sum(body) + np.sum(hub)
        gen[1] = -np.sum(y_wheel * body) + np.sum(y_wheel * hub) \
            - vehicle.sprung_cg_height * np.sum(link)
        gen[2] = (np.sum(x_wheel * body) - downforce * aero_arm - np.sum(x_wheel * hub)
                  + vehicle.sprung_cg_height * np.sum(link_longitudinal))
        gen[3:] = -hub
        # Self-consistency/regression check, not independent validation: `gen`
        # is rebuilt with the same body/hub decomposition and the same virtual
        # work the evaluator implements, so agreement pins that decomposition
        # (and that Du, D and the pitch pair really enter the rows) rather than
        # confirming the force path against outside data. The absolute transfer
        # oracle in test_axle_load_transfer_... is the independent one.
        np.testing.assert_allclose(out.generalized_forces, gen, rtol=1e-10, atol=1e-8)
        accel = out.rates[list(VERTICAL_ACCELERATION_INDEX)]
        block = vertical_mass_block(vehicle, state.as_vector()[13:17])
        # Self-consistency: this is the evaluator's own mass block applied to the
        # evaluator's own generalized forces, so it restates the solve step.
        np.testing.assert_allclose(block @ accel, out.generalized_forces, rtol=1e-10, atol=1e-8)

    def test_roll_generalized_force_is_invariant_to_the_roll_centre_height(self):
        """D is an internal pair, so h_rc must not appear in the body coordinates."""
        from dataclasses import replace
        from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

        def scaled(vehicle, factor):
            def scale(spline):
                return MonotoneSpline(spline.x, factor * spline.y)
            s = vehicle.suspension
            return replace(vehicle, suspension=replace(
                s, front=replace(s.front, roll_centre_height=scale(s.front.roll_centre_height)),
                rear=replace(s.rear, roll_centre_height=scale(s.rear.roll_centre_height))))

        state, tires, out = self._rightward()
        doubled = scaled(self.vehicle, 2.0)
        other = evaluate_vertical_dynamics(doubled, state, np.zeros(4), 0.0, tires)
        link, jacking, _ = self._jacking(out, float(out.body_accelerations[1]),
                                         state.as_vector()[13:17])
        self.assertTrue(np.any(np.abs(jacking) > 1.0), jacking)
        # The RC height reaches the jounce rows only.
        np.testing.assert_allclose(other.generalized_forces[0], out.generalized_forces[0], atol=1e-9)
        np.testing.assert_allclose(other.generalized_forces[1], out.generalized_forces[1], atol=1e-9)
        np.testing.assert_allclose(other.generalized_forces[2], out.generalized_forces[2], atol=1e-9)
        self.assertGreater(float(np.max(np.abs(other.generalized_forces[3:] - out.generalized_forces[3:]))),
                           1.0)

    def test_jacking_pair_mirrors_with_nonzero_lateral_force(self):
        """Y is odd, D and Du are even under y -> -y, and none of them is zero."""
        state, tires, out = self._rightward()
        swap = np.array([1, 0, 3, 2])
        q = state.as_vector()
        mirrored = q.copy()
        mirrored[1] *= -1.0
        mirrored[[3, 5, 7]] *= -1.0
        mirror_tires = tuple(TransientTireState(t.longitudinal_slip, -t.lateral_deformation,
                                                -t.trail_angle)
                           for t in (tires[i] for i in swap))
        mir = evaluate_vertical_dynamics(self.vehicle, VerticalState.from_vector(mirrored),
                                         np.zeros(4), 0.0, mirror_tires)
        ay = float(out.body_accelerations[1])
        mirror_ay = float(mir.body_accelerations[1])
        self.assertAlmostEqual(mirror_ay, -ay, delta=1e-9)
        jounce = q[13:17]
        link, d, du = self._jacking(out, ay, jounce)
        link_m, d_m, du_m = self._jacking(mir, mirror_ay, jounce[swap])
        self.assertTrue(np.all(np.abs(link) > 1.0))
        self.assertTrue(np.all(np.abs(d) > 1.0))
        self.assertTrue(np.all(np.abs(du) > 1.0))
        np.testing.assert_allclose(link_m, -link[swap], rtol=1e-9, atol=1e-9)
        np.testing.assert_allclose(d_m, d[swap], rtol=1e-9, atol=1e-9)
        np.testing.assert_allclose(du_m, du[swap], rtol=1e-9, atol=1e-9)


class LongitudinalLoadTransferTests(unittest.TestCase):
    """Task 6 phase 2: the longitudinal force path, unsprung transfer and anti.

    Every scenario is solved to a steady state of the evaluated equations. The
    independent oracles are the closed-form transfer ``m*a*h_cg/L``, the anti
    pair read out of the evaluator against the documented scale rule, and the
    spring-force/dive signs. The whole-vehicle moment identities that follow from
    the same force path are regression anchors, not independent validation; the
    axle-transfer oracle in :class:`LateralLoadTransferTests` is what pins the
    lateral split.
    """

    G = 9.80665
    SPEED = 30.0

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml").model

    def _braking_at(self, delta_ax, anti=True, speed=None):
        """Solve for the braking slip that changes ax by ``delta_ax`` from coasting."""
        vehicle = self.vehicle
        speed = self.SPEED if speed is None else speed
        coast = braking_state(vehicle, 0.0, speed=speed, anti=anti)
        target = float(coast[3].body_accelerations[0]) + delta_ax
        low, high, solution = -0.30, 0.0, None
        for _ in range(24):
            mid = 0.5 * (low + high)
            solution = braking_state(vehicle, mid, speed=speed, anti=anti)
            ax = float(solution[3].body_accelerations[0])
            if abs(ax - target) < 1e-6:
                break
            if ax > target:
                high = mid
            else:
                low = mid
        return solution, coast, target

    def test_braking_transfers_the_whole_vehicle_moment_to_the_front(self):
        """-1.5 g of braking must gain the front the exact whole-vehicle transfer."""
        vehicle = self.vehicle
        solution, coast, target = self._braking_at(-1.5 * self.G, anti=False)
        v, state, tires, out = solution
        ax_coast, ax = float(coast[3].body_accelerations[0]), float(out.body_accelerations[0])
        self.assertAlmostEqual(ax, target, delta=5e-5)
        self.assertAlmostEqual(ax_coast - ax, 1.5 * self.G, delta=5e-5)
        x_wheel = sprung_wheel_positions(v)[:, 0]
        loads, coast_loads = out.normal_loads, coast[3].normal_loads
        delta_ax = ax - ax_coast
        moment = float(np.sum(x_wheel * loads) - np.sum(x_wheel * coast_loads))
        expected_moment = -vehicle.planar.total_mass * vehicle.planar.cg_height * delta_ax
        self.assertAlmostEqual(moment, expected_moment, delta=1e-4 * abs(expected_moment))
        # The transfer is the front gain (equal to the rear loss).
        expected = (vehicle.planar.total_mass * 1.5 * self.G * vehicle.planar.cg_height
                    / vehicle.planar.wheelbase)
        self.assertAlmostEqual(expected, 825.1, delta=1.0)  # the brief's figure
        self.assertAlmostEqual(float(np.sum(loads[:2]) - np.sum(coast_loads[:2])), expected, delta=1.0)
        self.assertAlmostEqual(float(np.sum(coast_loads[2:]) - np.sum(loads[2:])), expected, delta=1.0)

    def test_anti_is_an_internal_pair_and_leaves_the_load_transfer_unchanged(self):
        """Anti changes the spring share only; the tire transfer is invariant."""
        vehicle = self.vehicle
        with_anti, coast_on, _ = self._braking_at(-1.5 * self.G, anti=True)
        without, coast_off, _ = self._braking_at(-1.5 * self.G, anti=False)
        _, state_on, _, out_on = with_anti
        _, state_off, _, out_off = without
        self.assertAlmostEqual(float(out_on.body_accelerations[0]),
                               float(out_off.body_accelerations[0]), delta=5e-5)
        # Anti forces are internal, so the tire loads agree within the solve
        # tolerance (measured 4.5e-13 N).
        np.testing.assert_allclose(out_on.normal_loads, out_off.normal_loads, rtol=0.0, atol=1e-6)
        self.assertAlmostEqual(
            float(np.sum(out_on.normal_loads[:2]) - np.sum(coast_on[3].normal_loads[:2])),
            float(np.sum(out_off.normal_loads[:2]) - np.sum(coast_off[3].normal_loads[:2])),
            delta=1e-6)
        expected = (vehicle.planar.total_mass * 1.5 * self.G * vehicle.planar.cg_height
                    / vehicle.planar.wheelbase)
        self.assertAlmostEqual(
            float(np.sum(out_on.normal_loads[:2]) - np.sum(coast_on[3].normal_loads[:2])),
            expected, delta=0.01)
        # With anti-dive the link carries part of the front transfer, so the front
        # spring force is lower and the dive is smaller; the rear spring force is
        # correspondingly higher (the pair pushes the rear body down).
        self.assertLess(float(np.sum(out_on.corner_forces.spring[:2])),
                        float(np.sum(out_off.corner_forces.spring[:2])))
        self.assertGreater(float(np.sum(out_on.corner_forces.spring[2:])),
                           float(np.sum(out_off.corner_forces.spring[2:])))
        self.assertLess(abs(state_on.pitch), abs(state_off.pitch))

    def _anti_pair(self, vehicle, state, tires, out):
        """Return the anti-dive/squat pair as the EVALUATOR applies it.

        The production vehicle and a copy with both axles' anti fractions zeroed
        are evaluated at the same state and tire states, so every other input
        (tire forces, loads, jacking, inertias) is identical and the difference
        of the reported generalized forces isolates the anti pair:
        ``gen_zeroed - gen_production = +anti`` in the four jounce rows. The
        heave, roll and pitch rows cancel because anti is an internal
        body/hub pair, which is itself asserted by the caller.
        """
        from dataclasses import replace
        s = vehicle.suspension

        def zeroed(axle):
            return replace(axle, anti_brake=0.0, anti_drive=0.0)

        without = replace(vehicle, suspension=replace(
            s, front=zeroed(s.front), rear=zeroed(s.rear)))
        out_without = evaluate_vertical_dynamics(without, state, np.zeros(4), 0.0, tires)
        return out_without.generalized_forces - out.generalized_forces

    def test_anti_sign_scale_and_per_wheel_selection(self):
        """Check the documented anti rule against the EVALUATOR's own force.

        The rule is ``anti_i = sign_i*anti_frac*(h_cg/L)*bfx_i/share``, with the
        brake bias as ``share`` under braking and the driven-axle share under
        drive. The force is read out of the evaluator (the anti-zeroed difference
        in :meth:`_anti_pair`), not recomputed and compared with a second copy of
        the same expression.
        """
        from simplified_vehicle_models.formulations.vertical_14dof import axle_drive_shares
        solution, _, _ = self._braking_at(-1.5 * self.G, anti=True)
        v, state, tires, out = solution
        bfx = out.tire_forces[:, 0]
        self.assertTrue(np.all(bfx < 0.0), bfx)  # pure braking: every wheel rearward
        pair = self._anti_pair(v, state, tires, out)
        anti = pair[3:]
        # Anti is internal: it cancels in heave, roll and pitch exactly.
        np.testing.assert_allclose(pair[:3], 0.0, rtol=0.0, atol=1e-9)
        # Front brake force is rearward, so anti-dive pushes the body up (+).
        self.assertTrue(np.all(anti[:2] > 0.0), anti)
        self.assertTrue(np.all(anti[2:] < 0.0), anti)
        # Scale: k = anti*(h_cg/L)/share, so a front wheel is
        # anti_brake*(h_cg/L)*bfx_front/bias (rear: 1-bias).
        p = v.planar
        bias = float(p.brake_bias)
        wheelbase = float(p.wheelbase)
        h_over_L = float(p.cg_height) / wheelbase
        for i in (0, 1):
            expected = -v.suspension.front.anti_brake * h_over_L * float(bfx[i]) / bias
            self.assertAlmostEqual(float(anti[i]), expected, delta=1e-6)
        for i in (2, 3):
            expected = v.suspension.rear.anti_brake * h_over_L * float(bfx[i]) / (1.0 - bias)
            self.assertAlmostEqual(float(anti[i]), expected, delta=1e-6)
        # The ratio is h_cg, not h_s: the two heights are 7% apart (0.300 vs
        # 0.321 m), so the 1e-6 scale checks above already exclude a mutant that
        # used h_s; this states the distinction explicitly.
        h_s_predict = (v.suspension.front.anti_brake * (v.sprung_cg_height / wheelbase)
                       * float(bfx[0]) / bias)
        self.assertGreater(abs(h_s_predict - float(anti[0])), 0.04 * abs(h_s_predict))
        # The old pro-dive expression (h_s and no reference share, as in the
        # previous implementation) is a different number again.
        old = float(v.suspension.front.anti_brake) * (v.sprung_cg_height / wheelbase) * float(bfx[0])
        self.assertGreater(abs(float(anti[0]) / old), 1.2)
        # Drive branch: an undriven front axle carries no anti-squat reference.
        drive_shares = axle_drive_shares(v)
        self.assertEqual(drive_shares, (0.0, 1.0))
        _, drive_state, drive_tires, drive = braking_state(v, 0.02, speed=self.SPEED,
                                                           anti=True)
        self.assertTrue(np.all(drive.tire_forces[:, 0] > 0.0), drive.tire_forces[:, 0])
        driven = self._anti_pair(v, drive_state, drive_tires, drive)[3:]
        # Zero driven-axle share: the evaluator applies no anti force at the
        # front instead of an unbounded one.
        self.assertEqual(float(driven[0]), 0.0)
        self.assertEqual(float(driven[1]), 0.0)
        # Anti-squat pushes the rear body up under traction (+).
        self.assertTrue(np.all(driven[2:] > 0.0), driven)
        for i in (2, 3):
            expected = (v.suspension.rear.anti_drive * h_over_L
                        * float(drive.tire_forces[i, 0]) / drive_shares[1])
            self.assertAlmostEqual(float(driven[i]), expected, delta=1e-6)
        # The sign is a body-attitude effect: with anti-squat the rear springs
        # carry less, so the rear body sits up (and the rear spring force drops).
        _, drive_off_state, _, drive_off = braking_state(v, 0.02, speed=self.SPEED,
                                                         anti=False)
        self.assertLess(float(np.sum(drive.corner_forces.spring[2:])),
                        float(np.sum(drive_off.corner_forces.spring[2:])))
        self.assertLess(drive_state.pitch, drive_off_state.pitch)  # less squat

    def test_pitch_squats_under_traction_and_dives_under_braking(self):
        """A forward force below the sprung CG is nose-up: +Fx squats, -Fx dives."""
        vehicle = self.vehicle
        _, drive_state, drive_tires, drive = braking_state(vehicle, 0.02, speed=self.SPEED,
                                                           anti=False)
        _, brake_state, brake_tires, brake = braking_state(vehicle, -0.02, speed=self.SPEED,
                                                           anti=False)
        self.assertTrue(np.all(drive.tire_forces[:, 0] > 0.0))
        self.assertTrue(np.all(brake.tire_forces[:, 0] < 0.0))
        # At the same vertical state, isolate the longitudinal-force pitch moment
        # from the drag and aero moments already present while coasting.
        _, coast_state, coast_tires, coast = braking_state(vehicle, 0.0, speed=self.SPEED,
                                                           anti=False)
        moment = float(coast.generalized_forces[2])
        drive_moment = float(evaluate_vertical_dynamics(
            vehicle, coast_state, np.zeros(4), 0.0, drive_tires).generalized_forces[2])
        brake_moment = float(evaluate_vertical_dynamics(
            vehicle, coast_state, np.zeros(4), 0.0, brake_tires).generalized_forces[2])
        self.assertGreater(drive_moment - moment, 0.0)   # +Fx below the CG: nose up
        self.assertLess(brake_moment - moment, 0.0)      # -Fx below the CG: nose down
        self.assertGreater(drive_state.pitch - coast_state.pitch, 0.0)   # squat
        self.assertLess(brake_state.pitch - coast_state.pitch, 0.0)      # dive

    def test_longitudinal_terms_mirror_under_reflection(self):
        """The braking rates and the per-corner transfer mirror left to right."""
        solution, _, _ = self._braking_at(-1.5 * self.G, anti=True)
        v, state, tires, out = solution
        swap = np.array([1, 0, 3, 2])
        q = state.as_vector()
        mirrored = q.copy()
        mirrored[1] *= -1.0
        mirrored[[3, 5, 7]] *= -1.0
        mirror_tires = tuple(TransientTireState(t.longitudinal_slip, -t.lateral_deformation,
                                                -t.trail_angle)
                             for t in (tires[i] for i in swap))
        mir = evaluate_vertical_dynamics(v, VerticalState.from_vector(mirrored), np.zeros(4),
                                         0.0, mirror_tires)
        np.testing.assert_allclose(mir.normal_loads, out.normal_loads[swap], rtol=0.0, atol=1e-9)
        # Heave, pitch and the four jounces are even; roll is odd.
        np.testing.assert_allclose(mir.rates[[2, 4, 17, 18, 19, 20]],
                                   np.r_[out.rates[2], out.rates[4], out.rates[17:21][swap]],
                                   rtol=1e-10, atol=1e-9)
        self.assertAlmostEqual(float(mir.rates[3]), -float(out.rates[3]), delta=1e-9)
        jp = longitudinal_jacking_pitch(v, float(out.body_accelerations[0]))
        self.assertGreater(abs(jp[0]), 1.0)
        np.testing.assert_allclose(jp[swap], jp, rtol=1e-12)


if __name__ == "__main__":
    unittest.main()
