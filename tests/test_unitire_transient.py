from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    TRANSIENT_COMBINED_SLIP,
    TransientCombinedSlip,
)
from simplified_vehicle_models.compiler.registry import component_descriptor
from simplified_vehicle_models.tires.unitire_simple import (
    compile_unitire_simple,
    evaluate_unitire,
)
from simplified_vehicle_models.tires.unitire_transient import (
    CONTACT_LOSS_TIME_CONSTANT,
    RATE_SPEED_FLOOR,
    WINDOW_FULL_LOAD,
    WINDOW_ZERO_LOAD,
    contact_force_window,
    TransientTireParameters,
    TransientTireState,
    check_relaxation_lengths,
    compile_unitire_transient,
    evaluate_transient_unitire,
    relaxation_lengths,
    steady_rolling_state,
)
from simplified_vehicle_models.xml.loader import load_model


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
STEADY_PATH = REPOSITORY_ROOT / "models/tires/iem24_murphy_real_unitire.xml"
TRANSIENT_PATH = (
    REPOSITORY_ROOT / "models/tires/iem24_murphy_real_unitire_transient.xml"
)
OMEGA = 40.0  # rad/s, a wheel speed of about 10 m/s


def _add(state: TransientTireState, rate: TransientTireState, dt: float):
    return TransientTireState(
        state.longitudinal_slip + dt * rate.longitudinal_slip,
        state.lateral_deformation + dt * rate.lateral_deformation,
        state.trail_angle + dt * rate.trail_angle,
    )


class TransientTireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.steady = compile_unitire_simple(load_model(STEADY_PATH))
        cls.transient = compile_unitire_transient(load_model(TRANSIENT_PATH))

    def _evaluate(self, state, alpha, kappa, fz, vx=10.0, omega=OMEGA):
        return evaluate_transient_unitire(
            self.steady, self.transient, state, alpha, kappa, fz, omega, vx
        )

    def _integrate(self, state, alpha, kappa, fz, duration, vx=10.0, dt=1e-4):
        """Integrate with RK4 at a constant command."""

        def rate(s):
            return self._evaluate(s, alpha, kappa, fz, vx)[1]

        def combine(s, ks, weights):
            return TransientTireState(
                s.longitudinal_slip
                + sum(w * k.longitudinal_slip for w, k in zip(weights, ks)),
                s.lateral_deformation
                + sum(w * k.lateral_deformation for w, k in zip(weights, ks)),
                s.trail_angle
                + sum(w * k.trail_angle for w, k in zip(weights, ks)),
            )

        history = [(0.0, state)]
        for step in range(int(round(duration / dt))):
            k1 = rate(state)
            k2 = rate(combine(state, [k1], [dt / 2]))
            k3 = rate(combine(state, [k2], [dt / 2]))
            k4 = rate(combine(state, [k3], [dt]))
            state = combine(
                state,
                [k1, k2, k3, k4],
                [dt / 6, dt / 3, dt / 3, dt / 6],
            )
            history.append(((step + 1) * dt, state))
        return history

    def _sigma_y(self, fz):
        return relaxation_lengths(self.steady, self.transient, fz)[1]

    # Registration and parameters ---------------------------------------

    def test_registry_protocol_and_proposed_values(self) -> None:
        descriptor = component_descriptor("tire-model", "unitire-transient-simple")

        self.assertEqual(descriptor.capability, TRANSIENT_COMBINED_SLIP)
        self.assertIs(descriptor.compile_function, compile_unitire_transient)
        self.assertIsInstance(self.transient, TransientCombinedSlip)
        self.assertIsInstance(self.transient, TransientTireParameters)
        self.assertEqual(self.transient.longitudinal_length_scale, 1.0)
        self.assertEqual(self.transient.lateral_length_scale, 1.0)
        self.assertEqual(self.transient.trail_pole_ratio, 1.0)
        self.assertEqual(self.transient.trail_lead_ratio, 0.25)
        text = TRANSIENT_PATH.read_text(encoding="utf-8")
        self.assertGreaterEqual(text.count("PROPOSED"), 4)
        self.assertNotIn("<component", text)

    def test_rejects_invalid_parameters(self) -> None:
        model = load_model(TRANSIENT_PATH)
        for name, value in (
            ("longitudinal_length_scale", 0.0),
            ("lateral_length_scale", -1.0),
            ("trail_pole_ratio", 0.0),
            ("trail_lead_ratio", -0.1),
            ("trail_lead_ratio", 1.5),
        ):
            parameters = dict(model.parameters)
            parameters[name] = value
            with self.subTest(name=name, value=value), self.assertRaises(
                ValueError
            ):
                compile_unitire_transient(replace(model, parameters=parameters))
        with self.assertRaises(ValueError):
            compile_unitire_transient(load_model(STEADY_PATH))

    def test_relaxation_lengths_and_domain_check(self) -> None:
        sigma_x, sigma_y = relaxation_lengths(self.steady, self.transient, 660.0)
        self.assertAlmostEqual(sigma_y, 0.1533, places=3)
        self.assertAlmostEqual(sigma_x, 0.11950, places=4)
        reference = evaluate_unitire(self.steady, 0.01, 0.0, 660.0, OMEGA)
        self.assertAlmostEqual(sigma_y, float(reference.relaxation_length), 12)
        check_relaxation_lengths(self.steady, self.transient, 3000.0)
        bad = replace(self.transient, lateral_length_scale=-1.0)
        with self.assertRaises(ValueError):
            check_relaxation_lengths(self.steady, bad, 3000.0)

    # Steady identity ---------------------------------------------------

    def test_steady_state_identity_and_zero_rates(self) -> None:
        for fz in (100.0, 330.0, 660.0, 1500.0):
            for alpha in (-0.12, -0.02, 0.0, 0.03, 0.15):
                for kappa in (-0.1, 0.0, 0.08):
                    with self.subTest(fz=fz, alpha=alpha, kappa=kappa):
                        state = steady_rolling_state(
                            self.steady, self.transient, alpha, kappa, fz
                        )
                        forces, rates = self._evaluate(state, alpha, kappa, fz)
                        ref = evaluate_unitire(
                            self.steady, alpha, kappa, fz, OMEGA
                        )
                        for name in ("longitudinal_force", "lateral_force"):
                            expected = float(getattr(ref, name))
                            got = float(getattr(forces, name))
                            self.assertLessEqual(
                                abs(got - expected),
                                1e-10 * max(abs(expected), 1.0),
                            )
                        self.assertLessEqual(
                            abs(
                                float(forces.aligning_moment)
                                - float(ref.aligning_moment)
                            ),
                            1e-10 * max(abs(float(ref.aligning_moment)), 1.0),
                        )
                        self.assertEqual(
                            float(forces.rolling_resistance_moment),
                            float(ref.rolling_resistance_moment),
                        )
                        self.assertLess(abs(rates.lateral_deformation), 1e-12)
                        self.assertLess(abs(rates.trail_angle), 1e-12)
                        self.assertLess(abs(rates.longitudinal_slip), 1e-12)

    def test_forces_settle_to_steady_after_a_step(self) -> None:
        fz, alpha, kappa = 800.0, 0.06, 0.05
        history = self._integrate(
            TransientTireState(0.0, 0.0, 0.0), alpha, kappa, fz, 1.5, dt=2e-4
        )
        forces, _ = self._evaluate(history[-1][1], alpha, kappa, fz)
        ref = evaluate_unitire(self.steady, alpha, kappa, fz, OMEGA)
        for name in ("longitudinal_force", "lateral_force", "aligning_moment"):
            expected = float(getattr(ref, name))
            self.assertAlmostEqual(
                float(getattr(forces, name)), expected, delta=1e-4 * abs(expected)
            )

    # Lateral dynamics --------------------------------------------------

    def test_lateral_step_time_constant(self) -> None:
        alpha, vx = 0.02, 10.0
        for fz in (330.0, 660.0, 1320.0):
            sigma = self._sigma_y(fz)
            history = self._integrate(
                TransientTireState(0.0, 0.0, 0.0), alpha, 0.0, fz, 0.2, vx=vx
            )
            target = alpha * sigma
            threshold = (1.0 - np.exp(-1.0)) * target
            time = next(
                t for t, s in history if s.lateral_deformation >= threshold
            )
            self.assertAlmostEqual(time, sigma / vx, delta=0.05 * sigma / vx)

    def test_two_loads_have_different_lateral_time_constants(self) -> None:
        low, high = self._sigma_y(330.0), self._sigma_y(1320.0)
        self.assertGreater(high, low * 1.5)
        times = []
        for fz in (330.0, 1320.0):
            history = self._integrate(
                TransientTireState(0.0, 0.0, 0.0), 0.02, 0.0, fz, 0.2
            )
            target = 0.02 * self._sigma_y(fz) * (1.0 - np.exp(-1.0))
            times.append(
                next(t for t, s in history if s.lateral_deformation >= target)
            )
        self.assertAlmostEqual(times[1] / times[0], high / low, delta=0.05 * high / low)

    def test_load_step_at_fixed_deformation_changes_angle_and_force(self) -> None:
        alpha = 0.05
        state = steady_rolling_state(self.steady, self.transient, alpha, 0.0, 660.0)
        before, _ = self._evaluate(state, alpha, 0.0, 660.0)
        after, rates = self._evaluate(state, alpha, 0.0, 1100.0)
        sigma_new = self._sigma_y(1100.0)
        alpha_effective = state.lateral_deformation / sigma_new
        self.assertLess(alpha_effective, alpha)
        expected = evaluate_unitire(self.steady, alpha_effective, 0.0, 1100.0, OMEGA)
        self.assertEqual(float(after.lateral_force), float(expected.lateral_force))
        # Force responds immediately to the larger load, and the rate drives the
        # carcass back toward the commanded angle.
        self.assertGreater(float(after.lateral_force), float(before.lateral_force))
        steady_new = evaluate_unitire(self.steady, alpha, 0.0, 1100.0, OMEGA)
        self.assertLess(float(after.lateral_force), float(steady_new.lateral_force))
        self.assertAlmostEqual(
            rates.lateral_deformation, 10.0 * (alpha - alpha_effective), 12
        )
        self.assertGreater(rates.lateral_deformation, 0.0)

    # Trail -------------------------------------------------------------

    def test_trail_phase_lag_and_zero_steady_lead(self) -> None:
        fz, alpha = 660.0, 0.06
        history = self._integrate(
            TransientTireState(0.0, 0.0, 0.0), alpha, 0.0, fz, 0.03
        )
        state = history[len(history) // 3][1]
        sigma = self._sigma_y(fz)
        alpha_effective = state.lateral_deformation / sigma
        self.assertGreater(alpha_effective, state.trail_angle)  # trail lags
        self.assertGreater(state.trail_angle, 0.0)
        forces, rates = self._evaluate(state, alpha, 0.0, fz)
        self.assertGreater(rates.trail_angle, 0.0)
        t_alpha = float(evaluate_unitire(self.steady, alpha_effective, 0.0, fz, OMEGA).pneumatic_trail)
        t_trail = float(evaluate_unitire(self.steady, state.trail_angle, 0.0, fz, OMEGA).pneumatic_trail)
        t_effective = t_trail + 0.25 * (t_alpha - t_trail)
        fy = float(forces.lateral_force)
        self.assertAlmostEqual(
            float(forces.aligning_moment), -(fy * t_effective), places=9
        )
        # Lead: effective trail lies between the delayed and instantaneous trail.
        self.assertNotEqual(t_trail, t_alpha)
        lo, hi = sorted((t_trail, t_alpha))
        self.assertTrue(lo < t_effective < hi)
        # Zero correction at steady state, for any lead ratio.
        for lead in (0.0, 0.25, 1.0):
            tire = replace(self.transient, trail_lead_ratio=lead)
            settled = steady_rolling_state(self.steady, tire, alpha, 0.0, fz)
            got, _ = evaluate_transient_unitire(
                self.steady, tire, settled, alpha, 0.0, fz, OMEGA, 10.0
            )
            ref = evaluate_unitire(self.steady, alpha, 0.0, fz, OMEGA)
            self.assertAlmostEqual(
                float(got.aligning_moment),
                float(ref.aligning_moment),
                delta=1e-10 * abs(float(ref.aligning_moment)),
            )

    def test_mz_equals_steady_when_trail_equals_effective_angle(self) -> None:
        fz, alpha, kappa = 700.0, 0.07, 0.03
        sigma_y = self._sigma_y(fz)
        deformation = 0.05 * sigma_y
        alpha_effective = deformation / sigma_y
        state = TransientTireState(kappa, deformation, alpha_effective)
        for lead in (0.0, 0.25, 1.0):
            tire = replace(self.transient, trail_lead_ratio=lead)
            got, _ = evaluate_transient_unitire(
                self.steady, tire, state, alpha, kappa, fz, OMEGA, 10.0
            )
            ref = evaluate_unitire(self.steady, alpha_effective, kappa, fz, OMEGA)
            self.assertEqual(float(got.aligning_moment), float(ref.aligning_moment))
            self.assertEqual(float(got.lateral_force), float(ref.lateral_force))
            self.assertEqual(float(got.longitudinal_force), float(ref.longitudinal_force))

    def test_trail_pole_ratio_scales_trail_rate(self) -> None:
        state = TransientTireState(0.0, 0.0, 0.0)
        # Command angle with alpha_effective = 0 so the trail has a non-zero input.
        state = TransientTireState(0.0, 0.05 * self._sigma_y(660.0), 0.0)
        _, base = self._evaluate(state, 0.05, 0.0, 660.0)
        tire = replace(self.transient, trail_pole_ratio=2.0)
        _, fast = evaluate_transient_unitire(
            self.steady, tire, state, 0.05, 0.0, 660.0, OMEGA, 10.0
        )
        self.assertAlmostEqual(fast.trail_angle, 2.0 * base.trail_angle, 12)

    # Longitudinal ------------------------------------------------------

    def test_longitudinal_step_convergence(self) -> None:
        kappa, fz, vx = 0.08, 660.0, 10.0
        sigma_x = relaxation_lengths(self.steady, self.transient, fz)[0]
        history = self._integrate(
            TransientTireState(0.0, 0.0, 0.0), 0.0, kappa, fz, 0.3, vx=vx
        )
        threshold = (1.0 - np.exp(-1.0)) * kappa
        time = next(t for t, s in history if s.longitudinal_slip >= threshold)
        self.assertAlmostEqual(time, sigma_x / vx, delta=0.05 * sigma_x / vx)
        self.assertAlmostEqual(history[-1][1].longitudinal_slip, kappa, delta=1e-4 * kappa)
        forces, _ = self._evaluate(history[-1][1], 0.0, kappa, fz)
        ref = evaluate_unitire(self.steady, 0.0, kappa, fz, OMEGA)
        self.assertAlmostEqual(
            float(forces.longitudinal_force),
            float(ref.longitudinal_force),
            delta=1e-3 * float(ref.longitudinal_force),
        )

    # Contact -----------------------------------------------------------

    def test_contact_loss_zero_forces_and_state_relaxation(self) -> None:
        state = TransientTireState(0.05, 0.004, 0.03)
        for fz in (WINDOW_ZERO_LOAD, 0.3, 0.0, -20.0):
            with self.subTest(fz=fz):
                forces, _ = self._evaluate(state, 0.05, 0.05, fz)
                self.assertEqual(float(forces.longitudinal_force), 0.0)
                self.assertEqual(float(forces.lateral_force), 0.0)
                self.assertEqual(float(forces.aligning_moment), 0.0)
                # Rolling resistance is not windowed; it is zero here because
                # the fitted rolling coefficients are zero and it scales with Fz.
                self.assertEqual(float(forces.rolling_resistance_moment), 0.0)
                # The window does not invent or remove vertical contact force.
                self.assertEqual(float(forces.vertical_force), max(fz, 0.0))
        for fz in (WINDOW_ZERO_LOAD, 0.3, 0.0, -20.0):
            with self.subTest(rates_fz=fz):
                _, rates = self._evaluate(state, 0.05, 0.05, fz)
                self.assertAlmostEqual(
                    rates.longitudinal_slip, -0.05 / CONTACT_LOSS_TIME_CONSTANT, 9
                )
                self.assertAlmostEqual(
                    rates.lateral_deformation, -0.004 / CONTACT_LOSS_TIME_CONSTANT, 9
                )
                self.assertAlmostEqual(
                    rates.trail_angle, -0.03 / CONTACT_LOSS_TIME_CONSTANT, 9
                )
        self.assertEqual(CONTACT_LOSS_TIME_CONSTANT, 0.01)

    def test_force_window_is_smooth_and_continuous(self) -> None:
        self.assertEqual((WINDOW_ZERO_LOAD, WINDOW_FULL_LOAD), (0.5, 2.0))
        self.assertEqual(contact_force_window(0.5), 0.0)
        self.assertEqual(contact_force_window(2.0), 1.0)
        self.assertEqual(contact_force_window(5.0), 1.0)
        self.assertAlmostEqual(contact_force_window(1.25), 0.5, 15)
        grid = np.linspace(0.0, 3.0, 3001)
        values = np.array([contact_force_window(f) for f in grid])
        self.assertTrue(np.all(np.diff(values) >= 0.0))
        self.assertLess(np.max(np.abs(np.diff(values))), 2e-3)

        h = 1e-6
        for edge in (WINDOW_ZERO_LOAD, WINDOW_FULL_LOAD):
            slope = (
                contact_force_window(edge + h) - contact_force_window(edge - h)
            ) / (2.0 * h)
            self.assertLess(abs(slope), 1e-5)  # C1 at both window edges

        state = steady_rolling_state(self.steady, self.transient, 0.08, 0.05, 600.0)
        fy, fx, mz = [], [], []
        for load in grid[:-1]:
            forces, _ = self._evaluate(state, 0.08, 0.05, float(load))
            fy.append(float(forces.lateral_force))
            fx.append(float(forces.longitudinal_force))
            mz.append(float(forces.aligning_moment))
        for series in (fy, fx, mz):
            self.assertLess(np.max(np.abs(np.diff(series))), 0.05)
        self.assertEqual(fy[0], 0.0)
        self.assertEqual(fy[500], 0.0)  # 0.5 N
        self.assertGreater(abs(fy[1000]), 0.0)  # 1.0 N is inside the window
        windowed, _ = self._evaluate(state, 0.08, 0.05, 1.0)
        full = evaluate_unitire(
            self.steady,
            state.lateral_deformation / self._sigma_y(1.0),
            state.longitudinal_slip,
            1.0,
            OMEGA,
        )
        self.assertAlmostEqual(
            float(windowed.lateral_force),
            contact_force_window(1.0) * float(full.lateral_force),
            12,
        )
        above, _ = self._evaluate(state, 0.08, 0.05, 2.0)
        ref = evaluate_unitire(
            self.steady, state.lateral_deformation / self._sigma_y(2.0), 0.05, 2.0, OMEGA
        )
        self.assertEqual(float(above.lateral_force), float(ref.lateral_force))

    def test_state_rates_blend_continuously_across_the_window(self) -> None:
        state = TransientTireState(0.03, 0.004, 0.02)
        previous = None
        for load in np.linspace(0.0, 3.0, 3001):
            _, rates = self._evaluate(state, 0.05, 0.05, float(load))
            current = np.array(
                [rates.longitudinal_slip, rates.lateral_deformation, rates.trail_angle]
            )
            if previous is not None:
                scale = np.maximum(np.abs(current), 1.0)
                self.assertLess(np.max(np.abs(current - previous) / scale), 0.02)
            previous = current
        # Inside the window the rate is the weighted mix of the two laws.
        _, low = self._evaluate(state, 0.05, 0.05, 0.4)
        _, high = self._evaluate(state, 0.05, 0.05, 2.0)
        _, mid = self._evaluate(state, 0.05, 0.05, 1.25)
        weight = contact_force_window(1.25)
        self.assertAlmostEqual(weight, 0.5, 15)
        self.assertTrue(
            min(low.lateral_deformation, high.lateral_deformation)
            < mid.lateral_deformation
            < max(low.lateral_deformation, high.lateral_deformation)
        )

    def test_recontact_starts_from_current_state_with_bounded_rates(self) -> None:
        state = TransientTireState(0.0, 0.0, 0.0)
        fz = 2.0
        _, rates = self._evaluate(state, 0.08, 0.06, fz)
        sigma_y = self._sigma_y(fz)
        sigma_x = relaxation_lengths(self.steady, self.transient, fz)[0]
        self.assertTrue(np.isfinite(rates.lateral_deformation))
        self.assertAlmostEqual(rates.lateral_deformation, 10.0 * 0.08, 9)
        _, slow = self._evaluate(state, 0.08, 0.06, fz, vx=0.0)
        self.assertAlmostEqual(
            slow.lateral_deformation, RATE_SPEED_FLOOR * 0.08, 12
        )
        self.assertAlmostEqual(
            rates.longitudinal_slip, 10.0 * 0.06 / sigma_x, 9
        )
        self.assertGreater(sigma_y, 0.0)
        # Relax at lifted, return at load: the state carries through.
        lifted = TransientTireState(0.03, 0.002, 0.02)
        _, rates_lifted = self._evaluate(lifted, 0.0, 0.0, 0.0)
        moved = _add(lifted, rates_lifted, 0.005)
        self.assertAlmostEqual(moved.trail_angle, 0.01, 12)
        forces, rates_back = self._evaluate(moved, 0.02, 0.0, 500.0)
        self.assertGreater(abs(float(forces.lateral_force)), 0.0)
        self.assertTrue(np.isfinite(rates_back.lateral_deformation))

    def test_zero_speed_rates_are_finite(self) -> None:
        state = TransientTireState(0.02, 0.003, 0.01)
        forces, rates = self._evaluate(state, 0.05, 0.1, 600.0, vx=0.0, omega=0.0)
        for value in (
            rates.longitudinal_slip,
            rates.lateral_deformation,
            rates.trail_angle,
            float(forces.longitudinal_force),
            float(forces.lateral_force),
            float(forces.aligning_moment),
        ):
            self.assertTrue(np.isfinite(value))
        sigma_x = relaxation_lengths(self.steady, self.transient, 600.0)[0]
        self.assertAlmostEqual(
            rates.longitudinal_slip,
            RATE_SPEED_FLOOR * (0.1 - 0.02) / sigma_x,
            12,
        )
        sigma_y = self._sigma_y(600.0)
        alpha_effective = 0.003 / sigma_y
        self.assertAlmostEqual(
            rates.lateral_deformation,
            RATE_SPEED_FLOOR * (0.05 - alpha_effective),
            12,
        )
        self.assertAlmostEqual(
            rates.trail_angle,
            RATE_SPEED_FLOOR * (alpha_effective - 0.01) / sigma_y,
            12,
        )

    def test_rolling_resistance_is_not_windowed(self) -> None:
        tire = replace(self.steady, f_rr_0=0.01)
        state = steady_rolling_state(tire, self.transient, 0.0, 0.0, 1.25)
        forces, _ = evaluate_transient_unitire(
            tire, self.transient, state, 0.0, 0.0, 1.25, OMEGA, 10.0
        )
        ref = evaluate_unitire(tire, 0.0, 0.0, 1.25, OMEGA)
        self.assertNotEqual(float(ref.rolling_resistance_moment), 0.0)
        self.assertEqual(
            float(forces.rolling_resistance_moment),
            float(ref.rolling_resistance_moment),
        )

    def test_rates_use_speed_magnitude(self) -> None:
        state = TransientTireState(0.0, 0.0, 0.0)
        _, forward = self._evaluate(state, 0.05, 0.05, 600.0, vx=8.0)
        _, reverse = self._evaluate(state, 0.05, 0.05, 600.0, vx=-8.0)
        self.assertEqual(forward, reverse)

    def test_slip_signs(self) -> None:
        for fz in (300.0, 900.0):
            state = steady_rolling_state(self.steady, self.transient, 0.05, 0.05, fz)
            positive, _ = self._evaluate(state, 0.05, 0.05, fz)
            mirrored = steady_rolling_state(
                self.steady, self.transient, -0.05, -0.05, fz
            )
            negative, _ = self._evaluate(mirrored, -0.05, -0.05, fz)
            self.assertGreater(float(positive.lateral_force), 0.0)
            self.assertGreater(float(positive.longitudinal_force), 0.0)
            self.assertLess(float(positive.aligning_moment), 0.0)
            self.assertAlmostEqual(
                float(negative.lateral_force), -float(positive.lateral_force), 9
            )
            self.assertAlmostEqual(
                float(negative.longitudinal_force),
                -float(positive.longitudinal_force),
                9,
            )
            # The Fx*Fy carcass-offset terms of Mz are even in the combined
            # slip, so Mz is odd only for pure lateral slip.
            pure_positive = steady_rolling_state(self.steady, self.transient, 0.05, 0.0, fz)
            pure_negative = steady_rolling_state(self.steady, self.transient, -0.05, 0.0, fz)
            mz_positive, _ = self._evaluate(pure_positive, 0.05, 0.0, fz)
            mz_negative, _ = self._evaluate(pure_negative, -0.05, 0.0, fz)
            self.assertAlmostEqual(
                float(mz_negative.aligning_moment),
                -float(mz_positive.aligning_moment),
                9,
            )
            reference = evaluate_unitire(self.steady, -0.05, -0.05, fz, OMEGA)
            self.assertAlmostEqual(
                float(negative.aligning_moment), float(reference.aligning_moment), 9
            )

    def test_steady_rolling_state_values(self) -> None:
        state = steady_rolling_state(self.steady, self.transient, 0.04, 0.02, 660.0)
        self.assertEqual(state.longitudinal_slip, 0.02)
        self.assertEqual(state.trail_angle, 0.04)
        self.assertAlmostEqual(
            state.lateral_deformation, 0.04 * self._sigma_y(660.0), 15
        )
        lifted = steady_rolling_state(self.steady, self.transient, 0.04, 0.02, 0.0)
        self.assertEqual(
            (lifted.longitudinal_slip, lifted.lateral_deformation, lifted.trail_angle),
            (0.0, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
