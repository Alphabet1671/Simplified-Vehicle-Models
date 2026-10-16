from pathlib import Path
import unittest

import numpy as np

import simplified_vehicle_models.formulations.planar_7dof as planar
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.load_interval import (
    certify_load_closure,
    load_map_enclosure,
)


FORMULATION = Path(__file__).resolve().parents[1] / "models/formulations/iem24_murphy_real_planar_7dof.xml"


def _point_map(vehicle, state, guess, steering):
    positions = planar.wheel_positions(vehicle)
    angles = planar.wheel_steering_angles(steering)
    longitudinal, lateral = planar.contact_velocities(state, positions, angles)
    step = planar._load_step(vehicle, state, angles, float(np.hypot(state.v_x, state.v_y)),
                             np.abs(state.wheel_speeds), longitudinal, lateral,
                             guess[0], guess[1], 1.0)
    return step.implied_longitudinal_acceleration, step.implied_lateral_acceleration


class LoadIntervalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(FORMULATION).model

    def _random_box(self, random):
        speed = random.uniform(6.0, 35.0)
        center = np.array([0, 0, 0, speed, random.uniform(-1.5, 1.5), random.uniform(-0.8, 0.8),
                           *(speed / 0.2 * random.uniform(0.9, 1.1, 4)), 0.8,
                           random.uniform(-0.3, 0.3)])
        half = np.array([0, 0, 0, 0.05, 0.05, 0.01, 0.5, 0.5, 0.5, 0.5, 0, 0.005])
        return center, half

    def test_enclosure_contains_the_point_map(self):
        random = np.random.default_rng(2)
        for _ in range(150):
            center, half = self._random_box(random)
            guess = random.uniform(-15.0, 15.0, 2)
            guess_half = np.array([0.01, 0.01])
            gx, gy = load_map_enclosure(self.vehicle, (center - half)[None], (center + half)[None],
                                        (guess - guess_half)[None], (guess + guess_half)[None])
            for _ in range(5):
                x = center + half * random.uniform(-1, 1, 12)
                a = guess + guess_half * random.uniform(-1, 1, 2)
                state = planar.PlanarState(x[3], x[4], x[5], x[6:10], 0.8)
                px, py = _point_map(self.vehicle, state, a, x[11])
                self.assertTrue(gx.lo[0, 0] <= px <= gx.hi[0, 0])
                self.assertTrue(gy.lo[0, 0] <= py <= gy.hi[0, 0])

    def test_zero_width_box_is_tight(self):
        random = np.random.default_rng(3)
        center, _ = self._random_box(random)
        guess = np.array([1.0, 4.0])
        gx, gy = load_map_enclosure(self.vehicle, center[None], center[None], guess[None], guess[None])
        self.assertLess(float(gx.hi[0, 0] - gx.lo[0, 0]), 1.0e-8)
        self.assertLess(float(gy.hi[0, 0] - gy.lo[0, 0]), 1.0e-8)

    def test_tangent_encloses_the_finite_difference_jacobian(self):
        random = np.random.default_rng(4)
        center, _ = self._random_box(random)
        guess = np.array([1.0, 4.0])
        state = planar.PlanarState(center[3], center[4], center[5], center[6:10], 0.8)
        gx, gy = load_map_enclosure(self.vehicle, center[None], center[None],
                                    (guess - 1e-6)[None], (guess + 1e-6)[None])
        h = 1.0e-5
        for k, enclosure in enumerate((gx, gy)):
            for j in range(2):
                step = np.zeros(2)
                step[j] = h
                derivative = (_point_map(self.vehicle, state, guess + step, center[11])[k]
                              - _point_map(self.vehicle, state, guess - step, center[11])[k]) / (2 * h)
                self.assertLessEqual(enclosure.tl[0, 0, j] - 1.0e-6, derivative)
                self.assertGreaterEqual(enclosure.th[0, 0, j] + 1.0e-6, derivative)

    def test_point_sized_state_boxes_certify_a_unique_root(self):
        random = np.random.default_rng(5)
        rows, guesses = [], []
        for _ in range(30):
            center, _ = self._random_box(random)
            state = planar.PlanarState(center[3], center[4], center[5], center[6:10], 0.8)
            evaluation = planar.evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), center[11])
            rows.append(center)
            guesses.append([evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                            evaluation.lateral_acceleration + state.yaw_rate * state.v_x])
        rows = np.array(rows)
        # Each row is a degenerate box between two identical nodes.
        boxes = np.stack([rows, rows], axis=-1)
        for i in range(len(rows)):
            result = certify_load_closure(self.vehicle, boxes[i:i + 1],
                                          np.array([guesses[i], guesses[i]]))
            self.assertTrue(result["certified"], result)
            self.assertLess(result["worst_krawczyk_contraction"], 1.0)

    def test_fails_closed_on_bad_input(self):
        box = np.zeros((1, 12, 2))
        self.assertFalse(certify_load_closure(self.vehicle, box, np.zeros((3, 2)))["certified"])
        box[:] = np.nan
        self.assertFalse(certify_load_closure(self.vehicle, box, np.zeros((2, 2)))["certified"])
        self.assertFalse(certify_load_closure(self.vehicle, np.zeros((0, 12, 2)),
                                              np.zeros((1, 2)))["certified"])

    def test_wide_state_boxes_are_not_certified(self):
        # Honesty pin: a box far wider than the true Jacobian allows must not
        # certify, however plausible the node solutions look.
        random = np.random.default_rng(6)
        center, half = self._random_box(random)
        half = half * 400.0
        half[3] = min(half[3], center[3] - 1.0)
        box = np.stack([center - half, center + half], axis=-1)[None]
        state = planar.PlanarState(center[3], center[4], center[5], center[6:10], 0.8)
        evaluation = planar.evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), center[11])
        guess = [evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                 evaluation.lateral_acceleration + state.yaw_rate * state.v_x]
        result = certify_load_closure(self.vehicle, box, np.array([guess, guess]))
        self.assertFalse(result["certified"])


if __name__ == "__main__":
    unittest.main()
