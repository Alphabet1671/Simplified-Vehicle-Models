"""Stateless modal suspension layouts (Task 4): axle roll/heave and four-mode.

The checks follow the plan:

(a) equal mode rates reproduce four independent springs;
(b) a pure mode displacement produces force in that mode only;
(c) a single-wheel displacement gives ``T^T K T``;
(d) zero jounce carries the static load, and a wrong load fails at compile time;
(e) a per-axle roll rate of ``k_corner + k_arb`` reproduces the corner springs
    plus the anti-roll bar of the independent-corner layout;
(f) a non-orthonormal ``T`` or a negative rate fails.
"""

from __future__ import annotations

import re
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from simplified_vehicle_models.compiler.compile import compile_formulation  # noqa: E402
from simplified_vehicle_models.suspension.layouts.axle_roll_heave import (  # noqa: E402
    build_axle_roll_heave,
)
from simplified_vehicle_models.suspension.layouts.four_mode import build_four_mode  # noqa: E402
from simplified_vehicle_models.suspension.layouts.modal import (  # noqa: E402
    AXLE_MODE_MATRIX,
    FOUR_MODE_MATRIX,
    ModalNetwork,
    check_orthonormal,
)
from simplified_vehicle_models.suspension.network import ForceNetwork  # noqa: E402

FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
VEHICLE = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
SUSPENSION_REF = "../suspensions/iem24_murphy_real_suspension.xml"
_CACHE: dict = {}


def reference():
    if "c" not in _CACHE:
        _CACHE["c"] = compile_formulation(FORMULATION).model.suspension
    return _CACHE["c"]


class NoDamper:
    def force(self, velocity):
        v = np.asarray(velocity, dtype=float)
        return SimpleNamespace(force=np.zeros_like(v), slope=np.zeros_like(v),
                               out_of_range=np.zeros(v.shape, dtype=bool))


class LinearDamper:
    def __init__(self, c):
        self.c = c

    def force(self, velocity):
        v = np.asarray(velocity, dtype=float)
        return SimpleNamespace(force=self.c * v, slope=np.full_like(v, self.c),
                               out_of_range=np.zeros(v.shape, dtype=bool))


def four_mode(rates=(80e3, 30e3, 50e3, 10e3), damper=NoDamper):
    h, r, p, w = rates
    return build_four_mode(
        kinematics=reference().kinematics,
        heave_rate=h, roll_rate=r, pitch_rate=p, warp_rate=w,
        heave_damper=damper(), roll_damper=damper(), pitch_damper=damper(),
        warp_damper=damper(),
    )


def axle_layout(rates=(80e3, 30e3, 60e3, 20e3), damper=NoDamper):
    hf, rf, hr, rr = rates
    return build_axle_roll_heave(
        kinematics=reference().kinematics,
        heave_rate_front=hf, roll_rate_front=rf, heave_rate_rear=hr, roll_rate_rear=rr,
        heave_damper_front=damper(), roll_damper_front=damper(),
        heave_damper_rear=damper(), roll_damper_rear=damper(),
    )


def spring_part(suspension, z):
    """Return the force above the static preload."""
    network = suspension.network
    return network.forces(z, np.zeros(4)).total - network.preload


def compile_with(assembly: Path, edit=None):
    """Compile the IEM24 14DoF formulation with ``assembly`` as its suspension."""

    text = VEHICLE.read_text()
    text = text.replace(SUSPENSION_REF, assembly.resolve().as_posix())
    text = re.sub(r'(ref|path)="\.\./', rf'\1="{(ROOT / "models").as_posix()}/', text)
    formulation = FORMULATION.read_text().replace(
        "../vehicles/iem24_murphy_real_planar.xml", "vehicle.xml")
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory).resolve()
        (directory / "vehicle.xml").write_text(text)
        (directory / "formulation.xml").write_text(formulation)
        if edit is not None:
            edit(directory)
        return compile_formulation(directory / "formulation.xml")


class ExampleAssemblyTests(unittest.TestCase):
    """The shipped example XML compile, nest their children and carry the static loads."""

    def test_axle_roll_heave_example(self):
        compiled = compile_with(ROOT / "models/suspensions/iem24_axle_roll_heave.xml")
        suspension = compiled.model.suspension
        self.assertEqual(suspension.layout, "axle-roll-heave")
        roles = {item.role for item in compiled.manifest.loaded_components}
        self.assertIn("suspension/heave-spring-front", roles)
        self.assertIn("suspension/roll-damper-rear", roles)
        np.testing.assert_allclose(suspension.network.static_check(),
                                   suspension.kinematics.static_sprung_load, rtol=1e-12)

    def test_four_mode_example(self):
        compiled = compile_with(ROOT / "models/suspensions/iem24_four_mode.xml")
        suspension = compiled.model.suspension
        self.assertEqual(suspension.layout, "four-mode")
        self.assertEqual(len(suspension.network.rates), 4)
        roles = {item.role for item in compiled.manifest.loaded_components}
        self.assertIn("suspension/warp-spring", roles)

    def test_the_axle_example_matches_the_independent_corner_springs(self):
        compiled = compile_with(ROOT / "models/suspensions/iem24_axle_roll_heave.xml")
        axle = compiled.model.suspension
        independent = reference()
        rng = np.random.default_rng(3)
        for _ in range(5):
            z = rng.uniform(-0.003, 0.003, 4)
            np.testing.assert_allclose(axle.network.static_forces(z),
                                       independent.network.static_forces(z),
                                       rtol=1e-4, atol=0.5)

    def test_a_missing_child_role_is_named(self):
        text = (ROOT / "models/suspensions/iem24_four_mode.xml").read_text()
        text = re.sub(r'\s*<component role="warp-damper"[^>]*/>', "", text)
        text = text.replace('ref="../', f'ref="{(ROOT / "models").as_posix()}/')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory).resolve() / "assembly.xml"
            path.write_text(text)
            with self.assertRaisesRegex(ValueError, "missing child role 'warp-damper'"):
                compile_with(path)

    def test_a_damper_in_a_spring_role_is_rejected(self):
        text = (ROOT / "models/suspensions/iem24_four_mode.xml").read_text()
        text = text.replace("iem24_fourmode_heave.xml", "../dampers/multimatic_dssv_vc01_mode.xml")
        text = text.replace('ref="../', f'ref="{(ROOT / "models").as_posix()}/')
        text = text.replace(f'{(ROOT / "models").as_posix()}/springs/../dampers',
                            f'{(ROOT / "models").as_posix()}/dampers')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory).resolve() / "assembly.xml"
            path.write_text(text)
            with self.assertRaisesRegex(ValueError, "heave-spring.*ModeRate"):
                compile_with(path)


class MatrixTests(unittest.TestCase):
    def test_both_mode_matrices_are_orthonormal(self):
        for matrix in (FOUR_MODE_MATRIX, AXLE_MODE_MATRIX):
            np.testing.assert_allclose(matrix @ matrix.T, np.eye(4), atol=1e-15)
            np.testing.assert_allclose(matrix.T @ matrix, np.eye(4), atol=1e-15)

    def test_four_mode_rows_follow_the_plan(self):
        z = np.array([1.0, 2.0, 3.0, 5.0])
        heave, roll, pitch, warp = FOUR_MODE_MATRIX @ z
        self.assertAlmostEqual(heave, (1 + 2 + 3 + 5) / 2)
        self.assertAlmostEqual(roll, (1 - 2 + 3 - 5) / 2)
        self.assertAlmostEqual(pitch, (1 + 2 - 3 - 5) / 2)
        self.assertAlmostEqual(warp, (1 - 2 - 3 + 5) / 2)

    def test_the_layouts_satisfy_the_network_protocol(self):
        for suspension in (four_mode(), axle_layout()):
            self.assertIsInstance(suspension.network, ForceNetwork)
            self.assertEqual(suspension.network.n_states, 0)
            self.assertFalse(suspension.network.has_inertia)


class EqualRateTests(unittest.TestCase):
    """(a) equal mode rates are four independent springs."""

    def test_four_mode_with_equal_rates(self):
        k = 65e3
        suspension = four_mode((k, k, k, k))
        z = np.array([0.004, -0.007, 0.011, 0.002])
        np.testing.assert_allclose(spring_part(suspension, z), k * z, rtol=1e-12, atol=1e-9)

    def test_axle_layout_with_equal_rates(self):
        k = 65e3
        suspension = axle_layout((k, k, k, k))
        z = np.array([0.004, -0.007, 0.011, 0.002])
        np.testing.assert_allclose(spring_part(suspension, z), k * z, rtol=1e-12, atol=1e-9)


class ModeSeparationTests(unittest.TestCase):
    """(b) a pure mode displacement produces force in that mode only."""

    def test_four_mode_separation(self):
        rates = np.array([80e3, 30e3, 50e3, 10e3])
        suspension = four_mode(tuple(rates))
        for index in range(4):
            mode = np.zeros(4)
            mode[index] = 0.01
            z = FOUR_MODE_MATRIX.T @ mode  # T^-1 = T^T
            force_mode = FOUR_MODE_MATRIX @ spring_part(suspension, z)
            expected = np.zeros(4)
            expected[index] = rates[index] * 0.01
            np.testing.assert_allclose(force_mode, expected, atol=1e-8)

    def test_axle_separation(self):
        rates = np.array([80e3, 30e3, 60e3, 20e3])
        suspension = axle_layout(tuple(rates))
        for index in range(4):
            mode = np.zeros(4)
            mode[index] = 0.01
            z = AXLE_MODE_MATRIX.T @ mode
            force_mode = AXLE_MODE_MATRIX @ spring_part(suspension, z)
            expected = np.zeros(4)
            expected[index] = rates[index] * 0.01
            np.testing.assert_allclose(force_mode, expected, atol=1e-8)

    def test_the_axles_do_not_couple(self):
        suspension = axle_layout()
        z = np.array([0.01, -0.004, 0.0, 0.0])
        rear = spring_part(suspension, z)[2:]
        np.testing.assert_allclose(rear, 0.0, atol=1e-9)


class StiffnessMatrixTests(unittest.TestCase):
    """(c) a single wheel displacement gives the columns of ``T^T K T``."""

    def test_single_wheel_displacement(self):
        for suspension, matrix in ((four_mode(), FOUR_MODE_MATRIX),
                                   (axle_layout(), AXLE_MODE_MATRIX)):
            rates = suspension.network.rates
            stiffness = matrix.T @ np.diag(rates) @ matrix
            np.testing.assert_allclose(suspension.network.stiffness_matrix, stiffness,
                                       rtol=1e-13)
            for wheel in range(4):
                z = np.zeros(4)
                z[wheel] = 1.0e-3
                np.testing.assert_allclose(spring_part(suspension, z),
                                           stiffness[:, wheel] * 1.0e-3, rtol=1e-10, atol=1e-9)

    def test_the_stiffness_is_symmetric_positive_semidefinite(self):
        for suspension in (four_mode(), axle_layout()):
            stiffness = suspension.network.stiffness_matrix
            np.testing.assert_allclose(stiffness, stiffness.T, atol=1e-9)
            self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(stiffness))), -1e-6)

    def test_force_is_the_virtual_work_gradient(self):
        """The spring force is minus the gradient of 0.5 m^T K m, so it does no net work."""
        suspension = four_mode()
        z = np.array([0.003, 0.001, -0.002, 0.004])
        h = 1e-6
        energy = lambda q: 0.5 * q @ suspension.network.stiffness_matrix @ q
        gradient = np.array([(energy(z + h * e) - energy(z - h * e)) / (2 * h)
                             for e in np.eye(4)])
        np.testing.assert_allclose(spring_part(suspension, z), gradient, rtol=1e-6, atol=1e-6)


class StaticTests(unittest.TestCase):
    """(d) zero jounce is the static equilibrium."""

    def test_zero_jounce_carries_the_sprung_static_loads(self):
        expected = reference().kinematics.static_sprung_load
        for suspension in (four_mode(), axle_layout()):
            np.testing.assert_allclose(suspension.network.static_check(), expected, rtol=1e-12)
            total = suspension.network.forces(np.zeros(4), np.zeros(4)).total
            np.testing.assert_allclose(total, expected, rtol=1e-12)

    def test_the_preload_follows_the_kinematics(self):
        """The preload is the kinematics' sprung load, so zero jounce is equilibrium by construction."""
        kinematics = reference().kinematics.with_axle(0, static_sprung_load=480.0)
        suspension = reference().with_kinematics(kinematics)
        network = four_mode().network.with_kinematics(kinematics)
        np.testing.assert_allclose(network.static_check(), kinematics.static_sprung_load)
        self.assertEqual(float(network.preload[0]), 480.0)
        self.assertIsNotNone(suspension)

    def test_zero_mode_rates_leave_only_the_preload(self):
        suspension = four_mode((1e3, 0.0, 0.0, 0.0))
        total = suspension.network.forces(np.zeros(4), np.zeros(4)).total
        np.testing.assert_allclose(total, reference().kinematics.static_sprung_load)


class IndependentEquivalenceTests(unittest.TestCase):
    """(e) corner springs plus an ARB are a per-axle layout with a larger roll rate."""

    def test_roll_rate_of_corner_plus_arb_reproduces_the_independent_law(self):
        independent = reference()
        network = independent.network
        k_front = float(network.springs.front.derivative(0.0))
        k_rear = float(network.springs.rear.derivative(0.0))
        arb_front, arb_rear = network.arb.front, network.arb.rear
        suspension = axle_layout((k_front, k_front + arb_front, k_rear, k_rear + arb_rear))
        rng = np.random.default_rng(7)
        for _ in range(8):
            z = rng.uniform(-0.004, 0.004, 4)  # inside the linear part of the springs
            expected = network.static_forces(z)
            got = suspension.network.static_forces(z)
            np.testing.assert_allclose(got, expected, rtol=1e-5, atol=1e-3)

    def test_the_arb_alone_is_a_pure_roll_rate(self):
        k = 40e3
        arb = 21e3
        suspension = axle_layout((k, k + arb, k, k))
        z = np.array([0.01, -0.01, 0.0, 0.0])
        force = spring_part(suspension, z)
        # Roll coordinate = (zL - zR)/sqrt(2); heave = 0.
        np.testing.assert_allclose(force[:2], (k + 0.5 * arb * 2) * np.array([0.01, -0.01]),
                                   rtol=1e-12)


class DamperTests(unittest.TestCase):
    """A mode damper acts on the wheel-referred mode velocity with no motion ratio."""

    def test_linear_mode_dampers_give_t_transpose_c_t(self):
        c = 1500.0
        suspension = four_mode(damper=lambda: LinearDamper(c))
        zdot = np.array([0.1, -0.2, 0.05, 0.3])
        z = np.zeros(4)
        result = suspension.network.forces(z, zdot)
        expected = c * zdot  # equal c on an orthonormal basis
        np.testing.assert_allclose(result.elements["damper"], expected, rtol=1e-12)
        self.assertEqual(result.out_of_range.shape, (4,))
        self.assertAlmostEqual(float(result.damper_velocity[0]),
                               float(np.max(np.abs(FOUR_MODE_MATRIX @ zdot))))

    def test_a_damper_on_one_mode_leaves_the_others_alone(self):
        dampers = (LinearDamper(2000.0), NoDamper(), NoDamper(), NoDamper())
        suspension = build_four_mode(
            kinematics=reference().kinematics, heave_rate=1e4, roll_rate=1e4,
            pitch_rate=1e4, warp_rate=1e4, heave_damper=dampers[0], roll_damper=dampers[1],
            pitch_damper=dampers[2], warp_damper=dampers[3])
        zdot = np.array([0.1, 0.1, 0.1, 0.1])  # pure heave: heave rate = 0.2
        force = suspension.network.forces(np.zeros(4), zdot).elements["damper"]
        np.testing.assert_allclose(force, 2000.0 * 0.2 * 0.5 * np.ones(4), rtol=1e-12)

    def test_a_damper_flag_propagates(self):
        class Flagged:
            def force(self, velocity):
                v = np.asarray(velocity, dtype=float)
                return SimpleNamespace(force=np.zeros_like(v), slope=np.zeros_like(v),
                                       out_of_range=np.ones(v.shape, dtype=bool))
        suspension = four_mode(damper=Flagged)
        result = suspension.network.forces(np.zeros(4), np.zeros(4))
        self.assertTrue(bool(np.all(result.out_of_range)))


class ValidationTests(unittest.TestCase):
    """(f) a non-orthonormal T or a negative rate fails at compile time."""

    def test_a_non_orthonormal_matrix_fails(self):
        bad = FOUR_MODE_MATRIX.copy()
        bad[0, 0] = 0.6
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            check_orthonormal(bad)
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            ModalNetwork(kinematics=reference().kinematics, transform=2.0 * FOUR_MODE_MATRIX,
                         mode_names=("a", "b", "c", "d"), rates=np.ones(4),
                         dampers=(NoDamper(),) * 4)

    def test_a_wrong_shape_fails(self):
        with self.assertRaisesRegex(ValueError, "4x4"):
            check_orthonormal(np.eye(3))

    def test_a_negative_rate_names_the_mode(self):
        with self.assertRaisesRegex(ValueError, "roll=-1"):
            four_mode((80e3, -1.0, 1.0, 1.0))
        with self.assertRaisesRegex(ValueError, "roll-rear"):
            axle_layout((80e3, 1.0, 60e3, -5.0))

    def test_a_nonpositive_heave_rate_fails(self):
        with self.assertRaisesRegex(ValueError, "heave"):
            four_mode((0.0, 1.0, 1.0, 1.0))

    def test_a_nonfinite_rate_fails(self):
        with self.assertRaises(ValueError):
            four_mode((np.nan, 1.0, 1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
