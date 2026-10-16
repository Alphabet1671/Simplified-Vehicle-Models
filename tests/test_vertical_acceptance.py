"""Task 6 acceptance gates for the vertical 14-DoF model (independent oracles).

The gates follow ``.superpowers/sdd/2026-10-16-vertical-14dof-model/task-6-brief.md``
and the ledger rulings in ``.../progress.md``:

* **Modal.** The plan's 3.4-4.7 Hz ride band and its 17.61 / 16.63 Hz hop pair
  are quarter-car, body-fixed references. The ruled acceptance is (i) the
  heave-dominated mode inside 3.4-4.7 Hz, (ii) the four *body-fixed* hop modes
  (heave/roll/pitch constrained) against the quarter-car formula with the
  tangent tire stiffness and the rear ARB, (iii) a regression pin of the seven
  full-vehicle undamped modes to the parameter-implied diagnosis values,
  identified from the eigenvectors, and (iv) the pitch mode reported as
  **outside** that band: the D5 pitch radius (0.386 m, user-reported) puts it
  near 7.3 Hz. Do not tune springs, ARB, tires, or inertias (final ledger
  ruling).
* **Inertance.** The inerter is the ruled two-terminal hub-to-chassis-point
  device whose relative velocity is ``jounce_rate / MR``, so its kinetic energy
  adds ``b / MR^2`` to the jounce diagonal only. ``M`` grows and the body-fixed
  hop frequencies fall like ``sqrt(m_u / (m_u + b/MR^2))``; the heave mode is
  nearly unchanged.
* **Energy.** With dampers, tire vertical damping, and inertance at zero the
  standalone 7-coordinate vertical subsystem is conservative. The gate uses
  the analytic spring/ARB/tire potential and the evaluator's own rates, RK45
  with ``rtol=1e-10``, ``atol=1e-12``. The quadratic form ``0.5 dx' K dx`` is
  only the small-amplitude limit; its measured drift is reported as evidence
  that the nonlinear potential, not the quadratic form, is the invariant.
* **Planar reduction.** With the vertical coordinates frozen by springs
  stiffened 1e4x about their preload and a test-only LLTD equal to the model's
  own elastic split, the planar rates, loads, and steady slip forces reproduce
  ``evaluate_planar_dynamics`` to 1e-2. The aero drag is zeroed for that
  comparison: see ``test_aero_drag_couple_explains_the_planar_reduction_gap``
  for the measured reason the aero-enabled comparison cannot reach 1e-2
  (the planar quasi-static load rule books the longitudinal transfer as
  ``m*a*h/L`` while the vertical model books the actual force couple).
  Tire stiffening is not representable: scaling the loaded-radius coefficients
  by 1e4 moves the static deflection below the evaluator's fixed 0.5 mm
  low-load ramp knot, where the elastic load is exactly zero, so no
  equilibrium exists. The physical tire law is kept.

All modal and energy gates run at speed 0, where the static solve is an exact
stationary point of the evaluator. At speed > 0 the static solver (which
carries no tire longitudinal force) leaves a drag pitch-link residual, so a
speed-dependent state would have to be refined on the evaluator's generalized
forces first; that refinement is done only for the planar-reduction states,
where it is required.
"""

import types
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.core.conventions import GRAVITY as PLANAR_GRAVITY
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState,
    aerodynamic_forces,
    contact_velocities,
    evaluate_normal_loads,
    evaluate_planar_dynamics,
    slip_quantities,
    wheel_positions,
    wheel_steering_angles,
)
from simplified_vehicle_models.formulations.vertical_14dof import (
    VERTICAL_ACCELERATION_INDEX,
    VerticalState,
    evaluate_vertical_dynamics,
    sprung_wheel_positions,
    unsprung_masses,
    vertical_mass_block,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.unitire_transient import (
    TransientTireState,
    steady_rolling_state,
)
from simplified_vehicle_models.tires.vertical import tire_deflection_load

try:  # The plan allows SciPy in tests; skip the energy class if it is absent.
    from scipy.integrate import solve_ivp
except ImportError:  # pragma: no cover - environment dependent
    solve_ivp = None


ROOT = Path(__file__).resolve().parents[1]
FORMULATION_XML = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"

GRAVITY = 9.80665
#: Position and rate rows of the 21-value vertical state layout.
POSITION_ROWS = (6, 7, 8, 13, 14, 15, 16)
RATE_ROWS = (2, 3, 4, 17, 18, 19, 20)
#: Shape of the small non-equilibrium perturbation, per unit amplitude.
PERTURBATION_SHAPE = np.array([1.0, 0.3, -0.2, 0.5, -0.4, 0.3, -0.2])

_VEHICLE_CACHE: dict = {}


def compiled_vehicle():
    """Return the compiled IEM24 14-DoF vehicle, cached across the module."""

    if "vehicle" not in _VEHICLE_CACHE:
        _VEHICLE_CACHE["vehicle"] = compile_formulation(FORMULATION_XML).model
    return _VEHICLE_CACHE["vehicle"]


class ZeroDamper:
    """A damper stub that returns zero force at every velocity (undamped gate)."""

    def force(self, velocity):
        velocity = np.asarray(velocity, dtype=float)
        return types.SimpleNamespace(
            force=np.zeros_like(velocity),
            out_of_range=np.zeros(velocity.shape, dtype=bool),
        )


def undamped(vehicle):
    """Return the vehicle with both dampers and the tire vertical damping removed."""

    return replace(
        vehicle,
        dampers=(ZeroDamper(), ZeroDamper()),
        tire_vertical_damping=0.0,
    )


def with_inertance(vehicle, front, rear):
    """Return the vehicle with the given axle inertance values [kg]."""

    suspension = vehicle.suspension
    return replace(
        vehicle,
        suspension=replace(
            suspension,
            front=replace(suspension.front, inertance=front),
            rear=replace(suspension.rear, inertance=rear),
        ),
    )


def stiff_springs(vehicle, factor=1.0e4):
    """Return the vehicle with both spring curves stiffened about their preload.

    The spring curve carries the static corner load at jounce 0, so the
    stiffening must preserve that value: ``F_new(j) = F(0) + factor*(F(j) - F(0))``.
    Scaling the whole curve instead would scale the preload and move the
    equilibrium to a large jounce.
    """

    suspension = vehicle.suspension

    def scaled(spline):
        preload = float(spline(0.0))
        return MonotoneSpline(spline.x, preload + factor * (spline.y - preload))

    return replace(
        vehicle,
        suspension=replace(
            suspension,
            front=replace(suspension.front, wheel_rate_spring=scaled(suspension.front.wheel_rate_spring)),
            rear=replace(suspension.rear, wheel_rate_spring=scaled(suspension.rear.wheel_rate_spring)),
        ),
    )


def static_state(vehicle):
    """Return ``(q0, tires)`` at the speed-0 static equilibrium."""

    q0 = solve_static_equilibrium(vehicle).as_vector()
    zero = (TransientTireState(0.0, 0.0, 0.0),) * 4
    loads = evaluate_vertical_dynamics(
        vehicle, VerticalState.from_vector(q0), np.zeros(4), 0.0, zero
    ).normal_loads
    tires = tuple(
        steady_rolling_state(vehicle.planar.tire, vehicle.transient_tire, 0.0, 0.0, float(load))
        for load in loads
    )
    return q0, tires


def state_with(x, rates=None):
    """Build a 21-value state from the seven vertical coordinates and rates."""

    q = np.zeros(21)
    q[list(POSITION_ROWS)] = np.asarray(x, dtype=float)
    if rates is not None:
        q[list(RATE_ROWS)] = np.asarray(rates, dtype=float)
    return VerticalState.from_vector(q)


def generalized_forces(vehicle, x, tires):
    """Return the seven vertical generalized forces at positions ``x`` with zero rates."""

    out = evaluate_vertical_dynamics(vehicle, state_with(x), np.zeros(4), 0.0, tires)
    return np.asarray(out.generalized_forces, dtype=float)


def stiffness_matrix(vehicle, x0, tires):
    """Central-difference 7x7 stiffness ``K = -dQ/dx`` of the vertical subsystem.

    ``Q`` is the evaluator's ``generalized_forces`` for ``[heave, roll, pitch,
    jounce x4]``, so this is the second-order system ``M xddot = -K dx`` whose
    eigenfrequencies are ``sqrt(eig(M^-1 K))``. Position-only differences leave
    the dampers and the tire damping out by construction (both are zero at zero
    velocity), which is the undamped gate the ledger requires.
    """

    x0 = np.asarray(x0, dtype=float)
    matrix = np.empty((7, 7))
    for column in range(7):
        step = 1.0e-7 * (1.0 + abs(x0[column]))
        plus, minus = x0.copy(), x0.copy()
        plus[column] += step
        minus[column] -= step
        matrix[:, column] = -(
            generalized_forces(vehicle, plus, tires) - generalized_forces(vehicle, minus, tires)
        ) / (2.0 * step)
    return matrix


def body_fixed_hop_stiffness(vehicle, x0, tires):
    """Return the 4x4 jounce stiffness with heave, roll, and pitch held fixed.

    Only the four jounce coordinates are perturbed, so this is the body-fixed
    hop subsystem: the sprung body is an inertial ground and the springs, the
    ARB coupling, and the tires act on the hub masses alone.
    """

    x0 = np.asarray(x0, dtype=float)
    matrix = np.empty((4, 4))
    for column in range(4):
        step = 1.0e-7
        plus, minus = x0.copy(), x0.copy()
        plus[3 + column] += step
        minus[3 + column] -= step
        matrix[:, column] = -(
            generalized_forces(vehicle, plus, tires)[3:] - generalized_forces(vehicle, minus, tires)[3:]
        ) / (2.0 * step)
    return matrix


def modal_solution(mass, stiffness):
    """Return ``(frequencies [Hz], mass-normalized eigenvectors)``.

    The generalized problem ``K v = w^2 M v`` is symmetrized with the Cholesky
    factor of the positive-definite mass matrix, and the eigenvectors are
    mapped back to physical coordinates with ``u = L^-T V``, so
    ``u.T @ M @ u`` is the identity and ``u_j^2 * M_jj`` is the mode's
    physical participation in coordinate ``j``.
    """

    lower = np.linalg.cholesky(mass)
    reduced = np.linalg.solve(lower, stiffness).T
    reduced = 0.5 * (np.linalg.solve(lower, reduced) + np.linalg.solve(lower, reduced).T)
    eigenvalues, eigenvectors = np.linalg.eigh(reduced)
    physical = np.linalg.solve(lower.T, eigenvectors)
    return np.sqrt(np.maximum(eigenvalues, 0.0)) / (2.0 * np.pi), physical


def hop_frequencies(vehicle, hop_stiffness, block):
    """Return the four body-fixed hop frequencies [Hz] of the given vehicles.

    The hub masses come from the jounce diagonal of the physical block, which
    already carries the ruled ``b / MR^2`` inerter term when inertance is set.
    """

    masses = np.diag(np.asarray(block, dtype=float))[3:]
    eigenvalues = np.linalg.eigvals(np.linalg.solve(np.diag(masses), hop_stiffness))
    return np.sqrt(np.sort(eigenvalues.real)) / (2.0 * np.pi)


def tangent_tire_stiffness(vehicle, x0):
    """Return the four elastic tire stiffnesses [N/m] at the static deflection."""

    positions = sprung_wheel_positions(vehicle)
    x0 = np.asarray(x0, dtype=float)
    body = x0[0] + positions[:, 1] * x0[1] - positions[:, 0] * x0[2]
    deflection = body - x0[3:]
    return np.asarray(
        tire_deflection_load(vehicle.planar.tire, deflection, np.zeros(4), 0.0).stiffness,
        dtype=float,
    )


def closed_form_mass(vehicle):
    """Independent 7x7 physical mass block built from the compiled parameters.

    ``M = diag(m_s, I_roll, I_pitch, 0) + sum_i m_ui c_i c_i^T`` with
    ``c_i = [1, y_i, -x_i, -e_i]`` the hub downward-velocity coefficients and
    the arms measured from the sprung CG (see the mass ledger entry).
    """

    positions = sprung_wheel_positions(vehicle)
    masses = unsprung_masses(vehicle)
    matrix = np.zeros((7, 7))
    matrix[0, 0] = vehicle.sprung_mass
    matrix[1, 1] = vehicle.roll_inertia
    matrix[2, 2] = vehicle.pitch_inertia
    for index in range(4):
        column = np.zeros(7)
        column[:3] = (1.0, positions[index, 1], -positions[index, 0])
        column[3 + index] = -1.0
        matrix += masses[index] * np.outer(column, column)
    return matrix


def closed_form_stiffness(vehicle, x0):
    """Independent 7x7 stiffness built from the compiled spring/tire parameters.

    With ``z_body,i = h + y_i roll - x_i pitch`` and ``hub_i = z_body,i - j_i``:

    * the elastic tire load perturbation is ``dN_i = k_t,i * d(hub_i)``;
    * the suspension force perturbation is the spring tangent plus the ARB
      half-rate ``0.5 * k_arb * (j_L - j_R)`` added to the left wheel;
    * the evaluator's generalized forces are
      ``gen[0] = -sum dN``, ``gen[1] = -sum y_i dN_i``,
      ``gen[2] = +sum x_i dN_i`` and ``gen[3+i] = -dF_i + dN_i``.

    This ignores anti-dive/squat and roll-centre jacking, which are exactly
    zero at the speed-0 static state (no tire tangential force), so the
    linearization there is purely springs, ARB, and tires.
    """

    positions = sprung_wheel_positions(vehicle)
    x0 = np.asarray(x0, dtype=float)
    tire = tangent_tire_stiffness(vehicle, x0)
    spring = np.empty(4)
    for axle, (left, right) in ((vehicle.suspension.front, (0, 1)), (vehicle.suspension.rear, (2, 3))):
        spring[[left, right]] = axle.wheel_rate_spring.derivative(x0[3:][[left, right]])
    x_arm = positions[:, 0]
    y_arm = positions[:, 1]
    arb = np.zeros((4, 4))
    for axle, (left, right) in ((vehicle.suspension.front, (0, 1)), (vehicle.suspension.rear, (2, 3))):
        half = 0.5 * axle.arb_wheel_rate
        arb[left, left] += half
        arb[left, right] -= half
        arb[right, left] -= half
        arb[right, right] += half
    matrix = np.zeros((7, 7))
    matrix[0, 0] = float(np.sum(tire))
    matrix[0, 1] = matrix[1, 0] = float(np.sum(y_arm * tire))
    matrix[0, 2] = matrix[2, 0] = -float(np.sum(x_arm * tire))
    matrix[1, 1] = float(np.sum(y_arm**2 * tire))
    matrix[1, 2] = matrix[2, 1] = -float(np.sum(y_arm * x_arm * tire))
    matrix[2, 2] = float(np.sum(x_arm**2 * tire))
    for index in range(4):
        matrix[0, 3 + index] = matrix[3 + index, 0] = -tire[index]
        matrix[1, 3 + index] = matrix[3 + index, 1] = -y_arm[index] * tire[index]
        matrix[2, 3 + index] = matrix[3 + index, 2] = x_arm[index] * tire[index]
    for row in range(4):
        for column in range(4):
            matrix[3 + row, 3 + column] = arb[row, column]
            if row == column:
                matrix[3 + row, 3 + column] += spring[row] + tire[row]
    return matrix


def mode_participation(mass, eigenvector):
    """Return the mass-weighted energy share of each coordinate of one mode."""

    share = (eigenvector**2) * np.diag(mass)
    return share / float(np.sum(share))


class VerticalModalGates(unittest.TestCase):
    """Modal gates: plan band, body-fixed hops, full-vehicle pin, closed form."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = undamped(compiled_vehicle())
        cls.q0, cls.tires = static_state(cls.vehicle)
        cls.x0 = np.asarray(cls.q0)[list(POSITION_ROWS)]
        cls.block = vertical_mass_block(cls.vehicle, cls.x0[3:])
        cls.stiffness = stiffness_matrix(cls.vehicle, cls.x0, cls.tires)
        cls.frequencies, cls.modes = modal_solution(cls.block, cls.stiffness)
        cls.hop_stiffness = body_fixed_hop_stiffness(cls.vehicle, cls.x0, cls.tires)
        cls.hop_frequencies = hop_frequencies(cls.vehicle, cls.hop_stiffness, cls.block)
        cls.closed_form = modal_solution(
            closed_form_mass(cls.vehicle), closed_form_stiffness(cls.vehicle, cls.x0)
        )

    def test_speed_zero_static_state_is_stationary(self):
        """The modal/energy operating point must be a genuine equilibrium."""

        np.testing.assert_allclose(generalized_forces(self.vehicle, self.x0, self.tires), 0.0, atol=1e-7)
        out = evaluate_vertical_dynamics(
            self.vehicle, VerticalState.from_vector(self.q0), np.zeros(4), 0.0, self.tires
        )
        np.testing.assert_allclose(np.asarray(out.rates)[list(RATE_ROWS)], 0.0, atol=1e-9)
        self.assertTrue(np.all(out.normal_loads > 0.0))
        # The stiffness must be symmetric: the conservative vertical forces are
        # the gradient of a potential, which the closed-form check relies on.
        self.assertLess(float(np.max(np.abs(self.stiffness - self.stiffness.T))), 1e-3)

    def test_heave_dominated_mode_lies_in_the_plan_band(self):
        """Gate (i): heave in 3.4-4.7 Hz, identified from the eigenvector."""

        heave_index = int(np.argmax([np.abs(mode[0]) for mode in self.modes.T]))
        heave_frequency = float(self.frequencies[heave_index])
        share = mode_participation(self.block, self.modes[:, heave_index])
        self.assertGreater(share[0], 0.8, share)
        self.assertGreater(heave_frequency, 3.4)
        self.assertLess(heave_frequency, 4.7)

    def test_body_fixed_hop_set_matches_the_plan_quarter_car_reference(self):
        """Gate (ii): the four body-fixed hubs against the quarter-car formula.

        The reference is ``f = (1/2*pi)*sqrt((k_w + k_t)/m_u)`` with the spring
        tangent ``k_w`` at jounce 0 and the tangent tire stiffness ``k_t`` at the
        static load. The rear ARB adds ``k_arb`` to the antisymmetric rear mode
        because ``corner_forces`` uses ``0.5*k_arb*(z_L - z_R)``: in the modal
        coordinate ``z = (d, -d)`` the two half-rates add to one full rate.
        The plan quotes 17.61 Hz (front) and 16.63 Hz (rear), which are exactly
        the no-ARB values; the ARB raises the rear warp mode.
        """

        tire = tangent_tire_stiffness(self.vehicle, self.x0)
        masses = unsprung_masses(self.vehicle)
        # Independent pin of the ARB input the warp expectation rests on:
        # 120 lbf/in rear, zero front (Task 4 DSS row 18/19 override, 1 lbf/in
        # = 4.4482216152605 N / 0.0254 m).
        self.assertAlmostEqual(self.vehicle.suspension.rear.arb_wheel_rate, 120.0 * 4.4482216152605 / 0.0254, delta=1.0)
        self.assertEqual(self.vehicle.suspension.front.arb_wheel_rate, 0.0)
        spring = np.empty(4)
        for axle, (left, right) in ((self.vehicle.suspension.front, (0, 1)), (self.vehicle.suspension.rear, (2, 3))):
            spring[[left, right]] = axle.wheel_rate_spring.derivative(self.x0[3:][[left, right]])
        quarter = np.sqrt((spring + tire) / masses) / (2.0 * np.pi)
        rear_warp = float(
            np.sqrt(
                (spring[2] + tire[2] + self.vehicle.suspension.rear.arb_wheel_rate) / masses[2]
            )
            / (2.0 * np.pi)
        )
        expected = np.sort(np.array([quarter[0], quarter[1], quarter[2], rear_warp]))
        measured = np.sort(np.asarray(self.hop_frequencies, dtype=float))
        message = f"measured {measured} expected {expected}"
        for value, reference in zip(measured, expected):
            self.assertLess(abs(value - reference) / reference, 0.03, message)
        # The plan's own numbers must be inside the measured set to 3 %.
        self.assertTrue(np.any(np.abs(measured - 17.61) / 17.61 < 0.03), message)
        self.assertTrue(np.any(np.abs(measured - 16.63) / 16.63 < 0.03), message)
        # The plan's front value appears twice (both front hubs), the rear pair
        # is symmetric 16.63 and ARB-raised 17.82.
        self.assertLess(abs(measured[0] - expected[0]) / expected[0], 1e-6, message)

    def test_full_vehicle_undamped_modes_pin_the_parameter_implied_values(self):
        """Gate (iii): the seven undamped modes, assigned by eigenvector.

        Diagnosis pin (ledger): heave 3.83, pitch 7.30, roll 9.90 Hz and hop
        modes 17.18, 17.71, 19.47, 23.97 Hz, within 1 %. The three body modes
        are the modes with the largest heave, pitch, and roll participation;
        the remaining four are the hops. The assignment is exercised, not
        assumed: the body modes must own their coordinate and the four hops
        must be dominated by jounce motion.
        """

        body_pin = {"heave": 3.83, "pitch": 7.30, "roll": 9.90}
        hop_pin = np.array([17.18, 17.71, 19.47, 23.97])
        coordinate = {"heave": 0, "pitch": 2, "roll": 1}
        participation = {
            name: np.array(
                [mode_participation(self.block, self.modes[:, mode])[index] for mode in range(7)]
            )
            for name, index in coordinate.items()
        }
        assigned = {name: int(np.argmax(values)) for name, values in participation.items()}
        self.assertEqual(len(set(assigned.values())), 3, assigned)
        for name, mode in assigned.items():
            with self.subTest(mode=name):
                self.assertGreater(float(participation[name][mode]), 0.8, participation[name])
                measured = float(self.frequencies[mode])
                self.assertLess(
                    abs(measured - body_pin[name]) / body_pin[name],
                    0.01,
                    f"{name} mode {measured:.4f} Hz vs pin {body_pin[name]}",
                )
        hop_modes = [mode for mode in range(7) if mode not in assigned.values()]
        for mode in hop_modes:
            with self.subTest(hop_mode=mode):
                share = mode_participation(self.block, self.modes[:, mode])
                self.assertGreater(float(np.sum(share[3:])), 0.5, share)
        measured_hops = np.sort([float(self.frequencies[mode]) for mode in hop_modes])
        for value, reference in zip(measured_hops, np.sort(hop_pin)):
            self.assertLess(
                abs(value - reference) / reference,
                0.01,
                f"hop set {measured_hops} vs pin {hop_pin}",
            )

    def test_pitch_mode_is_outside_the_plan_ride_frequency_band(self):
        """Gate (iv): the plan's 3.4-4.7 Hz band does NOT contain pitch.

        This is a documented deviation, not a pass: the ledger ruling keeps the
        user-reported 0.386 m pitch radius (D5 inertia provenance), which puts
        the pitch mode near 7.3 Hz. The assertion makes the deviation visible.
        """

        participation = np.array(
            [mode_participation(self.block, self.modes[:, mode])[2] for mode in range(7)]
        )
        pitch_mode = int(np.argmax(participation))
        pitch_frequency = float(self.frequencies[pitch_mode])
        self.assertGreater(pitch_frequency, 4.7, "the pitch mode entered the plan band; revisit the ruling")
        self.assertTrue(3.4 <= float(self.frequencies[0]) <= 4.7)

    def test_closed_form_mass_and_stiffness_reproduce_the_linearization(self):
        """Gate (v): independent closed-form M and K against the evaluator FD."""

        closed_frequencies, _ = self.closed_form
        with self.subTest(measured=np.round(np.sort(self.frequencies), 5).tolist(),
                          closed_form=np.round(np.sort(closed_frequencies), 5).tolist()):
            np.testing.assert_allclose(
                np.sort(closed_frequencies), np.sort(self.frequencies), rtol=1e-3
            )
        np.testing.assert_allclose(
            closed_form_mass(self.vehicle), self.block, rtol=1e-12, atol=1e-12
        )
        np.testing.assert_allclose(
            closed_form_stiffness(self.vehicle, self.x0), self.stiffness, rtol=1e-6, atol=1.0
        )


class VerticalInertanceGates(unittest.TestCase):
    """The ruled hub-to-chassis-point inerter: hops slow, heave barely moves."""

    FRONT_INERTANCE = 3.0
    REAR_INERTANCE = 5.0
    #: A genuinely small inertance for the "heave unchanged" gate.
    SMALL_FRONT_INERTANCE = 0.3
    SMALL_REAR_INERTANCE = 0.5

    @classmethod
    def setUpClass(cls):
        cls.vehicle = undamped(compiled_vehicle())
        cls.loaded = undamped(with_inertance(compiled_vehicle(), cls.FRONT_INERTANCE, cls.REAR_INERTANCE))
        cls.small = undamped(
            with_inertance(compiled_vehicle(), cls.SMALL_FRONT_INERTANCE, cls.SMALL_REAR_INERTANCE)
        )
        cls.q0, cls.tires = static_state(cls.vehicle)
        cls.x0 = np.asarray(cls.q0)[list(POSITION_ROWS)]
        cls.hop_stiffness = body_fixed_hop_stiffness(cls.vehicle, cls.x0, cls.tires)
        cls.block = vertical_mass_block(cls.vehicle, cls.x0[3:])
        cls.loaded_block = vertical_mass_block(cls.loaded, cls.x0[3:])
        cls.small_block = vertical_mass_block(cls.small, cls.x0[3:])
        cls.stiffness = stiffness_matrix(cls.vehicle, cls.x0, cls.tires)
        cls.frequencies, _ = modal_solution(cls.block, cls.stiffness)
        cls.loaded_frequencies, _ = modal_solution(cls.loaded_block, cls.stiffness)
        cls.small_frequencies, _ = modal_solution(cls.small_block, cls.stiffness)
        cls.hops, cls.hop_modes = modal_solution(np.diag(unsprung_masses(cls.vehicle)), cls.hop_stiffness)
        cls.loaded_hops = hop_frequencies(cls.vehicle, cls.hop_stiffness, cls.loaded_block)

    def _inertance_ratio(self, vehicle):
        """Return the per-corner ``(b/MR^2) / m_u`` ratios [1]."""

        suspension = vehicle.suspension
        inertance = np.array(
            [suspension.front.inertance] * 2 + [suspension.rear.inertance] * 2
        )
        ratio = np.array(
            [float(suspension.front.motion_ratio_damper(0.0))] * 2
            + [float(suspension.rear.motion_ratio_damper(0.0))] * 2
        )
        return inertance / ratio**2 / unsprung_masses(self.vehicle)

    def test_inertance_adds_b_over_mr_squared_to_the_jounce_diagonal(self):
        """The ruled topology touches only the four jounce diagonals."""

        difference = self.loaded_block - self.block
        expected = np.zeros((7, 7))
        suspension = self.loaded.suspension
        ratio = np.array(
            [float(suspension.front.motion_ratio_damper(0.0))] * 2
            + [float(suspension.rear.motion_ratio_damper(0.0))] * 2
        )
        inertance = np.array([self.FRONT_INERTANCE] * 2 + [self.REAR_INERTANCE] * 2)
        for index in range(4):
            expected[3 + index, 3 + index] = inertance[index] / ratio[index] ** 2
        np.testing.assert_allclose(difference, expected, rtol=1e-12, atol=1e-12)
        self.assertEqual(float(np.max(np.abs(difference[:3, :]))), 0.0)

    def test_small_inertance_leaves_the_undamped_heave_mode_unchanged(self):
        """b = 0.3 / 0.5 kg: the body mode moves by less than 0.2 %.

        The inerter does couple into the body mode because the heave mode has
        ~4.7 % of its energy in jounce motion, so the shift is proportional to
        b: 0.085 % here, 0.85 % at the brief's parenthetical 3 / 5 kg example.
        Both are asserted so that the example's size relative to the 0.2 %
        estimate is visible rather than silently adjusted.
        """

        small_shift = (self.small_frequencies[0] - self.frequencies[0]) / self.frequencies[0]
        large_shift = (self.loaded_frequencies[0] - self.frequencies[0]) / self.frequencies[0]
        self.assertLess(abs(small_shift), 2e-3, f"small-b heave shift {small_shift:.3e}")
        self.assertGreater(abs(small_shift), 0.0)
        # 3 / 5 kg is not "small" for this 0.2 % estimate: report the measured
        # value and the physical reason, do not loosen the gate above.
        self.assertGreater(abs(large_shift), 2e-3, f"3/5 kg heave shift {large_shift:.3e}")
        self.assertLess(abs(large_shift), 5e-2, f"3/5 kg heave shift {large_shift:.3e}")
        self.assertLess(self.loaded_frequencies[0], self.frequencies[0])  # hops slow the coupling

    def test_body_fixed_hop_frequencies_drop_by_the_inertance_mass_ratio(self):
        """Each hop frequency scales as sqrt(m_u / (m_u + b/MR^2)).

        The hop stiffness is block diagonal by axle, so every body-fixed hop
        mode lives on one axle and gains mass only on that axle's jounce
        diagonals; its frequency therefore scales by that axle's mass ratio
        with no mode mixing. The stiffness itself is unchanged: the inerter
        carries no static force and its relative velocity vanishes at rest.
        """

        # mass factor sqrt(1 + b/(MR^2 m_u)) = sqrt((m_u + b/MR^2)/m_u)
        mass_factor = np.sqrt(1.0 + self._inertance_ratio(self.loaded))
        predicted = []
        for mode in range(4):
            share = mode_participation(np.diag(unsprung_masses(self.vehicle)), self.hop_modes[:, mode])
            front = float(np.sum(share[:2]))
            rear = float(np.sum(share[2:]))
            # Each mode is confined to one axle (K_hop has no front-rear coupling).
            self.assertLess(min(front, rear), 1e-9, share)
            predicted.append(float(self.hops[mode]) / float(mass_factor[0] if front > rear else mass_factor[2]))
        np.testing.assert_allclose(
            np.sort(self.loaded_hops), np.sort(predicted), rtol=1e-9,
            err_msg=f"measured {np.sort(self.loaded_hops)} predicted {np.sort(predicted)}",
        )
        self.assertTrue(np.all(np.sort(self.loaded_hops) < np.sort(self.hops)))


def _quadrature_nodes(panels=24, nodes=8):
    """Return composite Gauss-Legendre nodes and weights on [0, 1]."""

    base_nodes, base_weights = np.polynomial.legendre.leggauss(nodes)
    edges = np.linspace(0.0, 1.0, panels + 1)
    times, weights = [], []
    for low, high in zip(edges[:-1], edges[1:]):
        middle, half = 0.5 * (low + high), 0.5 * (high - low)
        times.append(middle + half * base_nodes)
        weights.append(half * base_weights)
    return np.concatenate(times), np.concatenate(weights)


_QUAD_TIMES, _QUAD_WEIGHTS = _quadrature_nodes()


def suspension_forces(vehicle, jounce):
    """Return spring + ARB wheel forces [N] for a ``(n, 4)`` jounce array.

    Positive is a compression force: it pushes the hub down and the body up.
    The ARB term is ``0.5*k_arb*(z_L - z_R)`` on the left wheel, the same
    convention as ``corner_forces``.
    """

    forces = np.empty_like(jounce)
    for axle, (left, right) in ((vehicle.suspension.front, (0, 1)), (vehicle.suspension.rear, (2, 3))):
        forces[:, [left, right]] = axle.wheel_rate_spring(jounce[:, [left, right]])
        half = 0.5 * axle.arb_wheel_rate * (jounce[:, left] - jounce[:, right])
        forces[:, left] += half
        forces[:, right] -= half
    return forces


def vertical_potential(vehicle, x, x0):
    """Return ``U(x) - U(x0)`` for the seven vertical coordinates.

    ``U = -(m_s g h + sum m_ui g z_body,i) + sum m_ui g j_i
    + sum_i int F_susp,i dj_i + sum_i int N_i d(delta_i)``. Every term is built
    from the compiled curves (spring, ARB, tire elastic load) and the compiled
    geometry, and its gradient was checked against the evaluator's generalized
    forces, so this is an independent nonlinear invariant, not a quadratic
    approximation.
    """

    positions = sprung_wheel_positions(vehicle)
    masses = unsprung_masses(vehicle)
    heave, roll, pitch = (float(value) for value in x[:3])
    jounce = np.asarray(x[3:], dtype=float)
    heave0, roll0, pitch0 = (float(value) for value in x0[:3])
    jounce0 = np.asarray(x0[3:], dtype=float)
    body = heave + positions[:, 1] * roll - positions[:, 0] * pitch
    body0 = heave0 + positions[:, 1] * roll0 - positions[:, 0] * pitch0
    energy = -(vehicle.sprung_mass * GRAVITY * heave + float(np.sum(masses * GRAVITY * body)))
    energy += float(np.sum(masses * GRAVITY * jounce))
    base = -(vehicle.sprung_mass * GRAVITY * heave0 + float(np.sum(masses * GRAVITY * body0)))
    base += float(np.sum(masses * GRAVITY * jounce0))
    nodes = jounce0[None, :] + _QUAD_TIMES[:, None] * (jounce - jounce0)[None, :]
    energy += float(
        np.sum(_QUAD_WEIGHTS[:, None] * suspension_forces(vehicle, nodes) * (jounce - jounce0)[None, :])
    )
    deflection = body - jounce
    deflection0 = body0 - jounce0
    path = deflection0[None, :] + _QUAD_TIMES[:, None] * (deflection - deflection0)[None, :]
    load = np.asarray(tire_deflection_load(vehicle.planar.tire, path, 0.0, 0.0).load)
    energy += float(
        np.sum(_QUAD_WEIGHTS[:, None] * load * (deflection - deflection0)[None, :])
    )
    return energy - base


def integrate_vertical(vehicle, amplitude, duration, nonlinear):
    """Integrate the standalone vertical subsystem and return the drift measure.

    ``y = [heave, roll, pitch, jounce x4, their rates]``. The planar and wheel
    states are held at their equilibrium values and the four transient tire
    states are held at their steady rolling values, so the tangential tire
    forces vanish identically and the vertical subsystem is conservative.
    """

    q0, tires = static_state(vehicle)
    x0 = np.asarray(q0)[list(POSITION_ROWS)]
    block = vertical_mass_block(vehicle, x0[3:])
    stiffness = stiffness_matrix(vehicle, x0, tires)
    y0 = np.r_[x0 + amplitude * PERTURBATION_SHAPE, np.zeros(7)]

    def rhs(_time, y):
        out = evaluate_vertical_dynamics(vehicle, state_with(y[:7], y[7:]), np.zeros(4), 0.0, tires)
        return np.r_[y[7:], np.asarray(out.rates)[list(RATE_ROWS)]]

    def energy(y):
        dx = y[:7] - x0
        kinetic = 0.5 * float(y[7:] @ (block @ y[7:]))
        if nonlinear:
            return kinetic + vertical_potential(vehicle, y[:7], x0)
        return kinetic + 0.5 * float(dx @ (stiffness @ dx))

    solution = solve_ivp(rhs, (0.0, duration), y0, rtol=1e-10, atol=1e-12, method="RK45")
    self_energy = energy(y0)
    drifts = [abs(energy(solution.y[:, index]) - self_energy) / abs(self_energy)
              for index in range(solution.y.shape[1])]
    return max(drifts), solution, stiffness, block, x0, q0


@unittest.skipUnless(solve_ivp is not None, "SciPy is not available")
class VerticalEnergyGates(unittest.TestCase):
    """Energy conservation of the standalone 7-coordinate vertical subsystem."""

    AMPLITUDE = 1.0e-4

    @classmethod
    def setUpClass(cls):
        cls.vehicle = undamped(compiled_vehicle())
        cls.loaded = undamped(with_inertance(compiled_vehicle(), 3.0, 5.0))
        cls.tires = static_state(cls.vehicle)[1]
        cls.nonlinear, cls.solution, cls.stiffness, cls.block, cls.x0, cls.q0 = integrate_vertical(
            cls.vehicle, cls.AMPLITUDE, 2.0, nonlinear=True
        )
        cls.inertance, cls.loaded_solution, _, cls.loaded_block, _, _ = integrate_vertical(
            cls.loaded, cls.AMPLITUDE, 1.0, nonlinear=True
        )
        cls.quadratic = cls._quadratic_drift(cls.solution, cls.stiffness, cls.block, cls.x0)
        cls.linear = cls._linear_system_drift(cls.block, cls.stiffness, cls.AMPLITUDE)

    @staticmethod
    def _quadratic_drift(solution, stiffness, block, x0):
        """Largest relative drift of the quadratic form along the trajectory."""

        def quadratic(y):
            dx = y[:7] - x0
            return 0.5 * float(dx @ (stiffness @ dx)) + 0.5 * float(y[7:] @ (block @ y[7:]))

        base = quadratic(solution.y[:, 0])
        return max(
            abs(quadratic(solution.y[:, index]) - base) / abs(base)
            for index in range(solution.y.shape[1])
        )

    @staticmethod
    def _linear_system_drift(block, stiffness, amplitude):
        """Quadratic-energy drift of the linearized system ``M xddot = -K x``."""

        product = np.linalg.solve(block, stiffness)
        x0 = amplitude * PERTURBATION_SHAPE
        y0 = np.r_[x0, np.zeros(7)]

        def rhs(_time, y):
            return np.r_[y[7:], -(product @ y[:7])]

        solution = solve_ivp(rhs, (0.0, 2.0), y0, rtol=1e-12, atol=1e-14, method="RK45")

        def quadratic(y):
            return 0.5 * float(y[:7] @ (stiffness @ y[:7])) + 0.5 * float(y[7:] @ (block @ y[7:]))

        base = quadratic(y0)
        return max(
            abs(quadratic(solution.y[:, index]) - base) / abs(base)
            for index in range(solution.y.shape[1])
        )

    def test_holding_approach_freezes_the_planar_and_wheel_states(self):
        """The frozen planar/wheel states must give exactly zero slip forces."""

        out = evaluate_vertical_dynamics(
            self.vehicle, VerticalState.from_vector(self.q0), np.zeros(4), 0.0, self.tires
        )
        self.assertLess(float(np.max(np.abs(out.tire_forces[:, :2]))), 1e-9)
        self.assertTrue(np.all(out.normal_loads > 0.0))
        self.assertEqual(float(np.linalg.norm(np.asarray(self.q0)[[0, 1, 5, 9, 10, 11, 12]])), 0.0)

    def test_nonlinear_energy_is_conserved_without_inertance(self):
        """The gate: the analytic mechanical energy drifts by less than 1e-6."""

        self.assertLess(self.nonlinear, 1e-6, f"nonlinear drift {self.nonlinear:.3e}")
        self.assertGreater(abs(self.AMPLITUDE), 0.0)

    def test_nonlinear_energy_is_conserved_with_inertance(self):
        """Same gate with b = 3 / 5 kg, i.e. the inerter kinetic term included."""

        self.assertLess(self.inertance, 1e-6, f"nonlinear drift with inertance {self.inertance:.3e}")

    def test_quadratic_form_is_not_the_invariant_of_the_nonlinear_model(self):
        """Documents why the nonlinear potential is required at this amplitude.

        The quadratic form ``0.5 dx' K dx`` is conserved only in the linear
        limit: its measured relative drift along the same trajectory is far
        above 1e-6, while the nonlinear energy above is below it. Shrinking the
        amplitude shrinks the quadratic drift linearly, so the gate uses the
        nonlinear invariant rather than a tolerance that hides the difference.
        """

        self.assertGreater(self.quadratic, 1e-5, f"quadratic drift {self.quadratic:.3e}")

    def test_linearized_system_quadratic_energy_is_conserved(self):
        """The quadratic form is the invariant of the linearized system."""

        self.assertLess(self.linear, 1e-9, f"linear-system drift {self.linear:.3e}")


def _relative_error(measured, reference, floor):
    """Return the largest relative difference with an absolute floor."""

    measured = np.asarray(measured, dtype=float)
    reference = np.asarray(reference, dtype=float)
    return float(np.max(np.abs(measured - reference) / np.maximum(np.abs(reference), floor)))


def _steady_vertical_state(vehicle, speed, sideslip, kappa, steering=0.0, torques=None):
    """Return ``(state, tires, out)`` at the forced steady vertical state.

    The planar velocities and wheel speeds are prescribed; the seven vertical
    coordinates are solved to zero generalized force with the transient tire
    states held at their steady rolling values, and the loads are iterated to a
    fixed point because the loaded radius (hence the slip) depends on them.
    """

    planar = vehicle.planar
    torques = np.zeros(4) if torques is None else np.asarray(torques, dtype=float)
    q = solve_static_equilibrium(vehicle).as_vector()
    q[0], q[1] = speed, sideslip
    positions = wheel_positions(planar)
    loads = evaluate_normal_loads(planar, float(np.hypot(speed, sideslip)), 0.0, 0.0)

    def tire_states(current_loads):
        radii = loaded_radius(planar.tire, current_loads)
        q[9:13] = (1.0 + kappa) * speed / radii
        angles = wheel_steering_angles(steering, 4)
        longitudinal, lateral = contact_velocities(
            PlanarState(q[0], q[1], q[5], q[9:13]), positions, angles
        )
        slip_ratio, slip_angle = slip_quantities(q[9:13], radii, longitudinal, lateral, 1.0)
        return tuple(
            steady_rolling_state(
                planar.tire, vehicle.transient_tire, float(slip_angle[index]),
                float(slip_ratio[index]), float(current_loads[index])
            )
            for index in range(4)
        )

    tires = tire_states(loads)
    for _ in range(40):
        x = q[list(POSITION_ROWS)].copy()

        def residual(trial):
            # The full state is carried through the trial, so the aero speed and
            # the wheel speeds stay prescribed while the vertical coordinates move.
            local = q.copy()
            local[list(POSITION_ROWS)] = trial
            return np.asarray(
                evaluate_vertical_dynamics(
                    vehicle, VerticalState.from_vector(local), torques, steering, tires
                ).generalized_forces
            )

        for _ in range(50):
            value = residual(x)
            if np.linalg.norm(value, ord=np.inf) < 1e-11:
                break
            jacobian = np.empty((7, 7))
            for column in range(7):
                step = 1e-7 * (1.0 + abs(x[column]))
                plus, minus = x.copy(), x.copy()
                plus[column] += step
                minus[column] -= step
                jacobian[:, column] = (residual(plus) - residual(minus)) / (2.0 * step)
            delta = np.linalg.lstsq(jacobian, -value, rcond=None)[0]
            scale = 1.0
            while scale > 1e-5 and np.linalg.norm(residual(x + scale * delta)) >= np.linalg.norm(value):
                scale *= 0.5
            x = x + scale * delta
        q[list(POSITION_ROWS)] = x
        updated = evaluate_vertical_dynamics(
            vehicle, VerticalState.from_vector(q), torques, steering, tires
        ).normal_loads
        converged = np.max(np.abs(updated - loads)) < 1e-10
        loads = updated
        if converged:
            # ``tires`` matches the loads the positions were solved against.
            break
        tires = tire_states(loads)
    state = VerticalState.from_vector(q)
    out = evaluate_vertical_dynamics(vehicle, state, torques, steering, tires)
    return state, tires, out


def _planar_evaluation(vehicle, speed, sideslip, kappa, lltd, steering=0.0, torques=None):
    """Return the planar evaluator's result at the same forced state."""

    planar = vehicle.planar if lltd is None else replace(
        vehicle.planar, lateral_load_transfer_distribution=lltd
    )
    torques = np.zeros(4) if torques is None else np.asarray(torques, dtype=float)
    loads = evaluate_normal_loads(planar, float(np.hypot(speed, sideslip)), 0.0, 0.0)
    for _ in range(80):
        radii = loaded_radius(planar.tire, loads)
        state = PlanarState(speed, sideslip, 0.0, (1.0 + kappa) * speed / radii)
        evaluation = evaluate_planar_dynamics(planar, state, torques, steering)
        updated = evaluation.normal_loads
        if np.max(np.abs(updated - loads)) < 1e-13:
            break
        loads = updated
    return evaluate_planar_dynamics(planar, state, torques, steering)


class VerticalPlanarReductionGates(unittest.TestCase):
    """Planar reduction of the frozen vertical model against ``planar_7dof``.

    The reduced oracle is the plan's: springs stiffened 1e4x about their
    preload, the vertical coordinates solved to their forced steady state (so
    the vertical and wheel-rate contribution to the planar rates is zero), and
    a test-only LLTD equal to the elastic split that the stiff model produces.
    The vertical evaluator already uses the total 287 kg horizontal mass, the
    static loaded-radius total CG height, and the planar model's yaw inertia,
    so no yaw-inertia substitution is needed (unlike the plan's expectation);
    the CG-height consistency is asserted below.

    The aero drag is zeroed for the reduction comparison because the two load
    rules disagree whenever a horizontal force is not carried by the tires:
    the planar quasi-static rule books the longitudinal transfer as
    ``m*a*h_cg/L``, while the vertical model books the actual couple. The
    measured size and the rigid-body oracle for that difference are in
    ``test_aero_drag_couple_explains_the_planar_reduction_gap``.
    """

    SPEED = 30.0
    SIDESLIP = 0.5

    @classmethod
    def setUpClass(cls):
        base = stiff_springs(compiled_vehicle())
        cls.vehicle = replace(
            base,
            planar=replace(base.planar, aero=replace(base.planar.aero, downforce_area=0.0, drag_area=0.0)),
        )
        cls.cases = {}
        for name, kappa in (("coast", 0.0), ("drive", 0.01), ("brake", -0.01)):
            state, tires, out = _steady_vertical_state(
                cls.vehicle, cls.SPEED, cls.SIDESLIP, kappa
            )
            loads = out.normal_loads
            front_transfer = 0.5 * (loads[0] - loads[1])
            rear_transfer = 0.5 * (loads[2] - loads[3])
            lltd = front_transfer / (front_transfer + rear_transfer)
            evaluation = _planar_evaluation(cls.vehicle, cls.SPEED, cls.SIDESLIP, kappa, lltd)
            cls.cases[name] = (kappa, loads, lltd, out, evaluation)
        # Aero-enabled coasting states for the drag-couple oracle.
        cls.aero_vehicle = stiff_springs(compiled_vehicle())
        cls.aero_states = {}
        for speed in (5.0, 30.0):
            _, _, out = _steady_vertical_state(cls.aero_vehicle, speed, 0.0, 0.0)
            cls.aero_states[speed] = out

    def _comparison(self, name):
        kappa, loads, lltd, out, evaluation = self.cases[name]
        yaw_rate = 0.0
        measured = {
            "longitudinal acceleration": float(out.rates[0]) - yaw_rate * self.SIDESLIP,
            "lateral acceleration": float(out.rates[1]) + yaw_rate * self.SPEED,
            "yaw acceleration": float(out.rates[5]),
            "wheel accelerations": np.asarray(out.rates[9:13]),
            "tire forces": np.asarray(out.tire_forces),
            "normal loads": np.asarray(loads),
        }
        reference = {
            "longitudinal acceleration": evaluation.longitudinal_acceleration,
            "lateral acceleration": evaluation.lateral_acceleration,
            "yaw acceleration": evaluation.yaw_acceleration,
            "wheel accelerations": np.asarray(evaluation.wheel_accelerations),
            "tire forces": np.column_stack(
                (evaluation.longitudinal_forces, evaluation.lateral_forces, evaluation.aligning_moments)
            ),
            "normal loads": np.asarray(evaluation.normal_loads),
        }
        floors = {
            "longitudinal acceleration": 1e-3,
            "lateral acceleration": 1e-3,
            "yaw acceleration": 1e-4,
            "wheel accelerations": 1e-2,
            "tire forces": 1.0,
            "normal loads": 1.0,
        }
        return measured, reference, floors, lltd, loads, evaluation

    def test_stiff_reduction_state_is_consistent(self):
        """The reduced oracle's own invariants before the rate comparison."""

        # Total CG height from the sprung/unsprung split matches the planar value.
        total = (
            compiled_vehicle().sprung_mass * compiled_vehicle().sprung_cg_height
            + float(np.sum(unsprung_masses(compiled_vehicle()))) * compiled_vehicle().unsprung_cg_height
        ) / compiled_vehicle().planar.total_mass
        self.assertAlmostEqual(total, compiled_vehicle().planar.cg_height, delta=1e-9)
        for name, (_, loads, lltd, out, evaluation) in self.cases.items():
            with self.subTest(case=name):
                # Wheel-lift saturation in the planar model is inactive: the
                # front transfer stays below half the front axle load.
                self.assertLess(abs(0.5 * (loads[0] - loads[1])), 0.5 * float(np.sum(loads[:2])))
                self.assertGreater(lltd, 0.0)
                self.assertLess(lltd, 1.0)
                self.assertTrue(evaluation.load_converged)
                # The vertical accelerations are the frozen coordinates.
                np.testing.assert_allclose(
                    np.asarray(out.rates)[[2, 3, 4, 17, 18, 19, 20]], 0.0, atol=1e-6
                )

    def test_planar_rates_loads_and_forces_match_the_planar_evaluator(self):
        """The plan's reduced oracle at 1e-2 relative, with the fitted LLTD."""

        for name in ("coast", "drive", "brake"):
            measured, reference, floors, _, _, _ = self._comparison(name)
            for key in measured:
                with self.subTest(case=name, quantity=key):
                    error = _relative_error(measured[key], reference[key], floors[key])
                    self.assertLess(
                        error,
                        1e-2,
                        f"{name} {key}: measured {np.ravel(measured[key])} "
                        f"reference {np.ravel(reference[key])} relative error {error:.3e}",
                    )

    def test_aero_drag_couple_explains_the_planar_reduction_gap(self):
        """Why the aero-enabled comparison cannot reach 1e-2: the drag couple.

        Coasting, the tire longitudinal forces vanish and only the aero drag
        decelerates the car. The vertical model applies the drag at the sprung
        CG, so the rigid-body pitch balance about the total CG shifts load
        rearward by ``D*(h_s - h_cg)/L``; the planar quasi-static rule instead
        books ``m*a*h_cg/L = D*h_cg/L`` to the front. The measured front-axle
        load therefore differs by ``D*h_s/L`` (8 % at 30 m/s), and because the
        yaw moment is a near-cancellation of the front and rear lateral forces
        a ~0.4 % axle-load difference is amplified into tens of percent of the
        yaw acceleration. A second, unrelated offset is the gravity constant:
        the planar quasi-static law uses ``core.conventions.GRAVITY = 9.81``
        while the vertical model hard-codes 9.80665, i.e. a constant
        ``m_total*f_front*(9.81 - 9.80665)`` on the front axle. The oracle below
        pins the vertical model to the rigid-body value and reproduces the gap
        from those two terms rather than loosening the 1e-2 gate.
        """

        vehicle = self.aero_vehicle
        planar = vehicle.planar
        static_q0, static_tires = static_state(vehicle)
        static = evaluate_vertical_dynamics(
            vehicle, VerticalState.from_vector(static_q0), np.zeros(4), 0.0, static_tires
        ).normal_loads
        gravity_offset = (
            planar.total_mass * planar.front_mass_fraction * (PLANAR_GRAVITY - GRAVITY)
        )
        for speed, out in self.aero_states.items():
            with self.subTest(speed=speed):
                downforce, drag = aerodynamic_forces(planar, speed)
                expected = (
                    float(np.sum(static[:2]))
                    + downforce * planar.aero.aero_balance
                    - drag * (vehicle.sprung_cg_height - planar.cg_height) / planar.wheelbase
                )
                measured = float(np.sum(out.normal_loads[:2]))
                self.assertAlmostEqual(measured, expected, delta=0.05)
                reference = evaluate_normal_loads(
                    planar, speed, float(out.body_accelerations[0]), 0.0
                )
                gap = float(np.sum(reference[:2])) - measured
                predicted = drag * vehicle.sprung_cg_height / planar.wheelbase + gravity_offset
                self.assertGreater(gap, 1.0)  # the gap is real at both speeds
                self.assertAlmostEqual(gap, predicted, delta=0.05 * predicted)
                if speed >= 30.0:
                    self.assertGreater(gap / measured, 1e-2)


if __name__ == "__main__":
    unittest.main()
