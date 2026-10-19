"""Compile the vertical 14-DOF (``14dof-simple``) vehicle parameters.

The 14 mechanical degrees of freedom are the 7 planar ones plus heave, roll,
pitch, and four wheel-hub vertical motions. This module compiles parameters and
provides the 21-state mechanics evaluator.

The compiled value composes the planar 7-DOF vehicle (field ``planar``) and adds:

- the per-corner unsprung masses, the sprung-body radii of gyration, and the tire
  vertical damping (assembly parameters);
- the corner suspension, the front and rear dampers, and the transient tire
  coefficients (components);
- derived sprung-body values: ``sprung_mass``, ``sprung_cg_height``,
  ``sprung_front_fraction``, ``roll_inertia``, and ``pitch_inertia``.

Derivation. The planar vehicle gives the total mass, the total CG height and the
front static fraction. Each wheel assembly (hub, upright, wheel, tire, and the
unsprung share of the suspension) has its CG at the free tire radius, ``r_1`` of
the steady tire, so the sprung CG height and front fraction follow from moment
balances about the ground and the front axle::

    m_s       = m_total - 2 * (m_uf + m_ur)
    h_s       = (m_total * h - 2 * (m_uf + m_ur) * r_free) / m_s
    f_s_front = (m_total * f_front - 2 * m_uf) / m_s

``m_uf`` and ``m_ur`` are the unsprung mass of one front and one rear corner.

The tire vertical law requires the steady tire to have zero lateral loaded-radius
corrections, and its relaxation lengths to stay positive up to the largest
tire load that the model supports (``TIRE_OPERATING_MAXIMUM_LOAD``).
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simplified_vehicle_models.compiler.adapters import ResolutionResult
from simplified_vehicle_models.compiler.capabilities import (
    SUSPENSION_FORCES,
    TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.formulations.planar_7dof import (
    PLANAR_7DOF_REQUESTED_CAPABILITIES,
    PLANAR_7DOF_VEHICLE_PARAMETER_UNITS,
    PlanarState, contact_velocities, wheel_positions, wheel_steering_angles,
    slip_quantities, rotate_tire_forces_to_body, yaw_moment_of, aerodynamic_forces,
    _capability_value,
    compile_planar_7dof_vehicle,
)
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.suspension.network import CompiledSuspension, NetworkForces
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.unitire_transient import (
    TransientTireParameters,
    TransientTireState,
    check_relaxation_lengths,
    evaluate_transient_unitire,
)
from simplified_vehicle_models.tires.vertical import (
    require_exact_inverse_supported,
    tire_deflection_load,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import require_positive, required_parameters

if TYPE_CHECKING:
    from simplified_vehicle_models.compiler.manifest import CompileManifest

import numpy as np


VERTICAL_14DOF_REQUESTED_CAPABILITIES: tuple[str, ...] = (
    *PLANAR_7DOF_REQUESTED_CAPABILITIES,
    SUSPENSION_FORCES,
    TRANSIENT_COMBINED_SLIP,
)

#: Assembly parameters that only the vertical model consumes.
VERTICAL_14DOF_ONLY_PARAMETER_UNITS: dict[str, str] = {
    "unsprung_mass_front": "kg",
    "unsprung_mass_rear": "kg",
    "roll_radius_of_gyration": "m",
    "pitch_radius_of_gyration": "m",
    "tire_vertical_damping": "N*s/m",
}

VERTICAL_14DOF_VEHICLE_PARAMETER_UNITS: dict[str, str] = {
    **PLANAR_7DOF_VEHICLE_PARAMETER_UNITS,
    **VERTICAL_14DOF_ONLY_PARAMETER_UNITS,
}

#: Largest tire load [N] over which the transient relaxation lengths must stay positive.
TIRE_OPERATING_MAXIMUM_LOAD = 3000.0


@dataclass(frozen=True, slots=True)
class VerticalVehicleParameters:
    """Store the compiled parameters of the vertical 14-DOF vehicle."""

    planar: PlanarVehicleParameters
    unsprung_mass_front: float
    unsprung_mass_rear: float
    roll_radius_of_gyration: float
    pitch_radius_of_gyration: float
    tire_vertical_damping: float
    suspension: CompiledSuspension
    transient_tire: TransientTireParameters
    sprung_mass: float
    sprung_cg_height: float
    sprung_front_fraction: float
    roll_inertia: float
    pitch_inertia: float
    manifest: CompileManifest | None = field(
        default=None,
        compare=False,
        repr=False,
        hash=False,
    )

    @property
    def unsprung_cg_height(self) -> float:
        """Return the unsprung CG height: the free tire radius."""

        return self.planar.tire.r_1

    @property
    def total_unsprung_mass(self) -> float:
        """Return the mass of the four wheel assemblies."""

        return 2.0 * (self.unsprung_mass_front + self.unsprung_mass_rear)

    def with_manifest(self, manifest: CompileManifest) -> VerticalVehicleParameters:
        """Return a copy that holds the manifest here and in the composed planar model."""

        return dataclasses.replace(
            self,
            planar=dataclasses.replace(self.planar, manifest=manifest),
            manifest=manifest,
        )


def compile_vertical_14dof_vehicle(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
) -> VerticalVehicleParameters:
    """Compile one assembly for the vertical 14-DOF formulation."""

    planar = compile_planar_7dof_vehicle(assembly, resolutions)
    values, _ = required_parameters(assembly, VERTICAL_14DOF_ONLY_PARAMETER_UNITS)
    require_positive(assembly, values, *VERTICAL_14DOF_ONLY_PARAMETER_UNITS)

    suspension = _capability_value(assembly, resolutions, SUSPENSION_FORCES, CompiledSuspension)
    transient = _capability_value(
        assembly, resolutions, TRANSIENT_COMBINED_SLIP, TransientTireParameters
    )

    try:
        require_exact_inverse_supported(planar.tire)
        check_relaxation_lengths(planar.tire, transient, TIRE_OPERATING_MAXIMUM_LOAD)
    except ValueError as error:
        raise ValueError(f"{assembly.path}: {error}") from error

    unsprung = 2.0 * (values["unsprung_mass_front"] + values["unsprung_mass_rear"])
    sprung_mass = planar.total_mass - unsprung
    if not sprung_mass > 0.0:
        raise ValueError(
            f"{assembly.path}: the four unsprung masses ({unsprung} kg) must be "
            f"smaller than total_mass ({planar.total_mass} kg)"
        )
    sprung_cg_height = (
        planar.total_mass * planar.cg_height - unsprung * planar.tire.r_1
    ) / sprung_mass
    sprung_front_fraction = (
        planar.total_mass * planar.front_mass_fraction
        - 2.0 * values["unsprung_mass_front"]
    ) / sprung_mass
    if not sprung_cg_height > 0.0:
        raise ValueError(
            f"{assembly.path}: the derived sprung CG height {sprung_cg_height} m "
            "must be positive"
        )
    if not 0.0 < sprung_front_fraction < 1.0:
        raise ValueError(
            f"{assembly.path}: the derived sprung front fraction "
            f"{sprung_front_fraction} must lie between zero and one"
        )
    return VerticalVehicleParameters(
        planar=planar,
        unsprung_mass_front=values["unsprung_mass_front"],
        unsprung_mass_rear=values["unsprung_mass_rear"],
        roll_radius_of_gyration=values["roll_radius_of_gyration"],
        pitch_radius_of_gyration=values["pitch_radius_of_gyration"],
        tire_vertical_damping=values["tire_vertical_damping"],
        suspension=suspension,
        transient_tire=transient,
        sprung_mass=sprung_mass,
        sprung_cg_height=sprung_cg_height,
        sprung_front_fraction=sprung_front_fraction,
        roll_inertia=sprung_mass * values["roll_radius_of_gyration"] ** 2,
        pitch_inertia=sprung_mass * values["pitch_radius_of_gyration"] ** 2,
    )


@dataclass(frozen=True, slots=True)
class VerticalState:
    v_x: float = 0.0
    v_y: float = 0.0
    v_z: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    heave: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    wheel_speed_fl: float = 0.0
    wheel_speed_fr: float = 0.0
    wheel_speed_rl: float = 0.0
    wheel_speed_rr: float = 0.0
    jounce_fl: float = 0.0
    jounce_fr: float = 0.0
    jounce_rl: float = 0.0
    jounce_rr: float = 0.0
    jounce_rate_fl: float = 0.0
    jounce_rate_fr: float = 0.0
    jounce_rate_rl: float = 0.0
    jounce_rate_rr: float = 0.0

    def as_vector(self) -> np.ndarray:
        return np.asarray(dataclasses.astuple(self), dtype=float)

    @classmethod
    def from_vector(cls, values) -> "VerticalState":
        vector = np.asarray(values, dtype=float)
        if vector.shape != (21,) or not np.all(np.isfinite(vector)):
            raise ValueError("vertical state must contain 21 finite values")
        return cls(*map(float, vector))


#: Indices (in the 21-value state/rate layout) of the seven accelerated coordinates:
#: heave, roll, pitch (``v_z``, ``roll_rate``, ``pitch_rate`` rates) and the four
#: jounce accelerations. Every other row of the mechanical block is kinematic or
#: a planar/wheel-speed equation with unit mass.
VERTICAL_ACCELERATION_INDEX: tuple[int, ...] = (2, 3, 4, 17, 18, 19, 20)

GRAVITY = 9.80665


@dataclass(frozen=True, slots=True)
class VerticalEvaluation:
    """Result of one evaluation of the 21-value mechanical block.

    ``rates`` always holds the physical, already-solved time derivatives of the
    21 mechanical state values (accelerations for ``v_z``, ``roll_rate``,
    ``pitch_rate`` and the jounce rates).

    ``mass_matrix`` / ``forcing`` describe the same solution as a linear system
    ``mass_matrix @ rates = forcing`` suitable for ``RateSystem(forcing,
    mass_matrix).solve()``:

    - ``mass_matrix is None`` (all inertance zero): ``forcing`` is identical to
      ``rates`` and ``RateSystem(out.forcing).solve()`` returns ``rates``. The
      coupled physical sprung/unsprung mass block ``M0`` was already solved
      inside the evaluator.
    - otherwise ``mass_matrix`` is the 21x21 physical block: identity on every
      row except ``VERTICAL_ACCELERATION_INDEX``, where it holds the 7x7 matrix
      ``M0 + diag(0, 0, 0, b_i/MR_i^2)`` (kg, kg*m, kg*m^2). ``forcing`` is the
      generalized force on those rows (N, N*m, N, including the
      variable-motion-ratio inertance velocity term) and equals ``rates`` on
      all other rows. Solving ``RateSystem(out.forcing, out.mass_matrix)``
      recovers ``rates``; do not solve ``rates`` again.

    ``generalized_forces`` is the 7-vector Q for [heave (+down), roll
    (right-down), pitch (nose-up), jounce x4 (+compression)] before any
    inertance velocity term, including prescribed steering-lift acceleration;
    it is the forcing that ``M0 @ a = Q`` solves when inertance is zero.
    """

    rates: np.ndarray
    mass_matrix: np.ndarray | None
    tire_state_rates: np.ndarray
    normal_loads: np.ndarray
    tire_forces: np.ndarray
    corner_forces: NetworkForces | None
    lifted: np.ndarray
    out_of_range_count: int
    body_accelerations: np.ndarray
    forcing: np.ndarray | None = None
    generalized_forces: np.ndarray | None = None


def unsprung_masses(vehicle) -> np.ndarray:
    """Return the four per-corner unsprung masses in FL, FR, RL, RR order."""

    return np.array([vehicle.unsprung_mass_front] * 2 + [vehicle.unsprung_mass_rear] * 2,
                    dtype=float)


def axle_drive_shares(vehicle) -> tuple[float, float]:
    """Return the ``(front, rear)`` driven-axle shares of the drive force.

    The anti-squat percentage is referenced to the driven axle's share of the
    drive force, so the share comes from the transmission's front torque
    fraction. The compiled IEM24 model drives the rear axle only, which gives
    ``(0.0, 1.0)``. A drivetrain that does not expose a transmission is treated
    as rear-driven, the historic default.
    """

    transmission = getattr(getattr(vehicle.planar.drivetrain, "drivetrain", None),
                          "transmission", None)
    front = float(getattr(transmission, "front_torque_fraction", 0.0))
    return front, 1.0 - front


def sprung_cg_offset_x(vehicle) -> float:
    """Return the forward distance [m] of the sprung CG ahead of the total CG."""

    p = vehicle.planar
    return p.front_axle_to_cg - (1.0 - vehicle.sprung_front_fraction) * p.wheelbase


def sprung_wheel_positions(vehicle) -> np.ndarray:
    """Return wheel (x, y) positions relative to the SPRUNG CG, shape (4, 2).

    Heave is the vertical motion of the sprung CG and pitch rotates about it, so
    all vertical body-point kinematics use these arms; the yaw/planar equations
    keep the total-CG ``wheel_positions``.
    """

    pos = np.array(wheel_positions(vehicle.planar), dtype=float)
    pos[:, 0] -= sprung_cg_offset_x(vehicle)
    return pos


def vertical_mass_block(vehicle, jounce) -> np.ndarray:
    """Return the physical 7x7 mass matrix of [heave, roll, pitch, jounce x4].

    ``M = diag(m_s, I_roll, I_pitch, 0, 0, 0, 0) + sum_i m_ui c_i c_i^T
    + diag(0, 0, 0, b_i/MR_i^2)`` with ``c_i = [1, y_i, -x_i, -e_i]`` the hub
    downward-velocity coefficients (``x_i`` measured from the sprung CG, ``e_i`` the
    unit vector of jounce ``i``). The inerter is an ideal two-terminal device between
    the hub and a chassis point whose relative velocity is ``jounce_rate / MR``, so
    it adds ``b/MR^2`` only to the jounce diagonal and nothing couples the body.
    """

    z = np.asarray(jounce, dtype=float)
    pos = sprung_wheel_positions(vehicle)
    block = np.zeros((7, 7))
    block[0, 0] = vehicle.sprung_mass
    block[1, 1] = vehicle.roll_inertia
    block[2, 2] = vehicle.pitch_inertia
    um = unsprung_masses(vehicle)
    for i in range(4):
        c = np.zeros(7)
        c[:3] = (1.0, pos[i, 1], -pos[i, 0])
        c[3 + i] = -1.0
        block += um[i] * np.outer(c, c)
    network = vehicle.suspension.network
    if network.has_inertia:
        block[3:, 3:] += network.inertia_block(z)
    return block


def evaluate_vertical_dynamics(vehicle, state: VerticalState, wheel_torques, steering: float,
                               tire_states: tuple[TransientTireState, ...],
                               network_state=None, *, steering_rate: float = 0.0,
                               steering_acceleration: float = 0.0) -> VerticalEvaluation:
    """Evaluate the vertical 14-DOF equations in FL, FR, RL, RR order.

    ``network_state`` holds the suspension network's own states (hydraulic gas
    fractions). It is ``None`` for a stateless layout; for a stateful one,
    ``None`` means the network at rest at the evaluated jounces, and the rates of
    the states are returned in ``corner_forces.ds_dt``. Prescribed steering
    rate/acceleration are derivatives of the road-wheel command in rad/s and
    rad/s²; a changing steer trajectory must supply both for dynamic lift.
    """
    p = vehicle.planar
    torques = np.asarray(wheel_torques, dtype=float)
    if torques.shape != (4,) or len(tire_states) != 4:
        raise ValueError("four wheel torques and four transient tire states are required")
    q = state.as_vector()
    jounce, jounce_rate = q[13:17], q[17:21]
    pos = wheel_positions(p)  # total-CG arms: planar/yaw/contact kinematics
    pos_s = sprung_wheel_positions(vehicle)  # sprung-CG arms: vertical body points
    # Sign conventions: heave positive DOWN; roll positive right side DOWN
    # (y points right); pitch positive nose UP (x points forward); jounce
    # positive is COMPRESSION, i.e. the hub moves UP relative to the body.
    # Hence the body point above each wheel and its hub move down by
    # z_body = heave + roll*y - pitch*x and z_body - jounce respectively.
    # ``corner_forces`` positive (compression) pushes the body UP and the hub DOWN.
    # Heave is the sprung CG's motion, so the body-point arms are measured from it.
    z_body = state.heave + state.roll * pos_s[:, 1] - state.pitch * pos_s[:, 0]
    z_dot = state.v_z + state.roll_rate * pos_s[:, 1] - state.pitch_rate * pos_s[:, 0]
    commanded_angles = wheel_steering_angles(steering, 4)
    lift, lift_slope, lift_curvature = vehicle.suspension.kinematics.steering_lift(commanded_angles)
    if not np.isfinite(steering_rate) or not np.isfinite(steering_acceleration):
        raise ValueError("steering rate and acceleration must be finite")
    steer_rates = wheel_steering_angles(steering_rate, 4)
    steer_accelerations = wheel_steering_angles(steering_acceleration, 4)
    hub_down = z_body - jounce - lift
    hub_rate = z_dot - jounce_rate - lift_slope * steer_rates
    tire_vertical = tire_deflection_load(p.tire, hub_down, hub_rate,
                                         vehicle.tire_vertical_damping)
    loads = np.maximum(np.asarray(tire_vertical.load), 0.0)
    angles = commanded_angles + vehicle.suspension.kinematics.toe(jounce)
    planar_state = PlanarState(state.v_x, state.v_y, state.yaw_rate, q[9:13])
    vx, vy = contact_velocities(planar_state, pos, angles)
    radii = loaded_radius(p.tire, loads)
    sr, sa = slip_quantities(q[9:13], radii, vx, vy, 1.0)
    forces, tire_rates = [], []
    for i in range(4):
        tf, tr = evaluate_transient_unitire(p.tire, vehicle.transient_tire, tire_states[i],
                                             float(sa[i]), float(sr[i]), float(loads[i]),
                                             float(q[9+i]), float(vx[i]))
        forces.append(tf); tire_rates.append(tr)
    fx = np.array([f.longitudinal_force for f in forces]); fy = np.array([f.lateral_force for f in forces])
    mz = np.array([f.aligning_moment for f in forces])
    bfx, bfy = rotate_tire_forces_to_body(fx, fy, angles)
    drag_down, drag = aerodynamic_forces(p, float(np.hypot(state.v_x, state.v_y)))
    kinematics = vehicle.suspension.kinematics
    suspension = vehicle.suspension.network.forces(jounce, jounce_rate, network_state)
    total_force = np.array([suspension.total[i] for i in range(4)])
    # Body Newton-Euler; include gravity, aero, RC jacking and anti-dive/squat.
    # Roll-centre curves are queried at absolute roll capped to measured range.
    roll_query = float(np.clip(abs(state.roll), 0.0, 5.0*np.pi/180.0))
    # RC height is even under roll reversal (y -> -y): a mirrored vehicle has the
    # same RC height, so it is not sign-flipped with roll. The lateral jacking
    # below signs itself through bfy and sign(y_i).
    rc_front = kinematics.roll_centre_height(0, roll_query)
    rc_rear = kinematics.roll_centre_height(1, roll_query)
    mass = vehicle.sprung_mass
    ax = (np.sum(bfx) - drag) / p.total_mass
    ay = np.sum(bfy) / p.total_mass
    x_sprung = pos_s[:, 0]
    y_wheel = pos_s[:, 1]
    um = unsprung_masses(vehicle)
    wheelbase = float(p.wheelbase)
    total_unsprung = float(np.sum(um))
    # Hub longitudinal balance: the suspension links carry H_x_i = bfx_i - mu_i*ax,
    # so the body-side link forces sum to m_s*ax + drag. The tire forces act at the
    # contact patch, while the hub inertia mu_i*ax acts at the unsprung CG height
    # r_1; h_s*Sigma H_x below plus the unsprung couple jacking_pitch reproduce
    # that ground-level moment balance, so the whole-vehicle identity
    # Sigma x*N = -m_total*h_cg*ax - h_s*drag + aero_arm*downforce + Sigma x*um*g
    # holds. Sign: a forward force below the CG is nose-up, so acceleration squats
    # (+pitch) and braking dives (-pitch). Drag acts at the sprung CG.
    link_longitudinal = bfx - um * ax
    # Anti-dive/anti-squat. The thrust line through the side-view instant centre
    # carries the longitudinal contact force into the body as an internal vertical
    # pair (body up, hub down). Sign: a braking (rearward, bfx < 0) force at the
    # front pushes the body up (anti-dive) and a tractive (forward, bfx > 0) force
    # at the rear pushes the body up (anti-squat), so the body-side force is
    # -k*bfx at the front and +k*bfx at the rear. Scale: the DSS percentage is a
    # share of the whole-vehicle longitudinal load transfer m*a*h_cg/L, so
    # k_axle = anti*(h_cg/L)/share, with the brake bias under braking and the
    # driven-axle share under drive. See the ruling in the Task 6 roll report.
    front_drive_share, rear_drive_share = axle_drive_shares(vehicle)
    front_brake_share = float(p.brake_bias)
    anti = np.zeros(4)
    for axle, ix, sign, brake_share, drive_share in (
        (0, (0, 1), -1.0, front_brake_share, front_drive_share),
        (1, (2, 3), 1.0, 1.0 - front_brake_share, rear_drive_share),
    ):
        for i in ix:
            driving = bfx[i] >= 0.0
            share = drive_share if driving else brake_share
            percentage = (kinematics.anti_drive(axle) if driving
                          else kinematics.anti_brake(axle))
            if share <= 0.0:
                # An axle that carries none of the reference force has no
                # geometry force to generate; dividing by its zero share would
                # otherwise be unbounded.
                anti[i] = 0.0
            else:
                # anti*(h_cg/L)*F_x,axle/share. This equals the DSS reference
                # anti*m*a*h_cg/L only while the realized axle split equals the
                # design share (brake bias under braking, driven share under
                # drive). At speed drag contributes to m*a without an axle force
                # and the load-dependent tire split differs from the bias, so the
                # link force is not exactly anti*m*a*h_cg/L.
                anti[i] = sign * percentage * (float(p.cg_height) / wheelbase) * bfx[i] / share
    # Hub lateral balance: the suspension link carries Y_i = bfy_i - mu_i*ay, so
    # the body-side link forces sum to m_s*ay. There is no separate Sigma Fy*h_rc
    # roll moment: the lateral force acting at ground level is accounted for in
    # the generalized forces below by the -h_s*Sigma Y term, which reproduces the
    # whole-vehicle identity Sigma y_i*N_i = -m_total*h_cg*ay.
    link_lateral = bfy - um * ay
    # Roll-centre (swing-arm) jacking. A lateral link force at the RC height h_rc
    # reaches the hub and the body as an internal vertical pair D (body up, hub
    # down); the arm's lateral travel per unit vertical travel is h_rc/|y_i|, so
    # D_i = -sign(y_i)*2*h_rc_i*Y_i/track. The pair cancels in heave, roll and
    # pitch, and acts only on the jounce rows, which is what transfers load.
    rc_height = np.array([rc_front, rc_front, rc_rear, rc_rear])
    track = 2.0 * p.half_track
    sign_y = np.where(y_wheel > 0.0, 1.0, -1.0)
    jacking = -sign_y * 2.0 * rc_height * link_lateral / track
    # The unsprung lateral inertia mu_i*ay acts at the hub CG, r_1 above ground,
    # so it adds a hub-only jacking pair (no body partner).
    jacking_unsprung = -sign_y * 2.0 * um * ay * vehicle.unsprung_cg_height / track
    # Unsprung longitudinal inertial transfer, the pitch analog of the term above:
    # the unsprung CG is r_1 above ground, so the unsprung share of the
    # longitudinal load transfer acts as a hub-only vertical pair of total
    # magnitude m_u*ax*r_1/L, split half to each wheel, negative at the front and
    # positive at the rear (the front gains load in braking).
    jacking_pitch = (np.array([-1.0, -1.0, 1.0, 1.0])
                     * 0.5 * total_unsprung * ax * vehicle.unsprung_cg_height / wheelbase)
    # Anti-dive/anti-squat and the RC jacking are equal-and-opposite vertical
    # force pairs between sprung body and axle; the unsprung jacking pairs are
    # hub-only. B_i acts upward on the body, H_i downward on the hub.
    body_vertical_corner = total_force + anti + jacking
    hub_force = (um * GRAVITY + total_force + anti + jacking + jacking_unsprung
                 + jacking_pitch - loads)
    # Downforce acts down at the front/rear axle lever arms with the static
    # solver's aero_balance split. Gravity acts at the sprung CG and carries no
    # moment; an upward force at a wheel enters the nose-up pitch moment as +x*F.
    aero_balance = p.aero.aero_balance
    aero_arm = (aero_balance * 0.5 * (x_sprung[0] + x_sprung[1])
                + (1.0 - aero_balance) * 0.5 * (x_sprung[2] + x_sprung[3]))
    # Generalized forces Q for [heave +down, roll right-down, pitch nose-up,
    # jounce +compression x4], from virtual work with hub-down velocity
    # c_i . qdot, c_i = [1, y_i, -x_i, -e_i]. The body carries B_i upward
    # (hence -y_i*B_i and +x_i*B_i) and the hub carries H_i downward. The link
    # lateral and longitudinal forces act at ground level, h_s below the sprung
    # CG, giving the direct roll moment -h_s*Sigma Y and pitch moment
    # +h_s*Sigma H_x.
    gen = np.zeros(7)
    gen[0] = mass * GRAVITY + drag_down - np.sum(body_vertical_corner) + np.sum(hub_force)
    # Roll is positive right side DOWN: an upward force on the right (y > 0)
    # gives a negative roll moment, so +ay (rightward acceleration, a right turn)
    # rolls the body left side down, i.e. negative, and loads the left tires.
    gen[1] = (-np.sum(y_wheel * body_vertical_corner) + np.sum(y_wheel * hub_force)
              - vehicle.sprung_cg_height * np.sum(link_lateral))
    gen[2] = (np.sum(x_sprung * body_vertical_corner) - drag_down * aero_arm
              - np.sum(x_sprung * hub_force)
              + vehicle.sprung_cg_height * np.sum(link_longitudinal))
    gen[3:] = -hub_force

    has_inertance = bool(vehicle.suspension.network.has_inertia)
    block = vertical_mass_block(vehicle, jounce)
    forcing_block = gen.copy()
    # Prescribed steering subtracts h(delta) from hub-down position, so the
    # known h_ddot=h''*delta_dot^2+h'*delta_ddot enters M*a=Q+m*c*h_ddot.
    lift_accel = lift_curvature*steer_rates**2 + lift_slope*steer_accelerations
    if np.any(lift_accel):
        for i in range(4):
            c = np.array([1.0, y_wheel[i], -x_sprung[i],
                          *(float(j == i)*-1.0 for j in range(4))])
            forcing_block += um[i]*c*lift_accel[i]
    generalized = forcing_block.copy()
    if has_inertance:
        # A varying inerter motion ratio moves 0.5 b_eff' jdot^2 to the right-hand
        # side; the network owns the term (see ``inertial_forcing``).
        forcing_block[3:] += vehicle.suspension.network.inertial_forcing(jounce, jounce_rate)
    accel = np.linalg.solve(block, forcing_block)
    rr = np.zeros(21)
    rr[:2] = (ax + state.yaw_rate*state.v_y, ay - state.yaw_rate*state.v_x)
    rr[5] = yaw_moment_of(pos, bfx, bfy, mz) / p.yaw_inertia
    rr[list(VERTICAL_ACCELERATION_INDEX)] = accel
    rr[6:9] = (state.v_z, state.roll_rate, state.pitch_rate)
    rr[9:13] = (torques - fx*radii - np.array([f.rolling_resistance_moment for f in forces])) / p.wheel_inertia
    rr[13:17] = jounce_rate
    forcing = rr.copy()
    mass_matrix = None
    if has_inertance:
        mass_matrix = np.eye(21)
        index = list(VERTICAL_ACCELERATION_INDEX)
        mass_matrix[np.ix_(index, index)] = block
        forcing[index] = forcing_block
    return VerticalEvaluation(rr, mass_matrix, np.asarray([x for t in tire_rates for x in dataclasses.astuple(t)]),
                              loads, np.column_stack((fx, fy, mz)), suspension,
                              loads <= 0, int(np.count_nonzero(suspension.out_of_range)), np.array([ax, ay]),
                              forcing, generalized)
