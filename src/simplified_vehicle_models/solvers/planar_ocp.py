"""Solve a minimum-time hot lap with a spatial nonlinear program.

The lap is transcribed in the distance domain. The independent variable is the
path distance, so the mesh follows the track and the periodic boundary is
exact.

States at every node: the lateral deviation from the path, the heading error
against the path tangent, the body longitudinal and lateral velocity, the yaw
rate, the battery state of charge, and the elapsed time.

Controls at every node: the throttle, which scales a drive torque request, the
brake, which scales the brake torque, and the road-wheel steering angle.

Algebraic variables at every node: four vertical tire loads, four quasi-steady
wheel angular speeds, and the pack current. A simulation iterates the vertical load and the tire force to
consistency. A nonlinear program must not nest that iteration, so this module
carries the loads as variables and imposes the consistency equations. The
coupling is then exact and differentiable.

The motor limits enter as constraints on the wheel torque and speed, while
motor electrical demand equals pack terminal power at every node. Neither
motor torque nor bus power is clipped. The formulation stays smooth, and the solver sees
the limit instead of a kink.

CasADi and IPOPT are optional dependencies. Install ``casadi`` to use this
module. The tests skip when it is absent.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from simplified_vehicle_models.controls.brakes import brake_torques
from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.drivetrain.fixed_split import wheel_shares
from simplified_vehicle_models.formulations.planar_7dof import (
    evaluate_normal_loads,
)
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.types import TrackProfile

#: The drive split of the placeholder vehicle.
DRIVE_FRONT_FRACTION = 0.2

#: Number of states, controls, and algebraic values at one node.
#:
#: The wheel speeds are **algebraic**, not states. The wheel equation has a
#: time constant near 1.4 ms, which is about 3.5 cm of travel, so a mesh of
#: several metres is hundreds of time constants long. Carrying the wheel speed
#: as a state makes the sensitivity between two nodes vanish and leaves the
#: Jacobian badly conditioned. The quasi-steady balance ``T = Re*Fx - M`` is
#: the correct reduction at lap scale, and it removes the stiffness.
STATE_COUNT = 7
CONTROL_COUNT = 3
LOAD_COUNT = 4
ALGEBRAIC_COUNT = 9

#: Equality equations: four load balances, four quasi-steady wheel balances,
#: and pack power equal to summed four-motor electrical demand.
ALGEBRAIC_EQUATION_COUNT = 9

#: The first states repeat after one lap. Time and charge do not.
PERIODIC_STATE_COUNT = 5

#: Slip speed floor, matching the simulation model.
_SPEED_FLOOR = 1.0

#: Numerical guards. These keep an infeasible solver step from returning a
#: NaN. They sit far below any feasible value, so they never change a solution.
_PROGRESS_FLOOR = 0.25
_PATH_DENOMINATOR_FLOOR = 0.25

_casadi: Any = None


def _require_casadi() -> Any:
    """Import CasADi on first use, so the package stays optional."""

    global _casadi
    if _casadi is None:
        try:
            import casadi
        except ImportError as error:  # pragma: no cover - host dependent
            raise ImportError(
                "the planar optimal control needs the optional 'casadi' package"
            ) from error
        _casadi = casadi
    return _casadi


@dataclass(frozen=True, slots=True)
class PlanarOcpOptions:
    """Control one planar optimal-control solve."""

    #: Distance between two nodes in metres.
    mesh_spacing: float = 5.0
    #: Greatest lateral deviation from the path in metres.
    maximum_lateral_error: float = 0.05
    #: Greatest heading error against the tangent in radians.
    maximum_heading_error: float = 0.20
    #: Smallest allowed progress speed in metres per second.
    minimum_progress_speed: float = 2.0
    #: Greatest steering angle in radians. ``None`` uses the vehicle limit.
    maximum_steering: float | None = None
    #: Greatest brake pedal fraction.
    maximum_brake: float = 1.0
    #: Bound simultaneous throttle/brake to forbid artificial torque vectoring.
    maximum_throttle_brake_overlap: float = 1.0e-3
    #: Greatest yaw rate in radians per second. The value only keeps the
    #: solver out of a physically impossible region; it is not a model limit.
    maximum_yaw_rate: float = 8.0
    #: Greatest roller speed in metres per second.
    maximum_speed: float = 45.0
    #: Initial guess speed in metres per second.
    initial_speed: float = 15.0
    #: Initial guess throttle fraction.
    initial_throttle: float = 0.3
    #: Build a low-speed algebraically exact steady-turn starting trajectory.
    equilibrium_seed: bool = False
    equilibrium_speed: float = 8.0
    #: Solver iteration limit. A coarse mesh needs several thousand, because
    #: the vertical load and the wheel balance converge slowly.
    maximum_iterations: int = 9000
    #: Solver convergence tolerance.
    tolerance: float = 1.0e-6
    #: IPOPT print level. Zero keeps the output quiet.
    print_level: int = 0
    #: Accept a solution that IPOPT reports as solved to an acceptable level.
    accept_acceptable: bool = True
    #: Solve a coarse mesh first and refine, using each solution to start the
    #: next rung. **Off by default**, because it measured worse than a cold
    #: start: the converged 20 m solution drove the 10 m rung to a degenerate
    #: slow solution instead of a good one. It is kept for experiments.
    warm_start: bool = False
    #: The spacing of the first, coarsest rung of the refinement ladder.
    warm_start_spacing: float = 20.0
    #: Project wheel speeds and loads onto the local algebraic equations.
    project_initial_guess: bool = False
    #: Use the exact sparse Hessian rather than IPOPT's limited-memory update.
    exact_hessian: bool = False
    #: Balance physical equation units in IPOPT without changing the physics.
    scale_constraints: bool = False


@dataclass(frozen=True, slots=True)
class PlanarOcpResult:
    """Store one planar optimal-control result.

    Every array holds one value per node. A four-wide array uses the wheel
    order FL, FR, RL, RR.
    """

    status: str
    converged: bool
    lap_time: float
    distance: FloatArray
    elapsed_time: FloatArray
    lateral_error: FloatArray
    heading_error: FloatArray
    v_x: FloatArray
    v_y: FloatArray
    yaw_rate: FloatArray
    wheel_speeds: FloatArray
    state_of_charge: FloatArray
    throttle: FloatArray
    brake: FloatArray
    steering: FloatArray
    normal_loads: FloatArray
    wheel_torques: FloatArray
    pack_current: FloatArray
    bus_power: FloatArray
    periodicity_residual: float
    maximum_load_residual: float
    maximum_dynamics_residual: float
    maximum_inequality_violation: float
    maximum_power_mismatch: float
    maximum_control_overlap: float
    maximum_lateral_error: float
    iterations: int
    #: IPOPT multipliers for an exact same-mesh primal/dual restart.
    lam_x: FloatArray | None = None
    lam_g: FloatArray | None = None

    @property
    def maximum_algebraic_residual(self) -> float:
        """Return the largest load, wheel or electrical-power equality error.

        ``maximum_load_residual`` is retained as the older field name.
        The components have different SI units, so inspect the separated
        ``maximum_power_mismatch`` when evaluating electrical accuracy.
        """

        return self.maximum_load_residual

    @property
    def node_count(self) -> int:
        """Return the number of stored nodes."""

        return int(np.size(self.distance))

    @property
    def speed(self) -> FloatArray:
        """Return the ground speed at every node."""

        return np.hypot(self.v_x, self.v_y)


def _build_path_data(track: TrackProfile, spacing: float) -> dict[str, FloatArray]:
    """Sample the signed path on a uniform mesh that closes exactly."""

    coordinates = track.path_coordinates
    if coordinates is None:
        raise ValueError(
            "the planar optimal control needs a signed track with path"
            " coordinates"
        )
    if spacing <= 0.0 or not np.isfinite(spacing):
        raise ValueError("the mesh spacing must be finite and positive")
    length = track.lap_length
    intervals = int(round(length / spacing))
    if intervals < 16:
        raise ValueError("the mesh spacing is too coarse for this track")
    effective = length / intervals
    distance = np.arange(intervals + 1, dtype=np.float64) * effective
    distance[-1] = length
    _, _, heading, curvature = evaluate_path(coordinates, distance)
    return {
        "distance": distance,
        "spacing": np.asarray([effective], dtype=np.float64),
        "heading": np.asarray(heading, dtype=np.float64),
        "curvature": np.asarray(curvature, dtype=np.float64),
    }


def _brake_torque_magnitudes(vehicle: PlanarVehicleParameters) -> FloatArray:
    """Return the brake torque magnitude at full pedal."""

    result = brake_torques(
        1.0,
        vehicle.maximum_pedal_force,
        vehicle.pedal_curve,
        vehicle.hydraulic_ratio,
        vehicle.maximum_brake_pressure,
        vehicle.caliper_area,
        vehicle.brake_bias,
        vehicle.pad_mu_front,
        vehicle.pad_mu_rear,
        vehicle.brake_effective_radius_front,
        vehicle.brake_effective_radius_rear,
    )
    return result.wheel_torque_magnitudes


def _wheel_positions(vehicle: PlanarVehicleParameters) -> FloatArray:
    """Return the wheel positions in body axes, in FL, FR, RL, RR order."""

    return np.asarray(
        [
            (vehicle.front_axle_to_cg, -vehicle.half_track),
            (vehicle.front_axle_to_cg, vehicle.half_track),
            (-vehicle.rear_axle_to_cg, -vehicle.half_track),
            (-vehicle.rear_axle_to_cg, vehicle.half_track),
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True, slots=True)
class _NodeFunctions:
    """Hold the symbolic functions of one node."""

    dynamics: Any
    algebraic: Any
    bounds: Any


def _build_node_functions(
    casadi: Any,
    vehicle: PlanarVehicleParameters,
    drive_reference: float,
    brake_magnitude: FloatArray,
) -> _NodeFunctions:
    """Build the symbolic node functions once.

    ``dynamics`` returns the state derivative against distance.
    ``algebraic`` returns four load, four wheel and one battery-power residual.
    ``bounds`` returns voltage, bus power, per-motor torque/power/speed bounds,
    progress, path speed and the traction margin.
    """

    from simplified_vehicle_models.solvers.planar_symbolic import (
        symbolic_loaded_radius,
        symbolic_tire,
    )

    tire = vehicle.tire
    motor = vehicle.motor
    storage = vehicle.storage
    positions = _wheel_positions(vehicle)
    shares = wheel_shares(DRIVE_FRONT_FRACTION)

    x = casadi.SX.sym("x", STATE_COUNT)
    u = casadi.SX.sym("u", CONTROL_COUNT)
    algebraic = casadi.SX.sym("a", ALGEBRAIC_COUNT)
    kappa = casadi.SX.sym("kappa")
    f_z = algebraic[0:4]
    wheel_speed = algebraic[4:8]
    pack_current = algebraic[8]

    lateral_error, heading_error = x[0], x[1]
    v_x, v_y, yaw_rate = x[2], x[3], x[4]
    state_of_charge = x[5]
    throttle, brake, steering = u[0], u[1], u[2]

    steer_angle = casadi.vertcat(steering, steering, 0.0, 0.0)
    body_longitudinal = v_x - yaw_rate * positions[:, 1]
    body_lateral = v_y + yaw_rate * positions[:, 0]
    long_velocity = body_longitudinal * casadi.cos(
        steer_angle
    ) + body_lateral * casadi.sin(steer_angle)
    lat_velocity = -body_longitudinal * casadi.sin(
        steer_angle
    ) + body_lateral * casadi.cos(steer_angle)

    denominator = casadi.fmax(casadi.fabs(long_velocity), _SPEED_FLOOR)
    slip_ratio = casadi.SX.zeros(LOAD_COUNT)
    slip_angle = casadi.SX.zeros(LOAD_COUNT)
    long_force = casadi.SX.zeros(LOAD_COUNT)
    lat_force = casadi.SX.zeros(LOAD_COUNT)
    aligning = casadi.SX.zeros(LOAD_COUNT)
    rolling = casadi.SX.zeros(LOAD_COUNT)
    radius = casadi.SX.zeros(LOAD_COUNT)
    friction = casadi.SX.zeros(LOAD_COUNT)
    for wheel in range(LOAD_COUNT):
        # The loaded radius does not depend on the slip, so take it directly.
        # Evaluating the full tire at zero slip would put a square root of zero
        # in the graph, whose derivative is not finite.
        radius[wheel] = symbolic_loaded_radius(tire, f_z[wheel])
        slip_ratio[wheel] = (
            radius[wheel] * wheel_speed[wheel] - long_velocity[wheel]
        ) / denominator[wheel]
        slip_angle[wheel] = -casadi.atan2(
            lat_velocity[wheel], denominator[wheel]
        )
        one = symbolic_tire(
            tire,
            slip_angle[wheel],
            slip_ratio[wheel],
            f_z[wheel],
            wheel_speed[wheel],
        )
        long_force[wheel] = one.longitudinal_force
        lat_force[wheel] = one.lateral_force
        aligning[wheel] = one.aligning_moment
        rolling[wheel] = one.rolling_moment
        friction[wheel] = one.longitudinal_friction

    # The rotation must use one angle per wheel. A scalar angle would rotate a
    # straight rear wheel by the front steering angle.
    cosine = casadi.cos(steer_angle)
    sine = casadi.sin(steer_angle)
    body_fx = long_force * cosine - lat_force * sine
    body_fy = long_force * sine + lat_force * cosine

    speed_squared = v_x * v_x + v_y * v_y
    downforce = 0.5 * vehicle.aero.air_density * speed_squared * vehicle.aero.downforce_area
    drag = 0.5 * vehicle.aero.air_density * speed_squared * vehicle.aero.drag_area

    longitudinal_acceleration = (
        casadi.sum1(body_fx) - drag
    ) / vehicle.total_mass + yaw_rate * v_y
    lateral_acceleration = (
        casadi.sum1(body_fy) / vehicle.total_mass - yaw_rate * v_x
    )
    moment = casadi.sum1(
        positions[:, 0] * body_fy - positions[:, 1] * body_fx + aligning
    )
    yaw_acceleration = moment / vehicle.yaw_inertia

    # Quasi-static loads use *inertial* acceleration, not body velocity
    # derivatives: in a steady turn dv_y/dt=0 but a_y=r*v_x is nonzero.
    physical_longitudinal = (casadi.sum1(body_fx) - drag) / vehicle.total_mass
    physical_lateral = casadi.sum1(body_fy) / vehicle.total_mass
    front_axle = (
        vehicle.total_mass * GRAVITY * vehicle.front_mass_fraction
        + downforce * vehicle.aero.aero_balance
        - vehicle.total_mass * physical_longitudinal * vehicle.cg_height
        / vehicle.wheelbase
    )
    rear_axle = vehicle.total_mass * GRAVITY + downforce - front_axle
    total_transfer = (
        vehicle.total_mass * physical_lateral * vehicle.cg_height
        / vehicle.track_width
    )
    front_transfer = (
        total_transfer * vehicle.lateral_load_transfer_distribution
    )
    rear_transfer = total_transfer - front_transfer
    expected_loads = casadi.vertcat(
        front_axle * 0.5 + front_transfer,
        front_axle * 0.5 - front_transfer,
        rear_axle * 0.5 + rear_transfer,
        rear_axle * 0.5 - rear_transfer,
    )
    load_residual = f_z - expected_loads

    # Pack and motor.
    cell_voltage = casadi.interpolant(
        "open_circuit_voltage",
        "linear",
        [np.asarray(storage.state_of_charge_axis)],
        np.asarray(storage.open_circuit_voltage_axis),
    )(state_of_charge)
    cell_resistance = casadi.interpolant(
        "cell_resistance",
        "linear",
        [np.asarray(storage.state_of_charge_axis)],
        np.asarray(storage.resistance_axis),
    )(state_of_charge)
    open_circuit = storage.series_count * cell_voltage
    pack_resistance = (
        storage.series_count * cell_resistance / storage.parallel_count
    )
    terminal = open_circuit - pack_current * pack_resistance
    bus_power = terminal * pack_current
    # Friction braking has no regeneration. Only the positive motor torque
    # draws electrical power; the gear and inverter losses both cost energy.
    drive_torque = shares * (throttle * drive_reference)
    motor_bus_power = casadi.sum1(drive_torque * wheel_speed) / motor.drive_efficiency
    power_residual = bus_power - motor_bus_power

    # The voltage scales the base speed, so a low voltage reaches the power
    # bound sooner. Clamp the voltage at the nominal value.
    effective_voltage = casadi.fmax(
        casadi.fmin(terminal, motor.nominal_bus_voltage), 1.0
    )
    motor_speed = motor.gear_ratio * casadi.fabs(wheel_speed)
    scaled_speed = motor_speed * motor.nominal_bus_voltage / effective_voltage
    gearing = motor.gear_ratio * motor.motor_efficiency
    torque_bound = motor.peak_motor_torque * gearing
    power_bound = (
        motor.peak_power / casadi.fmax(scaled_speed, 1.0e-6)
    ) * gearing

    drive_total = throttle * drive_reference
    wheel_torque = shares * drive_total - brake * brake_magnitude

    # The quasi-steady wheel balance. The wheel inertia cannot appear in a
    # balance equation, so the wheel acceleration is not part of this model.
    wheel_balance = wheel_torque - radius * long_force + rolling

    path_speed = v_x * casadi.cos(heading_error) - v_y * casadi.sin(heading_error)
    # A guard, not physics. The constraint keeps the progress speed above the
    # minimum, but a solver evaluates infeasible points on the way there, and
    # a division by zero would return a NaN and stop the solver.
    progress = path_speed / casadi.fmax(
        1.0 - kappa * lateral_error, _PATH_DENOMINATOR_FLOOR
    )
    safe_progress = casadi.fmax(progress, _PROGRESS_FLOOR)

    derivative = casadi.vertcat(
        (
            v_x * casadi.sin(heading_error)
            + v_y * casadi.cos(heading_error)
        )
        / safe_progress,
        (yaw_rate - kappa * progress) / safe_progress,
        longitudinal_acceleration / safe_progress,
        lateral_acceleration / safe_progress,
        yaw_acceleration / safe_progress,
        -pack_current
        / (3600.0 * storage.pack_capacity_ah * safe_progress),
        1.0 / safe_progress,
    )

    dynamics = casadi.Function(
        "planar_dynamics", [x, u, algebraic, kappa], [derivative]
    )
    algebraic_function = casadi.Function(
        "planar_algebraic",
        [x, u, algebraic, kappa],
        [casadi.vertcat(load_residual, wheel_balance, power_residual)],
    )
    # The traction margin. A quasi-steady wheel can only balance a torque that
    # the tire can transmit. Without this bound the wheel balance has no
    # solution once the demand passes the friction limit, and a solver stalls
    # on an equation that nothing can satisfy.
    traction_margin = wheel_torque - radius * friction * f_z
    traction_margin = casadi.vertcat(
        traction_margin, -wheel_torque - radius * friction * f_z
    )
    bounds = casadi.Function(
        "planar_bounds",
        [x, u, algebraic, kappa],
        [
            terminal,
            bus_power,
            torque_bound,
            power_bound,
            progress,
            path_speed,
            traction_margin,
            scaled_speed - motor.maximum_motor_speed,
        ],
    )
    return _NodeFunctions(
        dynamics=dynamics,
        algebraic=algebraic_function,
        bounds=bounds,
    )


def solve_planar_hot_lap(
    vehicle: PlanarVehicleParameters,
    track: TrackProfile,
    options: PlanarOcpOptions = PlanarOcpOptions(),
) -> PlanarOcpResult:
    """Solve the minimum-time hot lap of the planar vehicle.

    The objective is the elapsed time. The mechanical states repeat after one
    lap, the initial charge is fixed, and the lateral error, the heading error,
    the progress speed, the steering, the tire loads, the motor limits and the
    pack limits all bound the solution.
    """

    casadi = _require_casadi()
    if options.warm_start:
        return _solve_by_refinement(casadi, vehicle, track, options)
    return _solve(casadi, vehicle, track, options, None)


def _ladder(target: float, coarsest: float) -> list[float]:
    """Return the mesh spacings from coarse to fine, ending at the target."""

    rungs: list[float] = []
    spacing = max(coarsest, target)
    while spacing > target * 1.05:
        rungs.append(spacing)
        spacing *= 0.5
    if not rungs or abs(rungs[-1] - target) > 1.0e-9:
        rungs.append(target)
    return rungs


def _solve_by_refinement(
    casadi: Any,
    vehicle: PlanarVehicleParameters,
    track: TrackProfile,
    options: PlanarOcpOptions,
) -> PlanarOcpResult:
    """Solve a ladder of meshes, each one starting from the last solution.

    A fine mesh solved from a simple guess usually stalls, because the problem
    is large and the first steps are far from feasible. A coarse mesh converges
    easily and gives a physically meaningful starting point for the next rung.
    """

    from dataclasses import replace

    result: PlanarOcpResult | None = None
    previous: dict[str, np.ndarray] | None = None
    for spacing in _ladder(options.mesh_spacing, options.warm_start_spacing):
        stage = replace(options, mesh_spacing=spacing, warm_start=False)
        result = _solve(casadi, vehicle, track, stage, previous)
        if not result.converged and spacing != options.mesh_spacing:
            raise RuntimeError(
                f"the {spacing:g} m rung is infeasible ({result.status}; "
                f"algebraic {result.maximum_load_residual:g}, "
                f"dynamics {result.maximum_dynamics_residual:g}); "
                "cannot use it as a finer-mesh start"
            )
        previous = _as_start(result)
    assert result is not None
    return result


def _quasi_steady_speed(
    path: dict[str, FloatArray],
    vehicle: PlanarVehicleParameters,
    options: PlanarOcpOptions,
) -> FloatArray:
    """Return a quasi-steady corner speed profile as an initial guess.

    The profile takes the cornering limit from the tire friction and the
    longitudinal limits from the drive and the brakes. A forward pass adds the
    drive limit and a backward pass adds the brake limit. The track is closed,
    so the two passes repeat until the profile is consistent around the lap.

    This is a guess, not a solution. It gives the nonlinear program a
    physically sensible starting point, which is what the solver needs.
    """

    distance = path["distance"]
    curvature = np.abs(path["curvature"])
    tire = vehicle.tire
    mu = min(float(tire.mu_0_x), float(tire.mu_0_y))
    # A seed near peak tire friction leaves no reserve for longitudinal
    # braking/drive or lateral load transfer; begin below that limit.
    lateral_limit = (0.75 if options.project_initial_guess else 1.0) * mu * GRAVITY
    static_loads = evaluate_normal_loads(vehicle, 0.0, 0.0, 0.0)
    speed_ceiling = min(
        options.maximum_speed,
        (0.82 if options.project_initial_guess else 0.98)
        * vehicle.motor.maximum_motor_speed / vehicle.motor.gear_ratio
        * float(np.min(loaded_radius(tire, static_loads))),
    )
    # Pull the drive limit from the motor at the wheel.
    peak_wheel = (
        vehicle.motor.peak_motor_torque
        * vehicle.motor.gear_ratio
        * vehicle.motor.motor_efficiency
    )
    drive_limit = 4.0 * peak_wheel / (0.15 * vehicle.total_mass)
    drive_limit = min(drive_limit, 12.0)
    brake_limit = float(
        np.sum(_brake_torque_magnitudes(vehicle)) / (0.15 * vehicle.total_mass)
    )

    with np.errstate(divide="ignore"):
        corner = np.where(curvature > 1.0e-9, np.sqrt(lateral_limit / np.maximum(curvature, 1.0e-12)), speed_ceiling)
    speed = np.minimum(corner, speed_ceiling)
    spacing = float(path["spacing"][0])
    for _ in range(8):
        # Forward pass for the drive limit.
        for index in range(1, speed.size):
            speed[index] = min(
                speed[index],
                np.sqrt(max(speed[index - 1] ** 2 + 2.0 * drive_limit * spacing, 0.0)),
            )
        speed[-1] = min(speed[-1], speed[0])
        # Backward pass for the brake limit.
        for index in range(speed.size - 2, -1, -1):
            speed[index] = min(
                speed[index],
                np.sqrt(max(speed[index + 1] ** 2 + 2.0 * brake_limit * spacing, 0.0)),
            )
        speed[0] = min(speed[0], speed[-1])
    return np.maximum(speed, options.minimum_progress_speed)


def _initial_guess(
    path: dict[str, FloatArray],
    vehicle: PlanarVehicleParameters,
    options: PlanarOcpOptions,
) -> dict[str, np.ndarray]:
    """Return a physically consistent starting point.

    The loads and the wheel speeds follow the quasi-steady balance at the
    guessed speed, so the algebraic equations start near zero.
    """

    distance = path["distance"]
    nodes = distance.size
    curvature = path["curvature"]
    speed = _quasi_steady_speed(path, vehicle, options)
    lateral_acceleration = speed**2 * curvature

    state = np.zeros((STATE_COUNT, nodes), dtype=np.float64)
    algebraic = np.zeros((ALGEBRAIC_COUNT, nodes), dtype=np.float64)
    for index in range(nodes):
        loads = evaluate_normal_loads(
            vehicle,
            float(speed[index]),
            0.0,
            float(lateral_acceleration[index]),
        )
        algebraic[0:4, index] = loads
        radii = loaded_radius(vehicle.tire, loads)
        algebraic[4:8, index] = float(speed[index]) / radii
    state[2] = speed
    state[4] = speed * curvature
    state[5] = vehicle.storage.initial_state_of_charge
    progress = np.maximum(speed, options.minimum_progress_speed)
    travel = np.concatenate(([0.0], np.cumsum(np.diff(distance) / progress[:-1])))
    state[6] = travel - travel[0]

    control = np.zeros((CONTROL_COUNT, nodes), dtype=np.float64)
    control[2] = np.arctan(vehicle.wheelbase * curvature)
    acceleration = np.gradient(speed**2, 2.0 * distance, edge_order=1)
    control[0] = np.clip(acceleration / 6.0, 0.0, 1.0)
    control[1] = np.clip(-acceleration / 12.0, 0.0, options.maximum_brake)
    storage = vehicle.storage
    algebraic[8] = min(0.4 * storage.discharge_current_limit, 200.0)
    return {"state": state, "control": control, "algebraic": algebraic}


def _as_start(result: PlanarOcpResult) -> dict[str, np.ndarray]:
    """Return a result as an initial guess for a finer mesh."""

    return {
        "distance": result.distance,
        "state": np.vstack(
            (
                result.lateral_error,
                result.heading_error,
                result.v_x,
                result.v_y,
                result.yaw_rate,
                result.state_of_charge,
                result.elapsed_time,
            )
        ),
        "control": np.vstack((result.throttle, result.brake, result.steering)),
        "algebraic": np.vstack(
            (result.normal_loads, result.wheel_speeds, result.pack_current)
        ),
        **({"lam_x": result.lam_x, "lam_g": result.lam_g}
           if result.lam_x is not None and result.lam_g is not None else {}),
    }


def _interpolate_start(
    start: dict[str, np.ndarray],
    distance: FloatArray,
) -> dict[str, np.ndarray]:
    """Interpolate an earlier solution onto a new mesh."""

    source = np.asarray(start["distance"], dtype=np.float64)

    def resample(values: np.ndarray) -> np.ndarray:
        rows = np.atleast_2d(values)
        out = np.empty((rows.shape[0], distance.size), dtype=np.float64)
        for index in range(rows.shape[0]):
            out[index] = np.interp(distance, source, rows[index])
        return out

    return {
        "state": resample(start["state"]),
        "control": resample(start["control"]),
        "algebraic": resample(start["algebraic"]),
    }


def _project_start(
    guess: dict[str, np.ndarray],
    vehicle: PlanarVehicleParameters,
    path: dict[str, FloatArray],
    functions: _NodeFunctions,
) -> dict[str, np.ndarray]:
    """Solve local load/wheel balances and pack current at each guess node.

    This only prepares IPOPT's starting point. It is not a traction controller:
    the NLP continues to choose all throttle, brake and steering controls.
    An infeasible node stays a best-effort seed; only the NLP may certify it.
    """

    from scipy.optimize import least_squares
    from simplified_vehicle_models.powertrain.energy_storage import (
        StorageState,
        solve_current,
    )
    from simplified_vehicle_models.powertrain.motor_envelope import total_bus_power

    casadi = _require_casadi()
    x_symbol = casadi.SX.sym("projection_x", STATE_COUNT)
    u_symbol = casadi.SX.sym("projection_u", CONTROL_COUNT)
    a_symbol = casadi.SX.sym("projection_a", ALGEBRAIC_COUNT)
    k_symbol = casadi.SX.sym("projection_curvature")
    expression = functions.algebraic(x_symbol, u_symbol, a_symbol, k_symbol)
    jacobian = casadi.Function(
        "projection_jacobian", [x_symbol, u_symbol, a_symbol, k_symbol],
        [casadi.jacobian(expression[:8], a_symbol[:8])],
    )
    state = guess["state"].copy()
    control = guess["control"].copy()
    algebraic = guess["algebraic"].copy()
    curvature = path["curvature"]
    # Load consistency must not be sacrificed to make a saturated wheel's
    # torque balance appear smaller at a physically impossible trial speed.
    scale = np.array([100.0] * 4 + [100.0] * 4)
    cap = vehicle.motor.maximum_motor_speed / vehicle.motor.gear_ratio
    lower = np.array([1e-6] * 4 + [0.0] * 4)
    upper = np.array([2e4] * 4 + [cap] * 4)
    peak = (vehicle.motor.peak_motor_torque * vehicle.motor.gear_ratio
            * vehicle.motor.motor_efficiency)
    shares = wheel_shares(DRIVE_FRONT_FRACTION)
    for index in range(state.shape[1]):
        x = state[:, index]
        u = control[:, index]
        a = algebraic[:, index]
        k = float(curvature[index])

        def local_residual(values: np.ndarray) -> np.ndarray:
            a[:8] = values
            return np.asarray(functions.algebraic(x, u, a, k)).ravel()[:8] / scale

        def local_jacobian(values: np.ndarray) -> np.ndarray:
            a[:8] = values
            return np.asarray(jacobian(x, u, a, k)) / scale[:, None]

        for attempt in range(6):
            solution = least_squares(
                local_residual, np.clip(a[:8], lower + 1e-8, upper - 1e-8),
                jac=local_jacobian, bounds=(lower, upper),
                max_nfev=50, ftol=1e-9, xtol=1e-9, gtol=1e-9,
            )
            a[:8] = solution.x
            drive = shares * (4.0 * peak * u[0])
            demand = total_bus_power(vehicle.motor, a[4:8], drive)
            current, feasible = solve_current(
                vehicle.storage, StorageState(float(x[5])), demand
            )
            a[8] = current
            physical_residual = float(np.max(np.abs(local_residual(a[:8]) * scale)))
            if feasible and physical_residual < 1e-3:
                break
            # This changes *only the guess*. IPOPT freely chooses the controls.
            # Back away from wheel saturation or the pack's discharge limit.
            u[0] *= 0.5
            u[1] *= 0.5
            if attempt >= 2:
                u[2] *= 0.9
        if physical_residual >= 1e-3:
            # An unseen sharper corner can make the interpolated speed
            # incapable of balancing the tire at *any* slip. Decrease only
            # the starting speed and recompute the local wheel equilibrium.
            for _ in range(8):
                x[2] *= 0.85
                x[4] = x[2] * k
                a[4:8] *= 0.85
                solution = least_squares(
                    local_residual, np.clip(a[:8], lower + 1e-8, upper - 1e-8),
                    jac=local_jacobian, bounds=(lower, upper),
                    max_nfev=50, ftol=1e-9, xtol=1e-9, gtol=1e-9,
                )
                a[:8] = solution.x
                physical_residual = float(np.max(np.abs(local_residual(a[:8]) * scale)))
                if physical_residual < 1e-3:
                    break
            drive = shares * (4.0 * peak * u[0])
            demand = total_bus_power(vehicle.motor, a[4:8], drive)
            a[8], _ = solve_current(
                vehicle.storage, StorageState(float(x[5])), demand
            )

    # Reintegrate the clock and charge using the same trapezoidal spatial
    # derivatives as the NLP. Iterating once accounts for SOC-dependent OCV.
    distance = path["distance"]
    for _ in range(2):
        state[5, 0] = vehicle.storage.initial_state_of_charge
        state[6, 0] = 0.0
        derivatives = np.column_stack([
            np.asarray(functions.dynamics(state[:, j], control[:, j],
                             algebraic[:, j], float(curvature[j]))).ravel()
            for j in range(state.shape[1])
        ])
        for index, step in enumerate(np.diff(distance)):
            state[5:7, index + 1] = state[5:7, index] + 0.5 * step * (
                derivatives[5:7, index] + derivatives[5:7, index + 1]
            )
        state[5] = np.clip(state[5], 0.02, 1.0)
        for index in range(state.shape[1]):
            drive = shares * (4.0 * peak * control[0, index])
            demand = total_bus_power(vehicle.motor, algebraic[4:8, index], drive)
            current, _ = solve_current(
                vehicle.storage, StorageState(float(state[5, index])), demand
            )
            algebraic[8, index] = current
    return {"state": state, "control": control, "algebraic": algebraic}


def _solve(
    casadi: Any,
    vehicle: PlanarVehicleParameters,
    track: TrackProfile,
    options: PlanarOcpOptions,
    start: dict[str, np.ndarray] | None,
) -> PlanarOcpResult:
    """Build and solve one mesh."""

    path = _build_path_data(track, options.mesh_spacing)
    nodes = path["distance"].size
    spacing = float(path["spacing"][0])
    curvature = path["curvature"]

    maximum_steering = (
        vehicle.steering_maximum_angle
        if options.maximum_steering is None
        else float(options.maximum_steering)
    )
    peak_wheel = (
        vehicle.motor.peak_motor_torque
        * vehicle.motor.gear_ratio
        * vehicle.motor.motor_efficiency
    )
    drive_reference = 4.0 * peak_wheel
    brake_magnitude = np.asarray(
        _brake_torque_magnitudes(vehicle), dtype=np.float64
    )

    functions = _build_node_functions(
        casadi, vehicle, drive_reference, brake_magnitude
    )

    state = casadi.MX.sym("state", STATE_COUNT, nodes)
    control = casadi.MX.sym("control", CONTROL_COUNT, nodes)
    algebraic = casadi.MX.sym("algebraic", ALGEBRAIC_COUNT, nodes)

    constraints: list[Any] = []
    lower: list[float] = []
    upper: list[float] = []

    def add(expression, low, high) -> None:
        flat = casadi.reshape(expression, -1, 1)
        constraints.append(flat)
        count = int(flat.numel())
        lower.extend([low] * count)
        upper.extend([high] * count)

    # Dynamics by the trapezoid rule between two nodes.
    for index in range(nodes - 1):
        here = functions.dynamics(
            state[:, index], control[:, index], algebraic[:, index],
            curvature[index],
        )
        there = functions.dynamics(
            state[:, index + 1], control[:, index + 1], algebraic[:, index + 1],
            curvature[index + 1],
        )
        defect = state[:, index + 1] - state[:, index] - 0.5 * spacing * (
            here + there
        )
        if options.scale_constraints:
            defect /= np.array([0.05, 0.05, 20.0, 5.0, 1.0, 0.1, 60.0])
        add(defect, 0.0, 0.0)

    # The algebraic equations and the physical bounds at every node.
    minimum_pack_voltage = (
        vehicle.storage.series_count * vehicle.storage.minimum_cell_voltage
    )
    shares = wheel_shares(DRIVE_FRONT_FRACTION)
    for index in range(nodes):
        arguments = (
            state[:, index],
            control[:, index],
            algebraic[:, index],
            curvature[index],
        )
        algebraic_residual = functions.algebraic(*arguments)
        if options.scale_constraints:
            algebraic_residual /= np.array([1000.0] * 4 + [100.0] * 4 + [1.0e5])
        add(algebraic_residual, 0.0, 0.0)
        terminal, _, torque_bound, power_bound, progress, _, traction, speed_margin = (
            functions.bounds(*arguments)
        )
        add(traction / (100.0 if options.scale_constraints else 1.0), -np.inf, 0.0)
        add(speed_margin / (1000.0 if options.scale_constraints else 1.0), -np.inf, 0.0)
        drive = shares * (control[0, index] * drive_reference)
        # One constraint per motor bound, so neither becomes a kink.
        motor_scale = 100.0 if options.scale_constraints else 1.0
        add((drive - torque_bound) / motor_scale, -np.inf, 0.0)
        add((drive - power_bound) / motor_scale, -np.inf, 0.0)
        voltage_scale = 400.0 if options.scale_constraints else 1.0
        add((terminal - minimum_pack_voltage) / voltage_scale, 0.0, np.inf)
        progress_scale = 20.0 if options.scale_constraints else 1.0
        add((progress - options.minimum_progress_speed) / progress_scale,
            0.0, np.inf)
        # Keep the path parametrization from folding over.
        add(1.0 - curvature[index] * state[0, index], 0.5, np.inf)
        add(control[1, index] * brake_magnitude, 0.0, np.inf)
        add(control[0, index] * control[1, index],
            -np.inf, options.maximum_throttle_brake_overlap)

    # The mechanical states repeat after one lap.
    add(
        state[:PERIODIC_STATE_COUNT, nodes - 1] - state[:PERIODIC_STATE_COUNT, 0],
        0.0,
        0.0,
    )

    decision = casadi.vertcat(
        casadi.reshape(state, -1, 1),
        casadi.reshape(control, -1, 1),
        casadi.reshape(algebraic, -1, 1),
    )
    objective = state[6, nodes - 1]

    # Variable bounds.
    state_lower = np.zeros(STATE_COUNT, dtype=np.float64)
    state_upper = np.zeros(STATE_COUNT, dtype=np.float64)
    state_lower[0], state_upper[0] = (
        -options.maximum_lateral_error,
        options.maximum_lateral_error,
    )
    state_lower[1], state_upper[1] = (
        -options.maximum_heading_error,
        options.maximum_heading_error,
    )
    state_lower[2], state_upper[2] = (
        options.minimum_progress_speed,
        options.maximum_speed,
    )
    state_lower[3], state_upper[3] = -options.maximum_speed, options.maximum_speed
    state_lower[4], state_upper[4] = (
        -options.maximum_yaw_rate,
        options.maximum_yaw_rate,
    )
    state_lower[5], state_upper[5] = 0.02, 1.0
    state_lower[6], state_upper[6] = 0.0, 1.0e4
    control_lower = np.asarray([0.0, 0.0, -maximum_steering])
    control_upper = np.asarray([1.0, options.maximum_brake, maximum_steering])
    # The four loads, the four wheel speeds, and the pack current.
    wheel_speed_upper = options.maximum_speed / 0.15
    algebraic_lower = np.concatenate(
        (
            np.zeros(LOAD_COUNT),
            np.zeros(LOAD_COUNT),
            np.asarray([-vehicle.storage.charge_current_limit]),
        )
    )
    algebraic_upper = np.concatenate(
        (
            np.full(LOAD_COUNT, 2.0e4),
            np.full(LOAD_COUNT, wheel_speed_upper),
            np.asarray([vehicle.storage.discharge_current_limit]),
        )
    )

    lower_vector = np.concatenate(
        [
            np.tile(state_lower, nodes),
            np.tile(control_lower, nodes),
            np.tile(algebraic_lower, nodes),
        ]
    )
    upper_vector = np.concatenate(
        [
            np.tile(state_upper, nodes),
            np.tile(control_upper, nodes),
            np.tile(algebraic_upper, nodes),
        ]
    )

    # Fix only the charge at the start and the clock. Every other state is
    # free, so the vehicle may enter the lap already turning and moving. The
    # periodicity constraint closes the mechanical states.
    initial_state_lower = lower_vector.copy()
    initial_state_upper = upper_vector.copy()
    initial_state_lower[5] = vehicle.storage.initial_state_of_charge
    initial_state_upper[5] = vehicle.storage.initial_state_of_charge
    initial_state_lower[STATE_COUNT - 1] = 0.0
    initial_state_upper[STATE_COUNT - 1] = 0.0

    if options.equilibrium_seed and start is None:
        from simplified_vehicle_models.solvers.planar_equilibrium import equilibrium_start
        guess = equilibrium_start(vehicle, path, functions, options.equilibrium_speed)
    else:
        guess = _initial_guess(path, vehicle, options)
    if start is not None:
        resampled = _interpolate_start(start, path["distance"])
        guess["state"] = resampled["state"]
        guess["control"] = resampled["control"]
        guess["algebraic"] = resampled["algebraic"]
        if not np.array_equal(np.asarray(start["distance"]), path["distance"]):
            # A narrow corner can be invisible on a *coarser* mesh. Preserve
            # every state and control on an exact same-mesh restart instead.
            corner_cap = 0.9 * _quasi_steady_speed(path, vehicle, options)
            speed = np.minimum(guess["state"][2], corner_cap)
            guess["state"][2] = speed
            guess["state"][4] = speed * curvature
            guess["control"][2] = np.arctan(vehicle.wheelbase * curvature)
    # Make a restarted point respect the driver's no-two-pedals policy. The
    # old unconstrained OCP exploited this to create a hidden axle yaw input.
    requested = guess["control"]
    overlap = options.maximum_throttle_brake_overlap
    if not np.isfinite(overlap) or overlap < 0.0:
        raise ValueError("maximum_throttle_brake_overlap must be nonnegative and finite")
    drive_dominant = requested[0] >= requested[1]
    requested[1, drive_dominant] = np.minimum(
        requested[1, drive_dominant],
        overlap / np.maximum(requested[0, drive_dominant], 1.0e-12),
    )
    requested[0, ~drive_dominant] = np.minimum(
        requested[0, ~drive_dominant],
        overlap / np.maximum(requested[1, ~drive_dominant], 1.0e-12),
    )
    if options.project_initial_guess and not (options.equilibrium_seed and start is None):
        guess = _project_start(guess, vehicle, path, functions)
    guess_vector = np.concatenate(
        [
            guess["state"].reshape(-1, order="F"),
            guess["control"].reshape(-1, order="F"),
            guess["algebraic"].reshape(-1, order="F"),
        ]
    )
    guess_vector = np.clip(guess_vector, lower_vector, upper_vector)
    same_mesh = start is not None and np.array_equal(
        np.asarray(start["distance"]), path["distance"]
    )
    warm_duals = bool(
        same_mesh and start is not None
        and "lam_x" in start and "lam_g" in start
        and np.size(start["lam_x"]) == decision.numel()
        and np.size(start["lam_g"]) == len(lower)
    )

    nlp = {
        "x": decision,
        "f": objective,
        "g": casadi.vertcat(*constraints),
    }
    solver = casadi.nlpsol(
        "planar_ocp",
        "ipopt",
        nlp,
        {
            "ipopt.print_level": options.print_level,
            "print_time": False,
            "ipopt.max_iter": options.maximum_iterations,
            "ipopt.tol": options.tolerance,
            "ipopt.acceptable_tol": max(options.tolerance * 10.0, 1.0e-6),
            "ipopt.acceptable_iter": 15,
            "ipopt.mu_strategy": "adaptive",
            "ipopt.warm_start_init_point": "yes" if warm_duals else "no",
            "ipopt.warm_start_bound_push": 1.0e-8,
            "ipopt.warm_start_mult_bound_push": 1.0e-8,
            "ipopt.hessian_approximation": (
                "exact" if options.exact_hessian else "limited-memory"
            ),
        },
    )
    solution = solver(
        x0=guess_vector,
        lbx=initial_state_lower,
        ubx=initial_state_upper,
        lbg=np.asarray(lower),
        ubg=np.asarray(upper),
        **({"lam_x0": start["lam_x"], "lam_g0": start["lam_g"]}
           if warm_duals and start is not None else {}),
    )
    status = solver.stats()["return_status"]
    converged = "Solve_Succeeded" in status or (
        options.accept_acceptable and "Solved_To_Acceptable_Level" in status
    )
    values = np.asarray(solution["x"]).ravel()
    state_values = values[: STATE_COUNT * nodes].reshape(
        (STATE_COUNT, nodes), order="F"
    )
    offset = STATE_COUNT * nodes
    control_values = values[offset : offset + CONTROL_COUNT * nodes].reshape(
        (CONTROL_COUNT, nodes), order="F"
    )
    offset += CONTROL_COUNT * nodes
    algebraic_values = values[offset : offset + ALGEBRAIC_COUNT * nodes].reshape(
        (ALGEBRAIC_COUNT, nodes), order="F"
    )

    return _build_result(
        vehicle,
        path,
        state_values,
        control_values,
        algebraic_values,
        functions,
        status,
        converged,
        int(solver.stats().get("iter_count", 0)),
        options.minimum_progress_speed,
        options.maximum_throttle_brake_overlap,
        np.asarray(solution["lam_x"]).ravel(),
        np.asarray(solution["lam_g"]).ravel(),
    )


def _build_result(
    vehicle: PlanarVehicleParameters,
    path: dict[str, FloatArray],
    state: np.ndarray,
    control: np.ndarray,
    algebraic: np.ndarray,
    functions: _NodeFunctions,
    status: str,
    converged: bool,
    iterations: int,
    minimum_progress_speed: float,
    maximum_control_overlap: float,
    lam_x: FloatArray,
    lam_g: FloatArray,
) -> PlanarOcpResult:
    """Evaluate the physics at the solution and pack the result."""

    nodes = state.shape[1]
    curvature = path["curvature"]
    loads = algebraic[0:4]
    wheel_speeds = algebraic[4:8]
    current = algebraic[8:9]
    shares = wheel_shares(DRIVE_FRONT_FRACTION)
    peak_wheel = (
        vehicle.motor.peak_motor_torque
        * vehicle.motor.gear_ratio
        * vehicle.motor.motor_efficiency
    )
    drive_reference = 4.0 * peak_wheel
    brake_magnitude = np.asarray(_brake_torque_magnitudes(vehicle))

    wheel_torque = np.zeros((LOAD_COUNT, nodes))
    bus = np.zeros(nodes)
    residual = np.zeros((ALGEBRAIC_EQUATION_COUNT, nodes))
    derivative = np.zeros((STATE_COUNT, nodes))
    worst_inequality = 0.0
    for index in range(nodes):
        arguments = (
            state[:, index],
            control[:, index],
            algebraic[:, index],
            curvature[index],
        )
        residual[:, index] = np.asarray(
            functions.algebraic(*arguments)
        ).ravel()
        derivative[:, index] = np.asarray(functions.dynamics(*arguments)).ravel()
        terminal, power, torque_cap, power_cap, progress, _, traction, speed_cap = (
            functions.bounds(*arguments)
        )
        bus[index] = float(power)
        drive = shares * control[0, index] * drive_reference
        inequalities = np.concatenate((
            np.asarray(traction).ravel(),
            np.asarray(speed_cap).ravel(),
            drive - np.asarray(torque_cap).ravel(),
            drive - np.asarray(power_cap).ravel(),
            [vehicle.storage.series_count * vehicle.storage.minimum_cell_voltage
             - float(terminal),
             minimum_progress_speed - float(progress),
             0.5 - (1.0 - curvature[index] * state[0, index]),
             control[0, index] * control[1, index] - maximum_control_overlap],
        ))
        worst_inequality = max(worst_inequality, float(np.max(inequalities)))
        wheel_torque[:, index] = (
            shares * control[0, index] * drive_reference
            - control[1, index] * brake_magnitude
        )

    periodicity = float(
        np.max(
            np.abs(
                state[:PERIODIC_STATE_COUNT, nodes - 1]
                - state[:PERIODIC_STATE_COUNT, 0]
            )
        )
    )
    defect = state[:, 1:] - state[:, :-1] - 0.5 * np.diff(path["distance"])[None, :] * (
        derivative[:, 1:] + derivative[:, :-1]
    )
    worst_algebraic = float(np.max(np.abs(residual)))
    worst_dynamics = float(np.max(np.abs(defect)))
    worst_power = float(np.max(np.abs(residual[8])))
    # IPOPT's status alone does not certify physical feasibility. In particular
    # Solved_To_Acceptable_Level can tolerate substantial unscaled defects.
    verified = (
        converged
        and worst_algebraic < 1.0e-3
        and worst_dynamics < 1.0e-4
        and worst_inequality < 1.0e-4
        and periodicity < 1.0e-4
    )
    return PlanarOcpResult(
        status=status,
        converged=verified,
        lap_time=float(state[6, nodes - 1]),
        distance=path["distance"],
        elapsed_time=state[6],
        lateral_error=state[0],
        heading_error=state[1],
        v_x=state[2],
        v_y=state[3],
        yaw_rate=state[4],
        wheel_speeds=wheel_speeds,
        state_of_charge=state[5],
        throttle=control[0],
        brake=control[1],
        steering=control[2],
        normal_loads=loads,
        wheel_torques=wheel_torque,
        pack_current=current[0],
        bus_power=bus,
        periodicity_residual=periodicity,
        maximum_load_residual=worst_algebraic,
        maximum_dynamics_residual=worst_dynamics,
        maximum_inequality_violation=max(0.0, worst_inequality),
        maximum_power_mismatch=worst_power,
        maximum_control_overlap=float(np.max(control[0] * control[1])),
        maximum_lateral_error=float(np.max(np.abs(state[0]))),
        iterations=iterations,
        lam_x=lam_x,
        lam_g=lam_g,
    )
