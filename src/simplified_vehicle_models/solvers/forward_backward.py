from __future__ import annotations

import json
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.formulations.point_mass import (
    evaluate_equilibrium,
    maximum_acceleration_limit,
    maximum_corner_speed,
    maximum_deceleration_limit,
    DriveCache,
    DriveCachePolicy,
)
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.results.traces import LapMode, LapResult, EnergyLapResult, EnergyReport, SolverProgress, StintReport
from simplified_vehicle_models.powertrain.energy_storage import StorageState, evaluate_storage, integrate_storage
from simplified_vehicle_models.powertrain.assembly import FullDrivetrainRuntime, resolve_braking
from simplified_vehicle_models.powertrain.power_plant import evaluate_shaft_power, motor_lut_counters, reset_motor_lut_counters
from simplified_vehicle_models.tracks.types import TrackProfile


class _HintArray(np.ndarray):
    """Private speed array carrying the context that produced it."""

    _hint_context: tuple[int, int, LapMode, tuple[int, ...]]


@dataclass
class DiagnosticsCollector:
    """Opt-in collector for per-iteration and per-lap diagnostics.

    When ``diagnostics_collector`` is supplied to ``solve_lap_with_energy``,
    records are flushed incrementally to the file at ``output_path`` as
    JSON-lines.  When it is ``None`` (the default) the solver behaves
    exactly as before: no extra allocations, no extra I/O, identical
    numerical results.
    """

    output_path: Path | None = None
    lap_number: int = 0
    _records: list[dict] = field(default_factory=list, repr=False)
    _lap_start: float = 0.0
    _solve_lap_time: float = 0.0
    _energy_trace_time: float = 0.0
    _other_time: float = 0.0
    _oracle_call_count: int = 0
    _pass_oracle_calls: list[int] = field(default_factory=list, repr=False)
    _pass_oracle_times: list[float] = field(default_factory=list, repr=False)
    _residual_sequence: list[float] = field(default_factory=list, repr=False)
    _iteration_wall_times: list[float] = field(default_factory=list, repr=False)
    _reconciliation_residuals: list[float] = field(default_factory=list, repr=False)
    _lut_counters_snapshot: list[dict] = field(default_factory=list, repr=False)
    _file_handle: object | None = field(default=None, repr=False)

    def _ensure_file(self) -> None:
        if self.output_path is not None and self._file_handle is None:
            self._file_handle = open(self.output_path, "w")

    def start_lap(self) -> None:
        self._lap_start = time.perf_counter()
        self._solve_lap_time = 0.0
        self._energy_trace_time = 0.0
        self._other_time = 0.0
        self._oracle_call_count = 0
        self._pass_oracle_calls = []
        self._pass_oracle_times = []
        self._residual_sequence = []
        self._iteration_wall_times = []
        self._reconciliation_residuals = []
        self._lut_counters_snapshot = []
        self._ensure_file()

    def record_solve_lap(self, elapsed: float) -> None:
        self._solve_lap_time += elapsed

    def record_energy_trace(self, elapsed: float, call_count: int) -> None:
        self._energy_trace_time += elapsed
        self._oracle_call_count += call_count
        self._pass_oracle_calls.append(call_count)
        self._pass_oracle_times.append(elapsed)

    def record_other(self, elapsed: float) -> None:
        self._other_time += elapsed

    def record_iteration(
        self,
        iteration: int,
        residual: float,
        wall_time: float,
        lut_counters: dict,
    ) -> None:
        self._residual_sequence.append(float(residual))
        self._iteration_wall_times.append(wall_time)
        self._lut_counters_snapshot.append(dict(lut_counters))
        record = {
            "lap": self.lap_number,
            "energy_iteration": iteration,
            "residual": residual,
            "wall_seconds": wall_time,
            "lut_counters": dict(lut_counters),
        }
        self._records.append(record)
        self._flush(record)

    def record_reconciliation(self, residual: float) -> None:
        self._reconciliation_residuals.append(float(residual))

    def finish_lap(self, total_energy_iterations: int) -> dict:
        total = time.perf_counter() - self._lap_start
        summary = {
            "lap": self.lap_number,
            "total_wall_seconds": total,
            "energy_iterations": total_energy_iterations,
            "solve_lap_seconds": self._solve_lap_time,
            "energy_trace_seconds": self._energy_trace_time,
            "other_seconds": self._other_time,
            "total_oracle_calls": self._oracle_call_count,
            "pass_oracle_calls": self._pass_oracle_calls,
            "pass_oracle_times": self._pass_oracle_times,
            "residual_sequence": self._residual_sequence,
            "iteration_wall_times": self._iteration_wall_times,
            "reconciliation_residuals": self._reconciliation_residuals,
            "lut_counters_snapshots": self._lut_counters_snapshot,
        }
        self._flush(summary)
        return summary

    def _flush(self, record: dict) -> None:
        if self._file_handle is not None:
            self._file_handle.write(json.dumps(record) + "\n")
            self._file_handle.flush()

    def get_records(self) -> list[dict]:
        return list(self._records)

    def get_summary(self) -> dict | None:
        if not self._records:
            return None
        return {
            "solve_lap_seconds": self._solve_lap_time,
            "energy_trace_seconds": self._energy_trace_time,
            "other_seconds": self._other_time,
            "total_oracle_calls": self._oracle_call_count,
            "reconciliation_residuals": self._reconciliation_residuals,
            "residual_sequence": self._residual_sequence,
        }


def _hint_context(vehicle, track, mode, shape) -> tuple[int, int, LapMode, tuple[int, ...]]:
    return (id(vehicle), id(track), mode, tuple(shape))


def _hint_is_compatible(hint, context) -> bool:
    hint_context = getattr(hint, "_hint_context", None)
    return hint_context is None or hint_context == context


def solve_lap(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    mode: LapMode,
    tolerance: float = 1.0e-6,
    maximum_iterations: int = 200,
    initial_speed: FloatArray | None = None,
    corner_speed_hint: FloatArray | None = None,
    root_tolerance: float = 1.0e-8,
    drive_cache_policy: DriveCachePolicy = DriveCachePolicy(),
    _storage_trace: FloatArray | None = None,
) -> LapResult:
    """Calculate a minimum-time speed profile on a prescribed path."""

    if tolerance <= 0.0 or not math.isfinite(tolerance):
        raise ValueError("tolerance must be finite and positive")
    if root_tolerance <= 0.0 or not math.isfinite(root_tolerance):
        raise ValueError("root_tolerance must be finite and positive")
    if maximum_iterations < 1:
        raise ValueError("maximum_iterations must be positive")
    if len(track.distance) != len(track.radius):
        raise ValueError("track distance and radius counts must match")
    if len(track.segment_length) != len(track.distance) - 1:
        raise ValueError("track segment count must be one less than node count")

    if _storage_trace is not None and len(_storage_trace) != len(track.distance):
        raise ValueError("storage trace must match track nodes")
    node_count = len(track.distance)
    drive_cache = DriveCache(drive_cache_policy)
    if corner_speed_hint is not None:
        corner_speed_hint = np.asarray(corner_speed_hint, dtype=np.float64)
        if corner_speed_hint.shape != (node_count,):
            raise ValueError("corner speed hint must match track nodes")
        if not np.all(np.isfinite(corner_speed_hint)):
            raise ValueError("corner speed hint must be finite")
    if initial_speed is not None:
        initial_speed = np.asarray(initial_speed, dtype=np.float64)
        if initial_speed.shape != (node_count,):
            raise ValueError("initial speed must match track nodes")
        if not np.all(np.isfinite(initial_speed)) or np.any(initial_speed < 0.0):
            raise ValueError("initial speed must be finite and nonnegative")
    speed_limit = np.asarray(
        [
            maximum_corner_speed(
                vehicle,
                float(radius),
                storage_state=(StorageState(float(_storage_trace[i])) if _storage_trace is not None else None),
                tolerance=root_tolerance,
                hint=(float(corner_speed_hint[i]) if corner_speed_hint is not None else None),
                drive_cache=drive_cache,
            )
            for i, radius in enumerate(track.radius)
        ],
        dtype=np.float64,
    )
    # Transition/profile warm starts are intentionally unsupported for now.  The
    # assembled capability can be non-monotone, so reusing a prior profile can
    # select a different fixed-point crossing.  Keep validating the public hint
    # for forward compatibility, but always begin from the conservative caps.
    speed = speed_limit.copy()
    if mode is LapMode.FLYING:
        _apply_periodic_boundary(speed)
    elif mode is LapMode.STANDING_START:
        speed[0] = 0.0
    else:
        raise ValueError(f"unsupported lap mode {mode!r}")

    converged = False
    maximum_change = math.inf
    iterations = 0
    for iterations in range(1, maximum_iterations + 1):
        previous = speed.copy()
        _forward_sweep(vehicle, track, speed, speed_limit, _storage_trace, root_tolerance, drive_cache)
        if mode is LapMode.FLYING:
            _apply_periodic_boundary(speed)
        else:
            speed[0] = 0.0
        _backward_sweep(vehicle, track, speed, speed_limit, _storage_trace, root_tolerance, drive_cache)
        if mode is LapMode.FLYING:
            _apply_periodic_boundary(speed)
        else:
            speed[0] = 0.0
        np.minimum(speed, speed_limit, out=speed)
        maximum_change = float(np.max(np.abs(speed - previous)))
        if maximum_change <= tolerance:
            converged = True
            break

    return _build_result(
        vehicle,
        track,
        speed,
        converged,
        iterations,
        maximum_change,
        _storage_trace,
        drive_cache,
        _hint_context(vehicle, track, mode, speed.shape),
    )


def _apply_periodic_boundary(speed: FloatArray) -> None:
    boundary_speed = min(float(speed[0]), float(speed[-1]))
    speed[0] = boundary_speed
    speed[-1] = boundary_speed


def _fixed_point_transition(
    reachable: Callable[[float], float],
    upper: float,
    tolerance: float,
) -> float:
    """Return the established conservative fixed-point iteration.

    Do not bisect the residual here: assembled drivetrain capability may be
    non-monotone, so residual bisection can converge to a different crossing.
    """
    candidate = upper
    for _ in range(20):
        updated = min(candidate, reachable(candidate))
        if abs(updated - candidate) <= tolerance:
            return updated
        candidate = updated
    raise RuntimeError("transition did not converge")


def _forward_sweep(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
    speed_limit: FloatArray,
    storage_trace: FloatArray | None = None,
    root_tolerance: float = 1.0e-8,
    drive_cache: DriveCache | None = None,
) -> None:
    for index, segment_length in enumerate(track.segment_length):
        start_speed = float(speed[index])
        upper_speed = float(speed[index + 1])
        cap = float(speed_limit[index + 1])
        if upper_speed <= start_speed:
            speed[index + 1] = min(upper_speed, cap)
            continue
        segment_curvature = max(
            abs(float(track.curvature[index])),
            abs(float(track.curvature[index + 1])),
        )

        def reachable(candidate: float) -> float:
            acceleration_limit = maximum_acceleration_limit(
                vehicle,
                0.5 * (start_speed + candidate),
                segment_curvature,
                StorageState(float(storage_trace[index])) if storage_trace is not None else None,
                drive_cache=drive_cache,
                tolerance=root_tolerance,
            )
            return math.sqrt(
                start_speed * start_speed
                + 2.0 * acceleration_limit * segment_length
            )

        speed[index + 1] = min(
            _fixed_point_transition(reachable, upper_speed, root_tolerance),
            cap,
        )


def _backward_sweep(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
    speed_limit: FloatArray,
    storage_trace: FloatArray | None = None,
    root_tolerance: float = 1.0e-8,
    drive_cache: DriveCache | None = None,
) -> None:
    for index in range(len(track.segment_length) - 1, -1, -1):
        next_speed = float(speed[index + 1])
        upper_speed = float(speed[index])
        cap = float(speed_limit[index])
        if upper_speed <= next_speed:
            speed[index] = min(upper_speed, cap)
            continue
        segment_length = float(track.segment_length[index])
        segment_curvature = max(
            abs(float(track.curvature[index])),
            abs(float(track.curvature[index + 1])),
        )

        def reachable(candidate: float) -> float:
            deceleration_limit = maximum_deceleration_limit(
                vehicle,
                0.5 * (candidate + next_speed),
                segment_curvature,
                StorageState(float(storage_trace[index + 1])) if storage_trace is not None else None,
                drive_cache=drive_cache,
                tolerance=root_tolerance,
            )
            return math.sqrt(
                next_speed * next_speed
                + 2.0 * deceleration_limit * segment_length
            )

        speed[index] = min(
            _fixed_point_transition(reachable, upper_speed, root_tolerance),
            cap,
        )


def _build_result(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
    converged: bool,
    iterations: int,
    maximum_change: float,
    storage_trace: FloatArray | None = None,
    drive_cache: DriveCache | None = None,
    hint_context: tuple[int, int, LapMode, tuple[int, ...]] | None = None,
) -> LapResult:
    start_speed = speed[:-1]
    end_speed = speed[1:]
    acceleration = (
        end_speed * end_speed - start_speed * start_speed
    ) / (2.0 * track.segment_length)
    segment_curvature = np.maximum(
        np.abs(track.curvature[:-1]),
        np.abs(track.curvature[1:]),
    )
    evaluation_speed = 0.5 * (start_speed + end_speed)
    lateral_acceleration = evaluation_speed * evaluation_speed * segment_curvature
    segment_count = len(acceleration)
    throttle = np.zeros(segment_count, dtype=np.float64)
    brake = np.zeros(segment_count, dtype=np.float64)
    loads = np.zeros((segment_count, 4), dtype=np.float64)
    longitudinal_forces = np.zeros((segment_count, 4), dtype=np.float64)
    lateral_forces = np.zeros((segment_count, 4), dtype=np.float64)
    utilization = np.zeros((segment_count, 4), dtype=np.float64)
    drive_limit = np.zeros(segment_count, dtype=np.float64)
    brake_limit = np.zeros(segment_count, dtype=np.float64)
    force_residual = np.zeros(segment_count, dtype=np.float64)
    yaw_residual = np.zeros(segment_count, dtype=np.float64)
    constraints: list[str] = []
    for index, required_acceleration in enumerate(acceleration):
        state = evaluate_equilibrium(
            vehicle,
            float(evaluation_speed[index]),
            float(segment_curvature[index]),
            float(required_acceleration),
            StorageState(float(storage_trace[index])) if storage_trace is not None else None,
            drive_cache,
        )
        wheel_force = float(np.sum(state.longitudinal_forces))
        if wheel_force > 0.0 and state.drive_force_limit > 0.0:
            throttle[index] = min(wheel_force / state.drive_force_limit, 1.0)
        elif wheel_force < 0.0 and state.brake_force_limit > 0.0:
            brake[index] = min(-wheel_force / state.brake_force_limit, 1.0)
        loads[index] = state.normal_loads
        longitudinal_forces[index] = state.longitudinal_forces
        lateral_forces[index] = state.lateral_forces
        utilization[index] = state.combined_utilization
        drive_limit[index] = state.drive_force_limit
        brake_limit[index] = state.brake_force_limit
        force_residual[index] = state.force_balance_residual
        yaw_residual[index] = state.yaw_moment_residual
        constraints.append(state.active_longitudinal_constraint)

    denominator = start_speed + end_speed
    if np.any(denominator <= 0.0):
        raise RuntimeError("a lap segment has zero average speed")
    time_step = 2.0 * track.segment_length / denominator
    elapsed_time = np.concatenate(
        (np.asarray([0.0], dtype=np.float64), np.cumsum(time_step))
    )
    if hint_context is not None:
        speed = speed.view(_HintArray)
        speed._hint_context = hint_context
    arrays = (
        speed,
        elapsed_time,
        acceleration,
        lateral_acceleration,
        throttle,
        brake,
        loads,
        longitudinal_forces,
        lateral_forces,
        utilization,
        drive_limit,
        brake_limit,
        force_residual,
        yaw_residual,
    )
    for array in arrays:
        array.setflags(write=False)
    return LapResult(
        speed=speed,
        elapsed_time=elapsed_time,
        longitudinal_acceleration=acceleration,
        lateral_acceleration=lateral_acceleration,
        throttle=throttle,
        brake=brake,
        normal_loads=loads,
        longitudinal_forces=longitudinal_forces,
        lateral_forces=lateral_forces,
        combined_utilization=utilization,
        drive_force_limit=drive_limit,
        brake_force_limit=brake_limit,
        force_balance_residual=force_residual,
        yaw_moment_residual=yaw_residual,
        active_longitudinal_constraint=tuple(constraints),
        lap_time=float(elapsed_time[-1]),
        converged=converged,
        iterations=iterations,
        maximum_speed_change=maximum_change,
        drive_cache_hits=drive_cache.hits if drive_cache is not None else 0,
        drive_cache_entries=len(drive_cache.values) if drive_cache is not None else 0,
        drive_cache_max_speed_displacement=(drive_cache.max_speed_displacement if drive_cache is not None else 0.0),
        drive_cache_max_soc_displacement=(drive_cache.max_soc_displacement if drive_cache is not None else 0.0),
        motor_forward_queries=motor_lut_counters()["forward"],
        motor_inverse_queries=motor_lut_counters()["inverse"],
        motor_fallback_queries=motor_lut_counters()["fallback"],
    )


def solve_lap_with_energy(
    vehicle,
    track,
    mode,
    initial_state=None,
    tolerance=1e-5,
    maximum_iterations=50,
    regen_enabled=False,
    lap_tolerance=1e-6,
    lap_maximum_iterations=200,
    root_tolerance=1e-8,
    drive_cache_policy=DriveCachePolicy(),
    initial_speed=None,
    corner_speed_hint=None,
    progress_callback: Callable[[SolverProgress], None] | None = None,
    *,
    _progress_lap: int = 1,
    _progress_lap_count: int = 1,
    diagnostics_collector: DiagnosticsCollector | None = None,
):
    """Solve a lap with an explicit, state-coupled outer SOC iteration.

    ``tolerance`` and ``maximum_iterations`` control the SOC fixed point;
    ``lap_*`` controls the inner profile solve.  Speed hints are validated by
    ``solve_lap`` but never used as profile warm starts.
    """
    if tolerance <= 0.0 or maximum_iterations < 1:
        raise ValueError("tolerance must be positive and maximum_iterations must be positive")
    if lap_tolerance <= 0.0 or not math.isfinite(lap_tolerance) or lap_maximum_iterations < 1:
        raise ValueError("lap_tolerance must be finite and positive and lap_maximum_iterations must be positive")
    started = time.perf_counter()
    if diagnostics_collector is not None:
        diagnostics_collector.start_lap()

    def report(stage, energy_iteration=0, profile_iteration=0):
        if progress_callback is not None:
            progress_callback(SolverProgress(
                stage, _progress_lap, _progress_lap_count,
                energy_iteration, profile_iteration,
                time.perf_counter() - started,
            ))

    report("lap_start")
    context = _hint_context(vehicle, track, mode, (len(track.distance),))
    speed_hint = initial_speed if _hint_is_compatible(initial_speed, context) else None
    corner_hint = corner_speed_hint if _hint_is_compatible(corner_speed_hint, context) else None
    storage = getattr(getattr(vehicle.drivetrain, "powertrain", None), "energy_storage", None)
    state0 = (initial_state.state_of_charge if isinstance(initial_state, StorageState)
              else storage.initial_state_of_charge if initial_state is None and storage
              else 1.0 if initial_state is None else float(initial_state))
    trace = np.full(len(track.distance), state0, dtype=np.float64)
    converged = False
    change = math.inf
    for iteration in range(1, maximum_iterations + 1):
        t0 = time.perf_counter()
        lap = solve_lap(
            vehicle,
            track,
            mode,
            tolerance=lap_tolerance,
            maximum_iterations=lap_maximum_iterations,
            initial_speed=speed_hint,
            corner_speed_hint=corner_hint,
            root_tolerance=root_tolerance,
            drive_cache_policy=drive_cache_policy,
            _storage_trace=trace,
        )
        t1 = time.perf_counter()
        if diagnostics_collector is not None:
            diagnostics_collector.record_solve_lap(t1 - t0)
        t2 = time.perf_counter()
        # Snapshot LUT counters before _energy_trace to count oracle calls
        counters_before = motor_lut_counters()
        bus, shaft, voltage, consumed, recovered, nxt = _energy_trace(vehicle, track, lap, trace, regen_enabled)
        t3 = time.perf_counter()
        counters_after = motor_lut_counters()
        oracle_calls_this_pass = (
            (counters_after["forward"] - counters_before["forward"])
            + (counters_after["inverse"] - counters_before["inverse"])
            + (counters_after["fallback"] - counters_before["fallback"])
            + (counters_after["limit_fallback"] - counters_before["limit_fallback"])
        )
        if diagnostics_collector is not None:
            diagnostics_collector.record_energy_trace(t3 - t2, oracle_calls_this_pass)
            # Residual BEFORE the damped update
            residual_before = float(np.max(np.abs(nxt - trace)))
            diagnostics_collector.record_iteration(iteration, residual_before, t3 - t0, counters_after)
        speed_hint = lap.speed
        corner_hint = lap.speed
        change = float(np.max(np.abs(nxt - trace)))
        report("energy_iteration", iteration, lap.iterations)
        trace = 0.5 * (trace + nxt)
        trace[0] = state0
        if change <= tolerance:
            converged = True
            break
    # Reconcile one final profile with the integrated SOC trace.  Keep the
    # profile input and reported state together to a measurable residual;
    # callers must not be told that the energy loop converged otherwise.
    profile_residual = math.inf
    for _ in range(max(1, maximum_iterations - iteration + 1)):
        t0 = time.perf_counter()
        lap = solve_lap(
            vehicle,
            track,
            mode,
            tolerance=lap_tolerance,
            maximum_iterations=lap_maximum_iterations,
            initial_speed=speed_hint,
            corner_speed_hint=corner_hint,
            root_tolerance=root_tolerance,
            drive_cache_policy=drive_cache_policy,
            _storage_trace=trace,
        )
        t1 = time.perf_counter()
        if diagnostics_collector is not None:
            diagnostics_collector.record_solve_lap(t1 - t0)
        counters_before = motor_lut_counters()
        bus, shaft, voltage, consumed, recovered, final_trace = _energy_trace(
            vehicle, track, lap, trace, regen_enabled
        )
        t2 = time.perf_counter()
        counters_after = motor_lut_counters()
        oracle_calls_this_pass = (
            (counters_after["forward"] - counters_before["forward"])
            + (counters_after["inverse"] - counters_before["inverse"])
            + (counters_after["fallback"] - counters_before["fallback"])
            + (counters_after["limit_fallback"] - counters_before["limit_fallback"])
        )
        if diagnostics_collector is not None:
            diagnostics_collector.record_energy_trace(t2 - t1, oracle_calls_this_pass)
            diagnostics_collector.record_reconciliation(
                float(np.max(np.abs(final_trace - trace)))
            )
        profile_residual = float(np.max(np.abs(final_trace - trace)))
        speed_hint = lap.speed
        corner_hint = lap.speed
        if profile_residual <= tolerance:
            trace = final_trace
            break
        if iteration >= maximum_iterations:
            trace = final_trace
            break
        trace = 0.5 * (trace + final_trace)
        trace[0] = state0
        iteration += 1
    energy_converged = converged and profile_residual <= tolerance
    report = EnergyReport(trace, bus, shaft, voltage, tuple(lap.active_longitudinal_constraint), consumed, recovered, consumed-recovered, float(trace[-1]), float(np.nanmin(voltage)) if len(voltage) else math.inf, energy_converged, iteration, max(change, profile_residual), profile_residual)
    report_progress = SolverProgress(
        "lap_complete", _progress_lap, _progress_lap_count, iteration,
        lap.iterations, time.perf_counter() - started,
    )
    if progress_callback is not None:
        progress_callback(report_progress)
    if diagnostics_collector is not None:
        diagnostics_collector.finish_lap(iteration)
    return EnergyLapResult(lap, report)


def _energy_trace(vehicle, track, lap, trace, regen_enabled):
    n = len(track.segment_length)
    bus = np.zeros(n, dtype=np.float64)
    shaft = np.zeros(n, dtype=np.float64)
    voltage = np.full(n, np.nan, dtype=np.float64)
    nxt = np.asarray(trace, dtype=np.float64).copy()
    state = StorageState(float(trace[0]))
    consumed = recovered = 0.0
    storage = getattr(getattr(vehicle.drivetrain, "powertrain", None), "energy_storage", None)
    for i in range(n):
        speed = 0.5 * (float(lap.speed[i]) + float(lap.speed[i + 1]))
        dt = float(lap.elapsed_time[i + 1] - lap.elapsed_time[i])
        wheel_force = float(np.sum(lap.longitudinal_forces[i]))
        if isinstance(vehicle.drivetrain, FullDrivetrainRuntime):
            motor = vehicle.drivetrain.powertrain.power_plant
            transmission = vehicle.drivetrain.drivetrain.transmission
            motor_speed = transmission.motor_speed(speed / vehicle.tire.effective_radius) * 60.0 / (2.0 * math.pi)
            if wheel_force >= 0.0:
                motor_torque = wheel_force * vehicle.tire.effective_radius / (
                    motor.unit_count * transmission.gear_ratio * transmission.gear_efficiency
                )
                operating = evaluate_shaft_power(
                    motor,
                    motor_speed,
                    motor_torque,
                    evaluate_storage(storage, state).bus_voltage,
                )
                # Shaft power is mechanical motor output, never inverter bus power.
                shaft[i] = motor_torque * motor_speed * 2.0 * math.pi / 60.0 * motor.unit_count
                bus[i] = operating.bus_power * motor.unit_count
            elif regen_enabled:
                # Friction absorbs the remainder; only the force actually
                # allocated to regen contributes motor shaft/recovered power.
                br = resolve_braking(
                    vehicle.drivetrain.powertrain,
                    vehicle.drivetrain.drivetrain,
                    speed,
                    state,
                    -wheel_force,
                    -wheel_force,
                    vehicle.tire.effective_radius,
                )
                motor_torque = -br.regen_force_used * vehicle.tire.effective_radius * transmission.gear_efficiency / (
                    motor.unit_count * transmission.gear_ratio
                )
                shaft[i] = motor_torque * motor_speed * 2.0 * math.pi / 60.0 * motor.unit_count
                bus[i] = -br.recovered_bus_power
            else:
                # With regen disabled all braking is friction braking.
                shaft[i] = 0.0
                bus[i] = 0.0
        else:
            shaft[i] = wheel_force * speed
            bus[i] = shaft[i] / vehicle.drivetrain.efficiency if shaft[i] >= 0.0 else 0.0
        if storage:
            voltage[i] = evaluate_storage(storage, state).bus_voltage
            consumed += max(bus[i], 0.0) * dt
            previous_soc = state.state_of_charge
            state = integrate_storage(storage, state, float(bus[i]), dt)
            nxt[i + 1] = state.state_of_charge
            if bus[i] < 0.0:
                accepted_energy = max(
                    0.0,
                    (state.state_of_charge - previous_soc)
                    * storage.pack_capacity_joules,
                )
                recovered += accepted_energy
                bus[i] = -accepted_energy / dt if dt > 0.0 else 0.0
    return bus, shaft, voltage, consumed, recovered, nxt


def _fixed_profile_energy_result(vehicle, track, result, initial_state, regen_enabled):
    """Integrate energy on a solved lap profile without re-solving its speed."""
    storage = getattr(getattr(vehicle.drivetrain, "powertrain", None), "energy_storage", None)
    if isinstance(initial_state, StorageState):
        state0 = initial_state.state_of_charge
    elif initial_state is None and storage is not None:
        state0 = storage.initial_state_of_charge
    elif initial_state is None:
        state0 = 1.0
    else:
        state0 = float(initial_state)
    trace = np.full(len(track.distance), state0, dtype=np.float64)
    bus, shaft, voltage, consumed, recovered, final_trace = _energy_trace(
        vehicle, track, result.lap, trace, regen_enabled
    )
    maximum_change = float(np.max(np.abs(final_trace - trace)))
    report = EnergyReport(
        final_trace,
        bus,
        shaft,
        voltage,
        tuple(result.lap.active_longitudinal_constraint),
        consumed,
        recovered,
        consumed - recovered,
        float(final_trace[-1]),
        float(np.nanmin(voltage)) if len(voltage) else math.inf,
        True,
        1,
        maximum_change,
        0.0,
        True,
    )
    return EnergyLapResult(result.lap, report)


def solve_stint(
    vehicle,
    track,
    mode=LapMode.FLYING,
    laps=1,
    initial_state=None,
    reuse_profile=False,
    lap_tolerance=1e-6,
    lap_maximum_iterations=200,
    root_tolerance=1e-8,
    drive_cache_policy=DriveCachePolicy(),
    progress_callback: Callable[[SolverProgress], None] | None = None,
    **kwargs,
):
    """Repeat laps while carrying the final battery state into the next lap.

    With ``reuse_profile=True``, only the first lap is state-coupled. Later
    laps reuse its speed/force profile and integrate energy against the carried
    SOC, which is substantially faster but is a fixed-profile approximation.
    """
    if laps < 1:
        raise ValueError("laps must be positive")
    results = []
    state = initial_state
    regen_enabled = kwargs.get("regen_enabled", False)
    started = time.perf_counter()
    def forward_progress(event):
        if progress_callback is not None:
            progress_callback(SolverProgress(
                event.stage, event.lap, laps, event.energy_iteration,
                event.profile_iteration, time.perf_counter() - started,
            ))
    speed_hint = kwargs.pop("initial_speed", None)
    corner_hint = kwargs.pop("corner_speed_hint", None)
    for lap_index in range(laps):
        if lap_index == 0 or not reuse_profile:
            result = solve_lap_with_energy(
                vehicle,
                track,
                mode,
                state,
                lap_tolerance=lap_tolerance,
                lap_maximum_iterations=lap_maximum_iterations,
                root_tolerance=root_tolerance,
                drive_cache_policy=drive_cache_policy,
                initial_speed=speed_hint,
                corner_speed_hint=corner_hint,
                progress_callback=forward_progress,
                _progress_lap=lap_index + 1,
                _progress_lap_count=laps,
                **kwargs,
            )
            speed_hint = result.lap.speed
            corner_hint = result.lap.speed
        else:
            forward_progress(SolverProgress(
                "lap_start", lap_index + 1, laps, 0, 0,
                time.perf_counter() - started,
            ))
            result = _fixed_profile_energy_result(
                vehicle, track, results[0], state, regen_enabled
            )
            forward_progress(SolverProgress(
                "lap_complete", lap_index + 1, laps,
                result.energy.energy_iterations, result.lap.iterations,
                time.perf_counter() - started,
            ))
        results.append(result)
        state = result.energy.final_state_of_charge
    times = np.asarray([r.lap_time for r in results])
    energies = np.asarray([r.energy.net_energy for r in results])
    total_consumed = sum(r.energy.consumed_energy for r in results)
    total_recovered = sum(r.energy.recovered_energy for r in results)
    total_net = sum(r.energy.net_energy for r in results)
    storage = getattr(getattr(vehicle.drivetrain, "powertrain", None), "energy_storage", None)
    if storage:
        # Count only energy still available after the completed stint.  The
        # curve is cell energy in Wh, and interpolation preserves its SOC
        # direction even when the stint started below full charge.
        axis = np.asarray(storage.state_of_charge_axis)
        energy = np.asarray(storage.energy_axis)
        remaining = float(np.interp(float(state), axis, energy) - energy[0])
        usable = max(0.0, remaining) * 3600.0 * storage.parallel_count
    else:
        usable = math.inf
    # Net lap energy can be zero/negative for a regen-dominated lap; never
    # report a negative endurance count in that direction.
    mean_energy = float(np.mean(energies))
    lap_count = usable / mean_energy if usable < math.inf and mean_energy > 0.0 else math.inf
    if progress_callback is not None:
        progress_callback(SolverProgress(
            "stint_complete", laps, laps, results[-1].energy.energy_iterations,
            results[-1].lap.iterations, time.perf_counter() - started,
        ))
    return StintReport(tuple(results), times, energies, float(state), total_consumed, total_recovered, total_net, lap_count)
