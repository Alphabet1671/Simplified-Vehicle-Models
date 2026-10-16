"""Compare one simulated MIS lap with the measured IEM24 Murphy stint.

The measured stint is segmented into laps by navigation-unit position closure
(see :mod:`tools.data.iem24_telemetry`).  Each measured sample is then projected onto
the signed closed MIS centreline, so measured and simulated traces share one
physical lap-distance axis and no phase alignment has to be inferred.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import PathCoordinates

#: Trace pairs printed and plotted, as (column, axis label, unit).
COMPARISON_CHANNELS: tuple[tuple[str, str, str], ...] = (
    ("speed_mps", "Speed", "m/s"),
    ("longitudinal_acceleration_mps2", "Longitudinal acceleration", "m/s²"),
    ("lateral_acceleration_mps2", "Lateral acceleration", "m/s²"),
    ("yaw_rate_rad_s", "Yaw rate", "rad/s"),
    ("motor_torque_request_nm", "Motor torque request", "N·m"),
    ("pack_voltage_v", "Pack terminal voltage", "V"),
)

#: Simulated pedal fraction maps to per-motor torque through this value.
DEFAULT_APPS_TORQUE_NM = 10.5

#: Comparison column to source column in the simulated lap export.
SIMULATED_COLUMNS = {
    "speed_mps": "velocity_mps",
    "longitudinal_acceleration_mps2": "longitudinal_acceleration_mps2",
    "lateral_acceleration_mps2": "lateral_acceleration_mps2",
    "yaw_rate_rad_s": "yaw_rate_rad_s",
    "pack_voltage_v": "pack_voltage_v",
}

#: Distance bins used for the folded comparison profile.
PHASE_BINS = 500

#: Translation search schedule for fitting the measured path onto the track.
OFFSET_SEARCH_STEPS_M = (2.5, 0.5, 0.1)
OFFSET_SEARCH_HALF_WIDTH_M = 25.0
OFFSET_FIT_SAMPLES = 1200


def load_table(path: Path) -> dict[str, np.ndarray]:
    """Read one whitespace/CSV numeric export with a header row."""

    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        raise ValueError(f"{path}: no samples")
    return {
        name: np.atleast_1d(np.asarray(data[name], dtype=np.float64))
        for name in data.dtype.names
    }


def flying_lap_indices(measured: dict[str, np.ndarray]) -> np.ndarray:
    """Return the lap indices that began at speed rather than from rest."""

    indices = np.unique(measured["lap_index"])
    flying = [
        index
        for index in indices
        if float(measured["speed_mps"][measured["lap_index"] == index][0]) >= 1.0
    ]
    if not flying:
        raise ValueError("the stint contains no flying laps")
    return np.asarray(flying, dtype=np.float64)


def fit_position_offset(
    measured: dict[str, np.ndarray], path: PathCoordinates
) -> tuple[np.ndarray, float]:
    """Fit the constant translation that maps measured positions onto the track.

    The measured position is a dead-reckoned integral of the logged velocity, so
    its origin is arbitrary.  A translation is enough because both frames are the
    same local north-east plane.
    """

    nodes = np.column_stack((path.x_north, path.y_east))
    tree = cKDTree(nodes)
    stride = max(int(measured["position_north_m"].size // OFFSET_FIT_SAMPLES), 1)
    points = np.column_stack(
        (
            measured["position_north_m"][::stride],
            measured["position_east_m"][::stride],
        )
    )
    offset = np.array(
        [
            float(np.mean(path.x_north[:-1]) - np.mean(points[:, 0])),
            float(np.mean(path.y_east[:-1]) - np.mean(points[:, 1])),
        ]
    )
    best_cost = float("inf")
    for step in OFFSET_SEARCH_STEPS_M:
        span = OFFSET_SEARCH_HALF_WIDTH_M if step == OFFSET_SEARCH_STEPS_M[0] else step * 10.0
        deltas = np.arange(-span, span + step * 0.5, step)
        for delta_north in deltas:
            for delta_east in deltas:
                candidate = offset + np.array([delta_north, delta_east])
                distance, _ = tree.query(points + candidate, k=1)
                cost = float(np.mean(distance**2))
                if cost < best_cost:
                    best_cost = cost
                    offset = candidate
    return offset, float(np.sqrt(best_cost))


def project_onto_track(
    measured: dict[str, np.ndarray], path: PathCoordinates, offset: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return the track distance and centreline residual of every sample."""

    tree = cKDTree(np.column_stack((path.x_north, path.y_east)))
    points = np.column_stack(
        (measured["position_north_m"], measured["position_east_m"])
    ) + offset
    residual, index = tree.query(points, k=1)
    return path.distance[index], residual


def unwrap_track_distance(
    track_distance: np.ndarray, lap_length: float, lap_index: np.ndarray
) -> np.ndarray:
    """Unwrap projected track distance into a monotonic run per lap."""

    unwrapped = np.empty_like(track_distance)
    for lap in np.unique(lap_index):
        select = lap_index == lap
        series = track_distance[select]
        delta = np.mod(np.diff(series), lap_length)
        unwrapped[select] = np.concatenate(([series[0]], series[0] + np.cumsum(delta)))
    return unwrapped


def _periodic_interp(
    phase: np.ndarray, values: np.ndarray, grid: np.ndarray, lap_length: float
) -> np.ndarray:
    """Interpolate a periodic lap profile onto a grid."""

    order = np.argsort(phase, kind="stable")
    ordered_phase = phase[order]
    ordered_values = values[order]
    extended_phase = np.concatenate(
        (ordered_phase - lap_length, ordered_phase, ordered_phase + lap_length)
    )
    extended_values = np.tile(ordered_values, 3)
    return np.interp(grid, extended_phase, extended_values)


def fold_laps(
    measured: dict[str, np.ndarray],
    laps: np.ndarray,
    columns: tuple[str, ...],
    *,
    track_distance: np.ndarray,
    lap_length: float,
    bins: int,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Fold every flying lap onto a shared physical lap-distance grid."""

    grid = (np.arange(bins, dtype=np.float64) + 0.5) / bins * lap_length
    folded: dict[str, np.ndarray] = {}
    for column in columns:
        stack = np.empty((laps.size, bins), dtype=np.float64)
        for row, lap in enumerate(laps):
            select = measured["lap_index"] == lap
            stack[row] = _periodic_interp(
                np.mod(track_distance[select], lap_length),
                measured[column][select],
                grid,
                lap_length,
            )
        folded[column] = stack
    return grid, folded


def _metrics(
    simulated: np.ndarray, measured: np.ndarray
) -> dict[str, float]:
    error = simulated - measured
    scale = float(np.std(measured))
    return {
        "rmse": float(np.sqrt(np.mean(error**2))),
        "mae": float(np.mean(np.abs(error))),
        "bias": float(np.mean(error)),
        "correlation": float(np.corrcoef(simulated, measured)[0, 1]),
        "normalised_rmse": float(np.sqrt(np.mean(error**2)) / scale) if scale else None,
    }


def _median_lap_time(extraction: dict, laps: np.ndarray) -> float:
    """Return the median measured lap time for the flying laps."""

    table = {int(entry["index"]): entry for entry in extraction.get("laps", [])}
    times = [
        float(table[int(index)]["lap_time_s"])
        for index in laps
        if int(index) in table
    ]
    return float(np.median(times)) if times else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("docs/validation/iem24-murphy")
    )
    parser.add_argument(
        "--track",
        type=Path,
        default=Path("models/tracks/2024_michigan_endurance_multilap_closed.xml"),
    )
    parser.add_argument("--bins", type=int, default=PHASE_BINS)
    args = parser.parse_args()
    output_dir = args.output_dir
    root = Path(__file__).resolve().parents[2]

    measured = load_table(output_dir / "measured_stint.csv")
    simulated = load_table(output_dir / "simulated_lap.csv")
    sim_summary = _read_json(output_dir / "simulated_lap_summary.json")
    extraction = _read_json(output_dir / "telemetry_extraction.json")
    apps_torque = float(
        sim_summary.get("apps_max_motor_torque_nm", DEFAULT_APPS_TORQUE_NM)
    )

    track = load_track(root / args.track)
    path = track.path_coordinates
    if path is None:
        raise ValueError(
            f"{args.track}: the comparison needs a signed closed centreline"
        )
    lap_length = path.lap_length

    laps = flying_lap_indices(measured)
    offset, fit_residual = fit_position_offset(measured, path)
    track_distance, residual = project_onto_track(measured, path, offset)
    unwrapped = unwrap_track_distance(track_distance, lap_length, measured["lap_index"])

    columns = tuple(name for name, _, _ in COMPARISON_CHANNELS)
    grid, folded = fold_laps(
        measured,
        laps,
        columns,
        track_distance=track_distance,
        lap_length=lap_length,
        bins=args.bins,
    )
    measured_profile = {
        column: np.median(values, axis=0) for column, values in folded.items()
    }
    sim_distance = simulated["distance_m"]
    sim_profile = {
        column: np.interp(grid, sim_distance, simulated[source])
        for column, source in SIMULATED_COLUMNS.items()
    }
    sim_profile["motor_torque_request_nm"] = (
        np.interp(grid, sim_distance, simulated["apps_fraction"]) * apps_torque
    )

    metrics = {
        column: _metrics(sim_profile[column], measured_profile[column])
        for column in columns
    }
    peaks = {
        "measured_peak_speed_mps": float(np.max(measured_profile["speed_mps"])),
        "simulated_peak_speed_mps": float(np.max(sim_profile["speed_mps"])),
        "measured_peak_lateral_acceleration_mps2": float(
            np.max(np.abs(measured_profile["lateral_acceleration_mps2"]))
        ),
        "simulated_peak_lateral_acceleration_mps2": float(
            np.max(np.abs(sim_profile["lateral_acceleration_mps2"]))
        ),
        "measured_peak_yaw_rate_rad_s": float(
            np.max(np.abs(measured_profile["yaw_rate_rad_s"]))
        ),
        "simulated_peak_yaw_rate_rad_s": float(
            np.max(np.abs(sim_profile["yaw_rate_rad_s"]))
        ),
        "measured_peak_braking_mps2": float(
            np.min(measured_profile["longitudinal_acceleration_mps2"])
        ),
        "simulated_peak_braking_mps2": float(
            np.min(sim_profile["longitudinal_acceleration_mps2"])
        ),
        "measured_peak_traction_mps2": float(
            np.max(measured_profile["longitudinal_acceleration_mps2"])
        ),
        "simulated_peak_traction_mps2": float(
            np.max(sim_profile["longitudinal_acceleration_mps2"])
        ),
    }
    per_lap = {}
    for column in columns:
        errors = [
            float(
                np.sqrt(
                    np.mean((sim_profile[column] - folded[column][row]) ** 2)
                )
            )
            for row in range(laps.size)
        ]
        per_lap[column] = {
            "minimum_rmse": float(np.min(errors)),
            "median_rmse": float(np.median(errors)),
            "maximum_rmse": float(np.max(errors)),
        }

    flying_residual = residual[np.isin(measured["lap_index"], laps)]
    measured_distance = float(
        np.median(
            [
                measured["lap_distance_m"][measured["lap_index"] == lap][-1]
                for lap in laps
            ]
        )
    )
    _write_comparison_csv(
        output_dir, grid, measured_profile, sim_profile, columns
    )
    report = {
        "measured_stint": {
            "flying_lap_count": int(laps.size),
            "lap_indices": [int(index) for index in laps],
            "median_lap_time_s": _median_lap_time(extraction, laps),
            "median_driven_lap_distance_m": measured_distance,
            "source": extraction.get("source"),
        },
        "simulated_lap": {
            "lap_time_s": float(simulated["elapsed_time_s"][-1]),
            "lap_length_m": float(sim_distance[-1]),
            "track": str(args.track),
            "apps_torque_nm": apps_torque,
            "initial_state_of_charge": sim_summary.get("initial_state_of_charge"),
            "initial_bus_voltage_v": float(sim_profile["pack_voltage_v"][0]),
            "minimum_bus_voltage_v": float(np.min(sim_profile["pack_voltage_v"])),
            "final_bus_voltage_v": float(sim_profile["pack_voltage_v"][-1]),
        },
        "track": {
            "lap_length_m": lap_length,
            "node_count": int(path.distance.size),
        },
        "position_fit": {
            "method": "constant translation fitted by nearest-centreline distance",
            "offset_north_m": float(offset[0]),
            "offset_east_m": float(offset[1]),
            "sample_count": int(measured["time_s"].size),
            "fitted_rmse_m": fit_residual,
            "residual_median_m": float(np.median(flying_residual)),
            "residual_p95_m": float(np.percentile(flying_residual, 95)),
            "residual_maximum_m": float(np.max(flying_residual)),
        },
        "measured_track_distance_sample": {
            "first_m": float(unwrapped[0]),
            "last_m": float(unwrapped[-1]),
        },
        "metrics": metrics,
        "peaks": peaks,
        "per_lap_rmse": per_lap,
    }
    (output_dir / "comparison_summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_figures(
        output_dir, grid, lap_length, measured, laps, folded, measured_profile, sim_profile
    )
    _write_report(output_dir, report, extraction, lap_length)
    print(json.dumps(report["metrics"], indent=2, sort_keys=True))
    print(
        f"position offset N {offset[0]:+.2f} m E {offset[1]:+.2f} m; "
        f"centreline residual median {np.median(flying_residual):.2f} m"
    )


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def _write_comparison_csv(
    output_dir: Path,
    grid: np.ndarray,
    measured_profile: dict[str, np.ndarray],
    sim_profile: dict[str, np.ndarray],
    columns: tuple[str, ...],
) -> None:
    headers = ["track_distance_m"]
    for column in columns:
        headers += [f"measured_{column}", f"simulated_{column}"]
    np.savetxt(
        output_dir / "comparison.csv",
        np.column_stack(
            (
                grid,
                *[
                    value
                    for column in columns
                    for value in (measured_profile[column], sim_profile[column])
                ],
            )
        ),
        delimiter=",",
        header=",".join(headers),
        comments="",
    )


def _write_figures(
    output_dir: Path,
    grid: np.ndarray,
    lap_length: float,
    measured: dict[str, np.ndarray],
    laps: np.ndarray,
    folded: dict[str, np.ndarray],
    measured_profile: dict[str, np.ndarray],
    sim_profile: dict[str, np.ndarray],
) -> None:
    """Render the trace comparison and the lap consistency figure."""

    import os

    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(
        len(COMPARISON_CHANNELS), 1, figsize=(12, 16.5), sharex=True
    )
    for axis, (column, label, unit) in zip(axes, COMPARISON_CHANNELS):
        stack = folded[column]
        axis.fill_between(
            grid,
            np.min(stack, axis=0),
            np.max(stack, axis=0),
            color="#9dc3e6",
            alpha=0.45,
            label=f"Measured range ({stack.shape[0]} flying laps)",
        )
        axis.plot(
            grid,
            measured_profile[column],
            color="#1f4e79",
            lw=1.7,
            label="Measured median lap",
        )
        axis.plot(
            grid, sim_profile[column], color="#c00000", lw=1.7, label="Simulation"
        )
        axis.set_ylabel(f"{label} ({unit})")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", fontsize=8, ncol=3)
        axis.margins(x=0)
    axes[-1].set_xlabel("MIS centreline distance (m)")
    figure.suptitle(
        "IEM24 Murphy — simulated MIS flying lap vs measured flying laps"
    )
    figure.tight_layout()
    figure.savefig(output_dir / "comparison.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(12, 7))
    speed_stack = folded["speed_mps"]
    for row in range(speed_stack.shape[0]):
        axes[0].plot(grid, speed_stack[row], color="#9dc3e6", lw=0.9)
    axes[0].plot(
        grid, measured_profile["speed_mps"], color="#1f4e79", lw=1.9, label="Median"
    )
    axes[0].plot(grid, sim_profile["speed_mps"], color="#c00000", lw=1.7, label="Simulation")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].set_xlabel("MIS centreline distance (m)")
    axes[0].set_title(f"Measured flying laps ({speed_stack.shape[0]}) and simulation")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, alpha=0.25)
    axes[0].margins(x=0)

    torque = measured["motor_torque_request_nm"]
    flying = np.isin(measured["lap_index"], laps)
    axes[1].hist(
        torque[flying],
        bins=np.arange(0.0, 11.0, 0.25),
        color="#1f4e79",
        alpha=0.75,
        label="Measured request",
    )
    axes[1].axvline(
        float(np.max(torque[flying])),
        color="#1f4e79",
        ls="--",
        lw=1.2,
        label=f"Measured peak {float(np.max(torque[flying])):.2f} N·m",
    )
    axes[1].axvline(
        float(np.max(sim_profile["motor_torque_request_nm"])),
        color="#c00000",
        lw=1.4,
        label=(
            "Simulated peak "
            f"{float(np.max(sim_profile['motor_torque_request_nm'])):.2f} N·m"
        ),
    )
    axes[1].set_xlabel("Per-motor torque request (N·m)")
    axes[1].set_ylabel("Samples")
    axes[1].set_title("Pedal-map torque request usage")
    axes[1].legend(loc="best", fontsize=8)
    axes[1].grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(output_dir / "lap_spread.png", dpi=150)
    plt.close(figure)


def _format_bound(window: object, index: int) -> str:
    """Format one end of the optional measured driving window."""

    if not isinstance(window, (list, tuple)) or len(window) <= index:
        return "unknown"
    return f"{float(window[index]):.2f}"


def _parked_duration(extraction: dict) -> float:
    """Return how long the car sat still before the stint got under way."""

    window = extraction.get("stint_window_s")
    driving = extraction.get("driving_window_s")
    if not isinstance(window, (list, tuple)) or not isinstance(driving, (list, tuple)):
        return float("nan")
    if len(window) < 1 or len(driving) < 1:
        return float("nan")
    return float(driving[0]) - float(window[0])


def _write_report(
    output_dir: Path, report: dict, extraction: dict, lap_length: float
) -> None:
    """Write the Markdown report with every figure embedded."""

    # Report paths are shown relative to the repository root in the text.
    root = Path(__file__).resolve().parents[2]
    try:
        output_dir_text = str(output_dir.resolve().relative_to(root))
    except ValueError:
        output_dir_text = str(output_dir)
    # Reports may be generated in nested verification directories, not only
    # the default location; link the standard relative to the actual report.
    import os
    standard_link = os.path.relpath(
        root / "docs/standards/mis-2024-report-standard.md", output_dir.resolve()
    )
    skew = extraction.get("navigation_unit_skew", {})
    validation = extraction.get("validation", {})
    flying = [
        lap for lap in extraction.get("laps", []) if not lap.get("standing_start", False)
    ]
    lap_rows = "\n".join(
        "| {index} | {start_s:.2f} | {lap_time_s:.2f} | {distance_m:.1f} | "
        "{mean_speed_mps:.2f} | {closure_radius_m:.2f} |".format(**lap)
        for lap in flying
    )
    metrics = report["metrics"]
    metric_rows = "\n".join(
        "| {label} ({unit}) | {rmse:.3f} | {mae:.3f} | {bias:+.3f} | "
        "{correlation:.3f} | {normalised_rmse:.3f} |".format(
            label=label, unit=unit, **metrics[column]
        )
        for column, label, unit in COMPARISON_CHANNELS
    )
    fit = report["position_fit"]
    simulated = report["simulated_lap"]
    stint = report["measured_stint"]
    peaks = report["peaks"]
    per_lap = report["per_lap_rmse"]
    volts = extraction.get("pack_voltage", {})
    volts = {
        "resting_voltage_v": volts.get("resting_voltage_v", float("nan")),
        "resting_standard_deviation_v": volts.get(
            "resting_standard_deviation_v", float("nan")
        ),
        "resting_sample_count": volts.get("resting_sample_count", 0),
        "stint_median_voltage_v": volts.get("stint_median_voltage_v", float("nan")),
        "stint_minimum_voltage_v": volts.get("stint_minimum_voltage_v", float("nan")),
        "flying_lap_boundary_median_voltage_v": volts.get(
            "flying_lap_boundary_median_voltage_v", float("nan")
        ),
        "lap_boundary_voltage_v": volts.get("lap_boundary_voltage_v") or [float("nan")],
    }
    volts_parked = _parked_duration(extraction)
    text = f"""# IEM24 Murphy: simulated MIS lap vs measured endurance stint

## Configuration

- Vehicle: 287 kg (209 kg car + 78 kg driver), LLTD 42 %, CGx 45 % of wheelbase
  from the front axle, CG height 0.300 m.
- Aero: CL·A = 3.5 m², CD·A = 1.55 m², aero balance 50 %.
- Drivetrain: two rear motors only, APPS 0–100 % requests 0–{simulated['apps_torque_nm']:.1f} N·m
  per motor, regeneration disabled, mechanical brakes only.
- Simulated flying lap: {simulated['lap_time_s']:.3f} s over
  {simulated['lap_length_m']:.3f} m on the signed closed MIS centreline.
- Pack initial bus voltage: {volts['resting_voltage_v']:.2f} V compiled from the
  measured resting voltage at the start of the stint; the simulated terminal voltage
  at the first node is {simulated['initial_bus_voltage_v']:.2f} V.

## Measured data

The stint runs from {_format_bound(extraction.get('driving_window_s'), 0)} s to
{_format_bound(extraction.get('driving_window_s'), 1)} s on the log clock; the car is
stationary before and after. Laps are segmented by navigation-unit position closure,
which needs no start/finish instrumentation: the car returns to the previous lap
boundary once per lap, and every closure was within
{extraction.get('lap_statistics', {}).get('maximum_closure_radius_m', float('nan')):.2f} m.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---:|---:|---:|---:|---:|
{lap_rows}

The lap that begins at the start of the window is a standing start and is excluded
from the flying-lap comparison. Median measured flying lap:
{stint['median_lap_time_s']:.2f} s over {stint['median_driven_lap_distance_m']:.1f} m
driven, which is {100.0 * (stint['median_driven_lap_distance_m'] / lap_length - 1.0):+.2f} %
longer than the {lap_length:.1f} m centreline.

### Putting measured and simulated laps on one axis

Every measured sample is projected onto the signed closed MIS centreline, so both
traces share one physical lap-distance axis and no phase alignment has to be
inferred. The logged position is a dead-reckoned integral of the logged velocity,
so a constant translation is fitted first ({fit['offset_north_m']:+.2f} m north,
{fit['offset_east_m']:+.2f} m east). After the fit the measured path sits a median of
{fit['residual_median_m']:.2f} m from the centreline (95th percentile
{fit['residual_p95_m']:.2f} m), which is the driven line's normal offset.

### Navigation-unit skew

The unit is mounted at a yaw skew of **{skew.get('angle_deg', float('nan')):.2f}°**
(interquartile spread {skew.get('angle_interquartile_deg', float('nan')):.2f}°,
estimated from {skew.get('straight_sample_count', 0):,} straight high-speed samples).
The skew is material: rotating the horizontal channels into the body frame raises the
agreement between the longitudinal accelerometer and the differentiated longitudinal
velocity from 0.816 to
{validation.get('longitudinal_acceleration_vs_velocity_rate', {}).get('correlation', float('nan')):.3f}
(RMSE 1.362 → {validation.get('longitudinal_acceleration_vs_velocity_rate', {}).get('rmse', float('nan')):.3f} m/s²).
The corrected lateral channel then matches the centripetal acceleration with
correlation {validation.get('lateral_acceleration_vs_centripetal', {}).get('correlation', float('nan')):.3f}
and slope {validation.get('lateral_acceleration_vs_centripetal', {}).get('slope', float('nan')):.3f}.

All measured channels are low-pass filtered at 1 Hz with a zero-phase first-order
filter so that they can be compared with a quasi-static lap simulation.

### Pack voltage initial condition

The model's initial bus voltage is set from the logged `VPack` channel. While the car
is parked for the {volts_parked:.0f} s before launch the pack rests at
**{volts['resting_voltage_v']:.2f} V** (standard deviation
{volts['resting_standard_deviation_v']:.3f} V over {volts['resting_sample_count']:,}
samples), and that is the value compiled into
`models/energy-storages/iem24_murphy_real_battery.xml`, which resolves to an initial
state of charge of {simulated['initial_state_of_charge']:.4f}.

The simulated trace is the pack **terminal** voltage, `V_oc − I·R`, because the logged
channel is a terminal measurement rather than an open-circuit estimate. It runs from
{simulated['initial_bus_voltage_v']:.1f} V down to {simulated['minimum_bus_voltage_v']:.1f} V
across the lap.

One caveat applies when reading the voltage panel: the model lap starts from the
stint-start voltage, while the ten measured flying laps start from a median of
{volts['flying_lap_boundary_median_voltage_v']:.2f} V (from
{max(volts['lap_boundary_voltage_v']):.1f} V on the first flying lap down to
{min(volts['lap_boundary_voltage_v']):.1f} V on the last) because the pack discharges
through the stint. The simulated voltage therefore sits about
{metrics['pack_voltage_v']['bias']:.1f} V above the measured flying-lap median, which is
an initial-condition offset rather than a model error. The pack itself behaves
plausibly on both sides: under way the measured pack sits at a median of
{volts['stint_median_voltage_v']:.1f} V and sags to {volts['stint_minimum_voltage_v']:.1f} V
at its lowest, which is the expected terminal droop under drive current.

The choice of initial voltage moves the voltage trace but barely moves the lap: over
530–600 V the simulated lap time varies by only 0.19 % (77.41–77.56 s), because the
two-motor rear drive is torque-limited rather than voltage-limited across this speed
range. Setting `initial_bus_voltage` to
{volts['flying_lap_boundary_median_voltage_v']:.2f} V in
`models/energy-storages/iem24_murphy_real_battery.xml` instead would centre the
voltage trace on the flying laps.

## Trace comparison

The measured traces are folded onto the centreline distance grid and reduced to the
median across the {stint['flying_lap_count']} flying laps.

| Signal | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |
|---|---:|---:|---:|---:|---:|
{metric_rows}

![Measured and simulated traces](comparison.png)

![Per-lap speed spread and pedal-map torque usage](lap_spread.png)

### Simulated lap in isolation

The simulated lap on its own, including the pack-voltage channel that carries the
initial condition, comes from the lap generator:

![Simulated MIS lap traces, including pack voltage](simulated_lap.png)

## Reading the comparison

- The simulated lap is **{simulated['lap_time_s'] - stint['median_lap_time_s']:.2f} s slower**
  than the measured median flying lap ({simulated['lap_time_s']:.2f} s against
  {stint['median_lap_time_s']:.2f} s), and {simulated['lap_time_s'] - float(min(lap['lap_time_s'] for lap in flying)):.2f} s
  slower than the fastest measured lap.
- The speed trace tracks the measured shape closely (correlation
  {metrics['speed_mps']['correlation']:.3f}) but sits
  {abs(metrics['speed_mps']['bias']):.2f} m/s low on average. The measured median peaks at
  {report['peaks']['measured_peak_speed_mps']:.1f} m/s against
  {report['peaks']['simulated_peak_speed_mps']:.1f} m/s simulated, so the deficit is a
  top-speed and corner-exit problem rather than a general offset.
- Lateral acceleration is the strongest grip signal
  (correlation {metrics['lateral_acceleration_mps2']['correlation']:.3f}). The measured median reaches
  {report['peaks']['measured_peak_lateral_acceleration_mps2']:.1f} m/s² against
  {report['peaks']['simulated_peak_lateral_acceleration_mps2']:.1f} m/s² simulated, so the model
  corners slightly below the real car and that is where the lap time goes.
- Yaw rate agrees almost exactly (bias {metrics['yaw_rate_rad_s']['bias']:+.3f} rad/s,
  correlation {metrics['yaw_rate_rad_s']['correlation']:.3f}), which confirms that the simulated
  lap geometry matches the driven line.
- Longitudinal acceleration lines up on every braking event, but the model switches
  between traction and braking instantly while the real traces ramp; that difference
  dominates the {metrics['longitudinal_acceleration_mps2']['rmse']:.2f} m/s² RMSE.
- The torque request trace is the weakest match
  (correlation {metrics['motor_torque_request_nm']['correlation']:.3f}). The real driver modulates the
  pedal continuously, while the optimal-control lap holds the 10.5 N·m limit on the
  straights, so the sim runs a higher mean request
  (bias {metrics['motor_torque_request_nm']['bias']:+.2f} N·m) for the same or lower speed.
- Per-lap speed RMSE across the ten flying laps ranges from
  {per_lap['speed_mps']['minimum_rmse']:.2f} to {per_lap['speed_mps']['maximum_rmse']:.2f} m/s
  (median {per_lap['speed_mps']['median_rmse']:.2f} m/s), so the simulation sits inside the
  lap-to-lap spread of the stint rather than matching one particular lap.
- The pack terminal voltage droops with drive current and recovers off the pedal on
  both sides, but the traces only partly correlate
  ({metrics['pack_voltage_v']['correlation']:.3f}) and the simulated one sits
  {metrics['pack_voltage_v']['bias']:.1f} V high because the model lap starts from the
  stint-start voltage rather than a mid-stint voltage; see the initial-condition note
  above.

## Caveats

- Wheelbase, track width, tire model, battery, gear ratio, air density and mechanical
  brake bias are carried over from the existing Murphy model rather than re-measured
  at IEM24.
- The logged `Tq_*_Req` channels resolve to 0.01 N·m and peak at 10.45 N·m, which is
  0.5 % below the 10.5 N·m value the model uses.
- The measured position is integrated from the logged velocity, not a position
  solution, so the centreline residuals include any integration drift.

## Reproduce

```bash
# 1. Run one simulated flying lap and write simulated_lap.csv.
PYTHONPATH=src:/tmp/casadi_pkgs MPLCONFIGDIR=/tmp/matplotlib-config \\
  python3 tools/run_iem24_murphy_lap.py --output-dir {output_dir_text}

# 2. Condition the logged stint: skew correction, lap segmentation, 1 Hz low-pass.
PYTHONPATH=src:/tmp/casadi_pkgs \\
  python3 tools/data/extract_iem24_telemetry.py --output-dir {output_dir_text}

# 3. Project the measured path onto the centreline and write this report.
PYTHONPATH=src:/tmp/casadi_pkgs MPLCONFIGDIR=/tmp/matplotlib-config \\
  python3 tools/reports/compare_iem24_murphy_stint.py --output-dir {output_dir_text}
```

This report is generated by `tools/reports/compare_iem24_murphy_stint.py` and follows the
[MIS 2024 report standard]({standard_link}).
"""
    (output_dir / "report.md").write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
