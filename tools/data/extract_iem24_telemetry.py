"""Extract the IEM24 Murphy stint channels from the exported MATLAB telemetry.

The navigation unit is mounted at a yaw skew, so the ``X``/``Y`` pairs are
rotated into the vehicle body frame before export.  The tool also segments the
stint into laps using position closure and writes a lap table for quality
control.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.data.iem24_telemetry import (
    LapBoundary,
    NavChannels,
    estimate_nav_skew_deg,
    integrate_position,
    lap_distance_axis,
    load_nav_channels,
    rotate_to_body_frame,
    segment_laps,
    zero_phase_lowpass,
)

#: Comparison signals are low-pass filtered because the simulated lap is
#: quasi-static and the raw 100 Hz channels carry suspension and kerb content.
COMPARISON_CUTOFF_HZ = 1.0

#: Export rate of the conditioned stint.  20 Hz resolves every MIS feature.
EXPORT_RATE_HZ = 20.0

STINT_WINDOW_S = (105.0, 938.0)

EXPORT_COLUMNS = (
    "time_s",
    "lap_index",
    "lap_distance_m",
    "lap_phase",
    "position_north_m",
    "position_east_m",
    "speed_mps",
    "longitudinal_velocity_mps",
    "lateral_velocity_mps",
    "longitudinal_acceleration_mps2",
    "lateral_acceleration_mps2",
    "yaw_rate_rad_s",
    "motor_torque_request_nm",
    "pack_voltage_v",
)

#: Compared and exported signals, in the order they appear after the four keys.
SIGNAL_COLUMNS = EXPORT_COLUMNS[6:]

#: Speed below which the car counts as stationary for the pack-voltage baseline.
STATIONARY_SPEED_MPS = 1.0

def build_body_channels(channels: NavChannels, angle_deg: float) -> NavChannels:
    """Return the skew-corrected channels with derived speed and torque."""

    body = rotate_to_body_frame(channels, angle_deg)
    values = dict(body.values)
    values["speed"] = np.hypot(body["VelX"], body["VelY"])
    values["motor_torque_request"] = 0.5 * (
        body["Tq_RL_Req"] + body["Tq_RR_Req"]
    )
    return NavChannels(time=body.time, values=values)


def validation_report(body: NavChannels, window: tuple[float, float]) -> dict:
    """Quantify the skew correction and channel consistency."""

    time = body.time
    interval = body.sample_interval
    mask = (
        (time >= window[0])
        & (time <= window[1])
        & (body["speed"] > 6.0)
    )
    smoothed_speed = zero_phase_lowpass(body["speed"], interval, COMPARISON_CUTOFF_HZ)
    longitudinal_velocity = zero_phase_lowpass(
        body["VelX"], interval, COMPARISON_CUTOFF_HZ
    )
    longitudinal_acceleration = zero_phase_lowpass(
        body["AccelX"], interval, COMPARISON_CUTOFF_HZ
    )
    lateral_acceleration = zero_phase_lowpass(
        body["AccelY"], interval, COMPARISON_CUTOFF_HZ
    )
    yaw_rate = zero_phase_lowpass(body["GyroZ"], interval, COMPARISON_CUTOFF_HZ)
    velocity_rate = np.gradient(longitudinal_velocity, time)
    usable = mask & np.isfinite(velocity_rate)

    def fit(reference: np.ndarray, measured: np.ndarray) -> dict[str, float]:
        slope, intercept = np.polyfit(reference[usable], measured[usable], 1)
        residual = measured[usable] - (slope * reference[usable] + intercept)
        return {
            "correlation": float(np.corrcoef(reference[usable], measured[usable])[0, 1]),
            "slope": float(slope),
            "rmse": float(np.sqrt(np.mean(residual**2))),
        }

    centripetal = longitudinal_velocity * yaw_rate
    return {
        "cutoff_hz": COMPARISON_CUTOFF_HZ,
        "sample_count": int(np.count_nonzero(usable)),
        "longitudinal_acceleration_vs_velocity_rate": fit(
            velocity_rate, longitudinal_acceleration
        ),
        "lateral_acceleration_vs_centripetal": fit(centripetal, lateral_acceleration),
        "peak_longitudinal_acceleration_mps2": float(
            np.max(np.abs(longitudinal_acceleration[mask]))
        ),
        "peak_lateral_acceleration_mps2": float(
            np.max(np.abs(lateral_acceleration[mask]))
        ),
        "peak_speed_mps": float(np.max(smoothed_speed[mask])),
    }


def _lap_rows(
    body: NavChannels,
    laps: list[LapBoundary],
    *,
    stride: int,
    north: np.ndarray,
    east: np.ndarray,
) -> list[tuple[float, ...]]:
    """Sample the conditioned lap channels onto the export grid."""

    time = body.time
    interval = body.sample_interval
    filtered = {
        "speed_mps": zero_phase_lowpass(body["speed"], interval, COMPARISON_CUTOFF_HZ),
        "longitudinal_velocity_mps": zero_phase_lowpass(
            body["VelX"], interval, COMPARISON_CUTOFF_HZ
        ),
        "lateral_velocity_mps": zero_phase_lowpass(
            body["VelY"], interval, COMPARISON_CUTOFF_HZ
        ),
        "longitudinal_acceleration_mps2": zero_phase_lowpass(
            body["AccelX"], interval, COMPARISON_CUTOFF_HZ
        ),
        "lateral_acceleration_mps2": zero_phase_lowpass(
            body["AccelY"], interval, COMPARISON_CUTOFF_HZ
        ),
        "yaw_rate_rad_s": zero_phase_lowpass(
            body["GyroZ"], interval, COMPARISON_CUTOFF_HZ
        ),
        "motor_torque_request_nm": zero_phase_lowpass(
            body["motor_torque_request"], interval, COMPARISON_CUTOFF_HZ
        ),
        "pack_voltage_v": zero_phase_lowpass(
            body["VPack"], interval, COMPARISON_CUTOFF_HZ
        ),
    }
    rows: list[tuple[float, ...]] = []
    for index, lap in enumerate(laps):
        axis = lap_distance_axis(time, body["speed"], lap)
        phase = axis / lap.distance_m
        samples = np.arange(lap.start_index, lap.end_index + 1, stride)
        for sample in samples:
            offset = sample - lap.start_index
            rows.append(
                (
                    time[sample],
                    float(index),
                    axis[offset],
                    phase[offset],
                    north[sample],
                    east[sample],
                    *(filtered[name][sample] for name in SIGNAL_COLUMNS),
                )
            )
    return rows


def pack_voltage_summary(
    body: NavChannels,
    laps: list[LapBoundary],
    window: tuple[float, float],
) -> dict:
    """Report the pack voltage at rest, at launch, and through the stint.

    The model needs one initial bus voltage for a single flying lap, so the
    resting value at the start of the stint is the natural choice.  The
    per-lap boundary voltages are reported as well because the pack sits lower
    once the stint is under way.
    """

    time = body.time
    inside = (time >= window[0]) & (time <= window[1])
    voltage = body["VPack"]
    speed = body["speed"]
    stationary = inside & (speed < STATIONARY_SPEED_MPS)
    # Only the run-in before the car first moves counts as the starting rest.
    moving = np.flatnonzero(inside & ~stationary)
    first_moving = int(moving[0]) if moving.size else int(np.flatnonzero(inside)[0])
    resting = stationary & (np.arange(time.size) < first_moving)
    if not np.any(resting):
        resting = inside
    return {
        "resting_voltage_v": float(np.median(voltage[resting])),
        "resting_sample_count": int(np.count_nonzero(resting)),
        "resting_standard_deviation_v": float(np.std(voltage[resting])),
        "launch_voltage_v": float(voltage[first_moving]),
        "stint_median_voltage_v": float(np.median(voltage[inside & ~stationary])),
        "stint_minimum_voltage_v": float(np.min(voltage[inside & ~stationary])),
        "lap_boundary_voltage_v": [
            float(voltage[lap.start_index]) for lap in laps
        ],
        "flying_lap_boundary_median_voltage_v": float(
            np.median(
                [
                    voltage[lap.start_index]
                    for lap in laps
                    if not lap.standing_start
                ]
            )
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mat_path",
        type=Path,
        nargs="?",
        default=Path.home() / "Desktop/0615CompEndurance.mat",
        help="exported MATLAB telemetry file",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs/validation/iem24-murphy"),
    )
    parser.add_argument("--start-s", type=float, default=STINT_WINDOW_S[0])
    parser.add_argument("--end-s", type=float, default=STINT_WINDOW_S[1])
    args = parser.parse_args()
    window = (args.start_s, args.end_s)

    channels = load_nav_channels(args.mat_path)
    skew = estimate_nav_skew_deg(channels, window=window)
    body = build_body_channels(channels, skew.angle_deg)
    laps = segment_laps(body, window=window)
    interval = channels.sample_interval
    stride = max(int(round(1.0 / (EXPORT_RATE_HZ * interval))), 1)
    north, east = integrate_position(
        channels.time, channels["VNorth"], channels["VEast"]
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "measured_stint.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(EXPORT_COLUMNS)
        writer.writerows(
            _lap_rows(body, laps, stride=stride, north=north, east=east)
        )

    lap_times = np.asarray([lap.lap_time_s for lap in laps])
    lap_distances = np.asarray([lap.distance_m for lap in laps])
    flying = [lap for lap in laps if not lap.standing_start]
    torque = body["motor_torque_request"]
    inside = (channels.time >= window[0]) & (channels.time <= window[1])
    moving = np.flatnonzero(inside & (body["speed"] > 1.0))
    summary = {
        "source": str(args.mat_path),
        "record": {
            "channel_count": len(channels.values),
            "sample_count": int(channels.time.size),
            "duration_s": float(channels.time[-1] - channels.time[0]),
            "sample_interval_s": interval,
        },
        "stint_window_s": list(window),
        "driving_window_s": (
            [
                float(channels.time[moving[0]]),
                float(channels.time[moving[-1]]),
            ]
            if moving.size
            else None
        ),
        "export_rate_hz": float(1.0 / (stride * interval)),
        "navigation_unit_skew": {
            "angle_deg": skew.angle_deg,
            "straight_sample_count": skew.sample_count,
            "angle_interquartile_deg": skew.angle_interquartile_deg,
            "residual_lateral_velocity_std_mps": skew.lateral_velocity_std_mps,
            "method": "median velocity heading over high-speed low-yaw samples",
        },
        "channels": {
            "longitudinal_acceleration": "AccelX rotated into the body frame",
            "lateral_acceleration": "AccelY rotated into the body frame",
            "yaw_rate": "GyroZ; unchanged by a yaw rotation",
            "motor_torque_request": "mean of Tq_RL_Req and Tq_RR_Req",
            "pack_voltage": "VPack, the pack terminal voltage",
        },
        "validation": validation_report(body, window),
        "pack_voltage": pack_voltage_summary(body, laps, window),
        "lap_torque_map": {
            "peak_request_nm": float(np.max(torque[inside])),
            "median_request_nm": float(np.median(torque[inside])),
        },
        "laps": [
            {
                "index": index,
                "start_s": float(channels.time[lap.start_index]),
                "end_s": float(channels.time[lap.end_index]),
                "lap_time_s": lap.lap_time_s,
                "distance_m": lap.distance_m,
                "mean_speed_mps": lap.distance_m / lap.lap_time_s,
                "start_speed_mps": lap.start_speed_mps,
                "standing_start": lap.standing_start,
                "closure_radius_m": lap.closure_radius_m,
            }
            for index, lap in enumerate(laps)
        ],
        "lap_statistics": {
            "count": len(laps),
            "flying_count": len(flying),
            "median_lap_time_s": float(np.median(lap_times)),
            "fastest_lap_time_s": float(np.min(lap_times)),
            "median_lap_distance_m": float(np.median(lap_distances)),
            "maximum_closure_radius_m": float(
                max(lap.closure_radius_m for lap in laps)
            ),
            "median_flying_lap_time_s": (
                float(np.median([lap.lap_time_s for lap in flying]))
                if flying
                else None
            ),
            "median_flying_lap_distance_m": (
                float(np.median([lap.distance_m for lap in flying]))
                if flying
                else None
            ),
        },
    }
    json_path = args.output_dir / "telemetry_extraction.json"
    json_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary["lap_statistics"], indent=2, sort_keys=True))
    print(f"skew {skew.angle_deg:.3f} deg; wrote {csv_path}")


if __name__ == "__main__":
    main()
