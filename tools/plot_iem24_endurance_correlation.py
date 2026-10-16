#!/usr/bin/env python3
"""Draw five-channel IEM24 measured/point-mass/7DoF distance and time overlays.

Use a freshly integrated forward-driver 7DoF export via --seven-dof.
The forward-driver lap is not a minimum-time optimum or validated periodic lap.
Point-mass power is estimated; forward-driver power is reconstructed from its
motor-wheel torque, wheel speeds and powertrain efficiencies.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from tools.compare_iem24_murphy_stint import (
    _periodic_interp, fit_position_offset, flying_lap_indices, load_table,
    project_onto_track,
)
from tools.iem24_telemetry import load_nav_channels, zero_phase_lowpass
from simplified_vehicle_models.tracks.profile import load_track

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "docs/validation/iem24-murphy"
#: Canonical ten-lap MIS track (repository default).
DEFAULT_TRACK = Path("models/tracks/2024_michigan_endurance_multilap_closed.xml")
#: Exports that record no track were built on the original track.
LEGACY_TRACK = Path("models/tracks/2024_michigan_endurance_closed.xml")


def check_export_track(summary: dict, track: Path) -> None:
    """Refuse to fold a 7DoF export on a track other than the one it ran on."""
    recorded = Path(summary.get("track") or LEGACY_TRACK)
    if (ROOT / recorded).resolve() != (ROOT / track).resolve():
        raise ValueError(
            f"7DoF export ran on {recorded} but --track is {track}; pass "
            f"--track {recorded} to reproduce it")
CHANNELS = (
    ("velocity_mps", "Velocity (m/s)"),
    ("ax_mps2", "ax (m/s²)"),
    ("ay_mps2", "ay (m/s²)"),
    ("rl_power_kw", "RL motor electrical power (kW)"),
    ("rr_power_kw", "RR motor electrical power (kW)"),
)
RADIUS_M = 0.193675
GEAR_RATIO = 10.9
TORQUE_LIMIT_NM = 10.5
# Empirical approximate mechanical/electrical ratio observed in this MAT log.
POWER_EFFICIENCY = 0.79


def measured_power(mat_path: Path, measured: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Filter real power at 100 Hz and sample at the exported 20 Hz timestamps."""
    log = load_nav_channels(mat_path, ("pwrRLReal", "pwrRRReal"))
    indices = np.searchsorted(log.time, measured["time_s"])
    if np.any(indices == log.time.size) or not np.allclose(
        log.time[indices], measured["time_s"], atol=1e-4, rtol=0
    ):
        raise ValueError("measured_stint timestamps do not match MAT power timestamps")
    return {
        f"{side.lower()}_power_kw": zero_phase_lowpass(
            log[f"pwr{side}Real"], log.sample_interval, 1.0
        )[indices] / 1000.0
        for side in ("RL", "RR")
    }


def model_channels(data: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Estimate each motor's electrical input using equal rear torque requests."""
    power = (data["apps_fraction"] * TORQUE_LIMIT_NM * GEAR_RATIO
             * data["velocity_mps"] / RADIUS_M / POWER_EFFICIENCY / 1000.0)
    return {
        "velocity_mps": data["velocity_mps"],
        "ax_mps2": data["longitudinal_acceleration_mps2"],
        "ay_mps2": data["lateral_acceleration_mps2"],
        "rl_power_kw": data.get("rl_power_kw", power),
        "rr_power_kw": data.get("rr_power_kw", power),
    }


def measured_channels(data: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {
        "velocity_mps": data["speed_mps"],
        "ax_mps2": data["longitudinal_acceleration_mps2"],
        "ay_mps2": data["lateral_acceleration_mps2"],
        "rl_power_kw": data["rl_power_kw"],
        "rr_power_kw": data["rr_power_kw"],
    }


def fold_measured(
    measured: dict[str, np.ndarray], lap_indices: np.ndarray,
    projected_distance: np.ndarray, lap_length: float, axis: str,
    bins: int = 500,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Return a common grid and per-lap curves; time stays unwarped."""
    if axis not in ("distance", "time"):
        raise ValueError(f"unknown axis: {axis}")
    channels = measured_channels(measured)
    if axis == "distance":
        grid = np.linspace(0.0, lap_length, bins)
    elif axis == "time":
        durations = [np.ptp(measured["time_s"][measured["lap_index"] == lap])
                     for lap in lap_indices]
        grid = np.linspace(0.0, max(durations), bins)
    else:
        raise ValueError(f"unknown axis: {axis}")
    folded = {key: [] for key in channels}
    for lap in lap_indices:
        select = measured["lap_index"] == lap
        for key, values in channels.items():
            if axis == "distance":
                curve = _periodic_interp(
                    np.mod(projected_distance[select], lap_length), values[select],
                    grid, lap_length,
                )
            else:
                times = measured["time_s"][select]
                curve = np.interp(grid, times - times[0], values[select],
                                  left=np.nan, right=np.nan)
            folded[key].append(curve)
    return grid, {key: np.stack(rows) for key, rows in folded.items()}


def render(
    destination: Path, axis: str, grid: np.ndarray,
    folded: dict[str, np.ndarray], models: dict[str, dict[str, np.ndarray]],
) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(5, 1, figsize=(13, 14), sharex=True)
    palette = {"Point Mass": "#c00000", "7DoF": "#ed7d31"}
    for ax, (key, label) in zip(axes, CHANNELS):
        values = folded[key]
        with np.errstate(all="ignore"):
            low = np.nanmin(values, axis=0)
            high = np.nanmax(values, axis=0)
            median = np.nanmedian(values, axis=0)
        ax.fill_between(grid, low, high, color="#9dc3e6", alpha=.45)
        ax.plot(grid, median, color="#1f4e79", lw=1.7)
        for name, data in models.items():
            ax.plot(data["distance_m" if axis == "distance" else "elapsed_time_s"],
                    model_channels(data)[key], color=palette[name], lw=1.5,
                    linestyle="--" if name == "7DoF" else "-")
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.margins(x=0)
    axes[-1].set_xlabel("MIS centreline distance (m)" if axis == "distance"
                        else "Elapsed time from lap start (s)")
    axes[0].legend(handles=[
        Patch(facecolor="#9dc3e6", alpha=.45, label="Driving Data range"),
        Line2D([], [], color="#1f4e79", label="Driving Data"),
        Line2D([], [], color="#c00000", label="Point Mass"),
        Line2D([], [], color="#ed7d31", linestyle="--", label="7DoF"),
    ], loc="upper right", ncol=4, fontsize=9)
    fig.suptitle("IEM24 Murphy MIS endurance correlation — " + axis)
    fig.tight_layout()
    fig.savefig(destination, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mat", type=Path, default=Path.home() / "Desktop/0615CompEndurance.mat")
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK,
                        help="Closed-xy-profile track XML; must match the export's track")
    parser.add_argument("--seven-dof", type=Path, required=True,
                        help="Fresh forward-driver simulated_lap_7dof.csv export")
    args = parser.parse_args()
    output = args.output_dir
    measured = load_table(output / "measured_stint.csv")
    measured.update(measured_power(args.mat, measured))
    laps = flying_lap_indices(measured)
    summary_path = args.seven_dof.with_name(args.seven_dof.stem + "_summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    check_export_track(summary, args.track)
    track = load_track(ROOT / args.track)
    path = track.path_coordinates
    if path is None:
        raise ValueError("MIS track lacks a signed closed centreline")
    offset, _ = fit_position_offset(measured, path)
    projected, _ = project_onto_track(measured, path, offset)
    models = {
        "Point Mass": load_table(output / "simulated_lap.csv"),
        "7DoF": load_table(args.seven_dof),
    }
    if (summary.get("method") != "transient_forward_driver"
            or summary.get("lap_completed") is not True
            or summary.get("minimum_time_optimum") is not False):
        raise ValueError("expected a completed non-optimal forward-driver 7DoF export")
    for axis in ("distance", "time"):
        grid, folded = fold_measured(measured, laps, projected, path.lap_length, axis)
        render(output / f"comparison_three_way_{axis}.png", axis, grid, folded, models)
    print("Wrote distance and time figures; 7DoF is a non-optimal transient forward-driver lap.")


if __name__ == "__main__":
    main()
