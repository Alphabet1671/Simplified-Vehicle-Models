#!/usr/bin/env python3
"""Overlay unchanged measured IEM24 laps with point-mass and planar 7-DOF runs."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.reports.compare_iem24_murphy_stint import (
    _median_lap_time,
    COMPARISON_CHANNELS,
    SIMULATED_COLUMNS,
    _metrics,
    _periodic_interp,
    fit_position_offset,
    flying_lap_indices,
    fold_laps,
    load_table,
    project_onto_track,
)
from simplified_vehicle_models.tracks.profile import load_track

ROOT = Path(__file__).resolve().parents[2]


def _zero_each_lap_voltage(measured: dict[str, np.ndarray], laps: np.ndarray) -> dict[str, np.ndarray]:
    """Copy measured data and offset each lap's voltage by its first sample."""
    adjusted = {key: value.copy() for key, value in measured.items()}
    voltage = adjusted["pack_voltage_v"]
    for lap in laps:
        select = adjusted["lap_index"] == lap
        sample_indices = np.flatnonzero(select)
        if sample_indices.size:
            voltage[sample_indices] -= voltage[sample_indices[0]]
    return adjusted


def _profile(path: Path, grid: np.ndarray, *, torque_nm: float) -> tuple[dict[str, np.ndarray], dict]:
    data = load_table(path)
    summary_path = path.with_name(path.stem + "_summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    distance = data["distance_m"]
    profile = {
        column: np.interp(grid, distance, data[source])
        for column, source in SIMULATED_COLUMNS.items()
    }
    profile["motor_torque_request_nm"] = np.interp(grid, distance, data["apps_fraction"]) * torque_nm
    profile["pack_voltage_v"] -= data["pack_voltage_v"][0]
    return profile, summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("docs/validation/iem24-murphy"))
    parser.add_argument("--track", type=Path, default=Path("models/tracks/2024_michigan_endurance_multilap_closed.xml"))
    parser.add_argument("--bins", type=int, default=1020, help="Distance-axis bins (about 1 m per bin on the MIS lap).")
    args = parser.parse_args()
    output = args.output_dir
    measured_original = load_table(output / "measured_stint.csv")
    laps = flying_lap_indices(measured_original)
    measured = _zero_each_lap_voltage(measured_original, laps)
    extraction_path = output / "telemetry_extraction.json"
    extraction = json.loads(extraction_path.read_text(encoding="utf-8"))
    median_lap_time = _median_lap_time(extraction, laps)
    track = load_track(ROOT / args.track)
    path = track.path_coordinates
    if path is None:
        raise ValueError("comparison requires the signed closed MIS centreline")
    offset, _ = fit_position_offset(measured, path)
    track_distance, residual = project_onto_track(measured, path, offset)
    columns = tuple(name for name, _, _ in COMPARISON_CHANNELS)
    grid, folded = fold_laps(
        measured, laps, columns, track_distance=track_distance,
        lap_length=path.lap_length, bins=args.bins,
    )
    # Include the exact start/finish node as well as bin centres. This makes
    # each independently zeroed pack-voltage trace visibly begin at 0 V.
    starts = {}
    for column in columns:
        lap_starts = []
        for lap in laps:
            select = measured["lap_index"] == lap
            phase = np.mod(track_distance[select], path.lap_length)
            lap_starts.append(float(_periodic_interp(
                phase, measured[column][select], np.array([0.0]), path.lap_length
            )[0]))
        starts[column] = np.asarray(lap_starts, dtype=np.float64)
        folded[column] = np.column_stack((starts[column], folded[column]))
    folded["pack_voltage_v"][:, 0] = 0.0
    grid = np.concatenate(([0.0], grid))
    measured_profile = {key: np.median(values, axis=0) for key, values in folded.items()}
    point_profile, point_summary = _profile(output / "simulated_lap.csv", grid, torque_nm=10.5)
    planar_profile, planar_summary = _profile(output / "simulated_lap_7dof.csv", grid, torque_nm=10.5)
    profiles = {"point_mass": point_profile, "planar_7dof": planar_profile}
    summaries = {"point_mass": point_summary, "planar_7dof": planar_summary}
    metrics = {
        model: {column: _metrics(profile[column], measured_profile[column]) for column in columns}
        for model, profile in profiles.items()
    }

    # Preserve the previous column-oriented CSV style while adding both model curves.
    csv_columns = [grid]
    headers = ["track_distance_m"]
    for column in columns:
        for key, profile in (("measured", measured_profile), ("point_mass", point_profile), ("planar_7dof", planar_profile)):
            headers.append(f"{key}_{column}")
            csv_columns.append(profile[column])
    np.savetxt(output / "comparison_three_way.csv", np.column_stack(csv_columns), delimiter=",", header=",".join(headers), comments="")

    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(COMPARISON_CHANNELS), 1, figsize=(12, 17), sharex=True)
    colors = {"point_mass": "#c00000", "planar_7dof": "#ed7d31"}
    labels = {"point_mass": "Real-model point mass", "planar_7dof": "Real-parameter 7DoF (UniTire)"}
    if not planar_summary.get("converged", False):
        labels["planar_7dof"] += " — unconverged iterate"
    for ax, (column, label, unit) in zip(axes, COMPARISON_CHANNELS):
        values = folded[column]
        ax.fill_between(grid, np.min(values, axis=0), np.max(values, axis=0), color="#9dc3e6", alpha=.38, label=f"Measured range ({len(laps)} laps)")
        ax.plot(grid, measured_profile[column], color="#1f4e79", lw=1.7, label="Measured median")
        for name, profile in profiles.items():
            linestyle = "--" if name == "planar_7dof" and not planar_summary.get("converged", False) else "-"
            ax.plot(grid, profile[column], color=colors[name], lw=1.5, linestyle=linestyle, label=labels[name])
        if column == "pack_voltage_v":
            label, unit = "Pack terminal voltage change from lap start", "V"
        ax.set_ylabel(f"{label} ({unit})")
        ax.grid(True, alpha=.25)
        ax.legend(loc="upper right", fontsize=7, ncol=2)
        ax.margins(x=0)
    axes[-1].set_xlabel("MIS centreline distance (m)")
    fig.suptitle("IEM24 Murphy — unchanged measured laps vs point-mass and 7DoF")
    fig.tight_layout()
    fig.savefig(output / "comparison_three_way.png", dpi=150)
    plt.close(fig)

    report = {
        "voltage_reference": "Each measured flying lap, point-mass lap, and 7DoF lap is offset by its own first sample; measured_stint.csv is read-only and unchanged.",
        "measured_lap_count": int(laps.size),
        "measured_median_lap_time_s": median_lap_time,
        "simulations": summaries,
        "metrics_vs_measured_median": metrics,
        "7dof_converged": bool(planar_summary.get("converged", False)),
        "7dof_metrics_valid": bool(planar_summary.get("converged", False)),
        "7dof_curve_kind": "converged solution" if planar_summary.get("converged", False) else "unconverged solver iterate; illustrative only",
        "position_residual_p95_m": float(np.percentile(residual[np.isin(measured["lap_index"], laps)], 95)),
        "track": str(args.track),
        "tire_note": "Planar 7DoF uses tester-fitted UniTire with provisional surface-mu scaling informed by measured peak deceleration/cornering; this is not a tire re-fit and differs from the point-mass tire law.",

    }
    (output / "comparison_three_way_summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    planar_lap_time = float(planar_summary.get("lap_time_s", float("nan")))
    planar_converged = bool(planar_summary.get("converged", False))
    if planar_converged:
        planar_result_line = (
            f"- Planar 7DoF model solution: {planar_lap_time:.3f} s; converged "
            f"({planar_summary.get('status', 'unknown')}). This prediction differs "
            f"from the measured median by {planar_lap_time - median_lap_time:+.3f} s; "
            "numerical convergence does not imply an experimental fit.\n\n"
        )
    else:
        planar_result_line = (
            f"- Planar 7DoF iterate: {planar_lap_time:.3f} s; not converged "
            f"({planar_summary.get('status', 'unknown')}). This is illustrative only, "
            "not a valid lap time.\n\n"
        )
    (output / "comparison_three_way_report.md").write_text(
        "# IEM24 Murphy three-way lap comparison\n\n"
        "Measured telemetry remains unchanged. The pack-voltage panel plots terminal-voltage change from each trace's own lap start, so all three start at zero and the mid-stint measured pack voltage offset is removed.\n\n"
        "The point-mass run uses the real IEM24 parameter set and measured rear-drive controller configuration. The 7DoF run uses the same mass, CG, LLTD, aero, rear-only topology, battery initial SOC, and 10.5 N·m torque cap. It requires a combined-slip tire and smooth motor envelope, so its tester-fitted UniTire uses provisional surface-mu scaling informed by measured peak deceleration/cornering (not a tire re-fit), while its smooth motor envelope approximates the measured map. Parallel steering, 3.5 in/rev Narco rack travel, and 65% front brake bias are user-confirmed; steering-arm/rate and hydraulic/pedal details remain provisional. The virtual brake request is extended only as needed to reach the current hydraulic pressure cap.\n\n"
        f"- Measured flying laps: {len(laps)}; median lap time {median_lap_time:.3f} s.\n"
        f"- Point-mass lap: {point_summary.get('lap_time_s', float('nan')):.3f} s (converged: {point_summary.get('converged', False)}).\n"
        f"- Planar 7DoF mesh: {planar_summary.get('mesh_spacing_m', float('nan')):.3f} m; {planar_summary.get('node_count', 'unknown')} nodes.\n"
        + planar_result_line
        + "![Three-way measured, point-mass and planar 7DoF overlay](comparison_three_way.png)\n\n"
        "The per-signal RMSE/MAE/bias/correlation metrics for each model are in `comparison_three_way_summary.json`. The shared distance-axis traces are in `comparison_three_way.csv`.\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
