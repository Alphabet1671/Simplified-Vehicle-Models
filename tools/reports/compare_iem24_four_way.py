#!/usr/bin/env python3
"""Four-way MIS 2024 comparison: measured stint vs point mass, 7DoF and 14DoF.

Produces the MIS 2024 report that compares one measured IEM24 Murphy endurance
stint against **three** simulations of the same canonical ten-lap-median MIS
track, all computed with the current checked-in vehicle, tire and formulation
files:

* the energy-coupled minimum-time **point mass** (``tools/run/run_iem24_point_mass_track_pair.py``,
  prescribed-path optimum);
* the planar **7DoF** forward-driver lap (``tools/run/run_iem24_forward_driver.py
  --model planar-7dof``, driven transient, not an optimum);
* the vertical **14DoF** forward-driver lap (``--model 14dof-simple``, driven
  transient, not an optimum).

The report follows ``docs/standards/mis-2024-report-standard.md``. The standard
assumes one simulation; this tool departs from it in exactly three disclosed
ways -- three simulation colours, one metric table with a model column, and a
second simulated-lap panel set -- and the generated report states the departure
in its first paragraph.

The tool refuses to report (``BLOCKER: ...``, exit status 2) when

* a forward attempt is not a completed transient lap, records no model-input
  hashes, or a recorded hash differs from the file on disk or from the attempt's
  own ``source_snapshot`` copy;
* the attempt, its channel export and the report disagree about the track, the
  lap time or the source hashes;
* the point-mass summary does not record the current track/track-geometry hashes
  and the current point-mass formulation, or is not a converged energy solve;
* a channels table is missing a required column, carries a non-finite value, or
  has a non-monotonic distance or clock;
* the model trace does not end at the canonical track length, or is not on the
  canonical ten-lap track the report claims.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT), str(ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from tools.reports.compare_iem24_murphy_stint import (  # noqa: E402
    COMPARISON_CHANNELS,
    _metrics,
    _periodic_interp,
    fit_position_offset,
    flying_lap_indices,
    fold_laps,
    load_table,
    project_onto_track,
)
from tools.reports.export_forward_attempt_channels import (  # noqa: E402
    ExportBlocker,
    drive_torque_limit_nm,
    verify_attempt,
)
from tools.reports.report_iem24_seven_dof_track_pair import (  # noqa: E402
    GATES,
    filter_model,
    sha256_file,
)
from simplified_vehicle_models.tracks.profile import load_track  # noqa: E402

BASE = ROOT / "docs/validation/iem24-murphy"
OUTPUT = BASE / "four_way_latest_model"
TRACK = "models/tracks/2024_michigan_endurance_multilap_closed.xml"
LEGACY_TRACK = "models/tracks/2024_michigan_endurance_closed.xml"
POINT_MASS_FORMULATION = "models/formulations/iem24_murphy_real_planar_7dof.xml"
PLANAR_FORMULATION = "models/formulations/iem24_murphy_real_planar_7dof.xml"
VERTICAL_FORMULATION = "models/formulations/iem24_murphy_real_14dof_simple.xml"
STANDARD = ROOT / "docs/standards/mis-2024-report-standard.md"
MAT = Path.home() / "Desktop/0615CompEndurance.mat"

#: Report order of the three simulated models.
MODELS: tuple[str, ...] = ("point_mass", "planar_7dof", "vertical_14dof")
LABELS = {"point_mass": "Point mass", "planar_7dof": "Planar 7DoF", "vertical_14dof": "Vertical 14DoF"}
#: Repo palette: measured stays ``#1f4e79``/``#9dc3e6``. The standard's single
#: simulation colour ``#c00000`` goes to the vertical 14DoF, the latest and
#: richest model; the two coarser simulations get distinct non-blue colours.
COLORS = {"point_mass": "#70ad47", "planar_7dof": "#ed7d31", "vertical_14dof": "#c00000"}
MEASURED_COLOR, BAND_COLOR = "#1f4e79", "#9dc3e6"
#: ``model_inputs`` role of the forward attempt behind each forward model.
ATTEMPT_MODEL = {"planar_7dof": "planar-7dof", "vertical_14dof": "14dof-simple"}
FORMULATION_OF = {"planar_7dof": PLANAR_FORMULATION, "vertical_14dof": VERTICAL_FORMULATION}

#: Report channel key -> column name in the exported / point-mass table. The
#: point mass writes the same quantity under its own established name.
SOURCE_COLUMNS = {
    "speed_mps": "velocity_mps",
    "longitudinal_acceleration_mps2": "longitudinal_acceleration_mps2",
    "lateral_acceleration_mps2": "lateral_acceleration_mps2",
    "yaw_rate_rad_s": "yaw_rate_rad_s",
    "motor_torque_request_nm": "motor_torque_request_nm",
    "pack_voltage_v": "pack_voltage_v",
}
POINT_MASS_SPEED_COLUMN = "speed_mps"
CHANNEL_KEYS = tuple(name for name, _, _ in COMPARISON_CHANNELS)


def source_columns(model: str) -> dict[str, str]:
    """Return the report channel -> table column map of one model."""
    mapping = dict(SOURCE_COLUMNS)
    if model == "point_mass":
        mapping["speed_mps"] = POINT_MASS_SPEED_COLUMN
    return mapping


def required_columns(model: str) -> tuple[str, ...]:
    """Return the channel columns one model's table must carry."""
    return ("distance_m", "elapsed_time_s") + tuple(source_columns(model).values())
CUTOFF_HZ = 1.0
MODEL_FILTER_HZ = 100.0
#: Point-mass SOC is not exported; integrating I = P/V must reproduce the
#: solver's recorded final SOC to better than this relative SOC-drop error.
SOC_RECONSTRUCTION_TOLERANCE = 0.02


class ReportBlocker(RuntimeError):
    """The inputs cannot support an honest four-way MIS 2024 report."""


# --------------------------------------------------------------------------- provenance
def resolve(path, root: Path = ROOT) -> Path:
    path = Path(path)
    return path if path.is_absolute() else root / path


def display(path, root: Path = ROOT) -> str:
    path = Path(path)
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def check_channels(table: dict, *, where: str, track_length: float,
                   required: tuple[str, ...]) -> None:
    """Refuse a channels table that cannot be compared on one distance axis."""
    missing = [name for name in required if name not in table]
    if missing:
        raise ReportBlocker(f"{where}: channels table lacks {', '.join(missing)}")
    for name in required:
        values = np.asarray(table[name], dtype=float)
        if values.ndim != 1 or values.size < 2:
            raise ReportBlocker(f"{where}: {name} is not a one-dimensional trace")
        if not np.all(np.isfinite(values)):
            raise ReportBlocker(f"{where}: {name} carries a non-finite value")
    time, distance = np.asarray(table["elapsed_time_s"]), np.asarray(table["distance_m"])
    if np.any(np.diff(time) <= 0.0):
        raise ReportBlocker(f"{where}: elapsed time is not strictly increasing")
    if np.any(np.diff(distance) <= 0.0):
        raise ReportBlocker(f"{where}: distance is not strictly increasing")
    if abs(float(distance[-1]) - float(track_length)) > 1.0e-3:
        raise ReportBlocker(f"{where}: trace ends at {distance[-1]:.4f} m, the canonical "
                            f"track is {track_length:.4f} m")


def verify_point_mass_run(run_dir, *, root: Path = ROOT, track: str = TRACK) -> dict:
    """Load a point-mass run summary and prove it used the current model and track."""
    run_dir = Path(run_dir)
    where = f"point mass run {run_dir}"
    try:
        summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ReportBlocker(f"{where}: cannot read summary.json ({error})") from error
    if str(summary.get("method", "")).split()[0:1] != ["energy-coupled"]:
        raise ReportBlocker(f"{where}: not an energy-coupled point-mass solve "
                            f"(method={summary.get('method')!r})")
    if summary.get("converged") is not True or summary.get("energy_converged") is not True:
        raise ReportBlocker(f"{where}: the lap or energy iteration did not converge "
                            f"(converged={summary.get('converged')!r}, "
                            f"energy_converged={summary.get('energy_converged')!r})")
    if summary.get("regen_enabled") is not False:
        raise ReportBlocker(f"{where}: regeneration must be recorded as disabled")
    if str(summary.get("track")) != str(track):
        raise ReportBlocker(f"{where}: ran on {summary.get('track')!r}, not {track!r}")
    track_xml = resolve(track, root)
    geometry = track_xml.with_suffix(".csv")
    if summary.get("track_sha256") != sha256_file(track_xml):
        raise ReportBlocker(f"{where}: recorded track hash differs from {track}")
    if summary.get("geometry_sha256") != sha256_file(geometry):
        raise ReportBlocker(f"{where}: recorded track-geometry hash differs from {geometry.name}")
    if str(summary.get("formulation")) != POINT_MASS_FORMULATION:
        raise ReportBlocker(f"{where}: recorded formulation {summary.get('formulation')!r} is not "
                            f"{POINT_MASS_FORMULATION!r}")
    table = load_table(run_dir / "simulated_lap.csv")
    lap_time = float(summary.get("lap_time_s"))
    if abs(float(table["elapsed_time_s"][-1]) - lap_time) > 1.0e-9:
        raise ReportBlocker(f"{where}: trace clock does not end at the recorded lap time")
    # The runner records the track and formulation by hash; the tire and vehicle
    # reach the solve through the compiled formulation, so resolve them from the
    # vehicle assembly and hash them here.
    from tools.run.run_iem24_forward_driver import component_xml
    assembly = root / "models/vehicles/iem24_murphy_real_planar.xml"
    tire_xml = component_xml(assembly, "tire")
    hashes = {"track_xml": summary["track_sha256"], "track_csv": summary["geometry_sha256"],
              "formulation_xml": sha256_file(root / POINT_MASS_FORMULATION),
              "tire_xml": sha256_file(root / tire_xml),
              "vehicle_xml": sha256_file(assembly)}
    paths = {"track_xml": track, "track_csv": str(Path(track).with_suffix(".csv")),
             "formulation_xml": POINT_MASS_FORMULATION, "tire_xml": str(tire_xml),
             "vehicle_xml": "models/vehicles/iem24_murphy_real_planar.xml"}
    return {"table": table, "summary": summary, "track": track, "hashes": hashes,
            "hash_paths": paths}


def verify_export(attempt, export_summary: dict, table: dict, *, model: str,
                  info: dict, root: Path = ROOT, track: str = TRACK) -> None:
    """Prove a channels export belongs to this attempt and this track."""
    where = f"{LABELS[model]} export"
    if resolve(export_summary.get("source_attempt", ""), root).resolve() != Path(attempt).resolve():
        raise ReportBlocker(f"{where}: export names a different source attempt")
    if str(export_summary.get("model")) != ATTEMPT_MODEL[model]:
        raise ReportBlocker(f"{where}: export model {export_summary.get('model')!r} is not "
                            f"{ATTEMPT_MODEL[model]!r}")
    if str(export_summary.get("track")) != str(track):
        raise ReportBlocker(f"{where}: export track {export_summary.get('track')!r} is not {track!r}")
    if str(export_summary.get("formulation")) != FORMULATION_OF[model]:
        raise ReportBlocker(f"{where}: export formulation is not the current {model} formulation")
    expected = {key: value["sha256"] for key, value in info["inputs"].items()}
    if export_summary.get("recorded_source_hashes") != expected:
        raise ReportBlocker(f"{where}: export source hashes differ from the attempt's model inputs")
    attempt_lap_time = float(info["summary"]["lap_time"])
    if abs(float(export_summary.get("lap_time_s", -1.0)) - attempt_lap_time) > 1.0e-9:
        raise ReportBlocker(f"{where}: export lap time differs from the attempt summary")
    if export_summary.get("lap_completed") is not True:
        raise ReportBlocker(f"{where}: export was not made from a completed lap")
    if abs(float(table["elapsed_time_s"][-1]) - attempt_lap_time) > 1.0e-9:
        raise ReportBlocker(f"{where}: exported clock does not end at the attempt lap time")
    missing = [name for name in ("soc", "apps_fraction", "rl_power_kw", "rr_power_kw")
               if name not in table]
    if missing:
        raise ReportBlocker(f"{where}: export lacks {', '.join(missing)}")


def load_forward_run(attempt, model: str, *, root: Path = ROOT, track: str = TRACK) -> dict:
    """Verify and load one forward attempt plus its channel export."""
    attempt = Path(attempt)
    try:
        info = verify_attempt(attempt, root=root)
    except ExportBlocker as blocker:
        raise ReportBlocker(str(blocker)) from blocker
    if str(info["model"]) != ATTEMPT_MODEL[model]:
        raise ReportBlocker(f"{LABELS[model]}: attempt model {info['model']!r} does not match")
    if str(info["track"]) != str(track):
        raise ReportBlocker(f"{LABELS[model]}: attempt ran on {info['track']!r}, not {track!r}")
    summary_path = attempt / f"{info['model']}_channels_summary.json"
    table_path = attempt / f"{info['model']}_channels.csv"
    if not summary_path.is_file() or not table_path.is_file():
        raise ReportBlocker(f"{LABELS[model]}: missing {table_path.name}; run "
                            f"python3 tools/reports/export_forward_attempt_channels.py {attempt}")
    export_summary = json.loads(summary_path.read_text(encoding="utf-8"))
    table = load_table(table_path)
    verify_export(attempt, export_summary, table, model=model, info=info, root=root, track=track)
    return {"table": table, "summary": info["summary"], "export_summary": export_summary,
            "attempt": attempt, "track": str(track),
            "hashes": {key: value["sha256"] for key, value in info["inputs"].items()},
            "hash_paths": {key: value["path"] for key, value in info["inputs"].items()}}


def point_mass_soc(table: dict, summary: dict, storage) -> np.ndarray:
    """Reconstruct the point-mass SOC trace and check it against the solve record."""
    time = np.asarray(table["elapsed_time_s"], dtype=float)
    power = (np.asarray(table["rl_power_kw"], dtype=float)
             + np.asarray(table["rr_power_kw"], dtype=float)) * 1000.0
    voltage = np.asarray(table["pack_voltage_v"], dtype=float)
    current = power / voltage
    integral = np.concatenate(([0.0], np.cumsum(0.5 * (current[1:] + current[:-1]) * np.diff(time))))
    soc = float(summary["initial_state_of_charge"]) - integral / storage.pack_capacity
    recorded = float(summary["final_state_of_charge"])
    drop = abs(float(summary["initial_state_of_charge"]) - recorded)
    error = abs(float(soc[-1]) - recorded) / max(drop, 1.0e-12)
    if error > SOC_RECONSTRUCTION_TOLERANCE:
        raise ReportBlocker("point mass: reconstructed state of charge does not reproduce the "
                            f"solver's final SOC (implied {soc[-1]:.6f} vs recorded {recorded:.6f})")
    return soc, error


def normalise_channels(table: dict, model: str) -> dict:
    """Map one model's table onto the report channel keys."""
    data = {"distance_m": np.asarray(table["distance_m"], dtype=float),
            "elapsed_time_s": np.asarray(table["elapsed_time_s"], dtype=float)}
    for key, source in source_columns(model).items():
        data[key] = np.asarray(table[source], dtype=float)
    for extra in ("apps_fraction", "rl_power_kw", "rr_power_kw", "soc"):
        if extra in table:
            data[extra] = np.asarray(table[extra], dtype=float)
    return data


# --------------------------------------------------------------------------- comparison
def fold_lap_clock(phase: np.ndarray, tau: np.ndarray, grid: np.ndarray,
                   lap_length: float) -> np.ndarray:
    """Fold one lap's elapsed-since-start clock onto the physical distance grid.

    A lap clock is **not** periodic, so the profile channels' periodic
    interpolation cannot be reused here: it would wrap the clock back to zero
    across the seam. The clock is instead made monotone, extended by one lap
    duration on each side (``tau(phase + L) = tau(phase) + duration``) and
    interpolated. The duration is the last sample's clock plus the time to cover
    the remaining distance at the lap's own mean speed, so a sample set that stops
    just short of the seam is still extrapolated correctly.
    """
    order = np.argsort(phase, kind="stable")
    ordered_phase, ordered_tau = phase[order], tau[order]
    ordered_tau = np.maximum.accumulate(ordered_tau)
    mean_speed = ordered_phase[-1] / ordered_tau[-1] if ordered_tau[-1] > 0 else 0.0
    remaining = max(lap_length - float(ordered_phase[-1]), 0.0)
    duration = float(ordered_tau[-1]) + (remaining / mean_speed if mean_speed > 0 else 0.0)
    extended_phase = np.concatenate((ordered_phase - lap_length, ordered_phase,
                                     ordered_phase + lap_length))
    extended_tau = np.concatenate((ordered_tau - duration, ordered_tau,
                                   ordered_tau + duration))
    return np.interp(grid, extended_phase, extended_tau)


def measured_cumulative_time(measured: dict, laps, track_distance: np.ndarray, grid: np.ndarray,
                             lap_length: float) -> np.ndarray:
    """Fold each flying lap's elapsed time from its own start onto the shared grid."""
    starts = {}
    for lap in np.unique(measured["lap_index"]):
        select = measured["lap_index"] == lap
        starts[int(lap)] = float(measured["time_s"][select][0])
    tau = np.asarray(measured["time_s"], dtype=float) - np.asarray(
        [starts[int(lap)] for lap in measured["lap_index"]], dtype=float)
    stack = np.empty((len(laps), grid.size), dtype=float)
    for row, lap in enumerate(laps):
        select = measured["lap_index"] == lap
        stack[row] = fold_lap_clock(np.mod(track_distance[select], lap_length), tau[select],
                                    grid, lap_length)
    return np.median(stack, axis=0)


def score_profile(profile: dict, median: dict, columns: tuple) -> dict:
    """Score one model profile against the measured median, channel by channel."""
    return {column: _metrics(profile[column], median[column]) for column in columns}


def compare(measured: dict, runs: dict, path, channels: tuple, *, laps) -> dict:
    """Score every model against the median flying lap on one physical distance axis."""
    columns = list(channels)
    offset, fit_rms = fit_position_offset(measured, path)
    projected, residual = project_onto_track(measured, path, offset)
    bins = int(round(path.lap_length))
    grid, folded = fold_laps(measured, laps, tuple(columns), track_distance=projected,
                             lap_length=path.lap_length, bins=bins)
    median = {column: np.median(folded[column], axis=0) for column in columns}
    measured_time = measured_cumulative_time(measured, laps, projected, grid, path.lap_length)
    models: dict[str, dict] = {}
    for model in MODELS:
        data = runs[model]["data"]
        filtered = filter_model(data, tuple(columns), fs=MODEL_FILTER_HZ, cutoff=CUTOFF_HZ)
        profile = {column: np.interp(grid, data["distance_m"], filtered[column])
                   for column in columns}
        metrics = score_profile(profile, median, tuple(columns))
        per_lap = {}
        for column in ("speed_mps", "lateral_acceleration_mps2"):
            per_lap[column] = [float(np.sqrt(np.mean((profile[column] - row) ** 2)))
                               for row in folded[column]]
        times = np.interp(grid, data["distance_m"], data["elapsed_time_s"])
        models[model] = {
            "profile": profile, "metrics": metrics, "per_lap": per_lap,
            "cumulative_time_s": times, "time_gap_s": times - measured_time,
            "lap_time_s": float(data["elapsed_time_s"][-1]),
            "mean_speed_mps": float(path.lap_length / data["elapsed_time_s"][-1]),
        }
    pairs = {}
    for first, second in (("point_mass", "planar_7dof"), ("point_mass", "vertical_14dof"),
                          ("planar_7dof", "vertical_14dof")):
        difference = models[first]["profile"]["speed_mps"] - models[second]["profile"]["speed_mps"]
        pairs[f"{first}_vs_{second}"] = {
            "speed_rmse_mps": float(np.sqrt(np.mean(difference ** 2))),
            "speed_mae_mps": float(np.mean(np.abs(difference))),
            "speed_bias_mps": float(np.mean(difference)),
            "coefficient_of_determination": float(
                1.0 - np.var(difference) / np.var(models[second]["profile"]["speed_mps"])),
        }
    flying = [entry for entry in runs["extraction"]["laps"] if not entry["standing_start"]]
    return {
        "grid": grid, "folded": folded, "median": median, "models": models, "pairs": pairs,
        "measured_cumulative_time_s": measured_time,
        "offset": offset, "fit_rmse_m": fit_rms, "bins": bins,
        "position_p95_m": float(np.percentile(residual[np.isin(measured["lap_index"], laps)], 95)),
        "median_lap_time_s": float(np.median([entry["lap_time_s"] for entry in flying])),
        "median_distance_m": float(np.median([entry["distance_m"] for entry in flying])),
        "laps": [int(lap) for lap in laps],
    }


# --------------------------------------------------------------------------- figures
def _panel(axis, result: dict, column: str, title: str, unit: str) -> None:
    grid = result["grid"]
    band = result["folded"][column]
    axis.fill_between(grid, np.min(band, axis=0), np.max(band, axis=0), color=BAND_COLOR,
                      alpha=0.35, label=f"Measured range ({band.shape[0]} flying laps)")
    axis.plot(grid, result["median"][column], color=MEASURED_COLOR, lw=1.5, label="Measured median")
    for model in MODELS:
        axis.plot(grid, result["models"][model]["profile"][column], color=COLORS[model], lw=1.3,
                  label=LABELS[model])
    axis.set_ylabel(f"{title} ({unit})")
    axis.grid(alpha=0.25)
    axis.margins(x=0)


def render(output: Path, result: dict, runs: dict) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    grid = result["grid"]
    count = len(COMPARISON_CHANNELS)
    fig, axes = plt.subplots(count, 1, figsize=(12.5, 3.1 * count), sharex=True, squeeze=False)
    for row, (column, title, unit) in enumerate(COMPARISON_CHANNELS):
        _panel(axes[row, 0], result, column, title, unit)
    axes[0, 0].legend(loc="upper right", fontsize=7, ncol=3)
    axes[-1, 0].set_xlabel("Canonical ten-lap-median MIS physical distance (m)")
    fig.suptitle("IEM24 Murphy MIS 2024: measured median flying lap vs point mass, planar 7DoF "
                 "and vertical 14DoF", y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(output / "comparison.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    for axis, column, name in ((axes[0, 0], "speed_mps", "Speed (m/s)"),
                               (axes[1, 0], "lateral_acceleration_mps2",
                                "Lateral acceleration (m/s²)")):
        for measured_row in result["folded"][column]:
            axis.plot(grid, measured_row, color=BAND_COLOR, lw=0.7)
        axis.plot([], [], color=BAND_COLOR, label="Each measured flying lap")
        axis.plot(grid, result["median"][column], color=MEASURED_COLOR, lw=1.7,
                  label="Measured median")
        for model in MODELS:
            axis.plot(grid, result["models"][model]["profile"][column], color=COLORS[model],
                      lw=1.4, label=LABELS[model])
        axis.set_ylabel(name)
        axis.set_xlabel("Canonical MIS physical distance (m)")
        axis.legend(fontsize=8)
        axis.grid(alpha=0.25)
        axis.margins(x=0)
    axes[0, 0].set_title("Every measured flying lap, the median and the three models")
    for model in MODELS:
        for row, column in ((0, "speed_mps"), (1, "lateral_acceleration_mps2")):
            values = result["models"][model]["per_lap"][column]
            low, high = float(np.min(values)), float(np.max(values))
            pad = max(0.08 * (high - low), 1.0e-3)
            axes[row, 1].hist(values, bins=np.linspace(low - pad, high + pad, 11),
                              color=COLORS[model], alpha=0.55,
                              label=f"{LABELS[model]} (median {np.median(values):.3f})")
    axes[0, 1].set_title("Per-lap speed RMSE against each measured flying lap")
    axes[0, 1].set_xlabel("Speed RMSE (m/s)")
    axes[1, 1].set_title("Per-lap lateral-acceleration RMSE against each measured flying lap")
    axes[1, 1].set_xlabel("Lateral-acceleration RMSE (m/s²)")
    for axis in axes[:, 1]:
        axis.set_ylabel("Measured-lap count")
        axis.legend(fontsize=8)
        axis.grid(alpha=0.25)
    fig.suptitle("IEM24 Murphy MIS 2024: lap-to-lap spread of the measured stint against the "
                 "three simulations", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(output / "lap_spread.png", dpi=140)
    plt.close(fig)

    rows = (("speed_mps", "Raw speed (m/s)"),
            ("longitudinal_acceleration_mps2", "Raw longitudinal acceleration (m/s²)"),
            ("lateral_acceleration_mps2", "Raw lateral acceleration (m/s²)"),
            ("yaw_rate_rad_s", "Raw yaw rate (rad/s)"),
            ("motor_torque_request_nm", "Requested motor torque (N·m)"),
            ("pack_voltage_v", "Terminal pack voltage (V)"),
            ("soc", "State of charge (–), initial condition"))
    fig, axes = plt.subplots(len(rows), 1, figsize=(12.5, 2.6 * len(rows)), sharex=True)
    for model in MODELS:
        data = runs[model]["data"]
        for axis, (key, title) in zip(axes, rows):
            axis.plot(data["distance_m"], data[key], color=COLORS[model], lw=1.1,
                      label=LABELS[model])
            axis.set_ylabel(title, fontsize=8)
            axis.grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Canonical ten-lap-median MIS physical distance (m)")
    fig.suptitle("Raw simulated laps on the canonical MIS track, all three starting from the "
                 "measured resting pack voltage", y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(output / "simulated_lap.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    for model in MODELS:
        axes[0].plot(grid, result["models"][model]["time_gap_s"], color=COLORS[model], lw=1.4,
                     label=LABELS[model])
    axes[0].axhline(0.0, color=MEASURED_COLOR, lw=1.0, ls="--")
    axes[0].set_xlabel("Canonical MIS physical distance (m)")
    axes[0].set_ylabel("Cumulative time gap vs measured median (s), sim − meas")
    axes[0].set_title("Cumulative time gap along the lap")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.25)
    axes[0].margins(x=0)
    for model in MODELS:
        axes[1].plot(result["models"][model]["cumulative_time_s"],
                     result["models"][model]["profile"]["speed_mps"], color=COLORS[model], lw=1.3,
                     label=LABELS[model])
    axes[1].plot(result["measured_cumulative_time_s"], result["median"]["speed_mps"],
                 color=MEASURED_COLOR, lw=1.6, label="Measured median")
    axes[1].set_xlabel("Elapsed lap time (s)")
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].set_title("Speed against elapsed lap time")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.25)
    fig.suptitle("IEM24 Murphy MIS 2024: where the lap time is spent", y=0.98)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    fig.savefig(output / "time_gap.png", dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------- report
def _fmt(value, spec=".3f") -> str:
    return "n/a" if value is None else format(value, spec)


def _gate_cell(value) -> str:
    return "PASS" if value is True else ("FAIL" if value is False else "not recorded")


def _closure_ratio(summary: dict) -> str:
    delta, tolerance = summary.get("periodic_state_delta"), summary.get("periodic_state_tolerances")
    if not delta or not tolerance or len(delta) != len(tolerance):
        return "not recorded"
    return f"{max(abs(d) / t for d, t in zip(delta, tolerance) if t > 0):.1f}×"


def source_line(extraction: dict, summary: dict) -> str:
    """Provenance sentence for the measured file, with or without the MAT present."""
    if summary.get("source_sha256"):
        return (f"Source `{extraction.get('source', '~/Desktop/0615CompEndurance.mat')}`, "
                f"SHA-256 `{summary['source_sha256']}`, {summary['source_bytes']} bytes")
    return (f"Source `{extraction.get('source')}`; the 334 MB MAT was not present at report "
            f"time, so its SHA-256 `{summary.get('source_recorded_sha256')}` and size "
            f"{summary.get('source_recorded_bytes')} bytes are taken from the record of "
            "earlier runs")


def write_report(output: Path, result: dict, runs: dict, measured: dict, *, root: Path = ROOT) -> None:
    from simplified_vehicle_models.compiler.compile import compile_formulation

    planar = compile_formulation(str(root / PLANAR_FORMULATION)).model
    vertical = compile_formulation(str(root / VERTICAL_FORMULATION)).model
    tire = planar.tire
    standard = os.path.relpath(STANDARD, output)
    extraction = runs["extraction"]
    summary = runs["summary"]
    laps = result["laps"]
    median_time = result["median_lap_time_s"]
    median_distance = result["median_distance_m"]
    grid_points = result["bins"]
    measured_mean_speed = median_distance / median_time
    columns = list(CHANNEL_KEYS)

    lap_rows = [
        f"| {entry['index']} {'(standing start)' if entry['standing_start'] else '(flying)'} "
        f"| {entry['start_s']:.2f} | {entry['lap_time_s']:.2f} | {entry['distance_m']:.1f} "
        f"| {entry['mean_speed_mps']:.2f} | {entry['closure_radius_m']:.2f} |"
        for entry in extraction["laps"]]

    metric_rows = []
    for column, title, unit in COMPARISON_CHANNELS:
        for model in MODELS:
            metrics = result["models"][model]["metrics"][column]
            metric_rows.append(
                f"| {title} ({unit}) | {LABELS[model]} | {metrics['rmse']:.3f} "
                f"| {metrics['mae']:.3f} | {metrics['bias']:+.3f} | {metrics['correlation']:.3f} "
                f"| {_fmt(metrics['normalised_rmse'])} |")

    lap_time_rows = []
    for model in MODELS:
        lap_time = result["models"][model]["lap_time_s"]
        lap_time_rows.append(
            f"| {LABELS[model]} | {lap_time:.3f} | {lap_time - median_time:+.3f} "
            f"| {result['models'][model]['mean_speed_mps']:.3f} |")
    pair_labels = {"point_mass_vs_planar_7dof": "Point mass − planar 7DoF",
                   "point_mass_vs_vertical_14dof": "Point mass − vertical 14DoF",
                   "planar_7dof_vs_vertical_14dof": "Planar 7DoF − vertical 14DoF"}
    pair_rows = "\n".join(
        f"| {pair_labels[key]} | {value['speed_rmse_mps']:.3f} | {value['speed_mae_mps']:.3f} "
        f"| {value['speed_bias_mps']:+.3f} | {value['coefficient_of_determination']:.4f} |"
        for key, value in result["pairs"].items())

    gates = {"point_mass": runs["point_mass"]["summary"],
             "planar_7dof": runs["planar_7dof"]["summary"],
             "vertical_14dof": runs["vertical_14dof"]["summary"]}
    gate_rows = []
    for key, name in GATES:
        gate_rows.append(f"| {name} | {_gate_cell(gates['planar_7dof'].get(key))} "
                         f"| {_gate_cell(gates['vertical_14dof'].get(key))} |")
    gate_rows.append(f"| **Recorded `usable` verdict** | **{_gate_cell(gates['planar_7dof'].get('usable'))}** "
                     f"| **{_gate_cell(gates['vertical_14dof'].get('usable'))}** |")

    hash_roles = ("track_xml", "track_csv", "tire_xml", "formulation_xml", "vehicle_xml")
    # The 14DoF attempt records extra component roles (transient tire, suspension,
    # both dampers and every data table they reference); show them too.
    extra_roles = [role for role in runs["vertical_14dof"]["hashes"] if role not in hash_roles]
    hash_rows = []
    for role in tuple(hash_roles) + tuple(sorted(extra_roles)):
        groups: dict = {}
        for model in MODELS:
            hashes = runs[model]["hashes"]
            if role not in hashes:
                continue
            path = runs[model].get("hash_paths", {}).get(role, role)
            groups.setdefault((str(path), hashes[role]), []).append(model)
        for (path, digest), models in groups.items():
            who = ", ".join(LABELS[model] for model in models)
            hash_rows.append(f"| {role} ({who}) | `{path}` | `{digest}` |")

    limit = drive_torque_limit_nm(planar)
    clicks = vertical.dampers[0].setting.clicks
    damper_clicks, damper_rebound = float(clicks["compression"]), float(clicks["rebound"])
    initial = {model: runs[model]["data"] for model in MODELS}
    aero = planar.aero
    motor = planar.motor
    storage = planar.storage
    speed_gap = {model: result["models"][model]["lap_time_s"] - median_time for model in MODELS}
    spread = {model: result["models"][model]["per_lap"]["speed_mps"] for model in MODELS}
    ay_spread = {model: result["models"][model]["per_lap"]["lateral_acceleration_mps2"]
                 for model in MODELS}
    peak = {}
    for model in MODELS:
        peak[model] = {
            "speed": float(np.max(result["models"][model]["profile"]["speed_mps"])),
            "ay": float(np.max(np.abs(result["models"][model]["profile"]["lateral_acceleration_mps2"]))),
            "braking": float(np.min(result["models"][model]["profile"]["longitudinal_acceleration_mps2"])),
            "voltage_mean": float(np.mean(result["models"][model]["profile"]["pack_voltage_v"])),
            "torque_mean": float(np.mean(result["models"][model]["profile"]["motor_torque_request_nm"])),
        }
    measured_peak = {
        "speed": float(np.max(result["median"]["speed_mps"])),
        "ay": float(np.max(np.abs(result["median"]["lateral_acceleration_mps2"]))),
        "braking": float(np.min(result["median"]["longitudinal_acceleration_mps2"])),
        "voltage_mean": float(np.mean(result["median"]["pack_voltage_v"])),
        "torque_mean": float(np.mean(result["median"]["motor_torque_request_nm"])),
    }
    tail_gap = {model: float(result["models"][model]["time_gap_s"][-1]) for model in MODELS}
    voltage_bias = {model: result["models"][model]["metrics"]["pack_voltage_v"]["bias"]
                    for model in MODELS}
    voltage_zeroed_bias = {}
    for model in MODELS:
        model_voltage = result["models"][model]["profile"]["pack_voltage_v"]
        voltage_zeroed_bias[model] = float(np.mean((model_voltage - model_voltage[0])
                                                   - (result["median"]["pack_voltage_v"]
                                                      - result["median"]["pack_voltage_v"][0])))
    voltage_demeaned_rmse = {}
    for model in MODELS:
        difference = (result["models"][model]["profile"]["pack_voltage_v"]
                      - result["median"]["pack_voltage_v"])
        voltage_demeaned_rmse[model] = float(np.sqrt(np.mean((difference - np.mean(difference)) ** 2)))
    source_text = source_line(extraction, summary)
    window = extraction["stint_window_s"]
    drive = extraction["driving_window_s"]
    sample_rate = 1.0 / extraction["record"]["sample_interval_s"]
    skew = extraction.get("navigation_unit_skew", {})
    validation = extraction.get("validation", {})
    volts = extraction.get("pack_voltage", {})
    settings = {"planar_7dof": runs["planar_7dof"]["summary"]["settings"],
                "vertical_14dof": runs["vertical_14dof"]["summary"]["settings"]}
    point = runs["point_mass"]["summary"]
    gates_14 = gates["vertical_14dof"]
    gates_7 = gates["planar_7dof"]
    gear = motor.gear_ratio
    both_usable = gates_7.get('usable') is True and gates_14.get('usable') is True
    verdict_cell = lambda g: '**PASS**' if g.get('usable') is True else '**FAIL**'
    gate_status_clause = ("both forward attempts pass the numerical acceptance gates (table in Configuration); "
                          "that is numerical acceptance, not physical validation" if both_usable else
                          "at least one forward attempt fails the acceptance gates (table in Configuration)")
    drive_limit_text = (f"{motor.unit_count} rear motors, {motor.peak_motor_torque:g} N·m per-motor "
                        f"torque cap, {motor.peak_power / 1000:g} kW per motor, {gear:g}:1 gear "
                        f"ratio, regeneration off")
    text = f"""# IEM24 Murphy MIS 2024 four-way comparison: measured endurance stint vs point-mass, planar 7DoF and vertical 14DoF simulated laps

Michigan International Speedway 2024 endurance circuit, the IEM24 Murphy car, **three simulated laps compared with one measured endurance stint**. This report follows the [MIS 2024 report standard]({standard}) **with one departure, stated here as the standard requires**: the standard describes a single simulation (one simulation colour, one simulation per metric row), and this report deliberately compares **three** — the energy-coupled minimum-time point mass, the planar 7DoF forward-driver lap and the vertical 14DoF forward-driver lap — because the question asked is how the newest 14DoF model ranks against the two models already in the repository on one common track. The departure is limited to (i) three distinguishable simulation colours instead of one, (ii) one metric table carrying a `Model` column, and (iii) a fourth comparison figure (`time_gap.png`) plus a model-vs-model table. Sections, channels, columns, units, sign conventions, disclosures and figures otherwise follow the standard unchanged. All three simulations are fresh runs on the canonical ten-lap-median track `{TRACK}` with the current checked-in vehicle, tire and formulation files (commit `1e1346a` corrected the track width to 1.240 m and the front mass fraction to 45 %); no earlier report's numbers are reused. **No claim of validation is made:** the measured lap is a human driven lap and the two forward laps are non-optimal driven transients; {gate_status_clause}.

## Configuration

All three simulations share one physical model of the car, one measured-driver plan and one track. Values below are compiled from the current files; the formulations are `{PLANAR_FORMULATION}` (point mass and planar 7DoF) and `{VERTICAL_FORMULATION}` (vertical 14DoF).

- Mass: **{planar.total_mass:g} kg** ({planar.mass_without_driver:g} kg car + {planar.driver_mass:g} kg driver), identical in all three runs. Yaw radius of gyration {planar.yaw_radius_of_gyration:g} m (measured yaw inertia), wheel inertia {planar.wheel_inertia:.4f} kg·m² per wheel. The vertical 14DoF splits this into a {vertical.sprung_mass:g} kg sprung body and {vertical.total_unsprung_mass:g} kg unsprung ({vertical.unsprung_mass_front:g} kg per front corner, {vertical.unsprung_mass_rear:g} kg per rear corner); the point mass and the planar 7DoF carry no vertical split.
- Chassis: wheelbase {planar.wheelbase:g} m, track width {planar.track_width:g} m, LLTD {planar.lateral_load_transfer_distribution * 100:g} %, **CGx** = {planar.front_mass_fraction * 100:g} % of the mass on the front axle ({planar.front_axle_to_cg:.4f} m behind the **front** axle), CG height {planar.cg_height:g} m, {planar.brake_bias * 100:g} % front mechanical brake bias. The vertical model's sprung CG sits {vertical.sprung_cg_height:.6f} m up with a sprung front fraction of {vertical.sprung_front_fraction:.6f}; unsprung CG height {vertical.unsprung_cg_height:g} m.
- Vertical inertias (14DoF only): pitch inertia {vertical.pitch_inertia:.6f} kg·m² (radius of gyration {vertical.pitch_radius_of_gyration:g} m, the user-reported D5 value), roll inertia {vertical.roll_inertia:.6f} kg·m² (radius {vertical.roll_radius_of_gyration:g} m). Both radii are **unvalidated**; see Caveats.
- Aero: air density {aero.air_density:g} kg/m³, CL·A = {aero.downforce_area:g} m², CD·A = {aero.drag_area:g} m², aero balance {aero.aero_balance * 100:g} %.
- Drivetrain: {drive_limit_text}; full pedal requests {limit:g} N·m per motor. Tire: UniTire (`models/tires/iem24_murphy_real_unitire.xml`) with a peak/sliding friction envelope μ₀ₓ = {tire.mu_0_x:.6f}, μₛₓ = {tire.mu_s_x:.6f}, μ₀ᵧ = {tire.mu_0_y:.6f}, μₛᵧ = {tire.mu_s_y:.6f}; the vertical model additionally uses the PROPOSED transient tire layer (length scales {vertical.transient_tire.longitudinal_length_scale:g} / {vertical.transient_tire.lateral_length_scale:g} m, trail pole/lead {vertical.transient_tire.trail_pole_ratio:g} / {vertical.transient_tire.trail_lead_ratio:g}) whose coefficients are **not fitted** to this tire.
- Battery: {storage.series_count}s{storage.parallel_count}p, {storage.discharge_current_limit:g} A discharge limit, {storage.cell_capacity:g} Ah per cell, SOC-dependent OCV and resistance from the cell curves.
- Initial conditions (all three runs): initial SOC {storage.initial_state_of_charge:.6f} from the measured resting pack voltage {volts.get('resting_voltage_v', float('nan')):.2f} V before launch (initial bus voltage {storage.initial_bus_voltage:g} V); first accepted forward speed {initial['planar_7dof']['speed_mps'][0]:.3f} m/s (planar 7DoF), {initial['vertical_14dof']['speed_mps'][0]:.3f} m/s (14DoF), and the point mass solves its own periodic flying-lap start speed {initial['point_mass']['speed_mps'][0]:.3f} m/s. No measured tire temperature or ambient state initialises any run.
- Simulated laps on `{TRACK}` (length {runs['track_length_m']:.3f} m): point mass **{result['models']['point_mass']['lap_time_s']:.3f} s**, planar 7DoF **{result['models']['planar_7dof']['lap_time_s']:.3f} s**, vertical 14DoF **{result['models']['vertical_14dof']['lap_time_s']:.3f} s**. Measured median flying lap **{median_time:.3f} s** ({median_distance:.2f} m driven).

Per-model differences (all on the same canonical track):

| Property | Point mass | Planar 7DoF | Vertical 14DoF |
|---|---|---|---|
| Degrees of freedom | 3 (path speed, rear-drive force, energy) | 7 planar (v_x, v_y, yaw, four wheel speeds) | 14 (planar 7 + heave, roll, pitch, four jounces) |
| Tire model | peak/sliding pure-slip envelope from the UniTire lab fit | steady UniTire combined slip with four wheel-speed states | the same steady UniTire plus the PROPOSED transient (relaxation) layer |
| Suspension / vertical load | none; a quasi-static load-transfer-free grip limit | none; per-wheel load from quasi-static lateral/longitudinal transfer | four-corner springs, bump stops, ARB, dampers ({damper_clicks:g}/{damper_rebound:g} clicks), jounce travel {gates_14.get('jounce_travel_limits_m', [0, 0])[0]:g}–{gates_14.get('jounce_travel_limits_m', [0, 0])[1]:g} m, DSS roll-centre table |
| Driver | prescribed-path minimum-time optimum (no driver) | preview driver on the live point-mass speed plan | same preview driver and plan as the 7DoF |
| Solver | energy-coupled forward/backward profile iterations | BDF, rtol {settings['planar_7dof']['rtol']:g}, maximum step {settings['planar_7dof']['maximum_step']:g} s | BDF, rtol {settings['vertical_14dof']['rtol']:g}, maximum step {settings['vertical_14dof']['maximum_step']:g} s |
| Path spacing | {point['spacing_m']:g} m mesh, {point['nodes']} nodes | {settings['planar_7dof']['spacing']:g} m plan | {settings['vertical_14dof']['spacing']:g} m plan |
| Lap-run status | converged in {point['iterations']} profile and {point['energy_iterations']} energy iterations | completed full lap, accepted steps {gates_7.get('accepted_steps')}, wall {gates_7.get('elapsed_wall_seconds', float('nan')):.1f} s, peak RSS {gates_7.get('peak_memory_bytes', 0) / 1e9:.2f} GB | completed full lap, accepted steps {gates_14.get('accepted_steps')}, wall {gates_14.get('elapsed_wall_seconds', float('nan')):.1f} s, peak RSS {gates_14.get('peak_memory_bytes', 0) / 1e9:.2f} GB |
| `usable` verdict | not applicable (not a driven lap); optimum | {verdict_cell(gates_7)} | {verdict_cell(gates_14)} |

Acceptance gates recorded by the two forward attempts (the point mass records convergence, not these gates):

| Gate | Planar 7DoF | Vertical 14DoF |
|---|---|---|
{chr(10).join(gate_rows)}

Model inputs and their recorded SHA-256 (re-verified against the files on disk and against each attempt's `source_snapshot` before this report was written; the attempted runs and the export all agree):

| Input | Path | SHA-256 |
|---|---|---|
{chr(10).join(hash_rows)}

The point-mass run records the track and formulation by hash rather than by `model_inputs`; its tire and vehicle reach the solve through `{POINT_MASS_FORMULATION}`, whose compiled parameters are the same objects the planar 7DoF report compiles.

## Measured data

{source_text}; shared log clock {sample_rate:.0f} Hz ({extraction['record']['sample_count']} samples), exported measured table at {extraction['export_rate_hz']:.2f} Hz. Conditioning uses {window[0]:.2f}–{window[1]:.2f} s for lap identification and a {drive[0]:.2f}–{drive[1]:.2f} s driving window. Channels consumed: navigation `VNorth`, `VEast`, `VelX`, `VelY`, `AccelX`, `AccelY`, `GyroZ`; per-motor requests `Tq_RL_Req`, `Tq_RR_Req`; pack voltage `VPack`. The existing `measured_stint.csv` and `telemetry_extraction.json` are read unchanged and were not regenerated by this report.

Corrections applied to the raw channels: the {skew.get('angle_deg', float('nan')):.2f}° navigation-unit yaw skew rotates VelX/VelY and AccelX/AccelY into the body frame (corrected longitudinal-acceleration-versus-velocity-rate RMSE {validation.get('longitudinal_acceleration_vs_velocity_rate', {}).get('rmse', float('nan')):.3f} m/s², correlation {validation.get('longitudinal_acceleration_vs_velocity_rate', {}).get('correlation', float('nan')):.3f}); the motor torque request is the mean of the two logged per-motor requests; navigation accelerations, speed, torque and voltage are low-pass filtered with a zero-phase first-order filter at **{CUTOFF_HZ:g} Hz** (cutoff {validation.get('cutoff_hz', CUTOFF_HZ):g} Hz, order 1, `sosfiltfilt` on a 100 Hz uniform clock for the simulated traces). No voltage zeroing, no inferred time shift and no equal-length scaling is applied to any trace in the metric table; the only offset convention used elsewhere in the repository — the three-way tool's per-lap pack-voltage zeroing, which subtracts each lap's first sample — is **not** applied here, and its effect is quantified in `Reading the comparison` so the residual offset can be separated from model error.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---|---:|---:|---:|---:|
{chr(10).join(lap_rows)}

Median of the 10 flying laps: **{median_time:.3f} s** over **{median_distance:.2f} m** ({measured_mean_speed:.2f} m/s mean). The measured driven line is {(median_distance / runs['track_length_m'] - 1) * 100:+.2f} % longer than the {runs['track_length_m']:.2f} m canonical modelled centreline. The standing-start lap is in the table and excluded from every statistic.

Alignment: each measured position is projected onto the canonical centreline after fitting a constant north/east translation (fit RMSE {result['fit_rmse_m']:.2f} m, residual p95 {result['position_p95_m']:.2f} m; offset north {result['offset'][0]:+.2f} m, east {result['offset'][1]:+.2f} m). Every flying lap is then interpolated onto the canonical track's own physical distance (integrated path coordinate `s` for the simulations, projected distance for the measurement) and the median and full range are recomputed on a {grid_points}-point grid. All four traces therefore share one physical distance axis; **no phase offset is inferred and no trace is stretched**, because all four laps close on the same 1009.195 m line.

Channel definitions, identical for the planar 7DoF and the vertical 14DoF: speed is the body-frame velocity magnitude `hypot(v_x, v_y)`; longitudinal and lateral acceleration are road-plane body-frame accelerations including the rotating-frame terms (`v_x_dot − yaw_rate·v_y` and `v_y_dot + yaw_rate·v_x`) — the road is flat, so body-frame and road-plane definitions coincide and no roll/pitch gravimetric projection is modelled; yaw rate is the traced `yaw_rate` state; motor torque request is the commanded pedal fraction times the {limit:g} N·m per-motor full-pedal limit (requested, not delivered); pack terminal voltage is the plant's own terminal-voltage closure. The point-mass model has no yaw state, wheel speeds or motor command in the same sense, so its channels come from its existing exporter exactly as in `point_mass_track_pair/report.md`: yaw rate is the kinematic `a_y / v` of the prescribed path, motor torque request is reconstructed from the driven rear-wheel force through the gear ratio and transmission efficiency and clipped at 10.5 N·m (zero while braking), pack terminal voltage is reconstructed from the solver's bus power and SOC with the library battery model, and its SOC trace is reconstructed by integrating `I = P/V` — that reconstruction reproduces the solver's recorded final SOC to {runs['point_mass']['soc_error'] * 100:.2f} % of the SOC drop. Point-mass `rl_power_kw`/`rr_power_kw` are equal halves of the total bus power, so its power channels are not independent per-motor measurements.

## Trace comparison

Every model is scored against the **median of the same 10 flying laps** on the shared {grid_points}-point canonical distance grid. Measured range shading is the minimum and maximum individual flying-lap value. Bias is signed **sim − meas** in the row's unit. The two forward laps are single transients from their own initial states (periodic closure gate: planar 7DoF {_gate_cell(gates_7.get('periodicity_valid'))}, vertical 14DoF {_gate_cell(gates_14.get('periodicity_valid'))}; closure is within the gate tolerance, not exact), so their start and end samples are not exactly steady-state; the point mass is a converged periodic optimum on the same geometry.

| Signal | Model | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |
|---|---|---:|---:|---:|---:|---:|
{chr(10).join(metric_rows)}

Lap-time summary (sign convention sim − meas against the measured median flying lap):

| Source | Lap time (s) | Gap vs measured median (s) | Mean speed (m/s) |
|---|---:|---:|---:|
| Measured median of 10 flying laps | {median_time:.3f} | 0.000 | {measured_mean_speed:.3f} |
| Measured fastest / slowest flying lap | {min(entry['lap_time_s'] for entry in extraction['laps'] if not entry['standing_start']):.3f} / {max(entry['lap_time_s'] for entry in extraction['laps'] if not entry['standing_start']):.3f} | — | — |
{chr(10).join(lap_time_rows)}

Model-to-model agreement on the same grid (point mass minus the listed model, so a positive bias means the point mass is faster at that distance):

| Pair | Speed RMSE (m/s) | Speed MAE (m/s) | Speed bias (m/s) | 1 − var(difference)/var(second) |
|---|---:|---:|---:|---:|
{pair_rows}

![Measured ten-flying-lap range and median against the point-mass, planar 7DoF and vertical 14DoF laps, all on the canonical ten-lap-median MIS physical distance](comparison.png)

![Every measured flying-lap speed with its median and the three models, the measured lateral acceleration with its median and the three models, and the per-lap speed and lateral-acceleration RMSE distributions of each model](lap_spread.png)

![Raw simulated laps on the canonical MIS track: speed, longitudinal and lateral acceleration, yaw rate, requested motor torque, terminal pack voltage and the initial-condition state of charge](simulated_lap.png)

![Cumulative time gap of each simulation against the measured median along the lap, and speed against elapsed lap time](time_gap.png)

## Reading the comparison

- **Lap time: all three models are faster than the measured median lap, by {abs(speed_gap['point_mass']):.2f} s (point mass, {result['models']['point_mass']['lap_time_s']:.3f} s), {abs(speed_gap['planar_7dof']):.2f} s (planar 7DoF, {result['models']['planar_7dof']['lap_time_s']:.3f} s) and {abs(speed_gap['vertical_14dof']):.2f} s (vertical 14DoF, {result['models']['vertical_14dof']['lap_time_s']:.3f} s) against {median_time:.3f} s.** The mean speeds are {result['models']['point_mass']['mean_speed_mps']:.2f}, {result['models']['planar_7dof']['mean_speed_mps']:.2f} and {result['models']['vertical_14dof']['mean_speed_mps']:.2f} m/s against {measured_mean_speed:.2f} m/s measured. **The headline result is that the vertical degrees of freedom change almost nothing.** The 14DoF lap is {result['models']['vertical_14dof']['lap_time_s'] - result['models']['planar_7dof']['lap_time_s']:+.3f} s against the planar 7DoF ({(result['models']['vertical_14dof']['lap_time_s'] / result['models']['planar_7dof']['lap_time_s'] - 1) * 100:+.2f} %), and their two speed traces differ by {result['pairs']['planar_7dof_vs_vertical_14dof']['speed_rmse_mps']:.3f} m/s RMSE on the shared grid, while each is roughly {abs(speed_gap['planar_7dof']):.1f} s from the measured median: adding suspension load transfer, four-corner vertical dynamics and transient tire relaxation moves the answer by less than a tenth of a second, so **the {abs(speed_gap['planar_7dof']):.1f} s gap to the measured lap is not explained by the vertical fidelity the 14DoF adds**. The point mass is {abs(speed_gap['point_mass'] - speed_gap['planar_7dof']):.2f} s faster than the planar 7DoF because it is a prescribed-path **minimum-time optimum** with no driver, no transient wheel dynamics and no steering lag — it is a bound, not a lap a driver could repeat. The fact that all three beat the measured median means the shared grip and powertrain priors (μ₀ᵧ = {tire.mu_0_y:.3f}, i.e. the 2/3-of-lab road prior) are optimistic for this track on this day; the deficit is a **model/prior** effect, not an alignment or initial-condition effect. All three models completed a full lap on the canonical track, so no legacy-track fallback table is needed.
- **Speed:** RMSE {result['models']['point_mass']['metrics']['speed_mps']['rmse']:.3f} (point mass), {result['models']['planar_7dof']['metrics']['speed_mps']['rmse']:.3f} (planar 7DoF) and {result['models']['vertical_14dof']['metrics']['speed_mps']['rmse']:.3f} m/s (vertical 14DoF); biases {result['models']['point_mass']['metrics']['speed_mps']['bias']:+.3f}, {result['models']['planar_7dof']['metrics']['speed_mps']['bias']:+.3f} and {result['models']['vertical_14dof']['metrics']['speed_mps']['bias']:+.3f} m/s; correlations {result['models']['point_mass']['metrics']['speed_mps']['correlation']:.3f}, {result['models']['planar_7dof']['metrics']['speed_mps']['correlation']:.3f} and {result['models']['vertical_14dof']['metrics']['speed_mps']['correlation']:.3f}. All three track the measured shape well, and all three sit **high** on average: the models carry more speed through the corner exits than the measured median. Peak speed is {peak['point_mass']['speed']:.1f} / {peak['planar_7dof']['speed']:.1f} / {peak['vertical_14dof']['speed']:.1f} m/s simulated against {measured_peak['speed']:.1f} m/s in the measured median. The point mass is the unfavourable extreme here: its mean bias is {result['models']['point_mass']['metrics']['speed_mps']['bias']:+.2f} m/s, more than twice the forward models' {result['models']['planar_7dof']['metrics']['speed_mps']['bias']:+.2f} / {result['models']['vertical_14dof']['metrics']['speed_mps']['bias']:+.2f} m/s, because a minimum-time optimum is a different kind of object from a driven lap.
- **Longitudinal acceleration:** RMSE {result['models']['point_mass']['metrics']['longitudinal_acceleration_mps2']['rmse']:.3f} / {result['models']['planar_7dof']['metrics']['longitudinal_acceleration_mps2']['rmse']:.3f} / {result['models']['vertical_14dof']['metrics']['longitudinal_acceleration_mps2']['rmse']:.3f} m/s² (point mass / 7DoF / 14DoF) with correlations {result['models']['point_mass']['metrics']['longitudinal_acceleration_mps2']['correlation']:.3f} / {result['models']['planar_7dof']['metrics']['longitudinal_acceleration_mps2']['correlation']:.3f} / {result['models']['vertical_14dof']['metrics']['longitudinal_acceleration_mps2']['correlation']:.3f}. After {CUTOFF_HZ:g} Hz filtering the models brake to {peak['point_mass']['braking']:.1f} / {peak['planar_7dof']['braking']:.1f} / {peak['vertical_14dof']['braking']:.1f} m/s² against {measured_peak['braking']:.1f} m/s² in the measured median: every model brakes harder and later than the human driver, which is where a large part of the lap-time gap is won. This row is the one where the models are individually worst (normalised RMSE {result['models']['planar_7dof']['metrics']['longitudinal_acceleration_mps2']['normalised_rmse']:.2f}), and the cause is the driver model (instantaneous torque switching against the human pedal ramp), not the vehicle model: the 7DoF and 14DoF rows are within {abs(result['models']['vertical_14dof']['metrics']['longitudinal_acceleration_mps2']['rmse'] - result['models']['planar_7dof']['metrics']['longitudinal_acceleration_mps2']['rmse']):.3f} m/s² of each other.
- **Lateral acceleration:** RMSE {result['models']['point_mass']['metrics']['lateral_acceleration_mps2']['rmse']:.3f} / {result['models']['planar_7dof']['metrics']['lateral_acceleration_mps2']['rmse']:.3f} / {result['models']['vertical_14dof']['metrics']['lateral_acceleration_mps2']['rmse']:.3f} m/s² with biases {result['models']['point_mass']['metrics']['lateral_acceleration_mps2']['bias']:+.3f} / {result['models']['planar_7dof']['metrics']['lateral_acceleration_mps2']['bias']:+.3f} / {result['models']['vertical_14dof']['metrics']['lateral_acceleration_mps2']['bias']:+.3f} m/s². Peak |a_y| is {peak['point_mass']['ay']:.1f} m/s² for the point mass ({(peak['point_mass']['ay'] / 9.80665):.2f} g, above the measured median's {measured_peak['ay']:.1f} m/s² and near the stint's individual peak of {validation.get('peak_lateral_acceleration_mps2', float('nan')):.1f} m/s²) but only {peak['planar_7dof']['ay']:.1f} / {peak['vertical_14dof']['ay']:.1f} m/s² for the two driven models, which match the measured median's cornering peaks closely. The positive bias in every model is at least partly the known measured-accelerometer offset (the point-mass pair report found measured a_y about −0.7 m/s² away from `v·yaw`), not pure model error: RMSE and correlation are the safer agreement measures here.
- **Yaw rate:** RMSE {result['models']['point_mass']['metrics']['yaw_rate_rad_s']['rmse']:.4f} / {result['models']['planar_7dof']['metrics']['yaw_rate_rad_s']['rmse']:.4f} / {result['models']['vertical_14dof']['metrics']['yaw_rate_rad_s']['rmse']:.4f} rad/s and biases {result['models']['point_mass']['metrics']['yaw_rate_rad_s']['bias']:+.4f} / {result['models']['planar_7dof']['metrics']['yaw_rate_rad_s']['bias']:+.4f} / {result['models']['vertical_14dof']['metrics']['yaw_rate_rad_s']['bias']:+.4f} rad/s. The two driven histories trace the measured yaw closely ({result['models']['planar_7dof']['metrics']['yaw_rate_rad_s']['correlation']:.3f} and {result['models']['vertical_14dof']['metrics']['yaw_rate_rad_s']['correlation']:.3f} correlation); the point-mass yaw rate is a kinematic `a_y / v` derived from a lateral acceleration that is itself quantised at the mesh, so its higher error ({result['models']['point_mass']['metrics']['yaw_rate_rad_s']['rmse']:.4f} rad/s) is an export artefact of the prescribed-path optimum, not a handling result.
- **Motor torque request:** RMSE {result['models']['point_mass']['metrics']['motor_torque_request_nm']['rmse']:.3f} / {result['models']['planar_7dof']['metrics']['motor_torque_request_nm']['rmse']:.3f} / {result['models']['vertical_14dof']['metrics']['motor_torque_request_nm']['rmse']:.3f} N·m with biases {result['models']['point_mass']['metrics']['motor_torque_request_nm']['bias']:+.3f} / {result['models']['planar_7dof']['metrics']['motor_torque_request_nm']['bias']:+.3f} / {result['models']['vertical_14dof']['metrics']['motor_torque_request_nm']['bias']:+.3f} N·m and mean requests {peak['point_mass']['torque_mean']:.2f} / {peak['planar_7dof']['torque_mean']:.2f} / {peak['vertical_14dof']['torque_mean']:.2f} N·m against {measured_peak['torque_mean']:.2f} N·m measured. **Definition difference, disclosed:** the point-mass row is a torque reconstructed from the driven rear-wheel force (so it is zero under braking and reaches the {limit:g} N·m clip), while the two forward rows are the commanded pedal fraction times {limit:g} N·m. The three rows are therefore not the same measurement and must not be ranked against each other; the measured channel is the logged mean of `Tq_RL_Req` and `Tq_RR_Req`, whose peak is 10.45 N·m.
- **Pack terminal voltage (reconstructed):** RMSE {result['models']['point_mass']['metrics']['pack_voltage_v']['rmse']:.3f} / {result['models']['planar_7dof']['metrics']['pack_voltage_v']['rmse']:.3f} / {result['models']['vertical_14dof']['metrics']['pack_voltage_v']['rmse']:.3f} V and biases {voltage_bias['point_mass']:+.2f} / {voltage_bias['planar_7dof']:+.2f} / {voltage_bias['vertical_14dof']:+.2f} V. **This row is dominated by an initial-condition difference, not by model structure.** The simulations all start at the stint-start SOC {storage.initial_state_of_charge:.6f} (the measured resting pack voltage {volts.get('resting_voltage_v', float('nan')):.2f} V), while the ten measured flying laps start from a median boundary voltage of {volts.get('flying_lap_boundary_median_voltage_v', float('nan')):.2f} V, i.e. part-way through the discharge. Applying the three-way tool's per-lap voltage-zeroing convention (subtract each trace's first sample) reduces the bias to {voltage_zeroed_bias['point_mass']:+.2f} / {voltage_zeroed_bias['planar_7dof']:+.2f} / {voltage_zeroed_bias['vertical_14dof']:+.2f} V, and removing the mean offset alone leaves a shape-only RMSE of {voltage_demeaned_rmse['point_mass']:.3f} / {voltage_demeaned_rmse['planar_7dof']:.3f} / {voltage_demeaned_rmse['vertical_14dof']:.3f} V. The metric table above keeps the **absolute** voltage (the standard's convention here, and the one the 7DoF paired-track report uses), so its voltage RMSE and bias must be read as an offset plus a shape error; the shape error is the only part that speaks about the battery model.
- **Spread:** the per-lap speed RMSE against the individual flying laps spans {min(spread['point_mass']):.3f}–{max(spread['point_mass']):.3f} m/s (point mass), {min(spread['planar_7dof']):.3f}–{max(spread['planar_7dof']):.3f} m/s (planar 7DoF) and {min(spread['vertical_14dof']):.3f}–{max(spread['vertical_14dof']):.3f} m/s (vertical 14DoF); the lateral-acceleration RMSE spans {min(ay_spread['point_mass']):.3f}–{max(ay_spread['point_mass']):.3f}, {min(ay_spread['planar_7dof']):.3f}–{max(ay_spread['planar_7dof']):.3f} and {min(ay_spread['vertical_14dof']):.3f}–{max(ay_spread['vertical_14dof']):.3f} m/s². The lap-to-lap spread of the human driver (69.99–77.31 s, a 7.32 s range) is larger than the model-to-model difference ({abs(result['models']['planar_7dof']['lap_time_s'] - result['models']['vertical_14dof']['lap_time_s']):.3f} s between the 7DoF and 14DoF), so a single measured lap cannot separate the two forward models.
- **{'Acceptance gates are green for both forward laps, which is numerical acceptance only; no result here is validated against measurement.' if both_usable else 'At least one acceptance gate is red for a forward lap, and no result here is validated.'}** Planar 7DoF: `path_tracking_valid` = {_gate_cell(gates_7.get('path_tracking_valid'))} (maximum lateral path error {gates_7.get('maximum_lateral_error', float('nan')):.3f} m against the 0.25 m gate), `periodicity_valid` = {_gate_cell(gates_7.get('periodicity_valid'))} (closure ratio {_closure_ratio(gates_7)}), `between_step_validation_complete` = {_gate_cell(gates_7.get('between_step_validation_complete'))}, `refinement_validated` = {_gate_cell(gates_7.get('refinement_validated'))}, so `usable` = **{_gate_cell(gates_7.get('usable'))}**. Vertical 14DoF: `path_tracking_valid` = {_gate_cell(gates_14.get('path_tracking_valid'))} (maximum lateral path error {gates_14.get('maximum_lateral_error', float('nan')):.3f} m), `periodicity_valid` = {_gate_cell(gates_14.get('periodicity_valid'))}, `between_step_validation_complete` = {_gate_cell(gates_14.get('between_step_validation_complete'))}, `refinement_validated` = {_gate_cell(gates_14.get('refinement_validated'))}, so `usable` = **{_gate_cell(gates_14.get('usable'))}**. The point mass is `converged` = {_gate_cell(point.get('converged'))} in {point['iterations']} profile and {point['energy_iterations']} energy iterations, which makes it an optimum of its own reduced problem, not an accepted lap of the full vehicle.
- **Where the lap time is spent:** the cumulative time gap against the measured median (fourth figure) ends at {tail_gap['point_mass']:+.2f} / {tail_gap['planar_7dof']:+.2f} / {tail_gap['vertical_14dof']:+.2f} s. The gap opens in the braking zones and the slow corners (where the models decelerate harder and carry more exit speed) rather than in the straights, which is consistent with the speed, longitudinal and lateral rows above.

## Caveats

- **These are the values we did not measure** for this car in the log: wheelbase, track width, yaw inertia, wheel inertia, tire force/slip characteristics and the whole transient-tire layer, battery resistance and capacity map, drivetrain and motor efficiency, steering geometry and rate, brake parameters, spring/damper/ARB/bump-stop curves, vertical load transfer, suspension travel limits, roll-centre table, ambient air density and mechanical brake bias. They are carried over from the IEM24 vehicle compilation (and, for the vertical layer, from the workbook-derived Task 6 tables), not inferred from this experiment. The {limit:g} N·m motor torque cap and the four friction values are model calibrations.
- **The vertical inertias are unvalidated and one modal gate is out of band.** The pitch radius of gyration {vertical.pitch_radius_of_gyration:g} m (D5) is user-reported and unvalidated; with it the vertical model's undamped pitch mode is **7.30 Hz, outside the plan's 3.4–4.7 Hz band**, and the roll radius {vertical.roll_radius_of_gyration:g} m gives a 9.90 Hz roll mode that is not gated at all. The ledger ruling was to report this as a plan-gate deviation rather than tune springs, ARB, tires or inertias; the heave mode (3.83 Hz) and the body-fixed hop set do meet their gates.
- **The transient tire coefficients are PROPOSED, not fitted to this tire.** Length scales {vertical.transient_tire.longitudinal_length_scale:g} / {vertical.transient_tire.lateral_length_scale:g} m and trail pole/lead {vertical.transient_tire.trail_pole_ratio:g} / {vertical.transient_tire.trail_lead_ratio:g} were chosen from a published reduced model whose equations were not independently re-verified (the paper text was unavailable), and a fitting tool does not yet exist in the repository. Any 14DoF-versus-7DoF difference therefore mixes suspension load transfer with an unfitted tire relaxation model.
- **{'Both forward laps pass the numerical acceptance gates' if both_usable else 'At least one forward lap fails the acceptance gates'}** (see Configuration): `usable` = {_gate_cell(gates_7.get('usable'))} for the planar 7DoF and {_gate_cell(gates_14.get('usable'))} for the vertical 14DoF. `usable` certifies that each lap closes periodically, tracks the path within 0.25 m, is reproduced by tighter-step/tolerance reruns and satisfies the between-step checks; it does not compare the model with measured behaviour. The planar between-step check includes an interval load-closure certificate; the vertical 14DoF between-step check proves only the evaluation-domain guards (table, jounce, damper-velocity, tire-inverse, plant guards) on the BDF dense enclosure, with the powertrain closure covered by its audit test and the tire-force outputs not enclosed (see `solvers/vertical_between_step.py`, `SCOPE`). These remain driven transients, not optimal laps; the traces are complete laps, so the comparison is meaningful, but no claim about full-vehicle fidelity follows from it.
- **The point mass is a prescribed-path minimum-time optimum, not a driven lap.** It has no steering, sideslip, wheel-speed or vertical dynamics and no driver model, so its lap is a bound on the shared-grip model's performance; the two forward laps are feasible points of a driver model on a live speed plan and are **not** optima (`minimum_time_optimum` = false in both summaries).
- **Aero drag couple.** The planar models omit the vertical-model drag couple `D·(h_s − h_cg)/L`; the plan's planar-reduction gate therefore had to compare with aero zeroed, and the residual gap (pinned by an oracle at `D·h_s/L`) is a disclosed structural difference between the planar 7DoF and the vertical 14DoF rows.
- **Gravity convention differs by tier:** the vertical modules use g = 9.80665 m/s² while the planar conventions use 9.81 m/s². The offset is 3.4·10⁻⁴ relative on front-axle load, inside the 1e-2 planar-reduction gate but not zero.
- **Different integration steps and driver/solver settings:** the point mass is a mesh solution (0.5 m, {point['nodes']} nodes, profile iterations); the planar 7DoF used BDF with rtol 1e-5 and maximum step 0.02 s ({gates_7.get('accepted_steps')} accepted steps); the vertical 14DoF used BDF with rtol 1e-5 and maximum step 0.005 s ({gates_14.get('accepted_steps')} accepted steps) because its transient tire/vertical modes are stiff. Differences below the 0.005 s step and the 0.02 s step are not resolved equally, so part of the 7DoF-versus-14DoF gap is numerical rather than physical.
- **Road grip is a proposed prior, not a measurement.** The four friction values are exactly 2/3 of the lab fit (a team-reported road prior), the lab longitudinal data come from a different tire size (43100 R20 against the modelled 43075), and the lateral 2/3 value is optimistic against the ≈0.60 lab ratio estimated from measured |a_y|. Every lap time and the ax/ay rows depend on that prior; the shared optimism is the most likely single cause of all three models beating the measured median. See [road grip calibration](../road_grip_calibration.md).
- **Residual offsets separated from model error:** the pack-voltage bias ({voltage_bias['point_mass']:+.2f} / {voltage_bias['planar_7dof']:+.2f} / {voltage_bias['vertical_14dof']:+.2f} V) is an initial-SOC offset (quantified above by re-zeroing); the lateral-acceleration bias ({result['models']['planar_7dof']['metrics']['lateral_acceleration_mps2']['bias']:+.2f} m/s² for the planar 7DoF) is at least partly the measured accelerometer offset against `v·yaw`; the longitudinal bias is dominated by the driver's pedal ramp against instantaneous model torque switching. The point-mass yaw row carries a kinematic-export artefact.
- **Why the superseded reports are not reused.** `point_mass_track_pair/report.md` (marked "Superseded 2026-10-16") records a **71.958 s** point-mass lap on the *same* ten-lap-median geometry — its recorded `geometry_sha256` `2db5a981f12d0c055afa3b167a9e6efa07cd09b1f6e7b2c51afea44f00e73d77` is byte-identical to the canonical track CSV used here — with the same formulation path. This report's fresh point-mass lap is **{result['models']['point_mass']['lap_time_s']:.3f} s**, {result['models']['point_mass']['lap_time_s'] - 71.95843431536265:+.3f} s from that figure. The pre-fix vehicle was not re-run, so attributing the difference to the corrected track width (1.016 → 1.240 m) and front mass fraction (55 % → 45 %) is by inspection of the recorded inputs, not a controlled experiment; either way, reusing the old number would have been wrong by far more than the 7DoF-versus-14DoF difference this report measures. The same reasoning applies to `seven_dof_track_pair/report.md`, whose corrected reference attempt is on the *legacy* 1016.701 m track and whose recorded runner hash does not match the current runner.
- **The measured reference is circular for the track.** The canonical line is the median of the same ten flying laps used as the measured reference, so agreement with it cannot establish the geometry as ground truth; the measured driven line is {(median_distance / runs['track_length_m'] - 1) * 100:+.2f} % longer than that centreline, and the residual p95 of {result['position_p95_m']:.2f} m is the normal offset of the driven line, not model error.
- The point-mass `rl_power_kw`/`rr_power_kw` rows are equal halves of the bus power (it has no independent wheel torques) and the logged `pwrRLReal`/`pwrRRReal` sensor definition is undocumented, so power rows are trace context only and are not scored here. No tire temperature, tire pressure or ambient state initialises any run.

## Reproduce

```bash
# Run from the repository root (the worktree that holds branch feature/14dof-simple).
OUT=docs/validation/iem24-murphy/four_way_latest_model

# 1. Point mass: energy-coupled minimum-time flying lap on the canonical track,
#    with the current compiled planar vehicle (library call, writes into $OUT/runs/point_mass).
PYTHONPATH=src:/tmp/casadi_pkgs python3 -c "
from pathlib import Path
from tools.run.run_iem24_point_mass_track_pair import export_run
from tools.reports.compare_planar_point_mass import FORMULATION, _point_mass_vehicle
from simplified_vehicle_models.compiler.compile import compile_formulation
planar = compile_formulation(str(FORMULATION)).model
vehicle, _ = _point_mass_vehicle(planar)
export_run(vehicle, Path('{TRACK}'), Path('{display(runs['point_mass']['run_dir'])}'), 0.5)"

# 2./3. Forward-driver acceptance chains. Each model runs cold, warm (from the
#    cold attempt), refined, finer and (14DoF) fourth, with the steering gains
#    chosen by tools/run/sweep_forward_steering.py (preview 0.20 s / heading 2.5 /
#    lateral 1.5 for the 7DoF; 0.25 s / 3.5 / 1.5 for the 14DoF). Base step is
#    0.02 s (7DoF) or 0.005 s (14DoF); each later stage halves step and rtol
#    (see the script for exact values).
CH=docs/validation/iem24-murphy/forward-usability-fixes
for s in cold warm refined finer; do
  tools/run/run_forward_acceptance_chain.sh planar-7dof $CH/seven_dof 0.2 2.5 1.5 $s; done
for s in cold warm refined finer fourth; do
  CHAIN_TIMEOUT=7200 tools/run/run_forward_acceptance_chain.sh 14dof-simple $CH/vertical_14dof 0.25 3.5 1.5 $s; done
# Post-run gates on the finest attempts (planar tube tool needs the pre-reorganisation
# module alias `tools.run_iem24_forward_driver`; see the forward-usability README):
#   tools/gates/check_iem24_forward_refinement.py  REF REFINED         (7DoF, consecutive stages)
#   tools/gates/check_iem24_forward_tube.py        REFINED FINER FOURTH (7DoF; the run used WARM REFINED FINER)
#   tools/gates/check_iem24_forward_refinement_vertical.py REF REFINED  (14DoF, consecutive stages)
#   tools/gates/check_iem24_forward_tube_vertical.py REFINED FINER FOURTH (14DoF)
#   tools/gates/certify_iem24_forward_between_step.py <finest attempt> --workers 8

# 4. Export the six comparison channels of each forward attempt (writes the
#    per-state channels CSV, gitignored, plus its small tracked summary JSON).
SEVEN={display(runs['planar_7dof']['attempt'])}
VERTICAL={display(runs['vertical_14dof']['attempt'])}
PYTHONPATH=src:. python3 tools/reports/export_forward_attempt_channels.py "$SEVEN" --output "$SEVEN"
PYTHONPATH=src:. python3 tools/reports/export_forward_attempt_channels.py "$VERTICAL" --output "$VERTICAL"

# 5. Build the report, the figures and the supporting tables.
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/reports/compare_iem24_four_way.py \\
  --point-mass-dir $OUT/runs/point_mass --seven-dof-attempt "$SEVEN" \\
  --vertical-14dof-attempt "$VERTICAL" --output-dir $OUT
```

This report was generated from the point-mass run `{display(runs['point_mass']['run_dir'])}`, the planar 7DoF attempt `{display(runs['planar_7dof']['attempt'])}` and the vertical 14DoF attempt `{display(runs['vertical_14dof']['attempt'])}`. A fresh reproduction creates new attempt directories and therefore new attempt timestamps; the simulated channel values and this report then reproduce byte for byte only if the same attempts are passed back in, which is why their directories are recorded here. Report generator: `tools/reports/compare_iem24_four_way.py`; channel exporter: `tools/reports/export_forward_attempt_channels.py`; measured conditioning `tools/data/extract_iem24_telemetry.py` (not re-run: `measured_stint.csv` and `telemetry_extraction.json` are read unchanged); follows the [MIS 2024 report standard]({standard}). The source MAT must be at `~/Desktop/0615CompEndurance.mat` for the provenance hash and the power channels; every number in this report is reproduced from the recorded hashes when it is absent.
"""
    (output / "report.md").write_text(text, encoding="utf-8")


# --------------------------------------------------------------------------- driver
def load_all(args, *, root: Path = ROOT) -> dict:
    """Load, verify and normalise the measured reference and the three models."""
    measured = load_table(args.measured_dir / "measured_stint.csv")
    extraction = json.loads((args.measured_dir / "telemetry_extraction.json").read_text(encoding="utf-8"))
    track = load_track(root / args.track)
    if track.path_coordinates is None:
        raise ReportBlocker(f"{args.track}: the comparison needs signed closed geometry")
    runs: dict = {"extraction": extraction, "track_length_m": float(track.lap_length)}
    point = verify_point_mass_run(args.point_mass_dir, root=root, track=str(args.track))
    runs["point_mass"] = point
    for model, option in (("planar_7dof", args.seven_dof_attempt),
                          ("vertical_14dof", args.vertical_14dof_attempt)):
        runs[model] = load_forward_run(option, model, root=root, track=str(args.track))
    for model in MODELS:
        check_channels(runs[model]["table"], where=LABELS[model],
                       track_length=runs["track_length_m"], required=required_columns(model))
        runs[model]["data"] = normalise_channels(runs[model]["table"], model)
        runs[model]["run_dir"] = (runs[model]["attempt"] if "attempt" in runs[model]
                                  else Path(args.point_mass_dir))
    from simplified_vehicle_models.compiler.compile import compile_formulation
    storage = compile_formulation(str(root / PLANAR_FORMULATION)).model.storage
    soc, error = point_mass_soc(runs["point_mass"]["table"], runs["point_mass"]["summary"], storage)
    runs["point_mass"]["data"]["soc"] = soc
    runs["point_mass"]["soc_error"] = error
    mat = Path(args.mat)
    runs["summary"] = {"source_sha256": sha256_file(mat) if mat.is_file() else None,
                       "source_bytes": mat.stat().st_size if mat.is_file() else None,
                       "mat_present": mat.is_file(),
                       "source_recorded_sha256": "091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7",
                       "source_recorded_bytes": 334513192,
                       "models": {model: {"lap_time_s": float(runs[model]["data"]["elapsed_time_s"][-1]),
                                          "attempt": display(runs[model]["run_dir"], root),
                                          "hashes": runs[model]["hashes"],
                                          "track": runs[model]["track"]}
                                 for model in MODELS}}
    runs["measured"] = measured
    return runs


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    parser.add_argument("--point-mass-dir", type=Path, default=OUTPUT / "runs/point_mass")
    parser.add_argument("--seven-dof-attempt", type=Path, required=True)
    parser.add_argument("--vertical-14dof-attempt", type=Path, required=True)
    parser.add_argument("--measured-dir", type=Path, default=BASE,
                        help="directory holding measured_stint.csv and telemetry_extraction.json")
    parser.add_argument("--track", default=TRACK)
    parser.add_argument("--mat", type=Path, default=MAT)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    try:
        runs = load_all(args, root=args.root)
        measured = runs["measured"]
        laps = flying_lap_indices(measured)
        path = load_track(args.root / args.track).path_coordinates
        result = compare(measured, runs, path, CHANNEL_KEYS, laps=laps)
        output = args.output_dir
        output.mkdir(parents=True, exist_ok=True)
        names = ["track_distance_m"]
        values = [result["grid"]]
        for column in CHANNEL_KEYS:
            names.append(f"measured_{column}")
            values.append(result["median"][column])
            for model in MODELS:
                names.append(f"{model}_{column}")
                values.append(result["models"][model]["profile"][column])
        np.savetxt(output / "four_way_channels.csv", np.column_stack(values), delimiter=",",
                   header=",".join(names), comments="")
        summary = {
            "result_status": ("three simulations of one canonical MIS lap against the median of 10 "
                              "measured flying laps; no result is validated"),
            "track": str(args.track),
            "track_length_m": float(runs["track_length_m"]),
            "measured_laps": result["laps"],
            "measured_median_lap_time_s": result["median_lap_time_s"],
            "measured_median_distance_m": result["median_distance_m"],
            "position_fit_rmse_m": result["fit_rmse_m"],
            "position_residual_p95_m": result["position_p95_m"],
            "position_offset_north_east_m": result["offset"].tolist(),
            "grid_points": result["bins"],
            "filter_cutoff_hz": CUTOFF_HZ,
            "simulation_filter_clock_hz": MODEL_FILTER_HZ,
            "source_file": {"path": str(args.mat), "present": runs["summary"]["mat_present"],
                            "sha256": runs["summary"]["source_sha256"],
                            "bytes": runs["summary"]["source_bytes"],
                            "recorded_sha256": runs["summary"]["source_recorded_sha256"],
                            "recorded_bytes": runs["summary"]["source_recorded_bytes"]},
            "point_mass_soc_reconstruction_relative_error": runs["point_mass"]["soc_error"],
            "models": {model: {
                "attempt": runs["summary"]["models"][model]["attempt"],
                "track": runs["summary"]["models"][model]["track"],
                "lap_time_s": runs["summary"]["models"][model]["lap_time_s"],
                "metrics": result["models"][model]["metrics"],
                "per_lap_speed_rmse": result["models"][model]["per_lap"]["speed_mps"],
                "per_lap_lateral_acceleration_rmse":
                    result["models"][model]["per_lap"]["lateral_acceleration_mps2"],
                "time_gap_final_s": float(result["models"][model]["time_gap_s"][-1]),
                "hashes": runs["summary"]["models"][model]["hashes"],
                "gates": ({key: runs[model]["summary"].get(key) for key, _ in GATES}
                          if model != "point_mass" else
                          {"converged": runs[model]["summary"].get("converged"),
                           "energy_converged": runs[model]["summary"].get("energy_converged")}),
                "usable": (runs[model]["summary"].get("usable") if model != "point_mass"
                           else "not applicable (optimum of a reduced problem)"),
            } for model in MODELS},
            "model_to_model_speed": result["pairs"],
        }
        (output / "comparison_summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                                        encoding="utf-8")
        render(output, result, runs)
        write_report(output, result, runs, measured, root=args.root)
    except ReportBlocker as blocker:
        print(f"BLOCKER: {blocker}", file=sys.stderr)
        return 2
    print(json.dumps({model: {"lap_time_s": summary["models"][model]["lap_time_s"],
                              "speed_rmse_mps": summary["models"][model]["metrics"]["speed_mps"]["rmse"],
                              "usable": summary["models"][model]["usable"]}
                      for model in MODELS}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
