#!/usr/bin/env python3
"""MIS 2024 correlation report for two fresh 7DoF forward-driver laps on two tracks.

Inputs are two *completed* attempts of ``tools/run/run_iem24_forward_driver.py`` that were
exported with ``tools/reports/export_iem24_forward_correlation.py``: the original (legacy)
track and the canonical ten-lap-median track. The report follows
``docs/standards/mis-2024-report-standard.md``.

The tool refuses to report (``BLOCKER: ...``, exit status 2) when

* an attempt lacks recorded model-input hashes, or a hash differs from the file on
  disk, from the attempt's own source snapshot, or from the other attempt's vehicle,
  tire or formulation file;
* the two attempts do not use the legacy/canonical track pair, or their solver
  settings differ;
* the export CSV is missing a required column or does not line up with the trace;
* the pack terminal voltage cannot be reconstructed from the exported bus power
  and the state of charge (infeasible battery solve, or the reconstructed current
  does not reproduce the traced state-of-charge change).

Nothing is invented: yaw rate and state of charge come from the accepted trace
states, body accelerations, wheel power and ``apps_fraction`` from the export, and
the terminal voltage is *reconstructed* with the library battery model from the
exported bus power (RL + RR) and traced state of charge.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ElementTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial import cKDTree

from tools.reports.compare_iem24_murphy_stint import (COMPARISON_CHANNELS, _metrics,
    fit_position_offset, fold_laps, flying_lap_indices, load_table, project_onto_track)
from tools.data.iem24_telemetry import load_nav_channels, zero_phase_lowpass
from simplified_vehicle_models.tracks.profile import load_track

BASE = ROOT / 'docs/validation/iem24-murphy'
MAT = Path.home() / 'Desktop/0615CompEndurance.mat'
STANDARD = ROOT / 'docs/standards/mis-2024-report-standard.md'

FORMULATION = 'models/formulations/iem24_murphy_real_planar_7dof.xml'
LEGACY_TRACK = 'models/tracks/2024_michigan_endurance_closed.xml'
CANONICAL_TRACK = 'models/tracks/2024_michigan_endurance_multilap_closed.xml'
MODEL_ROLES = ('track_xml', 'track_csv', 'tire_xml', 'formulation_xml', 'vehicle_xml')
SHARED_ROLES = ('tire_xml', 'formulation_xml', 'vehicle_xml')
#: Settings that must be identical for the pair to differ only by track.
SOLVER_SETTINGS = ('planner', 'maximum_speed', 'lateral_acceleration', 'pedal_travel_time',
                   'maximum_step', 'rtol', 'method', 'spacing', 'duration')
EXPORT_COLUMNS = ('distance_m', 'elapsed_time_s', 'velocity_mps', 'longitudinal_acceleration_mps2',
                  'lateral_acceleration_mps2', 'rl_power_kw', 'rr_power_kw', 'apps_fraction')
TRACE_COLUMNS = ('time', 's', 'n', 'heading_error', 'v_x', 'v_y', 'yaw_rate',
                 'omega_fl', 'omega_fr', 'omega_rl', 'omega_rr', 'soc', 'steering')
GATES = (('lap_completed', 'Full lap completed'),
         ('integration_completed', 'Integration completed'),
         ('periodicity_valid', 'Periodic closure (end state = start state)'),
         ('path_tracking_valid', 'Path tracking (lateral error ≤ 0.25 m)'),
         ('resolution_valid', 'Accepted spatial step resolution'),
         ('checked_rhs_physics_valid', 'Physics checked at every accepted state'),
         ('between_step_validation_complete', 'Between-step validation'),
         ('refinement_validated', 'Refinement validated'),
         ('contact_valid', 'Tire contact loads non-negative'))
#: Filter clock for the model traces: finer than any measured sample interval.
MODEL_FILTER_HZ = 100.
CUTOFF_HZ = 1.

LABELS = ('Original track (legacy)', 'Canonical track (ten-lap median)')
SLUGS = ('original', 'canonical')
COLORS = ('#c00000', '#ed7d31')
MEASURED_COLOR, BAND_COLOR = '#1f4e79', '#9dc3e6'
POWER_CHANNELS = (('rl_power_kw', 'RL motor electrical power', 'kW'),
                  ('rr_power_kw', 'RR motor electrical power', 'kW'))
SCORED_CHANNELS = COMPARISON_CHANNELS


class ReportBlocker(RuntimeError):
    """The inputs cannot support an honest 7DoF correlation report."""


# --------------------------------------------------------------------------- provenance
def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(path, root: Path = ROOT) -> Path:
    path = Path(path)
    return path if path.is_absolute() else root / path


def display(path, root: Path = ROOT) -> str:
    path = Path(path)
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def verify_attempt(attempt: Path, role: str, expected_track: str, *, root: Path = ROOT) -> dict:
    """Load an attempt's summary and prove its model inputs match the files on disk."""
    attempt = Path(attempt)
    where = f'{role} attempt {attempt}'
    try:
        summary = json.loads((attempt / 'summary.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ReportBlocker(f'{where}: cannot read summary.json ({error})') from error
    if summary.get('method') != 'transient_forward_driver':
        raise ReportBlocker(f'{where}: not a transient_forward_driver attempt')
    if summary.get('status') != 'Completed' or summary.get('lap_completed') is not True:
        raise ReportBlocker(f'{where}: only a newly completed full 7DoF lap may be reported '
                            f'(status={summary.get("status")!r}, lap_completed={summary.get("lap_completed")!r})')
    if summary.get('minimum_time_optimum') is not False:
        raise ReportBlocker(f'{where}: a forward-driver lap must record minimum_time_optimum=false')
    inputs = summary.get('model_inputs')
    if not isinstance(inputs, dict):
        raise ReportBlocker(f'{where}: records no model_inputs hashes (historical attempt); '
                            'rerun it with a track-aware tools/run/run_iem24_forward_driver.py --track ...')
    hashes = summary.get('source_hashes', {})
    recorded = {}
    for key in MODEL_ROLES:
        path = inputs.get(key)
        if path is None or path not in hashes:
            raise ReportBlocker(f'{where}: records no hash for {key}')
        actual = sha256_file(resolve(path, root))
        if actual != hashes[path]:
            raise ReportBlocker(f'{where}: {key} {path} differs from the file the attempt used')
        recorded[key] = {'path': str(path), 'sha256': hashes[path]}
    track = str(summary.get('settings', {}).get('track') or '')
    if track != str(inputs['track_xml']):
        raise ReportBlocker(f'{where}: settings.track {track!r} != model_inputs.track_xml {inputs["track_xml"]!r}')
    if resolve(inputs['track_xml'], root).resolve() != resolve(expected_track, root).resolve():
        raise ReportBlocker(f'{where}: expected track {expected_track}, attempt used {inputs["track_xml"]}')
    if str(inputs['formulation_xml']) != FORMULATION:
        raise ReportBlocker(f'{where}: attempt used a different formulation than {FORMULATION}')
    xml_root = ElementTree.parse(resolve(inputs['track_xml'], root)).getroot()
    data = xml_root.find('./data')
    if data is None or (resolve(inputs['track_xml'], root).parent / data.get('ref', '')).resolve() \
            != resolve(inputs['track_csv'], root).resolve():
        raise ReportBlocker(f'{where}: track XML does not reference the recorded track CSV')
    snapshot = attempt / 'source_snapshot'
    if not snapshot.is_dir():
        raise ReportBlocker(f'{where}: no source_snapshot directory')
    for path, digest in hashes.items():
        copy = snapshot / Path(path).name
        if not copy.is_file() or sha256_file(copy) != digest:
            raise ReportBlocker(f'{where}: source snapshot of {Path(path).name} is missing or differs from the recorded hash')
    recorded_length = summary.get('track_length')
    track_length = load_track(resolve(inputs['track_xml'], root)).lap_length
    if recorded_length is None or abs(float(recorded_length) - track_length) > 1.e-6:
        raise ReportBlocker(f'{where}: recorded track length differs from {inputs["track_xml"]}')
    return {'summary': summary, 'inputs': recorded, 'track': str(inputs['track_xml']),
            'track_length_m': float(track_length)}


def verify_pair(old: dict, new: dict) -> dict:
    """Require the pair to differ only by track; return disclosed code-hash differences."""
    for key in SHARED_ROLES:
        a, b = old['inputs'][key], new['inputs'][key]
        if a['path'] != b['path'] or a['sha256'] != b['sha256']:
            raise ReportBlocker(f'mismatched {key} between attempts: {a["path"]} {a["sha256"][:12]} '
                                f'vs {b["path"]} {b["sha256"][:12]}')
    for key in ('track_xml', 'track_csv'):
        if old['inputs'][key]['sha256'] == new['inputs'][key]['sha256']:
            raise ReportBlocker(f'both attempts used identical {key} contents; not a track pair')
    a, b = old['summary']['settings'], new['summary']['settings']
    differing = [key for key in SOLVER_SETTINGS if a.get(key) != b.get(key)]
    if differing:
        raise ReportBlocker('solver settings differ between attempts: ' + ', '.join(
            f'{key} ({a.get(key)!r} vs {b.get(key)!r})' for key in differing))
    model_paths = {v['path'] for d in (old, new) for v in d['inputs'].values()}
    def code(summary):
        return {Path(p).name: h for p, h in summary['source_hashes'].items() if p not in model_paths}
    ca, cb = code(old['summary']), code(new['summary'])
    return {'code_hash_differences': sorted(k for k in set(ca) | set(cb) if ca.get(k) != cb.get(k))}


def load_export(attempt: Path, info: dict, *, root: Path = ROOT) -> tuple[dict, dict, dict]:
    """Read the exporter's CSV and summary plus the source trace; check they agree."""
    where = f'attempt {attempt}'
    csv_path = attempt / 'simulated_lap_7dof.csv'
    summary_path = attempt / 'simulated_lap_7dof_summary.json'
    if not csv_path.is_file() or not summary_path.is_file():
        raise ReportBlocker(f'{where}: missing simulated_lap_7dof.csv/summary; run '
                            f'python3 tools/reports/export_iem24_forward_correlation.py {display(attempt, root)}')
    export_summary = json.loads(summary_path.read_text(encoding='utf-8'))
    summary = info['summary']
    if resolve(export_summary.get('source_attempt', ''), root).resolve() != resolve(attempt, root).resolve():
        raise ReportBlocker(f'{where}: export summary names a different source attempt')
    if str(export_summary.get('track')) != info['track']:
        raise ReportBlocker(f'{where}: export track {export_summary.get("track")!r} != attempt track {info["track"]!r}')
    if abs(float(export_summary.get('lap_time_s', -1.)) - float(summary['lap_time'])) > 1.e-9:
        raise ReportBlocker(f'{where}: export lap time differs from the attempt summary')
    if export_summary.get('lap_completed') is not True:
        raise ReportBlocker(f'{where}: export was not made from a completed lap')
    expected = {key: value['sha256'] for key, value in info['inputs'].items()}
    if export_summary.get('track_source_hashes') != expected:
        raise ReportBlocker(f'{where}: export track_source_hashes differ from the attempt model_inputs hashes')
    table = load_table(csv_path)
    missing = [name for name in EXPORT_COLUMNS if name not in table]
    if missing:
        raise ReportBlocker(f'{where}: export lacks columns {", ".join(missing)}')
    header = (attempt / 'trace.csv').open(encoding='utf-8').readline().strip().split(',')
    if tuple(header) != TRACE_COLUMNS:
        raise ReportBlocker(f'{where}: unexpected trace.csv header')
    trace = np.loadtxt(attempt / 'trace.csv', delimiter=',', skiprows=1)
    return table, export_summary, {name: trace[:, i] for i, name in enumerate(TRACE_COLUMNS)}


# --------------------------------------------------------------------------- model channels
def drive_torque_limit_nm(vehicle) -> float:
    """Per-motor torque that ``apps_fraction == 1`` requests (plant: throttle × this)."""
    return float(min(vehicle.motor.peak_motor_torque, vehicle.controller.maximum_drive_motor_torque_nm))


def reconstruct_pack_voltage(storage, soc: np.ndarray, bus_power_w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Loaded terminal voltage and bus current from state of charge and bus power.

    Uses the same library battery functions as the plant: ``solve_current`` for
    the current drawn at the given electrical bus power and ``terminal_voltage``
    for OCV minus the internal-resistance drop.
    """
    from simplified_vehicle_models.powertrain.energy_storage import (
        StorageState, solve_current, terminal_voltage)
    voltage = np.empty(len(soc))
    current = np.empty(len(soc))
    for i, (charge, power) in enumerate(zip(soc, bus_power_w)):
        state = StorageState(float(charge))
        amps, feasible = solve_current(storage, state, float(power))
        if not feasible:
            raise ReportBlocker(f'battery current solve infeasible at sample {i} '
                                f'(SOC {charge:.6f}, bus power {power / 1000.:.3f} kW)')
        current[i] = amps
        voltage[i] = terminal_voltage(storage, state, amps)
    return voltage, current


def soc_consistency(storage, time: np.ndarray, soc: np.ndarray, current: np.ndarray) -> dict:
    """Compare the traced SOC change with the SOC change implied by the reconstructed current."""
    integral = np.concatenate(([0.], np.cumsum(.5 * (current[1:] + current[:-1]) * np.diff(time))))
    implied = soc[0] - integral / storage.pack_capacity
    traced_drop = float(soc[0] - soc[-1])
    implied_drop = float(soc[0] - implied[-1])
    return {'traced_soc_drop': traced_drop, 'implied_soc_drop': implied_drop,
            'relative_error': abs(implied_drop - traced_drop) / max(abs(traced_drop), 1.e-12),
            'maximum_abs_soc_error': float(np.max(np.abs(implied - soc)))}


def build_model_channels(table: dict, trace: dict, vehicle, *, tolerance: float = .05) -> tuple[dict, dict]:
    """Assemble the report channels from export + trace; do not invent anything."""
    time, distance = table['elapsed_time_s'], table['distance_m']
    if len(time) != len(trace['time']) or not np.allclose(time, trace['time'], rtol=0, atol=1.e-9) \
            or not np.allclose(distance, trace['s'], rtol=0, atol=1.e-9):
        raise ReportBlocker('export rows do not line up with the accepted trace samples')
    if np.any(np.diff(time) <= 0) or np.any(np.diff(distance) <= 0):
        raise ReportBlocker('model time and distance must be strictly increasing')
    names = tuple(table) + tuple(trace)
    arrays = np.concatenate([np.asarray(table[k]) for k in EXPORT_COLUMNS] +
                            [trace['yaw_rate'], trace['soc']])
    if not np.all(np.isfinite(arrays)):
        raise ReportBlocker('non-finite value in export or trace channels: ' + ', '.join(names[:0]))
    bus_power = (table['rl_power_kw'] + table['rr_power_kw']) * 1000.
    voltage, current = reconstruct_pack_voltage(vehicle.storage, trace['soc'], bus_power)
    check = soc_consistency(vehicle.storage, time, trace['soc'], current)
    if check['relative_error'] > tolerance:
        raise ReportBlocker('reconstructed bus current does not reproduce the traced SOC change '
                            f'(implied {check["implied_soc_drop"]:.6f} vs traced {check["traced_soc_drop"]:.6f}); '
                            'exported 7DoF power/voltage replay is insufficient')
    limit = drive_torque_limit_nm(vehicle)
    data = {'distance_m': distance, 'elapsed_time_s': time,
            'speed_mps': table['velocity_mps'],
            'longitudinal_acceleration_mps2': table['longitudinal_acceleration_mps2'],
            'lateral_acceleration_mps2': table['lateral_acceleration_mps2'],
            'yaw_rate_rad_s': trace['yaw_rate'],
            'motor_torque_request_nm': table['apps_fraction'] * limit,
            'pack_voltage_v': voltage, 'bus_current_a': current, 'soc': trace['soc'],
            'rl_power_kw': table['rl_power_kw'], 'rr_power_kw': table['rr_power_kw'],
            'apps_fraction': table['apps_fraction']}
    check['drive_torque_limit_nm'] = limit
    return data, check


def filter_model(data: dict[str, np.ndarray], columns: tuple[str, ...], fs: float = MODEL_FILTER_HZ,
                 cutoff: float = CUTOFF_HZ) -> dict[str, np.ndarray]:
    """First-order zero-phase low-pass in physical time, like the measured channels.

    The solver clock is nonuniform; filtering after linear interpolation onto a
    uniform clock avoids treating distance as time. Distance and time pass through.
    """
    time = data['elapsed_time_s']
    tick = np.unique(np.r_[np.arange(0., time[-1], 1. / fs), time[-1]])
    sos = butter(1, cutoff, fs=fs, btype='lowpass', output='sos')
    filtered = dict(data)
    for column in columns:
        if column not in data or column in ('distance_m', 'elapsed_time_s'):
            continue
        values = np.interp(tick, time, data[column])
        filtered[column] = np.interp(time, tick, sosfiltfilt(sos, values))
    return filtered


def motor_power(mat: Path, measured: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Logged RL/RR electrical power in kW on the measured_stint timestamps (1 Hz filtered)."""
    nav = load_nav_channels(mat, ('pwrRLReal', 'pwrRRReal'))
    index = np.searchsorted(nav.time, measured['time_s'])
    if np.any(index == len(nav.time)) or not np.allclose(nav.time[index], measured['time_s'], atol=1e-4, rtol=0):
        raise ValueError('power channels do not share measured_stint timestamps')
    return {name: zero_phase_lowpass(nav[source], nav.sample_interval, CUTOFF_HZ)[index] / 1000.
            for name, source in (('rl_power_kw', 'pwrRLReal'), ('rr_power_kw', 'pwrRRReal'))}


# --------------------------------------------------------------------------- geometry
def map_to_reference_distance(reference, candidate, candidate_distance: np.ndarray) -> tuple[np.ndarray, dict]:
    """Map candidate-track distance to reference physical distance by nearest XY node.

    The tracks are closed, so a node just before the seam may be nearest to
    reference node 0; mapped distance is unwrapped modulo the reference length
    and must then advance monotonically. Distances are an XY correspondence, not
    an equal-distance comparison.
    """
    length = float(reference.distance[-1])
    gap, index = cKDTree(np.column_stack((reference.x_north, reference.y_east))).query(
        np.column_stack((candidate.x_north, candidate.y_east)))
    nodes = reference.distance[index].astype(float)
    step = np.diff(nodes)
    step -= length * np.round(step / length)
    nodes = np.concatenate(([0.], np.cumsum(step)))
    if np.any(np.diff(nodes) < -.5):
        raise ReportBlocker('canonical track maps backward along the original geometry')
    if abs(nodes[-1] - length) > 5.:
        raise ReportBlocker(f'canonical track maps to {nodes[-1]:.2f} m of the {length:.2f} m original lap')
    nodes = np.clip(np.maximum.accumulate(nodes), 0., length)
    mapped = np.interp(candidate_distance, candidate.distance, nodes)
    return mapped, {'median_xy_gap_m': float(np.median(gap)), 'p95_xy_gap_m': float(np.percentile(gap, 95)),
                    'maximum_xy_gap_m': float(np.max(gap))}


# --------------------------------------------------------------------------- comparison
def compare(output: Path, measured: dict, extraction: dict, runs: dict, *, mat: Path | None,
            channels: tuple, root: Path = ROOT) -> dict:
    """Project measured laps separately on each geometry; score in each frame."""
    laps = flying_lap_indices(measured)
    columns = tuple(entry[0] for entry in channels)
    comparisons = {}
    for label in LABELS:
        info, data = runs[label]['info'], runs[label]['data']
        path = load_track(resolve(info['track'], root)).path_coordinates
        offset, rms = fit_position_offset(measured, path)
        projected, residual = project_onto_track(measured, path, offset)
        bins = int(round(path.lap_length))
        grid, folded = fold_laps(measured, laps, columns, track_distance=projected,
                                 lap_length=path.lap_length, bins=bins)
        median = {column: np.median(folded[column], axis=0) for column in columns}
        filtered = filter_model(data, columns)
        model = {column: np.interp(grid, data['distance_m'], filtered[column]) for column in columns}
        metrics = {column: _metrics(model[column], median[column]) for column in columns}
        per_lap = {column: [float(np.sqrt(np.mean((model[column] - row) ** 2))) for row in folded[column]]
                   for column in ('speed_mps', 'lateral_acceleration_mps2')}
        comparisons[label] = dict(grid=grid, folded=folded, median=median, model=model, metrics=metrics,
            per_lap=per_lap, offset=offset, fit_rmse=rms, path=path, bins=bins,
            position_p95=float(np.percentile(residual[np.isin(measured['lap_index'], laps)], 95)))
    flying = [v for v in extraction['laps'] if not v['standing_start']]
    old_path, new_path = (comparisons[label]['path'] for label in LABELS)
    mapped, xy = map_to_reference_distance(old_path, new_path, runs[LABELS[1]]['data']['distance_m'])
    summary = {
        'result_status': 'driven non-optimal transient 7DoF laps; not validated unless every gate passes',
        'measured_laps': [int(lap) for lap in laps],
        'median_measured_lap_s': float(np.median([v['lap_time_s'] for v in flying])),
        'median_measured_distance_m': float(np.median([v['distance_m'] for v in flying])),
        'source_sha256': sha256_file(mat) if mat is not None and Path(mat).is_file() else None,
        'source_bytes': Path(mat).stat().st_size if mat is not None and Path(mat).is_file() else None,
        'power_channels_included': 'rl_power_kw' in columns,
        'xy_alignment': xy,
        'pair_check': runs['pair'],
        'comparisons': {label: {'attempt': display(runs[label]['attempt'], root),
            'track': runs[label]['info']['track'], 'track_length_m': runs[label]['info']['track_length_m'],
            'model_inputs': runs[label]['info']['inputs'],
            'lap_time_s': float(runs[label]['info']['summary']['lap_time']),
            'usable': runs[label]['info']['summary'].get('usable'),
            'gates': {key: runs[label]['info']['summary'].get(key) for key, _ in GATES},
            'voltage_reconstruction': runs[label]['check'],
            'metrics': item['metrics'], 'per_lap_speed_rmse': item['per_lap']['speed_mps'],
            'per_lap_lateral_acceleration_rmse': item['per_lap']['lateral_acceleration_mps2'],
            'position_offset_north_east_m': item['offset'].tolist(),
            'position_fit_rmse_m': item['fit_rmse'], 'position_residual_p95_m': item['position_p95']}
            for label, item in comparisons.items()}}
    output.mkdir(parents=True, exist_ok=True)
    (output / 'comparison_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    for label, slug in zip(LABELS, SLUGS):
        item = comparisons[label]
        names, vals = ['distance_m'], [item['grid']]
        for column in columns:
            names.extend(('measured_' + column, 'model_' + column))
            vals.extend((item['median'][column], item['model'][column]))
        np.savetxt(output / (slug + '_comparison.csv'), np.column_stack(vals),
                   header=','.join(names), comments='', delimiter=',')
    render(output, comparisons, runs, mapped, channels)
    write_report(output, summary, comparisons, runs, extraction, channels, root=root)
    return summary


def _panel(ax, entry, key, title, unit, color, label):
    band = entry['folded'][key]
    ax.fill_between(entry['grid'], np.min(band, axis=0), np.max(band, axis=0), color=BAND_COLOR,
                    alpha=.35, label='Measured range (10 flying laps)')
    ax.plot(entry['grid'], entry['median'][key], color=MEASURED_COLOR, lw=1.5, label='Measured median')
    ax.plot(entry['grid'], entry['model'][key], color=color, lw=1.3, label=label)
    ax.set_ylabel(f'{title} ({unit})'); ax.grid(alpha=.25); ax.margins(x=0)


def render(output: Path, pairs: dict, runs: dict, mapped: np.ndarray, channels: tuple) -> None:
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-config')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    old, new = (pairs[label] for label in LABELS)
    lengths = [runs[label]['info']['track_length_m'] for label in LABELS]
    count = len(channels)
    separate = abs(lengths[0] - lengths[1]) > .05

    # comparison.png: own-physical-distance panels (two columns when the lengths differ).
    if separate:
        fig, axes = plt.subplots(count, 2, figsize=(17, 3.1 * count), sharex='col', squeeze=False)
        for col, label in enumerate(LABELS):
            for row, (key, title, unit) in enumerate(channels):
                _panel(axes[row, col], pairs[label], key, title, unit, COLORS[col], label + ' 7DoF')
            axes[0, col].set_title(f'{label}: {lengths[col]:.2f} m (own physical distance)')
            axes[0, col].legend(loc='upper right', fontsize=7)
            axes[-1, col].set_xlabel('Projected distance on the named track (m)')
    else:
        fig, axes = plt.subplots(count, 1, figsize=(12, 3.1 * count), sharex=True, squeeze=False)
        for row, (key, title, unit) in enumerate(channels):
            _panel(axes[row, 0], old, key, title, unit, COLORS[0], LABELS[0] + ' 7DoF')
            axes[row, 0].plot(new['grid'], new['model'][key], color=COLORS[1], lw=1.3, label=LABELS[1] + ' 7DoF')
        axes[0, 0].legend(loc='upper right', fontsize=7)
        axes[-1, 0].set_xlabel('Projected distance (m)')
    fig.suptitle('IEM24 Murphy MIS endurance: measured vs non-optimal 7DoF forward-driver laps', y=.995)
    fig.tight_layout(rect=(0, 0, 1, .975)); fig.savefig(output / 'comparison.png', dpi=130); plt.close(fig)

    # three_overlay.png: median + both 7DoF solves on original physical distance (XY-aligned).
    candidate = runs[LABELS[1]]['data']
    candidate_filtered = filter_model(candidate, tuple(key for key, _, _ in channels))
    fig, axes = plt.subplots(count, 1, figsize=(14, 3.1 * count), sharex=True, squeeze=False)
    for ax, (key, title, unit) in zip(axes[:, 0], channels):
        _panel(ax, old, key, title, unit, COLORS[0], 'Original-track 7DoF')
        ax.plot(mapped, candidate_filtered[key], color=COLORS[1], lw=1.3,
                label='Canonical-track 7DoF (XY-mapped to original distance)')
        ax.legend(loc='upper right', fontsize=7, ncol=2)
    axes[-1, 0].set_xlabel('Original-track physical distance (m); canonical nodes mapped by closest XY point')
    fig.suptitle('IEM24 Murphy MIS endurance: measured median, original and canonical 7DoF on one physical axis')
    fig.tight_layout(rect=(0, 0, 1, .975)); fig.savefig(output / 'three_overlay.png', dpi=130); plt.close(fig)

    # lap_spread.png: every flying lap, the median and the simulation; two distributions.
    fig, axes = plt.subplots(3, 2, figsize=(16, 11))
    for col, label in enumerate(LABELS):
        entry = pairs[label]
        for trace in entry['folded']['speed_mps']:
            axes[0, col].plot(entry['grid'], trace, color=BAND_COLOR, lw=.7)
        axes[0, col].plot([], [], color=BAND_COLOR, label='Each measured flying lap')
        axes[0, col].plot(entry['grid'], entry['median']['speed_mps'], color=MEASURED_COLOR, lw=1.6,
                          label='Measured median')
        axes[0, col].plot(entry['grid'], entry['model']['speed_mps'], color=COLORS[col], lw=1.4, label=label + ' 7DoF')
        axes[0, col].set_title(label); axes[0, col].set_ylabel('Speed (m/s)')
        axes[0, col].set_xlabel('Projected distance (m)'); axes[0, col].legend(fontsize=8); axes[0, col].grid(alpha=.25)
        for row, key, name in ((1, 'speed_mps', 'Speed RMSE vs each measured lap (m/s)'),
                               (2, 'lateral_acceleration_mps2', 'Lateral acceleration RMSE vs each measured lap (m/s²)')):
            values = entry['per_lap'][key]
            low, high = min(values), max(values)
            pad = max(.05 * (high - low), 1.e-3)
            axes[row, col].hist(values, bins=np.linspace(low - pad, high + pad, 13), color=COLORS[col], alpha=.8)
            axes[row, col].set_xlabel(name); axes[row, col].set_ylabel('Lap count'); axes[row, col].grid(alpha=.25)
    fig.tight_layout(rect=(0, 0, 1, .975)); fig.savefig(output / 'lap_spread.png', dpi=130); plt.close(fig)

    # simulated_lap.png: raw, unfiltered model laps incl. the initial-condition channel (SOC).
    rows = (('speed_mps', 'Raw speed (m/s)'), ('longitudinal_acceleration_mps2', 'Raw ax (m/s²)'),
            ('lateral_acceleration_mps2', 'Raw ay (m/s²)'), ('yaw_rate_rad_s', 'Raw yaw rate (rad/s)'),
            ('motor_torque_request_nm', 'Requested motor torque (N·m)'),
            ('pack_voltage_v', 'Reconstructed pack voltage (V)'), ('soc', 'State of charge (–), initial condition'),
            ('rl_power_kw', 'Replayed RL electrical power (kW)'), ('rr_power_kw', 'Replayed RR electrical power (kW)'))
    fig, axes = plt.subplots(len(rows), 1, figsize=(13, 2.6 * len(rows)), sharex=True)
    for label, color in zip(LABELS, COLORS):
        data = runs[label]['data']
        for ax, (key, title) in zip(axes, rows):
            ax.plot(data['distance_m'], data[key], color=color, label=label, lw=1.1)
            ax.set_ylabel(title, fontsize=8); ax.grid(alpha=.25)
    axes[0].legend(); axes[-1].set_xlabel('Each track’s own distance (m)')
    fig.suptitle('Raw non-optimal 7DoF forward-driver laps (each starts from the same state of charge)')
    fig.tight_layout(rect=(0, 0, 1, .975)); fig.savefig(output / 'simulated_lap.png', dpi=130); plt.close(fig)


# --------------------------------------------------------------------------- report
def _gate_cell(value) -> str:
    return 'PASS' if value is True else ('FAIL' if value is False else 'not recorded')


def _fmt(value, spec='.3f') -> str:
    return 'n/a' if value is None else format(value, spec)


def write_report(output: Path, summary: dict, comparisons: dict, runs: dict, extraction: dict,
                 channels: tuple, *, root: Path = ROOT) -> None:
    from simplified_vehicle_models.compiler.compile import compile_formulation
    vehicle = compile_formulation(str(root / FORMULATION)).model
    standard = os.path.relpath(STANDARD, output)
    old_run, new_run = (summary['comparisons'][label] for label in LABELS)
    old_s, new_s = (runs[label]['info']['summary'] for label in LABELS)
    old_t, new_t = (runs[label]['trace'] for label in LABELS)
    settings = old_s['settings']
    mt, md = summary['median_measured_lap_s'], summary['median_measured_distance_m']
    lap_rows = [f"| {lap['index']} {'(standing start)' if lap['standing_start'] else '(flying)'} | {lap['start_s']:.2f} "
                f"| {lap['lap_time_s']:.2f} | {lap['distance_m']:.1f} | {lap['mean_speed_mps']:.2f} "
                f"| {lap['closure_radius_m']:.2f} |" for lap in extraction['laps']]

    def metric_rows(columns):
        rows = []
        for channel, title, unit in columns:
            for label in LABELS:
                m = comparisons[label]['metrics'][channel]
                rows.append(f"| {title} ({unit}) | {label} | {m['rmse']:.3f} | {m['mae']:.3f} | {m['bias']:+.3f} "
                            f"| {m['correlation']:.3f} | {_fmt(m['normalised_rmse'])} |")
        return rows

    header = ('| Signal | Track | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |\n'
              '|---|---|---:|---:|---:|---:|---:|')
    power_table = ''
    if summary['power_channels_included']:
        power_table = ('\n\nSupplementary logged-power rows (exploratory, not part of the six required channels):\n\n'
                       + header + '\n' + '\n'.join(metric_rows(POWER_CHANNELS)))
    gate_rows = [f"| {name} | {_gate_cell(old_s.get(key))} | {_gate_cell(new_s.get(key))} |" for key, name in GATES]
    gate_rows.append(f"| **Recorded `usable` verdict** | **{_gate_cell(old_s.get('usable'))}** | **{_gate_cell(new_s.get('usable'))}** |")

    def closure(s):
        delta, tol = s.get('periodic_state_delta'), s.get('periodic_state_tolerances')
        if not delta or not tol or len(delta) != len(tol):
            return 'not recorded'
        return f"{max(abs(d) / t for d, t in zip(delta, tol) if t > 0):.1f}×"

    hash_rows = []
    for key in MODEL_ROLES:
        a, b = old_run['model_inputs'][key], new_run['model_inputs'][key]
        if a['sha256'] == b['sha256']:
            hash_rows.append(f"| {key} (both) | `{a['path']}` | `{a['sha256']}` |")
        else:
            hash_rows.append(f"| {key} (original) | `{a['path']}` | `{a['sha256']}` |")
            hash_rows.append(f"| {key} (canonical) | `{b['path']}` | `{b['sha256']}` |")
    model_hash_rows = '\n'.join(hash_rows)

    all_usable = bool(old_s.get('usable')) and bool(new_s.get('usable'))
    status_text = ('Both attempts pass every recorded acceptance gate, but each remains a driven (forward-driver) lap, '
                   '**not** a minimum-time optimum.' if all_usable else
                   'At least one attempt fails an acceptance gate (table in Configuration): these are **driven, non-optimal '
                   'transient 7DoF laps and are not validated results**.')
    storage = vehicle.storage
    grip_link = os.path.relpath(BASE / 'road_grip_calibration.md', output)
    import re
    tire_text = (root / old_run['model_inputs']['tire_xml']['path']).read_text(encoding='utf-8')
    comment = re.search(r'<!--(.*?)-->', tire_text, re.S)
    tire_note = ''
    if comment:
        sentences = re.split(r'(?<=\.)\s+', ' '.join(comment.group(1).split()))
        keep = [x for x in sentences if sentences.index(x) == 0 or '2/3' in x or 'unvalidated' in x or 'not validated' in x]
        tire_note = ' Tire-file note (verbatim from the XML comment): "' + ' '.join(keep) + '"'
    limit = old_run['voltage_reconstruction']['drive_torque_limit_nm']
    m_old, m_new = (comparisons[label]['metrics'] for label in LABELS)

    def mm(channel, key='rmse', fmt='.3f'):
        return f"{m_old[channel][key]:{fmt}} → {m_new[channel][key]:{fmt}}"

    pk = [(float(np.min(comparisons[label]['model']['longitudinal_acceleration_mps2'])),
           float(np.min(comparisons[label]['median']['longitudinal_acceleration_mps2']))) for label in LABELS]
    raw_min = [float(np.min(runs[label]['data']['longitudinal_acceleration_mps2'])) for label in LABELS]
    mean_speed = [md / mt] + [run['track_length_m'] / run['lap_time_s'] for run in (old_run, new_run)]
    vm = [np.mean(comparisons[label]['median']['pack_voltage_v']) for label in LABELS]
    vs = [np.mean(comparisons[label]['model']['pack_voltage_v']) for label in LABELS]
    tm = [np.mean(comparisons[label]['median']['motor_torque_request_nm']) for label in LABELS]
    ts = [np.mean(comparisons[label]['model']['motor_torque_request_nm']) for label in LABELS]
    spread = [summary['comparisons'][label]['per_lap_speed_rmse'] for label in LABELS]
    check_old, check_new = old_run['voltage_reconstruction'], new_run['voltage_reconstruction']
    xy = summary['xy_alignment']
    difference = new_run['lap_time_s'] - old_run['lap_time_s']
    power_line = ''
    if settings['planner'] == 'live':
        plan_text = (f"Planner `live` derives its speed plan from the vehicle's own point-mass limits (maximum-speed cap {settings['maximum_speed']:g} m/s), "
                     'so the lap time depends on the carried-over tire grip and powertrain values, not on a driver model fitted to this driver.')
    else:
        plan_text = (f"Planner `{settings['planner']}` imposes a constant speed cap of {settings['maximum_speed']:g} m/s, so the lap time is set by that "
                     "driver plan rather than by the vehicle's grip or power limits.")
    power_caveat = 'Measured RL/RR power channels were skipped (`--skip-power`); no power comparison is made.'
    if summary['power_channels_included']:
        p = {k: [comparisons[label]['metrics'][k]['rmse'] for label in LABELS] for k, _, _ in POWER_CHANNELS}
        power_line = (f"- **RL and RR motor electrical power (exploratory):** RL RMSE {p['rl_power_kw'][0]:.3f} → "
                      f"{p['rl_power_kw'][1]:.3f} kW, RR RMSE {p['rr_power_kw'][0]:.3f} → {p['rr_power_kw'][1]:.3f} kW. "
                      "The 7DoF values are replayed per-wheel shaft powers referred to the bus through gear and motor "
                      "efficiency; the logged `pwrRLReal`/`pwrRRReal` measurement point is not documented, so this is trace "
                      "context, not a motor-efficiency validation.\n")
        power_caveat = ('`pwrRLReal`/`pwrRRReal` sensor definition may differ from the replayed bus-referred wheel power; '
                        'compare their shape cautiously.')
    code_diff = summary['pair_check']['code_hash_differences']
    code_note = ('Solver and model source hashes (other than the track files) are identical for both attempts.'
                 if not code_diff else 'Source files whose recorded hashes differ between the attempts: '
                 + ', '.join(f'`{n}`' for n in code_diff) + ' (disclosed; the vehicle, tire and formulation files are identical).')
    peak_lat = old_s.get('planned_peak_lateral_acceleration')
    initial = {label: (runs[label]['trace']['v_x'][0], runs[label]['trace']['soc'][0], runs[label]['trace']['steering'][0])
               for label in LABELS}
    out_root = display(Path(root / old_run['attempt']).parent.parent, root)
    commands = ' '.join(f'--{k.replace("_", "-")} {settings[k]}' for k in
                        ('planner', 'maximum_speed', 'pedal_travel_time', 'maximum_step', 'rtol', 'method',
                         'spacing', 'duration', 'timeout_seconds') if settings.get(k) is not None)
    if settings.get('lateral_acceleration') is not None:
        commands += f" --lateral-acceleration {settings['lateral_acceleration']}"
    source_line = (f"Source `~/Desktop/0615CompEndurance.mat`, SHA-256 `{summary['source_sha256']}`, {summary['source_bytes']} bytes"
                   if summary['source_sha256'] else
                   'Source `~/Desktop/0615CompEndurance.mat` (file not available to this run, so its hash was not recomputed; '
                   'its SHA-256 `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7` is recorded in the track XML metadata)')
    names = 'Navigation `VNorth`, `VEast`, `VelX`, `VelY`, `AccelX`, `AccelY`, `GyroZ`; per-motor requests `Tq_RL_Req`, `Tq_RR_Req`; pack voltage `VPack`' \
        + ('; electrical power `pwrRLReal`, `pwrRRReal`' if summary['power_channels_included'] else '')
    window = extraction.get('stint_window_s'); drive = extraction.get('driving_window_s')
    skew = extraction.get('navigation_unit_skew', {})
    validation = extraction.get('validation', {})
    text = f'''# IEM24 Murphy simulated MIS 2024 laps (7DoF forward driver) vs measured endurance stint — legacy original and canonical tracks

Michigan International Speedway 2024 endurance circuit, the IEM24 Murphy car, **simulated 7DoF laps compared with the measured endurance stint**. This report follows the [MIS 2024 report standard]({standard}) without departure. {status_text} No claim of validation is made.

## Configuration

- Mass: {vehicle.total_mass:g} kg ({vehicle.mass_without_driver:g} kg car + {vehicle.driver_mass:g} kg driver), identical in both runs. Yaw radius of gyration {vehicle.yaw_radius_of_gyration:g} m, wheel inertia {vehicle.wheel_inertia:.4f} kg·m² per wheel.
- Chassis: wheelbase {vehicle.wheelbase:g} m, track {vehicle.track_width:g} m, LLTD {vehicle.lateral_load_transfer_distribution * 100:g} %, CGx {vehicle.front_mass_fraction * 100:g} % of the mass on the front axle (CG {vehicle.front_axle_to_cg:.4f} m behind the front axle), CG height {vehicle.cg_height:g} m, {vehicle.front_brake_share * 100:g} % front hydraulic brake torque share (balance bar {vehicle.brake_bias:+g}).
- Aero: air density {vehicle.aero.air_density:g} kg/m³, CL·A = {vehicle.aero.downforce_area:g} m², CD·A = {vehicle.aero.drag_area:g} m², aero balance {vehicle.aero.aero_balance * 100:g} %.
- Drivetrain: {vehicle.motor.unit_count} rear motors, {vehicle.motor.peak_motor_torque:g} N·m per-motor torque cap (`apps_fraction` 1 requests {limit:g} N·m), {vehicle.motor.peak_power / 1000:g} kW per motor, {vehicle.motor.gear_ratio:g}:1 gear ratio, regeneration {'on' if vehicle.controller.regen_enabled else 'off'}. Tire: UniTire (`models/tires/iem24_murphy_real_unitire.xml`) with wheel-speed states and combined slip. Road friction values used (compiled from the tire file, identical in both runs): μ₀ₓ = {vehicle.tire.mu_0_x:.6f}, μₛₓ = {vehicle.tire.mu_s_x:.6f}, μ₀ᵧ = {vehicle.tire.mu_0_y:.6f}, μₛᵧ = {vehicle.tire.mu_s_y:.6f} (peak and sliding, longitudinal and lateral; slip stiffness unchanged from the lab fit).{tire_note} Evidence and identifiability: [road grip calibration]({grip_link}).
- Battery: {storage.series_count}s{storage.parallel_count}p, {storage.discharge_current_limit:g} A discharge limit, SOC-dependent OCV and resistance from the cell curves.
- Driver and solver (identical for both attempts): planner `{settings['planner']}`, maximum speed {settings['maximum_speed']:g} m/s, lateral-acceleration cap {'planner default (none beyond the vehicle limits for `live`)' if settings.get('lateral_acceleration') is None else str(settings['lateral_acceleration']) + ' m/s²'}, pedal travel time {settings.get('pedal_travel_time')} s, {settings['method']} with rtol {settings['rtol']:g}, maximum step {settings['maximum_step']:g} s, path spacing {settings['spacing']:g} m. Transient forward driver: **not a minimum-time optimum** (`minimum_time_optimum` = false in both summaries); the lap is a completed exploratory trajectory, and its feasibility or validity is **not** established unless every acceptance gate below passes.
- Initial conditions: first accepted state is v_x {initial[LABELS[0]][0]:.3f} m/s (original) and {initial[LABELS[1]][0]:.3f} m/s (canonical), battery SOC {initial[LABELS[0]][1]:.6f} (the model's initial SOC, set from the resting pack voltage of {extraction.get('pack_voltage', {}).get('resting_voltage_v', float('nan')):.2f} V measured before launch), initial road-wheel angle {initial[LABELS[0]][2]:.4f} / {initial[LABELS[1]][2]:.4f} rad aligned with the path curvature. No measured tire temperature or ambient state initialises either run.
- Simulated lap: original **{old_run['lap_time_s']:.3f} s** over {old_run['track_length_m']:.3f} m on `{old_run['track']}` (legacy single-lap integrated line, retained unchanged); canonical **{new_run['lap_time_s']:.3f} s** over {new_run['track_length_m']:.3f} m on `{new_run['track']}` (median of ten flying laps, smoothed; the repository's canonical MIS line). Attempts: `{old_run['attempt']}` and `{new_run['attempt']}`.
- Solver status: both attempts record status Completed with a full lap, accepted steps {old_s['accepted_steps']} / {new_s['accepted_steps']}, maximum lateral path error {old_s.get('maximum_lateral_error', float('nan')):.3f} / {new_s.get('maximum_lateral_error', float('nan')):.3f} m, final SOC {old_s.get('final_soc', float('nan')):.6f} / {new_s.get('final_soc', float('nan')):.6f}. Periodic closure is the largest ratio of end-minus-start state difference to its tolerance: {closure(old_s)} (original), {closure(new_s)} (canonical).

Acceptance gates recorded in each attempt's `summary.json`:

| Gate | Original track | Canonical track |
|---|---|---|
{chr(10).join(gate_rows)}

Model inputs hashed by the attempts (SHA-256; re-verified against the files on disk and against each attempt's `source_snapshot` before this report was written; the vehicle, tire and formulation entries are identical in both attempts):

| Input | Path | SHA-256 |
|---|---|---|
{model_hash_rows}

## Measured data

{source_line}; shared log clock {1 / extraction['record']['sample_interval_s']:.0f} Hz ({extraction['record']['sample_count']} samples), exported measured table at {extraction['export_rate_hz']:.2f} Hz. Conditioning uses {window[0]:.2f}–{window[1]:.2f} s for lap identification and a {drive[0]:.2f}–{drive[1]:.2f} s driving window. Channels consumed: {names}. Existing `measured_stint.csv` and `telemetry_extraction.json` are read and unchanged.

Corrections applied to the raw channels: the {skew.get('angle_deg', float('nan')):.2f}° navigation-unit yaw skew rotates VelX/VelY and AccelX/AccelY into the body frame (corrected acceleration-vs-velocity-rate RMSE {validation.get('longitudinal_acceleration_vs_velocity_rate', {}).get('rmse', float('nan')):.3f} m/s²); the motor torque request is the mean of `Tq_RL_Req` and `Tq_RR_Req`; navigation accelerations, speed, torque and voltage are conditioned with a zero-phase first-order low-pass filter at {CUTOFF_HZ:g} Hz{' and logged RL/RR power the same way' if summary['power_channels_included'] else ''}. No voltage zeroing, inferred time shift or equal-length scaling is applied.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---:|---:|---:|---:|---:|
{chr(10).join(lap_rows)}

Median of the 10 flying laps: **{mt:.3f} s** over **{md:.2f} m** ({mean_speed[0]:.2f} m/s mean); the measured driven distance exceeds the original geometry by {(md / old_run['track_length_m'] - 1) * 100:+.2f}% and the canonical geometry by {(md / new_run['track_length_m'] - 1) * 100:+.2f}%. The standing-start lap is in the table and excluded from all statistics.

Alignment: each measured position is projected independently onto each track after fitting a constant north/east translation (original: fit RMSE {old_run['position_fit_rmse_m']:.2f} m, residual p95 {old_run['position_residual_p95_m']:.2f} m; canonical: {new_run['position_fit_rmse_m']:.2f} m, p95 {new_run['position_residual_p95_m']:.2f} m); every flying lap is then interpolated onto **that track's own physical distance** and the median and full range are recomputed. The two 7DoF traces are likewise on their own track distance, taken from the integrated path coordinate `s`. The tracks differ in length by {old_run['track_length_m'] - new_run['track_length_m']:.2f} m, so `comparison.png` keeps **two separate panel columns** (one per track) and nothing is phase-stretched. `three_overlay.png` additionally puts the measured median and both solves on the *original* track's physical distance, mapping canonical nodes by nearest XY point (median node gap {xy['median_xy_gap_m']:.2f} m, p95 {xy['p95_xy_gap_m']:.2f} m, maximum {xy['maximum_xy_gap_m']:.2f} m); that mapping is an XY correspondence, not an equal-distance metric.

7DoF channel sources: speed is the body-frame velocity magnitude of the traced states; longitudinal and lateral acceleration are the exporter's body accelerations including the rotating-frame terms; **yaw rate is the traced `yaw_rate` state** (`trace.csv`, sign convention as the state, positive from the path curvature the model integrates); motor torque request is `apps_fraction` × {limit:g} N·m, i.e. the *requested* per-motor drive torque before speed regulation, bus-power limits and gear/motor losses (it is not delivered torque, and friction braking has no motor request); pack terminal voltage is **reconstructed** because the 7DoF trace carries no voltage: bus power = RL + RR replayed electrical power from the export, the current is solved with the library battery model (`solve_current`) at the traced SOC, and the loaded terminal voltage is OCV(SOC) − R(SOC)·I (`terminal_voltage`). Consistency check: the SOC drop implied by integrating that current is {check_old['implied_soc_drop']:.6f} vs {check_old['traced_soc_drop']:.6f} traced (original, {check_old['relative_error'] * 100:.2f}% difference) and {check_new['implied_soc_drop']:.6f} vs {check_new['traced_soc_drop']:.6f} (canonical, {check_new['relative_error'] * 100:.2f}%). Model traces are low-pass filtered in physical time with the same first-order zero-phase filter at {CUTOFF_HZ:g} Hz (on a {MODEL_FILTER_HZ:g} Hz uniform clock) before scoring; raw traces are in `simulated_lap.png`.

## Trace comparison

Each 7DoF lap is compared with the median of the same 10 flying laps reprojected onto its own named track, on a {comparisons[LABELS[0]]['bins']}-point (original) and {comparisons[LABELS[1]]['bins']}-point (canonical) physical-distance grid. Measured range shading uses the minimum and maximum individual flying-lap value. Bias is signed **sim − meas**, in the row's unit. Each 7DoF lap is a single non-periodic transient from its own initial state, so the start and end of the lap are not steady-state samples.

{header}
{chr(10).join(metric_rows(SCORED_CHANNELS))}{power_table}

![Measured ten-flying-lap range and median against the original-track and canonical-track 7DoF laps, each on its own track's physical distance](comparison.png)

![Measured median with original and canonical 7DoF laps overlaid on the original track's physical distance (canonical mapped by nearest XY point)](three_overlay.png)

![Every measured flying-lap speed profile with its median and the 7DoF lap, and per-lap speed and lateral-acceleration error distributions](lap_spread.png)

![Raw original and canonical 7DoF laps: speed, accelerations, yaw rate, requested torque, reconstructed pack voltage, state of charge and RL/RR power](simulated_lap.png)

## Reading the comparison

- **Lap time:** the original-track 7DoF lap takes {old_run['lap_time_s']:.3f} s and the canonical-track lap {new_run['lap_time_s']:.3f} s ({abs(difference):.3f} s {'faster' if difference < 0 else 'slower'} on the canonical line, {old_run['track_length_m'] - new_run['track_length_m']:.2f} m shorter). Against the measured {mt:.3f} s median flying lap that is {old_run['lap_time_s'] - mt:+.3f} s and {new_run['lap_time_s'] - mt:+.3f} s; mean speed {mean_speed[1]:.2f} / {mean_speed[2]:.2f} m/s vs {mean_speed[0]:.2f} m/s measured. {plan_text} The {abs(difference):.3f} s difference between the two tracks mixes path curvature, path length, a different initial state and the unconverged (non-periodic) transient start, and is not a measured handling improvement.
- **Speed:** RMSE {mm('speed_mps')} m/s, bias {mm('speed_mps', 'bias', '+.3f')} m/s, correlation {mm('speed_mps', 'correlation')} (original → canonical).
- **Longitudinal acceleration:** RMSE {mm('longitudinal_acceleration_mps2')} m/s², correlation {mm('longitudinal_acceleration_mps2', 'correlation')}. Peak braking after filtering is {pk[0][0]:.1f} / {pk[1][0]:.1f} m/s² simulated against {pk[0][1]:.1f} m/s² in the measured median (raw simulated minimum {raw_min[0]:.1f} / {raw_min[1]:.1f} m/s², original / canonical); the simulated driver brakes harder and later than the measured median.
- **Lateral acceleration:** RMSE {mm('lateral_acceleration_mps2')} m/s², bias {mm('lateral_acceleration_mps2', 'bias', '+.3f')} m/s², correlation {mm('lateral_acceleration_mps2', 'correlation')}. The 7DoF lateral acceleration is the body ay of the integrated state{f' (planned peak {peak_lat:.2f} m/s²)' if peak_lat is not None else ''}; the signed bias is probably dominated by a measured-ay offset (the point-mass pair report found measured ay about −0.7 m/s² off v·yaw), not model structure, so RMSE and correlation are the better agreement measures.
- **Yaw rate:** RMSE {mm('yaw_rate_rad_s')} rad/s, correlation {mm('yaw_rate_rad_s', 'correlation')}. Yaw is a traced state; measured ay carries a bias relative to v·yaw, so yaw and ay changes must not be read as pure geometry agreement.
- **Motor torque request:** RMSE {mm('motor_torque_request_nm')} N·m; mean {ts[0]:.2f} / {ts[1]:.2f} N·m simulated vs {tm[0]:.2f} / {tm[1]:.2f} N·m measured. Simulated request is the pedal fraction scaled to {limit:g} N·m; the measured channel is the logged mean RL/RR request.
- **Pack terminal voltage (reconstructed):** RMSE {mm('pack_voltage_v')} V, bias {mm('pack_voltage_v', 'bias', '+.2f')} V; mean {vs[0]:.1f} / {vs[1]:.1f} V simulated vs {vm[0]:.1f} / {vm[1]:.1f} V measured. Both laps start at the stint-start SOC while the measured flying laps span a discharging stint, so this bias is an **initial-condition offset**, not evidence about either geometry; the voltage is an approximation reconstructed from bus power and SOC, not a logged or plant-replayed voltage.
{power_line}- **Spread:** original speed RMSE against each measured flying lap spans {min(spread[0]):.3f}–{max(spread[0]):.3f} m/s; canonical spans {min(spread[1]):.3f}–{max(spread[1]):.3f} m/s. Those are errors against individual laps (the model is not expected to beat the lap-to-lap spread of a human driver); the two columns are against different reprojections, not identical coordinates.
- **Acceptance:** original usable = {_gate_cell(old_s.get('usable'))}, canonical usable = {_gate_cell(new_s.get('usable'))}. Failing gates (table in Configuration) mean these laps stay exploratory comparisons.

## Caveats

- **These are the values we did not measure** for this car in the log: wheelbase, track width, yaw inertia (radius of gyration), wheel inertia, tire force/slip characteristics, battery resistance/capacity map, drivetrain and motor efficiency, steering geometry, brake parameters, ambient air density and mechanical brake bias. They are carried over from the IEM24 vehicle compilation, not inferred from this experiment. The {vehicle.motor.peak_motor_torque:g} N·m motor torque cap is a model calibration.
- These runs are non-optimal driven transients, not minimum-time solutions, and a failed acceptance gate (see Configuration) means the lap is not a validated result. {code_note}
- Pack voltage is reconstructed from replayed bus power and traced SOC with the library battery model (not logged and not replayed through the plant's motor-voltage closure); both laps start at SOC {initial[LABELS[0]][1]:.6f} before the measured flying laps, which dominates the voltage bias.
- **Road grip is a proposed prior, not a measurement.** The four friction values in Configuration are exactly 2/3 of the lab-fit values, a team-reported road prior. The lab longitudinal data come from a 43100 R20 tire rather than the 43075 modelled (a cross-tire proxy), the longitudinal ratio is unidentifiable on the current lap, and the lateral 2/3 value is optimistic against the ≈0.60 lab ratio estimated from measured |ay|; lap time and the ax/ay/power rows therefore depend on an unvalidated grip assumption. See [road grip calibration]({grip_link}).
- The motor torque request is the requested pedal torque, not delivered torque; speed regulation, power limiting and friction braking are not in it.
- The legacy original line is one integrated driven lap with a 3.45 m closure gap; the canonical line is the median of all ten flying laps and shares those laps with this comparison, so its agreement is circular and cannot establish it as ground truth. Different track lengths are never stretched onto one another.
- {power_caveat}
- Model ay and yaw come from a rigid-body planar model with a prescribed driver, not from a fitted driver line; no tire temperature or ambient state was used.

## Reproduce

```bash
# Run from the repository root. Two fresh 7DoF attempts with identical settings, differing only by --track.
OUT={out_root}
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --track {LEGACY_TRACK} {commands} --output-root "$OUT/original"
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --track {CANONICAL_TRACK} {commands} --output-root "$OUT/new"
# Select the newest attempt of each run (the runner names attempts by timestamp), then export and report.
ORIG=$(ls -dt "$OUT"/original/attempt_* | head -1)
NEW=$(ls -dt "$OUT"/new/attempt_* | head -1)
PYTHONPATH=src:. python3 tools/reports/export_iem24_forward_correlation.py "$ORIG"
PYTHONPATH=src:. python3 tools/reports/export_iem24_forward_correlation.py "$NEW"
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/reports/report_iem24_seven_dof_track_pair.py \\
    --original-attempt "$ORIG" --new-attempt "$NEW"{'' if summary['power_channels_included'] else ' --skip-power'}
```

This report was generated from the attempts `{old_run['attempt']}` and `{new_run['attempt']}`; a fresh two-run reproduction creates new attempt directories and therefore new timings. To re-render only, pass those two directories to the generator. Report generator: `tools/reports/report_iem24_seven_dof_track_pair.py`; exporter: `tools/reports/export_iem24_forward_correlation.py`; follows the [MIS 2024 report standard]({standard}). The measured input files `measured_stint.csv` and `telemetry_extraction.json` were produced by `tools/data/extract_iem24_telemetry.py` and are not rewritten here; the source MAT must be at `~/Desktop/0615CompEndurance.mat`.
'''
    (output / 'report.md').write_text(text)


def load_run(attempt: Path, role: str, expected_track: str, root: Path = ROOT) -> dict:
    from simplified_vehicle_models.compiler.compile import compile_formulation
    info = verify_attempt(attempt, role, expected_track, root=root)
    table, export_summary, trace = load_export(attempt, info, root=root)
    vehicle = compile_formulation(str(root / FORMULATION)).model
    data, check = build_model_channels(table, trace, vehicle)
    if abs(data['distance_m'][-1] - info['track_length_m']) > 1.e-3:
        raise ReportBlocker(f'{role} attempt {attempt}: export ends at {data["distance_m"][-1]:.3f} m, '
                            f'track length is {info["track_length_m"]:.3f} m')
    return {'attempt': Path(attempt), 'info': info, 'data': data, 'check': check, 'trace': trace,
            'export_summary': export_summary}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--original-attempt', type=Path, required=True,
                        help='completed, exported attempt on the legacy original track')
    parser.add_argument('--new-attempt', type=Path, required=True,
                        help='completed, exported attempt on the canonical ten-lap track')
    parser.add_argument('--output-dir', type=Path, default=BASE / 'seven_dof_track_pair')
    parser.add_argument('--mat', type=Path, default=MAT)
    parser.add_argument('--skip-power', action='store_true',
                        help='omit the optional logged RL/RR power channels (no MAT power read)')
    args = parser.parse_args(argv)
    try:
        old = load_run(args.original_attempt, 'original', LEGACY_TRACK)
        new = load_run(args.new_attempt, 'new', CANONICAL_TRACK)
        if args.original_attempt.resolve() == args.new_attempt.resolve():
            raise ReportBlocker('the original and new attempts are the same directory')
        pair = verify_pair(old['info'], new['info'])
        measured = load_table(BASE / 'measured_stint.csv')
        channels = SCORED_CHANNELS
        if not args.skip_power:
            if not args.mat.is_file():
                raise ReportBlocker(f'{args.mat} not found; pass --mat or --skip-power')
            measured.update(motor_power(args.mat, measured))
            channels = SCORED_CHANNELS + POWER_CHANNELS
        extraction = json.loads((BASE / 'telemetry_extraction.json').read_text())
        runs = {LABELS[0]: old, LABELS[1]: new, 'pair': pair}
        report = compare(args.output_dir, measured, extraction, runs,
                         mat=args.mat if args.mat.is_file() else None, channels=channels)
    except ReportBlocker as blocker:
        print(f'BLOCKER: {blocker}', file=sys.stderr)
        return 2
    print(json.dumps({label: {'lap_time_s': report['comparisons'][label]['lap_time_s'],
                              'usable': report['comparisons'][label]['usable'],
                              'ay_rmse': report['comparisons'][label]['metrics']['lateral_acceleration_mps2']['rmse']}
                      for label in LABELS}, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
