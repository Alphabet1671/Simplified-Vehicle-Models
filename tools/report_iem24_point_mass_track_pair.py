#!/usr/bin/env python3
"""Standard MIS 2024 correlation report for paired point-mass track experiments."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial import cKDTree

from tools.compare_iem24_murphy_stint import (COMPARISON_CHANNELS, _metrics,
    fit_position_offset, fold_laps, flying_lap_indices, load_table, project_onto_track)
from tools.iem24_telemetry import load_nav_channels, zero_phase_lowpass
from simplified_vehicle_models.tracks.profile import load_track

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/validation/iem24-murphy'
MAT = Path.home() / 'Desktop/0615CompEndurance.mat'
LABELS = ('Original track', 'Experimental track')
COLORS = ('#c00000', '#ed7d31')
CHANNELS = COMPARISON_CHANNELS + (
    ('rl_power_kw', 'RL motor electrical power', 'kW'),
    ('rr_power_kw', 'RR motor electrical power', 'kW'),
)
# Scored metrics follow the six-channel repository report standard; requested
# RL/RR power is plotted as supplementary panels with the measurement caveat.
SCORED_CHANNELS = COMPARISON_CHANNELS


def filter_model(data: dict[str, np.ndarray], columns: tuple[str, ...]) -> dict[str, np.ndarray]:
    """Apply measured 1 Hz first-order zero-phase low-pass in physical time.

    The original 0.5 m solver mesh has nonuniform dt. Filtering after linear
    interpolation to a uniform 20 Hz clock avoids treating distance as time.
    """
    time = data['elapsed_time_s']
    tick = np.arange(0., time[-1], .05)
    tick = np.r_[tick, time[-1]]
    sos = butter(1, 1., fs=20., btype='lowpass', output='sos')
    filtered = dict(data)
    for column in columns:
        if column not in data or column in ('distance_m', 'elapsed_time_s'):
            continue
        values = np.interp(tick, time, data[column])
        filtered[column] = np.interp(time, tick, sosfiltfilt(sos, values))
    return filtered


def motor_power(mat: Path, measured: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    nav = load_nav_channels(mat, ('pwrRLReal', 'pwrRRReal'))
    index = np.searchsorted(nav.time, measured['time_s'])
    if np.any(index == len(nav.time)) or not np.allclose(nav.time[index], measured['time_s'], atol=1e-4, rtol=0):
        raise ValueError('power channels do not share measured_stint timestamps')
    return {name: zero_phase_lowpass(nav[source], nav.sample_interval, 1.)[index] / 1000.
            for name, source in (('rl_power_kw','pwrRLReal'),('rr_power_kw','pwrRRReal'))}


def compare(output: Path, measured: dict, extraction: dict, runs: dict, *, mat: Path) -> dict:
    """Project measured laps separately on each geometry; score in each frame."""
    laps = flying_lap_indices(measured)
    columns = tuple(entry[0] for entry in CHANNELS)
    comparisons = {}
    for label, payload in runs.items():
        info, data = payload
        track = load_track(ROOT / info['track'])
        path = track.path_coordinates
        offset, rms = fit_position_offset(measured, path)
        projected, residual = project_onto_track(measured, path, offset)
        grid, folded = fold_laps(measured, laps, columns,
            track_distance=projected, lap_length=path.lap_length, bins=1017)
        median = {column: np.median(folded[column], axis=0) for column in columns}
        filtered = filter_model(data, columns)
        model = {column: np.interp(grid, data['distance_m'], filtered[column]) for column in columns}
        metrics = {column: _metrics(model[column], median[column]) for column in columns}
        per_lap_speed = [float(np.sqrt(np.mean((model['speed_mps'] - row) ** 2)))
                         for row in folded['speed_mps']]
        comparisons[label] = dict(grid=grid, folded=folded, median=median, model=model,
            metrics=metrics, per_lap_speed_rmse=per_lap_speed, offset=offset, fit_rmse=rms,
            position_p95=float(np.percentile(residual[np.isin(measured['lap_index'], laps)], 95)))
    summary = {'measured_laps': [int(lap) for lap in laps], 'median_measured_lap_s': float(np.median(
                   [v['lap_time_s'] for v in extraction['laps'] if not v['standing_start']])),
               'median_measured_distance_m': float(np.median(
                   [v['distance_m'] for v in extraction['laps'] if not v['standing_start']])),
               'source_sha256': hashlib.sha256(mat.read_bytes()).hexdigest(),
               'source_bytes': mat.stat().st_size,
               'comparisons': {label: {'run': runs[label][0], 'metrics': item['metrics'],
                                  'per_lap_speed_rmse': item['per_lap_speed_rmse'],
                                  'position_offset_north_east_m': item['offset'].tolist(),
                                  'position_fit_rmse_m': item['fit_rmse'],
                                  'position_residual_p95_m': item['position_p95']}
                               for label, item in comparisons.items()}}
    output.mkdir(parents=True, exist_ok=True)
    (output / 'comparison_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    for label, item in comparisons.items():
        name = 'original' if label == LABELS[0] else 'experimental'
        names = ['distance_m']
        vals = [item['grid']]
        for column in columns:
            names.extend(('measured_' + column, 'model_' + column))
            vals.extend((item['median'][column], item['model'][column]))
        np.savetxt(output / (name + '_comparison.csv'), np.column_stack(vals),
                   header=','.join(names), comments='', delimiter=',')
    render(output, comparisons, runs, laps)
    write_report(output, summary, comparisons, extraction)
    return summary


def render(output: Path, pairs: dict, runs: dict, laps: np.ndarray) -> None:
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-config')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(CHANNELS), 2, figsize=(17, 24), sharex='col')
    for col, label in enumerate(LABELS):
        entry = pairs[label]
        grid = entry['grid']
        for row, (channel, title, unit) in enumerate(CHANNELS):
            ax = axes[row, col]
            values = entry['folded'][channel]
            ax.fill_between(grid, np.min(values,axis=0), np.max(values,axis=0), color='#9dc3e6',alpha=.35,
                label='Measured range (10 laps)')
            ax.plot(grid,entry['median'][channel],color='#1f4e79',lw=1.4,label='Measured median')
            ax.plot(grid,entry['model'][channel],color=COLORS[col],lw=1.35,label=label+' point mass')
            ax.set_ylabel(f'{title} ({unit})');ax.grid(alpha=.25);ax.margins(x=0)
            if row==0:ax.legend(loc='upper right',fontsize=8)
        axes[0,col].set_title(f'{label}: {runs[label][0]["track_length_m"]:.2f} m (own physical distance)')
        axes[-1,col].set_xlabel('Projected distance on named driven-line track (m)')
    fig.suptitle('IEM24 Murphy MIS endurance: measured vs point mass on original and experimental tracks')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=145);plt.close(fig)

    # Additional overlay on one *physical old-track axis*, using XY nearest
    # projection of every candidate model node instead of length-normalized phase.
    old_path = load_track(ROOT / runs[LABELS[0]][0]['track']).path_coordinates
    new_path = load_track(ROOT / runs[LABELS[1]][0]['track']).path_coordinates
    match = cKDTree(np.column_stack((old_path.x_north, old_path.y_east))).query(
        np.column_stack((new_path.x_north, new_path.y_east)))[1]
    mapped_nodes = old_path.distance[match].copy()
    candidate_data = runs[LABELS[1]][1]
    mapped = np.interp(candidate_data['distance_m'],new_path.distance,mapped_nodes)
    mapped[0] = 0.
    if np.any(np.diff(mapped) < -.5):
        raise ValueError('experimental track maps backward along original geometry')
    candidate_data = runs[LABELS[1]][1]
    candidate_filtered = filter_model(candidate_data, tuple(key for key,_,_ in CHANNELS))
    grid = pairs[LABELS[0]]['grid']
    fig, axes = plt.subplots(len(CHANNELS), 1, figsize=(14,24), sharex=True)
    for ax,(key,title,unit) in zip(axes,CHANNELS):
        orig=pairs[LABELS[0]]
        band=orig['folded'][key]
        ax.fill_between(grid,np.min(band,axis=0),np.max(band,axis=0),
            color='#9dc3e6',alpha=.35,label='Measured range (10 laps)')
        ax.plot(grid,orig['median'][key],color='#1f4e79',lw=1.6,label='Measured median')
        ax.plot(grid,orig['model'][key],color=COLORS[0],lw=1.35,label='Original-track point mass')
        ax.plot(mapped,candidate_filtered[key],color=COLORS[1],lw=1.35,
                label='Experimental-track point mass (XY projected)')
        ax.set_ylabel(f'{title} ({unit})');ax.grid(alpha=.25);ax.margins(x=0)
        ax.legend(loc='upper right',fontsize=7,ncol=2)
    axes[0].legend(loc='upper right',fontsize=7,ncol=2)
    axes[-1].set_xlabel('Original driven-line distance (m), candidate mapped by closest XY point')
    fig.suptitle('IEM24 Murphy MIS endurance — median measured and two point-mass track solves')
    fig.tight_layout();fig.savefig(output/'three_overlay.png',dpi=145);plt.close(fig)

    fig,axes=plt.subplots(2,2,figsize=(16,8))
    for col,label in enumerate(LABELS):
        entry=pairs[label];grid=entry['grid']
        for trace in entry['folded']['speed_mps']:
            axes[0,col].plot(grid,trace,color='#9dc3e6',lw=.7)
        axes[0,col].plot(grid,entry['median']['speed_mps'],color='#1f4e79',lw=1.6,label='Measured median')
        axes[0,col].plot(grid,entry['model']['speed_mps'],color=COLORS[col],lw=1.4,label=label)
        axes[0,col].set_title(label);axes[0,col].set_ylabel('Speed (m/s)');axes[0,col].legend()
        axes[1,col].hist(entry['per_lap_speed_rmse'],bins=np.linspace(0,4,17),color=COLORS[col],alpha=.8)
        axes[1,col].set_ylabel('Lap count');axes[1,col].set_xlabel('Speed RMSE vs each measured lap (m/s)')
        axes[1,col].grid(alpha=.25)
    fig.tight_layout();fig.savefig(output/'lap_spread.png',dpi=145);plt.close(fig)

    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    for label,color in zip(LABELS,COLORS):
        info,data=runs[label];x=data['distance_m']
        for ax,key,title in zip(axes,('speed_mps','lateral_acceleration_mps2',
                                     'motor_torque_request_nm','pack_voltage_v'),
                                ('Raw speed (m/s)','Raw ay (m/s²)',
                                 'Raw motor torque request (N·m)','Raw pack voltage (V)')):
            ax.plot(x,data[key],color=color,label=label,linewidth=1.2)
            ax.set_ylabel(title);ax.grid(alpha=.25)
    axes[0].legend();axes[-1].set_xlabel('Each track’s own distance (m)')
    fig.tight_layout();fig.savefig(output/'simulated_lap.png',dpi=145);plt.close(fig)


def write_report(output: Path, summary: dict, comparisons: dict, extraction: dict) -> None:
    import os
    standard=os.path.relpath(ROOT/'docs/standards/mis-2024-report-standard.md', output)
    rows=[]
    for lap in extraction['laps']:
        rows.append(f"| {lap['index']} {'(standing start)' if lap['standing_start'] else '(flying)'} | {lap['start_s']:.2f} | {lap['lap_time_s']:.2f} | {lap['distance_m']:.1f} | {lap['mean_speed_mps']:.2f} | {lap['closure_radius_m']:.2f} |")
    metric_rows=[]
    for channel,title,unit in SCORED_CHANNELS:
        for label in LABELS:
            metric=comparisons[label]['metrics'][channel]
            metric_rows.append(f"| {title} ({unit}) | {label} | {metric['rmse']:.3f} | {metric['mae']:.3f} | {metric['bias']:+.3f} | {metric['correlation']:.3f} | {metric['normalised_rmse']:.3f} |")
    old=summary['comparisons'][LABELS[0]]['run']; new=summary['comparisons'][LABELS[1]]['run']
    difference=new['lap_time_s']-old['lap_time_s']
    speed_old=comparisons[LABELS[0]]['metrics']['speed_mps'];speed_new=comparisons[LABELS[1]]['metrics']['speed_mps']
    ay_old=comparisons[LABELS[0]]['metrics']['lateral_acceleration_mps2'];ay_new=comparisons[LABELS[1]]['metrics']['lateral_acceleration_mps2']
    power_old=comparisons[LABELS[0]]['metrics']['rl_power_kw'];power_new=comparisons[LABELS[1]]['metrics']['rl_power_kw']
    md=summary['median_measured_distance_m']; mt=summary['median_measured_lap_s']
    excerpt=f'''# IEM24 Murphy simulated MIS 2024 laps vs measured endurance stint — original and experimental tracks

## Configuration

- Both runs: identical 287 kg car-and-driver mass (209 kg car + 78 kg driver); LLTD 42%, CGx 45% of wheelbase from the front axle (55% mass at front), CG height 0.300 m, 65% front mechanical brake bias.
- Aero: air density 1.22 kg/m³, CL·A 3.5 m², CD·A 1.55 m², front aero balance 50%.
- Drivetrain: two rear motors, 10.5 N·m per-motor limit, 10.9:1 motor gear ratio and 95% transmission efficiency; regeneration disabled. Reduced point-mass UniTire pure-slip peak envelope with the same compiled IEM24 planar vehicle parameters in both solves.
- Initial conditions: flying closed-lap speed solved periodically; battery SOC 0.829085 (resting pack 575.70 V measured before launch), terminal voltage at first reported segment {old['initial_terminal_voltage_v']:.2f} V in both runs; no measured tire temperature or ambient state initialized.
- Original path: `{old['track']}`, length {old['track_length_m']:.3f} m, {old['nodes']} nodes, flying lap {old['lap_time_s']:.3f} s. Experimental median driven line: `{new['track']}`, length {new['track_length_m']:.3f} m, {new['nodes']} nodes, flying lap {new['lap_time_s']:.3f} s. Mesh target 0.5 m in both.
- Solver: energy-coupled minimum-time *point-mass* forward/backward lap, no planar 7DoF. Original convergence: {old['converged']} in {old['iterations']} profile iterations and {old['energy_iterations']} energy iterations; experimental: {new['converged']} in {new['iterations']} and {new['energy_iterations']} respectively. Converged prescribed-path minimum-time model, **not** a measured validated lap or a joint optimized racing line.

## Measured data

Source `~/Desktop/0615CompEndurance.mat`, SHA-256 `{summary['source_sha256']}`, {summary['source_bytes']} bytes; shared log clock approximately 100 Hz, position/navigation fixes less frequent. Conditioning uses 105.00–938.00 s for lap identification, driving window 117.02–925.91 s. Navigation `VNorth`, `VEast`, `VelX`, `VelY`, `AccelX`, `AccelY`, `GyroZ`; per-motor requests `Tq_RL_Req`, `Tq_RR_Req`; pack voltage `VPack`; electrical power `pwrRLReal`, `pwrRRReal`. Existing `measured_stint.csv` is unchanged. The 8.55° navigation-unit yaw skew rotates VelX/VelY and AccelX/AccelY into the body frame; corrected acceleration/velocity-rate RMSE is 0.322 instead of 1.362 m/s². Navigation accelerations, speed and torque/voltage were preconditioned with a zero-phase first-order low-pass filter at 1 Hz; measured power channels are separately filtered the same way. The two model traces are independently low-pass filtered with first-order 1 Hz zero-phase filtering in **time** for their scored overlays; raw model exports are preserved. No voltage zeroing, no inferred time shift and no equal-length scaling were applied.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

Median of 10 flying laps: **{mt:.3f} s** over **{md:.2f} m**; measured driven distance exceeds original geometry by {(md/old['track_length_m']-1)*100:+.2f}% and experimental geometry by {(md/new['track_length_m']-1)*100:+.2f}%. Each measured position is projected independently onto each track after fitting a constant north/east translation; then each lap is interpolated on **that track's own physical distance** and the median and full range are computed anew. The two columns in `comparison.png` are **not pointwise aligned to each other**: their lengths differ by {old['track_length_m']-new['track_length_m']:.2f} m. Position residual p95 is {summary['comparisons'][LABELS[0]]['position_residual_p95_m']:.2f} m old, {summary['comparisons'][LABELS[1]]['position_residual_p95_m']:.2f} m experimental. This design avoids assuming a normalized phase is a physical distance.

## Trace comparison

Each model is compared to the median of the **same 10 flying laps**, reprojected onto its named track. Measured range shading uses the minimum and maximum individual lap values, never the standing-start lap. The six required channels below are scored on 1017 track-distance grid points. Bias is signed **sim − meas**, in the row's unit. Supplementary RL/RR power panels follow the standard channels; their full exploratory statistics are in `comparison_summary.json`, but their sensor/bus accounting is insufficient to interpret as motor-power validation. RL/RR model electrical powers are equal halves of the point-mass energy solver's total bus power; measured RL/RR are separately logged motor electrical-power channels, whose measurement location may differ from bus power.

| Signal | Track | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |
|---|---|---:|---:|---:|---:|---:|
{chr(10).join(metric_rows)}

![Measured ten-lap range and median, original-track point mass, and experimental-track point mass on each track's physical distance](comparison.png)

The requested single-axis overlay maps the experimental track's model nodes to the nearest original-track XY nodes; it retains both independent solved traces rather than stretching either lap to the other's length. Its distances are approximate XY correspondence, not an equal-distance comparison metric.

![Measured median with old and experimental point-mass tracks overlaid on the original track's physical distance](three_overlay.png)

![Individual measured flying-lap speed profiles, each projected to its own track, and both model speed-error distributions](lap_spread.png)

![Raw original and experimental point-mass laps with speed, lateral acceleration, motor torque request and pack voltage](simulated_lap.png)

## Reading the comparison

- **{abs(difference):.3f} s**: experimental-track model is {'faster' if difference<0 else 'slower'} than original ({new['lap_time_s']:.3f} vs {old['lap_time_s']:.3f} s). Differences mix altered path curvature and a {old['track_length_m']-new['track_length_m']:.2f} m shorter path; they are not a measured handling improvement. The old run is {old['lap_time_s']-mt:+.3f} s and the new run {new['lap_time_s']-mt:+.3f} s from the measured median flying lap.
- **Speed:** RMSE {speed_old['rmse']:.3f} → {speed_new['rmse']:.3f} m/s, correlation {speed_old['correlation']:.3f} → {speed_new['correlation']:.3f}; both runs use the same powertrain but different curvature/length constraints.
- **Longitudinal acceleration:** RMSE {comparisons[LABELS[0]]['metrics']['longitudinal_acceleration_mps2']['rmse']:.3f} → {comparisons[LABELS[1]]['metrics']['longitudinal_acceleration_mps2']['rmse']:.3f} m/s². Repeated braking impulses can survive common filtering: a quasi-static minimum-time planner need not brake as a human driver does.
- **Lateral acceleration:** RMSE {ay_old['rmse']:.3f} → {ay_new['rmse']:.3f} m/s², correlation {ay_old['correlation']:.3f} → {ay_new['correlation']:.3f}. At about 252–268 m on the original line, two stored curvature maxima create two point-mass ay maxima; the experimental ten-lap smoothed line has one broader apex. Exact horizontal coordinates differ on the shorter line.
- **Yaw rate:** RMSE {comparisons[LABELS[0]]['metrics']['yaw_rate_rad_s']['rmse']:.3f} → {comparisons[LABELS[1]]['metrics']['yaw_rate_rad_s']['rmse']:.3f} rad/s. Measured ay contains an approximately −0.7 m/s² bias relative to v·yaw, so yaw and ay changes must not be interpreted as pure geometry agreement.
- **Motor torque request:** RMSE {comparisons[LABELS[0]]['metrics']['motor_torque_request_nm']['rmse']:.3f} → {comparisons[LABELS[1]]['metrics']['motor_torque_request_nm']['rmse']:.3f} N·m. Positive simulated request is reconstructed from the model's rear-wheel drive force and 95% gear efficiency; friction braking has zero motor request and regen is disabled.
- **Pack terminal voltage:** bias {comparisons[LABELS[0]]['metrics']['pack_voltage_v']['bias']:+.2f} → {comparisons[LABELS[1]]['metrics']['pack_voltage_v']['bias']:+.2f} V. Both model laps start at stint-start SOC while measured flying laps span a discharging stint, so the voltage bias is predominantly an **initial-condition offset**, not evidence that either geometry predicts battery voltage better.
- **RL and RR motor electrical power (exploratory):** RL RMSE {power_old['rmse']:.3f} → {power_new['rmse']:.3f} kW; RR RMSE {comparisons[LABELS[0]]['metrics']['rr_power_kw']['rmse']:.3f} → {comparisons[LABELS[1]]['metrics']['rr_power_kw']['rmse']:.3f} kW. These statistics are *not directly comparable component errors*: point mass allocates equal halves of pack bus power and logged left/right motor-power accounting is unverified. They are shown for trace context only. No independent wheel torques were solved.
- **Spread:** original speed RMSE vs each flying lap spans {min(comparisons[LABELS[0]]['per_lap_speed_rmse']):.3f}–{max(comparisons[LABELS[0]]['per_lap_speed_rmse']):.3f} m/s; experimental spans {min(comparisons[LABELS[1]]['per_lap_speed_rmse']):.3f}–{max(comparisons[LABELS[1]]['per_lap_speed_rmse']):.3f} m/s. These are against each geometry's reprojection, not against identical coordinates.

## Caveats

- **These are the values we did not measure** for this car in the log: wheelbase, track width, tire force envelope, battery resistance/capacity map, drivetrain efficiency, ambient air density, and mechanical brake bias; they are carried over from the IEM24 vehicle compilation, not inferred from this paired-track experiment. The 10.5 N·m motor torque cap is a model calibration; logged requests peak near 10.45 N·m.
- The old 'centreline' is one integrated driven lap, not surveyed centreline; the experimental line is a median of all ten flying laps and shares those same laps with validation. It shrinks 7.5 m after XY smoothing. This is circular calibration and cannot establish the experimental geometry as ground truth. The original source window has a 3.45 m closure gap and tangent mismatch near the seam.
- `pwrRLReal`/`pwrRRReal` sensor definition and bus-power allocation may differ: compare the motor-power shape cautiously, not as a component-efficiency validation. Model power represents equal halves of total bus draw; measured power may include different inverter/motor accounting.
- Terminal voltages differ because SOC starts at 0.829085 in both models, before the actual measured flying stint. Initial model terminal voltage is reported at the first segment (the battery is at 575.70 V resting); no flight-lap SOC is matched. No log tire state or ambient temperature was used.
- Model ay and yaw are prescribed-track kinematics; point mass cannot represent steering, vehicle yaw transients, sideslip or differing driver lines. Segment acceleration and yaw are held at the last segment to plot the closing node; a distance-grid comparison is not proof of dynamic tracking. Figure columns must not be read as sharing identical physical XY locations.

## Reproduce

```bash
# Run from the repository root with Python, SciPy, NumPy and matplotlib installed.
# The source MAT must be available at ~/Desktop/0615CompEndurance.mat.
PYTHONPATH=src:. python3 tools/build_iem24_multilap_track.py
PYTHONPATH=src:/tmp/casadi_pkgs python3 tools/run_iem24_point_mass_track_pair.py
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/report_iem24_point_mass_track_pair.py
```

Report generator: `tools/report_iem24_point_mass_track_pair.py`, paired solver: `tools/run_iem24_point_mass_track_pair.py`; follows the [MIS 2024 report standard]({standard}). The measured input files `measured_stint.csv` and `telemetry_extraction.json` were produced by `tools/extract_iem24_telemetry.py` and are not rewritten here.
'''
    (output/'report.md').write_text(excerpt)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=BASE/'point_mass_track_pair')
    parser.add_argument('--mat',type=Path,default=MAT)
    args=parser.parse_args()
    measured=load_table(BASE/'measured_stint.csv')
    measured.update(motor_power(args.mat,measured))
    extraction=json.loads((BASE/'telemetry_extraction.json').read_text())
    runs={label:(json.loads((args.output_dir/name/'summary.json').read_text()),
                 load_table(args.output_dir/name/'simulated_lap.csv'))
          for label,name in zip(LABELS,('original','experimental'))}
    report=compare(args.output_dir,measured,extraction,runs,mat=args.mat)
    print(json.dumps({label: {'lap_time_s':report['comparisons'][label]['run']['lap_time_s'],
                              'ay_rmse':report['comparisons'][label]['metrics']['lateral_acceleration_mps2']['rmse']}
                      for label in LABELS},indent=2))


if __name__=='__main__':main()
