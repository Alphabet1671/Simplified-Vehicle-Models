#!/usr/bin/env python3
"""Run real MAT acceptance checks for closed-track telemetry imports.

Private fixture files are never discovered implicitly. Supply both paths with
--mis and --fsc; output is a JSON report and one overlap figure per source.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
import re

import numpy as np

from tools.track_gui.closed import ClosedSettings, build_closed, save_closed, summarize
from tools.track_gui.mat_channels import ChannelSelection
from tools.track_gui.track_ops import (
    from_closed_geometry, load_track_data, reverse_track, save_sections,
    save_track_data, split_closed,
)

MIS_DEFAULT = Path('/Users/alphabet1671/Desktop/0615CompEndurance.mat')
FSC_DEFAULT = Path('/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat')


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _plot(path: Path, overlaps: np.ndarray, geometry: dict, label: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8), constrained_layout=True)
    for line in overlaps:
        ax.plot(line[:, 1], line[:, 0], color='#90a4ae', alpha=.45, linewidth=.7)
    ax.plot(geometry['east'], geometry['north'], color='#c62828', linewidth=1.5, label='mean line')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
    ax.set_title(f'{label}: aligned laps and mean line'); ax.legend()
    fig.savefig(path, dpi=160); plt.close(fig)


def validate_fixture(label: str, source: Path, output: Path, *, gps: bool) -> dict:
    if not source.is_file():
        raise FileNotFoundError(f'{label} source does not exist: {source}')
    source = source.expanduser().resolve()
    cfg = ({'start_time':720.9,'end_time':1103.6,'spacing':.1,'smoothing':.5,
            'maximum_radius':100.,'return_radius':10.,'minimum_travel':150.,
            'minimum_duration':18.,'maximum_duration':35.} if gps else
           {'start_time':550.,'end_time':1050.,'spacing':.5,'smoothing':1.,
            'maximum_radius':300.,'return_radius':25.,'minimum_travel':150.,
            'minimum_duration':15.,'maximum_duration':120.})
    selection = ChannelSelection(x_channel='Latitude_MTI', y_channel='Longitude_MTI', mode='gps') if gps else ChannelSelection(x_channel='VNorth', y_channel='VEast', mode='velocity')
    settings = ClosedSettings(telemetry=source, channel_selection=selection, multi_lap=True, **cfg)
    geometry = build_closed(settings)
    stats = summarize(geometry)
    diag = geometry['lap_diagnostics']
    arrays = [np.asarray(geometry[key], dtype=float) for key in ('distance','north','east','heading','curvature')]
    if not all(np.all(np.isfinite(a)) for a in arrays):
        raise AssertionError(f'{label}: geometry contains nonfinite values')
    if int(diag['accepted_laps']) < 2 or abs(abs(stats['winding_rad']) - 2*math.pi) > .08:
        raise AssertionError(f'{label}: insufficient loops or invalid winding: {stats}')
    if not (200 <= stats['length_m'] <= 1500):
        raise AssertionError(f'{label}: implausible circuit length {stats["length_m"]}')
    overlaps = np.stack((geometry['overlap_north'], geometry['overlap_east']), axis=-1)
    if overlaps.ndim != 3 or not np.all(np.isfinite(overlaps)):
        raise AssertionError(f'{label}: invalid overlap diagnostic arrays')
    output.mkdir(parents=True, exist_ok=True)
    figure = output / f'{label.lower()}-overlap.png'
    _plot(figure, overlaps, geometry, label)
    track = from_closed_geometry(geometry, title=f'{label} acceptance', maximum_radius=settings.maximum_radius)
    directory = Path(tempfile.mkdtemp(prefix=f'track-gui-{label.lower()}-'))
    saved = save_closed(geometry, settings, directory, 'accepted', f'{label.lower()}_acceptance', f'{label} acceptance')
    loaded = load_track_data(saved.xml)
    if not loaded.is_closed or abs(loaded.length - track.length) > settings.spacing * 1.1:
        raise AssertionError(f'{label}: closed save/load length or closure mismatch')
    rev = reverse_track(loaded)
    if not np.allclose(rev.curvature, -loaded.curvature[::-1], atol=1e-9):
        raise AssertionError(f'{label}: reversal did not negate signed curvature on the reversed node order')
    if not np.allclose([rev.x_north[0], rev.y_east[0]], [loaded.x_north[0], loaded.y_east[0]], atol=1e-9):
        raise AssertionError(f'{label}: closed reversal moved the start point')
    rev_saved = save_track_data(rev, directory, 'reversed', f'{label.lower()}_reversed', f'{label} reversed')
    rev_loaded = load_track_data(rev_saved.xml)
    twice = reverse_track(rev_loaded)
    if abs(twice.length - loaded.length) > settings.spacing * 1.1 or not twice.is_closed:
        raise AssertionError(f'{label}: double reverse changed closed geometry')
    if not np.allclose(twice.x_north, loaded.x_north, atol=settings.spacing*1.1) or not np.allclose(twice.y_east, loaded.y_east, atol=settings.spacing*1.1):
        raise AssertionError(f'{label}: double reverse XY mismatch')
    cuts = [0., track.length/3., 2*track.length/3.]
    sections = split_closed(track, cuts)
    if len(sections) != 3 or any(section.is_closed for section in sections):
        raise AssertionError(f'{label}: expected three open split sections')
    section_saved = save_sections(track, cuts, directory, 'section', f'{label.lower()}_section', f'{label} section')
    if len(section_saved) != 3:
        raise AssertionError(f'{label}: saved section count mismatch')
    section_lengths = [load_track_data(item.xml).length for item in section_saved]
    length_error = abs(sum(section_lengths)-track.length)
    if length_error > max(.1, .001*track.length):
        raise AssertionError(f'{label}: split lengths do not reconstruct source (error {length_error:.6g} m)')
    section_departures = []
    for item in section_saved:
        xml_text = item.xml.read_text(encoding='utf-8')
        match = re.search(r'measured maximum departure from source polyline: ([0-9.eE+-]+) m; accepted limit ([0-9.eE+-]+) m', xml_text)
        if not match:
            raise AssertionError(f'{label}: section provenance lacks measured positional correction')
        departure, bound = map(float, match.groups())
        if departure > bound or departure > .02000001:
            raise AssertionError(f'{label}: section positional correction {departure:g} m exceeds its recorded bound {bound:g} m')
        section_departures.append({'maximum_correction_m':departure,'bound_m':bound})
    # Validate an individual open section through save, reverse, load, and double reversal.
    open_track = sections[0]
    open_saved = save_track_data(open_track, directory, 'open_section', f'{label.lower()}_open', f'{label} open section')
    open_loaded = load_track_data(open_saved.xml)
    open_reverse_saved = save_track_data(reverse_track(open_loaded), directory, 'open_reversed', f'{label.lower()}_open_reversed', f'{label} reversed open section')
    open_reversed = load_track_data(open_reverse_saved.xml)
    open_twice = reverse_track(open_reversed)
    if open_loaded.is_closed or open_reversed.is_closed or abs(open_twice.length-open_loaded.length) > .01 or not np.allclose(open_twice.x_north,open_loaded.x_north,atol=.02) or not np.allclose(open_twice.y_east,open_loaded.y_east,atol=.02):
        raise AssertionError(f'{label}: open-section save/reverse/load round trip failed')
    support = np.asarray(diag.get('overlap_support', []), dtype=int)
    compact_diagnostics = {
        'candidate_count': diag.get('candidate_count'),
        'rejected_candidate_count': diag.get('rejected_candidate_count'),
        'detection_rejections': diag.get('detection_rejections', []),
        'accepted_laps': diag.get('accepted_laps'),
        'rejected_laps': diag.get('rejected_laps', []),
        'lap_lengths_m': diag.get('lap_lengths_m', []),
        'lap_boundaries': diag.get('lap_boundaries', []),
        'clipped_samples': diag.get('clipped_samples'),
        'total_aligned_samples': diag.get('total_aligned_samples'),
        'clipped_fraction': float(diag.get('clipped_samples', 0)) / max(1, int(diag.get('total_aligned_samples', 0))),
        'overlap_support': {'minimum': int(support.min()) if support.size else None, 'median': float(np.median(support)) if support.size else None, 'maximum': int(support.max()) if support.size else None, 'node_count': int(support.size)},
        'residuals_m': diag.get('residuals_m'),
        'registration': diag.get('registration'),
    }
    return {
        'fixture':label, 'source':str(source), 'source_sha256':sha256(source),
        'channels':{'north':selection.x_channel,'east':selection.y_channel,'mode':selection.mode},
        'settings':{**cfg,'multi_lap':True,'deviation_limit':settings.deviation_limit,
                    'channel_selection':{'x_channel':selection.x_channel,'y_channel':selection.y_channel,'mode':selection.mode,'x_scale':selection.x_scale,'y_scale':selection.y_scale,'x_allow_unknown_units':selection.x_allow_unknown_units,'y_allow_unknown_units':selection.y_allow_unknown_units}},
        'lap_diagnostics':compact_diagnostics, 'geometry':stats,
        'point_clipping':{'samples':diag.get('clipped_samples'),'total_aligned_samples':diag.get('total_aligned_samples'),'fraction':(float(diag.get('clipped_samples',0))/max(1,int(diag.get('total_aligned_samples',0))))},
        'roundtrips':{'closed_save_load':True,'reverse_save_load':True,'double_reverse_xy':True,'reverse_preserves_start_and_flips_curvature':True,'sections':len(section_saved),'section_lengths_m':section_lengths,'split_length_error_m':sum(section_lengths)-track.length,'section_departures':section_departures,'open_section_save_reverse_load':True},
        'figure':str(figure), 'figure_visually_inspected':False,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mis', type=Path, default=None, help='MIS MAT path (required unless --skip-mis)')
    parser.add_argument('--fsc', type=Path, default=None, help='FSC MAT path (required unless --skip-fsc)')
    parser.add_argument('--output', type=Path, required=True, help='directory for JSON and overlap figures')
    parser.add_argument('--skip-mis', action='store_true'); parser.add_argument('--skip-fsc', action='store_true')
    args = parser.parse_args(argv)
    if not args.skip_mis and args.mis is None: parser.error('--mis is required unless --skip-mis is set')
    if not args.skip_fsc and args.fsc is None: parser.error('--fsc is required unless --skip-fsc is set')
    args.output.mkdir(parents=True, exist_ok=True)
    report={'schema_version':1,'command_options':{'mis':str(args.mis) if args.mis else None,'fsc':str(args.fsc) if args.fsc else None,'output':str(args.output.resolve())},'results':[]}
    if not args.skip_mis: report['results'].append(validate_fixture('MIS',args.mis,args.output,gps=False))
    if not args.skip_fsc: report['results'].append(validate_fixture('FSC',args.fsc,args.output,gps=True))
    target=args.output/'acceptance.json'; target.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(target)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
