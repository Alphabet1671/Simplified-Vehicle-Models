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
import re
import tempfile

import numpy as np

from tools.track_gui.closed import ClosedSettings, build_closed, save_closed, summarize
from tools.track_gui.mat_channels import ChannelSelection
from tools.track_gui.track_ops import (
    TrackData, from_closed_geometry, load_track_data, reverse_track,
    save_sections, save_track_data, split_closed,
)

MIS_DEFAULT = Path('/Users/alphabet1671/Desktop/0615CompEndurance.mat')
FSC_DEFAULT = Path('/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat')

DISTANCE_ATOL = 1e-7
XY_ATOL = 1e-7
HEADING_ATOL = 2e-9
CURVATURE_ATOL = 1e-9
ENDPOINT_ATOL = 0.02
CORRECTION_ATOL = 0.02
CUT_SPAN_RELATIVE_ATOL = 1e-4
DOUBLE_TERMINAL_HEADING_ATOL = 5e-4


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _maximum(values) -> float:
    array = np.asarray(values, dtype=float)
    return float(np.max(np.abs(array))) if array.size else 0.0


def _wrapped_maximum(left, right) -> float:
    difference = np.asarray(left, dtype=float) - np.asarray(right, dtype=float)
    return _maximum((difference + math.pi) % (2.0 * math.pi) - math.pi)


def _track_from_prefix(track: TrackData, end: int) -> TrackData:
    return TrackData(
        track.distance[:end], track.x_north[:end], track.y_east[:end],
        track.heading[:end], track.curvature[:end], track.is_closed,
        track.title, track.maximum_radius,
    )


def check_same_track(
    expected: TrackData,
    actual: TrackData,
    *,
    context: str,
    allow_serialized_closed_seam: bool = True,
    allow_trailing_duplicate: bool = False,
    closed_writer_seam_heading: bool = False,
    closed_seam_heading_atol: float | None = None,
) -> dict:
    """Compare every physical field and measure the actual maximum errors.

    A closed CSV writer appends one repeated seam row. That row is a storage
    convention, not additional geometry, so a single extra closed row is
    compared after removing it. The converter's terminal heading is allowed to
    differ by whole turns because heading is an angle, not an absolute winding
    count.
    """
    extra_rows = 0
    if allow_trailing_duplicate and actual.is_closed and len(actual.distance) == len(expected.distance) + 1:
        duplicate = (
            actual.x_north is not None
            and expected.x_north is not None
            and actual.x_north[-1] == actual.x_north[-2]
            and actual.y_east[-1] == actual.y_east[-2]
            and actual.curvature[-1] == actual.curvature[-2]
        )
        if duplicate:
            actual = _track_from_prefix(actual, len(actual.distance) - 1)
            extra_rows = 1

    if allow_serialized_closed_seam and expected.is_closed and len(actual.distance) == len(expected.distance) + 1:
        if actual.x_north is None or actual.y_east is None:
            raise AssertionError(f'{context}: serialized closed export lost XY geometry')
        if not (
            np.isclose(actual.x_north[-1], actual.x_north[0], rtol=0, atol=1e-9)
            and np.isclose(actual.y_east[-1], actual.y_east[0], rtol=0, atol=1e-9)
            and np.isclose(actual.curvature[-1], actual.curvature[0], rtol=0, atol=1e-12)
        ):
            raise AssertionError(f'{context}: extra closed row is not a repeated seam')
        prefix = check_same_track(
            expected, _track_from_prefix(actual, len(expected.distance)),
            context=f'{context} prefix', allow_serialized_closed_seam=False,
        )
        prefix['serialized_seam_rows'] = 1
        prefix['trailing_duplicate_rows'] = extra_rows
        return prefix

    if len(actual.distance) != len(expected.distance):
        raise AssertionError(
            f'{context}: node count differs: expected {len(expected.distance)}, '
            f'got {len(actual.distance)}'
        )
    if actual.x_north is None or actual.y_east is None or expected.x_north is None or expected.y_east is None:
        raise AssertionError(f'{context}: signed XY geometry is missing')
    if closed_writer_seam_heading and expected.is_closed:
        if len(expected.heading) < 3:
            raise AssertionError(f'{context}: closed heading comparison needs at least three rows')
        # The converter appends an approximate terminal heading. Compare every
        # base heading directly and the serialized seam against the writer's
        # explicit final-step extension instead.
        base_heading_error = _wrapped_maximum(actual.heading[:-1], expected.heading[:-1])
        writer_seam_heading = 2.0 * expected.heading[-2] - expected.heading[-3]
        seam_heading_error = _wrapped_maximum(
            actual.heading[-1:], np.asarray([writer_seam_heading]),
        )
        heading_error = max(base_heading_error, seam_heading_error)
        heading_gate_value = heading_error
        seam_limit = HEADING_ATOL
    elif closed_seam_heading_atol is not None and expected.is_closed:
        base_heading_error = _wrapped_maximum(actual.heading[:-1], expected.heading[:-1])
        seam_heading_error = _wrapped_maximum(actual.heading[-1:], expected.heading[-1:])
        heading_error = max(base_heading_error, seam_heading_error)
        if seam_heading_error > closed_seam_heading_atol:
            raise AssertionError(
                f'{context}: double-reverse terminal heading differs by '
                f'{seam_heading_error:g} rad, above {closed_seam_heading_atol:g} rad'
            )
        heading_gate_value = base_heading_error
        seam_limit = closed_seam_heading_atol
    else:
        heading_error = _wrapped_maximum(actual.heading, expected.heading)
        heading_gate_value = heading_error
        seam_limit = HEADING_ATOL
    metrics = {
        'nodes': int(len(expected.distance)),
        'distance_max_error_m': _maximum(actual.distance - expected.distance),
        'xy_max_error_m': max(
            _maximum(actual.x_north - expected.x_north),
            _maximum(actual.y_east - expected.y_east),
        ),
        'heading_wrapped_max_error_rad': heading_error,
        'curvature_max_error_inv_m': _maximum(actual.curvature - expected.curvature),
        'length_error_m': abs(actual.length - expected.length),
        'serialized_seam_rows': 0,
        'trailing_duplicate_rows': extra_rows,
        'heading_seam_source': 'writer_final_step_extension' if closed_writer_seam_heading else 'track_data',
    }
    if 'base_heading_error' in locals():
        metrics['base_heading_wrapped_max_error_rad'] = base_heading_error
        metrics['terminal_heading_wrapped_error_rad'] = seam_heading_error
        metrics['terminal_heading_limit_rad'] = seam_limit
    failures = [
        name for name, limit in (
            ('distance_max_error_m', DISTANCE_ATOL),
            ('xy_max_error_m', XY_ATOL),
            ('curvature_max_error_inv_m', CURVATURE_ATOL),
            ('length_error_m', DISTANCE_ATOL),
        ) if metrics[name] > limit
    ]
    if heading_gate_value > HEADING_ATOL:
        failures.append('heading_wrapped_max_error_rad')
    if failures:
        raise AssertionError(f'{context}: field mismatch {failures}: {metrics}')
    if actual.is_closed != expected.is_closed:
        raise AssertionError(f'{context}: closure status changed')
    return metrics


def _reverse_mapping(track: TrackData) -> TrackData:
    """Compute the physical reverse independently of reverse_track()."""
    distance = track.distance
    if track.is_closed:
        index = np.r_[0, np.arange(len(distance) - 2, 0, -1), 0]
    else:
        index = np.arange(len(distance) - 1, -1, -1)
    reversed_distance = np.r_[0.0, np.cumsum(np.diff(distance)[::-1])]
    heading = np.unwrap(track.heading[index] + math.pi)
    curvature = -track.curvature[index]
    if track.is_closed:
        curvature[-1] = curvature[0]
    return TrackData(
        reversed_distance, track.x_north[index], track.y_east[index],
        heading, curvature, track.is_closed, track.title, track.maximum_radius,
    )


def check_reverse(track: TrackData, reversed_track: TrackData, *, context: str) -> dict:
    """Check distance, XY, heading offset and curvature sign of a reversal."""
    expected = _reverse_mapping(track)
    metrics = check_same_track(
        expected, reversed_track, context=context,
        allow_serialized_closed_seam=False,
    )
    # The first segment must move opposite to the source's last segment.
    source_step = np.array([track.x_north[-1] - track.x_north[-2], track.y_east[-1] - track.y_east[-2]])
    reverse_step = np.array([reversed_track.x_north[1] - reversed_track.x_north[0], reversed_track.y_east[1] - reversed_track.y_east[0]])
    metrics['first_reverse_segment_dot'] = float(np.dot(source_step, reverse_step))
    metrics['start_moved_m'] = float(np.hypot(
        reversed_track.x_north[0] - track.x_north[0],
        reversed_track.y_east[0] - track.y_east[0],
    ))
    if not track.is_closed and metrics['first_reverse_segment_dot'] >= 0:
        raise AssertionError(f'{context}: reversal did not change travel direction')
    if track.is_closed and metrics['start_moved_m'] > XY_ATOL:
        raise AssertionError(f'{context}: closed reversal moved the start point')
    return metrics


def _periodic_points(source: TrackData, samples) -> np.ndarray:
    from scipy.interpolate import CubicSpline

    if source.x_north is None or source.y_east is None:
        raise AssertionError('periodic endpoint check needs signed XY source geometry')
    if not np.allclose([source.x_north[0], source.y_east[0]], [source.x_north[-1], source.y_east[-1]], atol=1e-9):
        raise AssertionError('periodic endpoint check needs a repeated closed seam')
    north = CubicSpline(source.distance, source.x_north, bc_type='periodic')
    east = CubicSpline(source.distance, source.y_east, bc_type='periodic')
    query = np.asarray(samples, dtype=float) % source.length
    return np.column_stack((north(query), east(query)))


def _section_correction(record) -> dict:
    text = Path(record.xml).read_text(encoding='utf-8')
    match = re.search(
        r'measured maximum departure from source polyline: ([0-9.eE+-]+) m; '
        r'accepted limit ([0-9.eE+-]+) m', text,
    )
    if not match:
        raise AssertionError(f'{record.xml}: section provenance lacks measured positional correction')
    departure, bound = (float(value) for value in match.groups())
    if bound > CORRECTION_ATOL + 1e-12 or departure > bound + 1e-12:
        raise AssertionError(
            f'{record.xml}: section positional correction {departure:g} m '
            f'exceeds recorded bound {bound:g} m'
        )
    return {'maximum_correction_m': departure, 'bound_m': bound}


def check_sections(
    source: TrackData,
    cuts,
    expected_sections,
    loaded_sections,
    *,
    saved=None,
    context: str,
) -> dict:
    """Check split outputs against source sections and independent cut points."""
    bounds = sorted(float(cut) for cut in cuts)
    if len(bounds) < 2:
        raise AssertionError(f'{context}: at least two cuts are required')
    if len(expected_sections) != len(bounds) or len(loaded_sections) != len(bounds):
        raise AssertionError(
            f'{context}: section count differs: cuts {len(bounds)}, '
            f'expected {len(expected_sections)}, loaded {len(loaded_sections)}'
        )
    if source.x_north is None or source.y_east is None:
        raise AssertionError(f'{context}: signed source XY is required')
    endpoint_errors = []
    compare_metrics = []
    arc_errors = []
    arc_vs_span = []
    for index, (start, expected, loaded) in enumerate(zip(bounds, expected_sections, loaded_sections)):
        if loaded.is_closed:
            raise AssertionError(f'{context}: section {index + 1} loaded closed')
        compare_metrics.append(check_same_track(
            expected, loaded, context=f'{context} section {index + 1}',
            allow_serialized_closed_seam=False,
        ))
        endpoint = bounds[index + 1] if index + 1 < len(bounds) else bounds[0] + source.length
        expected_points = _periodic_points(source, [start, endpoint])
        endpoint_errors.append(max(
            float(np.hypot(loaded.x_north[0] - expected_points[0, 0], loaded.y_east[0] - expected_points[0, 1])),
            float(np.hypot(loaded.x_north[-1] - expected_points[1, 0], loaded.y_east[-1] - expected_points[1, 1])),
        ))
        arc = float(np.sum(np.hypot(np.diff(loaded.x_north), np.diff(loaded.y_east))))
        arc_errors.append(abs(arc - loaded.length))
        span = endpoint - start
        arc_vs_span.append(abs(loaded.length - span))
    if max(endpoint_errors) > ENDPOINT_ATOL:
        raise AssertionError(f'{context}: cut endpoint error {max(endpoint_errors):g} m exceeds {ENDPOINT_ATOL:g} m')
    if max(arc_errors) > ENDPOINT_ATOL:
        raise AssertionError(f'{context}: stored distance differs from reconstructed arc length by {max(arc_errors):g} m')
    span_limit = max(ENDPOINT_ATOL, CUT_SPAN_RELATIVE_ATOL * source.length)
    if max(arc_vs_span) > span_limit:
        raise AssertionError(
            f'{context}: reconstructed arc length differs from cut parameter span by '
            f'{max(arc_vs_span):g} m, above {span_limit:g} m'
        )
    first = _periodic_points(source, [bounds[0]])[0]
    last_departure = float(np.hypot(
        loaded_sections[-1].x_north[-1] - first[0],
        loaded_sections[-1].y_east[-1] - first[1],
    ))
    if last_departure > ENDPOINT_ATOL:
        raise AssertionError(f'{context}: final section does not wrap to the start ({last_departure:g} m)')
    corrections = [_section_correction(record) for record in saved] if saved is not None else []
    return {
        'sections': len(bounds),
        'node_counts': [metric['nodes'] for metric in compare_metrics],
        'distance_max_error_m': max(metric['distance_max_error_m'] for metric in compare_metrics),
        'xy_max_error_m': max(metric['xy_max_error_m'] for metric in compare_metrics),
        'heading_wrapped_max_error_rad': max(metric['heading_wrapped_max_error_rad'] for metric in compare_metrics),
        'curvature_max_error_inv_m': max(metric['curvature_max_error_inv_m'] for metric in compare_metrics),
        'endpoint_max_error_m': max(endpoint_errors),
        'stored_distance_vs_arc_max_error_m': max(arc_errors),
        'arc_vs_cut_span_max_error_m': max(arc_vs_span),
        'cut_span_limit_m': span_limit,
        'section_lengths_m': [float(section.length) for section in loaded_sections],
        'cut_parameter_spans_m': [
            (bounds[index + 1] if index + 1 < len(bounds) else bounds[0] + source.length) - start
            for index, start in enumerate(bounds)
        ],
        'wrap_to_start_error_m': last_departure,
        'corrections': corrections,
    }


# Short aliases keep the acceptance gates easy to call from focused tests.
_check_same_track = check_same_track
_check_reverse = check_reverse
_check_sections = check_sections


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
    with tempfile.TemporaryDirectory(prefix=f'track-gui-{label.lower()}-') as folder:
        directory = Path(folder)
        saved = save_closed(geometry, settings, directory, 'accepted', f'{label.lower()}_acceptance', f'{label} acceptance')
        loaded = load_track_data(saved.xml)
        closed_metrics = check_same_track(
            track, loaded, context=f'{label} closed save/load',
            closed_writer_seam_heading=True,
        )
        rev = reverse_track(loaded)
        reverse_metrics = check_reverse(loaded, rev, context=f'{label} reversal')
        rev_saved = save_track_data(rev, directory, 'reversed', f'{label.lower()}_reversed', f'{label} reversed')
        rev_loaded = load_track_data(rev_saved.xml)
        reverse_export_metrics = check_same_track(
            rev, rev_loaded, context=f'{label} reversed save/load',
            allow_serialized_closed_seam=True,
        )
        twice = reverse_track(rev_loaded)
        double_metrics = check_same_track(
            loaded, twice, context=f'{label} double reversal',
            allow_trailing_duplicate=True,
            closed_seam_heading_atol=DOUBLE_TERMINAL_HEADING_ATOL,
        )
        cuts = [0., track.length/3., 2*track.length/3.]
        sections = split_closed(track, cuts)
        section_saved = save_sections(track, cuts, directory, 'section', f'{label.lower()}_section', f'{label} section')
        section_loaded = [load_track_data(item.xml) for item in section_saved]
        section_metrics = check_sections(
            track, cuts, sections, section_loaded, saved=section_saved,
            context=f'{label} sections',
        )
        open_track = sections[0]
        open_saved = save_track_data(open_track, directory, 'open_section', f'{label.lower()}_open', f'{label} open section')
        open_loaded = load_track_data(open_saved.xml)
        open_metrics = check_same_track(
            open_track, open_loaded, context=f'{label} open save/load',
            allow_serialized_closed_seam=False,
        )
        open_reverse = reverse_track(open_loaded)
        open_reverse_saved = save_track_data(open_reverse, directory, 'open_reversed', f'{label.lower()}_open_reversed', f'{label} reversed open section')
        open_reversed = load_track_data(open_reverse_saved.xml)
        open_reverse_metrics = check_reverse(open_loaded, open_reverse, context=f'{label} open reversal')
        open_reverse_export_metrics = check_same_track(
            open_reverse, open_reversed, context=f'{label} reversed open save/load',
            allow_serialized_closed_seam=False,
        )
        open_twice = reverse_track(open_reversed)
        open_double_metrics = check_same_track(
            open_loaded, open_twice, context=f'{label} open double reversal',
            allow_serialized_closed_seam=False,
        )
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
        'roundtrips':{
            'closed_save_load':closed_metrics,
            'reverse_mapping':reverse_metrics,
            'reverse_save_load':reverse_export_metrics,
            'double_reverse':double_metrics,
            'sections':section_metrics,
            'open_section_save_load':open_metrics,
            'open_reverse':open_reverse_metrics,
            'open_reverse_save_load':open_reverse_export_metrics,
            'open_double_reverse':open_double_metrics,
        },
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
