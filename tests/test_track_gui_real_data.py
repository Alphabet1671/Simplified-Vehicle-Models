"""Optional private-fixture acceptance and source-anchored export mutation tests."""
from __future__ import annotations
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

from tools.track_gui.track_ops import (
    TrackData, load_track_data, reverse_track, save_sections, save_track_data,
    split_closed,
)
from tools.track_gui.validate_track_imports import (
    _check_reverse as check_reverse,
    _check_same_track as check_same_track,
    _check_sections as check_sections,
    validate_fixture,
)

MIS_DEFAULT = Path('/Users/alphabet1671/Desktop/0615CompEndurance.mat')
FSC_DEFAULT = Path('/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat')
MIS = Path(os.environ.get('TRACK_GUI_MIS_MAT', MIS_DEFAULT))
FSC = Path(os.environ.get('TRACK_GUI_FSC_MAT', FSC_DEFAULT))
ROOT = Path(__file__).resolve().parents[1]
CANONICAL = ROOT / 'models' / 'tracks' / 'mis_ev_endurance_2024_linear.xml'
_OPTIONAL_DEPS = importlib.util.find_spec('scipy') is not None and importlib.util.find_spec('matplotlib') is not None
_PRIVATE_SKIP = 'requires optional SciPy and matplotlib dependencies'


def _assert_tight(test: unittest.TestCase, metrics: dict):
    test.assertLessEqual(metrics['distance_max_error_m'], 1e-7)
    test.assertLessEqual(metrics['xy_max_error_m'], 1e-7)
    test.assertLessEqual(metrics['heading_wrapped_max_error_rad'], 2e-9)
    test.assertLessEqual(metrics['curvature_max_error_inv_m'], 1e-9)
    test.assertLessEqual(metrics['length_error_m'], 1e-7)


def _assert_actual_acceptance(test: unittest.TestCase, result: dict):
    closed = result['roundtrips']['closed_save_load']
    test.assertEqual(closed['nodes'], result['geometry']['nodes'] + 1)
    _assert_tight(test, closed)
    test.assertEqual(closed['heading_seam_source'], 'writer_final_step_extension')
    test.assertLessEqual(closed['terminal_heading_wrapped_error_rad'], 2e-9)

    reverse = result['roundtrips']['reverse_mapping']
    _assert_tight(test, reverse)
    test.assertLess(reverse['first_reverse_segment_dot'], 0.0)
    test.assertLessEqual(reverse['start_moved_m'], 1e-7)
    _assert_tight(test, result['roundtrips']['reverse_save_load'])

    double = result['roundtrips']['double_reverse']
    test.assertLessEqual(double['distance_max_error_m'], 1e-7)
    test.assertLessEqual(double['xy_max_error_m'], 1e-7)
    test.assertLessEqual(double['curvature_max_error_inv_m'], 1e-9)
    test.assertLessEqual(double['base_heading_wrapped_max_error_rad'], 2e-9)
    test.assertLessEqual(double['terminal_heading_wrapped_error_rad'], 5e-4)
    test.assertLessEqual(double['terminal_heading_limit_rad'], 5e-4)

    sections = result['roundtrips']['sections']
    test.assertEqual(sections['sections'], 3)
    test.assertEqual(len(sections['node_counts']), 3)
    test.assertLessEqual(sections['distance_max_error_m'], 1e-7)
    test.assertLessEqual(sections['xy_max_error_m'], 1e-7)
    test.assertLessEqual(sections['heading_wrapped_max_error_rad'], 2e-9)
    test.assertLessEqual(sections['curvature_max_error_inv_m'], 1e-9)
    test.assertLessEqual(sections['endpoint_max_error_m'], .02)
    test.assertLessEqual(sections['stored_distance_vs_arc_max_error_m'], .02)
    test.assertLessEqual(
        sections['arc_vs_cut_span_max_error_m'],
        max(.02, 1e-4 * sum(sections['section_lengths_m'])),
    )
    test.assertLessEqual(
        sections['cut_span_limit_m'], max(.02, 1e-4 * sum(sections['section_lengths_m'])),
    )
    test.assertLessEqual(sections['wrap_to_start_error_m'], .02)
    test.assertEqual(len(sections['corrections']), 3)
    for correction in sections['corrections']:
        test.assertLessEqual(correction['bound_m'], .02)
        test.assertLessEqual(correction['maximum_correction_m'], correction['bound_m'])

    open_metrics = result['roundtrips']['open_section_save_load']
    _assert_tight(test, open_metrics)
    open_reverse = result['roundtrips']['open_reverse']
    _assert_tight(test, open_reverse)
    test.assertLess(open_reverse['first_reverse_segment_dot'], 0.0)
    _assert_tight(test, result['roundtrips']['open_reverse_save_load'])
    test.assertLessEqual(result['roundtrips']['open_double_reverse']['xy_max_error_m'], 1e-7)
    test.assertLessEqual(result['roundtrips']['open_double_reverse']['heading_wrapped_max_error_rad'], 2e-9)


@unittest.skipUnless(_OPTIONAL_DEPS and MIS.is_file(), f'{_PRIVATE_SKIP}; private MIS fixture absent: set TRACK_GUI_MIS_MAT (local default {MIS_DEFAULT})')
class MISRealDataAcceptance(unittest.TestCase):
    def test_actual_mis_fixture_roundtrips(self):
        with tempfile.TemporaryDirectory() as folder:
            result = validate_fixture('MIS', MIS, Path(folder), gps=False)
        self.assertGreaterEqual(result['lap_diagnostics']['accepted_laps'], 2)
        _assert_actual_acceptance(self, result)


@unittest.skipUnless(_OPTIONAL_DEPS and FSC.is_file(), f'{_PRIVATE_SKIP}; private FSC fixture absent: set TRACK_GUI_FSC_MAT (local default {FSC_DEFAULT})')
class FSCRealDataAcceptance(unittest.TestCase):
    def test_actual_fsc_fixture_roundtrips(self):
        with tempfile.TemporaryDirectory() as folder:
            result = validate_fixture('FSC', FSC, Path(folder), gps=True)
        self.assertGreaterEqual(result['lap_diagnostics']['accepted_laps'], 2)
        _assert_actual_acceptance(self, result)


@unittest.skipUnless(CANONICAL.is_file() and importlib.util.find_spec('scipy') is not None, 'requires canonical closed track and SciPy')
class SourceAnchoredExportMutationTests(unittest.TestCase):
    def setUp(self):
        self.track = load_track_data(CANONICAL)
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self.folder = Path(self._temporary.name)
        saved = save_track_data(self.track, self.folder, 'anchor', 'anchor_track', 'Anchor track')
        self.exported = load_track_data(saved.xml)

    def _mutated(self, **fields) -> TrackData:
        values = {
            'distance': self.exported.distance.copy(),
            'x_north': self.exported.x_north.copy(),
            'y_east': self.exported.y_east.copy(),
            'heading': self.exported.heading.copy(),
            'curvature': self.exported.curvature.copy(),
            'is_closed': self.exported.is_closed,
            'title': self.exported.title,
            'maximum_radius': self.exported.maximum_radius,
        }
        values.update(fields)
        return TrackData(**values)

    def test_canonical_export_is_tight_and_translation_is_rejected(self):
        metrics = check_same_track(self.exported, self.track, context='canonical export')
        _assert_tight(self, metrics)
        self.assertEqual(metrics['serialized_seam_rows'], 0)
        translated = self._mutated(
            x_north=self.exported.x_north + 10.0,
            y_east=self.exported.y_east + 10.0,
        )
        with self.assertRaisesRegex(AssertionError, 'xy_max_error_m'):
            check_same_track(self.exported, translated, context='translated')

    def test_row_rotation_is_rejected(self):
        rotated = self._mutated(
            x_north=self.exported.x_north[1:].tolist() + self.exported.x_north[:1].tolist(),
            y_east=self.exported.y_east[1:].tolist() + self.exported.y_east[:1].tolist(),
        )
        with self.assertRaisesRegex(AssertionError, 'xy_max_error_m'):
            check_same_track(self.exported, rotated, context='row rotation')

    def test_heading_offset_is_rejected(self):
        shifted = self._mutated(heading=self.exported.heading + .1)
        with self.assertRaisesRegex(AssertionError, 'heading_wrapped_max_error_rad'):
            check_same_track(self.exported, shifted, context='heading offset')

    def test_reverse_mapping_rejects_wrong_curvature_sign(self):
        reversed_track = reverse_track(self.exported)
        metrics = check_reverse(self.exported, reversed_track, context='canonical reverse')
        _assert_tight(self, metrics)
        wrong = TrackData(
            reversed_track.distance, reversed_track.x_north, reversed_track.y_east,
            reversed_track.heading, -reversed_track.curvature, True,
            reversed_track.title, reversed_track.maximum_radius,
        )
        with self.assertRaisesRegex(AssertionError, 'curvature_max_error_inv_m'):
            check_reverse(self.exported, wrong, context='wrong sign')

    def test_sections_match_and_shifted_cut_is_rejected(self):
        cuts = [0.0, self.track.length / 3.0, 2.0 * self.track.length / 3.0]
        sections = split_closed(self.track, cuts)
        saved = save_sections(self.track, cuts, self.folder, 'anchor_section', 'anchor_section', 'Anchor sections')
        loaded = [load_track_data(item.xml) for item in saved]
        metrics = check_sections(
            self.track, cuts, sections, loaded, saved=saved, context='canonical sections',
        )
        self.assertEqual(metrics['sections'], 3)
        self.assertLessEqual(metrics['endpoint_max_error_m'], .02)
        self.assertLessEqual(metrics['distance_max_error_m'], 1e-7)
        with self.assertRaisesRegex(AssertionError, 'cut endpoint'):
            check_sections(
                self.track, [cut + .5 for cut in cuts], sections, loaded,
                context='shifted cut',
            )

    def test_open_start_row_shift_is_rejected(self):
        cuts = [0.0, self.track.length / 2.0]
        section = split_closed(self.track, cuts)[0]
        saved = save_track_data(section, self.folder, 'open_anchor', 'open_anchor', 'Open anchor')
        loaded = load_track_data(saved.xml)
        _assert_tight(self, check_same_track(section, loaded, context='open canonical'))
        shifted = TrackData(
            loaded.distance[1:] - loaded.distance[1], loaded.x_north[1:],
            loaded.y_east[1:], loaded.heading[1:], loaded.curvature[1:], False,
            loaded.title, loaded.maximum_radius,
        )
        with self.assertRaisesRegex(AssertionError, 'node count'):
            check_same_track(loaded, shifted, context='open start shift')


if __name__ == '__main__':
    unittest.main()
