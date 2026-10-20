"""Optional acceptance checks for private MIS and FSC telemetry fixtures."""
from __future__ import annotations
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

from tools.track_gui.validate_track_imports import validate_fixture

MIS_DEFAULT = Path('/Users/alphabet1671/Desktop/0615CompEndurance.mat')
FSC_DEFAULT = Path('/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat')
MIS = Path(os.environ.get('TRACK_GUI_MIS_MAT', MIS_DEFAULT))
FSC = Path(os.environ.get('TRACK_GUI_FSC_MAT', FSC_DEFAULT))
_OPTIONAL_DEPS = importlib.util.find_spec('scipy') is not None and importlib.util.find_spec('matplotlib') is not None
_SKIP = 'requires optional SciPy and matplotlib dependencies'


@unittest.skipUnless(_OPTIONAL_DEPS and MIS.is_file(), f'{_SKIP}; private MIS fixture absent: set TRACK_GUI_MIS_MAT (local default {MIS_DEFAULT})')
class MISRealDataAcceptance(unittest.TestCase):
    def test_actual_mis_fixture_roundtrips(self):
        with tempfile.TemporaryDirectory() as folder:
            result = validate_fixture('MIS', MIS, Path(folder), gps=False)
        self.assertGreaterEqual(result['lap_diagnostics']['accepted_laps'], 2)
        self.assertTrue(result['roundtrips']['double_reverse_xy'])
        self.assertTrue(result['roundtrips']['reverse_preserves_start_and_flips_curvature'])
        self.assertTrue(result['roundtrips']['open_section_save_reverse_load'])
        self.assertEqual(result['roundtrips']['sections'], 3)


@unittest.skipUnless(_OPTIONAL_DEPS and FSC.is_file(), f'{_SKIP}; private FSC fixture absent: set TRACK_GUI_FSC_MAT (local default {FSC_DEFAULT})')
class FSCRealDataAcceptance(unittest.TestCase):
    def test_actual_fsc_fixture_roundtrips(self):
        with tempfile.TemporaryDirectory() as folder:
            result = validate_fixture('FSC', FSC, Path(folder), gps=True)
        self.assertGreaterEqual(result['lap_diagnostics']['accepted_laps'], 2)
        self.assertTrue(result['roundtrips']['double_reverse_xy'])
        self.assertTrue(result['roundtrips']['reverse_preserves_start_and_flips_curvature'])
        self.assertTrue(result['roundtrips']['open_section_save_reverse_load'])
        self.assertEqual(result['roundtrips']['sections'], 3)


if __name__ == '__main__':
    unittest.main()
