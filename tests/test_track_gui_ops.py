import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.track_gui.track_ops import TrackData, load_track_data, reverse_track, split_closed, save_sections
from tools.track_gui.export import SaveRefused

ROOT=Path(__file__).resolve().parents[1]
TRACKS=ROOT/"models"/"tracks"

class TrackOpsTests(unittest.TestCase):
    def test_open_reverse_and_double_reverse(self):
        t=load_track_data(TRACKS/"fsae_2026_skidpad.xml")
        r=reverse_track(t); rr=reverse_track(r)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9)
        np.testing.assert_allclose(rr.curvature,t.curvature,atol=1e-12)
        self.assertLess(np.min(r.curvature),0)

    def test_closed_start_and_reverse(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        r=reverse_track(t)
        np.testing.assert_allclose(r.x_north[0],t.x_north[0])
        np.testing.assert_allclose(r.y_east[0],t.y_east[0])
        rr=reverse_track(r)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9)

    def test_sections_and_batch_guard(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        parts=split_closed(t,[0,t.length/3,2*t.length/3])
        self.assertEqual(len(parts),3)
        self.assertFalse(any(p.is_closed for p in parts))
        with tempfile.TemporaryDirectory() as d:
            (Path(d)/"seg_02.xml").write_text("occupied")
            with self.assertRaises(SaveRefused):
                save_sections(t,[0,t.length/2],d,"seg","id","title")
            self.assertFalse((Path(d)/"seg_01.xml").exists())

    def test_invalid_cuts(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        for cuts in ([],[2,2],[float("nan"),3]):
            with self.assertRaises(ValueError): split_closed(t,cuts)

if __name__=="__main__": unittest.main()
