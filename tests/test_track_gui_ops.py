import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from tools.gui_common import REPOSITORY_ROOT
from tools.track_gui.track_ops import (TrackData, load_track_data, reverse_track, split_closed,
    save_track_data, save_sections, from_closed_geometry, from_open_course)
from tools.track_gui.export import SaveRefused
from tools.track_gui.design import skidpad_design

TRACKS=REPOSITORY_ROOT/"models"/"tracks"

class TrackOpsTests(unittest.TestCase):
    def test_open_save_load_reverse_immutable(self):
        t=load_track_data(TRACKS/"fsae_2026_skidpad.xml")
        self.assertFalse(t.distance.flags.writeable)
        with self.assertRaises(ValueError): t.distance[0]=2
        r=reverse_track(t); rr=reverse_track(r)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9)
        np.testing.assert_allclose(rr.curvature,t.curvature,atol=1e-12)
        with tempfile.TemporaryDirectory() as d:
            out=load_track_data(save_track_data(r,d,"open","open-id","Open & reverse").xml)
            self.assertFalse(out.is_closed); np.testing.assert_allclose(out.x_north,r.x_north,atol=1e-8)

    def test_closed_save_load_reverse_double_and_curvature(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        r=reverse_track(t); rr=reverse_track(r)
        np.testing.assert_allclose(r.x_north[0],t.x_north[0]); np.testing.assert_allclose(r.curvature,-t.curvature[::-1],atol=1e-10)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9)
        np.testing.assert_allclose(rr.curvature,t.curvature,atol=1e-10)
        with tempfile.TemporaryDirectory() as d:
            out=load_track_data(save_track_data(r,d,"closed","closed-id","Closed").xml)
            self.assertTrue(out.is_closed); np.testing.assert_allclose(out.curvature,r.curvature,atol=2e-9)

    def test_unsigned_asymmetric_segment_reversal_and_save(self):
        t=TrackData([0,1,3,6],None,None,[0]*4,[.1,.2,.3,.1],True,"legacy")
        r=reverse_track(t)
        np.testing.assert_allclose(np.diff(r.distance),[3,2,1]); np.testing.assert_allclose(r.curvature,[.3,.2,.1,.1]); self.assertIsNone(r.x_north)
        with tempfile.TemporaryDirectory() as d:
            out=load_track_data(save_track_data(r,d,"legacy","legacy-id","Unsigned").xml)
            self.assertIsNone(out.x_north); self.assertTrue(out.is_closed); np.testing.assert_allclose(out.curvature,r.curvature)

    def test_both_canonical_sources_split_save_and_seam(self):
        for name in ("2024_michigan_endurance_closed.xml","2024_michigan_endurance_multilap_closed.xml"):
            t=load_track_data(TRACKS/name)
            with self.subTest(name=name):
                parts=split_closed(t,[t.length,t.length/3,2*t.length/3]); self.assertEqual(len(parts),3)
                np.testing.assert_allclose([parts[0].x_north[0],parts[0].y_east[0]],[t.x_north[0],t.y_east[0]],atol=1e-8)
                np.testing.assert_allclose([parts[-1].x_north[-1],parts[-1].y_east[-1]],[t.x_north[0],t.y_east[0]],atol=1e-8)
                with tempfile.TemporaryDirectory() as d:
                    saved=save_sections(t,[0,t.length/3,2*t.length/3],d,"piece","piece-id","Measured")
                    self.assertEqual(len(saved),3)
                    for item in saved:
                        out=load_track_data(item.xml); self.assertFalse(out.is_closed)
                        self.assertIn("maximum departure from source polyline",item.xml.read_text())

    def test_invalid_cuts_and_late_guard(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        for cuts in ([],[2,2],[0,t.length],[float("nan"),3],[-1,3],[0,t.length+1]):
            with self.subTest(cuts=cuts),self.assertRaises(ValueError): split_closed(t,cuts)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); (p/"seg_02.xml").write_text("occupied")
            with self.assertRaises(SaveRefused): save_sections(t,[0,t.length/2],p,"seg","id","title")
            self.assertFalse((p/"seg_01.xml").exists())
        with self.assertRaises(SaveRefused): save_track_data(t,TRACKS,"forbidden_task3_probe","id","title")

    def test_later_install_failure_restores_batch(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        with tempfile.TemporaryDirectory() as d:
            save_sections(t,[0,t.length/2],d,"part","id","Title")
            before={p.name:p.read_bytes() for p in Path(d).iterdir()}; real=os.replace; calls=0
            def fail_late(src,dst):
                nonlocal calls
                calls+=1
                if calls==8: raise OSError("injected later target failure")
                return real(src,dst)
            with mock.patch("tools.track_gui.track_ops.os.replace",side_effect=fail_late):
                with self.assertRaisesRegex(OSError,"injected"): save_sections(t,[0,t.length/2],d,"part","id","Title",overwrite=True)
            self.assertEqual({p.name:p.read_bytes() for p in Path(d).iterdir()},before)

    def test_converters(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        self.assertTrue(from_closed_geometry({k:getattr(t,k) for k in ("distance","x_north","y_east","heading","curvature")}).is_closed)
        self.assertFalse(from_open_course(skidpad_design(0).build()).is_closed)

if __name__=="__main__": unittest.main()
