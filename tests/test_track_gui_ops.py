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
from tools.tracks.build_closed_track import geometry_from_arrays

TRACKS=REPOSITORY_ROOT/"models"/"tracks"

class TrackOpsTests(unittest.TestCase):
    def test_open_round_trip_and_reverse(self):
        t=load_track_data(TRACKS/"fsae_2026_skidpad.xml")
        self.assertFalse(t.distance.flags.writeable)
        with self.assertRaises(ValueError): t.distance[0]=2
        self.assertEqual(t.maximum_radius, 100000.)
        r=reverse_track(t); rr=reverse_track(r)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9)
        np.testing.assert_allclose(rr.curvature,t.curvature,atol=1e-12)
        with tempfile.TemporaryDirectory() as d:
            r=TrackData(r.distance,r.x_north,r.y_east,r.heading,r.curvature,r.is_closed,r.title,777.)
            out=load_track_data(save_track_data(r,d,"open","open-id","Open & reverse").xml)
            self.assertFalse(out.is_closed); self.assertEqual(out.maximum_radius,777.)
            from simplified_vehicle_models.tracks.profile import load_track
            self.assertEqual(float(load_track(saved_xml := Path(d)/"open.xml").radius.max()), 777.)
            np.testing.assert_allclose(out.x_north,r.x_north,atol=1e-8)

    def test_closed_save_reverse_curvature_and_double(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml"); r=reverse_track(t); rr=reverse_track(r)
        np.testing.assert_allclose(r.x_north[0],t.x_north[0]); np.testing.assert_allclose(r.curvature,-t.curvature[::-1],atol=1e-10)
        np.testing.assert_allclose(rr.x_north,t.x_north,atol=1e-9); np.testing.assert_allclose(rr.curvature,t.curvature,atol=1e-10)
        with tempfile.TemporaryDirectory() as d:
            out=load_track_data(save_track_data(r,d,"closed","closed-id","Closed").xml)
            self.assertTrue(out.is_closed); np.testing.assert_allclose(out.curvature,r.curvature,atol=2e-9)

    def test_unsigned_asymmetric_reverse_twice_and_save(self):
        t=TrackData([0,1,3,6],None,None,[0]*4,[.1,.2,.3,.1],True,"legacy",777.)
        r=reverse_track(t); np.testing.assert_allclose(np.diff(r.distance),[3,2,1]); np.testing.assert_allclose(r.curvature,[.1,.3,.2,.1])
        rr=reverse_track(r); np.testing.assert_allclose(rr.curvature,t.curvature); np.testing.assert_allclose(rr.distance,t.distance)
        with tempfile.TemporaryDirectory() as d:
            out=load_track_data(save_track_data(r,d,"legacy","legacy-id","Unsigned").xml)
            self.assertIsNone(out.x_north); self.assertTrue(out.is_closed); self.assertEqual(out.maximum_radius,777.)
            np.testing.assert_allclose(out.curvature,r.curvature)
            persisted=load_track_data(save_track_data(reverse_track(out),d,"legacy_twice","legacy-id","Unsigned").xml)
            np.testing.assert_allclose(persisted.curvature,t.curvature)

    def test_both_canonical_sections_exact_seam_and_saved_validation(self):
        for name in ("2024_michigan_endurance_closed.xml","2024_michigan_endurance_multilap_closed.xml"):
            t=load_track_data(TRACKS/name); parts=split_closed(t,[t.length,t.length/3,2*t.length/3])
            self.assertEqual(len(parts),3)
            np.testing.assert_allclose([parts[0].x_north[0],parts[0].y_east[0]],[t.x_north[0],t.y_east[0]],atol=1e-8)
            np.testing.assert_allclose([parts[-1].x_north[-1],parts[-1].y_east[-1]],[t.x_north[0],t.y_east[0]],atol=1e-8)
            with tempfile.TemporaryDirectory() as d:
                saved=save_sections(t,[0,t.length/3,2*t.length/3],d,"piece","piece-id","Measured")
                for item in saved:
                    self.assertFalse(load_track_data(item.xml).is_closed)
                    self.assertIn("maximum departure from source polyline",item.xml.read_text())

    def test_cuts_invalid_and_late_destination_guard(self):
        t=load_track_data(TRACKS/"2024_michigan_endurance_closed.xml")
        for cuts in ([],[2,2],[0,t.length],[float("nan"),3],[-1,3],[0,t.length+1]):
            with self.subTest(cuts=cuts),self.assertRaises(ValueError): split_closed(t,cuts)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); (p/"sec_02.xml").write_text("occupied")
            with self.assertRaises(SaveRefused): save_sections(t,[0,t.length/2],p,"sec","id","title")
            self.assertFalse((p/"sec_01.xml").exists())
        with self.assertRaises(SaveRefused): save_track_data(t,TRACKS,"forbidden_task3_probe","id","title")

    def test_late_install_failure_restores_all_originals(self):
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

    def test_converters_accept_real_builder_geometry_and_open_course(self):
        theta=np.linspace(0,2*np.pi,200,endpoint=False)
        geom=geometry_from_arrays(np.cos(theta)*10,np.sin(theta)*10,.5,0.)
        data=from_closed_geometry(geom,maximum_radius=777.)
        self.assertTrue(data.is_closed); self.assertEqual(data.maximum_radius,777.)
        self.assertEqual(data.x_north[0],data.x_north[-1])
        self.assertFalse(from_open_course(skidpad_design(0).build()).is_closed)

if __name__=="__main__": unittest.main()
