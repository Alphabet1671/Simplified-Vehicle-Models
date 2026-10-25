import json
import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.tracks.geometry_audit import (
    _segment_distance, _segment_event, audit_track_geometry,
)
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


class TrackGeometryAuditTests(unittest.TestCase):
    def circle(self, count=13, transform=None, left=None, right=None):
        radius = 20.0
        theta = np.linspace(0.0, 2.0*np.pi, count)
        distance = radius*theta
        north, east = radius*np.sin(theta), radius*(1.0-np.cos(theta))
        heading = theta
        if transform:
            angle, shift = transform
            north, east = (north*np.cos(angle)-east*np.sin(angle)+shift[0],
                           north*np.sin(angle)+east*np.cos(angle)+shift[1])
            heading += angle
        curvature = np.full(count, 1.0/radius)
        coords = PathCoordinates(distance, north, east, heading, curvature)
        track = TrackProfile(distance, np.diff(distance), np.full(count, radius), curvature,
                             True, "circle", path_coordinates=coords,
                             half_width_left=np.asarray(left if left is not None else np.linspace(4.,6.,count)),
                             half_width_right=np.asarray(right if right is not None else np.linspace(7.,3.,count)),
                             width_provenance="surveyed")
        return track, ReferenceCurve(track), corridor_from_track(track)

    def test_circle_analytic_fit_and_json(self):
        result = audit_track_geometry(*self.circle())
        json.dumps(result, allow_nan=False)
        self.assertEqual(result["claim"], "diagnostic_only")
        self.assertAlmostEqual(result["source"]["curvature"]["range"]["min_per_m"], .05)
        self.assertAlmostEqual(result["fit"]["metric"]["min_m_per_m"], 1., places=3)
        self.assertLess(result["source_vs_fit"]["xy"]["max_abs_m"], 1e-9)

    def test_translation_rotation_invariance(self):
        a = audit_track_geometry(*self.circle())
        b = audit_track_geometry(*self.circle(transform=(.71,(120.,-80.))))
        for path, key in (("fit.metric", "min_m_per_m"), ("fit.metric", "max_m_per_m"),
                          ("fit.curvature", "min_per_m"), ("fit.curvature", "max_per_m"),
                          ("source_vs_fit.xy", "max_abs_m"), ("source_vs_fit.xy", "rms_m"),
                          ("corridor", "narrowest_left_width_m"), ("corridor", "narrowest_right_width_m")):
            av, bv = a, b
            for part in path.split("."):
                av, bv = av[part], bv[part]
            self.assertAlmostEqual(av[key], bv[key], places=7)
        self.assertEqual(a["crossings"]["count"], b["crossings"]["count"])
        self.assertAlmostEqual(a["crossings"]["minimum_nonlocal_separation_m"]["minimum_m"],
                               b["crossings"]["minimum_nonlocal_separation_m"]["minimum_m"], places=7)
        # Absolute diagnostic coordinates (for example crossing north/east) are
        # not scalar rigid-motion invariants and are intentionally not compared.

    def test_ordered_coincident_source_nodes_are_diagnostic(self):
        t=np.linspace(0,2*np.pi,33); x=12*np.sin(t); y=12*np.sin(t)*np.cos(t)
        s=np.linspace(0,100,len(t)); dx,dy=np.gradient(x,s),np.gradient(y,s)
        h=np.unwrap(np.arctan2(dy,dx)); ddx,ddy=np.gradient(dx,s),np.gradient(dy,s)
        k=(dx*ddy-dy*ddx)/np.maximum(dx*dx+dy*dy,1e-12)**1.5
        x[-1],y[-1],k[-1]=x[0],y[0],k[0]
        c=PathCoordinates(s,x,y,h,k); tr=TrackProfile(s,np.diff(s),np.ones(len(s)),k,True,"coincident",path_coordinates=c)
        result=audit_track_geometry(tr,ReferenceCurve(tr),None,sample_budget=33)
        json.dumps(result,allow_nan=False)
        self.assertEqual(result["claim"], "diagnostic_only")
        self.assertEqual(result["crossings"]["route_order_policy"]["identity"], "source_station")
        records=result["crossings"]["records"]
        self.assertTrue(any(row["station_a_m"] != row["station_b_m"] for row in records))
        self.assertNotIn("status", result)

    def test_resampling_compares_route_facts_not_fit_invariance(self):
        a=audit_track_geometry(*self.circle(count=7)); b=audit_track_geometry(*self.circle(count=17))
        self.assertNotAlmostEqual(a["fit"]["metric"]["min_m_per_m"],b["fit"]["metric"]["min_m_per_m"],places=8)
        self.assertAlmostEqual(a["corridor"]["narrowest_left_width_m"],b["corridor"]["narrowest_left_width_m"])
        self.assertEqual(a["corridor"]["provenance"],b["corridor"]["provenance"])
        self.assertEqual(a["crossings"]["count"],b["crossings"]["count"])
        self.assertEqual(a["crossings"]["route_order_policy"],b["crossings"]["route_order_policy"])

    def test_nonuniform_parabola_analytic_curvature_and_metric(self):
        p=np.array([-2.,-1.1,-.4,0.,.15,.55,1.3,2.])
        s=p+2.0  # source distance is the polynomial parameter, despite unequal gaps
        x,y=p,p*p
        h=np.arctan2(2*p,np.ones_like(p)); k=2/(1+4*p*p)**1.5
        c=PathCoordinates(s,x,y,h,k,is_closed=False)
        t=TrackProfile(s,np.diff(s),1/k,k,False,"parabola",path_coordinates=c)
        r=audit_track_geometry(t,ReferenceCurve(t),None,sample_budget=9)
        self.assertAlmostEqual(r["source"]["curvature"]["range"]["max_per_m"],2.)
        self.assertAlmostEqual(r["source_vs_fit"]["xy"]["max_abs_m"],0.,places=12)
        expected_metric=np.sqrt(1.0+4.0*p*p)
        self.assertLess(r["fit"]["metric"]["max_m_per_m"],float(expected_metric.max())+1e-9)
        self.assertGreater(r["fit"]["metric"]["min_m_per_m"],float(expected_metric.min())-1e-9)
        self.assertLess(r["source_vs_fit"]["signed_curvature"]["max_abs_per_m"],1e-9)
        self.assertLess(r["source_vs_fit"]["heading_wrapped"]["max_abs_rad"],1e-9)

    def test_figure_eight_intersection_location_order_and_separation(self):
        t=np.linspace(0,2*np.pi,33); x=12*np.sin(t); y=12*np.sin(t)*np.cos(t)
        s=np.linspace(0,100,len(t)); dx,dy=np.gradient(x,s),np.gradient(y,s)
        h=np.unwrap(np.arctan2(dy,dx)); ddx,ddy=np.gradient(dx,s),np.gradient(dy,s)
        k=(dx*ddy-dy*ddx)/np.maximum(dx*dx+dy*dy,1e-12)**1.5
        x[-1],y[-1],k[-1]=x[0],y[0],k[0]
        c=PathCoordinates(s,x,y,h,k); tr=TrackProfile(s,np.diff(s),np.ones(len(s)),k,True,"8",path_coordinates=c)
        r=audit_track_geometry(tr,ReferenceCurve(tr),None,sample_budget=250)
        self.assertGreater(r["crossings"]["count"],0)
        event=r["crossings"]["records"][0]
        self.assertLess(event["station_a_bracket_m"][0],event["station_b_bracket_m"][0])
        self.assertTrue(np.isfinite(event["north_m"]))
        sep=r["crossings"]["minimum_nonlocal_separation_m"]
        self.assertTrue(sep["available"])
        self.assertEqual(sep["minimum_m"],0.)
        self.assertGreaterEqual(r["crossings"]["count"],len(r["crossings"]["records"]))
        self.assertFalse(r["crossings"]["records_truncated"])
        self.assertAlmostEqual(event["north_m"],0.,places=1)
        self.assertAlmostEqual(event["east_m"],0.,places=1)

    def test_reversal_audit_sign_and_width_swap(self):
        tr,_,_=self.circle(count=17)
        s=tr.distance[-1]-tr.distance[::-1]
        c=PathCoordinates(s,tr.path_coordinates.x_north[::-1],tr.path_coordinates.y_east[::-1],
                          tr.path_coordinates.heading[::-1]+np.pi,-tr.curvature[::-1])
        rev=TrackProfile(s,np.diff(s),tr.radius[::-1],-tr.curvature[::-1],True,"rev",path_coordinates=c,
                         half_width_left=tr.half_width_right[::-1],half_width_right=tr.half_width_left[::-1],
                         width_provenance="surveyed")
        orig=audit_track_geometry(tr,ReferenceCurve(tr),corridor_from_track(tr))
        got=audit_track_geometry(rev,ReferenceCurve(rev),corridor_from_track(rev))
        self.assertLess(got["source"]["curvature"]["range"]["max_per_m"],0)
        self.assertAlmostEqual(got["source_vs_fit"]["signed_curvature"]["rms_per_m"],orig["source_vs_fit"]["signed_curvature"]["rms_per_m"],places=5)
        self.assertAlmostEqual(got["corridor"]["narrowest_left_width_m"],orig["corridor"]["narrowest_right_width_m"],places=6)
        self.assertGreater(orig["fit"]["curvature"]["min_per_m"],0.)
        self.assertLess(got["fit"]["curvature"]["max_per_m"],0.)

    def test_kink_source_fit_heading_error(self):
        tr,_,co=self.circle(); h=tr.path_coordinates.heading.copy(); h[-1]+=.2
        c=PathCoordinates(tr.distance,tr.path_coordinates.x_north,tr.path_coordinates.y_east,h,tr.curvature)
        kink=TrackProfile(tr.distance,tr.segment_length,tr.radius,tr.curvature,True,"kink",path_coordinates=c,
                          half_width_left=tr.half_width_left,half_width_right=tr.half_width_right,width_provenance="surveyed")
        r=audit_track_geometry(kink,ReferenceCurve(kink),co)
        self.assertGreater(r["source_vs_fit"]["heading_wrapped"]["max_abs_rad"],.05)

    def test_hairpin_and_asymmetric_pinch(self):
        th=np.linspace(-np.pi/2,np.pi/2,9); s=np.linspace(0,8*np.pi,len(th)); k=np.full(len(th),1/8)
        c=PathCoordinates(s,8*np.cos(th),8*np.sin(th),th+np.pi/2,k,is_closed=False)
        t=TrackProfile(s,np.diff(s),np.full(len(th),8.),k,False,"hairpin",path_coordinates=c,
                       half_width_left=np.ones(len(th)),half_width_right=np.ones(len(th)),width_provenance="assumed")
        r=audit_track_geometry(t,ReferenceCurve(t),corridor_from_track(t))
        self.assertTrue(np.isfinite(r["fit"]["metric"]["min_m_per_m"]))
        tr,_,_=self.circle(); left=np.full(len(tr.distance),6.); left[5]=1.25
        p=TrackProfile(tr.distance,tr.segment_length,tr.radius,tr.curvature,True,"pinch",path_coordinates=tr.path_coordinates,
                       half_width_left=left,half_width_right=np.full(len(left),3.),width_provenance="assumed")
        q=audit_track_geometry(p,ReferenceCurve(p),corridor_from_track(p))
        self.assertAlmostEqual(q["corridor"]["narrowest_left_width_m"],1.25)
        self.assertEqual(q["corridor"]["pinch_station_left_m"],tr.distance[5])

    def test_resampling_exposes_fit_change(self):
        a=audit_track_geometry(*self.circle(count=7)); b=audit_track_geometry(*self.circle(count=17))
        self.assertNotAlmostEqual(a["fit"]["metric"]["min_m_per_m"],b["fit"]["metric"]["min_m_per_m"],places=8)

    def test_pair_budget_truncation_is_reported(self):
        out=audit_track_geometry(*self.circle(count=81),crossing_pair_budget=5)
        self.assertEqual(out["crossings"]["pairs_examined"],5)
        self.assertTrue(out["crossings"]["truncated"])
        self.assertTrue(out["crossings"]["unresolved"])

    def test_budget_bool_and_nonfinite_source_explicit(self):
        tr,ref,co=self.circle()
        with self.assertRaises(ValueError): audit_track_geometry(tr,ref,co,crossing_pair_budget=True)
        x=tr.path_coordinates.x_north.copy(); x[3]=np.nan
        malformed=SimpleNamespace(distance=tr.distance,curvature=tr.curvature,path_coordinates=SimpleNamespace(
            x_north=x,y_east=tr.path_coordinates.y_east,heading=tr.path_coordinates.heading))
        out=audit_track_geometry(malformed,ref,co)
        self.assertIn("unavailable",out["source_vs_fit"]["xy"])
        badco=SimpleNamespace(distance=np.array([0.,1.]),minimum=np.array([0.,np.nan]),
                              maximum=np.array([1.,2.]),provenance="unknown")
        out2=audit_track_geometry(tr,ref,badco)
        self.assertFalse(out2["corridor"]["available"])
        json.dumps(out,allow_nan=False); json.dumps(out2,allow_nan=False)

    def test_collinear_overlap_events_and_distances(self):
        fixtures=[((0.,0.),(2.,0.),(1.,0.),(3.,0.)),
                  ((0.,0.),(3.,0.),(1.,0.),(2.,0.)),
                  ((3.,0.),(0.,0.),(2.,0.),(1.,0.))]
        for points in fixtures:
            a,b,c,d=(np.asarray(p,float) for p in points)
            event=_segment_event(a,b,c,d)
            self.assertIsNotNone(event)
            self.assertEqual(_segment_distance(a,b,c,d),0.)
            point,u,v=event
            self.assertTrue(0.<=u<=1. and 0.<=v<=1.)
            self.assertAlmostEqual(point[1],0.)
        a,b,c,d=(np.asarray(p,float) for p in ((0.,0.),(2.,0.),(1.,1.),(3.,1.)))
        self.assertIsNone(_segment_event(a,b,c,d))
        self.assertAlmostEqual(_segment_distance(a,b,c,d),1.)
        a=b=np.array([1.,0.]); c=np.array([0.,0.]); d=np.array([2.,0.])
        self.assertIsNotNone(_segment_event(a,b,c,d))
        self.assertEqual(_segment_distance(a,b,c,d),0.)
        # Exact sampled-route overlap brackets are classified as an event too.
        tr,_,_=self.circle(count=17)
        route=tr.path_coordinates
        xx=route.x_north.copy(); yy=route.y_east.copy()
        # Force two non-neighbor sampled chords onto the same collinear interval.
        xx[7:11]=[0.,1.,2.,3.]; yy[7:11]=0.
        custom=PathCoordinates(route.distance,xx,yy,route.heading,route.curvature)
        overlap=TrackProfile(tr.distance,tr.segment_length,tr.radius,tr.curvature,True,"overlap",path_coordinates=custom)
        out=audit_track_geometry(overlap,ReferenceCurve(overlap),None,sample_budget=17,crossing_pair_budget=500)
        self.assertGreater(out["crossings"]["count"],0)
        self.assertEqual(out["crossings"]["minimum_nonlocal_separation_m"]["minimum_m"],0.)


if __name__ == "__main__": unittest.main()
