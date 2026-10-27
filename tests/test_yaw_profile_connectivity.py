import unittest
from unittest import mock
import hashlib
import json
from pathlib import Path
import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.coupled_yaw_profile import solve_coupled_profile
from simplified_vehicle_models.solvers.yaw_profile_transition import edge_evidence, transition_intervals


class ConnectivityDiagnosticsTests(unittest.TestCase):
    def fixture(self):
        speed=np.linspace(0.,8.,17); ax=np.linspace(-2.,2.,9); ay=np.linspace(-2.,2.,17)
        shape=(len(speed),len(ax),len(ay))
        cap=YawCapabilityMap(speed,ax,ay,np.full(shape,1.e6),np.full(shape,1.e6),np.ones(shape,bool),
                             1.,'physical_no_tv_v1','connectivity-fixed')
        return YawPath(np.array([0.,10.,20.]),np.array([.01,.02,.03]),np.array([.001,.001,.001]),False),cap

    def disconnected_once(self,module):
        real=module.solve_open_graph; calls={'n':0}
        def solver(graph,start,terminals,**kwargs):
            calls['n']+=1
            if calls['n']==1:
                o=graph.node_offsets
                return {'status':'disconnected','node_ids':np.array([],dtype=np.int64),'time_seconds':np.inf,
                        'reachable_nodes':np.arange(o[0],o[1]),'co_reachable_nodes':np.arange(o[-2],o[-1]),
                        'failed_layer':0,'disconnect_cut':(0,len(o)-2)}
            return real(graph,start,terminals,**kwargs)
        return solver

    def test_profile_reports_connectivity_diagnostics_and_intervals(self):
        from simplified_vehicle_models.solvers import coupled_yaw_profile as module
        path,cap=self.fixture()
        with mock.patch.object(module,'solve_open_graph',side_effect=self.disconnected_once(module)):
            result=solve_coupled_profile(path,cap,boundary='standing',maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=3))
        diag=result['diagnostics']
        for name in ('interval_calls','interval_cache_hits','strict_probe_count','pair_count','peak_pair_batch',
                     'frontier_by_round','candidate_cap_hits','deferred_requests'):
            self.assertIn(name,diag)
        self.assertGreater(diag['interval_calls'],0); self.assertGreater(diag['pair_count'],0)
        self.assertLessEqual(diag['peak_pair_batch'],65536)
        self.assertGreater(diag['strict_probe_count'],0)
        self.assertIn('interval_seconds',diag['timings'])
        self.assertGreater(len(diag['frontier_by_round']),0)

    def test_cache_key_includes_exact_query_and_bounds(self):
        from simplified_vehicle_models.solvers import coupled_yaw_profile as module
        path,cap=self.fixture(); real=module.transition_intervals; seen=[]
        def record(*args,**kwargs):
            seen.append((id(args[1]),args[2],float(args[3]).hex(),args[4],tuple(float(x).hex() for x in args[5])))
            return real(*args,**kwargs)
        with mock.patch.object(module,'transition_intervals',side_effect=record), \
             mock.patch.object(module,'solve_open_graph',side_effect=self.disconnected_once(module)):
            solve_coupled_profile(path,cap,boundary='standing',maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=3))
        self.assertTrue(seen)
        normalized=list(seen)
        self.assertEqual(len(normalized),len(set(normalized)))

    def test_chunking_preserves_order_and_partial_tail(self):
        pairs=[(i,j) for i in range(7) for j in range(11)]
        chunks=[pairs[k:k+16] for k in range(0,len(pairs),16)]
        self.assertEqual([p for chunk in chunks for p in chunk],pairs)
        self.assertLessEqual(max(map(len,chunks)),16); self.assertEqual(len(chunks[-1]),13)

    def test_small_pair_cap_preserves_winner(self):
        path,cap=self.fixture(); outputs=[]
        for cap_pairs in (65536,16):
            outputs.append(solve_coupled_profile(path,cap,boundary='standing',maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=2,max_pair_batch=cap_pairs)))
        np.testing.assert_array_equal(outputs[0]['q'],outputs[1]['q'])
        self.assertEqual(outputs[0]['time_seconds'],outputs[1]['time_seconds'])
        self.assertLessEqual(outputs[1]['diagnostics']['peak_pair_batch'],16)

    def test_early_diagnostic_failures_have_complete_shape(self):
        path,cap=self.fixture()
        with self.assertRaises(YawProfileSolveError) as caught:
            solve_coupled_profile(path,cap,boundary='standing',maximum_speed=8.,
                settings=CoupledProfileSettings(q_seed_count=33,max_candidates_per_station=2))
        diag=caught.exception.diagnostics
        for name in ('interval_calls','interval_cache_hits','strict_probe_count','pair_count','peak_pair_batch',
                     'frontier_by_round','candidate_cap_hits','deferred_requests'):
            self.assertIn(name,diag)
        self.assertIn('interval_seconds',diag['timings'])

    def test_deadline_stops_inside_interval_probe_work(self):
        path,cap=self.fixture()
        with mock.patch('simplified_vehicle_models.solvers.coupled_yaw_profile.time.monotonic',
                        side_effect=[0.,0.,0.,10.,10.,10.,10.,10.]):
            with self.assertRaises(YawProfileSolveError) as caught:
                solve_coupled_profile(path,cap,boundary='standing',maximum_speed=8.,
                    settings=CoupledProfileSettings(max_wall_seconds=.5,max_refinement_rounds=3))
        self.assertEqual(caught.exception.status,'budget_exceeded')
        self.assertEqual(caught.exception.diagnostics['budget_reason'],'wall_deadline')
        self.assertIn('strict_probe_count',caught.exception.diagnostics)

    def test_deadline_interrupts_endpoint_correction_without_partial_intervals(self):
        path,cap=self.fixture()
        counters={"deadline_check":lambda:False}
        intervals=transition_intervals(path,cap,0,16.0,"forward",(15.9,16.1),
                                       settings=CoupledProfileSettings(),counters=counters)
        self.assertTrue(intervals)
        calls={"n":0}
        def fake_clock_deadline():
            calls["n"]+=1
            return calls["n"] >= 55
        interrupted={"deadline_check":fake_clock_deadline}
        partial=transition_intervals(path,cap,0,16.0,"forward",(15.9,16.1),
                                     settings=CoupledProfileSettings(),counters=interrupted)
        self.assertEqual(partial,())
        self.assertTrue(interrupted.get("deadline_stop"))
        self.assertLess(interrupted.get("strict_probe_count",0),counters.get("strict_probe_count",0))

    def test_representative_narrow_physical_window_scalar_batch_equivalence(self):
        """Freeze an actual MIS 0.05 m failed-transition segment and compare both oracles."""
        fixture_path=Path(__file__).parent/'fixtures'/'yaw_profile'/'mis-raw-centerline-original-window.json'
        doc=json.loads(fixture_path.read_text())
        w,m=doc['window'],doc['map_evidence']; a=lambda x:np.asarray(x,float)
        cap=YawCapabilityMap(a(m['speed']),a(m['ax']),a(m['ay']),a(m['yaw_moment_into_turn']),
            a(m['yaw_moment_out_of_turn']),np.asarray(m['feasible_mask'],bool),float(m['yaw_inertia']),
            'physical_no_tv_v1',m['identity'])
        distance=a(w['distance_m']); distance-=distance[0]
        path=YawPath(distance,a(w['curvature_per_m']),a(w['curvature_rate_per_m2']),False)
        segment=int(w['failed_segment']); q_fixed=float(cap.speed[2]**2)
        bounds=(q_fixed-0.3,q_fixed+0.3)
        digest=hashlib.sha256()
        for values in (cap.speed,cap.longitudinal_acceleration,cap.lateral_acceleration,cap.feasible_mask,
                       cap.yaw_moment_into_turn,cap.yaw_moment_out_of_turn):
            digest.update(np.ascontiguousarray(values).tobytes())
        self.assertEqual(digest.hexdigest(),'6beca2ba9deb9c88fe47894c8a23fb57b761a5b19e11b4c2517e8cc4cd21fb49')
        self.assertAlmostEqual(float(path.segment_lengths[segment]),0.04999757312957953,places=14)
        self.assertAlmostEqual(float(path.curvature[segment]),-0.11853366,places=8)
        self.assertAlmostEqual(float(path.curvature_rate[segment]),0.00287788,places=8)
        scalar=transition_intervals(path,cap,segment,q_fixed,'forward',bounds,settings=CoupledProfileSettings())
        # Same actual q pairs are checked by the production batched strict oracle;
        # each batch result is matched against scalar edge_evidence in row-major order.
        from simplified_vehicle_models.solvers.yaw_profile_transition import edge_feasibility_matrix
        q_free=np.linspace(bounds[0],bounds[1],41)
        batched=edge_feasibility_matrix(path,cap,segment,np.full(q_free.size,q_fixed),q_free,pairwise=True)
        expected=np.asarray([edge_evidence(path,cap,segment,q_fixed,float(q),settings=CoupledProfileSettings())['feasible']
                             for q in q_free])
        np.testing.assert_array_equal(batched,expected)
        for iv in scalar:
            for q in (iv.lower,(iv.lower+iv.upper)*.5,iv.upper):
                self.assertTrue(edge_evidence(path,cap,segment,q_fixed,float(q),settings=CoupledProfileSettings())['feasible'])
        self.assertTrue(np.any(batched))
        # Freeze the actual strict component, not merely oracle agreement.
        self.assertEqual(len(scalar),1)
        self.assertAlmostEqual(scalar[0].lower,36.768163596591705,places=8)
        self.assertAlmostEqual(scalar[0].upper,36.9931526756748,places=8)
        self.assertEqual(scalar[0].support_id,())  # union includes multiple strict support cells
        self.assertTrue(all(edge_evidence(path,cap,segment,q_fixed,float(q),settings=CoupledProfileSettings())['feasible']
                            for q in (scalar[0].lower,(scalar[0].lower+scalar[0].upper)*.5,scalar[0].upper)))
        self.assertTrue(np.any(~cap.feasible_mask))

    def test_max_pair_batch_cannot_exceed_hard_allocation_limit(self):
        with self.assertRaisesRegex(ValueError,'must not exceed 65536'):
            CoupledProfileSettings(max_pair_batch=65537)

    def test_settings_append_preserves_positional_compatibility(self):
        settings=CoupledProfileSettings(7,6,55,66,77,8,9.,1e-5,2e-5,3e-5,4e-5,5e-5)
        self.assertEqual(settings.refinement_speed_tolerance,5e-5)
        self.assertEqual(settings.max_pair_batch,65536)


if __name__=='__main__': unittest.main()
