import unittest
from unittest import mock
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.coupled_yaw_profile import solve_coupled_profile
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile
from simplified_vehicle_models.solvers.yaw_profile_transition import edge_evidence


def make_map():
    v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
    mask=np.ones(shape,bool)
    for i,speed in enumerate(v): mask[i,:,np.abs(ay)>max(.25,1.25-.12*speed)]=False
    return YawCapabilityMap(v,ax,ay,np.full(shape,1.e6),np.full(shape,1.e6),mask,1.,"physical_no_tv_v1","coupled-test")


class CoupledProfileTests(unittest.TestCase):
    def test_finds_and_independently_validates_task1_witness(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance)); path=YawPath(distance,k,np.gradient(k,distance),False)
        result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=4))
        self.assertEqual(result["q"][0],0.)
        validation=validate_profile(path,make_map(),np.sqrt(result["q"]),maximum_speed=8.,boundary="standing",settings=CoupledProfileSettings())
        self.assertTrue(validation.valid, (result["q"], validation))
        self.assertEqual(result["diagnostics"]["status"],"feasible_sampled")

    def test_dense_map_initial_optional_seeds_are_bounded_and_deterministic(self):
        speed=np.linspace(0.,30.,80); ax=np.linspace(-3.,3.,9); ay=np.linspace(-25.,25.,450)
        shape=(len(speed),len(ax),len(ay))
        capability=YawCapabilityMap(speed,ax,ay,np.full(shape,1.e6),np.full(shape,1.e6),
                                    np.ones(shape,bool),1.,"physical_no_tv_v1","dense-seed")
        distance=np.array([0.,.1,.2]); path=YawPath(distance,np.array([.02,.03,.02]),np.zeros(3),False)
        settings=CoupledProfileSettings(max_refinement_rounds=1,max_candidates_per_station=256,
                                        initial_optional_seed_cap=32,refinement_reserve_per_station=32)
        reports=[]
        for _ in range(2):
            try:
                result=solve_coupled_profile(path,capability,boundary="standing",maximum_speed=20.,
                    warm_start_q=np.array([0.,64.,36.]),settings=settings)
                reports.append(result["diagnostics"])
            except YawProfileSolveError as error:
                reports.append(error.diagnostics)
        for diag in reports:
            self.assertTrue(all(count <= 224 for count in diag["candidate_counts"]))
            self.assertGreater(diag["initial_optional_omitted"]["curvature_lateral_knots"],0)
            self.assertEqual(diag["initial_seed_counts"][0]["mandatory"],1)
            self.assertEqual(diag["initial_seed_counts"][0]["optional"],0)
        self.assertEqual(reports[0]["initial_seed_counts"],reports[1]["initial_seed_counts"])

    def test_settings_positional_order_is_backward_compatible(self):
        settings=CoupledProfileSettings(7,6,55,66,77,8,9.,1e-5,2e-5,3e-5,4e-5,5e-5)
        self.assertEqual(settings.q_seed_count,7)
        self.assertEqual(settings.max_refinement_rounds,6)
        self.assertEqual(settings.max_candidates_per_station,55)
        self.assertEqual(settings.refinement_speed_tolerance,5e-5)
        self.assertEqual(settings.initial_optional_seed_cap,32)

    def test_open_profile_budget_failure_is_typed(self):
        distance=np.array([0.,.05,.1]); path=YawPath(distance,np.zeros(3),np.zeros(3),False)
        settings=CoupledProfileSettings(q_seed_count=33,max_candidates_per_station=2)
        with self.assertRaises(YawProfileSolveError) as caught:
            solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
        self.assertEqual(caught.exception.status,"budget_exceeded")

    def test_rotated_path_preserves_exact_original_edge_lengths(self):
        from simplified_vehicle_models.solvers.coupled_yaw_profile import _rotate_closed_path
        distance=np.array([0., .1, .3, .30000000000000004, 10., 10.1])
        curvature=np.zeros_like(distance); path=YawPath(distance,curvature,curvature,True)
        rotated, order=_rotate_closed_path(path,2)
        expected=np.r_[path.segment_lengths[2:],path.segment_lengths[:2]]
        np.testing.assert_array_equal(rotated.segment_lengths,expected)
        self.assertFalse(np.array_equal(np.diff(rotated.distance),expected))

    def test_closed_rotation_uses_tightest_domain_and_restores_original_order(self):
        d=np.array([0.,.1,.2,.3,.4]); k=np.array([0.,.1,.5,.2,0.]); path=YawPath(d,k,np.zeros_like(k),True)
        v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
        cap=YawCapabilityMap(v,ax,ay,np.full(shape,100.),np.full(shape,100.),np.ones(shape,bool),1.,"physical_no_tv_v1","closed-rotation")
        result=solve_coupled_profile(path,cap,boundary="flying",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=2,max_seam_anchors=8))
        diag=result["diagnostics"]
        self.assertEqual(diag["closed_seam_rotation"],2)
        self.assertEqual(len(diag["tested_seam_anchors"]),len(set(diag["tested_seam_anchors"])))
        self.assertLessEqual(len(diag["tested_seam_anchors"]),32)
        self.assertEqual(result["q"].shape,(len(d),))
        self.assertAlmostEqual(result["q"][0],result["q"][-1],places=12)
        self.assertAlmostEqual(diag["validation"].seam_speed_error,0.,places=12)

    def test_strict_validator_rejection_never_returns_success(self):
        distance=np.arange(0.,.2001,.05); k=np.zeros_like(distance)
        path=YawPath(distance,k,k,False)
        rejected=mock.Mock(valid=False, recomputed_time=None)
        with mock.patch("simplified_vehicle_models.solvers.coupled_yaw_profile.validate_profile", return_value=rejected):
            with self.assertRaises(YawProfileSolveError) as caught:
                solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                    settings=CoupledProfileSettings(max_refinement_rounds=2))
        self.assertNotEqual(caught.exception.status,"feasible_sampled")
        self.assertEqual(caught.exception.status,"strict_validation_failed")

    def test_repeat_run_is_deterministic_and_incumbent_time_does_not_increase(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance))
        path=YawPath(distance,k,np.gradient(k,distance),False)
        settings=CoupledProfileSettings(max_refinement_rounds=4)
        real_validate=validate_profile
        observed=[]
        def record_validation(*args,**kwargs):
            result=real_validate(*args,**kwargs)
            if result.valid:
                observed.append(float(result.recomputed_time))
            return result
        with mock.patch("simplified_vehicle_models.solvers.coupled_yaw_profile.validate_profile", side_effect=record_validation):
            first=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
            first_times=tuple(observed)
            observed.clear()
            second=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
            second_times=tuple(observed)
        np.testing.assert_array_equal(first["q"],second["q"])
        self.assertEqual(first["time_seconds"],second["time_seconds"])
        self.assertEqual(first["diagnostics"]["tested_seam_anchors"],second["diagnostics"]["tested_seam_anchors"])
        self.assertTrue(first_times)
        self.assertEqual(first_times,second_times)
        self.assertTrue(all(b <= a*(1.+1e-10) for a,b in zip(first_times,first_times[1:])),first_times)

    def _disconnected_once(self):
        """Wrap the real open solver, forcing only the first solve to report a cut at (0, 2)."""
        from simplified_vehicle_models.solvers import coupled_yaw_profile as module
        real=module.solve_open_graph
        calls={"n":0}
        def solver(graph,start,terminals,**kwargs):
            calls["n"]+=1
            if calls["n"]==1:
                return {"status":"disconnected","node_ids":np.array([],dtype=np.int64),"time_seconds":np.inf,
                        "reachable_nodes":np.arange(graph.node_offsets[0],graph.node_offsets[1]),
                        "co_reachable_nodes":np.arange(graph.node_offsets[2],graph.node_offsets[3]),
                        "failed_layer":0,"disconnect_cut":(0,2)}
            return real(graph,start,terminals,**kwargs)
        return mock.patch.object(module,"solve_open_graph",side_effect=solver)

    def test_frontier_wave_processes_snapshot_once_and_defers_new_requests(self):
        distance=np.array([0.,.05,.1]); path=YawPath(distance,np.zeros(3),np.zeros(3),False)
        with self._disconnected_once():
            result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=3))
        waves=result["diagnostics"]["refinement_waves"]
        self.assertEqual(waves[0]["source"],"frontier")
        for wave in waves:
            self.assertEqual(wave["processed_requests"],wave["snapshot_requests"])
            self.assertLessEqual(wave["max_candidates_after"],256)
        window=result["diagnostics"]["refinement_windows"][0]
        self.assertEqual(window["cut"],(0,2))
        validation=validate_profile(path,make_map(),np.sqrt(result["q"]),maximum_speed=8.,boundary="standing",settings=CoupledProfileSettings())
        self.assertTrue(validation.valid)

    def test_acceleration_knot_projections_are_strictly_evidenced_before_insertion(self):
        distance=np.array([0.,.05,.1]); k=np.array([0.,.01,.02]); path=YawPath(distance,k,np.gradient(k,distance),False)
        with self._disconnected_once():
            result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=3))
        witnesses=result["diagnostics"]["inserted_witnesses"]
        projections=[w for w in witnesses if w["kind"]=="acceleration_knot_projection"]
        self.assertTrue(projections)
        cap=make_map(); settings=CoupledProfileSettings()
        for w in projections:
            ds=float(distance[w["segment"]+1]-distance[w["segment"]])
            sign=1. if w["direction"]=="forward" else -1.
            self.assertTrue(any(abs(w["free_q"]-(w["fixed_q"]+sign*2*ds*a))<=1e-12 for a in cap.longitudinal_acceleration))
            left,right=(w["fixed_q"],w["free_q"]) if w["direction"]=="forward" else (w["free_q"],w["fixed_q"])
            ev=edge_evidence(path,cap,w["segment"],left,right,settings=settings)
            self.assertEqual(bool(ev["feasible"]),w["strict_feasible"])
            if w["inserted"]: self.assertTrue(w["strict_feasible"])

    def test_failed_window_is_local_on_long_path_and_doubles(self):
        distance=np.arange(0.,10.0001,.05); path=YawPath(distance,np.zeros_like(distance),np.zeros_like(distance),False)
        from simplified_vehicle_models.solvers import coupled_yaw_profile as module
        real=module.solve_open_graph; calls={"n":0}
        def solver(graph,start,terminals,**kwargs):
            calls["n"]+=1
            if calls["n"]<=2:
                o=graph.node_offsets
                return {"status":"disconnected","node_ids":np.array([],dtype=np.int64),"time_seconds":np.inf,
                        "reachable_nodes":np.arange(o[0],o[101]),"co_reachable_nodes":np.arange(o[150],o[-1]),
                        "failed_layer":0,"disconnect_cut":(100,150)}
            return real(graph,start,terminals,**kwargs)
        with mock.patch.object(module,"solve_open_graph",side_effect=solver):
            result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=4))
        windows=result["diagnostics"]["refinement_windows"]
        self.assertEqual(windows[0]["cut"],(100,150))
        self.assertEqual(windows[0]["radius"],32)
        self.assertEqual(windows[0]["window_segments"],(68,132))
        # The backward frontier (segment 149) lies outside the first window, so
        # the first wave cannot reach it and the window doubles.
        self.assertEqual(windows[0].get("backward_steps",0),0)
        self.assertEqual(windows[1]["radius"],64)
        self.assertEqual(windows[1]["window_segments"],(36,164))
        self.assertGreater(windows[1]["backward_steps"],0)
        # The forward sweep starts at segment 100 and cannot leave [68, 132].
        self.assertLessEqual(windows[0]["forward_steps"],132-100+1)

    def test_wall_deadline_reports_budget_stop(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance)); path=YawPath(distance,k,np.gradient(k,distance),False)
        with self.assertRaises(YawProfileSolveError) as caught:
            solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                settings=CoupledProfileSettings(max_wall_seconds=1e-9))
        self.assertEqual(caught.exception.status,"budget_exceeded")
        self.assertEqual(caught.exception.diagnostics["budget_reason"],"wall_deadline")

    def test_budget_stop_after_incumbent_returns_unconverged_validated_profile(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance)); path=YawPath(distance,k,np.gradient(k,distance),False)
        from simplified_vehicle_models.solvers import coupled_yaw_profile as module
        real_pack=module.pack_layered_q_graph; real_validate=module.validate_profile; state={"valid":False}
        def validate(*args,**kwargs):
            result=real_validate(*args,**kwargs)
            state["valid"]=state["valid"] or bool(result.valid)
            return result
        def pack(*args,**kwargs):
            # Only the graph built after a real validated incumbent exceeds its budget.
            return {"status":"budget_exceeded"} if state["valid"] else real_pack(*args,**kwargs)
        with mock.patch.object(module,"pack_layered_q_graph",side_effect=pack), \
             mock.patch.object(module,"validate_profile",side_effect=validate):
            result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                settings=CoupledProfileSettings(max_refinement_rounds=4))
        diag=result["diagnostics"]
        self.assertEqual(diag["budget_reason"],"graph_budget")
        self.assertFalse(diag["resolution_converged"])
        self.assertIn("graph_budget",diag["stopping_uncertainty"])
        validation=validate_profile(path,make_map(),np.sqrt(result["q"]),maximum_speed=8.,boundary="standing",settings=CoupledProfileSettings())
        self.assertTrue(validation.valid)

    def test_closed_periodic_profile(self):
        d=np.array([0.,.1,.2,.3]); k=np.zeros(4); path=YawPath(d,k,k,True)
        v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
        cap=YawCapabilityMap(v,ax,ay,np.full(shape,100.),np.full(shape,100.),np.ones(shape,bool),1.,"physical_no_tv_v1","closed")
        result=solve_coupled_profile(path,cap,boundary="flying",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=2))
        self.assertAlmostEqual(result["q"][0],result["q"][-1],places=12)
        self.assertGreater(result["time_seconds"],0)
