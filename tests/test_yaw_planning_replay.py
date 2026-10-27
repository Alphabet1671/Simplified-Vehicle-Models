import unittest
import tempfile
import json
from pathlib import Path
import numpy as np
from simplified_vehicle_models.planning.yaw_replay_metrics import yaw_approximation_diagnostics, tangential_acceleration_from_body, periodic_centered_derivative, _periodic_nonuniform_derivative
from tools.reports.compare_yaw_planning_replay import replay_info, plan_info

class YawApproximationTests(unittest.TestCase):
    def test_constant_radius_zero_sideslip(self):
        n=256; t=np.arange(n)*.01; speed=20.; kappa=.01
        result=yaw_approximation_diagnostics(t,np.arange(n)*speed*.01,np.full(n,speed),np.zeros(n),np.full(n,speed*kappa),np.zeros(n),np.zeros(n),np.full(n,kappa),filter_width=5)
        self.assertLess(result['raw']['course_rate_minus_Vkappa']['max_abs'],1e-12)
        self.assertLess(result['raw']['yaw_acceleration_minus_path_demand']['max_abs'],1e-12)

    def test_sinusoidal_beta_rate_correction(self):
        n=1000; dt=.002; t=np.arange(n)*dt; speed=18.; kappa=.015
        beta=.08*np.sin(2*np.pi*t/(n*dt)); beta_dot=.08*(2*np.pi/(n*dt))*np.cos(2*np.pi*t/(n*dt))
        r=speed*kappa-beta_dot; rdot=np.gradient(r,dt,edge_order=2)
        out=yaw_approximation_diagnostics(t,np.arange(n)*speed*dt,speed*np.cos(beta),speed*np.sin(beta),r,rdot,np.zeros(n),np.full(n,kappa))
        self.assertLess(out['raw']['course_rate_minus_Vkappa']['rms'],2e-6)
        self.assertGreater(out['raw']['yaw_acceleration_minus_path_demand']['rms'],0.)
        self.assertTrue(out['diagnostic_only_not_force_balance'])

    def test_nonuniform_spacing_rejected(self):
        with self.assertRaises(ValueError): yaw_approximation_diagnostics([0,1,3],[0,1,3],[1,1,1],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0])

    def test_sinusoidal_beta_derivative_and_analytic_yaw_accel(self):
        n=2048; dt=.01; t=np.arange(n)*dt; period_t=n*dt; v=17.; beta=.12*np.sin(2*np.pi*t/period_t)
        beta_dot=.12*(2*np.pi/period_t)*np.cos(2*np.pi*t/period_t)
        beta_ddot=-.12*(2*np.pi/period_t)**2*np.sin(2*np.pi*t/period_t)
        k=.012; r=v*k-beta_dot; rdot=-beta_ddot
        at=np.zeros(n); station=np.arange(n)*v*dt; lap=v*period_t
        out=yaw_approximation_diagnostics(t,station,v*np.cos(beta),v*np.sin(beta),r,rdot,at,np.full(n,k),lap_length=lap)
        np.testing.assert_allclose(out['beta_dot_rad_s'],beta_dot,atol=1e-6)
        self.assertLess(out['raw']['course_rate_minus_Vkappa']['rms'],1e-6)
        self.assertLess(out['raw']['yaw_acceleration_minus_path_demand']['rms'],.009)

    def test_wrapped_station_unwrap_and_nonuniform_periodic_derivative(self):
        # Lift a seam-crossing trace to an unwrapped coordinate, then derivative
        # of periodic sin(2πs/L) on a nonuniform mesh should remain accurate.
        period=12.; wrapped=np.array([9.,10.,11.,0.2,1.5,3.]); unwrapped=np.unwrap(wrapped*2*np.pi/period)*period/(2*np.pi)
        self.assertTrue(np.all(np.diff(unwrapped)>0))
        s=np.array([0.,.7,2.1,4.,6.8,9.2]); values=np.sin(2*np.pi*s/period)
        d=_periodic_nonuniform_derivative(values,s,period)
        exact=2*np.pi/period*np.cos(2*np.pi*s/period)
        self.assertLess(np.max(np.abs(d-exact)),.15)

    def test_tangential_acceleration_projects_body_frame(self):
        vx=np.array([10*np.cos(.4)]); vy=np.array([10*np.sin(.4)])
        at=tangential_acceleration_from_body(vx,vy,[2*np.cos(.4)-3*np.sin(.4)],[2*np.sin(.4)+3*np.cos(.4)])
        np.testing.assert_allclose(at,[2.])

    def test_yaw_acceleration_withheld_without_tangential_acceleration(self):
        t=np.arange(8)*.1; s=np.arange(8)*.2
        result=yaw_approximation_diagnostics(t,s,np.ones(8)*2,np.zeros(8),np.zeros(8),np.zeros(8),None,np.zeros(8))
        self.assertEqual(result['yaw_accel_residual_status'],'unknown_tangential_acceleration_unavailable')
        self.assertNotIn('yaw_acceleration_minus_path_demand',result['raw'])

    def test_identity_mismatch_inconclusive(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); (root/'summary.json').write_text(json.dumps({'lap_completed':True,'solver_validation_complete':True}))
            (root/'inputs.json').write_text(json.dumps({'args':{'track':'wrong','plan':'other.npz'},'map_identity':'wrong','study_contract':{'provenance':{'plan_array_digest':'wrong','source_revision':'unknown'}}}))
            row=replay_info(root,'plan.npz',expected_track='expected',expected_map='map',plan_digest='digest')
            self.assertTrue(row['exploratory_completion'])
            self.assertEqual(row['comparison_status'],'inconclusive')
            self.assertEqual(row['identity_checks']['status'],'inconclusive_or_mismatch')

    def test_known_source_revision_mismatch_is_inconclusive(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); (root/'summary.json').write_text(json.dumps({'lap_completed':True,'solver_validation_complete':True}))
            (root/'inputs.json').write_text(json.dumps({'args':{'track':'track','plan':'plan.npz'},'map_identity':'map','study_contract':{'provenance':{'plan_array_digest':'digest','plan_file_digest':'placeholder','source_revision':'replay-r2','source_dirty_hashes':{}}}}))
            row=replay_info(root,'plan.npz',expected_track='track',expected_map='map',plan_digest='digest',plan_source_revision='plan-r1',plan_dirty_hashes={})
            self.assertTrue(row['exploratory_completion'])
            self.assertFalse(row['identity_checks']['source_revision_match'])
            self.assertEqual(row['comparison_status'],'inconclusive')

    def test_unknown_plan_source_provenance_is_inconclusive(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); (root/'summary.json').write_text(json.dumps({'lap_completed':True,'solver_validation_complete':True}))
            (root/'inputs.json').write_text(json.dumps({'args':{'track':'track','plan':'plan.npz'},'map_identity':'map','study_contract':{'provenance':{'plan_array_digest':'digest','plan_file_digest':'placeholder','source_revision':'r1','source_dirty_hashes':{}}}}))
            row=replay_info(root,'plan.npz',expected_track='track',expected_map='map',plan_digest='digest',plan_source_revision=None,plan_dirty_hashes=None)
            self.assertTrue(row['exploratory_completion'])
            self.assertFalse(row['identity_checks']['plan_source_revision_known'])
            self.assertEqual(row['comparison_status'],'inconclusive')

    def test_missing_evidence_is_inconclusive_unknown(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'missing'; row=replay_info(p,'plan.npz')
            self.assertEqual(row['comparison_status'],'inconclusive')
            self.assertEqual(row['yaw_approximation_audit']['status'],'unknown')

if __name__=='__main__':unittest.main()
