import unittest
import tempfile
import json
from pathlib import Path
import numpy as np
from simplified_vehicle_models.planning.yaw_replay_metrics import yaw_approximation_diagnostics
from tools.reports.compare_yaw_planning_replay import plan_info, replay_info

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

    def test_missing_evidence_is_inconclusive_unknown(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'missing'; row=replay_info(p,'plan.npz')
            self.assertEqual(row['comparison_status'],'inconclusive')
            self.assertEqual(row['yaw_approximation_audit']['status'],'unknown')

if __name__=='__main__':unittest.main()
