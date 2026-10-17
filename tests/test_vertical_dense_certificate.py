import unittest
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.vertical_dense_certificate import certify_vertical_dense_domain

class VerticalDenseCertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation('models/formulations/iem24_murphy_real_14dof_simple.xml').model
        d = np.linspace(0., 200., 401)
        cls.plan = SpeedPlan(d, np.full(401, 10.), np.full(401, .05))
    def bounds(self, n=2):
        x = np.zeros(38); x[0]=50.; x[3]=15.; x[12:16]=50.; x[24]=.8
        b=np.stack([x-1e-3,x+1e-3],axis=-1); return np.repeat(b[None],n,axis=0)
    def test_passes_and_reports_scope(self):
        r=certify_vertical_dense_domain(self.vehicle,self.plan,self.bounds())
        self.assertTrue(r['certified']); self.assertFalse(r['full_between_step_certificate'])
    def test_physical_jounce_limit_rejects(self):
        b=self.bounds(); b[0,16,1]=self.vehicle.suspension.front.jounce_limits[1]+1e-3
        r=certify_vertical_dense_domain(self.vehicle,self.plan,b)
        self.assertFalse(r['certified']); self.assertIn('jounce_upper_margin',r['failed'])
    def test_nan_and_shape_fail(self):
        b=self.bounds(); b[0,3,0]=np.nan
        self.assertFalse(certify_vertical_dense_domain(self.vehicle,self.plan,b)['certified'])
        self.assertFalse(certify_vertical_dense_domain(self.vehicle,self.plan,b[:,:,:1])['certified'])
    def test_path_checks(self):
        b=self.bounds(); b[0,1,1]=20.
        r=certify_vertical_dense_domain(self.vehicle,self.plan,b)
        self.assertFalse(r['certified']); self.assertIn('path_denominator_over_minimum',r['failed'])

if __name__ == '__main__': unittest.main()
