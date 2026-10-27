from pathlib import Path
import unittest

import numpy as np

from tools.tire_gui.data import normalize_channels
from tools.tire_gui.model import import_mat_seed, load_snapshot, predict_steady, with_parameter
from tools.tire_gui.contracts import SampleSet
from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire, loaded_radius

STEADY = Path('models/tires/hoosier-43075-r20-simple.xml')
TRANSIENT = Path('models/tires/iem24_murphy_real_unitire_transient.xml')


class ModelTests(unittest.TestCase):
    def test_edit_is_immutable_and_transient_stales(self):
        old = load_snapshot(STEADY, TRANSIENT)
        new = with_parameter(old, 'mu_0_y', old.values['mu_0_y'] + .1)
        self.assertEqual(new.revision, old.revision + 1)
        self.assertNotEqual(old.steady.mu_0_y, new.steady.mu_0_y)
        self.assertEqual(old.values['mu_0_y'], old.steady.mu_0_y)
        self.assertEqual(old.provenance['transient_fit_status'], 'provisional')
        self.assertEqual(new.provenance['transient_fit_status'], 'stale')
        with self.assertRaises(TypeError):
            new.values['mu_0_y'] = 1

    def test_bad_value_refused(self):
        old = load_snapshot(STEADY)
        with self.assertRaises(ValueError):
            with_parameter(old, 'trail_lead_ratio', 1.1)

    def test_canonical_vectors_and_prediction_match_evaluator(self):
        snap = load_snapshot(STEADY)
        self.assertEqual(tuple(snap.values[f'k_x_{i}'] for i in (2, 1, 0)), snap.steady.k_x)
        samples = SampleSet(np.array([.02, -.03]), np.array([.01, 0]), np.array([500., 1100.]), np.zeros(2), np.zeros(2, dtype=int))
        pred = predict_steady(snap, samples, 40.193 / 3.6)
        from simplified_vehicle_models.tires.unitire_simple import loaded_radius
        radius = loaded_radius(snap.steady, samples.fz)
        expected_omega = np.abs((1 + samples.kappa) * (40.193 / 3.6) / np.maximum(radius, 1e-4))
        expected = evaluate_unitire(snap.steady, samples.alpha, samples.kappa, samples.fz, expected_omega)
        np.testing.assert_allclose(pred.lateral_force, expected.lateral_force)
        np.testing.assert_allclose(pred.longitudinal_force, expected.longitudinal_force)

    def test_synthetic_mat_import_without_fit_report(self):
        from scipy.io import savemat
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            mat_path = Path(td) / 'seed.mat'
            tire = {}
            # MATLAB exporter field names, all required supported scalars and vectors.
            fields = {'Fz_rated':'f_z_rated','Kcx':'k_cx','Kcy':'k_cy','E1':'e_1','mu0x':'mu_0_x','musx':'mu_s_x','hx':'h_x','vmx':'v_m_x','mu0y':'mu_0_y','musy':'mu_s_y','hy':'h_y','vmy':'v_m_y','K11':'k_11','K12':'k_12','K13':'k_13','K21':'k_21','K22':'k_22','K23':'k_23','MxR1':'m_x_r_1','MxR2':'m_x_r_2','MxR3':'m_x_r_3','R1':'r_1','R2':'r_2','R3':'r_3','Fy_shift0':'f_y_shift_0','Fy_shiftFz':'f_y_shift_fz','KRl0':'k_rl_0','KRl1':'k_rl_1','KRl2':'k_rl_2','frr0':'f_rr_0','frr1':'f_rr_1','frr2':'f_rr_2','hrr':'h_rr','omega_cr':'omega_cr','omega_eps':'omega_eps','vroll_min':'vroll_min','force_eps':'force_eps','stiff_eps':'stiff_eps','slip_eps':'slip_eps'}
            from dataclasses import fields as dataclass_fields
            names = {f.name for f in dataclass_fields(load_snapshot(STEADY).steady)} - {'effective_radius','k_x','k_y','d_x_0','d_e','d_1','d_2'}
            values = load_snapshot(STEADY).values
            reverse = {v:k for k,v in fields.items()}
            for name in names: tire[reverse[name]] = values[name]
            for matlab, canonical in [('Kx','k_x'),('Ky','k_y'),('Dx0','d_x_0'),('De','d_e'),('D1','d_1'),('D2','d_2')]:
                degrees = [2,1,0] if canonical in ('k_x','k_y') else [1,0]
                tire[matlab] = np.array([values[f'{canonical}_{d}'] for d in degrees])
            savemat(mat_path, {'tire': tire, 'fitData': {'ignored': np.arange(3)}})
            imported = import_mat_seed(mat_path, load_snapshot(STEADY))
            self.assertEqual(imported.provenance['seed_kind'], 'requested MATLAB fit')
            self.assertNotIn('fitData', imported.values)

    def test_alias_channel_conversion(self):
        normalized = normalize_channels({'SA': np.array([1.]), 'FZ': np.array([-10.]), 'SL': np.array([.2])}, {'SA':'deg','FZ':'N','SL':'dimensionless'})
        self.assertAlmostEqual(normalized['alpha'][0], -np.pi/180)
        self.assertEqual(normalized['fz'][0], 10)
        self.assertEqual(normalized['kappa'][0], .2)


if __name__ == '__main__':
    unittest.main()
