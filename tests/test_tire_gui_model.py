from pathlib import Path
import unittest

import numpy as np

try:
    import scipy.io  # noqa: F401
except ImportError as exc:
    raise unittest.SkipTest(f'tire GUI model tests require optional scipy: {exc}')

from tools.tire_gui.data import normalize_channels
from tools.tire_gui.parameters import MAT_SCALAR_FIELDS, MAT_VECTOR_FIELDS, VECTOR_DEGREES, flatten_steady, unflatten_steady
from tools.tire_gui.model import import_mat_seed, load_snapshot, predict_steady, with_parameter, load_transient_snapshot
from tools.tire_gui.contracts import SampleSet
from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire, loaded_radius

STEADY = Path('models/tires/hoosier-43075-r20-simple.xml')
TRANSIENT = Path('models/tires/iem24_murphy_real_unitire_transient.xml')


class ModelTests(unittest.TestCase):
    def test_edit_is_immutable_and_transient_stales(self):
        old = load_snapshot(TRANSIENT)
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
        precise = dict(snap.values)
        vector_names = {'k_x_2': 7660.409078173157, 'k_x_1': 20000.000000000004, 'k_x_0': 8191.007990757679,
                        'k_y_2': 654.9807055362453, 'k_y_1': 19999.99886208581, 'k_y_0': 10000.000000000002,
                        'd_x_0_1': .017161153667128679, 'd_x_0_0': .010864401757777131,
                        'd_e_1': .000204667577986679, 'd_e_0': -.000370467684249365,
                        'd_1_1': .007939258612776069, 'd_1_0': .18306301414726702,
                        'd_2_1': .06468408760663875, 'd_2_0': .03685184791816076}
        precise.update(vector_names)
        round_tripped = unflatten_steady(precise, snap.steady)
        flattened = flatten_steady(round_tripped)
        for name, value in vector_names.items(): self.assertEqual(flattened[name], value)
        samples = SampleSet(np.array([.02, -.03]), np.array([.01, 0]), np.array([500., 1100.]), np.zeros(2), np.zeros(2, dtype=int))
        pred = predict_steady(snap, samples, 40.193 / 3.6)
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
            values = load_snapshot(STEADY).values
            source_scalars = {source: values[canonical] for source, canonical in MAT_SCALAR_FIELDS.items()}
            source_vectors = {}
            vector_coefficients = {
                'k_x': [7660.409078173157, 20000.000000000004, 8191.007990757679],
                'k_y': [654.9807055362453, 19999.99886208581, 10000.000000000002],
                'd_x_0': [.017161153667128679, .010864401757777131],
                'd_e': [.000204667577986679, -.000370467684249365],
                'd_1': [.007939258612776069, .18306301414726702],
                'd_2': [.06468408760663875, .03685184791816076],
            }
            for canonical, coefficients in vector_coefficients.items():
                source = next(key for key, value in MAT_VECTOR_FIELDS.items() if value == canonical)
                source_vectors[source] = np.asarray(coefficients, dtype=np.float64)
            tire.update(source_scalars); tire.update(source_vectors)
            savemat(mat_path, {'tire': tire, 'fitData': {'ignored': np.arange(3)}})
            imported = import_mat_seed(mat_path, load_snapshot(STEADY))
            self.assertEqual(imported.provenance['seed_kind'], 'requested MATLAB fit')
            self.assertNotIn('fitData', imported.values)
            for vector, coefficients in vector_coefficients.items():
                np.testing.assert_array_equal(getattr(imported.steady, vector), coefficients)
                for degree, coefficient in zip(VECTOR_DEGREES[vector], coefficients):
                    self.assertEqual(imported.values[f'{vector}_{degree}'], float(coefficient))

    def test_self_contained_transient_loads_and_replaces_all_embedded_values(self):
        import shutil, tempfile
        from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
        from simplified_vehicle_models.xml.loader import load_model
        with tempfile.TemporaryDirectory() as folder:
            renamed = Path(folder) / 'renamed-anything.xml'
            shutil.copy2(TRANSIENT, renamed)
            snapshot = load_snapshot(renamed)
            self.assertEqual(snapshot.steady, compile_unitire_simple(load_model(renamed)))
            self.assertEqual(snapshot.transient.trail_lead_ratio, .25)
            self.assertEqual(snapshot.provenance['transient_path'], str(renamed.resolve()))
            self.assertEqual(snapshot.parameter_status['trail_lead_ratio'], 'transient XML (provisional)')
            self.assertEqual(snapshot.provenance['transient_fit_status'], 'provisional')

    def test_full_transient_open_replaces_current_steady_and_values(self):
        from dataclasses import replace
        current = with_parameter(load_snapshot(STEADY), 'mu_0_y', 1.23456789)
        current = replace(current, session_metadata={'locks': {'mu_0_y': True}})
        opened = load_transient_snapshot(TRANSIENT, snapshot=current)
        self.assertNotEqual(opened.steady.mu_0_y, current.steady.mu_0_y)
        self.assertEqual(opened.revision, current.revision + 1)

    def test_legacy_attach_helper_rejects_full_xml_pairing(self):
        with self.assertRaisesRegex(ValueError, 'self-contained|embedded|companion'):
            load_transient_snapshot(TRANSIENT, steady_xml=TRANSIENT)

    def test_alias_channel_conversion(self):
        normalized = normalize_channels({'SA': np.array([1.]), 'FZ': np.array([-10.]), 'SL': np.array([.2])}, {'SA':'deg','FZ':'N','SL':'dimensionless'})
        self.assertAlmostEqual(normalized['alpha'][0], -np.pi/180)
        self.assertEqual(normalized['fz'][0], 10)
        self.assertEqual(normalized['kappa'][0], .2)


if __name__ == '__main__':
    unittest.main()
