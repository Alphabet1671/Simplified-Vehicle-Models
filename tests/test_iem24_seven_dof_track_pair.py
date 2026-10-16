"""7DoF paired-track MIS report: provenance gates, channel reconstruction, alignment."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.reports import report_iem24_seven_dof_track_pair as report
from tools.reports.report_iem24_seven_dof_track_pair import ReportBlocker

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {'planner': 'live', 'maximum_speed': 60., 'lateral_acceleration': None,
            'pedal_travel_time': .175, 'maximum_step': .02, 'rtol': 1e-5, 'method': 'BDF',
            'spacing': .5, 'duration': 300.}


def make_attempt(directory: Path, track: str, *, settings=None, with_inputs=True) -> Path:
    """A synthetic attempt directory carrying real model-file hashes and snapshots."""
    from simplified_vehicle_models.tracks.profile import load_track
    directory.mkdir(parents=True)
    csv = str(Path(track).with_suffix('.csv'))
    inputs = {'track_xml': track, 'track_csv': csv,
              'tire_xml': 'models/tires/iem24_murphy_real_unitire.xml',
              'formulation_xml': report.FORMULATION,
              'vehicle_xml': 'models/vehicles/iem24_murphy_real_planar.xml'}
    hashes = {path: report.sha256_file(ROOT / path) for path in inputs.values()}
    hashes['src/code.py'] = 'ab' * 32
    snapshot = directory / 'source_snapshot'
    snapshot.mkdir()
    for path in inputs.values():
        shutil.copy(ROOT / path, snapshot / Path(path).name)
    (snapshot / 'code.py').write_text('x')
    hashes['src/code.py'] = report.sha256_file(snapshot / 'code.py')
    summary = {'status': 'Completed', 'lap_completed': True, 'usable': False,
               'method': 'transient_forward_driver', 'minimum_time_optimum': False,
               'settings': {**(settings or SETTINGS), 'track': track},
               'source_hashes': hashes, 'track_length': load_track(ROOT / track).lap_length}
    if with_inputs:
        summary['model_inputs'] = inputs
    (directory / 'summary.json').write_text(json.dumps(summary))
    return directory


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)

    def pair(self, **kwargs):
        old = report.verify_attempt(make_attempt(self.tmp / 'old', report.LEGACY_TRACK), 'original', report.LEGACY_TRACK)
        new = report.verify_attempt(make_attempt(self.tmp / 'new', report.CANONICAL_TRACK, **kwargs),
                                    'new', report.CANONICAL_TRACK)
        return old, new

    def test_matching_pair_passes_and_hashes_differ_for_tracks_only(self):
        old, new = self.pair()
        self.assertEqual(report.verify_pair(old, new), {'code_hash_differences': []})
        self.assertNotEqual(old['inputs']['track_xml']['sha256'], new['inputs']['track_xml']['sha256'])
        self.assertEqual(old['inputs']['tire_xml'], new['inputs']['tire_xml'])

    def test_historical_attempt_without_model_inputs_is_rejected(self):
        directory = make_attempt(self.tmp / 'old', report.LEGACY_TRACK, with_inputs=False)
        with self.assertRaisesRegex(ReportBlocker, 'model_inputs'):
            report.verify_attempt(directory, 'original', report.LEGACY_TRACK)

    def test_wrong_track_for_role_is_rejected(self):
        directory = make_attempt(self.tmp / 'new', report.CANONICAL_TRACK)
        with self.assertRaisesRegex(ReportBlocker, 'expected track'):
            report.verify_attempt(directory, 'original', report.LEGACY_TRACK)

    def test_tampered_snapshot_is_rejected(self):
        directory = make_attempt(self.tmp / 'old', report.LEGACY_TRACK)
        (directory / 'source_snapshot' / 'iem24_murphy_real_unitire.xml').write_text('<changed/>')
        with self.assertRaisesRegex(ReportBlocker, 'snapshot'):
            report.verify_attempt(directory, 'original', report.LEGACY_TRACK)

    def test_recorded_hash_that_differs_from_file_on_disk_is_rejected(self):
        directory = make_attempt(self.tmp / 'old', report.LEGACY_TRACK)
        path = directory / 'summary.json'
        summary = json.loads(path.read_text())
        summary['source_hashes'][report.LEGACY_TRACK] = '0' * 64
        path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ReportBlocker, 'differs from the file'):
            report.verify_attempt(directory, 'original', report.LEGACY_TRACK)

    def test_mismatched_tire_between_attempts_is_rejected(self):
        old, new = self.pair()
        new['inputs']['tire_xml'] = {**new['inputs']['tire_xml'], 'sha256': '1' * 64}
        with self.assertRaisesRegex(ReportBlocker, 'mismatched tire_xml'):
            report.verify_pair(old, new)

    def test_identical_track_contents_are_not_a_pair(self):
        old, new = self.pair()
        new['inputs']['track_xml'] = old['inputs']['track_xml']
        with self.assertRaisesRegex(ReportBlocker, 'identical track_xml'):
            report.verify_pair(old, new)

    def test_solver_setting_mismatch_is_rejected(self):
        old, new = self.pair(settings={**SETTINGS, 'maximum_speed': 20.})
        with self.assertRaisesRegex(ReportBlocker, 'maximum_speed'):
            report.verify_pair(old, new)

    def test_incomplete_attempt_is_rejected(self):
        directory = make_attempt(self.tmp / 'old', report.LEGACY_TRACK)
        path = directory / 'summary.json'
        summary = json.loads(path.read_text())
        summary['lap_completed'] = False
        path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ReportBlocker, 'completed full 7DoF lap'):
            report.verify_attempt(directory, 'original', report.LEGACY_TRACK)


class ChannelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        cls.vehicle = compile_formulation(str(ROOT / report.FORMULATION)).model

    def synthetic(self, n=200, power_kw=0.):
        time = np.linspace(0., 20., n)
        soc0 = float(self.vehicle.storage.initial_state_of_charge)
        table = {'distance_m': np.linspace(0., 300., n), 'elapsed_time_s': time,
                 'velocity_mps': np.full(n, 15.), 'longitudinal_acceleration_mps2': np.zeros(n),
                 'lateral_acceleration_mps2': np.zeros(n), 'rl_power_kw': np.full(n, power_kw),
                 'rr_power_kw': np.full(n, power_kw), 'apps_fraction': np.linspace(0., 1., n)}
        trace = {'time': time, 's': table['distance_m'].copy(), 'yaw_rate': np.linspace(-.5, .5, n),
                 'soc': np.full(n, soc0)}
        return table, trace

    def test_zero_power_voltage_is_open_circuit_and_torque_is_pedal_times_limit(self):
        table, trace = self.synthetic()
        data, check = report.build_model_channels(table, trace, self.vehicle)
        self.assertTrue(np.all(data['bus_current_a'] == 0.))
        np.testing.assert_allclose(data['motor_torque_request_nm'], table['apps_fraction'] * 10.5)
        np.testing.assert_array_equal(data['yaw_rate_rad_s'], trace['yaw_rate'])
        self.assertAlmostEqual(float(data['pack_voltage_v'][0]), 575.7, delta=.5)

    def test_power_draw_lowers_loaded_voltage_below_open_circuit(self):
        soc = np.full(3, float(self.vehicle.storage.initial_state_of_charge))
        voltage, current = report.reconstruct_pack_voltage(self.vehicle.storage, soc, np.array([0., 10e3, 20e3]))
        self.assertTrue(np.all(np.diff(voltage) < 0) and np.all(np.diff(current) > 0))
        np.testing.assert_allclose(voltage[1:] * current[1:], [10e3, 20e3], rtol=1e-9)

    def test_infeasible_power_is_a_blocker(self):
        with self.assertRaisesRegex(ReportBlocker, 'infeasible'):
            report.reconstruct_pack_voltage(self.vehicle.storage, np.array([.8]), np.array([5e6]))

    def test_soc_that_ignores_the_drawn_current_is_a_blocker(self):
        table, trace = self.synthetic(power_kw=10.)  # constant SOC cannot match a 20 kW draw
        with self.assertRaisesRegex(ReportBlocker, 'replay is insufficient'):
            report.build_model_channels(table, trace, self.vehicle)

    def test_export_rows_must_align_with_trace(self):
        table, trace = self.synthetic()
        trace['time'] = trace['time'] + .1
        with self.assertRaisesRegex(ReportBlocker, 'line up'):
            report.build_model_channels(table, trace, self.vehicle)

    def test_time_domain_filter_does_not_change_distance_or_clock(self):
        t = np.linspace(0, 10, 101)
        sample = {'elapsed_time_s': t, 'distance_m': 2 * t, 'speed_mps': 5 + np.sin(2 * np.pi * .25 * t)}
        result = report.filter_model(sample, ('speed_mps',))
        np.testing.assert_array_equal(result['distance_m'], sample['distance_m'])
        np.testing.assert_array_equal(result['elapsed_time_s'], t)
        self.assertLess(np.max(np.abs(result['speed_mps'] - sample['speed_mps'])), 1.)


class AlignmentTests(unittest.TestCase):
    def test_canonical_nodes_map_monotonically_onto_original_distance(self):
        from simplified_vehicle_models.tracks.profile import load_track
        old = load_track(ROOT / report.LEGACY_TRACK).path_coordinates
        new = load_track(ROOT / report.CANONICAL_TRACK).path_coordinates
        mapped, gap = report.map_to_reference_distance(old, new, new.distance)
        self.assertTrue(np.all(np.diff(mapped) >= -1e-9))
        self.assertAlmostEqual(float(mapped[0]), 0.)
        self.assertAlmostEqual(float(mapped[-1]), float(old.distance[-1]), delta=5.)
        self.assertLess(gap['p95_xy_gap_m'], 5.)
        identity, _ = report.map_to_reference_distance(old, old, old.distance)
        np.testing.assert_allclose(identity, old.distance, atol=1e-9)

    def test_track_pair_has_different_hashes_and_lengths(self):
        from simplified_vehicle_models.tracks.profile import load_track
        a, b = (load_track(ROOT / t) for t in (report.LEGACY_TRACK, report.CANONICAL_TRACK))
        self.assertGreater(abs(a.lap_length - b.lap_length), 1.)
        self.assertNotEqual(report.sha256_file(ROOT / report.LEGACY_TRACK),
                            report.sha256_file(ROOT / report.CANONICAL_TRACK))


if __name__ == '__main__':
    unittest.main()
