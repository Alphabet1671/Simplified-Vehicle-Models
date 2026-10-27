"""Protect single-replay scope and numerical report claims."""
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np
from tools.reports.compare_yaw_mis_replay import metric, prepare_full_lap, map_lap_through_plan, cumulative_time_on_reference, paired_sector_gap_table, validate_record_plan, reconstruct_torque_request

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'docs/validation/iem24-murphy-yaw-replay'


class YawMisReportTests(unittest.TestCase):
    def test_metric_signed_bias_and_error(self):
        result=metric(np.array([2.,4.,6.]),np.array([1.,3.,5.]))
        self.assertEqual(result['bias'],1.)
        self.assertEqual(result['rmse'],1.)
        self.assertEqual(result['mae'],1.)

    def test_fresh_closure_has_structured_comparison_and_separate_traces(self):
        summary=json.loads((ROOT/'docs/validation/iem24-murphy-yaw-closure/metrics.json').read_text())
        report=(ROOT/'docs/validation/iem24-murphy-yaw-closure/report.md').read_text()
        self.assertIn('sector_comparison',summary)
        self.assertIn('operational_traces',summary)
        self.assertIn('not run',report)
        self.assertIn('operational_traces.csv',report)
        self.assertIn('Largest positive:',report)
        self.assertIn('300–350 m',report)
        self.assertAlmostEqual(summary['sector_comparison']['measured_boundary_elapsed_s'],77.31001281738281,places=6)
        self.assertAlmostEqual(summary['sector_comparison']['replay_boundary_elapsed_s'],64.80222974543592,places=6)
        self.assertEqual(summary['sector_comparison']['rows'][-1]['end_m'],summary['simulation']['reference_length_m'])
        self.assertEqual(summary['sector_comparison']['mapping'],'measured projected station unwrapped within lap; physical→reference map from saved plan')
        self.assertNotIn(r'\n\n',report)
        for field in ('requested_motor_torque_per_motor_nm','delivered_rear_wheel_torque_nm','electrical_bus_power_w','throttle_saturation','brake_saturation','steering_rate_saturation'):
            self.assertIn(field,summary['operational_traces'])
        self.assertTrue((ROOT/'docs/validation/iem24-murphy-yaw-closure/operational_traces.csv').is_file())

    def test_full_lap_endpoints_are_required_and_duplicate_time_is_retained(self):
        elapsed,station=prepare_full_lap(np.array([10.,11.,13.,14.]),np.array([0.,50.,50.,100.]),100.)
        np.testing.assert_allclose(elapsed,[0.,3.,4.])
        np.testing.assert_allclose(station,[0.,50.,100.])
        declared,declared_s=prepare_full_lap([10.,11.,13.],[0.,50.,99.],100.,boundary_duration=8.)
        self.assertEqual(declared[-1],8.)
        with self.assertRaisesRegex(ValueError,'endpoints unsupported'):
            prepare_full_lap([0.,1.,2.],[0.,50.,98.],100.)
        with self.assertRaisesRegex(ValueError,'endpoints unsupported'):
            prepare_full_lap([0.,1.,2.],[2.,50.,100.],100.)
        # Cross-seam projection may begin at 1008 m and finish at 1032 m on L=24 m.
        lap_time,lap_s=prepare_full_lap([5.,6.,7.],[1008.,1020.,1032.],24.)
        np.testing.assert_allclose(lap_s,[0.,12.,24.])
        self.assertAlmostEqual(lap_time[-1],2.)
        mapped=map_lap_through_plan(lap_s,[0.,12.,24.],[0.,11.,24.],24.)
        np.testing.assert_allclose(mapped,[0.,11.,24.])

    def test_endpoints_derived_locally_without_clamping(self):
        elapsed,station=prepare_full_lap([0.,1.,3.],[1.,51.,99.5],100.,boundary_tolerance=2.)
        self.assertEqual(station[0],0.)
        self.assertEqual(station[-1],100.)
        self.assertGreater(elapsed[-1],elapsed[0])
        with self.assertRaisesRegex(ValueError,'unsupported'):
            prepare_full_lap([0.,1.,2.],[2.,50.,98.],100.,boundary_tolerance=1.)

    def test_sector_table_uses_exact_full_lap_time_and_cross_seam_mapping(self):
        elapsed,station=prepare_full_lap([100.,101.,104.],[2.,52.,1002.],1000.,start_time=100.)
        np.testing.assert_allclose(station,[0.,50.,1000.])
        rows=paired_sector_gap_table([0.,1.,10.],[0.,50.,100.],[0.,1.,4.],[0.,50.,100.],100.)
        self.assertEqual(rows[-1]['end_m'],100.)
        self.assertAlmostEqual(rows[-1]['measured_s'],3.)

    def test_generator_admission_rejects_time_channel_and_provenance_defects(self):
        channels=('speed','acceleration-longitudinal','acceleration-lateral','throttle','bus-voltage','soc','wheel-drive-torque')
        states=('s','yaw_rate')
        manifest=SimpleNamespace(segment={'lap_completed':True},channel_names=channels,channel_units=('',)*len(channels),channel_frames=('',)*len(channels),state_names=states,sources={},model={'yaw_profile':{'array_digest':'a','track_identity':'t','map_identity':'m'},'controller':'ctl'})
        rec=SimpleNamespace(time=np.array([0.,1.,2.]),manifest=manifest,channel_validity=lambda n:np.ones(3,dtype=bool),channel_values=lambda n:np.zeros(3))
        plan={'array_digest':'a','track_identity':'t','map_identity':'m'}
        for bad in (np.array([0.,np.nan,2.]),np.array([0.,1.,1.])):
            rec.time=bad
            with self.assertRaisesRegex(ValueError,'strictly increasing'):
                validate_record_plan(rec,plan)
        rec.time=np.array([0.,1.,2.]);manifest.channel_names=channels[:-1]
        with self.assertRaisesRegex(ValueError,'declarations are incomplete'):
            validate_record_plan(rec,plan)
        manifest.channel_names=channels;manifest.channel_units=('',)
        with self.assertRaisesRegex(ValueError,'declarations are incomplete'):
            validate_record_plan(rec,plan)
        manifest.channel_units=('',)*len(channels);manifest.channel_frames=('',)*len(channels);rec.channel_validity=lambda n:np.array([True,False,True])
        with self.assertRaisesRegex(ValueError,'invalid/missing values'):
            validate_record_plan(rec,plan)
        manifest.model['controller']=''
        with self.assertRaisesRegex(ValueError,'controller identity'):
            validate_record_plan(rec,plan)

    def test_request_reconstruction_changes_with_limit_only(self):
        req_a=reconstruct_torque_request([.2,.8],10.5);req_b=reconstruct_torque_request([.2,.8],8.)
        np.testing.assert_allclose(req_a,[2.1,8.4]);np.testing.assert_allclose(req_b,[1.6,6.4])

    def test_generated_second_duration_and_torque_limit(self):
        from tools.reports.compare_yaw_mis_replay import record_profile
        import tempfile
        record=ROOT/'runs/yaw_lap_suite/mis-modelaware-matchedpreview-validated/run_record.npz'
        plan=ROOT/'runs/yaw_lap_suite/mis-fixed-1m/plan.npz'
        grid=np.linspace(0.,1016.7,500)
        a=record_profile(record,plan,grid,1016.70064959,torque_limit=10.5)
        b=record_profile(record,plan,grid,1016.70064959,torque_limit=8.0)
        self.assertAlmostEqual(float(a[0].time[-1]-a[0].time[0]),64.802231718,places=6)
        np.testing.assert_allclose(a[4]['motor_torque_request_nm'],b[4]['motor_torque_request_nm']*10.5/8.0)
        np.testing.assert_array_equal(a[4]['delivered_wheel_torque_nm'],b[4]['delivered_wheel_torque_nm'])
        self.assertNotAlmostEqual(float(a[0].time[-1]-a[0].time[0]),61.25,places=3)

    def test_report_uses_exact_final_replay_and_all_flying_laps(self):
        report=(OUTPUT/'report.md').read_text()
        summary=json.loads((OUTPUT/'metrics.json').read_text())
        self.assertAlmostEqual(summary['simulation']['lap_time_s'],64.80223171819523,places=9)
        self.assertEqual(summary['measured']['median_lap_time_s'],71.68501281738281)
        self.assertEqual(summary['simulation']['record_sha256'],'38fb555f097b00959f397915937615fc05aa0ac25534ce179474c048bc8bba01')
        self.assertIn('0 (standing start; excluded)',report)
        self.assertIn('10 (flying)',report)
        self.assertIn('not filtered again',report)
        self.assertIn('not delivered wheel torque',report)
        self.assertIn('not an optimum',report)
        self.assertNotIn('FSC',report)

    def test_metric_csv_and_comparison_arrays_match_json(self):
        summary=json.loads((OUTPUT/'metrics.json').read_text())
        data=np.genfromtxt(OUTPUT/'comparison.csv',delimiter=',',names=True)
        simulated=[name for name in data.dtype.names if name.startswith('simulated_')]
        self.assertEqual(len(simulated),6)
        self.assertEqual(len(data),500)
        for name in simulated:
            signal=name[len('simulated_'):]
            actual=metric(data[name],data['measured_'+signal])
            for key,value in actual.items():
                self.assertAlmostEqual(value,summary['metrics'][signal][key],places=10)
