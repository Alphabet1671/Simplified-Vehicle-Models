"""Protect single-replay scope and numerical report claims."""
import json
from pathlib import Path
import unittest
import numpy as np
from tools.reports.compare_yaw_mis_replay import metric

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
        self.assertNotIn(r'\n\n',report)
        for field in ('requested_motor_torque_per_motor_nm','delivered_rear_wheel_torque_nm','electrical_bus_power_w','throttle_saturation','brake_saturation','steering_rate_saturation'):
            self.assertIn(field,summary['operational_traces'])
        self.assertTrue((ROOT/'docs/validation/iem24-murphy-yaw-closure/operational_traces.csv').is_file())
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
