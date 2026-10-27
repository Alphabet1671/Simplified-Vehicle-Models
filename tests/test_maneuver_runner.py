import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tools.maneuver_gui.artifacts import load_manifest, write_artifacts
from tools.maneuver_gui.runner import run_child, run_definition
from simplified_vehicle_models.controls.maneuvers.schema import validate_definition


def definition():
    return validate_definition({
        'schema':'maneuver-v1','name':'simple','preset':'step_steer','backend':'simulation',
        'vehicle':{'model':'not-used'},'initial':{'speed_mps':10.,'x_m':0.,'y_m':0.,'heading_rad':0.},
        'clock':{'controller_tick_s':.01},'steering':{'mode':'peak_swa','amplitude_rad':0.1},
        'longitudinal':{'mode':'constant_accelerator','accelerator':0.1},
        'profile':{'duration_s':.02},'limits':{},'termination':{'duration_s':.02},
        'calibration':{'maximum_swa_rad':8.,'maximum_swa_rate_rad_s':2.,'rack_travel_per_revolution_m':.04,'steer_arm_length_m':.10,'maximum_road_wheel_angle_rad':.6,'ackermann_inner_angle_rad':[0.,.6],'ackermann_outer_offset_rad':[0.,.1],'accelerator_travel':[0.,1.],'accelerator_force_n':[0.,10.5],'brake_travel':[0.,1.],'brake_force_n':[0.,500.]},'csv':None})


class ArtifactTests(unittest.TestCase):
    def result(self):
        return SimpleNamespace(time_s=np.array([0.,.01]), state=np.zeros((2,2)), state_rate=np.ones((2,2)),
            events=(), applied_commands=(), execution_outcome='completed', termination_reason='duration')

    def test_manifest_definition_calibration_model_and_summary_consistency(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td)/'attempt'; d.mkdir()
            write_artifacts(d, definition(), self.result(), {'execution_outcome':'completed'})
            manifest=json.loads((d/'manifest.json').read_text())
            manifest['model']={'tampered':True}
            (d/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaises(ValueError): load_manifest(d)

    def test_safe_generated_names_atomic_npz_and_manifest_integrity(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td)/'attempt'; d.mkdir()
            write_artifacts(d, definition(), self.result(), {'input_fidelity':'exact'})
            self.assertFalse((Path(td)/'outside').exists())
            self.assertEqual(load_manifest(d)['schema'],'maneuver-artifacts-v1')
            with np.load(d/'run_record.npz',allow_pickle=False) as data:
                np.testing.assert_array_equal(data['time_s'],[0.,.01])
            with self.assertRaises(FileExistsError): write_artifacts(d, definition(), self.result(), {})
            (d/'run_record.npz').write_bytes(b'corrupt')
            with self.assertRaises(ValueError): load_manifest(d)

    def test_arbitrary_formulation_path_rejected(self):
        from tools.maneuver_gui.runner import _allowed_formulation
        with self.assertRaises(ValueError):
            _allowed_formulation("/tmp/attacker.xml")
        with self.assertRaises(ValueError):
            _allowed_formulation("models/../../tmp/attacker.xml")

    def test_unsuccessful_child_stage_is_removed_and_success_publish_is_directory_atomic(self):
        import os
        import subprocess
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); source=root/'definition.json'
            payload=definition()
            payload['vehicle']={'model':'models/formulations/iem24_murphy_real_planar_7dof.xml'}
            source.write_text(json.dumps(payload))
            observed={}
            def fake_child(command, **kwargs):
                stage=Path(command[-1]); observed['stage']=stage
                # A failed child may have partially written files; parent discards all.
                (stage/'partial').write_text('incomplete')
                return 7
            with patch('tools.maneuver_gui.runner.run_child', side_effect=fake_child):
                attempt=run_definition(source, root/'out')
            self.assertEqual(json.loads((attempt/'summary.json').read_text())['execution_outcome'],'failed')
            self.assertFalse((attempt/'result').exists())
            self.assertFalse(observed['stage'].exists())
            self.assertEqual(list((root/'out').glob('*.result-stage')), [])

    def test_successful_result_directory_published_as_single_entry(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); source=root/'definition.json'
            payload=definition()
            payload['vehicle']={'model':'models/formulations/iem24_murphy_real_planar_7dof.xml'}
            source.write_text(json.dumps(payload))
            observed={}
            def fake_child(command, **kwargs):
                stage=Path(command[-1]); observed['stage']=stage
                d=validate_definition(json.loads(Path(command[-2]).read_text()))
                from simplified_vehicle_models.solvers.maneuver_planar import ManeuverPlanarPlant
                n=len(ManeuverPlanarPlant.state_names)
                result=SimpleNamespace(time_s=np.array([0.,.01]),state=np.zeros((2,n)),state_rate=np.ones((2,n)),
                    events=(),applied_commands=(SimpleNamespace(swa_rad=0.,accelerator=0.,brake=0.),)*2,
                    channels=({},{}),execution_outcome='completed',termination_reason='duration')
                write_artifacts(stage,d,result,{'execution_outcome':'completed'},model_kind='planar-7dof')
                return 0
            with patch('tools.maneuver_gui.runner.run_child', side_effect=fake_child):
                attempt=run_definition(source, root/'out')
            self.assertTrue((attempt/'result'/'manifest.json').is_file(), (attempt, list(attempt.iterdir()), (attempt/'summary.json').read_text()))
            summary=json.loads((attempt/'summary.json').read_text())
            self.assertEqual(summary['execution_outcome'],'completed')
            self.assertEqual(Path(summary['run_record']), (attempt/'run_record.npz').resolve())
            from simplified_vehicle_models.results.run_record import load_run_record
            self.assertEqual(load_run_record(attempt/'run_record.npz', verify_sources=False).manifest.schema, 'run-record-v1')
            self.assertFalse(observed['stage'].exists())

    def test_post_publish_exception_removes_result_and_keeps_failure_summary(self):
        from unittest.mock import patch
        import os
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); source=root/'definition.json'
            payload=definition()
            payload['vehicle']={'model':'models/formulations/iem24_murphy_real_planar_7dof.xml'}
            source.write_text(json.dumps(payload))
            observed={}
            def fake_child(command, **kwargs):
                stage=Path(command[-1]); observed['stage']=stage
                d=validate_definition(json.loads(Path(command[-2]).read_text()))
                from simplified_vehicle_models.solvers.maneuver_planar import ManeuverPlanarPlant
                n=len(ManeuverPlanarPlant.state_names)
                result=SimpleNamespace(time_s=np.array([0.,.01]),state=np.zeros((2,n)),state_rate=np.ones((2,n)),
                    events=(),applied_commands=(SimpleNamespace(swa_rad=0.,accelerator=0.,brake=0.),)*2,
                    channels=({},{}),execution_outcome='completed',termination_reason='duration')
                write_artifacts(stage,d,result,{'execution_outcome':'completed'},model_kind='planar-7dof')
                return 0
            real_replace=os.replace
            failed_once={'value':False}
            def fail_summary_replace(src, dst):
                if Path(dst).name == 'summary.json' and not failed_once['value']:
                    failed_once['value']=True
                    raise OSError('forced summary publish failure')
                return real_replace(src, dst)
            with patch('tools.maneuver_gui.runner.run_child', side_effect=fake_child), \
                 patch('tools.maneuver_gui.runner.os.replace', side_effect=fail_summary_replace):
                attempt=run_definition(source, root/'out')
            summary=json.loads((attempt/'summary.json').read_text())
            self.assertEqual(summary['execution_outcome'],'failed')
            self.assertIn('forced summary publish failure',summary['error'])
            self.assertFalse((attempt/'result').exists())
            self.assertFalse(observed['stage'].exists())
            self.assertEqual(list((root/'out').glob('*.result-stage')), [])

    def test_definition_invalid_rejected_without_child_physics(self):
        with tempfile.TemporaryDirectory() as td:
            source=Path(td)/'input.json'
            payload=definition(); payload['preset']='invalid'
            original=json.dumps(payload)
            source.write_text(original)
            attempt=run_definition(source,Path(td)/'out')
            self.assertEqual(source.read_text(),original)
            summary=json.loads((attempt/'summary.json').read_text())
            self.assertEqual(summary['execution_outcome'],'failed')
            self.assertFalse((attempt/'run_record.npz').exists())

    def test_attempt_directories_are_unique_and_help_cli(self):
        import subprocess
        import sys
        from tools.maneuver_gui.artifacts import create_attempt_directory
        with tempfile.TemporaryDirectory() as td:
            self.assertNotEqual(create_attempt_directory(Path(td)),create_attempt_directory(Path(td)))
        result = subprocess.run([sys.executable, '-m', 'tools.maneuver_gui.runner', '--help'],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--definition', result.stdout)

    def test_run_child_forwards_progress_and_cancels_process_group(self):
        import os
        import signal
        import sys
        import time
        events = []
        with tempfile.TemporaryDirectory() as td:
            pidfile = Path(td) / 'grandchild.pid'
            child = (
                'import subprocess,sys,time; '
                'p=subprocess.Popen([sys.executable,"-c","import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)"]); '
                'open(sys.argv[1],"w").write(str(p.pid)); '
                'print(\'{"event":"progress","time_s":0.1,"fraction":0.2,"phase":"test"}\',flush=True); '
                'time.sleep(60)'
            )
            started = time.monotonic()
            def cancel():
                return events and time.monotonic() - started > 0.2
            status = run_child([sys.executable, '-c', child, str(pidfile)],
                               on_progress=events.append, cancel=cancel,
                               terminate_timeout_s=0.2)
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0]['phase'], 'test')
            self.assertLess(time.monotonic() - started, 3.0)
            grandchild = int(pidfile.read_text())
            # A killed process may briefly remain as a zombie; it must not execute.
            with self.assertRaises(ProcessLookupError):
                os.kill(grandchild, 0)



if __name__ == '__main__': unittest.main()
