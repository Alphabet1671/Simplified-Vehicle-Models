import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.execute import PreparedCase, child_command, finish_case
from tools.sim_gui.run_yaw_case import _point_mass, _yaw
from tools.sim_gui.settings import RunSettings
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters


class CommandAndStatusTests(unittest.TestCase):
    def test_child_always_invokes_adapter_with_case_and_attempt(self):
        case = Path('/tmp/case')
        attempt = Path('/tmp/attempt')
        command = child_command(PreparedCase(case, case/'f.xml', case/'v.xml', [], solver_kind='point-mass'), attempt)
        self.assertEqual(command[-4:], ['--case-dir', str(case), '--child-attempt', str(attempt)])
        self.assertTrue(command[2].endswith('tools/sim_gui/run_yaw_case.py'))
        self.assertNotIn('run_yaw_lap_suite.py', command)
        self.assertNotIn('run_iem24_forward_driver.py', command)

    def test_finish_requires_typed_status_and_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            case = Path(temp)/'study'/'cases'/'c'
            attempt = case/'attempt_1'
            attempt.mkdir(parents=True)
            prepared = PreparedCase(case, case/'f', case/'v', [], solver_kind='yaw-aware')
            kwargs = dict(return_code=0, timed_out=False, cancelled=False, wall_seconds=1., command=[])
            (attempt/'summary.json').write_text(json.dumps({'status':'planned','lap_completed':False}))
            (attempt/'inputs.json').write_text('{}')
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')
            (attempt/'plan.npz').touch()
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'planned')
            (attempt/'summary.json').write_text(json.dumps({'status':'completed','lap_completed':True,'solver_validation_complete':True}))
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')
            (attempt/'run_record.npz').touch()
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'completed')
            (attempt/'summary.json').write_text('{')
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')

    def test_missing_summary_and_timeout_are_not_completed(self):
        with tempfile.TemporaryDirectory() as temp:
            case = Path(temp)/'study'/'cases'/'c'; attempt = case/'attempt'; attempt.mkdir(parents=True)
            prepared = PreparedCase(case, case/'f', case/'v', [])
            common = dict(return_code=0, cancelled=False, wall_seconds=1., command=[])
            self.assertEqual(finish_case(prepared, attempt, timed_out=False, **common)['status'], 'failed')
            self.assertEqual(finish_case(prepared, attempt, timed_out=True, **common)['status'], 'timed_out')


class PreparationTests(unittest.TestCase):
    def test_yaw_binds_variant_and_arguments(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); model=root/'model'; model.mkdir(); form=model/'formulation.xml'; form.touch()
            case_dir=root/'case'; (case_dir/'model').mkdir(parents=True)
            attempt=root/'attempt'; attempt.mkdir()
            form=case_dir/'model'/'formulation.xml'; form.touch()
            track=root/'track.xml'; track.touch(); yaw_map=root/'map.npz'; yaw_map.touch()
            case={'model':'planar-7dof','track':str(REPOSITORY_ROOT / 'models/tracks/2024_michigan_endurance_closed.xml'),'formulation':'model/formulation.xml',
                  'argv':['--yaw-map',str(yaw_map),'--track',str(track),'--mode','fixed']}
            yaw_map=REPOSITORY_ROOT / 'runs/yaw_aware_driver/mis-coupled-plan-smoke/yaw_map.npz'
            settings=RunSettings(yaw_map=str(yaw_map))
            with mock.patch('tools.sim_gui.run_yaw_case.validate'), mock.patch('tools.sim_gui.settings._validate_map'), \
                 mock.patch('simplified_vehicle_models.compiler.compile.compile_formulation', return_value=mock.Mock(model=object.__new__(PlanarVehicleParameters))), \
                 mock.patch('tools.run.run_yaw_lap_suite.parse_args', side_effect=lambda argv: argv) as parse, \
                 mock.patch('tools.run.run_yaw_lap_suite.run', return_value=0):
                self.assertEqual(_yaw(case, root/'attempt', settings, case_dir), 0)
            argv=parse.call_args.args[0]
            self.assertEqual(argv[argv.index('--formulation')+1], str(form.resolve()))
            self.assertEqual(argv[argv.index('--yaw-map')+1], str(yaw_map.resolve()))
            self.assertEqual(argv[argv.index('--track')+1], str((REPOSITORY_ROOT / 'models/tracks/2024_michigan_endurance_closed.xml').resolve()))
            self.assertNotIn('--planner', argv)
            self.assertNotIn('run_iem24_forward_driver', argv)

    def test_point_mass_boundary_rejects_open_track_before_solver(self):
        settings=RunSettings(solver_kind='point-mass')
        with mock.patch('tools.sim_gui.run_yaw_case._resolve', return_value=REPOSITORY_ROOT / 'models/tracks/2024_michigan_endurance_closed.xml'), \
             mock.patch('simplified_vehicle_models.tracks.profile.load_track', return_value=mock.Mock(is_closed=False)):
            with self.assertRaisesRegex(ValueError,'closed tracks only'):
                _point_mass({'track':'track.xml','model':'planar-7dof','formulation':'model/formulation.xml'},
                            Path('/tmp/attempt'),settings,Path('/tmp/case'))


if __name__ == '__main__':
    unittest.main()
