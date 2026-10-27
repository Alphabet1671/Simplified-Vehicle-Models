import json
import tempfile
import unittest
import hashlib
import os
import sys
from pathlib import Path
from unittest import mock

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.execute import PreparedCase, child_command, finish_case
from tools.sim_gui.run_yaw_case import _point_mass, _yaw
from tools.sim_gui.settings import RunSettings
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from tools.sim_gui import run_yaw_case


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
            track=REPOSITORY_ROOT/'models/tracks/2024_michigan_endurance_closed.xml'
            formulation=REPOSITORY_ROOT/'models/formulations/iem24_murphy_real_planar_7dof.xml'
            yaw_map=REPOSITORY_ROOT/'runs/yaw_map_build/iem24-vehicle-limit-final.npz'
            identity={'solver_kind':'yaw-aware','source_model':'planar-7dof','solution_model':'planar-7dof',
                'track':str(track),'track_sha256':hashlib.sha256(track.read_bytes()).hexdigest(),
                'formulation':str(formulation),'formulation_sha256':hashlib.sha256(formulation.read_bytes()).hexdigest(),
                'yaw_map':str(yaw_map),'yaw_map_sha256':hashlib.sha256(yaw_map.read_bytes()).hexdigest()}
            (case/'case.json').write_text(json.dumps({'input_identity':identity}))
            inputs={'solver_kind':'yaw-aware','source_model':'planar-7dof','solution_model':'planar-7dof',
                'track':str(track),'track_sha256':identity['track_sha256'],'formulation':str(formulation),
                'formulation_sha256':identity['formulation_sha256'],'yaw_map':str(yaw_map),'map_sha256':identity['yaw_map_sha256'],
                'map_identity':'id','track_identity':'track'}
            (attempt/'inputs.json').write_text(json.dumps(inputs))
            import numpy as np
            arrays={'distance':np.arange(3.),'curvature':np.zeros(3),'curvature_rate':np.zeros(3),'speed':np.ones(3),
                    'reference_distance':np.arange(3.),'north':np.zeros(3),'east':np.zeros(3),'heading':np.zeros(3)}
            from simplified_vehicle_models.planning.yaw_lap_artifact import _digest, ARRAYS, SCHEMA
            meta={'schema':SCHEMA,'track_identity':'track','map_identity':'id','array_digest':_digest(arrays),
                  'is_closed':False,'profile_settings':{'max_wall_seconds':1.,'max_refinement_rounds':1,'q_seed_count':2,
                  'max_candidates_per_station':2,'max_total_nodes':2,'max_total_edges':2,'max_seam_anchors':1,
                  'interval_q_tolerance':1e-8,'acceleration_tolerance':1e-7,'yaw_moment_tolerance':1e-7,
                  'refinement_time_relative_tolerance':1e-3,'refinement_speed_tolerance':.02,
                  'initial_optional_seed_cap':0,'refinement_reserve_per_station':0,'max_pair_batch':10}}
            plan=attempt/'plan.npz'; np.savez(plan,**arrays,metadata=np.array(json.dumps(meta)))
            digest=hashlib.sha256(plan.read_bytes()).hexdigest(); inputs.update(plan_sha256=digest)
            (attempt/'inputs.json').write_text(json.dumps(inputs))
            (attempt/'summary.json').write_text(json.dumps({'status':'planned','lap_completed':False,'plan_artifact_sha256':digest}))
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'planned')
            plan.write_bytes(b'corrupt'); self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')
            inputs['formulation_sha256']='mismatch'; (attempt/'inputs.json').write_text(json.dumps(inputs))
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')
            (attempt/'summary.json').write_text('{')
            self.assertEqual(finish_case(prepared, attempt, **kwargs)['status'], 'failed')

    def test_main_has_single_parent_watchdog_and_timeout_kills_descendants(self):
        import subprocess, time
        with tempfile.TemporaryDirectory() as temp:
            case=Path(temp)/'case'; case.mkdir(); attempt=Path(temp)/'attempt'; attempt.mkdir()
            (case/'case.json').write_text(json.dumps({'solver_kind':'point-mass','settings':RunSettings(solver_kind='point-mass').to_json()}))
            # Public main must directly call run_case once; the outer process group kills it.
            code=("import sys,time; from pathlib import Path; from unittest import mock; "
                  "from tools.sim_gui import run_yaw_case as r; "
                  "p=mock.patch.object(r,'run_case',side_effect=lambda *a: (Path(a[1]).joinpath('ran').write_text('x'),time.sleep(10))[1]); "
                  "p.start(); r.main(['--case-dir',sys.argv[1],'--child-attempt',sys.argv[2]])")
            proc=subprocess.Popen([sys.executable,'-c',code,str(case),str(attempt)],cwd=REPOSITORY_ROOT,start_new_session=True)
            try:
                deadline=__import__('time').monotonic()+2
                while __import__('time').monotonic()<deadline and not (attempt/'ran').exists(): time.sleep(.02)
                self.assertTrue((attempt/'ran').exists())
                import signal, os
                os.killpg(proc.pid,signal.SIGKILL); proc.wait(timeout=3)
                time.sleep(.1)
                self.assertIsNotNone(proc.poll())
                self.assertFalse(any((attempt/'descendant').exists() for _ in [0]))
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()

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

    def test_suite_compiles_selected_14dof_planar_member_for_map(self):
        """The suite must pass the selected vertical composition into map resolution."""
        from types import SimpleNamespace
        from tools.run import run_yaw_lap_suite as suite
        from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters

        class StopAfterMapInput(RuntimeError):
            pass

        formulation = REPOSITORY_ROOT / 'models/formulations/iem24_murphy_real_14dof_simple.xml'
        planar = mock.Mock(yaw_inertia=17.0, model_type='planar-7dof')
        vertical = object.__new__(VerticalVehicleParameters)
        object.__setattr__(vertical, 'planar', planar)
        compile_mock = mock.Mock(return_value=SimpleNamespace(model=vertical, manifest=object()))
        point_mass = object()
        with tempfile.TemporaryDirectory() as temp:
            args = SimpleNamespace(formulation=formulation, solution_model='14dof-simple', output_dir=Path(temp) / 'out',
                                   yaw_map_ax_min=-1.0, yaw_map_ax_max=1.0)
            with mock.patch('simplified_vehicle_models.compiler.compile.compile_formulation', compile_mock), \
                 mock.patch('simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar', return_value=point_mass) as reduce, \
                 mock.patch('tools.run.run_yaw_aware_forward_driver.physical_map_speed_limit', return_value=10.0), \
                 mock.patch('tools.run.yaw_map_options.resolve_yaw_map', side_effect=StopAfterMapInput) as resolve:
                with self.assertRaises(StopAfterMapInput):
                    suite.run(args)
        self.assertEqual(compile_mock.call_args.args[0], str(formulation.resolve()))
        reduce.assert_called_once_with(planar)
        self.assertIs(resolve.call_args.args[0], point_mass)
        self.assertEqual(resolve.call_args.args[1], 17.0)

    def test_point_mass_boundary_rejects_open_track_before_solver(self):
        settings=RunSettings(solver_kind='point-mass')
        with mock.patch('tools.sim_gui.run_yaw_case._resolve', return_value=REPOSITORY_ROOT / 'models/tracks/2024_michigan_endurance_closed.xml'), \
             mock.patch('simplified_vehicle_models.tracks.profile.load_track', return_value=mock.Mock(is_closed=False)):
            with self.assertRaisesRegex(ValueError,'closed tracks only'):
                _point_mass({'track':'track.xml','model':'planar-7dof','formulation':'model/formulation.xml'},
                            Path('/tmp/attempt'),settings,Path('/tmp/case'))


if __name__ == '__main__':
    unittest.main()
