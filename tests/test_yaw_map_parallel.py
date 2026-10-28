import unittest
import os
import signal
import subprocess
import sys
import tempfile
import time
import threading
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from dataclasses import dataclass
from unittest.mock import patch

from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus
from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator
try:
    from .yaw_parallel_blocking import blocking_cell
except ImportError:
    from yaw_parallel_blocking import blocking_cell
from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope


@dataclass
class TimeoutVehicle:
    total_mass: float
    tire: object


def _validation_cell(vehicle, limits, reduced, point):
    return DirectYawCell(ProbeStatus.FEASIBLE, 1., 1.)


def _fast_slow_cell(vehicle, limits, reduced, point):
    if point[0] == 2.:
        time.sleep(5.)
    else:
        time.sleep(.05)
    return DirectYawCell(ProbeStatus.FEASIBLE, point[0], point[0] + 1.)


def _timeout_vehicle():
    import numpy as np
    return TimeoutVehicle(40., UniTirePeakEnvelope(
        vertical_loads=np.array([0., 500., 1000.]), longitudinal_peak_force=np.array([0., 100., 100.]),
        lateral_peak_force=np.array([0., 120., 120.]), effective_radius=.2, reference_speed=3.))


class BatchEvaluatorTests(unittest.TestCase):
    def test_worker_timeout_composes_through_incomplete_workflow_and_artifact_save(self):
        from dataclasses import asdict
        from simplified_vehicle_models.formulations.yaw_map_artifact import json_values, load_yaw_map, save_yaw_map
        from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
        from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
        vehicle = _timeout_vehicle()
        settings = YawMapWorkflowSettings(discovery=YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.25,
            probe_budget=50, lateral_interpolation_loss_tolerance=1.), maximum_lateral_cell_width=1.,
            maximum_map_nodes=1000, maximum_audit_probes=50, maximum_refinement_rounds=1, time_budget_seconds=.4)
        evaluator = ParallelYawEvaluator(vehicle, workers=2, batch_size=2, time_budget_seconds=.4,
                                         _worker_function=blocking_cell)
        with tempfile.TemporaryDirectory() as folder:
            artifact = Path(folder) / 'timeout.npz'
            events = []
            try:
                result = build_yaw_map_workflow(vehicle, [0., 1.], [-1., 1.], evaluator,
                    yaw_inertia=1., settings=settings, map_builder=evaluator.build,
                    progress_callback=lambda event, fields: events.append(event))
                self.assertTrue(evaluator._closed)
                self.assertIn('validation_discovery_complete', events)
                self.assertIn('validation_map_audit_complete', events)
                self.assertFalse(result.complete)
                self.assertTrue(result.domain.diagnostics.unresolved_probes)
                report = {k: json_values(v) for k, v in asdict(result).items() if k != 'capability'}
                save_yaw_map(artifact, result.capability, report=report, identity='timeout-test')
                _, metadata = load_yaw_map(artifact, expected_identity='timeout-test', allow_incomplete=True)
                self.assertFalse(metadata['report']['complete'])
                with self.assertRaisesRegex(ValueError, 'incomplete'):
                    load_yaw_map(artifact, expected_identity='timeout-test')
            finally:
                evaluator.close()

    def test_physical_batch_preserves_order_duplicates_cache_and_deadline(self):
        evaluator = PhysicalYawEvaluator(object())
        points = [(10., -1., 0.), (10., -1., 0.), (10., 1., 2.)]
        def direct(_vehicle, speed, ax, ay, _limits):
            return DirectYawCell(ProbeStatus.FEASIBLE, abs(ax), abs(ay))
        with patch('simplified_vehicle_models.formulations.yaw_map_physical.direct_yaw_cell', side_effect=direct) as mock:
            results = evaluator.cells_many(points)
        self.assertEqual([c.into_turn for c in results], [1., 1., 1.])
        self.assertEqual(mock.call_count, 2)
        self.assertEqual([p.status for p in evaluator.probe_many(points)], [ProbeStatus.FEASIBLE] * 3)

    def test_serial_parallel_batch_and_exception_propagation(self):
        points = [(1., -1., 0.), (1., 1., 0.)]
        evaluator = ParallelYawEvaluator(object(), workers=1)
        with patch('simplified_vehicle_models.formulations.yaw_map_parallel._evaluate_cell',
                   return_value=DirectYawCell(ProbeStatus.FEASIBLE, 1., 2.)):
            self.assertEqual(len(evaluator.cells_many(points)), 2)
        evaluator.close()
        evaluator = ParallelYawEvaluator(object(), workers=1)
        with patch('simplified_vehicle_models.formulations.yaw_map_parallel._evaluate_cell', side_effect=RuntimeError('boom')):
            with self.assertRaisesRegex(RuntimeError, 'boom'):
                evaluator.cells_many(points)

    def test_positive_inflight_deadline_returns_unresolved_and_reaps_workers(self):
        evaluator = ParallelYawEvaluator(object(), workers=2, batch_size=2, time_budget_seconds=.5,
                                         _worker_function=blocking_cell)
        started = time.monotonic()
        cells = evaluator.cells_many([(1., 0., 0.), (2., 0., 0.)])
        self.assertLess(time.monotonic() - started, 3.)
        self.assertEqual([cell.status for cell in cells], [ProbeStatus.UNRESOLVED] * 2)
        self.assertTrue(all('budget' in cell.detail for cell in cells))
        self.assertIsNone(evaluator._pool)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            evaluator.cells_many([(2., 0., 0.)])

    def test_timeout_keeps_completed_indices_and_marks_only_missing_unresolved(self):
        evaluator = ParallelYawEvaluator(object(), workers=2, batch_size=2, time_budget_seconds=3.,
                                        _worker_function=_fast_slow_cell)
        try:
            cells = evaluator.cells_many([(1., 0., 0.), (2., 0., 0.)])
            self.assertEqual(cells[0].status, ProbeStatus.FEASIBLE)
            self.assertEqual(cells[0].into_turn, 1.)
            self.assertEqual(cells[1].status, ProbeStatus.UNRESOLVED)
            self.assertEqual(evaluator.cells[(1., 0., 0.)].status, ProbeStatus.FEASIBLE)
            self.assertEqual(evaluator.cells[(2., 0., 0.)].status, ProbeStatus.UNRESOLVED)
        finally:
            evaluator.close()

    def test_close_interrupts_blocking_dispatch_without_deadline(self):
        evaluator = ParallelYawEvaluator(object(), workers=2, batch_size=2,
                                         _worker_function=blocking_cell)
        failures = []
        dispatch = threading.Thread(target=lambda: self._capture_dispatch(evaluator, failures))
        dispatch.start()
        deadline = time.monotonic() + 5.
        while evaluator._pool is None and time.monotonic() < deadline:
            time.sleep(.01)
        time.sleep(.1)
        started = time.monotonic()
        closer = threading.Thread(target=evaluator.close)
        closer.start()
        closer.join(timeout=3.)
        dispatch.join(timeout=3.)
        self.assertFalse(closer.is_alive(), 'close blocked behind active dispatch')
        self.assertFalse(dispatch.is_alive(), 'dispatch did not wake on close')
        self.assertLess(time.monotonic() - started, 3.)
        self.assertTrue(failures)
        self.assertIsNone(evaluator._pool)

    @staticmethod
    def _capture_dispatch(evaluator, failures):
        try:
            evaluator.cells_many([(1., 0., 0.), (2., 0., 0.)])
        except BaseException as error:
            failures.append(error)

    def test_enter_after_close_does_not_restart_workers(self):
        evaluator = ParallelYawEvaluator(object(), workers=2)
        evaluator.close()
        baseline = {child.pid for child in mp.active_children()}
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            evaluator.__enter__()
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            evaluator._start_pool()
        self.assertIsNone(evaluator._pool)
        self.assertEqual({child.pid for child in mp.active_children()}, baseline)

    def test_spawned_exception_cleans_up_workers(self):
        evaluator = ParallelYawEvaluator(object(), workers=2, batch_size=1)
        with self.assertRaises(AttributeError):
            evaluator.cells_many([(1., 0., 0.)])
        self.assertIsNone(evaluator._pool)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            evaluator.cells_many([(2., 0., 0.)])

    def test_parent_sigkill_closes_spawned_workers(self):
        with tempfile.TemporaryDirectory() as tmp:
            pids = Path(tmp) / 'pids'
            script = r'''import sys,time
from pathlib import Path
from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator
e=ParallelYawEvaluator(object(),workers=2,batch_size=2); e.__enter__()
while not all(p.is_alive() for p in e._pool): time.sleep(.01)
Path(sys.argv[1]).write_text(','.join(str(p.pid) for p in e._pool))
while True: time.sleep(1)
'''
            env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'src'))
            parent = subprocess.Popen([sys.executable, '-c', script, str(pids)], env=env)
            try:
                deadline = time.time() + 30
                while not pids.exists() and time.time() < deadline: time.sleep(.05)
                self.assertTrue(pids.exists())
                worker_pids = [int(pid) for pid in pids.read_text().split(',')]
                parent.send_signal(signal.SIGKILL); parent.wait(timeout=5)
                deadline = time.time() + 10
                while time.time() < deadline:
                    alive = []
                    for pid in worker_pids:
                        try:
                            os.kill(pid, 0)
                            stat_path = Path(f'/proc/{pid}/stat')
                            if stat_path.exists() and stat_path.read_text().split()[2] == 'Z': continue
                            alive.append(pid)
                        except ProcessLookupError: pass
                    if not alive: break
                    time.sleep(.05)
                self.assertFalse(alive, f'orphan workers remain: {alive}')
            finally:
                if parent.poll() is None: parent.kill(); parent.wait()

    def test_real_full_and_reduced_spawn_match_serial_cells_and_maps(self):
        import numpy as np
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid
        model_path = Path(__file__).resolve().parents[1] / 'models/formulations/iem24_murphy_real_planar_7dof.xml'
        model = compile_formulation(str(model_path)).model
        vehicle = point_mass_view_of_planar(model)
        points = [(5., -1., 0.), (5., 0., 0.), (5., 1., 1.), (5., -1., 0.)]
        for reduced in (False, True):
            with self.subTest(reduced=reduced):
                expected = PhysicalYawEvaluator(vehicle, use_reduced=reduced).cells_many(points)
                with ParallelYawEvaluator(vehicle, workers=2, use_reduced=reduced, batch_size=2) as parallel:
                    actual = parallel.cells_many(points)
                    self.assertEqual(len(parallel.cells), 3)
                    self.assertEqual([c.status for c in expected], [c.status for c in actual])
                    np.testing.assert_allclose([c.into_turn or 0. for c in expected], [c.into_turn or 0. for c in actual])
                    np.testing.assert_allclose([c.out_of_turn or 0. for c in expected], [c.out_of_turn or 0. for c in actual])
                grid = YawCapabilityGrid([5.], [-1., 0.], [0., 1.])
                expected_map = PhysicalYawEvaluator(vehicle, use_reduced=reduced).build(vehicle, grid, yaw_inertia=model.yaw_inertia)
                with ParallelYawEvaluator(vehicle, workers=2, use_reduced=reduced, batch_size=2) as parallel:
                    actual_map = parallel.build(vehicle, grid, yaw_inertia=model.yaw_inertia)
                np.testing.assert_array_equal(actual_map.feasible_mask, expected_map.feasible_mask)
                np.testing.assert_allclose(actual_map.yaw_moment_into_turn, expected_map.yaw_moment_into_turn)
                np.testing.assert_allclose(actual_map.yaw_moment_out_of_turn, expected_map.yaw_moment_out_of_turn)

    def test_workflow_deadline_caps_physical_evaluator_without_own_budget(self):
        from types import SimpleNamespace
        import numpy as np
        from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
        from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
        vehicle = SimpleNamespace(total_mass=40., tire=UniTirePeakEnvelope(vertical_loads=np.array([0.,500.,1000.]),
            longitudinal_peak_force=np.array([0.,100.,100.]), lateral_peak_force=np.array([0.,120.,120.]),
            effective_radius=.2, reference_speed=3.))
        evaluator = ParallelYawEvaluator(vehicle, workers=2, batch_size=2, _worker_function=blocking_cell)
        settings = YawMapWorkflowSettings(discovery=YawMapDomainSettings(scan_spacing=20., bracket_tolerance=10.,
            probe_budget=20, speed_ax_audit_subdivisions=0), maximum_lateral_cell_width=20., maximum_map_nodes=20,
            maximum_audit_probes=20, maximum_refinement_rounds=1, time_budget_seconds=.6)
        started = time.monotonic()
        def builder(_v, grid, _limits, *, yaw_inertia):
            from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
            shape=(len(grid.speed),len(grid.longitudinal_acceleration),len(grid.lateral_acceleration))
            zeros=np.zeros(shape); return YawCapabilityMap(grid.speed,grid.longitudinal_acceleration,
                grid.lateral_acceleration,zeros,zeros,np.zeros(shape,bool),yaw_inertia,'physical_no_tv_v1','fixture')
        result = build_yaw_map_workflow(vehicle, [0., 10.], [0., 1.], evaluator, yaw_inertia=1., settings=settings, map_builder=builder)
        self.assertLess(time.monotonic() - started, 3.)
        self.assertFalse(result.domain.diagnostics.complete)
        self.assertTrue(result.domain.diagnostics.unresolved_probes)
        evaluator.close()

    def test_workflow_owned_validation_workers_close_on_success_and_exception(self):
        import numpy as np
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
        from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
        root = Path(__file__).resolve().parents[1]
        model = compile_formulation(str(root / 'models/formulations/iem24_murphy_real_planar_7dof.xml')).model
        vehicle = point_mass_view_of_planar(model)
        settings = YawMapWorkflowSettings(discovery=YawMapDomainSettings(scan_spacing=100., bracket_tolerance=50.,
            probe_budget=10, speed_ax_audit_subdivisions=0), maximum_lateral_cell_width=100., maximum_map_nodes=100,
            maximum_audit_probes=10, maximum_refinement_rounds=1, time_budget_seconds=10.)
        training = ParallelYawEvaluator(vehicle, workers=2, batch_size=1, _worker_function=_validation_cell)
        def builder(_v, grid, _limits, *, yaw_inertia):
            from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
            shape=(len(grid.speed),len(grid.longitudinal_acceleration),len(grid.lateral_acceleration))
            zeros=np.zeros(shape); return YawCapabilityMap(grid.speed,grid.longitudinal_acceleration,
                grid.lateral_acceleration,zeros,zeros,np.zeros(shape,bool),yaw_inertia,'physical_no_tv_v1','fixture')
        baseline = {child.pid for child in mp.active_children()}
        try:
            events = []
            result = build_yaw_map_workflow(vehicle, [0., 10.], [-1., 1.], training,
                yaw_inertia=model.yaw_inertia, settings=settings, map_builder=builder,
                progress_callback=lambda event, fields: events.append(event))
            self.assertIn('validation_discovery_complete', events)
            self.assertFalse(result.complete)
            training_pids = {p.pid for p in training._pool or ()}
            expected = baseline | training_pids
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline and {p.pid for p in mp.active_children()} != expected: time.sleep(.02)
            self.assertEqual({p.pid for p in mp.active_children()}, expected,
                             'workflow-owned validation workers survived successful return')
            with self.assertRaisesRegex(RuntimeError, 'validation callback failure'):
                build_yaw_map_workflow(vehicle, [0., 10.], [-1., 1.], training,
                    yaw_inertia=model.yaw_inertia, settings=settings, map_builder=builder,
                    progress_callback=lambda event, fields: (_ for _ in ()).throw(RuntimeError('validation callback failure'))
                    if event == 'validation_discovery_complete' else None)
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline and {p.pid for p in mp.active_children()} != expected: time.sleep(.02)
            self.assertEqual({p.pid for p in mp.active_children()}, expected,
                             'workflow-owned validation workers survived exceptional return')
        finally:
            training.close()

    def test_concurrent_repeated_close_and_exception_cleanup(self):
        baseline = {child.pid for child in mp.active_children()}
        evaluators = [ParallelYawEvaluator(object(), workers=2, batch_size=1) for _ in range(2)]
        worker_pids = set()
        for evaluator in evaluators:
            evaluator.__enter__()
            worker_pids.update(process.pid for process in evaluator._pool)
        try:
            with ThreadPoolExecutor(max_workers=4) as pool:
                futures = []
                for evaluator in evaluators:
                    futures.extend((pool.submit(evaluator.close), pool.submit(evaluator.close),
                                    pool.submit(evaluator.cells_many, [(1., 0., 0.)])))
                for future in futures:
                    try: future.result(timeout=10)
                    except RuntimeError as error: self.assertRegex(str(error), 'closed')
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline and {child.pid for child in mp.active_children()} != baseline: time.sleep(.02)
            self.assertEqual({child.pid for child in mp.active_children()}, baseline)
            self.assertFalse(worker_pids & {child.pid for child in mp.active_children()})
            self.assertTrue(all(e._pool is None for e in evaluators))
        finally:
            for evaluator in evaluators: evaluator.close()

    def test_workers_validated(self):
        for workers in (0, -1, True):
            with self.subTest(workers=workers), self.assertRaises(ValueError): ParallelYawEvaluator(object(), workers=workers)


if __name__ == '__main__':
    unittest.main()
