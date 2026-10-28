"""Bounded offscreen tests for yaw-map controller lifecycle."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from PyQt6.QtCore import QEventLoop, QTimer
from PyQt6.QtWidgets import QApplication, QMessageBox

from tools.sim_gui.qt.yaw_map_controller import YawMapController
from tools.sim_gui.yaw_map import MapBuildRequest
from tools.sim_gui.qt.main_window import MainWindow
from tools.sim_gui import REPOSITORY_ROOT
from PyQt6.QtCore import QSettings


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        formulation = self.root / "formulation.xml"
        formulation.write_text("synthetic")
        self.request = MapBuildRequest(formulation, None, self.root / "out.npz", "identity", "missing map")

    def wait_event(self, event, timeout=2.0):
        loop = QEventLoop()
        timer = QTimer(); timer.setSingleShot(True); timer.timeout.connect(loop.quit); timer.start(int(timeout * 1000))
        poll = QTimer(); poll.timeout.connect(lambda: loop.quit() if event.is_set() else None); poll.start(5)
        if not event.is_set(): loop.exec()
        poll.stop(); timer.stop()
        self.assertTrue(event.is_set(), "timed out waiting for controller completion")

    def make_window(self):
        store_path = self.root / "settings.ini"
        store = QSettings(str(store_path), QSettings.Format.IniFormat)
        window = MainWindow(output_root=self.root / "runs", settings_store=store)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.track_tab.set_track(REPOSITORY_ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml")
        window.settings_tab.solver_kind.setCurrentIndex(window.settings_tab.solver_kind.findData("yaw-aware"))
        window.settings_tab.yaw_map.setText(str(self.root / "synthetic-map.npz"))
        window.single_tab.set_export_folder(self.root / "single")
        def close_window():
            window._running = False
            window._run_mode = None
            window._run_requested = False
            if window._validation_thread is not None:
                window.stop_cases()
                self.wait_until(lambda: window._validation_thread is None, timeout=8)
            window.close()
        self.addCleanup(close_window)
        return window

    def wait_until(self, predicate, timeout=3):
        loop = QEventLoop(); timer = QTimer(); timer.setSingleShot(True); timer.timeout.connect(loop.quit); timer.start(timeout * 1000)
        poll = QTimer(); poll.timeout.connect(lambda: loop.quit() if predicate() else None); poll.start(5)
        if not predicate(): loop.exec()
        poll.stop(); timer.stop(); self.assertTrue(predicate(), "window did not reach bounded target")

    def command_factory(self, request):
        from tools.sim_gui.yaw_map import MapBuildRequest
        if isinstance(request, MapBuildRequest):
            script = f"from pathlib import Path; print('fake child', flush=True); Path({str(request.output)!r}).write_bytes(b'fake')"
            return [sys.executable, "-u", "-c", script]
        return [sys.executable, "-u", "-c", "pass"]

    def install_resolver(self, window, callback):
        """Patch inspection/build acceptance while leaving worker->GUI request wiring real."""
        from tools.sim_gui.qt.yaw_map_controller import YawMapController as controller_type
        original = controller_type.__init__
        def init(instance, *args, **kwargs):
            kwargs["cache_root"] = self.root / "cache"
            kwargs["command_factory"] = self.command_factory
            original(instance, *args, **kwargs)
        self.controller_init_patch = mock.patch.object(controller_type, "__init__", init)
        self.controller_init_patch.start(); self.addCleanup(self.controller_init_patch.stop)
        if isinstance(callback, MapBuildRequest):
            inspection = mock.patch("tools.sim_gui.qt.yaw_map_controller.inspect_yaw_map", return_value=callback)
        else:
            inspection = mock.patch("tools.sim_gui.qt.yaw_map_controller.inspect_yaw_map", return_value=callback)
        inspection.start(); self.addCleanup(inspection.stop)
        accepted = mock.patch("tools.sim_gui.qt.yaw_map_controller.accept_yaw_map", return_value=self.root / "synthetic-map.npz")
        accepted.start(); self.addCleanup(accepted.stop)
        validation = mock.patch("tools.sim_gui.qt.workers.validate_case", return_value=None)
        validation.start(); self.addCleanup(validation.stop)
        self.validate_patch = validation
        PreparedCase = __import__("tools.sim_gui.execute", fromlist=["PreparedCase"]).PreparedCase
        def fake_prepare(directory, definition, case, **kwargs):
            resolver = kwargs.get("yaw_map_resolver")
            if resolver is not None:
                resolver(self.root / "f.xml", definition.settings)
            return PreparedCase(Path(directory) / "cases" / case.id, self.root / "f.xml", self.root / "v.xml", [], solver_kind="yaw-aware")
        preparation = mock.patch("tools.sim_gui.qt.workers.prepare_case", side_effect=fake_prepare)
        preparation.start(); self.addCleanup(preparation.stop)
        return controller_type

    def item(self):
        return {"build": self.request, "event": threading.Event(), "result": None, "error": None}

    def run_script(self, script, timeout_ms=1200, accept=True, save_grace_ms=0):
        controller = YawMapController(cache_root=self.root, timeout_ms=timeout_ms,
            save_grace_ms=save_grace_ms,
            command_factory=lambda _: [sys.executable, "-u", "-c", script])
        item = self.item(); controller._active = item; controller._consented = True
        with mock.patch("tools.sim_gui.qt.yaw_map_controller.accept_yaw_map",
                       return_value=self.request.output if accept else None,
                       side_effect=None if accept else ValueError("incomplete artifact")):
            controller._start_request(item); self.wait_event(item["event"])
        return controller, item

    def test_success_logs_and_completes_once(self):
        marker = self.root / "completion"
        controller, item = self.run_script(f"import pathlib; print('hello log',flush=True); pathlib.Path({str(marker)!r}).write_text('done')")
        self.assertEqual(item["result"], self.request.output); self.assertIsNone(item["error"])
        self.assertEqual(marker.read_text(), "done")
        log = self.request.output.with_suffix(".log").read_text()
        self.assertIn("hello log", log)
        self.assertIn('"event": "launch"', log)
        self.assertIn('"event": "terminal"', log)
        self.assertIn('"status": "accepted"', log)
        events = [__import__('json').loads(line) for line in log.splitlines() if line.startswith('{')]
        launch = next(event for event in events if event['event'] == 'launch')
        self.assertEqual(launch['hard_timeout_ms'], launch['build_budget_ms'] + launch['save_grace_ms'])
        self.assertIsNone(controller._item)
        controller.shutdown()

    def test_nonzero_incomplete_startup_and_timeout(self):
        controller, item = self.run_script("raise SystemExit(8)")
        self.assertIn("status 8", item["error"]); controller.shutdown()
        incomplete_report = self.request.output.with_suffix('.json')
        incomplete_report.write_text('{"complete": false}')
        controller, item = self.run_script("pass", accept=False)
        self.assertIn("incomplete artifact", item["error"])
        self.assertIn('"status": "incomplete"', self.request.output.with_suffix('.log').read_text())
        controller.shutdown()
        controller = YawMapController(cache_root=self.root, command_factory=lambda _: [str(self.root / "absent-command")])
        item = self.item(); controller._active = item; controller._consented = True
        controller._start_request(item); self.wait_event(item["event"])
        self.assertIn("failed to start", item["error"]); controller.shutdown()
        controller, item = self.run_script("import time; time.sleep(2)", timeout_ms=25, save_grace_ms=0)
        self.assertIn("timed out", item["error"])
        timeout_log = self.request.output.with_suffix('.log').read_text()
        self.assertIn('"status": "timeout"', timeout_log)
        controller.shutdown()

    def test_builder_budget_precedes_hard_timeout_by_save_grace(self):
        script = "import time; time.sleep(.16)"
        controller = YawMapController(cache_root=self.root, timeout_ms=50, save_grace_ms=400,
            cleanup_grace_ms=100, command_factory=lambda _: [sys.executable, '-u', '-c', script])
        item = self.item(); controller._active = item; controller._consented = True
        controller._start_request(item)
        launch = next(__import__('json').loads(line) for line in self.request.output.with_suffix('.log').read_text().splitlines()
                      if line.startswith('{') and __import__('json').loads(line).get('event') == 'launch')
        self.assertEqual(launch['hard_timeout_ms'], 450)
        self.assertEqual(launch['build_budget_ms'], 50)
        self.assertEqual(launch['save_grace_ms'], 400)
        self.wait_event(item['event'])
        self.assertIn('not accepted', item['error'])
        self.assertNotIn('timed out', item['error'])
        controller.shutdown()

    def test_prompt_cancel_never_starts_process(self):
        controller = YawMapController(cache_root=self.root, command_factory=lambda _: ["/absent"])
        item = self.item(); controller._active = item
        with mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Cancel):
            controller._start_request(item)
        self.assertTrue(item["event"].is_set()); self.assertIn("cancelled", item["error"])
        self.assertIsNone(controller.process)

    def test_stop_and_close_cancel_running_child(self):
        controller = YawMapController(cache_root=self.root, timeout_ms=5000,
            command_factory=lambda _: [sys.executable, "-c", "import time; time.sleep(10)"])
        item = self.item(); controller._active = item; controller._consented = True
        controller._start_request(item)
        QTimer.singleShot(30, controller.shutdown)
        self.wait_event(item["event"])
        self.assertIn("cancelled", item["error"])
        self.assertTrue(controller.process is None or controller.process.state().value == 0)

    def test_single_run_real_worker_cancel_does_not_queue(self):
        window = self.make_window()
        self.install_resolver(window, self.request)
        self.validate_patch.stop()
        with mock.patch("tools.sim_gui.qt.workers.validate_case", return_value="invalid case"), \
             mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Cancel), \
             mock.patch.object(window._queue, "start") as start:
            window.run_single()
            self.wait_until(lambda: window._validation_thread is None)
            self.assertFalse(window.go_action.isEnabled()) if window._run_mode == "single" else None
            start.assert_not_called()
        self.assertIsNone(window._run_mode)

    def test_changed_definition_discards_prepared_cases_and_rebuilds(self):
        window = self.make_window()
        original = object()
        window._prepared = [original]
        window._prepared_definition = {"stale": True}
        with mock.patch.object(window, "build_cases") as build, \
             mock.patch.object(window, "_start_study_queue") as start:
            window.run_cases()
            build.assert_called_once()
            start.assert_not_called()
            self.assertEqual(window._prepared, [])
            self.assertIsNone(window._prepared_definition)

    def test_study_request_dispatch_success_and_failure_no_partial_launch(self):
        window = self.make_window()
        self.install_resolver(window, self.request)
        with mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Yes), \
             mock.patch.object(window._queue, "start") as start:
            window.run_cases()
            self.wait_until(lambda: window._validation_thread is None)
            self.assertEqual(start.call_count, 1)
            self.assertEqual(len(start.call_args.args[0]), 1)
        window._prepared = []
        window._run_mode = None
        window._run_requested = True
        window.study_index = None
        window._queue.study_index = None
        # A failed acceptance marks the case invalid; queued study continuation is suppressed.
        self.install_resolver(window, self.request)
        self.validate_patch.stop()
        with mock.patch("tools.sim_gui.qt.workers.validate_case", return_value="invalid case"), \
             mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Yes), \
             mock.patch("tools.sim_gui.qt.yaw_map_controller.accept_yaw_map", side_effect=ValueError("incomplete")), \
             mock.patch.object(window._queue, "start") as start:
            window.run_cases()
            self.wait_until(lambda: window._validation_thread is None)
            start.assert_not_called()
        self.assertFalse(window._run_requested)
        self.assertIsNone(window._run_mode)

    def test_matching_cache_fast_path_no_prompt_and_single_continuation(self):
        window = self.make_window()
        cache = self.root / "matching.npz"
        cache.write_bytes(b"fake but helper inspection is mocked")
        self.install_resolver(window, cache)
        with mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm") as prompt, \
             mock.patch.object(window._queue, "start") as start:
            window.run_single()
            self.wait_until(lambda: start.called)
            start.assert_called_once()
            prompt.assert_not_called()
        self.assertFalse(window._run_requested)
        # A fresh Build Cases action must not inherit single-run continuation intent.
        with mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm") as prompt, \
             mock.patch.object(window._queue, "start") as start:
            window.build_cases()
            self.wait_until(lambda: window._validation_thread is None)
            start.assert_not_called()
            prompt.assert_not_called()

    def test_failed_shutdown_retains_controller_ownership(self):
        window = self.make_window()
        controller = mock.Mock()
        controller.shutdown.return_value = False
        window._yaw_map_controller = controller
        window._run_mode = "building"
        window._validation_thread = None
        window._validation_worker = None
        with mock.patch.object(window, "_finish_single_without_queue"):
            window._validation_finished()
        self.assertIs(window._yaw_map_controller, controller)
        controller.deleteLater.assert_not_called()

    def test_window_close_waits_for_ignoring_child_and_reaps_thread(self):
        window = self.make_window()
        controller_type = self.install_resolver(window, self.request)
        original = controller_type.__init__
        def hard_init(instance, *args, **kwargs):
            kwargs["command_factory"] = lambda request: [sys.executable, "-u", "-c", "import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(10)"] if isinstance(request, MapBuildRequest) else self.command_factory(request)
            kwargs["timeout_ms"] = 30000
            original(instance, *args, **kwargs)
        self.validate_patch.stop()
        with mock.patch("tools.sim_gui.qt.workers.validate_case", return_value="invalid after preparation"), \
             mock.patch.object(controller_type, "__init__", hard_init), \
             mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Yes):
            window.run_single()
            self.wait_until(lambda: window._yaw_map_controller is not None and window._yaw_map_controller.process is not None and window._yaw_map_controller.process.state().value != 0)
            controller = window._yaw_map_controller
            process = controller.process
            window.close()
            self.wait_until(lambda: window._validation_thread is None)
            self.assertIsNone(window._yaw_map_controller)
            self.assertFalse(window._validation_thread)
            self.assertIsNone(controller.process)
            with self.assertRaises(RuntimeError):
                process.state()
            self.assertFalse(window.isVisible())

    def test_distinct_submissions_require_distinct_consent(self):
        for _ in range(2):
            controller = YawMapController(cache_root=self.root, command_factory=lambda _: [str(self.root / "absent")])
            item = self.item(); controller._active = item
            with mock.patch("tools.sim_gui.qt.yaw_map_controller._confirm", return_value=QMessageBox.StandardButton.Cancel) as prompt:
                controller._start_request(item)
            prompt.assert_called_once()
            controller.shutdown()


if __name__ == "__main__":
    unittest.main()
