import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
from pathlib import Path
import sys
import tempfile
import time
import shutil

from unittest import mock
import json

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])

if QApplication is not None:
    from PyQt6.QtGui import QFont
    from tools.sim_gui import REPOSITORY_ROOT
    from tools.sim_gui.vehicle import VehicleConfiguration
    from tools.sim_gui.settings import RunSettings
    from tools.sim_gui.qt.model_panel import ModelPanel
    from tools.sim_gui.qt.settings_tab import SettingsTab
    from tools.sim_gui.qt.study_tab import StudyTab
    from tools.sim_gui.qt.workers import ValidationWorker
    from tools.sim_gui.qt.queue import CaseQueue
    from tools.sim_gui.qt.main_window import MainWindow
    from PyQt6.QtCore import QSettings, QEventLoop, QTimer, QThread
    from PyQt6.QtWidgets import QMessageBox
    from tools.sim_gui.study import Dimension, StudyDefinition, CaseSpec, expand, WARNING_CASES, TooManyCases
    from tools.sim_gui.execute import PreparedCase
    from PyQt6.QtCore import QEventLoop, QTimer, Qt


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class ModelPanelTests(unittest.TestCase):
    def setUp(self):
        self.source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
        self.cfg = VehicleConfiguration.open(self.source)
        self.panel = ModelPanel(self.cfg)

    def tearDown(self):
        self.panel.deleteLater()

    def _combo(self, key):
        return self.panel._slot_widgets[key][0]

    def test_tire_combo_has_two_items(self):
        self.assertEqual(self._combo("tire").count(), 2)

    def test_selecting_hoosier_emits_and_bolds(self):
        emitted = []
        self.panel.configurationChanged.connect(emitted.append)
        combo = self._combo("tire")
        index = next(i for i in range(combo.count()) if "hoosier-43075" in combo.itemData(i))
        combo.setCurrentIndex(index)
        self.assertEqual(len(emitted), 1)
        self.assertEqual(emitted[0].choices["tire"], (REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml").resolve())
        self.assertTrue(combo.font().bold())

    def test_powertrain_swap_rebuilds_children(self):
        combo = self._combo("powertrain")
        old_child = self._combo("powertrain/power-plant")
        combo.setCurrentIndex(1 if combo.currentIndex() == 0 else 0)
        self.assertIn("powertrain/power-plant", self.panel._slot_widgets)
        self.assertIsNot(old_child, self._combo("powertrain/power-plant"))
        from PyQt6.QtCore import QCoreApplication, QEvent
        app.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        try:
            from PyQt6 import sip
            self.assertTrue(sip.isdeleted(old_child))
        except ImportError:
            self.assertTrue(old_child.parent() is None or old_child not in self.panel.findChildren(type(old_child)))

    def test_new_configuration_resets_baseline(self):
        first = self.panel.configuration()
        combo = self._combo("tire")
        combo.setCurrentIndex(1 if combo.currentIndex() == 0 else 0)
        second = VehicleConfiguration.open(
            REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        )
        self.panel.set_configuration(second)
        self.assertFalse(self._combo("tire").font().bold())
        self.assertFalse(self.panel._slot_widgets["tire"][2].isEnabled())
        self.assertEqual(self.panel.baseline(), second)

    def test_incompatible_candidate_warns_and_restores(self):
        combo = self._combo("tire")
        old = combo.currentIndex()
        combo.addItem("incompatible", str(REPOSITORY_ROOT / "models/aero/aero_2024.xml"))
        with mock.patch("tools.sim_gui.qt.model_panel._warn") as warn:
            combo.setCurrentIndex(combo.count() - 1)
        warn.assert_called_once()
        self.assertEqual(combo.currentIndex(), old)

    def test_reset_restores_combo_font_and_button(self):
        combo = self._combo("tire")
        combo.setCurrentIndex(1 if combo.currentIndex() == 0 else 0)
        self.assertTrue(combo.font().bold())
        reset = self.panel._slot_widgets["tire"][2]
        self.assertTrue(reset.isEnabled())
        reset.click()
        self.assertEqual(combo.currentData(), str(self.cfg.choices["tire"]))
        self.assertFalse(combo.font().bold())
        self.assertFalse(reset.isEnabled())

    def test_rebuild_retains_bold_changed_tire(self):
        tire = self._combo("tire")
        tire.setCurrentIndex(1 if tire.currentIndex() == 0 else 0)
        powertrain = self._combo("powertrain")
        powertrain.setCurrentIndex(1 if powertrain.currentIndex() == 0 else 0)
        self.assertTrue(self._combo("tire").font().bold())

    def test_point_mass_model_is_shown_without_emission(self):
        point_mass = VehicleConfiguration.open(
            REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml"
        )
        panel = ModelPanel(point_mass)
        try:
            self.assertEqual(panel._model.currentText(), "point-mass")
            self.assertIn("no forward runner", panel._error.text())
            emitted = []
            panel.configurationChanged.connect(emitted.append)
            panel._model.setCurrentText("planar-7dof")
            self.assertEqual(emitted, [panel.configuration()])
        finally:
            panel.deleteLater()

    def test_track_is_preselected(self):
        self.assertIsNotNone(self.panel.track())


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class SettingsTabTests(unittest.TestCase):
    def setUp(self):
        self.tab = SettingsTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_quasi_static_disabled_for_14dof(self):
        self.tab.set_model("14dof-simple")
        self.assertFalse(self.tab.quasi_static_attitude.isEnabled())

    def test_planar_timeout_maximum(self):
        self.tab.set_model("planar-7dof")
        self.assertEqual(self.tab.timeout_seconds.maximum(), 300.0)

    def test_14dof_clears_quasi_static_and_uses_long_timeout(self):
        self.tab.quasi_static_attitude.setChecked(True)
        self.tab.set_model("14dof-simple")
        self.assertFalse(self.tab.quasi_static_attitude.isChecked())
        self.assertFalse(self.tab.settings().quasi_static_attitude)
        self.assertEqual(self.tab.timeout_seconds.value(), 3600.0)
        self.assertEqual(self.tab.timeout_seconds.maximum(), 1e6)

    def test_set_settings_emits_once_and_sets_lateral_cap(self):
        emitted = []
        self.tab.settingsChanged.connect(emitted.append)
        from tools.sim_gui.settings import RunSettings
        self.tab.set_settings(RunSettings(lateral_acceleration=2.0))
        self.assertEqual(len(emitted), 1)
        self.assertTrue(self.tab.lateral_cap.isChecked())
        self.assertTrue(self.tab.lateral_value.isEnabled())

    def test_preset_emits_once(self):
        emitted = []
        self.tab.settingsChanged.connect(emitted.append)
        self.tab.preset_button.click()
        self.assertEqual(len(emitted), 1)
        self.assertEqual(emitted[0], RunSettings.for_model("planar-7dof"))

    def test_nonplanar_settings_clear_quasi_static_and_validate(self):
        self.tab.set_model("14dof-simple")
        self.tab.set_settings(RunSettings(quasi_static_attitude=True))
        self.assertFalse(self.tab.settings().quasi_static_attitude)

    def test_choice_lists_match_runner_parser(self):
        from tools.run import run_iem24_forward_driver as runner
        for name, values in self.tab._CHOICES.items():
            for value in values:
                args = runner.parse_args(["--model", "planar-7dof", f"--{name.replace('_', '-')}", value])
                self.assertEqual(getattr(args, name), value)


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class StudyTabTests(unittest.TestCase):
    def setUp(self):
        self.cfg = VehicleConfiguration.open(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        self.settings = RunSettings()
        self.tab = StudyTab(self.cfg, self.settings)

    def tearDown(self):
        self.tab.deleteLater()

    def test_preview_count_three(self):
        tire = self.cfg.choices["tire"]
        hoosier = REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml"
        self.tab._dimensions = [Dimension("tire", (str(tire), str(hoosier)))]
        self.tab._refresh_preview()
        self.assertEqual(self.tab.preview.rowCount(), 2)

    def test_warning_count_red(self):
        values = tuple(f"model-{i}" for i in range(WARNING_CASES + 1))
        self.tab._dimensions = [Dimension("model", values)]
        self.tab._refresh_preview()
        self.assertIn("red", self.tab.preview_count.styleSheet())

    def test_name_validator_rejects_spaces(self):
        self.tab.study_name.setText("a b")
        self.assertNotEqual(self.tab.study_name.text(), "a b")

    def test_too_many_cases_is_preview_error(self):
        self.tab._dimensions = [Dimension("tire", tuple(str(self.cfg.choices["tire"]) + str(i) for i in range(501)))]
        self.tab._refresh_preview()
        self.assertIn("maximum", self.tab.preview_count.text())

    def test_parallel_cases_spinbox_defaults_and_range(self):
        from tools.sim_gui.run_study import default_parallel
        self.assertEqual(self.tab.parallel_cases.minimum(), 1)
        self.assertEqual(self.tab.parallel_cases.maximum(), os.cpu_count() or 1)
        self.assertEqual(self.tab.parallel_cases.value(), default_parallel())


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class ValidationWorkerTests(unittest.TestCase):
    def test_real_cases_prepare_and_validate(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        cfg = VehicleConfiguration.open(source)
        hoosier = REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml"
        definition = StudyDefinition("test", str(source), "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", {"tire": str(cfg.choices["tire"])}, RunSettings(), "one-at-a-time", ())
        cases = (CaseSpec("case_000", "baseline", "planar-7dof", definition.track, definition.choices, {}), CaseSpec("case_001", "hoosier", "planar-7dof", definition.track, {"tire": str(hoosier)}, {}))
        with tempfile.TemporaryDirectory() as tmp:
            worker = ValidationWorker(Path(tmp), definition, cases)
            ready, progress = [], []
            worker.caseReady.connect(lambda item, error: ready.append((item, error)))
            worker.progress.connect(lambda done, total: progress.append((done, total)))
            worker.run()
        self.assertEqual(len(ready), 2)
        self.assertTrue(all(item is not None and error is None for item, error in ready))
        self.assertEqual(progress, [(1, 2), (2, 2)])


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class CaseQueueTests(unittest.TestCase):
    def _prepared(self, root, name="case_000", timeout=2.0):
        directory = Path(root) / name
        directory.mkdir(parents=True)
        return PreparedCase(directory, directory / "formulation.xml", directory / "vehicle.xml", [], timeout_seconds=timeout)

    def _wait(self, queue, timeout=5):
        loop = QEventLoop()
        queue.allFinished.connect(loop.quit)
        QTimer.singleShot(int(timeout * 1000), loop.quit)
        loop.exec()

    def test_cancelled_case_finishes_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = self._prepared(tmp)
            command = [sys.executable, "-c", "import json,time; print(json.dumps({'rhs_time': 1.0}), flush=True); time.sleep(30)"]
            queue = CaseQueue(command_factory=lambda _p, _a: command)
            finished = []
            queue.caseFinished.connect(lambda key, fields: finished.append(fields["status"]))
            queue.caseProgress.connect(lambda *_: queue.stop())
            queue.start([prepared], 1)
            self._wait(queue)
            self.assertEqual(finished, ["cancelled"])

    def test_timeout_case(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = self._prepared(tmp, timeout=2.0)
            command = [sys.executable, "-c", "import time; time.sleep(30)"]
            queue = CaseQueue(command_factory=lambda _p, _a: command)
            finished = []
            queue.caseFinished.connect(lambda key, fields: finished.append(fields["status"]))
            queue.start([prepared], 1, timeout=0.5)
            self._wait(queue)
            self.assertEqual(finished, ["timed_out"])

    def test_normal_exit_without_summary_is_failed(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = self._prepared(tmp)
            command = [sys.executable, "-c", "print('{\\\"rhs_time\\\": 1}', flush=True)"]
            queue = CaseQueue(command_factory=lambda _p, _a: command)
            finished = []
            queue.caseFinished.connect(lambda key, fields: finished.append(fields["status"]))
            queue.start([prepared], 1)
            self._wait(queue)
            self.assertEqual(finished, ["failed"])

    def test_finish_error_is_failed_and_queue_continues(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = self._prepared(tmp)
            queue = CaseQueue(
                command_factory=lambda _p, _a: [sys.executable, "-c", "pass"],
                finish_factory=lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("boom")),
            )
            finished = []
            queue.caseFinished.connect(lambda key, fields: finished.append(fields))
            queue.start([prepared], 1)
            self._wait(queue)
            self.assertEqual(finished[0]["status"], "failed")
            self.assertIn("boom", finished[0]["error"])

    def test_progress_uses_simulated_duration(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = PreparedCase(
                Path(tmp) / "case_000",
                Path(tmp) / "formulation.xml",
                Path(tmp) / "vehicle.xml",
                [],
                duration=0.3,
            )
            prepared.directory.mkdir()
            command = [sys.executable, "-c", "print('{\\\"rhs_time\\\": 0.15}', flush=True)"]
            queue = CaseQueue(command_factory=lambda _p, _a: command)
            progress = []
            queue.caseProgress.connect(lambda key, value: progress.append(value))
            queue.start([prepared], 1)
            self._wait(queue)
            self.assertEqual(progress, [0.5])

    def test_shutdown_kills_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = self._prepared(tmp)
            queue = CaseQueue(command_factory=lambda _p, _a: [sys.executable, "-c", "import time; time.sleep(30)"])
            queue.start([prepared], 1)
            app.processEvents()
            process = next(iter(queue._running.values())).process
            queue.shutdown()
            self.assertEqual(process.state(), process.ProcessState.NotRunning)

    def test_add_dimension_menu_and_checklist(self):
        cfg = VehicleConfiguration.open(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        tab = StudyTab(cfg, RunSettings())
        try:
            action = next(action for action in tab._dimension_menu.actions() if action.text() == "tire")
            action.trigger()
            self.assertEqual(tab._dimensions[0].key, "tire")
            tab.dimension_list.setCurrentRow(0)
            before = tab.preview.rowCount()
            tab.member_list.item(0).setCheckState(Qt.CheckState.Unchecked)
            self.assertLess(tab.preview.rowCount(), before)
        finally:
            tab.deleteLater()

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class MainWindowTests(unittest.TestCase):

    def _window(self, store=None):
        if store is None:
            store_dir = Path(tempfile.mkdtemp())
            self.addCleanup(lambda: shutil.rmtree(store_dir, ignore_errors=True))
            store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        return window

    def _wait(self, queue, timeout=5):
        loop = QEventLoop()
        queue.allFinished.connect(loop.quit)
        QTimer.singleShot(int(timeout * 1000), loop.quit)
        loop.exec()
        queue.shutdown()

    def test_main_window_end_to_end_case(self):
        output = REPOSITORY_ROOT / "runs" / "_test"
        if output.exists():
            shutil.rmtree(output)
        self.addCleanup(lambda: shutil.rmtree(output) if output.exists() else None)
        self.assertTrue(str(output).endswith("runs/_test"))
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.settings_tab.duration.setValue(0.3)
        window.settings_tab.timeout_seconds.setValue(120.0)
        window.run_cases()
        loop = QEventLoop()
        window._queue.allFinished.connect(loop.quit)
        QTimer.singleShot(120000, loop.quit)
        loop.exec()
        self.assertEqual(window.queue_table.item(0, 2).text(), "completed")
        record = window.study_index.directory / window.study_index.cases[0]["run_record"]
        self.assertTrue(record.exists())

    def _build_with_tire_dimension(self, window, validate):
        """Build a baseline + Hoosier study with a patched validate_case."""
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        hoosier = str(REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml")
        baseline_tire = str(window.configuration.choices["tire"])
        window.study_tab._dimensions = [Dimension("tire", (baseline_tire, hoosier))]
        with mock.patch("tools.sim_gui.qt.workers.validate_case", side_effect=validate):
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
        self.assertIsNone(window._validation_thread)
        return json.loads((window.study_index.directory / "study.json").read_text())

    def test_invalid_case_is_recorded_like_run_study(self):
        window = self._window()
        def validate(prepared):
            return "bad tire" if prepared.directory.name != "case_000" else None
        payload = self._build_with_tire_dimension(window, validate)
        statuses = {case["id"]: case for case in payload["cases"]}
        self.assertEqual(statuses["case_000"]["status"], "pending")
        invalid = [case for case in payload["cases"] if case["id"] != "case_000"]
        self.assertEqual(len(invalid), 1)
        self.assertEqual(invalid[0]["status"], "invalid")
        self.assertEqual(invalid[0]["error"], "bad tire")
        self.assertEqual(invalid[0]["directory"], f"cases/{invalid[0]['id']}")
        statuses_shown = {window.queue_table.item(row, 2).text() for row in range(window.queue_table.rowCount())}
        self.assertEqual(statuses_shown, {"pending", "invalid"})

    def test_prepare_failure_is_recorded_invalid(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        with mock.patch("tools.sim_gui.qt.workers.prepare_case", side_effect=RuntimeError("no file")):
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
        case = window.study_index.cases[0]
        self.assertEqual(case["status"], "invalid")
        self.assertIn("no file", case["error"])

    def test_all_invalid_does_not_rebuild_or_start(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        with mock.patch("tools.sim_gui.qt.workers.validate_case", return_value="bad"), \
                mock.patch.object(window, "build_cases", wraps=window.build_cases) as build, \
                mock.patch.object(window._queue, "start") as start:
            window.run_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
            for _ in range(5):
                app.processEvents()
        self.assertEqual(build.call_count, 1)
        start.assert_not_called()
        self.assertTrue(window.run_action.isEnabled())
        self.assertEqual(window.study_index.cases[0]["status"], "invalid")

    def test_panel_tire_swap_reaches_built_baseline(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        combo = window.model_panel._slot_widgets["tire"][0]
        index = next(i for i in range(combo.count()) if "hoosier-43075" in combo.itemData(i))
        combo.setCurrentIndex(index)
        window.build_cases()
        deadline = time.time() + 10
        while window._validation_thread is not None and time.time() < deadline:
            app.processEvents()
        self.assertEqual(Path(window.study_index.cases[0]["choices"]["tire"]).name, "hoosier-43075-r20-simple.xml")

    def test_open_study_restores_changed_model_tire_settings_and_recent(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        combo = window.model_panel._slot_widgets["tire"][0]
        combo.setCurrentIndex(next(i for i in range(combo.count()) if "hoosier-43075" in combo.itemData(i)))
        window.model_panel._model.setCurrentText("14dof-simple")
        definition = window._definition()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.study.json"
            path.write_text(json.dumps(definition.to_json()))
            with mock.patch("tools.sim_gui.qt.main_window.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                window.open_study()
            self.assertEqual(window.configuration.model, "14dof-simple")
            self.assertTrue(window.model_panel._slot_widgets["tire"][0].font().bold())
            self.assertEqual(window.settings_tab.settings(), definition.settings)
            self.assertIn(str(path), window._recent)

    def test_open_study_restores_track_and_built_case(self):
        window = self._window()
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        window.open_model(source)
        definition = window._definition()
        track = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"
        definition = definition.__class__(definition.name, definition.source, str(track), definition.model, definition.choices, definition.settings, definition.mode, definition.dimensions)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "track.study.json"
            path.write_text(json.dumps(definition.to_json()))
            with mock.patch("tools.sim_gui.qt.main_window.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                window.open_study()
            self.assertEqual(window.model_panel.track().resolve(), track.resolve())
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
            self.assertEqual(Path(window.study_index.cases[0]["track"]).resolve(), track.resolve())

    def test_point_mass_disables_run(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        self.assertFalse(window.run_action.isEnabled())

    def test_save_open_study_round_trip(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        window.open_model(source)
        definition = window._definition()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "definition.study.json"
            path.write_text(json.dumps(definition.to_json()))
            restored = StudyDefinition.from_json(json.loads(path.read_text()))
        self.assertEqual(restored.to_json(), definition.to_json())

    def test_validation_runs_in_thread_and_warning_and_cap_hooks(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        main_thread = QThread.currentThread()
        worker_threads = []
        original_run = ValidationWorker.run
        def record_thread(worker):
            worker_threads.append(QThread.currentThread())
            original_run(worker)
        with mock.patch.object(ValidationWorker, "run", record_thread):
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
        self.assertTrue(worker_threads)
        self.assertIsNot(worker_threads[0], main_thread)
        self.assertIsNone(window._validation_thread)
        self.assertFalse(window._validation_thread is not None and window._validation_thread.isRunning())
        # Closing the progress dialog on a normal finish must not count as a cancel.
        self.assertFalse(window._validation_cancelled)
        self.assertEqual(len(window._prepared), len(window.study_index.cases))
        prior_index = window.study_index
        many_cases = tuple(mock.Mock() for _ in range(51))
        with mock.patch("tools.sim_gui.qt.main_window.expand", return_value=many_cases), mock.patch("tools.sim_gui.qt.main_window._confirm", return_value=QMessageBox.StandardButton.No) as confirm:
            window.build_cases()
        confirm.assert_called_once()
        self.assertIs(window.study_index, prior_index)
        too_many = tuple(mock.Mock() for _ in range(501))
        with mock.patch("tools.sim_gui.qt.main_window.expand", return_value=too_many), mock.patch("tools.sim_gui.qt.main_window._warn") as warn:
            window.build_cases()
        warn.assert_called_once()
        self.assertIs(window.study_index, prior_index)

    def test_validation_cancel_stops_after_first_case(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        progress_count = []
        original = window._validation_progress
        def cancel_after_first(done, total):
            progress_count.append(done)
            if done == 1:
                window._cancel_validation()
            original(done, total)
        with mock.patch.object(window, "_validation_progress", side_effect=cancel_after_first):
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
        self.assertTrue(progress_count)
        self.assertEqual(progress_count, [1])
        self.assertEqual(window._prepared, [])
        self.assertTrue(window.run_action.isEnabled())

    def test_log_selection_tails_and_timer_tracks_running(self):
        window = self._window()
        with tempfile.TemporaryDirectory() as tmp:
            case_dir = Path(tmp)
            log = case_dir / "run.log"
            log.write_text("\n".join(str(i) for i in range(2500)))
            window.study_index = mock.Mock(directory=case_dir, cases=({"attempt": ".", "status": "running"},))
            window.queue_table.setRowCount(1)
            window.queue_table.selectRow(0)
            window._refresh_log()
            self.assertLessEqual(len(window.log_view.toPlainText().splitlines()), 2000)
            window._running = True
            window._log_timer.start()
            self.assertTrue(window._log_timer.isActive())
            window._all_finished()
            self.assertFalse(window._log_timer.isActive())

    def test_actions_shortcuts_output_folder_and_geometry_round_trip(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        window = self._window(store)
        self.assertEqual(window.open_action.shortcut().toString(), "Ctrl+O")
        self.assertEqual(window.save_action.shortcut().toString(), "Ctrl+S")
        self.assertEqual(window.run_action.shortcut().toString(), "F5")
        with mock.patch("tools.sim_gui.qt.main_window.QDesktopServices.openUrl") as opened:
            window.open_output_folder()
            opened.assert_called_once()
        window.resize(777, 555)
        window.close()
        restored = self._window(store)
        self.assertEqual(restored.size().width(), 777)

    def test_save_study_is_atomic_json(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.study.json"
            with mock.patch("tools.sim_gui.qt.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")):
                window.save_study()
            self.assertEqual(json.loads(path.read_text())["model"], "planar-7dof")
            self.assertFalse(path.with_suffix(path.suffix + ".tmp").exists())

    def test_close_no_keeps_window_and_yes_stops(self):
        from PyQt6.QtGui import QCloseEvent
        window = self._window()
        fake_process = mock.Mock()
        fake_process.Running = object()
        fake_process.state.return_value = fake_process.Running
        queue = mock.Mock()
        queue._running = {"case_000": mock.Mock(process=fake_process)}
        window._queue = queue
        window._running = True
        self.addCleanup(lambda: queue.shutdown())
        with mock.patch("tools.sim_gui.qt.main_window._confirm", return_value=QMessageBox.StandardButton.No):
            event = QCloseEvent()
            window.closeEvent(event)
        self.assertFalse(event.isAccepted())
        queue.stop.assert_not_called()
        queue.shutdown.assert_not_called()
        self.assertEqual(fake_process.state(), fake_process.Running)
        queue.allFinished.connect.reset_mock()
        with mock.patch("tools.sim_gui.qt.main_window._confirm", return_value=QMessageBox.StandardButton.Yes):
            with mock.patch("tools.sim_gui.qt.main_window.QEventLoop") as loop_type:
                event = QCloseEvent()
                window.closeEvent(event)
        self.assertTrue(event.isAccepted())
        queue.stop.assert_called_once()
        queue.shutdown.assert_called_once()
        window._running = False

    def test_recent_files_are_limited_to_ten(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        for index in range(12):
            window._remember(Path(f"/tmp/model-{index}.xml"))
        self.assertEqual(len(window._recent), 10)

    def _prepared(self, root, name="case_000", timeout=2.0):
        directory = Path(root) / name
        directory.mkdir(parents=True)
        return PreparedCase(directory, directory / "formulation.xml", directory / "vehicle.xml", [], timeout_seconds=timeout)

    def test_running_status_persisted_before_process_finishes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            from tools.sim_gui.study import StudyIndex
            definition = StudyDefinition("queue", "", "", "point-mass", {}, RunSettings(), "one-at-a-time", ())
            index = StudyIndex.create(root / "study", definition, (CaseSpec("case_000", "x", "point-mass", "", {}, {}),))
            prepared = PreparedCase(index.directory / "cases" / "case_000", root / "f", root / "v", [], timeout_seconds=5)
            prepared.directory.mkdir(parents=True)
            queue = CaseQueue(study_index=index, command_factory=lambda _p, _a: [sys.executable, "-c", "import time; time.sleep(30)"])
            queue.start([prepared], 1)
            app.processEvents()
            payload = json.loads((index.directory / "study.json").read_text())
            self.assertEqual(payload["cases"][0]["status"], "running")
            self.assertIn("attempt", payload["cases"][0])
            queue.stop()
            queue.shutdown()

    def test_parallel_limit_legacy(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = [self._prepared(tmp, f"case_{i}") for i in range(3)]
            command = [sys.executable, "-c", "import time; time.sleep(.2)"]
            queue = CaseQueue(command_factory=lambda _p, _a: command)
            active = [0]
            maximum = [0]
            queue.caseStarted.connect(lambda *_: (active.__setitem__(0, active[0] + 1), maximum.__setitem__(0, max(maximum[0], active[0]))))
            queue.caseFinished.connect(lambda *_: active.__setitem__(0, active[0] - 1))
            queue.start(prepared, 2)
            self._wait(queue)
            self.assertLessEqual(maximum[0], 2)
