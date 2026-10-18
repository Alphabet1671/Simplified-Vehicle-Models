import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
from pathlib import Path
import sys
import tempfile
import time

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
    from PyQt6.QtCore import QSettings, QEventLoop, QTimer
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
    
    def test_main_window_end_to_end_case(self):
        import shutil
        output = REPOSITORY_ROOT / "runs" / "_test"
        if output.exists():
            shutil.rmtree(output)
        self.addCleanup(lambda: shutil.rmtree(output) if output.exists() else None)
        store = QSettings(str(Path(tempfile.mkdtemp()) / "settings.ini"), QSettings.Format.IniFormat)
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

    def test_point_mass_disables_run(self):
        store = QSettings(str(Path(tempfile.mkdtemp()) / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(settings_store=store)
        self.addCleanup(window.close)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        self.assertFalse(window.run_action.isEnabled())

    def test_save_open_study_round_trip(self):
        store = QSettings(str(Path(tempfile.mkdtemp()) / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(settings_store=store)
        self.addCleanup(window.close)
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        window.open_model(source)
        definition = window._definition()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "definition.study.json"
            path.write_text(json.dumps(definition.to_json()))
            restored = StudyDefinition.from_json(json.loads(path.read_text()))
        self.assertEqual(restored.to_json(), definition.to_json())

    def test_close_no_keeps_window_and_yes_stops(self):
        from PyQt6.QtGui import QCloseEvent
        store = QSettings(str(Path(tempfile.mkdtemp()) / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(settings_store=store)
        self.addCleanup(window.close)
        window._running = True
        with mock.patch("tools.sim_gui.qt.main_window._confirm", return_value=QMessageBox.StandardButton.No):
            event = QCloseEvent()
            window.closeEvent(event)
            self.assertTrue(event.isAccepted() is False)
        window._running = False

    def test_recent_files_are_limited_to_ten(self):
        store = QSettings(str(Path(tempfile.mkdtemp()) / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(settings_store=store)
        self.addCleanup(window.close)
        for index in range(12):
            window._remember(Path(f"/tmp/model-{index}.xml"))
        self.assertEqual(len(window._recent), 10)

    def _prepared(self, root, name="case_000", timeout=2.0):
        directory = Path(root) / name
        directory.mkdir(parents=True)
        return PreparedCase(directory, directory / "formulation.xml", directory / "vehicle.xml", [], timeout_seconds=timeout)

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
