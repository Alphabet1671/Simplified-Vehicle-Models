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
    from tools.sim_gui.qt.single_run_tab import SingleRunTab
    from tools.sim_gui.qt.track_tab import TrackTab
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

    def test_point_mass_model_is_supported_source(self):
        point_mass = VehicleConfiguration.open(
            REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml"
        )
        panel = ModelPanel(point_mass)
        try:
            self.assertEqual(panel._model.currentText(), "point-mass")
            self.assertEqual(panel._error.text(), "")
            emitted = []
            panel.configurationChanged.connect(emitted.append)
            panel._model.setCurrentText("planar-7dof")
            self.assertEqual(emitted, [panel.configuration()])
        finally:
            panel.deleteLater()

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class TrackTabTests(unittest.TestCase):
    def setUp(self):
        self.tab = TrackTab()
        self.addCleanup(self.tab.deleteLater)

    def test_initial_track_history_is_empty_and_session_local(self):
        self.assertEqual(self.tab._combo.count(), 0)
        self.assertIsNone(self.tab.track())
        self.assertEqual(self.tab.preview.points(), ())
        second = TrackTab()
        try:
            self.assertEqual(second._combo.count(), 0)
            self.assertIsNone(second.track())
            path = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
            self.tab.set_track(path)
            self.tab.set_track(path.parent / ".." / "tracks" / path.name)
            self.assertEqual(self.tab._combo.count(), 1)
            self.assertEqual(second._combo.count(), 0)
        finally:
            second.deleteLater()

    def test_browse_selects_a_legacy_track_without_a_centerline(self):
        path = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"
        with mock.patch(
            "tools.sim_gui.qt.track_tab.QFileDialog.getOpenFileName",
            return_value=(str(path), ""),
        ):
            self.tab._browse.click()
        self.assertEqual(self.tab.track(), path.resolve())
        self.assertEqual(self.tab.preview.points(), ())
        self.assertTrue(self.tab.preview.message())

    def test_invalid_file_warns_and_restores(self):
        before_track = self.tab.track()
        before_points = self.tab.preview.points()
        before_count = self.tab._combo.count()
        emitted = []
        self.tab.trackChanged.connect(emitted.append)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "not_a_track.xml"
            path.write_text('<model type="aero" subtype="simple" id="bad" />')
            with mock.patch(
                "tools.sim_gui.qt.track_tab.QFileDialog.getOpenFileName",
                return_value=(str(path), ""),
            ), mock.patch("tools.sim_gui.qt.track_tab._warn") as warn:
                self.tab._browse.click()
            warn.assert_called_once()
        self.assertEqual(self.tab.track(), before_track)
        self.assertEqual(self.tab.preview.points(), before_points)
        self.assertEqual(emitted, [])
        self.assertEqual(self.tab._combo.count(), before_count)

    def test_preview_paints_the_centerline(self):
        self.tab.set_track(REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
        from PyQt6.QtGui import QImage
        preview = self.tab.preview
        preview.resize(320, 240)
        image = QImage(320, 240, QImage.Format.Format_RGB32)
        image.fill(0)
        preview.render(image)
        colours = {image.pixel(x, y)
                   for x in range(0, 320, 4) for y in range(0, 240, 4)}
        self.assertGreater(len(colours), 1)

    def test_track_changed_emits_the_resolved_path(self):
        self.tab.set_track(REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
        emitted = []
        self.tab.trackChanged.connect(emitted.append)
        target = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_multilap_closed.xml"
        self.tab.set_track(target)
        self.assertEqual(emitted, [target.resolve()])


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class SingleRunTabTests(unittest.TestCase):
    def setUp(self):
        self.tab = SingleRunTab()
        self.addCleanup(self.tab.deleteLater)
        self.model = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        self.track = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"

    def _index_of(self, combo, path):
        target = Path(path).resolve()
        for index in range(combo.count()):
            value = combo.itemData(index)
            if value and Path(value).resolve() == target:
                return index
        raise AssertionError(f"{path} is not in the combo")

    def test_defaults_and_signals(self):
        self.assertEqual(self.tab.model_combo.count(), 0)
        self.assertEqual(self.tab.track_combo.count(), 0)
        self.assertIsNone(self.tab.model_path())
        self.assertIsNone(self.tab.track_path())
        second = SingleRunTab()
        try:
            self.assertEqual(second.model_combo.count(), 0)
            self.assertEqual(second.track_combo.count(), 0)
        finally:
            second.deleteLater()
        self.assertEqual(self.tab.export_folder(), REPOSITORY_ROOT / "runs")
        go, stop, models, chosen_tracks = [], [], [], []
        self.tab.goRequested.connect(go.append)
        self.tab.stopRequested.connect(lambda: stop.append(True))
        self.tab.modelSelected.connect(models.append)
        self.tab.trackSelected.connect(chosen_tracks.append)
        self.tab.set_model(self.model)
        self.tab.set_track(self.track)
        self.tab.set_model(self.model.parent / ".." / "formulations" / self.model.name)
        self.tab.set_track(self.track.parent / ".." / "tracks" / self.track.name)
        self.assertEqual(self.tab.model_combo.count(), 1)
        self.assertEqual(self.tab.track_combo.count(), 1)
        self.tab.set_progress(1.5)
        self.assertEqual(models, [])
        self.assertEqual(chosen_tracks, [])
        self.assertEqual(self.tab.model_path(), self.model.resolve())
        self.assertEqual(self.tab.track_path(), self.track.resolve())
        self.assertEqual(self.tab.progress_bar.value(), 100)
        with tempfile.TemporaryDirectory() as tmp:
            self.tab.set_export_folder(Path(tmp))
            self.assertEqual(go, [])
            self.tab.go_button.click()
            self.assertEqual(go, [Path(tmp)])
        self.assertTrue(self.tab.go_button.isEnabled())
        self.assertFalse(self.tab.stop_button.isEnabled())
        self.tab.set_running(True)
        self.assertFalse(self.tab.go_button.isEnabled())
        self.assertTrue(self.tab.stop_button.isEnabled())
        self.tab.set_running(False)
        self.assertTrue(self.tab.go_button.isEnabled())
        self.assertFalse(self.tab.stop_button.isEnabled())

    def test_bad_model_and_track_are_refused(self):
        self.tab.set_model(self.model)
        self.tab.set_track(self.track)
        before_model = self.tab.model_path()
        before_track = self.tab.track_path()
        before_model_count = self.tab.model_combo.count()
        before_track_count = self.tab.track_combo.count()
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.xml"
            bad.write_text('<model type="aero" subtype="simple" id="bad" />')
            with mock.patch(
                "tools.sim_gui.qt.single_run_tab.QFileDialog.getOpenFileName",
                return_value=(str(bad), ""),
            ), mock.patch("tools.sim_gui.qt.single_run_tab._warn") as warn:
                self.tab.model_browse.click()
            warn.assert_called_once()
            with mock.patch(
                "tools.sim_gui.qt.single_run_tab.QFileDialog.getOpenFileName",
                return_value=(str(bad), ""),
            ), mock.patch("tools.sim_gui.qt.single_run_tab._warn") as warn:
                self.tab.track_browse.click()
            warn.assert_called_once()
        self.assertEqual(self.tab.model_path(), before_model)
        self.assertEqual(self.tab.track_path(), before_track)
        self.assertEqual(self.tab.model_combo.count(), before_model_count)
        self.assertEqual(self.tab.track_combo.count(), before_track_count)


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class SettingsTabTests(unittest.TestCase):
    def setUp(self):
        self.tab = SettingsTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_workflow_fields_and_replay_visibility(self):
        self.assertIn("completed, identity-checked", self.tab.prerequisite.text())
        self.assertIn("sampled/discrete", self.tab.limitation.text())
        self.assertEqual(self.tab.solver_kind.currentData(), "yaw-aware")
        self.assertTrue(self.tab.plan_row.isHidden())
        self.tab.mode.setCurrentText("replay")
        self.assertFalse(self.tab.plan_row.isHidden())
        self.assertEqual(self.tab._settings.mode, "replay")
        self.assertTrue(hasattr(self.tab, "profile_seconds"))
        self.assertTrue(hasattr(self.tab, "point_mass_maximum_speed"))
        self.assertFalse(hasattr(self.tab, "planner"))
        self.assertFalse(hasattr(self.tab, "quasi_static_attitude"))

    def test_workflow_switch_and_model_validation(self):
        emitted = []
        self.tab.settingsChanged.connect(emitted.append)
        self.tab.solver_kind.setCurrentIndex(self.tab.solver_kind.findData("point-mass"))
        self.assertEqual(self.tab._settings.solver_kind, "point-mass")
        self.tab.set_model("14dof-simple")
        self.assertEqual(self.tab.settings().solver_kind, "point-mass")
        self.assertTrue(emitted)
        self.assertTrue(self.tab.profile_seconds.isHidden())
        self.tab.solver_kind.setCurrentIndex(self.tab.solver_kind.findData("yaw-aware"))
        self.assertFalse(self.tab.profile_seconds.isHidden())

    def test_map_browser_selects_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map.npz"
            with mock.patch("tools.sim_gui.qt.settings_tab.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                self.tab._browse_map()
            self.assertEqual(self.tab.yaw_map.text(), str(path))


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
        window.track_tab.set_track(REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
        self.addCleanup(window.close)
        return window

    def _wait(self, queue, timeout=5):
        loop = QEventLoop()
        queue.allFinished.connect(loop.quit)
        QTimer.singleShot(int(timeout * 1000), loop.quit)
        loop.exec()
        queue.shutdown()

    def _until(self, condition, timeout=60):
        deadline = time.time() + timeout
        while not condition() and time.time() < deadline:
            app.processEvents()
            time.sleep(0.01)
        self.assertTrue(condition())

    def test_main_window_point_mass_source_workflow_gate(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        window.settings_tab.solver_kind.setCurrentIndex(window.settings_tab.solver_kind.findData("point-mass"))
        self.assertTrue(window.go_action.isEnabled())
        with mock.patch("tools.sim_gui.qt.main_window._warn") as warn:
            window.build_cases()
            warn.assert_called_once()
            self.assertIn("point-mass adaptation unavailable", warn.call_args.args[2])
        window.settings_tab.solver_kind.setCurrentIndex(window.settings_tab.solver_kind.findData("yaw-aware"))
        self.assertFalse(window.go_action.isEnabled())

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
        window.track_tab.set_track(REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
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
        self.assertTrue(window.go_action.isEnabled())
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
            self.assertEqual(window.track_tab.track().resolve(), track.resolve())
            window.build_cases()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
            self.assertEqual(Path(window.study_index.cases[0]["track"]).resolve(), track.resolve())

    def test_build_cases_requires_an_explicit_track(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.track_tab._combo.clear()
        window.single_tab.set_track(None)
        with mock.patch("tools.sim_gui.qt.main_window._warn") as warn, \
                mock.patch("tools.sim_gui.qt.main_window.expand") as expand_mock:
            window.build_cases()
        warn.assert_called_once()
        self.assertIn("Select a track", warn.call_args.args[2])
        expand_mock.assert_not_called()

    def test_definition_uses_the_track_tab(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        track = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
        window.track_tab.set_track(track)
        self.assertEqual(Path(window._definition().track).resolve(), track.resolve())

    def test_single_run_completes_and_writes_a_record(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.track_tab.set_track(REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
        window.settings_tab.duration.setValue(0.3)
        window.settings_tab.timeout_seconds.setValue(120.0)
        export = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(export, ignore_errors=True))
        window.single_tab.set_export_folder(export)
        window.single_tab.go_button.click()
        self._wait(window._queue, 120)
        self.assertEqual(window.single_tab.status_label.text(), "completed")
        directory = window.study_index.directory
        self.assertEqual(directory.parent, export)
        self.assertTrue(directory.name.startswith("single-run_"))
        records = list(export.glob("single-run_*/cases/case_000/attempt_*/run_record.npz"))
        self.assertEqual(len(records), 1)
        self.assertEqual(window.queue_table.rowCount(), 0)
        log_text = window.single_tab.log_view.toPlainText()
        self.assertTrue(log_text)
        self.assertLessEqual(len(log_text.splitlines()), 2000)

    def test_single_run_does_not_start_without_a_track(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.single_tab.set_track(None)
        self.assertIsNone(window.single_tab.track_path())
        with mock.patch("tools.sim_gui.qt.main_window._warn") as warn, \
                mock.patch.object(window._queue, "start") as start:
            window.run_single()
        warn.assert_called_once()
        start.assert_not_called()
        self.assertIsNone(window._run_mode)

    def test_missing_export_folder_is_created(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        missing = Path(tempfile.mkdtemp()) / "missing"
        self.addCleanup(lambda: shutil.rmtree(missing.parent, ignore_errors=True))
        window.single_tab.set_export_folder(missing)
        with mock.patch.object(ValidationWorker, "run", lambda worker: worker.finished.emit()), \
                mock.patch("tools.sim_gui.qt.main_window._warn") as warn:
            window.run_single()
            deadline = time.time() + 10
            while window._validation_thread is not None and time.time() < deadline:
                app.processEvents()
        self.assertTrue(missing.is_dir())
        warn.assert_not_called()
        self.assertEqual(window.settings_store.value("single_run_output"), str(missing))
        self.assertIsNone(window._run_mode)

    def test_build_cases_does_not_start_twice(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.build_cases()
        first = window._validation_thread
        self.assertIsNotNone(first)
        window.build_cases()
        self.assertIs(window._validation_thread, first)
        deadline = time.time() + 20
        while window._validation_thread is not None and time.time() < deadline:
            app.processEvents()
        self.assertIsNone(window._validation_thread)

    def test_model_and_track_sync_both_ways(self):
        window = self._window()
        single = window.single_tab
        planar = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        first_model = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
        window.open_model(planar)
        window.open_model(first_model)
        index = next(
            i for i in range(single.model_combo.count())
            if Path(single.model_combo.itemData(i)).name == planar.name
        )
        single.model_combo.setCurrentIndex(index)
        self.assertIsNotNone(window.configuration)
        self.assertEqual(window.configuration.source.resolve(), planar.resolve())
        self.assertEqual(single.model_path().resolve(), planar.resolve())
        track = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
        other = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_multilap_closed.xml"
        window.track_tab.set_track(other)
        index = next(
            i for i in range(single.track_combo.count())
            if Path(single.track_combo.itemData(i)).resolve() == track.resolve()
        )
        single.track_combo.setCurrentIndex(index)
        self.assertEqual(window.track_tab.track().resolve(), track.resolve())
        self.assertEqual(Path(window._definition().track).resolve(), track.resolve())
        self.assertEqual(single.track_path().resolve(), track.resolve())
        window.track_tab.set_track(other)
        self.assertEqual(single.track_path().resolve(), other.resolve())

    def test_stop_cancels_a_single_run(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.settings_tab.duration.setValue(60.0)
        export = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(export, ignore_errors=True))
        window.single_tab.set_export_folder(export)
        window.single_tab.go_button.click()
        self._until(lambda: bool(window._queue._running), 90)
        window.single_tab.stop_button.click()
        self._wait(window._queue, 15)
        self.assertEqual(window.single_tab.status_label.text(), "cancelled")
        self.assertTrue(window.single_tab.go_button.isEnabled())
        self.assertEqual(window.queue_table.rowCount(), 0)

    def test_study_run_still_uses_the_queue(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.settings_tab.duration.setValue(0.3)
        window.settings_tab.timeout_seconds.setValue(120.0)
        window.build_cases()
        self._until(lambda: window._validation_thread is None, 60)
        window.run_cases()
        self._wait(window._queue, 120)
        self.assertEqual(window.queue_table.rowCount(), 1)
        self.assertEqual(window.queue_table.item(0, 2).text(), "completed")
        self.assertEqual(window.single_tab.status_label.text(), "")
        self.assertEqual(window.single_tab.log_view.toPlainText(), "")

    def test_all_finished_only_updates_the_active_mode(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        window.queue_table.setRowCount(1)
        window.log_view.setPlainText("study log")
        window.single_tab.set_status("single status")
        window._run_mode = "study"
        window._all_finished()
        self.assertEqual(window.single_tab.status_label.text(), "single status")
        self.assertEqual(window.queue_table.rowCount(), 1)
        self.assertEqual(window.log_view.toPlainText(), "study log")
        window._run_mode = "single"
        window._all_finished()
        self.assertEqual(window.queue_table.rowCount(), 1)
        self.assertEqual(window.log_view.toPlainText(), "study log")

    def test_single_run_refuses_an_export_folder_inside_models(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        folder = REPOSITORY_ROOT / "models" / f"tmp-export-{time.time_ns()}"
        self.addCleanup(lambda: shutil.rmtree(folder, ignore_errors=True))
        window.single_tab.set_export_folder(folder)
        with mock.patch("tools.sim_gui.qt.main_window._warn") as warn, \
                mock.patch.object(window._queue, "start") as start:
            window.run_single()
        warn.assert_called_once()
        self.assertIn("inside models/", warn.call_args.args[2])
        start.assert_not_called()
        self.assertFalse(folder.exists())
        self.assertIsNone(window._run_mode)

    def test_single_run_allows_a_models_sweeps_export_folder(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        folder = REPOSITORY_ROOT / "models/sweeps" / f"single-run-test-{time.time_ns()}"
        self.addCleanup(lambda: shutil.rmtree(folder, ignore_errors=True))
        folder.mkdir(parents=True)
        window.single_tab.set_export_folder(folder)
        try:
            with mock.patch("tools.sim_gui.qt.main_window._warn") as warn, \
                    mock.patch.object(window._queue, "start") as start:
                window.run_single()
                self._until(lambda: start.called, 120)
                warn.assert_not_called()
                start.assert_called_once()
        finally:
            window.single_tab.set_running(False)
            window._single_log_timer.stop()
            window._run_mode = None
            window._running = False
            window._refresh_actions()

    def test_open_model_twice_keeps_one_study_tab(self):
        window = self._window()
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        window.open_model(source)
        first = window.study_tab
        window.open_model(source)
        names = [window.tabs.tabText(index) for index in range(window.tabs.count())]
        self.assertEqual(names.count("Study"), 1)
        self.assertIsNot(window.study_tab, first)

    def test_single_run_tab_is_first_and_track_tab_is_second(self):
        window = self._window()
        self.assertEqual(window.tabs.tabText(0), "Single Run")
        self.assertEqual(window.tabs.tabText(1), "Track")

    def test_point_mass_source_can_run_point_mass_but_not_yaw_aware(self):
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(store_dir, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        window.settings_tab.solver_kind.setCurrentIndex(window.settings_tab.solver_kind.findData("point-mass"))
        self.assertTrue(window.go_action.isEnabled())
        window.settings_tab.solver_kind.setCurrentIndex(window.settings_tab.solver_kind.findData("yaw-aware"))
        self.assertFalse(window.go_action.isEnabled())

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
        self.assertTrue(window.go_action.isEnabled())

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
        self.assertEqual(window.go_action.shortcut().toString(), "F5")
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
        store.setValue("recent_files", ["/tmp/prior-session-model.xml"])
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(output, ignore_errors=True))
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        self.assertEqual(window.single_tab.model_combo.count(), 0)
        self.assertEqual(window.single_tab.track_combo.count(), 0)
        self.assertEqual(window.track_tab._combo.count(), 0)
        self.assertEqual(window._recent, ["/tmp/prior-session-model.xml"])
        for index in range(12):
            window._remember(Path(f"/tmp/model-{index}.xml"))
        self.assertEqual(len(window._recent), 10)

    def test_maneuver_tab_requires_model_not_track_and_shares_configuration(self):
        # Construct directly: unlike _window(), this test intentionally selects no track.
        store_dir = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(store_dir, ignore_errors=True))
        output = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: shutil.rmtree(output, ignore_errors=True))
        store = QSettings(str(store_dir / "settings.ini"), QSettings.Format.IniFormat)
        window = MainWindow(output_root=output, settings_store=store)
        self.addCleanup(window.close)
        self.assertIsNone(window.track_tab.track())
        labels = [window.tabs.tabText(i) for i in range(window.tabs.count())]
        self.assertIn("Maneuvers", labels)
        self.assertIn("Single Run", labels)
        self.assertFalse(window.maneuver_tab.run_button.isEnabled())
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        self.assertTrue(window.maneuver_tab.run_button.isEnabled())
        self.assertEqual(window.maneuver_tab.editor.to_dict()["vehicle"]["model"],
                         "models/formulations/iem24_murphy_real_planar_7dof.xml")
        # Maneuvers intentionally have no Track precondition; this window never selected one.
        self.assertTrue(window.maneuver_tab.run_button.isEnabled())

    def test_maneuver_lifecycle_excludes_other_runs_and_restores_controls(self):
        window = self._window()
        window.open_model(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")
        definition = window.maneuver_tab.definition()
        with mock.patch.object(window.maneuver_controller, "start") as start:
            window._run_maneuver(definition)
        start.assert_called_once_with(definition)
        self.assertEqual(window._run_mode, "maneuver")
        self.assertFalse(window.go_action.isEnabled())
        window._maneuver_busy_changed(False)
        self.assertIsNone(window._run_mode)
        self.assertTrue(window.go_action.isEnabled())

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
