import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from PyQt6.QtCore import QEventLoop, QTimer
    from PyQt6.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])


if QApplication is not None:
    from tools.track_gui import closed
    from tools.track_gui.export import SaveRefused
    from tools.track_gui.design import SectionSpec
    from tools.track_gui.qt.main_window import MainWindow


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class WindowTests(unittest.TestCase):
    def setUp(self):
        self.window = MainWindow()
        self.addCleanup(self.window.close)
        self.tab = self.window.design_tab

    def test_starts_with_a_valid_preset(self):
        self.assertIsNotNone(self.tab.course)
        self.assertTrue(self.tab.save_button.isEnabled())
        self.assertIn("m,", self.tab.info.text())

    def test_every_preset_loads(self):
        for index in range(self.tab.preset.count()):
            self.tab.preset.setCurrentIndex(index)
            self.tab.load_preset()
            self.assertIsNotNone(self.tab.course, self.tab.preset.currentText())

    def test_a_bad_edit_shows_the_message_and_blocks_saving(self):
        self.tab.table.set_sections([SectionSpec("straight", "a", length=5.0), SectionSpec("arc", "b", length=5.0, curvature=0.1)])
        self.tab.jumps.setChecked(False)
        self.assertIsNone(self.tab.course)
        self.assertIn("curvature jumps", self.tab.problem.text())
        self.assertFalse(self.tab.save_button.isEnabled())
        self.tab.jumps.setChecked(True)
        self.assertIsNotNone(self.tab.course)

    def test_save_writes_a_loadable_track(self):
        with tempfile.TemporaryDirectory() as folder:
            bar = self.tab.save_bar
            bar.folder.setText(folder)
            bar.stem.setText("gui_test")
            bar.model_id.setText("gui-test")
            bar.title.setText("GUI test")
            messages = []
            self.tab.status.connect(messages.append)
            self.tab.save()
            self.assertTrue((Path(folder) / "gui_test.xml").is_file())
            self.assertIn("saved", messages[-1])

    def test_closed_tab_without_telemetry_reports_the_problem(self):
        tab = self.window.closed_tab
        tab.telemetry.setText("")
        tab.build()
        self.assertTrue(tab.notice.text())
        self.assertIsNone(tab.thread)

    def test_closed_build_clears_old_geometry_before_starting(self):
        tab = self.window.closed_tab
        tab.geometry = {"old": True}
        tab.save_button.setEnabled(True)
        tab.telemetry.setText("")
        tab.build()
        self.assertIsNone(tab.geometry)
        self.assertFalse(tab.save_button.isEnabled())

    def test_closed_build_save_handles_export_oserror(self):
        tab = self.window.closed_tab
        tab.geometry = {"old": True}
        tab.settings = object()
        tab.save_button.setEnabled(True)
        with patch("tools.track_gui.qt.main_window.closed.save_closed", side_effect=OSError("disk full")), \
                patch("tools.track_gui.qt.main_window.QMessageBox.warning") as warning:
            tab.save()
        warning.assert_called_once()
        self.assertIn("disk full", warning.call_args.args[2])

    def test_app_reports_bad_design_file(self):
        from tools.track_gui import app as app_module
        os.environ["TRACK_GUI_QUIT_AFTER_MS"] = "1"
        self.addCleanup(os.environ.pop, "TRACK_GUI_QUIT_AFTER_MS", None)
        self.assertEqual(app_module.main(["/no/such/design.json"]), 0)

    def test_close_event_keeps_window_when_build_thread_will_not_stop(self):
        tab = self.window.closed_tab
        thread = type("StuckThread", (), {"quit": lambda self: None, "wait": lambda self, timeout: False})()
        tab.thread = thread
        event = type("Event", (), {"ignore": lambda self: setattr(self, "ignored", True), "accept": lambda self: setattr(self, "accepted", True)})()
        with patch("tools.track_gui.qt.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes):
            self.window.closeEvent(event)
        self.assertTrue(getattr(event, "ignored", False))
        self.assertFalse(getattr(event, "accepted", False))
        self.assertIn("still running", tab.notice.text())
        tab.thread = None

    def test_design_store_refuses_protected_folder_without_confirmation(self):
        with patch("tools.track_gui.qt.main_window.QFileDialog.getSaveFileName", return_value=("/tmp/design.json", "")), \
                patch("tools.track_gui.qt.main_window.protected_directory_problem", return_value="protected"), \
                patch("tools.track_gui.qt.main_window.QMessageBox.warning") as warning:
            self.tab.store_design()
        warning.assert_called_once()

    def test_design_store_requires_overwrite_confirmation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "design.json"
            path.write_text("old", encoding="utf-8")
            with patch("tools.track_gui.qt.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")), \
                    patch("tools.track_gui.qt.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.No):
                self.tab.store_design()
            self.assertEqual(path.read_text(encoding="utf-8"), "old")

    def test_design_store_allows_confirmed_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "design.json"
            path.write_text("old", encoding="utf-8")
            with patch("tools.track_gui.qt.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")), \
                    patch("tools.track_gui.qt.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes):
                self.tab.store_design()
            self.assertIn("track-design-v1", path.read_text(encoding="utf-8"))

    def test_design_store_handles_oserror(self):
        with patch("tools.gui_common.files.atomic_write_text", side_effect=OSError("read only")), \
                patch("tools.track_gui.qt.main_window.QFileDialog.getSaveFileName", return_value=("/tmp/design.json", "")), \
                patch("tools.track_gui.qt.main_window.QMessageBox.warning") as warning:
            self.tab.store_design()
        warning.assert_called_once()
        self.assertIn("read only", warning.call_args.args[2])

    def test_closed_save_confirms_generated_replacement(self):
        tab = self.window.closed_tab
        tab.geometry = {"geometry": True}
        tab.settings = object()
        values = tab.save_bar.values()
        refusal = SaveRefused("/tmp/track.xml is generated by tools/build.py. Change the file name, or confirm")
        saved = type("Saved", (), {"xml": "/tmp/track.xml", "length": 10.0, "node_count": 3})()
        with patch("tools.track_gui.qt.main_window.closed.save_closed", side_effect=[refusal, saved]) as save, \
                patch("tools.track_gui.qt.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes):
            tab.save()
        self.assertEqual(save.call_count, 2)
        self.assertTrue(save.call_args.kwargs["overwrite_generated"])

    def test_closed_save_declines_generated_replacement(self):
        tab = self.window.closed_tab
        tab.geometry = {"geometry": True}
        tab.settings = object()
        refusal = SaveRefused("/tmp/track.xml is generated by tools/build.py. Change the file name, or confirm")
        with patch("tools.track_gui.qt.main_window.closed.save_closed", side_effect=refusal) as save, \
                patch("tools.track_gui.qt.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.No):
            tab.save()
        save.assert_called_once()

    def test_closed_tab_builds_and_saves_a_circle(self):
        if not closed.scipy_available():
            self.skipTest("SciPy is not installed")
        from tests.test_track_gui_closed import write_circle
        tab = self.window.closed_tab
        with tempfile.TemporaryDirectory() as folder:
            mat = Path(folder) / "c.mat"
            write_circle(mat)
            tab.telemetry.setText(str(mat))
            tab.end.setValue(60.0)
            tab.build()
            loop = QEventLoop()
            timer = QTimer()
            timer.timeout.connect(lambda: loop.quit() if tab.geometry is not None else None)
            timer.start(50)
            QTimer.singleShot(15000, loop.quit)
            loop.exec()
            self.assertIsNotNone(tab.geometry, tab.notice.text())
            self.assertIn("minimum_radius_m", tab.report.toPlainText())
            tab.save_bar.folder.setText(folder)
            tab.save_bar.stem.setText("c_out")
            tab.save_bar.model_id.setText("c-out")
            tab.save_bar.title.setText("C out")
            tab.save()
            self.assertTrue((Path(folder) / "c_out.xml").is_file())

    def test_app_main_quits_after_the_hook(self):
        from tools.track_gui import app as app_module
        os.environ["TRACK_GUI_QUIT_AFTER_MS"] = "50"
        self.addCleanup(os.environ.pop, "TRACK_GUI_QUIT_AFTER_MS", None)
        self.assertEqual(app_module.main([]), 0)


if __name__ == "__main__":
    unittest.main()
