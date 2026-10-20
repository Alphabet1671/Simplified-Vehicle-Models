import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from PyQt6.QtCore import QEventLoop, QPoint, QRectF, Qt, QTimer
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None

try:
    from scipy.interpolate import CubicSpline  # noqa: F401
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


def wait_until(predicate, timeout_ms=5000):
    loop = QEventLoop()
    timer = QTimer()
    timer.timeout.connect(lambda: loop.quit() if predicate() else None)
    timer.start(10)
    QTimer.singleShot(timeout_ms, loop.quit)
    loop.exec()
    return predicate()

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])

if QApplication is not None:
    import numpy as np
    from simplified_vehicle_models.tracks.profile import load_track
    from tools.track_gui.mat_channels import ChannelInfo
    from tools.track_gui.track_ops import TrackData
    from tools.track_gui.qt.main_window import MainWindow
    from tools.track_gui.qt.plots import MARGIN


def circle_track(nodes=400, radius=50.0):
    angle = np.linspace(0.0, 2.0 * np.pi, nodes)
    north = radius * np.cos(angle)
    east = radius * np.sin(angle)
    north[-1], east[-1] = north[0], east[0]
    distance = np.r_[0.0, np.cumsum(np.hypot(np.diff(north), np.diff(east)))]
    return TrackData(distance, north, east, np.unwrap(angle), np.full(nodes, 1.0 / radius), True, "Circle", 300.0)

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class TrackGuiToolsQtTests(unittest.TestCase):
    def setUp(self):
        self.window = MainWindow()
        self.addCleanup(self.window.close)

    def track(self, closed=True, xy=True):
        d=np.array([0.,1.,2.,3.]); n=np.array([0.,1.,1.,0.]) if xy else None; e=np.array([0.,0.,1.,0.]) if xy else None
        return TrackData(d,n,e,np.zeros(4),np.array([.1,.1,-.1,-.1]),closed,"Fixture",100.)

    def test_closed_tab_exposes_channel_mode_controls_and_unknown_flag(self):
        tab=self.window.closed_tab
        self.assertTrue(hasattr(tab,"x_channel")); self.assertTrue(hasattr(tab,"mode"))
        tab.set_channels([ChannelInfo("VNorth","m/s",100,0,10),ChannelInfo("VEast","m/s",100,0,10)])
        self.assertTrue(any("VNorth" in tab.x_channel.itemText(i) for i in range(tab.x_channel.count())))
        self.assertFalse(tab.x_unknown.isChecked())

    def test_changing_the_telemetry_path_clears_discovered_channels(self):
        tab = self.window.closed_tab
        tab.set_channels([ChannelInfo("VNorth", "m/s", 100, 0, 10), ChannelInfo("VEast", "m/s", 100, 0, 10)])
        self.assertEqual(tab.x_channel.count(), 2)
        tab.telemetry.setText("/tmp/another-telemetry.mat")
        self.assertEqual(tab.x_channel.count(), 0)
        self.assertEqual(tab.y_channel.count(), 0)
        self.assertTrue(tab._discovery_required)
        with self.assertRaisesRegex(ValueError, "Discover channels"):
            tab._selection()

    def test_failed_discovery_clears_stale_choices(self):
        tab = self.window.closed_tab
        with tempfile.TemporaryDirectory() as folder:
            missing = Path(folder) / "missing.mat"
            tab.telemetry.setText(str(missing))
            tab.set_channels([ChannelInfo("VNorth", "m/s", 100, 0, 10), ChannelInfo("VEast", "m/s", 100, 0, 10)])
            tab.discover_channels()
            self.assertTrue(wait_until(lambda: tab.discovery_thread is None))
        self.assertEqual(tab.x_channel.count(), 0)
        self.assertEqual(tab.y_channel.count(), 0)
        self.assertTrue(tab._discovery_required)
        self.assertIn("Could not discover channels", tab.notice.text())

    @unittest.skipUnless(HAS_SCIPY, "SciPy is required to write a synthetic MAT file")
    def test_async_discovery_populates_metadata_and_default_axes(self):
        from scipy.io import savemat
        tab = self.window.closed_tab
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "channels.mat"
            clock = np.array([0.0, 1.0, 2.0])
            savemat(path, {
                "VNorth": {"Time": clock, "Value": np.array([1.0, 1.0, 1.0]), "Units": "m/s"},
                "VEast": {"Time": clock, "Value": np.array([0.0, 0.0, 0.0]), "Units": "m/s"},
            })
            tab.telemetry.setText(str(path))
            tab.discover_channels()
            self.assertTrue(wait_until(lambda: tab.discovery_thread is None))
        self.assertEqual(tab.x_channel.count(), 2)
        self.assertEqual(tab.x_channel.currentData(), "VNorth")
        self.assertEqual(tab.y_channel.currentData(), "VEast")
        self.assertFalse(tab._discovery_required)
        self.assertIn("Discovered 2 channels", tab.notice.text())

    def test_initial_discovery_stale_after_path_change_cannot_fall_back_to_legacy(self):
        tab = self.window.closed_tab
        release = threading.Event()

        def delayed_discovery(path):
            release.wait(5)
            return [ChannelInfo("VNorth", "m/s", 100, 0, 10), ChannelInfo("VEast", "m/s", 100, 0, 10)]

        try:
            with patch("tools.track_gui.qt.main_window.list_channels", side_effect=delayed_discovery):
                tab.telemetry.setText("/tmp/initial-discovery.mat")
                tab.discover_channels()
                self.assertIsNotNone(tab.discovery_thread)
                tab.telemetry.setText("/tmp/replaced-telemetry.mat")
                release.set()
                self.assertTrue(wait_until(lambda: tab.discovery_thread is None))
        finally:
            release.set()
        self.assertEqual(tab.x_channel.count(), 0)
        self.assertTrue(tab._discovery_required)
        with self.assertRaisesRegex(ValueError, "Discover channels"):
            tab._selection()
        tab.build()
        self.assertIsNone(tab.thread)
        self.assertIn("Discover channels", tab.notice.text())

    def test_out_of_date_discovery_result_is_discarded(self):
        tab = self.window.closed_tab
        tab.telemetry.setText("/tmp/first-telemetry.mat")
        tab.set_channels([ChannelInfo("A", "m/s", 100, 0, 10)])
        generation = tab._discovery_generation
        tab.telemetry.setText("/tmp/second-telemetry.mat")
        tab._discovered(str(Path("/tmp/first-telemetry.mat")), [ChannelInfo("B", "m/s", 100, 0, 10)], generation)
        self.assertEqual(tab.x_channel.count(), 0)
        self.assertIn("Discarded", tab.notice.text())

    def test_untouched_legacy_defaults_work_but_explicit_mode_requires_discovery(self):
        tab=self.window.closed_tab
        self.assertIsNone(tab._selection())
        tab.mode.setCurrentText("gps")
        with self.assertRaisesRegex(ValueError,"Discover channels"):
            tab._selection()

    def test_closed_settings_snapshot_passes_explicit_selection_and_multi_controls(self):
        tab=self.window.closed_tab
        with tempfile.NamedTemporaryFile(suffix=".mat") as f:
            tab.telemetry.setText(f.name)
            tab.x_channel.addItem("north","north"); tab.x_channel.setCurrentIndex(tab.x_channel.count()-1)
            tab.y_channel.addItem("east","east"); tab.y_channel.setCurrentIndex(tab.y_channel.count()-1)
            tab.mode.setCurrentText("position"); tab.x_scale.setValue(2.); tab.y_scale.setValue(3.)
            tab.x_unknown.setChecked(True); tab.multi_lap.setChecked(True); tab.return_radius.setValue(17.)
            with patch("tools.track_gui.qt.main_window._BuildWorker") as worker:
                tab.build()
                settings=worker.call_args.args[0]
                self.assertEqual(settings.channel_selection.x_channel,"north")
                self.assertEqual(settings.channel_selection.mode,"position")
                self.assertEqual(settings.channel_selection.x_scale,2.)
                self.assertTrue(settings.channel_selection.x_allow_unknown_units)
                self.assertTrue(settings.multi_lap); self.assertEqual(settings.return_radius,17.)
                tab.thread.quit(); tab.thread.wait(); tab.thread=None

    def test_close_event_keeps_window_when_discovery_will_not_stop(self):
        tab = self.window.closed_tab
        tab.discovery_thread = type("StuckDiscovery", (), {
            "quit": lambda self: None,
            "wait": lambda self, timeout: False,
        })()
        event = type("Event", (), {
            "ignore": lambda self: setattr(self, "ignored", True),
            "accept": lambda self: setattr(self, "accepted", True),
        })()
        with patch("tools.track_gui.qt.main_window.QMessageBox.question",
                   return_value=QMessageBox.StandardButton.Yes):
            self.window.closeEvent(event)
        self.assertTrue(getattr(event, "ignored", False))
        self.assertFalse(getattr(event, "accepted", False))
        self.assertIn("discovery is still running", tab.notice.text())
        tab.discovery_thread = None

    def test_parameter_edit_invalidates_previous_closed_result(self):
        tab=self.window.closed_tab
        tab.geometry={"north":[0,1],"east":[0,1]}; tab.save_button.setEnabled(True); tab.transfer_button.setEnabled(True)
        tab.spacing.setValue(tab.spacing.value()+.1)
        self.assertIsNone(tab.geometry); self.assertFalse(tab.save_button.isEnabled()); self.assertFalse(tab.transfer_button.isEnabled())

    def test_viewer_load_reverse_and_unsigned_notice(self):
        viewer=self.window.viewer_tab
        viewer.set_track(self.track())
        self.assertTrue(viewer.reverse_button.isEnabled())
        self.assertTrue(viewer.split_button.isEnabled())
        before=viewer.track.curvature.copy(); viewer.reverse()
        np.testing.assert_allclose(viewer.track.curvature,-before[[0,2,1,0]])
        viewer.set_track(self.track(xy=False))
        self.assertIn("unsigned",viewer.notice.text().lower())
        self.assertFalse(viewer.split_button.isEnabled())

    def test_failed_viewer_load_clears_actionable_track(self):
        viewer=self.window.viewer_tab; viewer.set_track(self.track())
        with patch("tools.track_gui.qt.track_viewer.load_track_data",side_effect=ValueError("bad file")):
            viewer.load_path("/bad/file.xml")
        self.assertIsNone(viewer.track)
        self.assertFalse(viewer.reverse_button.isEnabled())

    @unittest.skipUnless(HAS_SCIPY, "SciPy is required to save a closed track")
    def test_actual_viewer_save_writes_a_loadable_closed_track(self):
        viewer = self.window.viewer_tab
        viewer.set_track(circle_track())
        with tempfile.TemporaryDirectory() as folder:
            viewer.save_bar.folder.setText(folder)
            viewer.save_bar.stem.setText("viewer_saved")
            viewer.save_bar.model_id.setText("viewer-saved")
            viewer.save_bar.title.setText("Viewer saved")
            viewer.save()
            xml = Path(folder) / "viewer_saved.xml"
            self.assertTrue(xml.is_file(), viewer.notice.text())
            self.assertTrue((Path(folder) / "viewer_saved.csv").is_file())
            self.assertTrue(load_track(xml).is_closed)
            self.assertIn("Saved", viewer.notice.text())

    def test_viewer_save_uses_core_track_data_writer(self):
        viewer=self.window.viewer_tab; viewer.set_track(self.track(closed=False))
        with patch("tools.track_gui.qt.track_viewer.save_track_data") as save:
            save.return_value=type("Saved",(),{"xml":"/tmp/open.xml"})()
            viewer.save()
        save.assert_called_once()
        self.assertTrue(viewer.notice.text().startswith("Saved"))

    def test_actual_mouse_click_equal_edit_and_remove_update_both_plots(self):
        viewer = self.window.viewer_tab
        track = circle_track()
        viewer.set_track(track)
        self.window.resize(1100, 800)
        self.window.tabs.setCurrentWidget(viewer)
        self.window.show()
        QApplication.processEvents()
        rect = QRectF(MARGIN, 8, max(viewer.map.width() - MARGIN - 8, 10),
                      max(viewer.map.height() - MARGIN - 8, 10))
        to_point, _ = viewer.map.transform(rect)
        node = 100
        point = to_point(track.x_north[node], track.y_east[node])
        QTest.mouseClick(viewer.map, Qt.MouseButton.LeftButton,
                         pos=QPoint(round(point.x()), round(point.y())))
        QApplication.processEvents()
        self.assertEqual(len(viewer.cuts), 1)
        self.assertAlmostEqual(viewer.cuts[0], float(track.distance[node]), delta=1.5)
        self.assertEqual(len(viewer.map.boundary_points), 1)
        self.assertEqual(len(viewer.profile.boundaries), 1)

        viewer.equal_count.setValue(4)
        QTest.mouseClick(viewer.equal_button, Qt.MouseButton.LeftButton)
        QApplication.processEvents()
        self.assertEqual(len(viewer.cuts), 4)
        self.assertEqual(viewer.cut_table.rowCount(), 4)
        self.assertEqual(len(viewer.map.boundary_points), 4)
        self.assertEqual(len(viewer.profile.boundaries), 4)

        replacement = float(track.length) * 0.625
        viewer.cut_table.item(1, 0).setText(f"{replacement:.6f}")
        QApplication.processEvents()
        self.assertTrue(any(np.isclose(cut, replacement, atol=1e-5) for cut in viewer.cuts))
        self.assertEqual(len(viewer.map.boundary_points), 4)
        self.assertEqual(len(viewer.profile.boundaries), 4)

        viewer.cut_table.setCurrentCell(0, 0)
        QTest.mouseClick(viewer.remove_button, Qt.MouseButton.LeftButton)
        QApplication.processEvents()
        self.assertEqual(len(viewer.cuts), 3)
        self.assertEqual(viewer.cut_table.rowCount(), 3)
        self.assertEqual(len(viewer.map.boundary_points), 3)
        self.assertEqual(len(viewer.profile.boundaries), 3)

    def test_section_export_refuses_existing_targets_before_core_write(self):
        viewer=self.window.viewer_tab; viewer.set_track(self.track())
        viewer.cuts=[0.,1.,2.]; viewer._refresh_cuts()
        with tempfile.TemporaryDirectory() as folder:
            viewer.save_bar.folder.setText(folder); viewer.save_bar.stem.setText("part")
            Path(folder,"part_01.xml").write_text("existing")
            viewer.save_bar.overwrite.setChecked(False)
            with patch("tools.track_gui.qt.track_viewer.save_sections") as save, \
                 patch("tools.track_gui.qt.track_viewer.QMessageBox.warning") as warning:
                viewer.export_sections()
            save.assert_not_called(); warning.assert_called_once()

    @unittest.skipUnless(HAS_SCIPY, "SciPy is required for section reconstruction")
    def test_actual_gui_section_export_writes_loadable_numbered_pairs(self):
        viewer = self.window.viewer_tab
        track = circle_track()
        viewer.set_track(track)
        viewer.cuts = [track.length * i / 4.0 for i in range(4)]
        viewer._refresh_cuts()
        with tempfile.TemporaryDirectory() as folder:
            viewer.save_bar.folder.setText(folder)
            viewer.save_bar.stem.setText("sections")
            viewer.save_bar.model_id.setText("sections")
            viewer.save_bar.title.setText("Sections")
            viewer.export_sections()
            paths = [Path(folder) / f"sections_{index:02d}.xml" for index in range(1, 5)]
            self.assertTrue(all(path.is_file() for path in paths), viewer.notice.text())
            self.assertTrue(all((Path(folder) / f"sections_{index:02d}.csv").is_file()
                                for index in range(1, 5)))
            profiles = [load_track(path) for path in paths]
            self.assertTrue(all(not profile.is_closed for profile in profiles))
            self.assertTrue(all(profile.distance.size > 2 for profile in profiles))
            self.assertIn("Exported 4 sections", viewer.notice.text())

    @unittest.skipUnless(HAS_SCIPY, "SciPy is required for section reconstruction")
    def test_export_requires_overwrite_and_generated_confirmation(self):
        viewer = self.window.viewer_tab
        track = circle_track()
        viewer.set_track(track)
        viewer.cuts = [track.length * i / 4.0 for i in range(4)]
        viewer._refresh_cuts()
        with tempfile.TemporaryDirectory() as folder:
            viewer.save_bar.folder.setText(folder)
            viewer.save_bar.stem.setText("sections")
            viewer.save_bar.model_id.setText("sections")
            viewer.save_bar.title.setText("Sections")
            viewer.export_sections()
            target = Path(folder) / "sections_01.xml"
            generated_text = ('<?xml version="1.0"?><!-- Generated by tools/test_fixture.py. -->'
                              '<model schema-version="1.0" id="old" type="track-model" '
                              'subtype="open-course"><metadata><title>Old</title></metadata></model>')
            target.write_text(generated_text, encoding="utf-8")
            viewer.save_bar.overwrite.setChecked(False)
            with patch("tools.track_gui.qt.track_viewer.QMessageBox.warning") as warning:
                viewer.export_sections()
            warning.assert_called_once()
            self.assertEqual(target.read_text(encoding="utf-8"), generated_text)

            viewer.save_bar.overwrite.setChecked(True)
            with patch("tools.track_gui.qt.track_viewer.QMessageBox.question",
                       side_effect=[QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No]) as question:
                viewer.export_sections()
            self.assertEqual(question.call_count, 2)
            self.assertEqual(target.read_text(encoding="utf-8"), generated_text)

            with patch("tools.track_gui.qt.track_viewer.QMessageBox.question",
                       side_effect=[QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.Yes]):
                viewer.export_sections()
            self.assertNotIn("Generated by tools/test_fixture.py", target.read_text(encoding="utf-8"))
            for index in range(1, 5):
                self.assertFalse(load_track(Path(folder) / f"sections_{index:02d}.xml").is_closed)

if __name__ == "__main__": unittest.main()
