import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None

@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])

if QApplication is not None:
    import numpy as np
    from tools.track_gui.mat_channels import ChannelInfo
    from tools.track_gui.track_ops import TrackData
    from tools.track_gui.qt.main_window import MainWindow

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

    def test_viewer_save_uses_core_track_data_writer(self):
        viewer=self.window.viewer_tab; viewer.set_track(self.track(closed=False))
        with patch("tools.track_gui.qt.track_viewer.save_track_data") as save:
            save.return_value=type("Saved",(),{"xml":"/tmp/open.xml"})()
            viewer.save()
        save.assert_called_once()
        self.assertTrue(viewer.notice.text().startswith("Saved"))

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

if __name__ == "__main__": unittest.main()
