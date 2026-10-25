import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

if QApplication is not None:
    from tools.maneuver_gui.qt import ManeuverTab, PreviewWidget


@unittest.skipIf(QApplication is None, "PyQt6 is unavailable")
class ManeuverQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_tab_controls_and_signals(self):
        tab = ManeuverTab()
        self.assertFalse(tab.run_button.isEnabled())
        self.assertFalse(tab.hardware_arm_button.isEnabled())
        self.assertEqual(tab.backend_label.text(), "Backend: simulation")
        stopped = []
        tab.stopRequested.connect(lambda: stopped.append(True))
        tab.stop_button.click()
        self.assertEqual(stopped, [True])

    def test_preview_keeps_requested_and_applied_distinct(self):
        preview = PreviewWidget()
        requested = {"steering": {"amplitude_rad": .2}}
        applied = {"steering": {"amplitude_rad": .19}}
        preview.set_state(requested, applied)
        self.assertEqual(preview.requested, requested)
        self.assertEqual(preview.applied, applied)
        requested["steering"]["amplitude_rad"] = .1
        self.assertEqual(preview.requested["steering"]["amplitude_rad"], .2)


if __name__ == "__main__":
    unittest.main()
