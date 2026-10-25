import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

if QApplication is not None:
    from tools.maneuver_gui.qt import ManeuverTab, ManeuverEditor, PreviewWidget
    from tests.test_maneuver_tab import payload


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
        self.assertFalse(tab.stop_button.isEnabled())
        tab.set_busy(True)
        tab.stop_button.click()
        self.assertEqual(stopped, [True])
        tab.set_busy(False)

    def test_editor_definition_validation_and_round_trip(self):
        editor = ManeuverEditor(payload())
        definition = editor.definition()
        self.assertEqual(definition.to_dict(), payload())
        editor.load_definition(definition)
        self.assertEqual(editor.to_dict(), definition.to_dict())
        invalid = payload()
        invalid['clock']['controller_tick_s'] = 0
        editor.load_definition(invalid)
        with self.assertRaises(ValueError):
            editor.definition()
        self.assertTrue(editor.validation_label.text())

    def test_tab_public_api_busy_and_result(self):
        data = payload()
        tab = ManeuverTab()
        tab.set_vehicle_configuration(data)
        self.assertEqual(tab.definition().to_dict(), data)
        self.assertTrue(tab.run_button.isEnabled())
        tab.set_busy(True)
        self.assertFalse(tab.editor.isEnabled())
        self.assertFalse(tab.run_button.isEnabled())
        self.assertFalse(tab.calibration_button.isEnabled())
        self.assertTrue(tab.stop_button.isEnabled())
        tab.set_busy(False)
        self.assertTrue(tab.run_button.isEnabled())
        requested = {'steering': {'amplitude_rad': .2}}
        applied = {'steering': {'amplitude_rad': .19}}
        tab.set_result({'status': 'Finished', 'requested': requested, 'applied': applied})
        self.assertEqual(tab.status_label.text(), 'Finished')
        self.assertEqual(tab.preview.requested, requested)
        self.assertEqual(tab.preview.applied, applied)
        self.assertFalse(tab.hardware_arm_button.isEnabled())

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
