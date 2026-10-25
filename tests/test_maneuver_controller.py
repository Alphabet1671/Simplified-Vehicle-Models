import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

if QApplication is not None:
    from tools.maneuver_gui.qt import ManeuverTab
    from tools.maneuver_gui.qt.controller import ManeuverController
    from tests.test_maneuver_tab import payload


@unittest.skipIf(QApplication is None, "PyQt6 is unavailable")
class ManeuverControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_start_snapshot_and_terminal_outcomes_restore_tab(self):
        tab = ManeuverTab(payload())
        self.addCleanup(tab.deleteLater)
        with tempfile.TemporaryDirectory() as tmp:
            controller = ManeuverController(tab, output_root=Path(tmp))
            emitted = []
            controller.finished.connect(emitted.append)
            definition = tab.definition()
            with patch.object(controller._job, "start") as start:
                controller.start(definition)
            self.assertTrue(tab.stop_button.isEnabled())
            arguments = start.call_args.args[0]
            self.assertIn("tools.maneuver_gui.runner", arguments)
            definition_path = Path(arguments[arguments.index("--definition") + 1])
            self.assertTrue(definition_path.exists())
            controller._finished(0, "done")
            self.assertFalse(tab.stop_button.isEnabled())
            self.assertEqual(emitted[-1]["execution_outcome"], "completed")
            self.assertFalse(definition_path.exists())
            controller._finished(9, "cancelled")
            self.assertEqual(emitted[-1]["execution_outcome"], "cancelled")

    def test_cancel_delegates_to_common_process_job(self):
        tab = ManeuverTab(payload())
        self.addCleanup(tab.deleteLater)
        controller = ManeuverController(tab, output_root=Path(tempfile.gettempdir()))
        with patch.object(controller._job, "cancel") as cancel:
            controller.cancel()
        cancel.assert_called_once()


if __name__ == "__main__":
    unittest.main()
