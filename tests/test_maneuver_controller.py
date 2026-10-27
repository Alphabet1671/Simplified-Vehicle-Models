import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import unittest
import csv
import json
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
import numpy as np

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

if QApplication is not None:
    from tools.maneuver_gui.qt import ManeuverTab
    from tools.maneuver_gui.qt.controller import ManeuverController
    from tools.maneuver_gui.artifacts import write_artifacts
    from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
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
            self.assertEqual(emitted[-1]["execution_outcome"], "failed")
            self.assertIn('expected exactly one completed maneuver attempt', emitted[-1]['error'])
            self.assertNotIn('run_record', emitted[-1])
            self.assertFalse(definition_path.exists())
            controller._finished(9, "cancelled")
            self.assertEqual(emitted[-1]["execution_outcome"], "cancelled")

    def test_progress_log_and_completed_artifact_populate_tab(self):
        tab = ManeuverTab(payload())
        self.addCleanup(tab.deleteLater)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            controller = ManeuverController(tab, output_root=root)
            controller._run_root = root
            attempt = root / 'attempt'; artifact = attempt / 'result'; artifact.mkdir(parents=True)
            definition = tab.definition().to_dict()
            definition['vehicle']['model'] = 'models/formulations/iem24_murphy_real_planar_7dof.xml'
            result = SimpleNamespace(
                time_s=np.array([0., .1]), state=np.zeros((2, 12)), state_rate=np.zeros((2, 12)),
                applied_commands=(RobotCommand(.1, .2, 0.), RobotCommand(.1, .2, 0.)),
                channels=({}, {}), execution_outcome='completed', termination_reason='duration')
            write_artifacts(artifact, definition, result, {}, model_kind='planar-7dof')
            from simplified_vehicle_models.results.maneuver_run_record import write_maneuver_run_record
            write_maneuver_run_record(attempt / 'run_record.npz', definition, artifact,
                                      model_kind='planar-7dof')
            controller._line(json.dumps({'event': 'progress', 'fraction': .5, 'time_s': .1, 'phase': 'hold'}))
            self.assertEqual(tab.progress_bar.value(), 50)
            self.assertIn('hold', tab.log_view.toPlainText())
            received = []; controller.finished.connect(received.append)
            controller._finished(0, 'done')
            self.assertEqual(received[-1]['execution_outcome'], 'completed')
            self.assertEqual(received[-1]['artifact'], str(artifact))
            self.assertFalse(hasattr(tab, 'animation_button'))
            self.assertFalse(hasattr(controller, '_animation_job'))
            self.assertFalse(hasattr(controller, 'export_animation'))
            self.assertEqual(received[-1]['run_record'], str(attempt / 'run_record.npz'))
            self.assertEqual(len(tab.preview.requested_commands), 2)

    def test_cancel_delegates_to_common_process_job(self):
        tab = ManeuverTab(payload())
        self.addCleanup(tab.deleteLater)
        controller = ManeuverController(tab, output_root=Path(tempfile.gettempdir()))
        with patch.object(controller._job, "cancel") as cancel:
            controller.cancel()
        cancel.assert_called_once()


if __name__ == "__main__":
    unittest.main()
