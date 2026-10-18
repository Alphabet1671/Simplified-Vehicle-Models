import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
from pathlib import Path
from unittest import mock

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
    from tools.sim_gui.qt.model_panel import ModelPanel
    from tools.sim_gui.qt.settings_tab import SettingsTab


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

    def test_incompatible_candidate_warns_and_restores(self):
        combo = self._combo("tire")
        old = combo.currentIndex()
        combo.addItem("incompatible", str(REPOSITORY_ROOT / "models/aero/aero_2024.xml"))
        with mock.patch("tools.sim_gui.qt.model_panel._warn") as warn:
            combo.setCurrentIndex(combo.count() - 1)
        warn.assert_called_once()
        self.assertEqual(combo.currentIndex(), old)


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
