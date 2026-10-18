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
    from tools.sim_gui.settings import RunSettings
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
