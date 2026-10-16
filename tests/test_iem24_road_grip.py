from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from simplified_vehicle_models.tires.unitire_simple import (
    compile_unitire_simple,
    evaluate_unitire,
)
from simplified_vehicle_models.xml.loader import load_model


ROOT = Path(__file__).resolve().parents[1]
ROAD_PATH = ROOT / "models/tires/iem24_murphy_real_unitire.xml"
LAB_PATH = ROOT / "models/tires/hoosier-43075-r20-simple.xml"
MU_NAMES = ("mu_0_x", "mu_s_x", "mu_0_y", "mu_s_y")
ROAD_FACTOR = 2.0 / 3.0


def _raw(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    return {
        node.get("name"): float(node.get("value"))
        for node in root.iter("parameter")
    }


class Iem24RoadGripTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.lab = _raw(LAB_PATH)
        cls.road = _raw(ROAD_PATH)
        cls.lab_parameters = compile_unitire_simple(load_model(LAB_PATH))
        cls.road_parameters = compile_unitire_simple(load_model(ROAD_PATH))

    def test_friction_is_exactly_two_thirds_of_the_lab_fit(self) -> None:
        for name in MU_NAMES:
            self.assertAlmostEqual(
                self.road[name] / self.lab[name],
                ROAD_FACTOR,
                delta=1e-15,
                msg=name,
            )
        self.assertAlmostEqual(self.road["mu_0_x"], 1.55744, places=5)
        self.assertAlmostEqual(self.road["mu_0_y"], 1.58692, places=5)
        self.assertAlmostEqual(self.road["mu_s_x"], 1.34396, places=5)
        self.assertAlmostEqual(self.road["mu_s_y"], 1.36904, places=5)

    def test_only_friction_differs_from_the_lab_fit(self) -> None:
        self.assertEqual(set(self.road), set(self.lab))
        for name, value in self.lab.items():
            if name not in MU_NAMES:
                self.assertEqual(self.road[name], value, msg=name)

    def test_proposed_status_and_proxy_are_disclosed(self) -> None:
        text = ROAD_PATH.read_text(encoding="utf-8")
        for phrase in (
            "PROPOSED",
            "not a measured or validated road coefficient",
            "2/3",
            "Contents_Round9.pdf",
            "43100",
        ):
            self.assertIn(phrase, text)

    def test_friction_cap_bounds_force_per_load(self) -> None:
        slip_angle = np.linspace(-0.5, 0.5, 101)
        slip_ratio = np.linspace(-1.0, 1.0, 101)
        for load in (400.0, 660.0, 1000.0):
            lateral = evaluate_unitire(
                self.road_parameters, slip_angle, 0.0, load, 50.0
            ).lateral_force
            longitudinal = evaluate_unitire(
                self.road_parameters, 0.0, slip_ratio, load, 50.0
            ).longitudinal_force
            self.assertLessEqual(
                float(np.max(np.abs(lateral))) / load,
                self.road_parameters.mu_0_y * (1.0 + 1e-9),
            )
            self.assertLessEqual(
                float(np.max(np.abs(longitudinal))) / load,
                self.road_parameters.mu_0_x * (1.0 + 1e-9),
            )
            # The cap is approached, so the lower grip is really in force.
            self.assertGreater(
                float(np.max(np.abs(lateral))) / load,
                0.99 * self.road_parameters.mu_0_y,
            )
            self.assertGreater(
                float(np.max(np.abs(longitudinal))) / load,
                0.97 * self.road_parameters.mu_0_x,
            )

            grid_a, grid_k = np.meshgrid(slip_angle, slip_ratio)
            combined = evaluate_unitire(
                self.road_parameters, grid_a, grid_k, load, 50.0
            )
            ellipse = np.hypot(
                combined.longitudinal_force / self.road_parameters.mu_0_x,
                combined.lateral_force / self.road_parameters.mu_0_y,
            )
            self.assertLessEqual(
                float(np.max(ellipse)) / load, 1.0 + 1e-9
            )

    def test_road_peak_force_is_two_thirds_of_lab_peak(self) -> None:
        angles = np.linspace(0.0, 0.5, 101)
        for load in (400.0, 660.0, 1000.0):
            road = evaluate_unitire(
                self.road_parameters, angles, 0.0, load, 50.0
            ).lateral_force
            lab = evaluate_unitire(
                self.lab_parameters, angles, 0.0, load, 50.0
            ).lateral_force
            self.assertAlmostEqual(
                float(np.max(np.abs(road)) / np.max(np.abs(lab))),
                ROAD_FACTOR,
                delta=5e-3,
            )

    def test_stiffness_is_unchanged_so_small_slip_force_is_not_scaled(
        self,
    ) -> None:
        # Linear region: stiffness dominates, so forces nearly match the lab.
        road = evaluate_unitire(
            self.road_parameters, 0.002, 0.0, 660.0, 50.0
        ).lateral_force
        lab = evaluate_unitire(
            self.lab_parameters, 0.002, 0.0, 660.0, 50.0
        ).lateral_force
        self.assertAlmostEqual(float(road / lab), 1.0, delta=0.05)

    def test_legacy_lab_fit_is_untouched(self) -> None:
        self.assertEqual(self.lab["mu_0_x"], 2.3361624600071096)
        self.assertEqual(self.lab["mu_s_x"], 2.0159412226800204)
        self.assertEqual(self.lab["mu_0_y"], 2.3803804777987394)
        self.assertEqual(self.lab["mu_s_y"], 2.053563821658717)


if __name__ == "__main__":
    unittest.main()
