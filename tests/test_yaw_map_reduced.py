import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus
from simplified_vehicle_models.formulations.yaw_map_reduced import direct_reduced_cell

ROOT = Path(__file__).resolve().parents[1]


class ReducedYawMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = compile_formulation(str(ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        cls.vehicle = point_mass_view_of_planar(model)

    def test_thousands_of_rear_no_tv_cells_match_direct_lp_status_and_moments(self):
        # Include boundary-adjacent random targets as well as normal map range.
        from simplified_vehicle_models.formulations.yaw_capability import direct_yaw_cell
        rng = np.random.default_rng(2061026)
        points = [(float(rng.uniform(0., 33.3)), float(rng.uniform(-6., 3.)),
                   float(rng.uniform(-26., 26.))) for _ in range(1500)]
        points += [(v, ax, ay) for v in (0., 5., 15., 33.) for ax in (-6., -3., 0., 3.)
                   for ay in np.linspace(-25., 25., 25)]
        for speed, ax, ay in points:
            with self.subTest(speed=speed, ax=ax, ay=ay):
                reduced = direct_reduced_cell(self.vehicle, speed, ax, ay)
                direct = direct_yaw_cell(self.vehicle, speed, ax, ay)
                self.assertEqual(reduced.status, direct.status)
                if direct.status is ProbeStatus.FEASIBLE:
                    self.assertAlmostEqual(reduced.into_turn, direct.into_turn, places=7)
                    self.assertAlmostEqual(reduced.out_of_turn, direct.out_of_turn, places=7)

    def test_near_motor_limit_perturbations_agree_with_canonical_lp(self):
        from simplified_vehicle_models.formulations.yaw_capability import direct_yaw_cell
        # Locate the active rear-drive boundary and perturb it by 1e-9 m/s².
        # The conservative reduced route must use the canonical verdict if its
        # interval arithmetic is boundary-conditioned.
        speed = 15.
        lo, hi = 0., 5.
        for _ in range(45):
            middle = (lo + hi) / 2.
            if direct_yaw_cell(self.vehicle, speed, middle, 0.).status is ProbeStatus.FEASIBLE:
                lo = middle
            else:
                hi = middle
        for ax in (lo - 1e-9, lo + 1e-9):
            with self.subTest(ax=ax):
                self.assertEqual(direct_reduced_cell(self.vehicle, speed, ax, 0.).status,
                                 direct_yaw_cell(self.vehicle, speed, ax, 0.).status)

    def test_reduced_cache_reuses_canonical_ambiguous_verdict(self):
        from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
        from simplified_vehicle_models.formulations.yaw_map_reduced import _ReducedAmbiguous
        evaluator=PhysicalYawEvaluator(self.vehicle,use_reduced=True)
        sentinel=DirectYawCell(ProbeStatus.UNRESOLVED,detail='canonical numerical evidence')
        with patch('simplified_vehicle_models.formulations.yaw_map_reduced._interval_for_axle',
                   side_effect=_ReducedAmbiguous('fixture active boundary')), \
             patch('simplified_vehicle_models.formulations.yaw_map_reduced.direct_yaw_cell',return_value=sentinel) as direct:
            self.assertIs(evaluator.cell(15.,0.,0.),sentinel)
            self.assertIs(evaluator.cell(15.,0.,0.),sentinel)
        direct.assert_called_once()

    def test_nonrear_or_tv_configurations_fall_back_to_direct_evaluator(self):
        sentinel = DirectYawCell(ProbeStatus.UNRESOLVED, detail="fallback")
        with patch("simplified_vehicle_models.formulations.yaw_map_reduced.direct_yaw_cell", return_value=sentinel) as direct:
            result = direct_reduced_cell(self.vehicle, 5., 0., 0., ideal=True)
        self.assertIs(result, sentinel)
        direct.assert_called_once()


if __name__ == "__main__":
    unittest.main()
