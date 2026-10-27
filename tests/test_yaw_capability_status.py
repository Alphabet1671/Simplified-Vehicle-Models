"""Direct LP status is evidence, not a feasibility mask."""
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations import yaw_capability as yc
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus

ROOT = Path(__file__).resolve().parents[1]


class DirectStatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = compile_formulation(str(ROOT / 'models/formulations/iem24_murphy_real_planar_7dof.xml')).model
        cls.vehicle = point_mass_view_of_planar(model)

    def test_only_infeasible_status_is_physical_infeasibility(self):
        for status in (1, 2, 3, 4):
            with self.subTest(status=status), patch.object(yc, '_require_linprog', return_value=lambda *a, **k:
                    SimpleNamespace(success=False, status=status, message='fixture')):
                cell = yc.direct_yaw_cell(self.vehicle, 10., 0., 0.)
            self.assertEqual(cell.status, ProbeStatus.INFEASIBLE if status == 2 else ProbeStatus.UNRESOLVED)

    def test_success_objective_and_zero_moment_containment(self):
        for objective, expected in ((-10., ProbeStatus.FEASIBLE), (2., ProbeStatus.INFEASIBLE),
                                    (float('nan'), ProbeStatus.UNRESOLVED)):
            with self.subTest(objective=objective), patch.object(yc, '_require_linprog', return_value=lambda *a, **k:
                    SimpleNamespace(success=True, status=0, fun=objective)):
                cell = yc.direct_yaw_cell(self.vehicle, 10., 0., 0.)
            self.assertEqual(cell.status, expected)

    def test_builder_does_not_silently_mask_numerical_failure(self):
        with patch.object(yc, '_require_linprog', return_value=lambda *a, **k:
                SimpleNamespace(success=False, status=4, message='numerical problem')):
            with self.assertRaises(yc.YawCapabilityNumericalError):
                yc.build_physical_yaw_map(self.vehicle, yc.YawCapabilityGrid([10.], [0.], [0.]), yaw_inertia=1.)

    def test_invalid_query_is_not_physical_infeasibility(self):
        with self.assertRaises(ValueError):
            yc.direct_yaw_cell(self.vehicle, 10., float('nan'), 0.)
