import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.execute import PreparedCase, child_command
from tools.sim_gui.run_case import _point_mass
from tools.sim_gui.settings import RunSettings


class GuiCaseRunnerTests(unittest.TestCase):
    def test_child_invokes_renamed_adapter(self):
        case, attempt = Path('/tmp/case'), Path('/tmp/attempt')
        command = child_command(PreparedCase(case, case/'f.xml', case/'v.xml', [], solver_kind='point-mass'), attempt)
        self.assertEqual(command[-4:], ['--case-dir', str(case), '--child-attempt', str(attempt)])
        self.assertTrue(command[2].endswith('tools/sim_gui/run_case.py'))

    def test_point_mass_boundary_rejects_open_track_before_solver(self):
        settings = RunSettings(solver_kind='point-mass')
        with mock.patch('tools.sim_gui.run_case._resolve', return_value=REPOSITORY_ROOT / 'models/tracks/2024_michigan_endurance_closed.xml'), \
             mock.patch('simplified_vehicle_models.tracks.profile.load_track', return_value=mock.Mock(is_closed=False)):
            with self.assertRaisesRegex(ValueError, 'closed tracks only'):
                _point_mass({'track':'track.xml','model':'planar-7dof','formulation':'model/formulation.xml'},
                            Path('/tmp/attempt'), settings, Path('/tmp/case'))

    def test_run_case_rejects_retired_solver_kind(self):
        from tools.sim_gui.run_case import run_case
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); case = root/'case'; case.mkdir(); attempt = root/'attempt'
            (case/'case.json').write_text('{"solver_kind":"yaw-aware","settings":{}}')
            with self.assertRaisesRegex(ValueError, 'retired yaw workflow'):
                run_case(case, attempt)


if __name__ == '__main__':
    unittest.main()
