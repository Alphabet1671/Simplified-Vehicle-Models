import json
import shutil
import subprocess
import unittest
from pathlib import Path

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.execute import child_command, new_attempt, prepare_case
from tools.sim_gui.settings import RunSettings
from tools.sim_gui.study import CaseSpec, StudyDefinition
from tools.sim_gui.vehicle import VehicleConfiguration

try:
    import scipy  # noqa: F401
except ImportError:
    SCIPY_AVAILABLE = False
else:
    SCIPY_AVAILABLE = True


@unittest.skipUnless(SCIPY_AVAILABLE, "scipy is required for the forward-driver integration test")
class RunCaseIntegrationTests(unittest.TestCase):
    study_dir = REPOSITORY_ROOT / "runs" / "_test"

    def tearDown(self):
        path = self.study_dir
        self.assertTrue(str(path).endswith("runs/_test"))
        if path.exists():
            shutil.rmtree(path)

    def test_planar_child_writes_verified_record_with_hoosier(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        baseline = VehicleConfiguration.open(source)
        choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix()
                   for key, path in baseline.choices.items()}
        choices["tire"] = "models/tires/hoosier-43075-r20-simple.xml"
        definition = StudyDefinition(
            "_test", source.as_posix(),
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", choices,
            RunSettings(duration=0.3, timeout_seconds=120.0), "one-at-a-time", (),
        )
        case = CaseSpec("case_000", "hoosier", "planar-7dof", definition.track,
                        choices, {})
        prepared = prepare_case(self.study_dir, definition, case)
        attempt = new_attempt(prepared)
        result = subprocess.run(child_command(prepared, attempt), cwd=REPOSITORY_ROOT,
                                env=child_environment(), timeout=180)
        self.assertEqual(result.returncode, 0)
        record = attempt / "run_record.npz"
        self.assertTrue(record.is_file())
        from simplified_vehicle_models.results.run_record import load_run_record
        loaded = load_run_record(record, root=REPOSITORY_ROOT)
        sources = {str(path) for path in loaded.manifest.sources}
        self.assertIn("models/tires/hoosier-43075-r20-simple.xml", sources)


if __name__ == "__main__":
    unittest.main()
