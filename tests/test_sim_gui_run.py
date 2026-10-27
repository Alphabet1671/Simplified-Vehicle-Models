import json
import os
import shutil
import subprocess
import sys
import time
import unittest
from unittest import mock
from pathlib import Path

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.execute import child_command, new_attempt, prepare_case
from tools.sim_gui.settings import RunSettings
from tools.sim_gui.study import CaseSpec, StudyDefinition, StudyIndex, expand
from tools.sim_gui.run_study import run_study, main as run_study_main, _study_directory, default_parallel
from tools.sim_gui.execute import PreparedCase
from tools.sim_gui.vehicle import VehicleConfiguration

try:
    import scipy  # noqa: F401
except ImportError:
    SCIPY_AVAILABLE = False
else:
    SCIPY_AVAILABLE = True


class HeadlessRunnerUnitTests(unittest.TestCase):
    def setUp(self):
        self.root = REPOSITORY_ROOT / "runs" / "_test"
        self.assertTrue(str(self.root).endswith("runs/_test"))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)

    def definition(self, name="unit"):
        return StudyDefinition(name, "", "", "point-mass", {}, RunSettings(), "one-at-a-time", ())

    def test_study_directory_uses_local_timestamp_and_suffixes_clashes(self):
        first = _study_directory(self.root, self.definition())
        second = _study_directory(self.root, self.definition())
        self.assertRegex(first.name, r"^unit_\d{8}-\d{6}$")
        self.assertEqual(second.name, first.name + "_2")

    def test_worker_failure_is_recorded_and_other_cases_finish(self):
        definition = self.definition("failure")
        fake_dir = self.root / "failure_000" / "cases" / "case_000"
        fake_dir.mkdir(parents=True)
        fake = PreparedCase(fake_dir, fake_dir / "f.xml", fake_dir / "v.xml", ["--record"])
        with mock.patch("tools.sim_gui.run_study.prepare_case", return_value=fake), \
             mock.patch("tools.sim_gui.run_study.validate_case", return_value=None), \
             mock.patch("tools.sim_gui.run_study.child_command", side_effect=RuntimeError("boom")):
            output = run_study(definition, output_root=self.root, parallel=1)
        case = json.loads((output / "study.json").read_text())["cases"][0]
        self.assertEqual(case["status"], "failed")
        self.assertEqual(case["error"], "RuntimeError: boom")

    def test_all_invalid_cli_returns_one(self):
        definition = self.definition("all-invalid")
        path = self.root / "definition.study.json"
        self.root.mkdir(parents=True)
        path.write_text(json.dumps(definition.to_json()))
        with mock.patch("tools.sim_gui.run_study.validate_case", return_value="bad"):
            self.assertEqual(run_study_main(["--definition", str(path), "--output-root", str(self.root)]), 1)

    def test_default_parallel_uses_bounded_cpu_half_and_cli_default(self):
        with mock.patch("tools.sim_gui.run_study.os.cpu_count", side_effect=[1, 2, 8, 32, None]):
            self.assertEqual([default_parallel() for _ in range(5)], [1, 1, 4, 4, 1])
        definition = self.definition("parallel-cli")
        path = self.root / "definition.study.json"
        self.root.mkdir(parents=True)
        path.write_text(json.dumps(definition.to_json()))
        with mock.patch("tools.sim_gui.run_study.run_study", return_value=self.root / "out") as run:
            (self.root / "out").mkdir()
            (self.root / "out" / "study.json").write_text(json.dumps({"cases": [{"status": "completed"}]}))
            with mock.patch("tools.sim_gui.run_study.os.cpu_count", return_value=8):
                self.assertEqual(run_study_main(["--definition", str(path), "--output-root", str(self.root)]), 0)
            self.assertEqual(run.call_args.args[2], 4)

    def test_interrupt_in_one_run_does_not_stop_another_run(self):
        import threading
        items = {}
        for name in ("conc-a", "conc-b"):
            directory = self.root / name / "case_000"
            directory.mkdir(parents=True)
            items[name] = PreparedCase(directory, directory / "f", directory / "v", [], timeout_seconds=60)
        a_started = threading.Event()
        processes = {}
        real_popen = subprocess.Popen

        def prepare(_directory, definition, _case):
            return items[definition.name]

        def command(item, _attempt):
            if item is items["conc-b"]:
                self.assertTrue(a_started.wait(10))
                raise KeyboardInterrupt()
            return [sys.executable, "-c", "import time; time.sleep(4)"]

        def capture_popen(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            # Popen is patched module-wide; keep only run A's sleep child.
            if "time.sleep(4)" in " ".join(map(str, args[0])):
                processes["a"] = process
                a_started.set()
            return process

        cases = (CaseSpec("case_000", "only", "point-mass", "", {}, {}),)
        errors = []

        def run_a():
            try:
                run_study(self.definition("conc-a"), output_root=self.root, parallel=1)
            except BaseException as error:  # recorded and checked below
                errors.append(error)

        with mock.patch("tools.sim_gui.run_study.expand", return_value=cases), \
             mock.patch("tools.sim_gui.run_study.prepare_case", side_effect=prepare), \
             mock.patch("tools.sim_gui.run_study.validate_case", return_value=None), \
             mock.patch("tools.sim_gui.run_study.child_command", side_effect=command), \
             mock.patch("tools.sim_gui.run_study.subprocess.Popen", side_effect=capture_popen):
            thread = threading.Thread(target=run_a)
            thread.start()
            try:
                with self.assertRaises(KeyboardInterrupt):
                    run_study(self.definition("conc-b"), output_root=self.root, parallel=1)
                # Run A's child must survive run B's interrupt.
                self.assertIsNone(processes["a"].poll())
            finally:
                thread.join(30)
                if processes.get("a") is not None and processes["a"].poll() is None:
                    processes["a"].kill()
                    processes["a"].wait()
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        a_status = json.loads(next(self.root.glob("conc-a_*/study.json")).read_text())["cases"][0]["status"]
        self.assertNotEqual(a_status, "cancelled")

    def test_keyboard_interrupt_stops_child_and_cancels_cases(self):
        definition = self.definition("interrupt")
        fake_items = [
            PreparedCase(self.root / "case_000", self.root / "f0", self.root / "v0", [], timeout_seconds=60),
            PreparedCase(self.root / "case_001", self.root / "f1", self.root / "v1", [], timeout_seconds=60),
        ]
        for item in fake_items:
            item.directory.mkdir(parents=True)
        child = {}
        def prepare(_directory, _definition, case):
            return fake_items[int(case.id[-1])]
        cases = (CaseSpec("case_000", "running", "point-mass", "", {}, {}), CaseSpec("case_001", "pending", "point-mass", "", {}, {}))
        commands = iter([
            [sys.executable, "-c", "import time; time.sleep(60)"],
            KeyboardInterrupt(),
        ])
        def command(_item, _attempt):
            value = next(commands)
            if isinstance(value, BaseException):
                raise value
            return value
        real_popen = subprocess.Popen
        def capture_popen(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            child["process"] = process
            return process
        with mock.patch("tools.sim_gui.run_study.expand", return_value=cases), \
             mock.patch("tools.sim_gui.run_study.prepare_case", side_effect=prepare), \
             mock.patch("tools.sim_gui.run_study.validate_case", return_value=None), \
             mock.patch("tools.sim_gui.run_study.child_command", side_effect=command), \
             mock.patch("tools.sim_gui.run_study.subprocess.Popen", side_effect=capture_popen), \
             self.assertRaises(KeyboardInterrupt):
            run_study(definition, output_root=self.root, parallel=2)
        self.assertIsNotNone(child["process"].poll())
        outputs = list(self.root.glob("interrupt_*/study.json"))
        self.assertTrue(outputs)
        statuses = {row["id"]: row["status"] for row in json.loads(outputs[0].read_text())["cases"]}
        self.assertEqual(set(statuses.values()), {"cancelled"})


@unittest.skipUnless(SCIPY_AVAILABLE, "scipy is required for the forward-driver integration test")
class RunCaseIntegrationTests(unittest.TestCase):
    study_dir = REPOSITORY_ROOT / "runs" / "_test"

    def setUp(self):
        path = self.study_dir
        self.assertTrue(str(path).endswith("runs/_test"))
        self.addCleanup(shutil.rmtree, path, ignore_errors=True)

    @unittest.skip("physical yaw runner smoke requires completed map; covered separately when available")
    def test_headless_study_runs_cases_and_cli(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        baseline = VehicleConfiguration.open(source)
        choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix()
                   for key, path in baseline.choices.items()}
        definition = StudyDefinition(
            "_test", source.as_posix(),
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", choices,
            RunSettings(duration=0.3, profile_seconds=120.0), "one-at-a-time",
            (__import__('tools.sim_gui.study', fromlist=['Dimension']).Dimension(
                "tire", (choices["tire"], "models/tires/hoosier-43075-r20-simple.xml")),),
        )
        definition_path = self.study_dir / "definition.study.json"
        self.study_dir.mkdir(parents=True)
        definition_path.write_text(json.dumps(definition.to_json()))
        updates = []
        output = run_study(definition, output_root=self.study_dir, parallel=2,
                           on_update=lambda case_id, fields: updates.append((case_id, fields)))
        payload = json.loads((output / "study.json").read_text())
        self.assertEqual([case["status"] for case in payload["cases"]], ["completed", "completed"])
        for case in payload["cases"]:
            self.assertTrue((output / case["run_record"]).is_file())
            statuses = [fields.get("status") for case_id, fields in updates if case_id == case["id"]]
            self.assertIn("running", statuses)
            self.assertIn("completed", statuses)
        self.assertEqual(run_study_main(["--definition", str(definition_path),
                                         "--output-root", str(self.study_dir), "--parallel", "1"]), 0)

    def test_invalid_case_is_recorded_and_not_started(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        baseline = VehicleConfiguration.open(source)
        choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix() for key, path in baseline.choices.items()}
        definition = StudyDefinition("invalid", source.as_posix(),
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", choices,
            RunSettings(duration=0.3, profile_seconds=120.0), "one-at-a-time", ())
        with unittest.mock.patch("tools.sim_gui.run_study.prepare_case", side_effect=ValueError("bad variant")):
            output = run_study(definition, output_root=self.study_dir, parallel=1)
        case = json.loads((output / "study.json").read_text())["cases"][0]
        self.assertEqual(case["status"], "invalid")
        self.assertEqual(case["error"], "ValueError: bad variant")
        self.assertNotIn("attempt", case)

    @unittest.skip("physical yaw replay smoke requires a strict completed map and compatible saved plan")
    def test_planar_child_writes_verified_record_with_hoosier(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        baseline = VehicleConfiguration.open(source)
        choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix()
                   for key, path in baseline.choices.items()}
        choices["tire"] = "models/tires/hoosier-43075-r20-simple.xml"
        definition = StudyDefinition(
            "_test", source.as_posix(),
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", choices,
            RunSettings(duration=0.3, profile_seconds=120.0), "one-at-a-time", (),
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
