"""Tests for the no-TV runtime compatibility manifest."""
import importlib.metadata
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.diagnostics import no_tv_runtime as runtime


class FakeModule:
    def __init__(self, name, version):
        self.__version__ = version
        self.__file__ = f"/venv/lib/{name}.py"


class FakeMetadata:
    class PackageNotFoundError(importlib.metadata.PackageNotFoundError):
        pass

    versions = {"numpy": "2.5.4", "scipy": "1.17.1", "casadi": "3.8.1"}

    @classmethod
    def version(cls, name):
        if name not in cls.versions:
            raise cls.PackageNotFoundError(name)
        return cls.versions[name]


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.modules = {name: FakeModule(name, version) for name, version in FakeMetadata.versions.items()}

    def importer(self, name):
        if name not in self.modules:
            raise ImportError(name)
        return self.modules[name]

    def test_collect_reports_executed_modules_and_platform(self):
        report = runtime.collect_runtime(importer=self.importer, metadata=FakeMetadata,
                                        py_version=(3, 13, 2), implementation="CPython",
                                        system_platform="test-platform", machine="test-machine")
        self.assertEqual(report["packages"]["casadi"]["executed_version"], "3.8.1")
        self.assertEqual(report["packages"]["numpy"]["module_file"], "/venv/lib/numpy.py")
        self.assertEqual(report["platform"]["machine"], "test-machine")

    def test_minimum_and_exact_runtime_pass(self):
        result = runtime.check_runtime(require_exact=True, importer=self.importer, metadata=FakeMetadata,
                                       py_version=(3, 13, 0))
        self.assertTrue(result["compatibility"]["compatible"], result["compatibility"]["problems"])

    def test_missing_disagreement_and_unsupported_version_fail(self):
        del self.modules["casadi"]
        result = runtime.check_runtime(importer=self.importer, metadata=FakeMetadata,
                                       py_version=(3, 13, 0))
        self.assertFalse(result["compatibility"]["compatible"])
        self.assertTrue(any("casadi could not be imported" in p for p in result["compatibility"]["problems"]))
        self.modules["casadi"] = FakeModule("casadi", "4.0")
        result = runtime.check_runtime(importer=self.importer, metadata=FakeMetadata, py_version=(3, 13, 0))
        self.assertTrue(any("outside the supported range" in p for p in result["compatibility"]["problems"]))

    def test_cli_writes_only_exclusive_json_output(self):
        valid = runtime.check_runtime
        report = {"compatibility": {"compatible": True}}
        with tempfile.TemporaryDirectory() as directory, patch.object(runtime, "check_runtime", return_value=report):
            target = Path(directory) / "manifest.json"
            self.assertEqual(runtime.main(["--output", str(target)]), 0)
            self.assertEqual(json.loads(target.read_text()), report)
            with self.assertRaises(SystemExit) as error:
                runtime.main(["--output", str(target)])
            self.assertEqual(error.exception.code, 2)
            link = Path(directory) / "link.json"
            link.symlink_to(target)
            with self.assertRaises(SystemExit):
                runtime.main(["--output", str(link)])
            dangling = Path(directory) / "dangling.json"
            dangling.symlink_to(Path(directory) / "not-created.json")
            with self.assertRaises(SystemExit):
                runtime.main(["--output", str(dangling)])
            self.assertFalse((Path(directory) / "not-created.json").exists())

    def test_exact_flag_rejects_other_python_minor(self):
        result = runtime.check_runtime(require_exact=True, importer=self.importer, metadata=FakeMetadata,
                                       py_version=(3, 12, 9))
        self.assertFalse(result["compatibility"]["compatible"])
        self.assertTrue(any("exact runtime requires Python 3.13" in p for p in result["compatibility"]["problems"]))

    def test_configuration_and_pins(self):
        root = Path(__file__).resolve().parents[1]
        import tomllib
        config = tomllib.loads((root / "pyproject.toml").read_text())
        self.assertIn("casadi>=3.6,<4", config["project"]["optional-dependencies"]["no-tv"])
        requirements = (root / "requirements/no-tv-runtime-py313.txt").read_text()
        for package, version in runtime.TESTED_VERSIONS.items():
            self.assertIn(f"{package}=={version}", requirements)


if __name__ == "__main__":
    unittest.main()
