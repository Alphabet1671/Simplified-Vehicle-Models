import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from unittest import mock

from tools.maintenance import archive_retired_gui_bridge as archive_tool


class RetiredGuiBridgeArchiveTests(unittest.TestCase):
    def test_archive_copies_hashes_and_zips_exact_members(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "tools/sim_gui/run_case.py"
            source.parent.mkdir(parents=True)
            source.write_text("retired bridge\n", encoding="utf-8")
            output = root / "local-backups"
            with mock.patch.object(archive_tool, "REPOSITORY_ROOT", root), \
                 mock.patch.object(archive_tool, "RETIRED_FILES", (Path("tools/sim_gui/run_case.py"),)), \
                 mock.patch("subprocess.check_output", side_effect=["deadbeef\n", "feature/test\n"]):
                directory, bundle = archive_tool.archive(output, now=__import__("datetime").datetime(2026, 10, 27, tzinfo=__import__("datetime").UTC))
            manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
            copied = directory / "tracked-files/tools/sim_gui/run_case.py"
            self.assertEqual(manifest["files"][0]["sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertEqual(manifest["files"][0]["bytes"], source.stat().st_size)
            self.assertEqual(copied.read_bytes(), source.read_bytes())
            with zipfile.ZipFile(bundle) as contents:
                self.assertEqual(set(contents.namelist()), {"manifest.json", "tracked-files/tools/sim_gui/run_case.py"})


if __name__ == "__main__":
    unittest.main()
