import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class VehicleGuiDocsTests(unittest.TestCase):
    def test_the_guide_names_the_start_command(self):
        text = (ROOT / "docs" / "dev" / "vehicle-gui.md").read_text(encoding="utf-8")
        self.assertIn("python3 tools/vehicle_gui/app.py", text)

    def test_tools_layout_lists_the_package_and_links_the_guide(self):
        text = (ROOT / "docs" / "dev" / "tools-layout.md").read_text(encoding="utf-8")
        self.assertRegex(text, r"\| `tools/vehicle_gui/` \|[^\n]*\[vehicle-gui\.md\]\(vehicle-gui\.md\)")

    def test_the_default_output_folder_is_ignored(self):
        lines = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()
        self.assertIn("/vehicles_out/", lines)


if __name__ == "__main__":
    unittest.main()
