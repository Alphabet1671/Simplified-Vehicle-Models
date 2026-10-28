"""Regression tests for direct and installed GUI launch interfaces."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / ".venv" / "bin" / "python"


class GuiLauncherTests(unittest.TestCase):
    def run_gui(self, script: Path, cwd: Path, *, installed=False) -> subprocess.CompletedProcess[str]:
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["TIRE_GUI_QUIT_AFTER_MS"] = "500"
        env["SIM_GUI_QUIT_AFTER_MS"] = "500"
        process = subprocess.Popen(
            [str(script)] if installed else [str(PYTHON), str(script)], cwd=cwd, env=env,
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate()
            self.fail(f"GUI exceeded 20-second watchdog; stdout={stdout!r}, stderr={stderr!r}")
        return subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)

    @unittest.skipUnless(PYTHON.exists(), "repository .venv is unavailable")
    def test_direct_tire_script_resolves_sibling_tools_from_other_cwd(self):
        result = self.run_gui(ROOT / "tools/tire_gui/app.py", Path("/tmp"))
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(PYTHON.exists(), "repository .venv is unavailable")
    def test_direct_maneuver_script_has_help_and_launches_from_other_cwd(self):
        result = self.run_gui(ROOT / "tools/maneuver_gui/app.py", Path("/tmp"))
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless((PYTHON.parent / 'tire-gui').is_file() and (PYTHON.parent / 'maneuver-gui').is_file(),
                         'refresh editable installation to test installed commands')
    def test_installed_commands_launch_outside_checkout(self):
        for command in ('tire-gui', 'maneuver-gui'):
            with self.subTest(command=command):
                result = self.run_gui(PYTHON.parent / command, Path(tempfile.gettempdir()), installed=True)
                self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(PYTHON.exists(), 'repository .venv is unavailable')
    def test_transient_tire_cli_opens_single_self_contained_xml(self):
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        env.update(QT_QPA_PLATFORM='offscreen', TIRE_GUI_QUIT_AFTER_MS='100')
        command = [str(PYTHON), str(ROOT / 'tools/tire_gui/app.py'),
                   str(ROOT / 'models/tires/iem24_murphy_real_unitire_transient.xml')]
        process = subprocess.Popen(command, cwd=tempfile.gettempdir(), env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True)
        try:
            stdout, stderr = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate()
            self.fail(f'transient pair launch timed out: {stdout!r} {stderr!r}')
        self.assertEqual(process.returncode, 0, stderr)

    def test_launcher_adapter_exposes_console_entry_functions_without_qt_import(self):
        sys.path.insert(0, str(ROOT / "src"))
        try:
            from simplified_vehicle_models import gui_launchers
            self.assertTrue(callable(gui_launchers.tire_main))
            self.assertTrue(callable(gui_launchers.maneuver_main))
        finally:
            sys.path.remove(str(ROOT / "src"))

    def test_console_scripts_are_declared(self):
        text = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
        self.assertIn("tire-gui =", text)
        self.assertIn("maneuver-gui =", text)


if __name__ == "__main__":
    unittest.main()
