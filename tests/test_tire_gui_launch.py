"""Direct venv-compatible launch of explicitly paired transient tire XMLs."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(all(importlib.util.find_spec(name) for name in ('PyQt6', 'scipy', 'matplotlib')),
                     'optional tire-GUI dependencies unavailable')
class TireLaunchTests(unittest.TestCase):
    def test_transient_pair_launches_without_pythonpath_from_another_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            env = dict(os.environ)
            env.pop('PYTHONPATH', None)
            env.update(QT_QPA_PLATFORM='offscreen', TIRE_GUI_QUIT_AFTER_MS='100',
                       MPLCONFIGDIR=folder)
            command = [sys.executable, str(ROOT / 'tools/tire_gui/app.py'),
                       str(ROOT / 'models/tires/iem24_murphy_real_unitire_transient.xml'),
                       '--steady', str(ROOT / 'models/tires/iem24_murphy_real_unitire.xml')]
            process = subprocess.Popen(command, cwd=folder, env=env, text=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       start_new_session=True)
            try:
                stdout, stderr = process.communicate(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
                self.fail('transient pair GUI exceeded 30-second watchdog')
            self.assertEqual(process.returncode, 0, stdout + stderr)


if __name__ == '__main__':
    unittest.main()
