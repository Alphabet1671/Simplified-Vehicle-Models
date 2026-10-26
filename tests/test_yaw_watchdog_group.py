"""The suite must not spawn a new session inside an existing hard watchdog."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


class SuiteWatchdogGroupTests(unittest.TestCase):
    def test_guarded_entrypoint_runs_main_without_nested_watchdog(self):
        root=Path(__file__).resolve().parents[1]
        # Intercept main through a patched runpy argument parser: a nested
        # watchdog call would execute a subprocess instead of this sentinel.
        code='''
import os,runpy
from unittest.mock import patch
import tools.run.run_with_deadline as deadline
os.environ['YAW_PROCESS_WATCHDOG_ACTIVE']='1'
with patch.object(deadline,'run_with_deadline',side_effect=AssertionError('nested watchdog')):
    with patch('argparse.ArgumentParser.parse_args',side_effect=RuntimeError('reached parser')):
        try:runpy.run_path('tools/run/run_yaw_lap_suite.py',run_name='__main__')
        except RuntimeError as error:
            assert str(error)=='reached parser'
'''
        result=subprocess.run([sys.executable,'-c',code],cwd=root,env=dict(os.environ),capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
