"""Command assembly tests for cold-then-warm forward acceptance chains."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/run/run_forward_acceptance_chain.sh'


class ForwardAcceptanceChainTests(unittest.TestCase):
    def test_default_runs_cold_then_matching_warm_with_shared_track(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            wrapper = root / 'python-wrapper'
            calls = root / 'calls.jsonl'
            wrapper.write_text('#!' + sys.executable + '\n' + '''
import json, os, pathlib, sys
args = sys.argv[1:]
if args and args[0] == '-':
    os.execv(sys.executable, [sys.executable] + args)
with open(os.environ['CALLS'], 'a') as stream:
    stream.write(json.dumps(args) + '\\n')
out = pathlib.Path(args[args.index('--output-root') + 1])
if out.name == 'cold':
    for name, complete in [('attempt_001', True), ('attempt_002', False)]:
        path = out / name
        path.mkdir(parents=True)
        (path / 'summary.json').write_text(json.dumps({
            'status': 'Completed' if complete else 'Failed', 'lap_completed': complete}))
''')
            wrapper.chmod(0o755)
            output = root / 'out'
            result = subprocess.run(['bash', str(SCRIPT), '14dof-simple', str(output),
                                     '.15', '1.5', '1.5', 'both', '--track', 'test-track.xml'],
                                    cwd=ROOT, env=dict(os.environ, PYTHON=str(wrapper), CALLS=str(calls)),
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            commands = [json.loads(line) for line in calls.read_text().splitlines()]
            self.assertEqual(len(commands), 2)
            for command in commands:
                self.assertEqual(command[command.index('--track') + 1], 'test-track.xml')
            self.assertNotIn('--warm-start', commands[0])
            self.assertEqual(commands[1][commands[1].index('--warm-start') + 1],
                             str(output / 'cold/attempt_001'))

    def test_removed_refinement_stage_is_rejected(self):
        result = subprocess.run(['bash', str(SCRIPT), '14dof-simple', '/unused',
                                 '.15', '1.5', '1.5', 'refined'], cwd=ROOT,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('refinement stages were removed', result.stderr)

    def test_stage_owned_arguments_cannot_be_overridden(self):
        result = subprocess.run(['bash', str(SCRIPT), '14dof-simple', '/unused',
                                 '.15', '1.5', '1.5', 'both', '--warm-start=other'],
                                cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('owned by the acceptance chain', result.stderr)
