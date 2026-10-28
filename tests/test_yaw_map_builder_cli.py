import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.run import build_vehicle_yaw_map as builder


class BuilderCliTests(unittest.TestCase):
    def test_workers_parser_is_positive(self):
        args = builder.parser().parse_args(['--output', 'map.npz', '--workers', '2'])
        self.assertEqual(args.workers, 2)
        for value in ('0', '-2', '2.1', 'nope'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                builder.parser().parse_args(['--output', 'map.npz', '--workers', value])

    def test_startup_and_failure_report_are_flushed_and_machine_readable(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'map.npz'
            stream = io.StringIO()
            with patch('simplified_vehicle_models.compiler.compile.compile_formulation',
                       side_effect=RuntimeError('synthetic compile failure')), \
                 contextlib.redirect_stdout(stream):
                status = builder.main(['--output', str(output), '--workers', '2'])
            self.assertEqual(status, 1)
            events = [json.loads(line) for line in stream.getvalue().splitlines()]
            self.assertEqual(events[0]['event'], 'startup')
            self.assertEqual(events[0]['workers'], 2)
            self.assertTrue(any(event['event'] == 'error' for event in events))
            report = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(report['error'], 'synthetic compile failure')
            self.assertFalse(report['complete'])

    def test_gui_default_workers_are_four_on_every_host(self):
        from tools.sim_gui.yaw_map import default_yaw_map_workers, map_build_command, MapBuildRequest
        for cpu_count in (1, 2, 8, 32, None):
            with self.subTest(cpu_count=cpu_count), patch('tools.sim_gui.yaw_map.os.cpu_count', return_value=cpu_count):
                self.assertEqual(default_yaw_map_workers(), 4)
                request = MapBuildRequest(Path('form.xml'), None, Path('map.npz'), 'identity', 'missing')
                command = map_build_command(request)
                self.assertEqual(command[command.index('--workers') + 1], '4')
        command = map_build_command(request, workers=1)
        self.assertEqual(command[command.index('--workers') + 1], '1')


if __name__ == '__main__':
    unittest.main()
