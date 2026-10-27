import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
from simplified_vehicle_models.tires.unitire_transient import compile_unitire_transient
from simplified_vehicle_models.xml.loader import load_model
from tools.tire_gui.model import load_snapshot, with_parameter
from tools.tire_gui.parameters import CANONICAL_STEADY_NAMES, TRANSIENT_NAMES
from tools.tire_gui.save import load_session, save_session

SEED = Path('models/tires/hoosier-43075-r20-simple.xml')


class SaveTests(unittest.TestCase):
    def setUp(self):
        self.seed = load_snapshot(SEED)

    def test_save_refuses_collision(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'candidate', include_transient=True)
            with self.assertRaises(FileExistsError):
                save_session(self.seed, Path(folder), 'candidate', include_transient=True)

    def test_path_escape_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                save_session(self.seed, Path(folder), '../escape', include_transient=False)

    def test_compiled_outputs_preserve_canonical_precision(self):
        snapshot = with_parameter(self.seed, 'mu_0_y', 0.9123456789012345)
        with tempfile.TemporaryDirectory() as folder:
            paths = save_session(snapshot, Path(folder), 'precise', include_transient=True)
            self.assertEqual(len(paths), 3)
            steady = compile_unitire_simple(load_model(paths[0]))
            transient = compile_unitire_transient(load_model(paths[1]))
            self.assertEqual(steady.mu_0_y, snapshot.values['mu_0_y'])
            self.assertEqual(transient.lateral_length_scale, snapshot.values['lateral_length_scale'])
            self.assertEqual(set(load_model(paths[0]).parameters), set(CANONICAL_STEADY_NAMES))
            self.assertEqual(set(load_model(paths[1]).parameters), set(TRANSIENT_NAMES))

    def test_session_restores_private_metadata_and_no_measurements(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'private', include_transient=False,
                         metadata={'selections': {'run': 8}, 'locks': {'mu_0_y': True},
                                   'bounds': {'mu_0_y': [0.1, 1.2]}, 'events': [{'start': 1.0, 'end': 2.0}],
                                   'source_files': ['/private/ttc/run8.mat']})
            restored = load_session(Path(folder) / 'private')
            self.assertEqual(dict(restored.session_metadata)['selections'], {'run': 8})
            manifest = json.loads((Path(folder) / 'private' / 'session.json').read_text())
            self.assertNotIn('measurements', manifest)
            self.assertIn('source_files', manifest)
            self.assertEqual(restored.parameter_status['longitudinal_length_scale'], 'provisional')

    def test_overwrite_known_bundle_keeps_backup_without_deleting(self):
        with tempfile.TemporaryDirectory() as folder:
            first = save_session(self.seed, Path(folder), 'replaceable', include_transient=False,
                                 metadata={'selections': {'old': True}})
            old_manifest = (Path(folder) / 'replaceable' / 'session.json').read_text()
            edited = with_parameter(self.seed, 'mu_0_y', self.seed.values['mu_0_y'] * 0.99)
            new_paths = save_session(edited, Path(folder), 'replaceable', include_transient=True,
                                     overwrite=True)
            self.assertEqual(len(new_paths), 3)
            self.assertNotEqual((Path(folder) / 'replaceable' / 'session.json').read_text(), old_manifest)
            backups = list(Path(folder).glob('.replaceable.backup-*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual((backups[0] / 'session.json').read_text(), old_manifest)
            self.assertTrue(all(p.is_file() for p in first))
            self.assertEqual(load_session(Path(folder) / 'replaceable').values['mu_0_y'], edited.values['mu_0_y'])

    def test_edits_preserve_session_metadata_through_save_and_load(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'original', include_transient=False,
                         metadata={'selections': {'run': 8}, 'locks': {'mu_0_y': True},
                                   'events': [{'start': 2, 'stop': 9}]})
            loaded = load_session(Path(folder) / 'original')
            edited = with_parameter(loaded, 'mu_0_y', loaded.values['mu_0_y'] * 0.99)
            save_session(edited, Path(folder), 'edited', include_transient=False)
            restored = load_session(Path(folder) / 'edited')
            self.assertEqual(restored.session_metadata['selections'], {'run': 8})
            self.assertEqual(restored.session_metadata['locks'], {'mu_0_y': True})
            self.assertEqual(restored.session_metadata['events'], [{'start': 2, 'stop': 9}])

    def test_symlink_output_parent_escape_refused(self):
        with tempfile.TemporaryDirectory() as folder, tempfile.TemporaryDirectory() as outside:
            link = Path(folder) / 'link'
            link.symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                save_session(self.seed, link, 'candidate', include_transient=False)

    def test_models_tree_is_protected(self):
        with self.assertRaises(ValueError):
            save_session(self.seed, Path('models'), 'do-not-overwrite', include_transient=False)
        self.assertTrue(SEED.is_file())

    def test_interrupted_pair_write_leaves_no_partial_bundle(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch('tools.tire_gui.save._validate_xml', side_effect=[None, RuntimeError('transient compilation interrupted')]):
                with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                    save_session(self.seed, Path(folder), 'interrupted', include_transient=True)
            self.assertEqual(list(Path(folder).iterdir()), [])

    def test_unknown_version_and_stale_fingerprint_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'bundle', include_transient=True)
            manifest_path = Path(folder) / 'bundle' / 'session.json'
            manifest = json.loads(manifest_path.read_text())
            manifest['version'] = 987
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_session(Path(folder) / 'bundle')
            manifest['version'] = 1
            manifest['steady_fingerprint'] = '0' * 64
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_session(Path(folder) / 'bundle')

    def test_malformed_session_values_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'bundle', include_transient=False)
            manifest_path = Path(folder) / 'bundle' / 'session.json'
            manifest = json.loads(manifest_path.read_text())
            manifest['values'][0]['name'] = 'not_a_parameter'
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_session(Path(folder) / 'bundle')


if __name__ == '__main__':
    unittest.main()
