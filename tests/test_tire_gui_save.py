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
                                   'bounds': {'mu_0_y': [0.1, 1.2]}, 'events': [{'start': 1.0, 'stop': 2.0}],
                                   'source_files': ['/private/ttc/run8.mat']})
            restored = load_session(Path(folder) / 'private')
            self.assertEqual(dict(restored.session_metadata)['selections'], {'run': 8})
            manifest = json.loads((Path(folder) / 'private' / 'session.json').read_text())
            self.assertNotIn('measurements', manifest)
            self.assertIn('source_files', manifest)
            self.assertEqual(restored.parameter_status['longitudinal_length_scale'], 'provisional')

    def test_overwrite_revalidates_destination_after_staging(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'racy', include_transient=False)
            original_validate = __import__('tools.tire_gui.save', fromlist=['_validate_xml'])._validate_xml
            mutated = False
            def mutate_during_staging(path, transient):
                nonlocal mutated
                result = original_validate(path, transient)
                if not mutated:
                    (Path(folder) / 'racy' / 'unexpected.txt').write_text('unknown')
                    mutated = True
                return result
            edited = with_parameter(self.seed, 'mu_0_y', self.seed.values['mu_0_y'] * 0.99)
            with patch('tools.tire_gui.save._validate_xml', side_effect=mutate_during_staging):
                with self.assertRaises(FileExistsError):
                    save_session(edited, Path(folder), 'racy', include_transient=False, overwrite=True)
            self.assertTrue((Path(folder) / 'racy' / 'unexpected.txt').is_file())
            self.assertTrue((Path(folder) / 'racy' / 'racy-unitire-simple.xml').is_file())
            self.assertFalse(list(Path(folder).glob('.racy.backup-*')))

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
            self.assertEqual(restored.session_metadata['events'][0]['start'], 2)
            self.assertEqual(restored.session_metadata['events'][0]['stop'], 9)

    def test_nested_raw_payload_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                save_session(self.seed, Path(folder), 'bad-meta', include_transient=False,
                             metadata={'events': [{'id': 'e1', 'samples': [1.0, 2.0]}]})
            with self.assertRaises(ValueError):
                save_session(self.seed, Path(folder), 'bad-meta2', include_transient=False,
                             metadata={'session_metadata': {'raw_payload': {'channels': [1, 2]}}})

    def test_session_metadata_is_deeply_immutable(self):
        source = {'selections': {'run': 8}, 'events': [{'start': 2, 'stop': 9}]}
        snapshot = load_snapshot(SEED)
        from tools.tire_gui.model import _make_snapshot
        frozen = _make_snapshot(snapshot.steady, snapshot.transient, snapshot.revision,
                                snapshot.provenance, snapshot.parameter_status, snapshot.load_domain, source)
        source['selections']['run'] = 99
        self.assertEqual(frozen.session_metadata['selections']['run'], 8)
        with self.assertRaises(TypeError):
            frozen.session_metadata['selections']['run'] = 9
        with self.assertRaises(TypeError):
            frozen.session_metadata['events'][0]['start'] = 10

    def test_load_rejects_extra_entry_and_malformed_xml_id(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'strict', include_transient=True)
            bundle = Path(folder) / 'strict'
            extra = bundle / 'extra.txt'
            extra.write_text('untrusted')
            with self.assertRaises(ValueError):
                load_session(bundle)
            extra.unlink()
            xml = next(bundle.glob('*-unitire-simple.xml'))
            text = xml.read_text()
            xml.write_text(text.replace('id="strict-steady-', 'id="bad id-', 1))
            with self.assertRaises(ValueError):
                load_session(bundle)

    def test_load_rejects_duplicate_model_ids(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'dupid', include_transient=True)
            bundle = Path(folder) / 'dupid'
            steady = next(bundle.glob('*-unitire-simple.xml')).read_text()
            transient_path = next(bundle.glob('*-unitire-transient-simple.xml'))
            text = transient_path.read_text()
            steady_id = steady.split('id="', 1)[1].split('"', 1)[0]
            transient_path.write_text(text.replace(text.split('id="', 1)[1].split('"', 1)[0], steady_id, 1))
            with self.assertRaises(ValueError):
                load_session(bundle)

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
