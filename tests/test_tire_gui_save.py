import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
from simplified_vehicle_models.tires.unitire_transient import compile_unitire_transient
from simplified_vehicle_models.xml.loader import load_model
from tools.tire_gui.model import load_snapshot, with_parameter
from tools.tire_gui.parameters import CANONICAL_STEADY_NAMES, TRANSIENT_NAMES, flatten_steady, UNITS
from tools.tire_gui.save import load_session, save_session, _xml, _STEADY_XML_NAMES

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
            self.assertEqual(len(paths), 2)
            combined = load_model(paths[0])
            steady = compile_unitire_simple(combined)
            transient = compile_unitire_transient(combined)
            self.assertEqual(steady.mu_0_y, snapshot.values['mu_0_y'])
            self.assertEqual(transient.lateral_length_scale, snapshot.values['lateral_length_scale'])
            self.assertEqual(flatten_steady(transient.steady), {n: snapshot.values[n] for n in CANONICAL_STEADY_NAMES})
            for name in TRANSIENT_NAMES:
                self.assertEqual(getattr(transient, name), snapshot.values[name])
            self.assertTrue(set(CANONICAL_STEADY_NAMES).issubset(combined.parameters))
            self.assertTrue(set(TRANSIENT_NAMES).issubset(combined.parameters))

    def _write_v1_pair(self, bundle, stem, *, reference=False, duplicate_ids=False):
        from tools.tire_gui.save import _manifest
        from simplified_vehicle_models.xml.model import ModelReference
        bundle.mkdir()
        steady_path = bundle / f'{stem}-unitire-simple.xml'
        transient_path = bundle / f'{stem}-unitire-transient-simple.xml'
        _xml(steady_path, f'{stem}-steady-' + '1' * 32, 'unitire-simple', self.seed.values, _STEADY_XML_NAMES)
        transient_id = (f'{stem}-steady-' + '1' * 32) if duplicate_ids else (f'{stem}-transient-' + '2' * 32)
        _xml(transient_path, transient_id, 'unitire-transient-simple', self.seed.values, TRANSIENT_NAMES)
        if reference:
            import xml.etree.ElementTree as ET
            tree = ET.parse(transient_path)
            steady_path = bundle / f'{stem}-unitire-simple.xml'
            ET.SubElement(tree.getroot(), 'components')
            ET.SubElement(tree.getroot().find('components'), 'tire', {'role': 'tire', 'ref': steady_path.name})
            tree.write(transient_path, encoding='utf-8', xml_declaration=True)
        manifest = _manifest(self.seed, None, True)
        manifest.update(version=1, tool='tire-gui')
        (bundle / 'session.json').write_text(json.dumps(manifest))
        return bundle

    def test_v1_pair_loads_and_can_be_rewritten_as_v2(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = self._write_v1_pair(Path(folder) / 'legacy-v1', 'legacy-v1')
            legacy = load_session(bundle)
            new_paths = save_session(legacy, Path(folder), 'rewritten', include_transient=True)
            self.assertEqual(len(new_paths), 2)
            self.assertEqual(load_session(Path(folder) / 'rewritten').values['mu_0_y'], legacy.values['mu_0_y'])
            self.assertEqual(json.loads(new_paths[-1].read_text())['version'], 2)

    def test_v1_duplicate_ids_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = self._write_v1_pair(Path(folder) / 'legacy-duplicate', 'legacy-duplicate', duplicate_ids=True)
            with self.assertRaises(ValueError):
                load_session(bundle)

    def test_v1_bundle_can_be_overwritten_and_backup_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = self._write_v1_pair(Path(folder) / 'legacy-overwrite', 'legacy-overwrite')
            old_manifest = (bundle / 'session.json').read_text()
            changed = with_parameter(self.seed, 'mu_0_y', self.seed.values['mu_0_y'] + .01)
            result = save_session(changed, Path(folder), 'legacy-overwrite', include_transient=False, overwrite=True)
            self.assertEqual(len(result), 2)
            backup = next(Path(folder).glob('.legacy-overwrite.backup-*'))
            self.assertEqual((backup / 'session.json').read_text(), old_manifest)
            restored = load_session(bundle)
            self.assertEqual(dict(restored.values), dict(changed.values))

    def test_v1_transient_references_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = self._write_v1_pair(Path(folder) / 'legacy-reference', 'legacy-reference', reference=True)
            with self.assertRaisesRegex(ValueError, 'references'):
                load_session(bundle)

    def test_v2_full_transient_embedded_steady_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = save_session(self.seed, Path(folder), 'tampered-steady', include_transient=True)
            xml = paths[0]
            text = xml.read_text()
            self.assertIn('name="mu_0_y"', text)
            marker = text.index('name="mu_0_y"')
            value_start = text.index('value="', marker) + len('value="')
            value_end = text.index('"', value_start)
            original = text[value_start:value_end]
            xml.write_text(text[:value_start] + repr(float(original) + 1e-6) + text[value_end:])
            with self.assertRaisesRegex(ValueError, 'match canonical'):
                load_session(Path(folder) / 'tampered-steady')

    def test_v2_steady_bundle_has_one_xml_and_restores_manifest_transient_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = save_session(self.seed, Path(folder), 'steady-only', include_transient=False,
                                 metadata={'locks': {'mu_0_y': True}})
            self.assertEqual(len(paths), 2)
            self.assertEqual({p.name for p in (Path(folder) / 'steady-only').iterdir()},
                             {'session.json', 'steady-only-unitire-simple.xml'})
            self.assertEqual(load_session(Path(folder) / 'steady-only').session_metadata['locks']['mu_0_y'], True)

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
            self.assertEqual(len(new_paths), 2)
            self.assertNotEqual((Path(folder) / 'replaceable' / 'session.json').read_text(), old_manifest)
            backups = list(Path(folder).glob('.replaceable.backup-*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual((backups[0] / 'session.json').read_text(), old_manifest)
            backup_xml = list(backups[0].glob('*.xml'))
            self.assertEqual(len(backup_xml), 1)
            self.assertTrue(backup_xml[0].is_file())
            self.assertEqual(load_session(Path(folder) / 'replaceable').values['mu_0_y'], edited.values['mu_0_y'])

    def test_legacy_holdout_rows_survive_session_load_and_rewrite(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'legacy-holdout', include_transient=False,
                         metadata={'selections': {'holdout_ids': [[72, 0]]}})
            bundle = Path(folder) / 'legacy-holdout'
            manifest_path = bundle / 'session.json'
            manifest = json.loads(manifest_path.read_text())
            # Emulate the v1 JSON row shape, then verify loader retains it so the
            # Qt layer can expand it to every matching group-qualified segment.
            manifest['selections']['holdout_ids'] = [[72, 0]]
            manifest_path.write_text(json.dumps(manifest))
            restored = load_session(bundle)
            self.assertEqual(restored.session_metadata['selections']['holdout_ids'], ((72, 0),))
            save_session(restored, Path(folder), 'migrated-holdout', include_transient=False)
            rewritten = load_session(Path(folder) / 'migrated-holdout')
            self.assertEqual(rewritten.session_metadata['selections']['holdout_ids'], ((72, 0),))

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
            xml = next(bundle.glob('*-unitire-transient-simple.xml'))
            text = xml.read_text()
            xml.write_text(text.replace('id="strict-transient-', 'id="bad id-', 1))
            with self.assertRaises(ValueError):
                load_session(bundle)

    def test_load_rejects_malformed_transient_model_id(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'dupid', include_transient=True)
            bundle = Path(folder) / 'dupid'
            transient_path = next(bundle.glob('*-unitire-transient-simple.xml'))
            text = transient_path.read_text()
            transient_id = text.split('id="', 1)[1].split('"', 1)[0]
            transient_path.write_text(text.replace(transient_id, 'not-a-valid-id', 1))
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

    def test_interrupted_single_xml_write_leaves_no_partial_bundle(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch('tools.tire_gui.save._validate_xml', side_effect=RuntimeError('single XML compilation interrupted')):
                with self.assertRaisesRegex(RuntimeError, 'single XML'):
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
            manifest['version'] = 2
            manifest['steady_fingerprint'] = '0' * 64
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'fingerprint'):
                load_session(Path(folder) / 'bundle')

    def test_manifest_rejects_boolean_version_and_non_boolean_transient_flag(self):
        with tempfile.TemporaryDirectory() as folder:
            save_session(self.seed, Path(folder), 'strict-types', include_transient=False)
            manifest_path = Path(folder) / 'strict-types' / 'session.json'
            original = json.loads(manifest_path.read_text())
            for field, value in (('version', True), ('transient_included', 0), ('transient_included', [])):
                data = dict(original)
                data[field] = value
                manifest_path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load_session(manifest_path.parent)

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
