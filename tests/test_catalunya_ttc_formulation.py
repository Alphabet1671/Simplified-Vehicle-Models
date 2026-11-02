import hashlib
import unittest
from pathlib import Path

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple
from simplified_vehicle_models.tires.unitire_transient import compile_unitire_transient

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "models/tires/ttc-r20-unitire-transient.xml"
STEADY = ROOT / "models/tires/ttc-r20-unitire-steady.xml"
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_ttc_r20_7dof.xml"
DEFAULT_FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
EXPECTED_SHA256 = "204fd0269c0b8bd82a6583c10f8c917f33255646e5a463367434cc3c9f4d3120"
TRANSIENT_NAMES = {"longitudinal_length_scale", "lateral_length_scale", "trail_pole_ratio", "trail_lead_ratio"}
TRANSIENT_ONLY_MODEL = ROOT / "models/tires/ttc-r20-unitire-transient-companion.xml"


class CatalunyaTtcFormulationTests(unittest.TestCase):
    def test_artifact_hash_and_exact_steady_parameter_transcription(self):
        self.assertEqual(hashlib.sha256(SOURCE.read_bytes()).hexdigest(), EXPECTED_SHA256)
        source = load_model(SOURCE)
        steady = load_model(STEADY)
        source_params = {name: (value, source.units[name]) for name, value in source.parameters.items()}
        steady_params = {name: (value, steady.units[name]) for name, value in steady.parameters.items()}
        transient_model = load_model(TRANSIENT_ONLY_MODEL)
        transient_params = {name: (value, transient_model.units[name]) for name, value in transient_model.parameters.items()}
        self.assertEqual(set(steady_params), set(source_params) - TRANSIENT_NAMES)
        self.assertEqual(set(transient_params), TRANSIENT_NAMES)
        self.assertEqual(set(steady_params).intersection(transient_params), set())
        self.assertEqual({**steady_params, **transient_params}, source_params)
        self.assertEqual(source_params["k_11"][0], 0.01969999999999983)

    def test_compiled_ttc_coefficients_and_planar_assembly(self):
        steady = compile_unitire_simple(load_model(STEADY))
        source = load_model(SOURCE)
        transient = compile_unitire_transient(load_model(TRANSIENT_ONLY_MODEL))
        self.assertEqual(transient.longitudinal_length_scale, 0.574)
        self.assertEqual(transient.lateral_length_scale, 0.5742000000000003)
        self.assertEqual(transient.trail_pole_ratio, 0.01670000000000016)
        self.assertEqual(transient.trail_lead_ratio, 0.6172)
        self.assertEqual(steady.k_x, (7660.409078173158, 20000.0, 8191.007990757677))
        self.assertEqual(steady.k_y, (654.9807055362467, 19999.998862085806, 10000.0))
        self.assertEqual(steady.k_11, 0.01969999999999983)
        compiled = compile_formulation(FORMULATION)
        vehicle = compiled.model
        self.assertEqual(vehicle.tire.k_11, steady.k_11)
        self.assertFalse(hasattr(vehicle, "transient_tire"))
        paths = {component.role: component.path for component in compiled.manifest.loaded_components}
        self.assertTrue(any(path.name == "ttc-r20-unitire-steady.xml" for path in paths.values()))
        unused = {component.role: component.path for component in compiled.manifest.unused_components}
        self.assertTrue(any(path.name == "ttc-r20-unitire-transient-companion.xml" for path in unused.values()))
        self.assertEqual(vehicle.tire, steady)
        self.assertNotIn("transient-combined-slip", {record.capability for record in compiled.manifest.capabilities})
        self.assertEqual(compiled.manifest.capability("steady-combined-slip").component_role, "tire")
        self.assertFalse(any(record.component_role == "tire-transient" for record in compiled.manifest.capabilities))
        default = compile_formulation(DEFAULT_FORMULATION)
        default_paths = {component.role: component.path for component in default.manifest.loaded_components}
        self.assertTrue(any(path.name == "iem24_murphy_real_unitire.xml" for path in default_paths.values()))
        self.assertFalse(any("ttc-r20" in str(path) for path in default_paths.values()))


if __name__ == "__main__":
    unittest.main()
