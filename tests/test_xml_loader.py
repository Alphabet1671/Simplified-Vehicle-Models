from __future__ import annotations

import dataclasses
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.xml.loader import load_model, load_point_mass_vehicle


FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"


class XmlLoaderTests(unittest.TestCase):
    def test_loads_legacy_point_mass_parameters(self) -> None:
        vehicle = load_point_mass_vehicle(FORMULATION_PATH)

        self.assertIsInstance(vehicle, PointMassVehicleParameters)
        self.assertEqual(vehicle.total_mass, 299.0)
        self.assertEqual(vehicle.wheelbase, 1.535)
        self.assertEqual(vehicle.track_width, 1.016)
        self.assertEqual(vehicle.cg_height, 0.3)
        self.assertEqual(vehicle.front_mass_fraction, 0.47)
        self.assertEqual(vehicle.lateral_load_transfer_distribution, 0.44)
        self.assertEqual(vehicle.tire.longitudinal_mu_scale, 0.25)
        self.assertEqual(vehicle.tire.lateral_mu_scale, 0.4745)
        self.assertEqual(vehicle.tire.load_sensitivity_slope, -0.000404656098)
        self.assertEqual(vehicle.tire.load_sensitivity_intercept, 2.9)
        self.assertEqual(vehicle.tire.effective_radius, 0.193675)
        self.assertEqual(vehicle.aero.air_density, 1.22)
        self.assertEqual(vehicle.aero.downforce_area, 4.2)
        self.assertEqual(vehicle.aero.drag_area, 1.8)
        self.assertEqual(vehicle.aero.aero_balance, 0.44)
        self.assertEqual(vehicle.drivetrain.motor_count, 4)
        self.assertEqual(vehicle.drivetrain.gear_ratio, 10.9)
        self.assertEqual(
            vehicle.drivetrain.peak_motor_torque,
            21.834945614425717,
        )
        self.assertEqual(vehicle.drivetrain.power_limit, 80000.0)
        self.assertEqual(vehicle.drivetrain.efficiency, 0.95)
        self.assertEqual(vehicle.drivetrain.maximum_motor_speed, 18600.0)
        self.assertEqual(vehicle.brake_bias, 0.65)

    def test_parameters_are_immutable(self) -> None:
        vehicle = load_point_mass_vehicle(FORMULATION_PATH)

        with self.assertRaises(dataclasses.FrozenInstanceError):
            vehicle.total_mass = 300.0

    def test_rejects_unknown_model_type(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unknown.xml"
            path.write_text(
                '<model schema-version="1.0" id="unknown" '
                'type="unknown-model" subtype="unknown"/>',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "unknown model type"):
                load_point_mass_vehicle(path)

    def test_rejects_missing_parameter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_valid_component_files(root)
            (root / "vehicle.xml").write_text(
                """<model schema-version="1.0" id="vehicle"
    type="vehicle-assembly" subtype="four-wheel">
    <parameters>
        <parameter name="wheelbase" value="1.535" unit="m"/>
        <parameter name="track_width" value="1.016" unit="m"/>
        <parameter name="cg_height" value="0.3" unit="m"/>
        <parameter name="front_mass_fraction" value="0.47" unit="1"/>
        <parameter name="lateral_load_transfer_distribution" value="0.44" unit="1"/>
        <parameter name="brake_bias" value="0.65" unit="1"/>
    </parameters>
    <components>
        <component role="tire" ref="tire.xml"/>
        <component role="aero" ref="aero.xml"/>
        <component role="drivetrain" ref="drivetrain.xml"/>
    </components>
</model>
""",
                encoding="utf-8",
            )
            self._write_formulation(root)

            with self.assertRaisesRegex(ValueError, "total_mass"):
                load_point_mass_vehicle(root / "formulation.xml")

    def test_rejects_component_reference_cycle_with_chain(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "first.xml").write_text(
                '<model schema-version="1.0" id="first" '
                'type="vehicle-formulation" subtype="point-mass">'
                '<source ref="second.xml"/></model>',
                encoding="utf-8",
            )
            (root / "second.xml").write_text(
                '<model schema-version="1.0" id="second" '
                'type="vehicle-assembly" subtype="four-wheel">'
                '<component role="tire" ref="first.xml"/></model>',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                r"first\.xml.*second\.xml.*first\.xml",
            ):
                load_point_mass_vehicle(root / "first.xml")

    def test_rejects_duplicate_parameter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.xml"
            path.write_text(
                """<model schema-version="1.0" id="duplicate"
    type="tire-model" subtype="load-sensitive-friction-ellipse">
    <parameters>
        <parameter name="longitudinal_mu_scale" value="0.25" unit="1"/>
        <parameter name="longitudinal_mu_scale" value="0.30" unit="1"/>
    </parameters>
</model>
""",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "duplicate parameter"):
                load_point_mass_vehicle(path)

    def test_rejects_non_numeric_parameter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.xml"
            path.write_text(
                """<model schema-version="1.0" id="invalid"
    type="tire-model" subtype="load-sensitive-friction-ellipse">
    <parameters>
        <parameter name="longitudinal_mu_scale" value="not-a-number" unit="1"/>
    </parameters>
</model>
""",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "valid float"):
                load_point_mass_vehicle(path)

    def test_rejects_nonphysical_parameter_range(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "models"
            shutil.copytree(REPOSITORY_ROOT / "models", target)
            vehicle_path = target / "vehicles/murphy_2024.xml"
            text = vehicle_path.read_text(encoding="utf-8")
            vehicle_path.write_text(
                text.replace(
                    'name="total_mass" value="299.0"',
                    'name="total_mass" value="0"',
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "total_mass must be positive"):
                load_point_mass_vehicle(
                    target / "formulations/murphy_point_mass.xml"
                )

    def test_parses_external_data_reference_relative_to_model(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data_path = root / "tables" / "operating-map.npz"
            data_path.parent.mkdir()
            data_path.write_bytes(b"table")
            model_path = root / "model.xml"
            model_path.write_text(
                """<model schema-version=\"1.0\" id=\"model\"
    type=\"tire-model\" subtype=\"load-sensitive-friction-ellipse\">
    <data role=\"operating-map\" path=\"tables/operating-map.npz\"
          format=\"npz\" sha256=\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"/>
</model>""",
                encoding="utf-8",
            )

            model = load_model(model_path)

            self.assertEqual(model.data[0].role, "operating-map")
            self.assertEqual(model.data[0].path, data_path.resolve())
            self.assertEqual(model.data[0].format, "npz")
            self.assertEqual(model.data[0].sha256, "a" * 64)

    def test_rejects_duplicate_data_roles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "one.npz").write_bytes(b"one")
            (root / "two.npz").write_bytes(b"two")
            model_path = root / "model.xml"
            model_path.write_text(
                """<model schema-version=\"1.0\" id=\"model\" type=\"tire-model\" subtype=\"load-sensitive-friction-ellipse\">
    <data role=\"map\" path=\"one.npz\" format=\"npz\"/>
    <data role=\"map\" path=\"two.npz\" format=\"npz\"/>
</model>""",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate data role"):
                load_model(model_path)

    def test_rejects_invalid_external_data_declarations(self) -> None:
        cases = (
            ("missing", "missing.npz", "npz", None, "regular file"),
            ("format", "file.bin", "bin", None, "unsupported data format"),
            ("sha", "file.npz", "npz", "not-a-digest", "sha256"),
            ("role", "file.npz", "npz", None, "role"),
        )
        for case, relative_path, data_format, digest, error_text in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                if case != "missing":
                    (root / relative_path).write_bytes(b"data")
                digest_attribute = "" if digest is None else f' sha256="{digest}"'
                model_path = root / "model.xml"
                role = "" if case == "role" else "map"
                model_path.write_text(
                    f'<model schema-version="1.0" id="model" type="tire-model" subtype="load-sensitive-friction-ellipse">'
                    f'<data role="{role}" path="{relative_path}" format="{data_format}"{digest_attribute}/>'
                    "</model>",
                    encoding="utf-8",
                )
                with self.assertRaisesRegex(ValueError, error_text):
                    load_model(model_path)

    def test_allows_uppercase_sha256(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "file.csv").write_text("x,y\\n1,2\\n", encoding="utf-8")
            model_path = root / "model.xml"
            model_path.write_text(
                '<model schema-version="1.0" id="model" type="tire-model" subtype="load-sensitive-friction-ellipse">'
                '<data role="map" path="file.csv" format="csv" sha256="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"/>'
                "</model>",
                encoding="utf-8",
            )
            self.assertEqual(load_model(model_path).data[0].sha256, "A" * 64)

    def _write_formulation(self, root: Path) -> None:
        (root / "formulation.xml").write_text(
            '<model schema-version="1.0" id="formulation" '
            'type="vehicle-formulation" subtype="point-mass">'
            '<source ref="vehicle.xml"/></model>',
            encoding="utf-8",
        )

    def _write_valid_component_files(self, root: Path) -> None:
        (root / "tire.xml").write_text(
            """<model schema-version="1.0" id="tire"
    type="tire-model" subtype="load-sensitive-friction-ellipse">
    <parameters>
        <parameter name="longitudinal_mu_scale" value="0.25" unit="1"/>
        <parameter name="lateral_mu_scale" value="0.4745" unit="1"/>
        <parameter name="load_sensitivity_slope" value="-0.000404656098" unit="1/N"/>
        <parameter name="load_sensitivity_intercept" value="2.9" unit="1"/>
        <parameter name="effective_radius" value="0.193675" unit="m"/>
    </parameters>
</model>
""",
            encoding="utf-8",
        )
        (root / "aero.xml").write_text(
            """<model schema-version="1.0" id="aero"
    type="aero-model" subtype="fixed-aero">
    <parameters>
        <parameter name="air_density" value="1.22" unit="kg/m^3"/>
        <parameter name="downforce_area" value="4.2" unit="m^2"/>
        <parameter name="drag_area" value="1.8" unit="m^2"/>
        <parameter name="aero_balance" value="0.44" unit="1"/>
    </parameters>
</model>
""",
            encoding="utf-8",
        )
        (root / "drivetrain.xml").write_text(
            """<model schema-version="1.0" id="drivetrain"
    type="drivetrain-model" subtype="four-wheel-electric">
    <parameters>
        <parameter name="motor_count" value="4" unit="1"/>
        <parameter name="gear_ratio" value="10.9" unit="1"/>
        <parameter name="peak_motor_torque"
            value="21.834945614425717" unit="N*m"/>
        <parameter name="power_limit" value="80000" unit="W"/>
        <parameter name="efficiency" value="0.95" unit="1"/>
        <parameter name="maximum_motor_speed" value="20000" unit="rpm"/>
    </parameters>
</model>
""",
            encoding="utf-8",
        )


if __name__ == "__main__":
    unittest.main()
