"""Explicit fixtures for regressions whose contract is the historical corner law.

The active Murphy model now uses additive physical struts. Tests of the former
independent-corner spring/ARB/damper law opt into this fixture so they continue
to test the historical contract without disguising the active model.
"""
from dataclasses import replace
from pathlib import Path
import re
import tempfile

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.suspension.components import (
    compile_axle_arb, compile_corner_springs, compile_damper,
    compile_suspension_kinematics,
)
from simplified_vehicle_models.suspension.layouts.independent_corner import build_independent_corner
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.xml.loader import load_model

ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
VEHICLE_XML = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
SUSPENSION_XML = ROOT / "models/suspensions/iem24_murphy_real_suspension_legacy.xml"


def legacy_network(kinematics=None, inertance_front=0.0, inertance_rear=0.0):
    """Build the historical independent-corner network from original components."""
    models = ROOT / "models"
    kin = kinematics or compile_suspension_kinematics(load_model(
        models / "suspension-kinematics/iem24_murphy_real_suspension_kinematics.xml"))
    springs = compile_corner_springs(load_model(models / "springs/iem24_murphy_real_suspension_springs.xml"))
    arb = compile_axle_arb(load_model(models / "anti-roll-bars/iem24_murphy_real_suspension_arb.xml"))
    front = compile_damper(load_model(models / "dampers/multimatic_dssv_vc01_front.xml"))
    rear = compile_damper(load_model(models / "dampers/multimatic_dssv_vc01_rear.xml"))
    return build_independent_corner(kinematics=kin, springs=springs, arb=arb,
        damper_front=front, damper_rear=rear, inertance_front=inertance_front,
        inertance_rear=inertance_rear).network


def legacy_suspension(kinematics=None, inertance_front=0.0, inertance_rear=0.0):
    """Return the assembled legacy suspension wrapper."""
    network = legacy_network(kinematics, inertance_front, inertance_rear)
    return CompiledSuspension(network.kinematics, network, "independent-corner")


def legacy_formulation():
    """Create a temporary formulation tree with the active XML ref swapped to legacy.

    The historic on-disk suspension XML is preserved in run provenance, so this
    fixture writes only under the platform temp dir and is used by consumers that
    need XML compiler behavior (e.g. the original sweep study tests).
    """
    text = VEHICLE_XML.read_text()
    # Make every relative reference/data path stable after moving this XML to temp.
    text = re.sub(r'(ref|path)="\.\./', rf'\1="{(ROOT / "models").as_posix()}/', text)
    text = re.sub(r'ref="[^"]*iem24_murphy_real_suspension\.xml"',
                  f'ref="{SUSPENSION_XML}"', text)
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    (root / "vehicle.xml").write_text(text)
    formulation = FORMULATION.read_text().replace(
        "../vehicles/iem24_murphy_real_planar.xml", "vehicle.xml")
    (root / "formulation.xml").write_text(formulation)
    # Keep the TemporaryDirectory alive for the caller's cached formulation path.
    legacy_formulation._temporary = temporary
    return root / "formulation.xml"


def legacy_vehicle():
    """Compile active vehicle parameters with explicit legacy network for legacy-law tests."""
    vehicle = compile_formulation(FORMULATION).model
    suspension = replace(vehicle.suspension,
        network=legacy_network(vehicle.suspension.kinematics))
    return replace(vehicle, suspension=suspension)
