"""Loader of the ``hydraulic-circuit-v1`` NPZ (sha256 pinned).

The XML loader accepts only scalar parameters and csv/npz data, so the circuit
graph lives in an NPZ written by ``tools/model_data/generate_hydraulic_circuit.py``.

Arrays (N nodes, L lines):

* ``node_names`` (N,) text; ``areas`` (N, 4) signed wheel-referred areas [m^2];
* ``precharge_pressure`` (N,) gauge [Pa]; ``precharge_volume`` (N,) [m^3];
  ``exponent`` (N,); ``minimum_volume`` (N,) [m^3]; ``design_pressure`` (N,) gauge [Pa];
* ``line_from``, ``line_to`` (L,) node indices; ``line_linear`` (L,) [Pa s/m^3];
  ``line_quadratic`` (L,) [Pa s^2/m^6];
* ``schema``, ``generator_version``, ``metadata`` (JSON text).
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from simplified_vehicle_models.hydraulics.circuit import HydraulicCircuit
from simplified_vehicle_models.hydraulics.elements import AccumulatorSet, OrificeSet

SCHEMA = "hydraulic-circuit-v1"
REQUIRED_KEYS = (
    "schema", "generator_version", "metadata", "node_names", "areas",
    "precharge_pressure", "precharge_volume", "exponent", "minimum_volume",
    "design_pressure", "line_from", "line_to", "line_linear", "line_quadratic",
)


def load_circuit_npz(path, expected_sha256: str) -> HydraulicCircuit:
    """Load a ``hydraulic-circuit-v1`` NPZ after checking its SHA-256."""

    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != str(expected_sha256).strip().lower():
        raise ValueError(f"hydraulic circuit sha256 mismatch for {path}: got {digest}")
    with np.load(path, allow_pickle=False) as data:
        missing = [key for key in REQUIRED_KEYS if key not in data.files]
        if missing:
            raise ValueError(f"hydraulic circuit {path} is missing keys {missing}")
        arrays = {key: data[key] for key in REQUIRED_KEYS}
    if str(arrays["schema"]) != SCHEMA:
        raise ValueError(f"unsupported hydraulic circuit schema {arrays['schema']!s}")
    try:
        json.loads(str(arrays["metadata"]))
    except json.JSONDecodeError as error:
        raise ValueError("hydraulic circuit metadata must be a JSON string") from error
    return HydraulicCircuit(
        node_names=tuple(str(name) for name in arrays["node_names"]),
        areas=arrays["areas"],
        accumulators=AccumulatorSet(
            precharge_pressure=arrays["precharge_pressure"],
            precharge_volume=arrays["precharge_volume"],
            exponent=arrays["exponent"],
            minimum_volume=arrays["minimum_volume"],
        ),
        design_pressure=arrays["design_pressure"],
        line_from=arrays["line_from"],
        line_to=arrays["line_to"],
        orifices=OrificeSet(linear=arrays["line_linear"], quadratic=arrays["line_quadratic"]),
    )
