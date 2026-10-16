"""Export the fitted simplified UniTire parameters to one XML model.

The source is the reviewed MATLAB fit and the reviewed solver:

- ``Hoosier_R20_Combined_Simple.mat`` holds the ``tire`` structure.
- ``unitire_simple_solve.m`` defines how every field is used.

Run this tool only when the fit changes. Record the reported source hash in
``docs/model-data/provenance.md``.

The XML schema stores one scalar in each ``parameter`` element. This tool
therefore flattens each MATLAB ``polyval`` coefficient vector into one
parameter for each degree. A name ends with the degree of ``Fzn``:

- ``k_x_2`` is the ``Fzn`` squared coefficient of ``Kx``.
- ``d_x_0_1`` is the ``Fzn`` coefficient of ``Dx0``.

The parameter units are derived from the source structure, not from a unit
table in the MATLAB file. The derivation follows:

- ``phi_x = Kx*Sx/Fxm`` is dimensionless, so ``Kx`` and ``Ky`` use newtons.
- ``Xc = Fx/Kcx`` is a length, so ``Kcx`` and ``Kcy`` use newtons per metre.
- ``Dx0``, ``De``, ``R1``, ``R2`` and ``R3`` use metres.
- ``v_m_x``, ``v_m_y``, ``vroll_min``, ``omega_cr`` and ``omega_eps`` use
  seconds-based SI units.
- The source overturning-moment term ``Mx2`` is dimensionally inhomogeneous.
  ``k_11`` to ``k_23`` are zero in this fit, so the term vanishes. The XML
  declares the linear-term unit and records this limit in a comment.
- ``stiff_eps`` is one numerical floor that the source applies to stiffness
  terms with different units. It is dimensionless in the source.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.io import loadmat


SOURCE_MAT = Path(
    "/Users/alphabet1671/Dev/FSAE-VD-Personal-Scripts"
    "/Yaw Dynamics/Tire Model/Hoosier_R20_Combined_Simple.mat"
)

MODEL_ID = "hoosier-43075-r20-simple"
TIRE_SUBTYPE = "unitire-simple"

# Scalar fields that precede the stiffness polynomial vectors.
_HEAD_SCALAR_FIELDS: tuple[tuple[str, str], ...] = (
    ("Fz_rated", "N"),
    ("Kcx", "N/m"),
    ("Kcy", "N/m"),
)

# Scalar fields that follow the stiffness polynomial vectors.
_SCALAR_FIELDS: tuple[tuple[str, str], ...] = (
    ("E1", "1"),
    ("mu0x", "1"),
    ("musx", "1"),
    ("hx", "1"),
    ("vmx", "m/s"),
    ("mu0y", "1"),
    ("musy", "1"),
    ("hy", "1"),
    ("vmy", "m/s"),
    ("K11", "N*m"),
    ("K12", "N*m"),
    ("K13", "N*m"),
    ("K21", "N*m"),
    ("K22", "N*m"),
    ("K23", "N*m"),
    ("MxR1", "N*m"),
    ("MxR2", "N*m"),
    ("MxR3", "N*m"),
    ("R1", "m"),
    ("R2", "m"),
    ("R3", "m"),
    ("Fy_shift0", "N"),
    ("Fy_shiftFz", "N"),
    ("KRl0", "m/N^2"),
    ("KRl1", "m/N^2"),
    ("KRl2", "m/N^2"),
    ("frr0", "1"),
    ("frr1", "1"),
    ("frr2", "1"),
    ("hrr", "1"),
    ("omega_cr", "rad/s"),
    ("omega_eps", "rad/s"),
    ("vroll_min", "m/s"),
    ("force_eps", "N"),
    ("stiff_eps", "1"),
    ("slip_eps", "1"),
)

_VECTOR_FIELDS: tuple[tuple[str, str], ...] = (
    ("Kx", "N"),
    ("Ky", "N"),
    ("Dx0", "m"),
    ("De", "m"),
    ("D1", "1"),
    ("D2", "1"),
)

# Python parameter name for each scalar MATLAB field.
_SCALAR_NAMES = {
    "Fz_rated": "f_z_rated",
    "Kcx": "k_cx",
    "Kcy": "k_cy",
    "E1": "e_1",
    "mu0x": "mu_0_x",
    "musx": "mu_s_x",
    "hx": "h_x",
    "vmx": "v_m_x",
    "mu0y": "mu_0_y",
    "musy": "mu_s_y",
    "hy": "h_y",
    "vmy": "v_m_y",
    "K11": "k_11",
    "K12": "k_12",
    "K13": "k_13",
    "K21": "k_21",
    "K22": "k_22",
    "K23": "k_23",
    "MxR1": "m_x_r_1",
    "MxR2": "m_x_r_2",
    "MxR3": "m_x_r_3",
    "R1": "r_1",
    "R2": "r_2",
    "R3": "r_3",
    "Fy_shift0": "f_y_shift_0",
    "Fy_shiftFz": "f_y_shift_fz",
    "KRl0": "k_rl_0",
    "KRl1": "k_rl_1",
    "KRl2": "k_rl_2",
    "frr0": "f_rr_0",
    "frr1": "f_rr_1",
    "frr2": "f_rr_2",
    "hrr": "h_rr",
    "omega_cr": "omega_cr",
    "omega_eps": "omega_eps",
    "vroll_min": "vroll_min",
    "force_eps": "force_eps",
    "stiff_eps": "stiff_eps",
    "slip_eps": "slip_eps",
}

# Python parameter name for each vector MATLAB field.
_VECTOR_NAMES = {
    "Kx": "k_x",
    "Ky": "k_y",
    "Dx0": "d_x_0",
    "De": "d_e",
    "D1": "d_1",
    "D2": "d_2",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _number(value: float) -> str:
    """Format one float so that Python reads back the same value."""

    return repr(float(value))


def _flatten(
    tire,
) -> tuple[list[tuple[str, float, str]], dict[str, np.ndarray]]:
    """Return the flat scalar parameters and the polynomial vectors."""

    def scalar_rows(
        fields: tuple[tuple[str, str], ...],
    ) -> list[tuple[str, float, str]]:
        rows: list[tuple[str, float, str]] = []
        for field, unit in fields:
            value = float(np.asarray(getattr(tire, field)).reshape(()))
            rows.append((_SCALAR_NAMES[field], value, unit))
        return rows

    vectors: dict[str, np.ndarray] = {}
    vector_rows: list[tuple[str, float, str]] = []
    for field, unit in _VECTOR_FIELDS:
        coefficients = np.atleast_1d(
            np.asarray(getattr(tire, field), dtype=np.float64)
        ).ravel()
        vectors[_VECTOR_NAMES[field]] = coefficients
        degree = coefficients.size - 1
        for index, coefficient in enumerate(coefficients):
            name = f"{_VECTOR_NAMES[field]}_{degree - index}"
            vector_rows.append((name, float(coefficient), unit))
    return (
        scalar_rows(_HEAD_SCALAR_FIELDS)
        + vector_rows
        + scalar_rows(_SCALAR_FIELDS),
        vectors,
    )


def build_model(source: Path) -> ElementTree.Element:
    """Build the XML model element from the fitted MATLAB structure."""

    tire = loadmat(source, squeeze_me=True, struct_as_record=False)["tire"]
    rows, _ = _flatten(tire)
    sha256 = _sha256(source)

    model = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": MODEL_ID,
            "type": "tire-model",
            "subtype": TIRE_SUBTYPE,
        },
    )
    model.append(
        ElementTree.Comment(
            " Simplified steady-state UniTire, zero inclination angle."
            " Ported from unitire_simple_solve.m in the FSAE-VD-Personal-Scripts"
            " Yaw Dynamics/Tire Model directory."
        )
    )
    model.append(
        ElementTree.Comment(
            f" Source: {source.name}"
            f" sha256={sha256}"
            " (Hoosier 43075 R20, 12 psi, 8 inch rim, TTC B2356 runs 8 and 72)."
        )
    )
    model.append(
        ElementTree.Comment(
            " Each polynomial coefficient is one parameter."
            " The trailing number is the degree of the normalized load Fzn."
            " Example: k_x_2 multiplies Fzn squared."
        )
    )
    model.append(
        ElementTree.Comment(
            " Units are derived from the source structure, not from a unit table."
            " k_11 to k_23 and m_x_r_1 to m_x_r_3 are zero in this fit."
            " The source Mx2 term is dimensionally inhomogeneous, so the"
            " declared N*m unit is the linear-term unit only."
            " stiff_eps is one numerical floor that the source applies to"
            " stiffness terms with different units."
        )
    )

    parameters = ElementTree.SubElement(model, "parameters")
    for name, value, unit in rows:
        ElementTree.SubElement(
            parameters,
            "parameter",
            {"name": name, "value": _number(value), "unit": unit},
        )
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE_MAT)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()

    model = build_model(options.source.expanduser().resolve())
    ElementTree.indent(model, space="    ")
    document = ElementTree.ElementTree(model)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    document.write(options.output, encoding="UTF-8", xml_declaration=True)
    text = options.output.read_text(encoding="utf-8")
    # Match the declaration style of the other model files.
    text = text.replace(
        "<?xml version='1.0' encoding='UTF-8'?>",
        '<?xml version="1.0" encoding="UTF-8"?>',
        1,
    )
    options.output.write_text(text + "\n", encoding="utf-8")

    count = len(model.findall("./parameters/parameter"))
    print(f"wrote {count} parameters to {options.output}")
    print(f"source sha256 {_sha256(options.source.expanduser().resolve())}")


if __name__ == "__main__":
    main()
