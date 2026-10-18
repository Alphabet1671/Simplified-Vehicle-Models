"""Generate deterministic component-file sweeps for simulation studies."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Literal, Sequence
import xml.etree.ElementTree as ElementTree

import numpy as np

from simplified_vehicle_models.xml.loader import load_model
from tools.model_data.generate_damper_lut import atomic_write_bytes, npz_bytes
from tools.model_data.xml_references import rewrite_references
from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.catalogue import read_entry

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
Operation = Literal["value", "offset", "scale"]


def _repository_commit() -> str | None:
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
                                capture_output=True, text=True, check=False)
    except OSError:
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    os.replace(temporary, path)


def _parse_target(target: str) -> tuple[str, str | None, str]:
    parts = target.split(":")
    if len(parts) == 2 and parts[0] == "parameter" and parts[1]:
        return "parameter", None, parts[1]
    if len(parts) == 3 and parts[0] == "data" and parts[1] and parts[2]:
        return "data", parts[1], parts[2]
    raise ValueError("target must be parameter:<name> or data:<role>:<key>")


def _format_label(key: str, operation: Operation, x: float, unit: str) -> str:
    if operation == "value":
        value = f"{x:.4g}"
        prefix = f"{key} = {value}"
    elif operation == "offset":
        prefix = f"{key} {x:+.3g}"
    else:
        prefix = f"{key} x{x:.3g}"
    return prefix if operation == "scale" else prefix + f" {unit}"


def _remove_output(output: Path) -> None:
    # Deliberately refuse to remove arbitrary existing directories.
    marker = output / "sweep.json"
    if not marker.is_file():
        raise ValueError(f"refusing to remove existing folder without sweep.json: {output}")
    import shutil
    shutil.rmtree(output)


def generate_sweep(base: Path, target: str, operation: Operation, numbers: Sequence[float],
                   output: Path, *, name: str, unit: str | None = None,
                   force: bool = False, command: list[str] | None = None) -> Path:
    base = Path(base).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    numbers = [float(number) for number in numbers]
    if not NAME_RE.fullmatch(name):
        raise ValueError("name must match [A-Za-z0-9_-]{1,40}")
    if operation not in {"value", "offset", "scale"}:
        raise ValueError(f"unsupported operation: {operation}")
    if not 1 <= len(numbers) <= 100:
        raise ValueError("sweep needs 1 to 100 numbers")
    if any(not math.isfinite(number) for number in numbers):
        raise ValueError("sweep numbers must be finite")
    if output.exists():
        if not force:
            raise ValueError(f"output folder already exists: {output}")
        _remove_output(output)
    entry = read_entry(base)
    if entry.kind != "component" or ElementTree.parse(base).getroot().find("components") is not None:
        raise ValueError(f"base must be a leaf component: {base}")
    kind, role, key = _parse_target(target)
    tree = ElementTree.parse(base)
    root = tree.getroot()
    parameter = None
    data_element = None
    source_npz = None
    baseline = None
    if kind == "parameter":
        parameter = next((p for p in root.findall("./parameters/parameter") if p.get("name") == key), None)
        if parameter is None:
            raise ValueError(f"parameter not found: {key}")
        xml_unit = parameter.get("unit", "1")
        if unit is not None and unit != xml_unit:
            raise ValueError(f"unit {unit!r} does not match XML unit {xml_unit!r}")
        unit = xml_unit
        try:
            baseline = float(parameter.get("value", ""))
        except ValueError as error:
            raise ValueError(f"parameter is not numeric: {key}") from error
    else:
        data_element = next((d for d in root.findall("./data") if d.get("role") == role), None)
        if data_element is None:
            raise ValueError(f"data role not found: {role}")
        if data_element.get("format", "npz").lower() != "npz" or Path(data_element.get("path", "")).suffix.lower() != ".npz":
            raise ValueError("only npz data arrays can be swept")
        if unit is None:
            raise ValueError("data targets require --unit")
        source_npz = (base.parent / data_element.get("path", "")).resolve()
        with np.load(source_npz, allow_pickle=False) as archive:
            if key not in archive.files:
                raise ValueError(f"data array not found: {key}")
            baseline = np.array(archive[key], copy=True)
            if baseline.dtype.kind not in "f":
                raise ValueError("only floating-point data arrays can be swept")
    if operation == "value" and (kind == "data" and np.asarray(baseline).size != 1):
        raise ValueError("--values requires a scalar target")
    output.mkdir(parents=True)
    base_bytes = base.read_bytes()
    base_sha = hashlib.sha256(base_bytes).hexdigest()
    members = []
    try:
        for index, number in enumerate(numbers):
            stem = f"{base.stem}__{index:02d}"
            member_xml = output / f"{stem}.xml"
            member_data = None
            if kind == "parameter":
                value = number if operation == "value" else (baseline + number if operation == "offset" else baseline * number)
                value_array = value
            else:
                old = np.asarray(baseline)
                value_array = number if operation == "value" else (old + number if operation == "offset" else old * number)
                value_array = np.asarray(value_array, dtype=old.dtype)
                with np.load(source_npz, allow_pickle=False) as archive:
                    arrays = {name_: np.array(archive[name_], copy=True) for name_ in archive.files}
                arrays[key] = value_array
                data_name = f"{stem}.npz"
                member_data = output / data_name
                data_bytes = npz_bytes(arrays)
                atomic_write_bytes(member_data, data_bytes)
                data_element.set("path", data_name)
                data_element.set("sha256", hashlib.sha256(data_bytes).hexdigest())
            member_root = ElementTree.fromstring(ElementTree.tostring(root, encoding="unicode"))
            if kind == "parameter":
                member_parameter = next(p for p in member_root.findall("./parameters/parameter") if p.get("name") == key)
                member_parameter.set("value", repr(float(value_array)))
            else:
                member_data_root = next(d for d in member_root.findall("./data") if d.get("role") == role)
                member_data_root.set("path", f"{stem}.npz")
                member_data_root.set("sha256", hashlib.sha256(member_data.read_bytes()).hexdigest())
            member_root.set("id", f"{root.get('id', '')}.sweep-{name}-{index:02d}")
            member_tree = ElementTree.ElementTree(member_root)
            rewrite_references(member_tree, base.parent, output)
            if member_data is not None:
                member_data_root = next(d for d in member_root.findall("./data") if d.get("role") == role)
                member_data_root.set("path", member_data.name)
                member_data_root.set("sha256", hashlib.sha256(member_data.read_bytes()).hexdigest())
            comment = ElementTree.Comment(
                f" Generated by tools/model_data/generate_component_sweep.py: {target} {operation} {number} {unit}, from {base}. Do not edit. "
            )
            member_tree.write(member_xml, encoding="utf-8", xml_declaration=True)
            xml_bytes = member_xml.read_bytes()
            marker = (f"<!-- Generated by tools/model_data/generate_component_sweep.py: {target} {operation} {number} {unit}, from {base}. Do not edit. -->\n").encode()
            declaration = b'?>'
            split = xml_bytes.find(declaration)
            if split >= 0:
                xml_bytes = xml_bytes[:split + 2] + b"\n" + marker + xml_bytes[split + 2:]
            else:
                xml_bytes = marker + xml_bytes
            member_xml.write_bytes(xml_bytes)
            load_model(member_xml)
            if member_data is not None:
                with np.load(member_data, allow_pickle=False):
                    pass
            output_value = value_array.tolist() if kind == "data" else float(value_array)
            members.append({"index": index, "file": member_xml.name,
                            "data": [member_data.name] if member_data else [],
                            "label": _format_label(key, operation, number, unit), "x": number,
                            "value": output_value})
    except Exception:
        import shutil
        shutil.rmtree(output, ignore_errors=True)
        raise
    payload = {"schema": "component-sweep-v1", "name": name,
               "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
               "repository_commit": _repository_commit(), "command": command or [],
               "base": os.path.relpath(base, REPOSITORY_ROOT).replace(os.sep, "/"),
               "base_sha256": base_sha, "model_type": root.get("type"), "subtype": root.get("subtype"),
               "target": target, "operation": operation, "unit": unit,
               "baseline_value": baseline.tolist() if kind == "data" else float(baseline), "members": members}
    _atomic_json(output / "sweep.json", payload)
    return output / "sweep.json"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--set", dest="target", required=True)
    parser.add_argument("--unit")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--values", nargs="+", type=float)
    group.add_argument("--offsets", nargs="+", type=float)
    group.add_argument("--scales", nargs="+", type=float)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    operation = "value" if args.values is not None else "offset" if args.offsets is not None else "scale"
    numbers = args.values if args.values is not None else args.offsets if args.offsets is not None else args.scales
    output = args.output_dir or REPOSITORY_ROOT / "models" / "sweeps" / args.name
    try:
        generate_sweep(args.base, args.target, operation, numbers, output, name=args.name,
                       unit=args.unit, force=args.force, command=list(argv) if argv is not None else sys.argv[1:])
    except (ValueError, OSError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
