"""Safe atomic artifacts for headless maneuver executions."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import uuid

import numpy as np

ARTIFACT_SCHEMA = "maneuver-artifacts-v1"


def _jsonable(value):
    if hasattr(value, "to_dict"):
        return _jsonable(value.to_dict())
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if hasattr(value, "__dataclass_fields__"):
        return {k: _jsonable(getattr(value, k)) for k in value.__dataclass_fields__}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _atomic_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    except BaseException:
        try:
            os.unlink(temp)
        except FileNotFoundError:
            pass
        raise


def _safe_name(name: str) -> str:
    """Convert a label into a generated leaf filename; never accept paths."""
    leaf = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name)).strip("._") or "artifact"
    return leaf[:100]


def create_attempt_directory(output_root: Path, *, overwrite: bool = False) -> Path:
    root = Path(output_root).expanduser().resolve()
    models_root = (Path(__file__).resolve().parents[2] / "models").resolve()
    if root == models_root or models_root in root.parents:
        raise ValueError("output root may not be inside models/")
    root.mkdir(parents=True, exist_ok=True)
    for _ in range(32):
        candidate = root / str(uuid.uuid4())
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise FileExistsError("could not allocate a unique attempt directory")


def write_artifacts(directory: Path, definition, result, summary: dict) -> None:
    """Atomically persist a validated definition, evidence NPZ and manifest.

    Existing destinations are rejected, preventing accidental mixing of runs.
    The only permitted filenames are fixed generated leaf names in ``directory``.
    """
    directory = Path(directory).resolve()
    if not directory.is_dir():
        raise FileNotFoundError(directory)
    names = ("maneuver.json", "summary.json", "events.json", "commands.csv", "run_record.npz", "manifest.json")
    if any((directory / name).exists() for name in names):
        raise FileExistsError(f"artifacts already exist in {directory}")
    data = _jsonable(definition)
    if not isinstance(data, dict) or data.get("schema") != "maneuver-v1":
        raise ValueError("definition must be a validated maneuver-v1 definition")
    summary_doc = _jsonable(summary)
    outcome = getattr(result, "execution_outcome", "unknown")
    summary_doc.setdefault("execution_outcome", outcome)
    summary_doc.setdefault("termination_reason", getattr(result, "termination_reason", "unknown"))

    # Keep evidence arrays in a pickle-free NPZ; use BytesIO so replacement is atomic.
    import io
    arrays = {"time_s": np.asarray(result.time_s), "state": np.asarray(result.state),
              "state_rate": np.asarray(result.state_rate)}
    buf = io.BytesIO()
    np.savez_compressed(buf, **arrays)
    definition_bytes = json.dumps(data, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    summary_bytes = json.dumps(summary_doc, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    events = _jsonable(getattr(result, "events", ()))
    events_bytes = json.dumps(events, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    commands = ["index,time_s,swa_rad,accelerator,brake"]
    for index, item in enumerate(getattr(result, "applied_commands", ())):
        if isinstance(item, dict):
            v = item.get("applied", item.get("command", item))
        else:
            v = item
        if hasattr(v, "swa_rad"):
            row = (v.swa_rad, v.accelerator, v.brake)
        elif isinstance(v, (tuple, list, np.ndarray)) and len(v) >= 3:
            row = tuple(v[:3])
        else:
            continue
        time_value = float(result.time_s[min(index, len(result.time_s)-1)])
        commands.append(f"{index},{time_value:.17g},{float(row[0]):.17g},{float(row[1]):.17g},{float(row[2]):.17g}")
    command_bytes = ("\n".join(commands) + "\n").encode()
    npz_bytes = buf.getvalue()
    canonical = lambda value: json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    hashes = {"definition_sha256": hashlib.sha256(canonical(data)).hexdigest(),
              "calibration_sha256": hashlib.sha256(canonical(data.get("calibration"))).hexdigest(),
              "model_sha256": hashlib.sha256(canonical(data.get("vehicle"))).hexdigest(),
              "result_sha256": hashlib.sha256(npz_bytes).hexdigest(),
              "summary_sha256": hashlib.sha256(summary_bytes).hexdigest(),
              "events_sha256": hashlib.sha256(events_bytes).hexdigest(),
              "commands_sha256": hashlib.sha256(command_bytes).hexdigest()}
    manifest = {"schema": ARTIFACT_SCHEMA, "version": 1, "definition": data,
                "calibration": data.get("calibration"), "model": data.get("vehicle"),
                "hashes": hashes, "execution_outcome": summary_doc["execution_outcome"]}
    manifest_bytes = json.dumps(manifest, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    payloads = {"maneuver.json": definition_bytes, "summary.json": summary_bytes,
                "events.json": events_bytes, "commands.csv": command_bytes,
                "run_record.npz": npz_bytes, "manifest.json": manifest_bytes}
    # Publish through a same-filesystem sibling directory. A UUID destination is
    # reserved with mkdir (never reused); readers only see a complete directory.
    parent = directory.parent
    parent.mkdir(parents=True, exist_ok=True)
    if directory.is_symlink():
        raise ValueError("artifact destination may not be a symlink")
    if directory.exists() and any(directory.iterdir()):
        raise FileExistsError(f"artifacts already exist in {directory}")
    stage = parent / f".{directory.name}.{uuid.uuid4().hex}.staging"
    stage.mkdir()
    try:
        for name, payload in payloads.items():
            if Path(name).name != name:
                raise ValueError("unsafe generated artifact path")
            (stage / name).write_bytes(payload)
        # Manifest is the commit marker; within a published directory it is last.
        if directory.exists():
            directory.rmdir()
        os.rename(stage, directory)
    except BaseException:
        import shutil
        shutil.rmtree(stage, ignore_errors=True)
        raise


def write_artifact(output_base: Path, definition, result, *, run_id: str | None = None) -> Path:
    """Write one immutable run under a required output base and return its directory.

    Run identifiers are generated UUIDs by default. A caller-supplied identifier is
    accepted only as one plain path component; it can never select a location outside
    the resolved output base.
    """
    if output_base is None:
        raise TypeError("output_base is required")
    raw_root = Path(output_base).expanduser()
    root = raw_root.resolve()
    models_root = (Path(__file__).resolve().parents[2] / "models").resolve()
    if root == models_root or models_root in root.parents:
        raise ValueError("output base may not be inside models/")
    root.mkdir(parents=True, exist_ok=True)
    if run_id is None:
        run_id = str(uuid.uuid4())
    elif (not isinstance(run_id, str) or not run_id or run_id in (".", "..")
          or Path(run_id).name != run_id or "/" in run_id or "\\\\" in run_id):
        raise ValueError("run_id must be a safe single path component")
    run_dir = root / run_id
    if run_dir.parent != root:
        raise ValueError("run directory must remain under output base")
    run_dir.mkdir(exist_ok=False)

    data = _jsonable(definition)
    if not isinstance(data, dict) or data.get("schema") != "maneuver-v1":
        run_dir.rmdir()
        raise ValueError("definition must have maneuver-v1 schema")
    import io
    archive = io.BytesIO()
    np.savez_compressed(archive, time_s=np.asarray(result.time_s),
                        state=np.asarray(result.state), state_rate=np.asarray(result.state_rate))
    npz_bytes = archive.getvalue()
    definition_bytes = json.dumps(data, sort_keys=True, allow_nan=False).encode()
    canonical = lambda value: json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    summary_bytes = json.dumps({"execution_outcome": str(getattr(result, "execution_outcome", "unknown"))}, sort_keys=True).encode() + b"\n"
    events_bytes = b"[]\n"
    commands_bytes = b"index,time_s,swa_rad,accelerator,brake\n"
    hashes = {
        "definition_sha256": hashlib.sha256(canonical(data)).hexdigest(),
        "calibration_sha256": hashlib.sha256(canonical(data.get("calibration"))).hexdigest(),
        "model_sha256": hashlib.sha256(canonical(data.get("vehicle"))).hexdigest(),
        "result_sha256": hashlib.sha256(npz_bytes).hexdigest(),
        "summary_sha256": hashlib.sha256(summary_bytes).hexdigest(),
        "events_sha256": hashlib.sha256(events_bytes).hexdigest(),
        "commands_sha256": hashlib.sha256(commands_bytes).hexdigest(),
    }
    outcome = str(getattr(result, "execution_outcome", "unknown"))
    manifest = {"schema": ARTIFACT_SCHEMA, "version": 1, "definition": data,
                "execution_outcome": outcome, "hashes": hashes,
                "calibration": data.get("calibration"), "model": data.get("vehicle")}
    try:
        _atomic_bytes(run_dir / "maneuver.json", (json.dumps(data, sort_keys=True, indent=2, allow_nan=False) + "\n").encode())
        _atomic_bytes(run_dir / "summary.json", (json.dumps({"execution_outcome": outcome}, sort_keys=True) + "\n").encode())
        _atomic_bytes(run_dir / "events.json", b"[]\n")
        _atomic_bytes(run_dir / "commands.csv", b"index,time_s,swa_rad,accelerator,brake\n")
        _atomic_bytes(run_dir / "run_record.npz", npz_bytes)
        _atomic_bytes(run_dir / "manifest.json",
                      (json.dumps(manifest, sort_keys=True, indent=2, allow_nan=False) + "\n").encode())
    except BaseException:
        for name in ("maneuver.json", "summary.json", "events.json", "commands.csv", "run_record.npz", "manifest.json"):
            try:
                (run_dir / name).unlink()
            except FileNotFoundError:
                pass
        run_dir.rmdir()
        raise
    return run_dir


def load_artifact(directory: Path) -> tuple[dict, dict[str, np.ndarray]]:
    """Load an artifact manifest and pickle-free NPZ arrays, checking integrity."""
    manifest = load_manifest(directory)
    try:
        with np.load(Path(directory).resolve() / "run_record.npz", allow_pickle=False) as archive:
            arrays = {name: archive[name].copy() for name in archive.files}
    except (OSError, ValueError, KeyError) as exc:
        raise ValueError(f"invalid maneuver result NPZ: {exc}") from exc
    return manifest, arrays


def load_manifest(directory: Path) -> dict:
    """Load and integrity-check an artifact manifest and its evidence payload."""
    directory = Path(directory).resolve()
    try:
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        if manifest.get("schema") != ARTIFACT_SCHEMA or manifest.get("version") != 1:
            raise ValueError("artifact schema/version mismatch")
        if manifest.get("definition", {}).get("schema") != "maneuver-v1":
            raise ValueError("definition schema mismatch")
        hashes = manifest.get("hashes", {})
        definition = manifest["definition"]
        def canonical(value):
            return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        definition_file = json.loads((directory / "maneuver.json").read_text(encoding="utf-8"))
        if definition_file != definition:
            raise ValueError("manifest/definition mismatch")
        for key, value in (("definition_sha256", definition_file),
                           ("calibration_sha256", definition_file.get("calibration")),
                           ("model_sha256", definition_file.get("vehicle"))):
            if hashes.get(key) != hashlib.sha256(canonical(value)).hexdigest():
                raise ValueError(f"{key} mismatch")
        if manifest.get("calibration") != definition_file.get("calibration") or manifest.get("model") != definition_file.get("vehicle"):
            raise ValueError("manifest model/calibration mismatch")
        summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
        if summary.get("execution_outcome") != manifest.get("execution_outcome"):
            raise ValueError("manifest/summary outcome mismatch")
        for key, filename in (("summary_sha256", "summary.json"),
                              ("events_sha256", "events.json"),
                              ("commands_sha256", "commands.csv")):
            if hashes.get(key) != hashlib.sha256((directory / filename).read_bytes()).hexdigest():
                raise ValueError(f"{filename} corruption detected")
        for name in ("maneuver.json", "summary.json", "events.json", "commands.csv", "run_record.npz"):
            if not (directory / name).is_file():
                raise ValueError(f"missing artifact {name}")
        expected = manifest.get("hashes", {}).get("result_sha256")
        actual = hashlib.sha256((directory / "run_record.npz").read_bytes()).hexdigest()
        if expected != actual:
            raise ValueError("result artifact corruption detected")
        with np.load(directory / "run_record.npz", allow_pickle=False) as archive:
            if not {"time_s", "state", "state_rate"}.issubset(archive.files):
                raise ValueError("result arrays are incomplete")
        return manifest
    except (OSError, json.JSONDecodeError, KeyError) as exc:
        raise ValueError(f"invalid maneuver artifact set: {exc}") from exc
