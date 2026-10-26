"""Canonical, fail-closed run manifest sidecar primitives.

Hashes identify bytes; they are not signatures or qualification evidence.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile

_MANIFEST = "run_manifest_v1.json"
_TRAJECTORY = "trajectory_metadata.npz"
_REQUIRED = {
    "track", "formulation", "chart", "transitive_dependencies",
    "effective_request", "runtime", "code", "boundary_event_contract",
    "validation_report", "policy",
}


def _safe_member(root: Path, name: str) -> Path:
    if not isinstance(name, str) or not name or Path(name).is_absolute():
        raise ValueError("manifest member path must be a non-empty relative filename")
    candidate = root / name
    if any(part in {"..", ""} for part in Path(name).parts):
        raise ValueError("manifest member path traversal is forbidden")
    resolved_root = root.resolve(strict=True)
    resolved = candidate.resolve(strict=True)
    if not resolved.is_relative_to(resolved_root) or candidate.is_symlink():
        raise ValueError("manifest member escapes output root")
    if not resolved.is_file():
        raise ValueError("manifest member must be a regular file")
    return resolved


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_manifest_identities(root, *, track, formulation, chart, transitive_dependencies,
                              effective_request, runtime, code, boundary_event_contract,
                              validation_report, policy):
    """Create root-relative descriptors from actual files and content identities."""
    root = Path(root)
    values = {"track": track, "formulation": formulation, "chart": chart,
              "transitive_dependencies": transitive_dependencies,
              "effective_request": effective_request, "runtime": runtime, "code": code,
              "boundary_event_contract": boundary_event_contract,
              "validation_report": validation_report, "policy": policy}
    result = {}
    for key, value in values.items():
        if key == "transitive_dependencies":
            result[key] = []
            for dependency in value:
                rel = dependency["path"] if isinstance(dependency, dict) else str(dependency)
                expected = dependency.get("sha256") if isinstance(dependency, dict) else None
                member = _safe_member(root, rel)
                actual = _hash(member)
                if expected is not None and expected != actual:
                    raise ValueError(f"dependency hash mismatch: {rel}")
                result[key].append({"path": rel, "sha256": actual})
        elif key in {"track", "formulation", "chart", "validation_report"}:
            candidate = Path(value)
            if not candidate.is_absolute():
                candidate = root / candidate
            rel = candidate.resolve(strict=True).relative_to(root.resolve(strict=True)).as_posix()
            result[key] = {"path": rel, "sha256": _hash(_safe_member(root, rel))}
        else:
            rel = f"{key}.json"
            target = root / rel
            payload = (json.dumps(value, sort_keys=True, separators=(",", ":"),
                                  allow_nan=False) + "\n").encode()
            if target.exists():
                if target.read_bytes() != payload:
                    raise ValueError(f"existing manifest identity document conflicts: {rel}")
            else:
                fd, temporary = tempfile.mkstemp(prefix=f".{key}.", suffix=".tmp", dir=root)
                try:
                    with os.fdopen(fd, "wb") as handle:
                        handle.write(payload)
                        handle.flush()
                        os.fsync(handle.fileno())
                    os.link(temporary, target)
                finally:
                    try:
                        os.unlink(temporary)
                    except FileNotFoundError:
                        pass
            result[key] = {"path": rel, "sha256": _hash(_safe_member(root, rel)),
                           **({"status": "unqualified"} if key == "policy" else {})}
    _validate_identity_fields(result)
    return result


def snapshot_source_inputs(root, source_hashes):
    """Copy exact worker-identified input bytes into a manifest-local snapshot."""
    root = Path(root)
    snapshot = root / "inputs" / "source_tree"
    mapping, descriptors = {}, []
    sources = {str(Path(name).resolve(strict=True)): sha for name, sha in source_hashes.items()}
    if sources:
        common = Path(os.path.commonpath(list(sources)))
    else:
        common = Path.cwd()
    # Include track XML data and role-data references while preserving relative layout.
    import xml.etree.ElementTree as ET
    for original in list(sources):
        xml_path = Path(original)
        if xml_path.suffix.lower() != ".xml":
            continue
        root_element = ET.parse(xml_path).getroot()
        if root_element.get("type") == "track-model":
            for element in root_element.findall("./data"):
                ref = element.get("ref")
                if not ref:
                    continue
                dependency = (xml_path.parent / ref).resolve(strict=True)
                sources[str(dependency)] = _hash(dependency)
    # Resolve the same actual model-reference graph consumed by the model loader,
    # then include its data-file graph as well.
    xml_roots = [Path(name) for name in sources if Path(name).suffix.lower() == ".xml"]
    if xml_roots:
        from simplified_vehicle_models.xml.loader import load_model_shallow
        pending_models = []
        for path in xml_roots:
            try:
                model = load_model_shallow(path)
            except ValueError as error:
                if "unknown model type track-model/" in str(error):
                    continue  # Track XML uses its separate schema; track data refs are below.
                raise
            pending_models.append(path)
        visited_models = set()
        while pending_models:
            model_path = Path(pending_models.pop()).resolve(strict=True)
            if model_path in visited_models:
                continue
            visited_models.add(model_path)
            model = load_model_shallow(model_path)
            for reference in model.references:
                dependency = Path(reference.model.path).resolve(strict=True)
                expected = _hash(dependency)
                prior = sources.get(str(dependency))
                if prior is not None and prior != expected:
                    raise ValueError(f"transitive input identity mismatch: {dependency}")
                sources[str(dependency)] = expected
                pending_models.append(dependency)
    # Recompute shared root after resolving XML links and their relative payloads.
    common = Path(os.path.commonpath(list(sources))) if sources else Path.cwd()
    if common.is_file():
        common = common.parent
    root_resolved = root.resolve(strict=True)
    planned = []
    for original, expected in sorted(sources.items()):
        source = Path(original)
        if not source.is_file() or _hash(source) != expected:
            raise ValueError(f"worker source identity changed: {original}")
        relative = Path("inputs") / "source_tree" / source.relative_to(common)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"snapshot path escapes output root: {relative}")
        destination = root / relative
        planned.append((source, expected, relative, destination))

    # Preflight every target path before creating any directories or copying any
    # bytes. Reject symlinks at every existing component, not just the leaf.
    for _, _, relative, destination in planned:
        current = root
        for part in relative.parts:
            current = current / part
            if current.is_symlink():
                raise ValueError(f"snapshot path contains symlink: {current}")
            if current.exists():
                resolved = current.resolve(strict=True)
                if not resolved.is_relative_to(root_resolved):
                    raise ValueError(f"snapshot path escapes output root: {current}")
        if destination.exists() and not destination.is_file():
            raise ValueError(f"snapshot destination is not a regular file: {relative}")

    # Existing path components (including inputs and source_tree) are checked
    # before this first write-side directory creation.
    for parent in (root / "inputs", snapshot):
        if parent.is_symlink():
            raise ValueError(f"snapshot path contains symlink: {parent}")
        if parent.exists() and not parent.resolve(strict=True).is_relative_to(root_resolved):
            raise ValueError(f"snapshot path escapes output root: {parent}")
    snapshot.mkdir(parents=True, exist_ok=True)

    for source, expected, relative, destination in planned:
        # Recheck each parent immediately before creation/copy, including paths
        # created by earlier planned files; no symlink may redirect a write.
        current = root
        for part in relative.parts[:-1]:
            current = current / part
            if current.is_symlink():
                raise ValueError(f"snapshot path contains symlink: {current}")
            current.mkdir(exist_ok=True)
            if not current.resolve(strict=True).is_relative_to(root_resolved):
                raise ValueError(f"snapshot path escapes output root: {current}")
        if destination.is_symlink():
            raise ValueError(f"snapshot path contains symlink: {destination}")
        if destination.exists():
            if _hash(destination) != expected:
                raise ValueError(f"snapshot path collision: {relative}")
        else:
            import shutil
            shutil.copyfile(source, destination)
        if _hash(destination) != expected:
            raise ValueError(f"snapshot copy hash mismatch: {relative}")
        rel = relative.as_posix()
        mapping[str(source)] = rel
        descriptors.append({"path": rel, "sha256": expected})
    return mapping, descriptors


def _validate_identity_fields(data: dict) -> None:
    if not isinstance(data, dict) or set(data) != _REQUIRED:
        raise ValueError("manifest has missing or unknown identity fields")
    for key in ("track", "formulation", "chart", "validation_report"):
        value = data[key]
        if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
            raise ValueError(f"{key} requires path and sha256")
        sha = value["sha256"]
        if not isinstance(sha, str) or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            raise ValueError(f"{key} sha256 is invalid")
        if not isinstance(value["path"], str) or not value["path"]:
            raise ValueError(f"{key} path is invalid")
    if not isinstance(data["transitive_dependencies"], list):
        raise ValueError("transitive_dependencies must be a list")
    for key in ("effective_request", "runtime", "code", "boundary_event_contract", "policy"):
        if not isinstance(data[key], dict):
            raise ValueError(f"{key} must be an object")
    for key in ("effective_request", "runtime", "code", "boundary_event_contract", "policy"):
        descriptor = data[key]
        if not {"path", "sha256"} <= set(descriptor):
            raise ValueError(f"{key} must reference a content-hashed root-relative document")
        if not isinstance(descriptor["path"], str) or not descriptor["path"]:
            raise ValueError(f"{key} path is invalid")
        sha = descriptor["sha256"]
        if not isinstance(sha, str) or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            raise ValueError(f"{key} sha256 is invalid")
    for dependency in data["transitive_dependencies"]:
        if not isinstance(dependency, dict) or set(dependency) != {"path", "sha256"}:
            raise ValueError("each transitive dependency requires path and sha256")
    json.dumps(data, sort_keys=True, allow_nan=False)


def _verify_identities(root: Path, data: dict) -> None:
    entries = [data[k] for k in ("track", "formulation", "chart", "validation_report",
                                 "effective_request", "runtime", "code",
                                 "boundary_event_contract", "policy")]
    entries.extend(data["transitive_dependencies"])
    seen = {}
    for entry in entries:
        name = entry["path"]
        prior = seen.get(name)
        if prior is not None and prior != entry["sha256"]:
            raise ValueError(f"conflicting hashes for manifest member: {name}")
        seen[name] = entry["sha256"]
        member = _safe_member(root, name)
        if _hash(member) != entry["sha256"]:
            raise ValueError(f"manifest member hash mismatch: {name}")


def write_run_manifest_v1(root, identities, *, trajectory_path=_TRAJECTORY):
    """Atomically create exclusive manifest after hashing loader-format trajectory."""
    root = Path(root)
    _validate_identity_fields(identities)
    if trajectory_path != _TRAJECTORY:
        raise ValueError(f"loader trajectory must be exactly {_TRAJECTORY}")
    trajectory = _safe_member(root, _TRAJECTORY)
    # The exact trajectory loader validates field-set, metadata, arrays and semantics.
    try:
        from simplified_vehicle_models.solvers.trajectory_artifact import load_trajectory_artifact
        load_trajectory_artifact(trajectory)
    except (OSError, ValueError) as exc:
        raise ValueError(f"invalid loader trajectory artifact: {exc}") from exc
    _verify_identities(root, identities)
    manifest = {"schema": "run_manifest_v1",
                "trajectory": {"filename": _TRAJECTORY, "sha256": _hash(trajectory)},
                **identities,
                "qualification": {"approved": False, "status": "unqualified"}}
    payload = (json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    target = root / _MANIFEST
    fd, temp_name = tempfile.mkstemp(prefix=".run_manifest_v1.", suffix=".tmp", dir=root)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        # Hard-link provides exclusive publication; never replace an existing manifest.
        os.link(temp_name, target)
    finally:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
    return target


def load_run_manifest_v1(root):
    """Read sidecar, validate safe member and exact content hash; never approve."""
    root = Path(root)
    path = _safe_member(root, _MANIFEST)
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid run manifest: {exc}") from exc
    allowed = _REQUIRED | {"schema", "trajectory", "qualification"}
    if not isinstance(manifest, dict) or set(manifest) != allowed or manifest.get("schema") != "run_manifest_v1":
        raise ValueError("wrong run manifest format or unexpected fields")
    trajectory = manifest.get("trajectory")
    if not isinstance(trajectory, dict) or set(trajectory) != {"filename", "sha256"}:
        raise ValueError("manifest trajectory has missing or unexpected fields")
    if trajectory.get("filename") != _TRAJECTORY:
        raise ValueError("manifest does not identify the exact loader trajectory")
    member = _safe_member(root, _TRAJECTORY)
    if trajectory.get("sha256") != _hash(member):
        raise ValueError("trajectory member hash mismatch")
    from simplified_vehicle_models.solvers.trajectory_artifact import load_trajectory_artifact
    load_trajectory_artifact(member)
    identities = {key: manifest.get(key) for key in _REQUIRED}
    _validate_identity_fields(identities)
    _verify_identities(root, identities)
    qualification = manifest.get("qualification")
    if (not isinstance(qualification, dict)
            or set(qualification) != {"approved", "status"}
            or qualification.get("approved") is not False
            or qualification.get("status") != "unqualified"):
        raise ValueError("manifest qualification must remain explicitly unqualified")
    return manifest
