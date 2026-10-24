"""Spawned, deadline-owning supervisor for minimum-time harness attempts.

The child entry point is deliberately fixed; requests cannot select Python code.
"""
from __future__ import annotations

import json
import hashlib
import math
import multiprocessing as mp
from pathlib import Path
import time


MAX_REQUEST_BYTES = 1024 * 1024
MAX_CONTROL_BYTES = 256 * 1024
MAX_RESULT_BYTES = 1024 * 1024
MAX_DEPTH = 32
MAX_ITEMS = 100_000
MAX_PATH_CHARS = 4096


def _safe(value, depth=0, counter=None):
    if counter is None:
        counter = [0, 0]
    counter[0] += 1
    if depth > MAX_DEPTH or counter[0] > MAX_ITEMS:
        raise ValueError("request exceeds depth/item resource limit")
    if value is None or isinstance(value, (str, bool, int)):
        if isinstance(value, str):
            if len(value) > MAX_REQUEST_BYTES:
                raise ValueError("request exceeds byte resource limit")
            counter[1] += len(value.encode("utf-8"))
            if counter[1] > MAX_REQUEST_BYTES:
                raise ValueError("request exceeds byte resource limit")
        elif isinstance(value, int):
            approx = (value.bit_length() + 2) // 3
            counter[1] += approx
            if counter[1] > MAX_REQUEST_BYTES or approx > MAX_REQUEST_BYTES:
                raise ValueError("request integer exceeds byte resource limit")
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("request values must be finite")
        return value
    if isinstance(value, list):
        return [_safe(v, depth + 1, counter) for v in value]
    if isinstance(value, tuple):
        return [_safe(v, depth + 1, counter) for v in value]
    if isinstance(value, dict) and all(isinstance(k, str) for k in value):
        if any(len(k) > MAX_PATH_CHARS for k in value):
            raise ValueError("request key exceeds resource limit")
        for key in value:
            counter[1] += len(key.encode("utf-8"))
        if counter[1] > MAX_REQUEST_BYTES:
            raise ValueError("request exceeds byte resource limit")
        return {k: _safe(v, depth + 1, counter) for k, v in value.items()}
    raise TypeError("request must contain only JSON-safe values")


def _bounded_json(path: Path, limit: int):
    with path.open("rb") as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"control file exceeds {limit}-byte resource limit")
    return json.loads(data.decode("utf-8"))


def _write_new(path: Path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, allow_nan=False)


def _worker(request_path: str, result_path: str, artifact_path: str):
    """Fixed spawn target; dispatch only to repository-owned worker kinds."""
    try:
        request = _bounded_json(Path(request_path), MAX_REQUEST_BYTES)
        if request.get("worker_kind") == "public_spline_setup_v1":
            from tools.run.solve_minimum_time_line import _prepare_public_spline_request_in_worker
            _prepare_public_spline_request_in_worker(request, request_path, result_path, artifact_path)
        elif request.get("worker_kind") == "compat_line_v1":
            from tools.run.solve_minimum_time_line import _compat_worker_entry
            _compat_worker_entry(request, request_path, result_path, artifact_path)
        elif request.get("worker_kind") == "no_tv_spline_ladder_v1":
            from tools.diagnostics.verify_no_tv_minimum_lap import _attempt_ladder_worker
            _attempt_ladder_worker(request_path, result_path, artifact_path)
        else:
            from tools.diagnostics.verify_no_tv_minimum_lap import _attempt_worker
            _attempt_worker(request_path, result_path, artifact_path)
    except BaseException as error:
        try:
            _write_new(Path(result_path), {"status": "worker_exception", "accepted": False,
                       "solver_succeeded": False, "solve_completed": False,
                       "evidence_complete": False, "error_type": type(error).__name__,
                       "error": str(error)})
        except FileExistsError:
            pass


def _write_compact_artifact_index(root: Path, extra_paths=(), *, stage="complete", external_root=None):
    """Hash fixed artifact outputs; the volatile stage document is excluded."""
    allowed = {"summary.json", "result.json", "ladder_result.json", "public_result.json",
               "public_summary.snapshot.json", "compat_summary.snapshot.json", "line.csv", "racing_line.csv",
               "racing_line_track.csv", "racing_line_track.xml", "rebuilt_width.csv", "trajectory.npz"}
    names = ["attempt_index.json", "request.json", "prepared_ladder_request.json", "ladder_result.json"]
    for index in range(32):
        names.extend(f"attempts/rung-{index}/{name}" for name in
                     ("request.json", "result.json", "solution.npz", "trajectory_metadata.npz"))
    for name in extra_paths:
        if name in allowed and name not in names:
            names.append(name)
    if len(names) > 256:
        raise ValueError("artifact index exceeds 256-entry resource limit")
    entries = []
    for name in names:
        path = root / name
        try:
            if path.is_file():
                data = path.read_bytes()
                entries.append({"path": name, "base": "supervision", "status": "complete", "size_bytes": len(data),
                                "sha256": hashlib.sha256(data).hexdigest()})
            elif name in extra_paths and external_root is not None:
                external = external_root / name
                entries.append({"path": name, "base": "caller_output",
                                "status": "unknown_or_partial" if external.exists() else "not_created",
                                "sha256": None})
        except OSError:
            entries.append({"path": name, "base": "supervision", "status": "hash_unavailable", "sha256": None})
    manifest = {"artifacts": entries, "evidence_complete": False, "index_stage": stage}
    temp = root / "artifacts.json.tmp"
    temp.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    temp.replace(root / "artifacts.json")
    return manifest


def _inventory(root: Path):
    """Read worker digest index; absent index yields bounded known-name references."""
    try:
        manifest = _bounded_json(root / "artifacts.json", MAX_CONTROL_BYTES)
        if (not isinstance(manifest, dict) or not isinstance(manifest.get("artifacts"), list)
                or len(manifest["artifacts"]) > 256):
            raise ValueError("invalid artifact digest index")
        for item in manifest["artifacts"]:
            if not isinstance(item, dict) or not isinstance(item.get("path"), str):
                raise ValueError("invalid artifact entry")
            rel = Path(item["path"])
            if item.get("base", "supervision") not in {"supervision", "caller_output"}:
                raise ValueError("invalid artifact base")
            if item.get("base") == "caller_output" and item["path"] not in {
                    "summary.json", "line.csv", "racing_line.csv", "racing_line_track.csv",
                    "racing_line_track.xml", "rebuilt_width.csv", "trajectory.npz"}:
                raise ValueError("unsafe caller-output artifact reference")
            if rel.is_absolute() or ".." in rel.parts or len(item["path"]) > MAX_PATH_CHARS:
                raise ValueError("unsafe artifact reference")
        manifest["evidence_complete"] = False
        return manifest
    except (OSError, ValueError, UnicodeError, RecursionError):
        known = []
        for name in ("request.json", "stage.json", "result.json", "solution.npz", "timeout_result.json",
                     "cancel_result.json", "supervisor_result.json", "attempt_index.json", "prepared_ladder_request.json"):
            try:
                if (root / name).is_file():
                    known.append({"path": name, "status": "unknown_or_partial", "sha256": None})
            except OSError:
                continue
        return {"artifacts": known[:32], "evidence_complete": False,
                "digest_index_status": "unavailable_or_oversize"}


def _read_stage(root: Path):
    try:
        value = _bounded_json(root / "stage.json", MAX_CONTROL_BYTES)
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError, RecursionError, UnicodeError):
        return {}


def _persist_result(root: Path, result: dict) -> dict:
    result["accepted"] = False
    result["evidence_complete"] = False
    try:
        _write_new(root / "supervisor_result.json", result)
    except FileExistsError:
        result = {"status": "supervisor_result_collision", "accepted": False,
                  "solve_completed": bool(result.get("solve_completed")), "evidence_complete": False}
    manifest = _inventory(root)
    result["artifacts"] = manifest["artifacts"]
    try:
        _write_new(root / "artifacts.json", manifest)
    except (FileExistsError, OSError):
        pass
    return result


def _cleanup_process(proc):
    """Best-effort bounded terminate/kill/reap, preserving any cleanup error."""
    errors = []
    if proc is None:
        return "not_started", errors
    try:
        alive = proc.is_alive()
    except BaseException as error:
        errors.append(f"is_alive: {type(error).__name__}: {error}")
        alive = True
    if not alive:
        try:
            proc.join(0)
        except BaseException as error:
            errors.append(f"join: {type(error).__name__}: {error}")
        return "already_exited", errors
    try:
        proc.terminate()
    except BaseException as error:
        errors.append(f"terminate: {type(error).__name__}: {error}")
    try:
        proc.join(0.25)
    except BaseException as error:
        errors.append(f"join_after_terminate: {type(error).__name__}: {error}")
    try:
        alive = proc.is_alive()
    except BaseException as error:
        errors.append(f"is_alive_after_terminate: {type(error).__name__}: {error}")
        alive = True
    if alive:
        try:
            proc.kill()
        except BaseException as error:
            errors.append(f"kill: {type(error).__name__}: {error}")
        try:
            proc.join(0.5)
        except BaseException as error:
            errors.append(f"join_after_kill: {type(error).__name__}: {error}")
        try:
            alive = proc.is_alive()
        except BaseException as error:
            errors.append(f"is_alive_after_kill: {type(error).__name__}: {error}")
            alive = True
        return ("reap_failed" if alive else "killed"), errors
    return "terminated", errors


def run_request(request: dict, output_directory: str) -> dict:
    """Run the fixed harness worker under a hard end-to-end wall deadline.

    `_worker` is a private Python-only test seam; requests cannot choose code.
    """
    started_at = time.monotonic()
    if not isinstance(request, dict):
        raise TypeError("request must be a dict")
    request = _safe(request)
    encoded_request = json.dumps(request, sort_keys=True, allow_nan=False).encode("utf-8")
    if len(encoded_request) > MAX_REQUEST_BYTES:
        raise ValueError(f"request exceeds {MAX_REQUEST_BYTES}-byte resource limit")
    cap = request.get("wall_cap_s")
    if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not math.isfinite(cap) or cap <= 0:
        raise ValueError("wall_cap_s must be finite and positive")
    deadline = started_at + float(cap)
    if request.get("worker_kind") in {"public_spline_setup_v1", "compat_line_v1"}:
        output_text = request.get("output")
        if not isinstance(output_text, str) or len(output_text) > MAX_PATH_CHARS:
            raise ValueError("supervised output path exceeds resource limit")
        if request.get("worker_kind") == "compat_line_v1" and request.get("config", {}).get("output") != request.get("output"):
            raise ValueError("compatibility output path mismatch")
        if request.get("worker_kind") == "compat_line_v1" and set(request) != {"worker_kind", "wall_cap_s", "output", "config"}:
            raise ValueError("compatibility request has unknown options")
        if request.get("worker_kind") == "compat_line_v1":
            allowed = {"track", "output", "formulation", "surface", "margin", "half_width", "spacings",
                       "offset_spacing", "wall_cap_s", "path_smoothing", "control_mode", "control_policy",
                       "replay_adapter", "solver", "legacy_solver", "compare_control_modes"}
            if set(request["config"]) - allowed:
                raise ValueError("compatibility request has unknown config options")
    if request.get("worker_kind") == "public_spline_setup_v1":
        if not {"track", "formulation", "surface", "output", "margin", "half_width", "spacings",
                "offset_spacing", "path_smoothing"}.issubset(request):
            raise ValueError("public request missing fixed options")
        if set(request) != {"worker_kind", "wall_cap_s", "track", "formulation", "surface", "output",
                "margin", "half_width", "spacings", "offset_spacing", "path_smoothing"}:
            raise ValueError("public request has unknown options")
        if any(not isinstance(request[k], str) or len(request[k]) > MAX_PATH_CHARS
               for k in ("track", "formulation", "surface", "output")):
            raise ValueError("public request path exceeds resource limit")
    root = Path(output_directory)
    if request.get("worker_kind") in {"public_spline_setup_v1", "compat_line_v1"}:
        root.parent.mkdir(parents=True, exist_ok=True)
    root.mkdir(parents=True, exist_ok=False)
    request_path, result_path = root / "request.json", root / "result.json"
    artifact_path = root / "solution.npz"
    proc = None
    started = False
    status = None
    cleanup_status = "not_started"
    cleanup_errors = []
    try:
        request_path.write_bytes(encoded_request)
        if time.monotonic() >= deadline:
            status = "wall_timeout"
        else:
            proc = mp.get_context("spawn").Process(target=_worker,
                args=(str(request_path), str(result_path), str(artifact_path)))
            try:
                proc.start()
                started = True
            except BaseException as error:
                status = "worker_start_failure"
                cleanup_errors.append(f"start: {type(error).__name__}: {error}")
        cancel_path = request.get("cancel_marker_path")
        while started and status is None:
            try:
                if cancel_path and Path(cancel_path).exists():
                    status = "cancelled"
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    status = "wall_timeout"
                    break
                if not proc.is_alive():
                    proc.join(0)
                    if time.monotonic() >= deadline:
                        status = "wall_timeout"
                    break
                proc.join(min(0.05, remaining))
                if not proc.is_alive() and time.monotonic() >= deadline:
                    status = "wall_timeout"
            except BaseException as error:
                status = "supervisor_error"
                cleanup_errors.append(f"poll: {type(error).__name__}: {error}")
        if started and status is None and time.monotonic() >= deadline:
            status = "wall_timeout"
    except BaseException as error:
        status = "supervisor_error"
        cleanup_errors.append(f"supervision: {type(error).__name__}: {error}")
    finally:
        if proc is not None:
            if status is not None:
                cleanup_status, errors = _cleanup_process(proc)
                cleanup_errors.extend(errors)
            elif started:
                try:
                    proc.join(0)
                except BaseException as error:
                    cleanup_errors.append(f"final_join: {type(error).__name__}: {error}")
                    cleanup_status, errors = _cleanup_process(proc)
                    cleanup_errors.extend(errors)
                    status = "supervisor_error"

    stage = _read_stage(root)
    solve_completed = bool(stage.get("solve_completed"))
    any_rung_solve_completed = bool(stage.get("any_rung_solve_completed", solve_completed))
    ladder_solve_completed = bool(stage.get("ladder_solve_completed", solve_completed))
    if status is not None:
        result = {"status": status, "accepted": False, "solver_succeeded": False,
                  "solve_completed": ladder_solve_completed,
                  "ladder_solve_completed": ladder_solve_completed,
                  "any_rung_solve_completed": any_rung_solve_completed,
                  "evidence_complete": False,
                  "stage": stage.get("stage", "setup"), "termination": cleanup_status,
                  "error_type": "TimeoutError" if status == "wall_timeout" else status,
                  "wall_cap_s": cap}
        if cleanup_errors:
            result["cleanup_errors"] = cleanup_errors
        name = "timeout_result.json" if status == "wall_timeout" else "cancel_result.json" if status == "cancelled" else "supervisor_result.json"
        if name != "supervisor_result.json":
            try:
                _write_new(root / name, result)
            except FileExistsError:
                pass
    elif result_path.exists():
        try:
            child = _bounded_json(result_path, MAX_RESULT_BYTES)
            if not isinstance(child, dict):
                raise ValueError("worker result must be a JSON object")
            result = child
            result.setdefault("solve_completed", solve_completed or bool(result.get("solver_succeeded")))
            result.setdefault("evidence_complete", False)
            result["accepted"] = False
            if proc.exitcode not in (0, None):
                result = {"status": "worker_exit_failure", "accepted": False,
                          "solver_succeeded": False, "solve_completed": solve_completed,
                          "evidence_complete": False, "error_type": "WorkerExit",
                          "exitcode": proc.exitcode, "child_result": child}
        except (OSError, ValueError, TypeError) as error:
            try:
                with result_path.open("rb") as stream:
                    raw = stream.read(4097).decode("utf-8", errors="replace")
            except OSError:
                raw = ""
            result = {"status": "worker_malformed_result", "accepted": False,
                      "solver_succeeded": False, "solve_completed": solve_completed,
                      "evidence_complete": False, "error_type": "WorkerResultError",
                      "error": str(error), "raw_result": raw[:4096],
                      "raw_result_path": result_path.name}
    else:
        exitcode = None
        try:
            exitcode = proc.exitcode if proc is not None else None
        except BaseException:
            pass
        result = {"status": "worker_no_result", "accepted": False, "solver_succeeded": False,
                  "solve_completed": solve_completed, "evidence_complete": False,
                  "error_type": "WorkerExit", "exitcode": exitcode}
        try:
            _write_new(result_path, result)
        except (FileExistsError, OSError):
            pass
    if request.get("worker_kind") == "no_tv_spline_ladder_v1":
        result.setdefault("ladder_solve_completed", ladder_solve_completed)
        result.setdefault("any_rung_solve_completed", any_rung_solve_completed)
        result["solve_completed"] = bool(result.get("ladder_solve_completed"))
    elif request.get("worker_kind") == "public_spline_setup_v1":
        result["solve_completed"] = bool(result.get("solve_completed") or solve_completed)
    else:
        result["solve_completed"] = bool(result.get("solve_completed") or solve_completed)
    if cleanup_errors:
        result["cleanup_errors"] = cleanup_errors
        if status is None:
            result["supervisor_cleanup_warning"] = True
    return _persist_result(root, result)
