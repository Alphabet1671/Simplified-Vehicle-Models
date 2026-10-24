"""Spawned, deadline-owning supervisor for minimum-time harness attempts.

The child entry point is deliberately fixed; requests cannot select Python code.
"""
from __future__ import annotations

import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import time


def _safe(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("request values must be finite")
        return value
    if isinstance(value, list):
        return [_safe(v) for v in value]
    if isinstance(value, tuple):
        return [_safe(v) for v in value]
    if isinstance(value, dict) and all(isinstance(k, str) for k in value):
        return {k: _safe(v) for k, v in value.items()}
    raise TypeError("request must contain only JSON-safe values")


def _write_new(path: Path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, allow_nan=False)


def _worker(request_path: str, result_path: str, artifact_path: str):
    """Fixed spawn target; private module replacement is a test-only seam."""
    try:
        from tools.diagnostics.verify_no_tv_minimum_lap import _attempt_worker
        _attempt_worker(request_path, result_path, artifact_path)
    except BaseException as error:
        # Also capture failures before the harness reaches its own exception guard.
        try:
            _write_new(Path(result_path), {"status": "worker_exception", "accepted": False,
                       "solver_succeeded": False, "solve_completed": False,
                       "evidence_complete": False, "error_type": type(error).__name__,
                       "error": str(error)})
        except FileExistsError:
            pass


def _inventory(root: Path):
    entries = []
    for path in sorted(root.iterdir()):
        if path.is_file() and path.name != "artifacts.json":
            data = path.read_bytes()
            entries.append({"path": path.name, "status": "present", "size_bytes": len(data),
                            "sha256": hashlib.sha256(data).hexdigest()})
    manifest = {"artifacts": entries, "evidence_complete": False}
    try:
        _write_new(root / "artifacts.json", manifest)
    except FileExistsError:
        pass
    return manifest


def run_request(request: dict, output_directory: str) -> dict:
    """Run the fixed harness worker under a hard end-to-end wall deadline.

    Private `_worker` is a test seam for importable spawn-safe fixtures and is
    intentionally not read from request JSON.
    """
    started_at = time.monotonic()
    request = _safe(request)
    cap = request.get("wall_cap_s")
    if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not math.isfinite(cap) or cap <= 0:
        raise ValueError("wall_cap_s must be finite and positive")
    root = Path(output_directory)
    root.mkdir(parents=True, exist_ok=False)
    _deadline = started_at + float(cap)
    if time.monotonic() >= _deadline:
        raise TimeoutError("deadline elapsed before worker spawn")
    request_path, result_path = root / "request.json", root / "result.json"
    artifact_path = root / "solution.npz"
    _write_new(request_path, request)
    cancel_path = request.get("cancel_marker_path")
    # Fixed worker; tests may replace this module-level Python seam, never JSON.
    worker = _worker
    proc = mp.get_context("spawn").Process(target=worker,
        args=(str(request_path), str(result_path), str(artifact_path)))
    proc.start()
    deadline = _deadline
    status = None
    while proc.is_alive():
        if cancel_path and Path(cancel_path).exists():
            status = "cancelled"
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            status = "wall_timeout"
            break
        proc.join(min(0.05, remaining))
    # Child completion is authoritative only if observed before the hard cap.
    if status is None and time.monotonic() >= deadline:
        status = "wall_timeout"
    if status:
        proc.terminate()
        proc.join(0.25)
        termination = "terminated"
        if proc.is_alive():
            if hasattr(proc, "kill"):
                proc.kill()
            proc.join(0.5)
            termination = "killed" if not proc.is_alive() else "reap_failed"
        active = _active_stage(root)
        verdict = {"status": status, "accepted": False, "solver_succeeded": False,
                   "solve_completed": active in ("validate", "export", "complete"), "evidence_complete": False,
                   "stage": active, "termination": termination,
                   "error_type": "TimeoutError" if status == "wall_timeout" else "Cancelled",
                   "wall_cap_s": cap}
        name = "timeout_result.json" if status == "wall_timeout" else "cancel_result.json"
        try:
            _write_new(root / name, verdict)
        except FileExistsError:
            pass
        result = verdict
    elif result_path.exists():
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            result = {"status": "worker_malformed_result", "accepted": False,
                      "solver_succeeded": False, "solve_completed": False,
                      "evidence_complete": False, "error_type": "WorkerResultError"}
            try:
                result["raw_result"] = result_path.read_text(encoding="utf-8")
            except OSError:
                pass
        if not isinstance(result, dict):
            result = {"status": "worker_malformed_result", "accepted": False,
                      "solver_succeeded": False, "solve_completed": False,
                      "evidence_complete": False, "error_type": "WorkerResultError"}
        result.setdefault("solve_completed", bool(result.get("solver_succeeded")))
        result.setdefault("evidence_complete", False)
        result["accepted"] = False
        if proc.exitcode not in (0, None):
            result = {"status": "worker_exit_failure", "accepted": False,
                      "solver_succeeded": False, "solve_completed": False,
                      "evidence_complete": False, "error_type": "WorkerExit",
                      "exitcode": proc.exitcode,
                      "child_result": result}
    else:
        result = {"status": "worker_no_result", "accepted": False, "solver_succeeded": False,
                  "solve_completed": False, "evidence_complete": False,
                  "error_type": "WorkerExit", "exitcode": proc.exitcode}
        _write_new(result_path, result)
    result["accepted"] = False
    result.setdefault("solve_completed", bool(result.get("solver_succeeded")))
    result["evidence_complete"] = False
    try:
        _write_new(root / "supervisor_result.json", result)
    except FileExistsError:
        # A pre-existing output is rejected before this point; this is defensive.
        result = {"status": "supervisor_result_collision", "accepted": False,
                  "solve_completed": False, "evidence_complete": False}
    manifest = _inventory(root)
    result["artifacts"] = manifest["artifacts"]
    return result


def _active_stage(root: Path):
    path = root / "stage.json"
    try:
        return json.loads(path.read_text(encoding="utf-8")).get("stage", "setup")
    except (OSError, ValueError):
        return "setup"
