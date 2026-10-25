"""Headless subprocess boundary for simulation-only maneuver definitions."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
from queue import Empty, Queue

from tools.maneuver_gui.artifacts import create_attempt_directory, write_artifacts


def _rejection(directory: Path, message: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    payload = {"execution_outcome": "failed", "input_fidelity": "envelope_rejected",
               "objective": "not_evaluated", "error": str(message)}
    temp = directory / f".summary.{uuid.uuid4().hex}.tmp"
    temp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temp, directory / "summary.json")


def _cancel_requested(cancel) -> bool:
    if cancel is None:
        return False
    return bool(cancel() if callable(cancel) else cancel.is_set())


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _terminate_group(proc, grace: float) -> None:
    """Signal the process group, then unconditionally SIGKILL after grace."""
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    deadline = time.monotonic() + max(0.0, grace)
    while time.monotonic() < deadline:
        try:
            os.killpg(proc.pid, 0)
        except ProcessLookupError:
            break
        except PermissionError:
            break
        time.sleep(min(0.02, deadline - time.monotonic()))
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=max(1.0, grace))
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_child(command, *, on_progress=None, cancel=None, timeout_s=None,
              terminate_timeout_s: float = 2.0, env=None, cwd=None) -> int:
    """Run a child in its own process group, forwarding JSON progress events.

    ``cancel`` may be a callback or an event-like object with ``is_set``. A
    cancellation or timeout sends SIGTERM to the entire group, then SIGKILL
    after ``terminate_timeout_s``. Returns the child's exit status (cancellation
    normally yields a negative signal status); timeouts raise TimeoutExpired.
    """
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace", bufsize=1,
                            start_new_session=True, env=env, cwd=cwd)
    lines = Queue()

    def read_output():
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                lines.put(line)
        finally:
            lines.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    timed_out = False
    eof = False
    try:
        while not eof:
            if _cancel_requested(cancel):
                break
            if deadline is not None and time.monotonic() >= deadline:
                timed_out = True
                break
            try:
                line = lines.get(timeout=0.05)
            except Empty:
                if proc.poll() is not None and not reader.is_alive():
                    break
                continue
            if line is None:
                eof = True
                continue
            line = line.rstrip("\\r\\n")
            try:
                event = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                event = None
            if isinstance(event, dict) and "event" in event:
                if on_progress is not None:
                    on_progress(event)
            elif line:
                print(line, file=sys.stderr, flush=True)
        if not eof:
            _terminate_group(proc, terminate_timeout_s)
            if timed_out:
                raise subprocess.TimeoutExpired(command, timeout_s)
            return proc.returncode
        # Leader completion is not group completion: kill lingering descendants
        # only if the group still exists, including TERM-ignoring grandchildren.
        if not _group_exists(proc.pid):
            return proc.wait()
        _terminate_group(proc, terminate_timeout_s)
        return proc.returncode
    finally:
        if _group_exists(proc.pid):
            _terminate_group(proc, terminate_timeout_s)
        else:
            proc.wait()
        if proc.stdout:
            proc.stdout.close()
        reader.join(timeout=terminate_timeout_s)


def _allowed_formulation(value: str) -> Path:
    """Only compile a checked-in model formulation, never arbitrary paths."""
    candidate = Path(value).expanduser()
    repo = Path(__file__).resolve().parents[2]
    models = (repo / "models").resolve()
    resolved = candidate.resolve() if candidate.is_absolute() else (repo / candidate).resolve()
    if resolved.suffix.lower() != ".xml" or models not in resolved.parents:
        raise ValueError("formulation must resolve to an XML file under the repository models/ root")
    if not resolved.is_file() or not resolved.resolve(strict=True):
        raise ValueError("formulation does not exist")
    return resolved


def run_definition(definition_path: Path, output_root: Path, *, cancel=None, timeout_s=305) -> Path:
    """Validate and run a definition in an isolated Python child process."""
    attempt = create_attempt_directory(output_root)
    source = Path(definition_path).expanduser().resolve()
    stage = None
    source_copy = None
    temp_summary = None
    final = None
    try:
        from simplified_vehicle_models.controls.maneuvers.schema import load_definition
        definition = load_definition(source)  # strict validation before child execution
        data = definition.to_dict()
        if data["backend"] != "simulation":
            raise ValueError("headless runner accepts simulation backend only; no physical commands are available")
        # Task5 calibration records embed model and actuator mappings under artifact.
        calibration = data.get("calibration") or {}
        artifact = calibration.get("artifact", calibration)
        if not isinstance(artifact, dict):
            raise ValueError("simulation calibration artifact must be a JSON object")
        vehicle_spec = data.get("vehicle", {})
        formulation = vehicle_spec.get("formulation") or vehicle_spec.get("model")
        if not formulation:
            raise ValueError("vehicle.model must name a planar-7dof formulation path")
        formulation_path = _allowed_formulation(str(formulation))
        vehicle_spec["model"] = str(formulation_path)
        source_copy = attempt / "maneuver-source.json"
        source_copy.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        stage = attempt.parent / f".{attempt.name}.{uuid.uuid4().hex}.result-stage"
        stage.mkdir()
        helper = r'''import json, sys, traceback
from pathlib import Path
from simplified_vehicle_models.controls.maneuvers.schema import load_definition
from simplified_vehicle_models.controls.maneuvers.program import ManeuverProgram
from simplified_vehicle_models.solvers.maneuver_planar import ActuatorCalibration, ManeuverPlanarPlant
from simplified_vehicle_models.solvers.maneuver_simulation import SimulationSettings, simulate_maneuver
from simplified_vehicle_models.compiler.compile import compile_formulation
from tools.maneuver_gui.artifacts import write_artifacts
p=Path(sys.argv[1]); out=Path(sys.argv[2]); d=load_definition(p); data=d.to_dict()
try:
    cal=data.get('calibration') or {}; artifact=cal.get('artifact', cal)
    vehicle_spec=data['vehicle']; formulation=vehicle_spec.get('formulation') or vehicle_spec.get('model')
    if not formulation: raise ValueError('vehicle.model must name a planar-7dof formulation path')
    model=compile_formulation(formulation).model
    actuator=ActuatorCalibration.from_dict(artifact)
    plant=ManeuverPlanarPlant(model, actuator); initial=plant.initial_state(**{k:v for k,v in data['initial'].items() if k in ('speed_mps','x_m','y_m','heading_rad')})
    program=ManeuverProgram(d)
    result=simulate_maneuver(plant, program, initial, SimulationSettings(tick_s=float(data['clock']['controller_tick_s']), max_wall_s=300.))
    outcome=result.execution_outcome
    summary={'execution_outcome':outcome,'input_fidelity':'exact','objective':'not_evaluated','termination_reason':result.termination_reason}
    write_artifacts(out,d,result,summary)
    print(json.dumps({'event':'complete','time_s':float(result.time_s[-1]),'fraction':1.0,'phase':result.termination_reason}), flush=True)
    sys.exit(0 if outcome == 'completed' else 2)
except BaseException as e:
    summary={'execution_outcome':'failed','input_fidelity':'envelope_rejected','objective':'not_evaluated','error':str(e)}
    (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    traceback.print_exc(); sys.exit(2)
'''
        command = [sys.executable, "-c", helper, str(source_copy), str(stage)]
        env = os.environ.copy()
        root = Path(__file__).resolve().parents[2]
        env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(root / "src"), str(root), env.get("PYTHONPATH", "")]))
        def forward_progress(event):
            if {"event", "time_s", "fraction", "phase"}.issubset(event):
                print(json.dumps(event), flush=True)
        try:
            code = run_child(command, on_progress=forward_progress, cancel=cancel, timeout_s=timeout_s, env=env)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            import shutil
            shutil.rmtree(stage, ignore_errors=True)
            source_copy.unlink(missing_ok=True)
            _rejection(attempt, "simulation interrupted or timed out")
            return attempt
        if code < 0 or _cancel_requested(cancel):
            import shutil
            shutil.rmtree(stage, ignore_errors=True)
            _rejection(attempt, "cancelled")
            return attempt
        if code != 0:
            import shutil
            shutil.rmtree(stage, ignore_errors=True)
            _rejection(attempt, f"simulation child exited {code}")
            return attempt
        # Validate in isolation then publish the complete result as one atomic
        # directory entry; never copy files individually into the attempt.
        from tools.maneuver_gui.artifacts import load_manifest
        load_manifest(stage)
        # Prepare every fallible summary operation and remove the source copy
        # before publication. The result directory is the final success marker.
        summary_bytes = (stage / "summary.json").read_bytes()
        summary_doc = json.loads(summary_bytes)
        if summary_doc.get("execution_outcome") != "completed":
            raise ValueError("child result is not completed")
        temp_summary = attempt / f".summary.{uuid.uuid4().hex}.tmp"
        temp_summary.write_bytes(summary_bytes)
        source_copy.unlink(missing_ok=True)
        final = attempt / "result"
        os.replace(stage, final)
        stage = None
        os.replace(temp_summary, attempt / "summary.json")
        return attempt
    except Exception as exc:
        import shutil
        if final is not None:
            shutil.rmtree(final, ignore_errors=True)
        if stage is not None:
            shutil.rmtree(stage, ignore_errors=True)
            stage = None
        _rejection(attempt, exc)
        return attempt
    finally:
        if stage is not None:
            import shutil
            shutil.rmtree(stage, ignore_errors=True)
        if source_copy is not None:
            source_copy.unlink(missing_ok=True)
        if temp_summary is not None:
            temp_summary.unlink(missing_ok=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definition", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    args = parser.parse_args(argv)
    attempt = run_definition(args.definition, args.output_root)
    try:
        summary = json.loads((attempt / "summary.json").read_text(encoding="utf-8"))
    except Exception:
        return 2
    return 0 if summary.get("execution_outcome") == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
