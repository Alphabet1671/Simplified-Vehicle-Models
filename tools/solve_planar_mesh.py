#!/usr/bin/env python3
"""Run reproducible planar OCP mesh experiments without hiding solver failure.

Usage:
  PYTHONPATH=src:/tmp/casadi_pkgs python3 tools/solve_planar_mesh.py \
      --spacings 20 10 5 2 1 --iterations 12000 --output /tmp/planar-mesh

A rung with nonzero physical defects or an unsuccessful IPOPT status stops the
ladder. For a cold-started isolated fine mesh, give only that spacing.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState,
    evaluate_planar_dynamics,
)
from simplified_vehicle_models.solvers.planar_ocp import (
    PlanarOcpOptions,
    _as_start,
    _build_path_data,
    _require_casadi,
    _solve,
)
from simplified_vehicle_models.tracks.profile import load_track

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spacings", nargs="+", type=float, default=[20, 10, 5, 2, 1])
    parser.add_argument("--iterations", type=int, default=12000)
    parser.add_argument("--output", type=Path, default=Path("/tmp/planar-mesh"))
    parser.add_argument("--exact-hessian", action="store_true")
    parser.add_argument("--equilibrium-seed", action="store_true",
                        help="start from a balanced low-speed turning trajectory")
    parser.add_argument("--equilibrium-speed", type=float, default=8.0)
    parser.add_argument("--projection", action="store_true",
                        help="experimentally project node algebraics before IPOPT")
    parser.add_argument("--scaling", action="store_true",
                        help="experimentally scale constraint rows")
    parser.add_argument("--restart", type=Path, default=None,
                        help="Start from a saved NPZ solution")
    parser.add_argument("--resume-same-mesh", action="store_true",
                        help="allow a non-converged checkpoint only at its original mesh")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    vehicle = compile_formulation(ROOT / "models/formulations/placeholder_planar_7dof.xml").model
    track = load_track(ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
    previous = None
    if args.restart is not None:
        with np.load(args.restart, allow_pickle=False) as saved:
            if not bool(saved["converged"]):
                same_mesh = (
                    args.resume_same_mesh
                    and np.array_equal(
                        saved["distance"],
                        _build_path_data(track, args.spacings[0])["distance"],
                    )
                )
                if not same_mesh:
                    raise ValueError(
                        "an unconverged checkpoint may only restart its own mesh; "
                        "pass --resume-same-mesh"
                    )
            previous = {key: saved[key] for key in ("distance", "state", "control", "algebraic")}
            for key in ("lam_x", "lam_g"):
                if key in saved.files:
                    previous[key] = saved[key]
    for spacing in args.spacings:
        options = PlanarOcpOptions(
            mesh_spacing=spacing,
            maximum_iterations=args.iterations,
            exact_hessian=args.exact_hessian,
            equilibrium_seed=args.equilibrium_seed,
            equilibrium_speed=args.equilibrium_speed,
            project_initial_guess=args.projection,
            scale_constraints=args.scaling,
            warm_start=False,
        )
        start = time.monotonic()
        result = _solve(_require_casadi(), vehicle, track, options, previous)
        duration = time.monotonic() - start
        reference = track.path_coordinates
        assert reference is not None
        sampled_path = _build_path_data(track, spacing)
        curvature = sampled_path["curvature"]
        interpolated = np.interp(reference.distance, result.distance, curvature)
        mismatch = float(np.max(np.abs(reference.curvature - interpolated)))
        heading_closure_error = abs(float(
            np.trapezoid(curvature, result.distance)
            - (sampled_path["heading"][-1] - sampled_path["heading"][0])
        ))
        # Re-evaluate feasible nodes independently with the NumPy seven-DOF
        # dynamics (which runs its own quasi-static vertical-load iteration).
        replay_load_error = 0.0
        replay_wheel_balance = 0.0
        replay_failures = 0
        if result.converged:
            for index in range(result.node_count):
                state = PlanarState(
                    float(result.v_x[index]), float(result.v_y[index]),
                    float(result.yaw_rate[index]), result.wheel_speeds[:, index],
                )
                evaluation = evaluate_planar_dynamics(
                    vehicle, state, result.wheel_torques[:, index],
                    float(result.steering[index]),
                )
                replay_failures += not evaluation.load_converged
                replay_load_error = max(replay_load_error, float(np.max(np.abs(
                    evaluation.normal_loads - result.normal_loads[:, index]
                ))))
                replay_wheel_balance = max(replay_wheel_balance, float(np.max(
                    np.abs(evaluation.wheel_accelerations * vehicle.wheel_inertia)
                )))
        replay_valid = (replay_failures == 0 and replay_load_error < 0.01
                        and replay_wheel_balance < 0.001)
        report = {
            "spacing_m": spacing,
            "nodes": result.node_count,
            "project_initial_guess": options.project_initial_guess,
            "scale_constraints": options.scale_constraints,
            "exact_hessian": options.exact_hessian,
            "maximum_allowed_pedal_overlap": options.maximum_throttle_brake_overlap,
            "equilibrium_seed": options.equilibrium_seed,
            "lap_time_s": result.lap_time,
            "status": result.status,
            "verified": bool(result.converged and replay_valid),
            "iterations": result.iterations,
            "wall_time_s": round(duration, 2),
            "maximum_algebraic_residual": result.maximum_load_residual,
            "maximum_dynamics_residual": result.maximum_dynamics_residual,
            "maximum_inequality_violation": result.maximum_inequality_violation,
            "maximum_power_mismatch_w": result.maximum_power_mismatch,
            "maximum_simultaneous_pedals": result.maximum_control_overlap,
            "periodicity_residual": result.periodicity_residual,
            "seam_control_jump": float(max(
                abs(result.throttle[-1] - result.throttle[0]),
                abs(result.brake[-1] - result.brake[0]),
                abs(result.steering[-1] - result.steering[0]),
            )),
            "seam_wheel_speed_jump_rad_per_s": float(np.max(np.abs(
                result.wheel_speeds[:, -1] - result.wheel_speeds[:, 0]
            ))),
            "maximum_curvature_interpolation_error_per_m": mismatch,
            "heading_curvature_closure_error_rad": heading_closure_error,
            "minimum_speed_m_per_s": float(np.min(result.speed)),
            "final_soc": float(result.state_of_charge[-1]),
            "numpy_load_replay_error_n": replay_load_error,
            "numpy_wheel_balance_error_nm": replay_wheel_balance,
            "numpy_load_failures": replay_failures,
            "lap_trustworthy": bool(
                result.converged and replay_valid and mismatch < 0.003
                and heading_closure_error < 0.01
                and result.maximum_control_overlap <= options.maximum_throttle_brake_overlap + 1e-6
            ),
        }
        print(json.dumps(report, indent=2), flush=True)
        filename = args.output / f"mesh-{spacing:g}m"
        (filename.with_suffix(".json")).write_text(json.dumps(report, indent=2) + "\n")
        state = _as_start(result)
        np.savez_compressed(
            filename.with_suffix(".npz"),
            **state,
            converged=np.asarray(report["verified"]),
        )
        if not report["verified"]:
            print("STOP: physical feasibility, NumPy replay or solver status not certified", flush=True)
            break
        previous = state


if __name__ == "__main__":
    main()
