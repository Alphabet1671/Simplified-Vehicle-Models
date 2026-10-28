"""Profile one lap of the point-mass, planar 7DoF or vertical 14DoF model.

The run uses the settings of the four-way comparison attempts under
``docs/validation/iem24-murphy/four_way_latest_model/runs/``: the ten-lap MIS
track, 0.5 m spacing, the live planner, 60 m/s speed cap, 0.175 s pedal travel,
BDF with rtol 1e-5 / atol 1e-7, and a 0.02 s (7DoF) or 0.005 s (14DoF) maximum
step. It calls the library directly, so it writes no attempt directory, hashes
no sources and checks no gates.

Usage, from the repository root::

    OMP_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 PYTHONPATH=src:. \\
        python3 tools/diagnostics/profile_models.py MODEL [--no-profile] [--output DIR]

``MODEL`` is ``point_mass``, ``planar_7dof`` or ``vertical_14dof``. The tool
writes ``DIR/<MODEL>.json`` (phase wall times and counters) and, unless
``--no-profile``, ``DIR/<MODEL>.pstats`` plus a module/function summary in
``DIR/<MODEL>_summary.json``. cProfile roughly doubles the wall time of this
call-heavy code, so take absolute times from a ``--no-profile`` run and shares
from the profile. See ``docs/dev/computational-cost.md``.
"""
from __future__ import annotations

import argparse
import cProfile
from collections import defaultdict
import json
from pathlib import Path
import pstats
import time

import numpy as np

TRACK = Path("models/tracks/mis_ev_endurance_2024_multilap_linear.xml")
MODEL_NAMES = ("point_mass", "planar_7dof", "vertical_14dof")


def _module_of(path: str) -> str:
    """Group a source path into a repository, SciPy, NumPy or builtin module."""
    if "simplified_vehicle_models/" in path:
        return "svm/" + path.split("simplified_vehicle_models/", 1)[1]
    for package in ("scipy/", "numpy/"):
        if package in path:
            parts = path.split(package, 1)[1].split("/")
            return package + "/".join(parts[:2])
    return "builtins (C)" if path == "~" else "other"


def summarise(stats_path: Path, top: int = 40) -> dict:
    """Return self time by module and the top functions by self and cumulative time."""
    raw = pstats.Stats(str(stats_path)).stats
    total = sum(value[2] for value in raw.values())
    modules = defaultdict(float)
    for (path, _, _), value in raw.items():
        modules[_module_of(path)] += value[2]

    def label(key):
        return f"{_module_of(key[0])}:{key[1]}({key[2]})"

    def rows(items):
        return [{"function": label(k), "calls": v[1], "self_s": v[2], "cumulative_s": v[3]}
                for k, v in items]

    repository = [(k, v) for k, v in raw.items() if "simplified_vehicle_models" in k[0]]
    return {
        "profiled_self_total_s": total,
        "modules": [{"module": m, "self_s": t, "share": t / total}
                    for m, t in sorted(modules.items(), key=lambda x: -x[1])],
        "by_self_time": rows(sorted(raw.items(), key=lambda x: -x[1][2])[:top]),
        "by_cumulative_time": rows(sorted(repository, key=lambda x: -x[1][3])[:top]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("model", choices=MODEL_NAMES)
    parser.add_argument("--no-profile", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("/tmp/svm_profile"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    profiler = None if args.no_profile else cProfile.Profile()
    phases: dict[str, float] = {}
    counters: dict[str, object] = {}

    def phase(name, function, *call_args, **kwargs):
        start = time.perf_counter()
        if profiler:
            profiler.enable()
        try:
            return function(*call_args, **kwargs)
        finally:
            if profiler:
                profiler.disable()
            phases[name] = time.perf_counter() - start

    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.tracks.profile import load_track

    if args.model == "point_mass":
        from simplified_vehicle_models.results.traces import LapMode
        from simplified_vehicle_models.solvers.forward_backward import solve_lap_with_energy
        from tools.reports.compare_planar_point_mass import (
            FORMULATION, _point_mass_track, _point_mass_vehicle)

        planar = phase("compile_formulation",
                       lambda: compile_formulation(str(FORMULATION)).model)
        vehicle, _ = phase("reduce_to_point_mass", _point_mass_vehicle, planar)
        track = phase("load_and_mesh_track",
                      lambda: _point_mass_track(load_track(TRACK), 0.5))
        # Same call as tools/run/run_iem24_point_mass_track_pair.py.
        solution = phase("solve_lap_with_energy", solve_lap_with_energy, vehicle, track,
                         LapMode.FLYING, tolerance=1.e-5, maximum_iterations=40,
                         lap_tolerance=1.e-6, lap_maximum_iterations=200,
                         regen_enabled=False)
        counters.update(lap_time=float(solution.lap.lap_time),
                        lap_iterations=int(solution.lap.iterations),
                        energy_iterations=int(solution.energy.energy_iterations),
                        nodes=len(track.distance))
    else:
        from simplified_vehicle_models.solvers.forward_driver import (
            PreviewDriver, build_speed_plan, solve_forward)
        from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
        from tools.run.run_iem24_forward_driver import MODELS, plant_class_for

        spec = MODELS["planar-7dof" if args.model == "planar_7dof" else "14dof-simple"]
        maximum_step = .02 if args.model == "planar_7dof" else .005
        vehicle = phase("compile_formulation",
                        lambda: compile_formulation(str(spec.formulation)).model)
        control = getattr(vehicle, "planar", vehicle)
        track = phase("load_track", load_track, TRACK)
        distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / .5)) + 1)
        _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
        plan = phase("build_speed_plan_live", build_speed_plan, distance, curvature, 60.,
                     None, 1., 2., vehicle=control, live_limit=True)
        plant = phase("construct_plant", plant_class_for(spec, vehicle), vehicle, plan)
        driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
                               control.steering_maximum_rate, control.maximum_pedal_force,
                               live_limit=True, pedal_travel_time=.175)
        initial = phase("initial_state", plant.initial_state, float(plan.speed[0]))
        result = phase("solve_forward", solve_forward, plant, driver, initial,
                       duration=300., wall_seconds=3500., maximum_wall_seconds=3600.,
                       maximum_step=maximum_step, rtol=1.e-5, atol=1.e-7, method="BDF")
        counters.update(lap_time=float(result.time[-1]),
                        lap_completed=bool(result.lap_completed),
                        accepted_steps=int(result.time.size - 1),
                        integrator_rhs_evaluations=int(result.evaluations),
                        plant_evaluations=int(plant.evaluations),
                        state_size=int(plant.state_size),
                        rejected_domain_trials=int(result.rejected_domain_trials))

    stem = args.model + (".noprof" if args.no_profile else "")
    report = {"model": args.model, "profiled": profiler is not None,
              "phases_s": phases, "counters": counters}
    (args.output / f"{stem}.json").write_text(json.dumps(report, indent=2))
    if profiler:
        stats_path = args.output / f"{args.model}.pstats"
        profiler.dump_stats(stats_path)
        (args.output / f"{args.model}_summary.json").write_text(
            json.dumps(summarise(stats_path), indent=1))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
