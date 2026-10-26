"""Bind a loaded energy-Hermite artifact to the explicit 3h chart checker.

This opt-in record checks a *real polynomial extension whose parameters are
binary64 cached-node values*.  It does not assert exactness of analytic-node
acceleration/metric physics: cached G can differ from analytic G by rounding.
It is not an energy_hs_v1 physical qualification or a finite-public-evaluation
certificate, and does not change artifact loading or interpretation.
"""
from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.chart_element_certificate import (
    ChartElementSettings, _settings, check_chart_element,
)
from simplified_vehicle_models.solvers.trajectory_artifact import LoadedTrajectory, load_trajectory_artifact
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory

_SEMANTICS = "binary64_cached_node_Hermite_Lagrange_real_extension_v1"
_CONTROL = "quadratic_energy_rate_over_metric_v1"
_STATE = "energy_hs_v1"


def _fail(reason, started, **extra):
    out = {"status": "not_proven", "chart_constraints_proven": False, "reason": reason,
           "binding_semantics": _SEMANTICS, "finite_public_eval_not_certified": True,
           "solver_artifact_qualified": False, "physical_node_controls_exactly_preserved": False,
           "accepted": False, "physical_feasibility_certified": False,
           "original_vehicle_validated": False, "event_qualified": False,
           "U3_qualified": False, "policy_approved": False,
           "elapsed_wall_seconds": float(max(0., time.monotonic()-started))}
    out.update(extra)
    return out


def _snapshot(loaded):
    ev, tr = loaded.state_evaluator, loaded.trajectory
    arrays = {}
    for name, owner in (("node_station", loaded), ("mesh", ev), ("speed", ev),
                        ("acceleration", ev), ("midpoint_acceleration", ev),
                        ("metric_nodes", ev), ("metric_midpoints", ev)):
        value = np.asarray(getattr(owner, name))
        if value.dtype != np.dtype("float64"):
            raise ValueError(f"{name} must retain binary64 representation")
        arrays[name] = np.array(value, dtype="<f8", copy=True)
    # Fingerprint actual in-memory source data used by geometry, not just the NPZ hash.
    # Fingerprint source samples and exact spline kernels actually evaluated.
    for name, value in (("reference_station", tr.reference.source_station),
                        ("reference_x_t", tr.reference._x.t), ("reference_x_c", tr.reference._x.c),
                        ("reference_x_k", tr.reference._x.k),
                        ("reference_y_t", tr.reference._y.t), ("reference_y_c", tr.reference._y.c),
                        ("reference_y_k", tr.reference._y.k),
                        ("offset_control_station", tr.spline.control_distance),
                        ("offset_coefficients", tr.coefficients),
                        ("offset_curve_x", tr._offset_curve.x), ("offset_curve_c", tr._offset_curve.c)):
        raw = np.asarray(value)
        if raw.dtype != np.dtype("float64") and name not in ("reference_x_k", "reference_y_k"):
            raise ValueError(f"source {name} must retain binary64 representation")
        # Spline degrees are discrete integer metadata, not floating parameters.
        arrays["source_"+name] = np.array(raw, dtype=("<i8" if name.endswith("_k") else "<f8"), copy=True)
    h = hashlib.sha256()
    h.update(json.dumps({"state": loaded.state_repr, "control": loaded.control_repr,
                         "evaluator_control": ev.control_representation,
                         "quadrature": loaded.time_quadrature,
                         "is_closed": bool(tr.reference.is_closed),
                         "interpolation_periodic": bool(tr.reference.interpolation_periodic),
                         "reference_start": float(tr.reference.start),
                         "reference_end": float(tr.reference.end)},
                        sort_keys=True, separators=(",", ":")).encode())
    for name in sorted(arrays):
        a = arrays[name]
        h.update(name.encode()+b"\0")
        h.update(np.asarray(a.shape, dtype="<i8").tobytes())
        h.update(a.tobytes(order="C"))
    return arrays, h.hexdigest()


def check_artifact_chart_element(loaded_or_path, surface, element_index, *, settings=None,
                                 speed_domain_margin_mps=0., minimum_fold=.2,
                                 maximum_speed_rate_defect=1e-6) -> dict:
    """Check one loaded artifact element against a signed chart, fail-closed.

    For paths, file identity is SHA-256 verified by the unchanged strict loader.
    For a LoadedTrajectory, the fingerprint identifies its current in-memory
    arrays/source kernels; the original file cannot be reverified from that object.
    """
    started = time.monotonic()
    loaded = None
    file_verified = False
    try:
        st = ChartElementSettings() if settings is None else settings
        wall = _settings(st)
        if isinstance(loaded_or_path, (str, Path)):
            loaded = load_trajectory_artifact(loaded_or_path)
            file_verified = True
        elif type(loaded_or_path) is LoadedTrajectory:
            loaded = loaded_or_path
        else:
            raise TypeError("expected a LoadedTrajectory or artifact path")
        if type(surface) is not SignedTransientCapability:
            raise TypeError("unsupported chart class")
        from simplified_vehicle_models.solvers.qss_spline_ocp import _SplineStateEvaluator, TIME_QUADRATURE_NAME
        if (type(loaded.state_evaluator) is not _SplineStateEvaluator or
                type(loaded.trajectory) is not OffsetTrajectory or
                loaded.state_evaluator.trajectory is not loaded.trajectory):
            raise ValueError("loaded trajectory/evaluator identity mismatch")
        ev = loaded.state_evaluator
        if (loaded.control_repr != _CONTROL or ev.control_representation != _CONTROL or
                loaded.state_repr != _STATE or loaded.time_quadrature != TIME_QUADRATURE_NAME):
            raise ValueError("unsupported control/state/time representation")
        if isinstance(element_index, bool) or not isinstance(element_index, int):
            raise TypeError("element_index must be an integer non-bool")
        arrays, fingerprint = _snapshot(loaded)
        mesh = arrays["mesh"]
        source_station = arrays["source_reference_station"]
        domain_start, domain_end = float(source_station[0]), float(source_station[-1])
        if (not math.isfinite(loaded.trajectory.reference.start)
                or not math.isfinite(loaded.trajectory.reference.end)
                or not domain_start < domain_end
                or loaded.trajectory.reference.start != domain_start
                or loaded.trajectory.reference.end != domain_end
                or not np.array_equal(mesh[[0, -1]], np.array([domain_start, domain_end]))):
            raise ValueError("artifact mesh must cover the exact analytic reference domain")
        n = mesh.size
        if (arrays["node_station"].shape != mesh.shape or not np.array_equal(arrays["node_station"], mesh)
                or arrays["speed"].shape != mesh.shape or arrays["acceleration"].shape != mesh.shape
                or arrays["midpoint_acceleration"].shape != (n-1,)
                or arrays["metric_nodes"].shape != mesh.shape or arrays["metric_midpoints"].shape != (n-1,)
                or n < 2 or not all(np.all(np.isfinite(a)) for a in arrays.values())
                or np.any(np.diff(mesh) <= 0) or np.any(arrays["speed"] <= 0)
                or np.any(arrays["metric_nodes"] <= 0) or np.any(arrays["metric_midpoints"] <= 0)):
            raise ValueError("invalid or incoherent artifact arrays")
        if not 0 <= element_index < n-1:
            raise ValueError("element_index outside mesh")
        # Cached metrics must be those produced by this exact trajectory evaluator.
        mid = .5*(mesh[:-1]+mesh[1:])
        if (not np.array_equal(np.asarray(loaded.trajectory.evaluate(mesh).metric), arrays["metric_nodes"])
                or not np.array_equal(np.asarray(loaded.trajectory.evaluate(mid).metric), arrays["metric_midpoints"])):
            raise ValueError("cached metric does not match loaded trajectory")
        i = element_index
        # Preserve NumPy binary64 operator semantics before exact rationalization.
        W = np.square(arrays["speed"][[i, i+1]])
        F = 2.0 * arrays["acceleration"][[i, i+1]] * arrays["metric_nodes"][[i, i+1]]
        Fmid = 2.0 * arrays["midpoint_acceleration"][i] * arrays["metric_midpoints"][i]
        if not np.all(np.isfinite(np.r_[W,F,Fmid])):
            raise ValueError("node energy/rate overflow")
        wq = tuple(Fraction.from_float(float(x)) for x in W)
        fq = tuple(Fraction.from_float(float(x)) for x in (F[0], Fmid, F[1]))
        setup_elapsed = time.monotonic()-started
        remaining = wall-setup_elapsed
        if remaining <= 0:
            return _fail("wall_budget_exhausted_during_setup", started, artifact_sha256=loaded.source_sha256,
                         input_fingerprint_sha256=fingerprint, artifact_file_identity_reverified=file_verified)
        delegated_settings = replace(st, maximum_wall_seconds=remaining)
        child = check_chart_element(loaded.trajectory, surface, (float(mesh[i]), float(mesh[i+1])),
                                    wq, fq, settings=delegated_settings,
                                    speed_domain_margin_mps=speed_domain_margin_mps,
                                    minimum_fold=minimum_fold,
                                    maximum_speed_rate_defect=maximum_speed_rate_defect)
        _, after = _snapshot(loaded)
        elapsed = time.monotonic()-started
        mutated = after != fingerprint
        proof = bool(child.get("chart_constraints_proven")) and not mutated and elapsed < wall
        reason = "cached_polynomial_extension_proven" if proof else (
            "source_mutated_during_check" if mutated else
            "wall_budget_exhausted" if elapsed >= wall else "element_not_proven")
        out = dict(child)
        out.update(status="chart_constraints_proven" if proof else "not_proven",
                   chart_constraints_proven=proof, reason=reason, binding_semantics=_SEMANTICS,
                   artifact_sha256=str(loaded.source_sha256), element_index=i,
                   source_interval=[float(mesh[i]), float(mesh[i+1])],
                   source_interval_exact=[_fraction_pair(x) for x in (Fraction.from_float(float(mesh[i])), Fraction.from_float(float(mesh[i+1])))],
                   energy_endpoints_exact=[_fraction_pair(x) for x in wq],
                   energy_rate_sites_exact=[_fraction_pair(x) for x in fq],
                   input_fingerprint_sha256=fingerprint, artifact_file_identity_reverified=file_verified,
                   solver_artifact_qualified=False, physical_node_controls_exactly_preserved=False,
                   finite_public_eval_not_certified=True, accepted=False,
                   physical_feasibility_certified=False, original_vehicle_validated=False,
                   event_qualified=False, U3_qualified=False, policy_approved=False,
                   source_mutation_detected=mutated, elapsed_wall_seconds=float(max(0.,elapsed)))
        if not proof:
            out["element_check"] = child
        return out
    except Exception as exc:
        return _fail(f"input_error:{type(exc).__name__}:{exc}", started,
                     artifact_sha256=(str(loaded.source_sha256) if loaded is not None else None),
                     artifact_file_identity_reverified=file_verified)


def _fraction_pair(q):
    return {"numerator": q.numerator, "denominator": q.denominator}
