"""Exact-coordinate diagnostics for a built spline NLP seed (never acceptance)."""
from __future__ import annotations

import hashlib
import math
import time

import numpy as np


def _json_number(value):
    value = float(value)
    return value if math.isfinite(value) else None


def inspect_spline_seed(built, seed, *, maximum_reported_rows=32) -> dict:
    """Evaluate physical ``f``/``g`` and bounds at *exactly* ``seed``.

    This is a diagnostic only. It neither modifies the seed nor certifies any
    feasibility or qualification policy.
    """
    started = time.monotonic()
    result = {
        "diagnostic_only": True,
        "accepted": False,
        "physical_feasibility_certified": False,
        "numerical_qualified": False,
        "event_qualified": False,
        "U3_qualified": False,
        "policy_approved": False,
        "built_problem_source_identity_verified": False,
        "finite_input": False,
        "evaluation_complete": False,
        "objective_value": None,
        "objective_nonfinite": False,
        "nonfinite_constraint_rows": [],
        "maximum_constraint_violation": None,
        "maximum_bound_violation": None,
        "worst_constraint_rows": [],
        "worst_bound_rows": [],
        "constraint_row_count": 0,
        "constraint_rows_reported": 0,
        "constraint_rows_truncated": False,
        "bound_row_count": 0,
        "bound_rows_reported": 0,
        "bound_rows_truncated": False,
        "coordinate_failure_rows": [],
        "constraint_family_maxima": {},
        "unclassified_constraint_maximum": None,
        "seed_sha256": None,
        "error": None,
    }
    try:
        if isinstance(maximum_reported_rows, (bool, np.bool_)) or not isinstance(maximum_reported_rows, (int, np.integer)) or maximum_reported_rows <= 0:
            raise ValueError("maximum_reported_rows must be a positive integer, not bool")
        import casadi as ca

        nlp = built["nlp"]
        xexpr, fexpr, gexpr = nlp["x"], nlp["f"], nlp["g"]
        nx, ng = int(xexpr.numel()), int(gexpr.numel())
        if int(fexpr.numel()) != 1:
            raise ValueError("objective must be scalar")
        count, controls = built["count"], built["control_count"]
        if isinstance(count, (bool, np.bool_)) or not isinstance(count, (int, np.integer)) or count <= 0:
            raise ValueError("count must be a positive integer")
        if isinstance(controls, (bool, np.bool_)) or not isinstance(controls, (int, np.integer)) or controls <= 0:
            raise ValueError("control_count must be a positive integer")
        mesh = np.asarray(built["mesh"])
        if mesh.ndim != 1 or mesh.size != count + 1 or not np.issubdtype(mesh.dtype, np.number) or np.iscomplexobj(mesh) or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0):
            raise ValueError("mesh must be finite, increasing, and have count+1 entries")
        if nx != controls + 3 * (count + 1) - 1:
            raise ValueError("NLP decision size does not match spline layout")
        if isinstance(seed, (list, tuple)) and any(isinstance(v, (bool, np.bool_)) for v in seed):
            raise ValueError("seed elements must not be bool")
        raw = np.asarray(seed)
        if raw.dtype.kind == "O" and raw.ndim == 1 and any(isinstance(v, (bool, np.bool_)) for v in raw):
            raise ValueError("seed elements must not be bool")
        if raw.ndim != 1 or raw.size != nx or raw.dtype.kind not in "iuf":
            raise ValueError("seed must be a 1-D real numeric non-bool vector of NLP length")
        try:
            x = np.asarray(raw, dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("seed cannot be represented as float64") from exc
        if not np.all(np.isfinite(x)):
            raise ValueError("seed must be finite")
        result["finite_input"] = True
        digest = hashlib.sha256()
        digest.update(np.asarray(x.shape, dtype="<i8").tobytes())
        digest.update(np.asarray(x, dtype="<f8").tobytes())
        result["seed_sha256"] = digest.hexdigest()
        xl, xu = np.asarray(built["x_lower"], dtype=float), np.asarray(built["x_upper"], dtype=float)
        gl, gu = np.asarray(built["g_lower"], dtype=float), np.asarray(built["g_upper"], dtype=float)
        if xl.shape != (nx,) or xu.shape != (nx,) or gl.shape != (ng,) or gu.shape != (ng,):
            raise ValueError("bound vector shapes must match NLP dimensions")
        if (np.any(np.isnan(xl)) or np.any(np.isnan(xu)) or np.any(np.isnan(gl)) or np.any(np.isnan(gu))
                or np.any(xl > xu) or np.any(gl > gu)
                or np.any(np.isposinf(xl)) or np.any(np.isneginf(xu))
                or np.any(np.isposinf(gl)) or np.any(np.isneginf(gu))):
            raise ValueError("bounds must be non-NaN, ordered, and unbounded only outward")

        # Local function names are unique; this does not call an NLP solver.
        suffix = str(id(result))
        fval, gval = ca.Function("seed_diag_f_" + suffix, [xexpr], [fexpr])(x), ca.Function("seed_diag_g_" + suffix, [xexpr], [gexpr])(x)
        fvalue = float(fval)
        gv = np.asarray(gval, dtype=float).reshape(-1)
        result["objective_nonfinite"] = not math.isfinite(fvalue)
        result["objective_value"] = _json_number(fvalue)
        badg = np.flatnonzero(~np.isfinite(gv))
        result["nonfinite_constraint_rows"] = badg.astype(int).tolist()
        result["constraint_row_count"] = ng
        finiteg = np.isfinite(gv)
        lower_v = np.where(np.isfinite(gl), np.maximum(gl - gv, 0.), 0.)
        upper_v = np.where(np.isfinite(gu), np.maximum(gv - gu, 0.), 0.)
        violation = np.maximum(lower_v, upper_v)
        violation[~finiteg] = np.inf
        result["maximum_constraint_violation"] = _json_number(np.max(violation)) if ng else 0.0
        bxlo, bxhi = np.maximum(xl - x, 0.), np.maximum(x - xu, 0.)
        bviol = np.maximum(bxlo, bxhi)
        result["maximum_bound_violation"] = _json_number(np.max(bviol)) if nx else 0.0

        def rows(values, lows, highs, lo_v, hi_v, *, indices=None):
            candidates = np.flatnonzero(values > 0.) if indices is None else np.asarray(indices, dtype=int)
            candidates = candidates[values[candidates] > 0.]
            ordered = sorted(candidates.tolist(), key=lambda i: (-float(values[i]), int(i)))
            out = []
            for i in ordered[:int(maximum_reported_rows)]:
                use_lower = lo_v[i] >= hi_v[i]
                out.append({"index": int(i), "side": "lower" if use_lower else "upper",
                            "value": _json_number(x[i] if indices is not None else gv[i]),
                            "lower": _json_number(lows[i]), "upper": _json_number(highs[i]),
                            "lower_unbounded": bool(np.isneginf(lows[i])),
                            "upper_unbounded": bool(np.isposinf(highs[i])),
                            "violation": _json_number(values[i])})
            return ordered, out

        bindices, brows = rows(bviol, xl, xu, bxlo, bxhi, indices=np.arange(nx))
        result["bound_row_count"] = len(bindices)
        result["bound_rows_reported"] = len(brows)
        result["bound_rows_truncated"] = len(bindices) > len(brows)
        result["worst_bound_rows"] = brows
        result["coordinate_failure_rows"] = bindices
        cindices, crows = rows(violation, gl, gu, lower_v, upper_v)
        result["constraint_rows_reported"] = len(crows)
        result["constraint_rows_truncated"] = len(cindices) > len(crows)
        result["worst_constraint_rows"] = crows

        family_keys = (
            "node_acceleration_lower_indices", "node_acceleration_upper_indices",
            "midpoint_acceleration_lower_indices", "midpoint_acceleration_upper_indices",
            "bernstein_middle_indices", "off_site_acceleration_lower_indices",
            "off_site_acceleration_upper_indices", "node_domain_speed_indices",
            "midpoint_domain_speed_indices", "off_site_domain_speed_indices",
            "off_site_corridor_indices", "off_site_speed_cap_indices",
        )
        named = set()
        for key in family_keys:
            if key not in built:
                continue
            raw_indices = built[key]
            values = np.asarray(raw_indices)
            if values.ndim != 1 or values.dtype.kind not in "iu":
                raise ValueError(f"{key} must be a one-dimensional integer index array")
            ids = values.astype(np.int64, copy=False)
            if np.any(ids < 0) or np.any(ids >= ng):
                raise ValueError(f"{key} contains out-of-range constraint indices")
            named.update(ids.tolist())
            result["constraint_family_maxima"][key[:-8]] = _json_number(np.max(violation[ids])) if ids.size else 0.0
        unclassified = np.array([i for i in range(ng) if i not in named], dtype=int)
        result["unclassified_constraint_maximum"] = _json_number(np.max(violation[unclassified])) if unclassified.size else 0.0
        result["evaluation_complete"] = not result["objective_nonfinite"] and not badg.size
        if not result["evaluation_complete"]:
            result["error"] = "non-finite objective or constraint evaluation"
    except Exception as exc:
        result["evaluation_complete"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
    result["evaluation_wall_seconds"] = float(time.monotonic() - started)
    return result
