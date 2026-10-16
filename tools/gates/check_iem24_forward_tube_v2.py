#!/usr/bin/env python3
"""NON-BINDING tube diagnostic v2 (protocol: docs/validation/iem24-murphy/
seven_dof_track_pair/tube_v2_protocol.md, frozen before any fourth level).

Inputs are completed attempts of one lap ordered coarse, medium, fine (and
optionally a held-out, finer fourth attempt ``g``), each holding
``dense_polynomials.npz``. This tool is *diagnostic only*: it never edits any
``summary.json``, never touches ``usable`` or the acceptance gate, and does
not import or modify ``tools/gates/check_iem24_forward_tube.py`` (v1). It writes only
``tube_v2_certificate.json`` beside the fine attempt.

* ``T = 10 * max(d_cm, d_mf)`` componentwise, from c, m, f only. No ratio, no
  cap, no convergence assumption. ``T`` is a numerical diagnostic scale and is
  **not** a true ODE error bound.
* Test 4a (numerical domain margin): ``certify_planar_dense_domain`` on the fine
  run's dense bounds widened by ``2T``. This is *not* a convergence test.
* Test 4b (empirical convergence): raw uncapped ratios, strict shrink, and,
  with ``g``, held-out containment ``d_fg <= T``. Reported separately; there is
  deliberately no combined "passed" field.
* All sups are *sampled* sups (9 points per interval of the finer run of each
  pair); they are lower bounds on the true piecewise-polynomial sup. An exact
  union-knot sup is NOT implemented.
* Event windows come from the coarse level only: pedal crossing 0 and +-1, a
  wheel speed crossing 0, steering crossing +-steering limit; half-width
  ``3 * maximum_step(coarse)``. Preview-knot / feedforward-kink
  non-smoothness is deliberately NOT windowed (protocol revision 2), so the
  out-of-window shrink test may fail because of it; zero out-of-window samples
  gives null (not established), never pass.

Fails closed (raises, no file written) on missing/mismatched inputs and on any
non-finite value.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

PROTOCOL = "tube_v2_protocol.md"
VERSION = "tube_v2"
SAMPLES_PER_INTERVAL = 9
TUBE_FACTOR = 10.
DOMAIN_WIDENING = 2.
WINDOW_STEPS = 3.
CROSSING_TOLERANCE = 1.e-9
NAMES = ["s", "n", "heading", "v_x", "v_y", "yaw_rate", "w_fl", "w_fr", "w_rl", "w_rr",
         "soc", "steering", "pedal"]
WHEELS = (6, 7, 8, 9)
STEERING, PEDAL = 11, 12
SAME_SETTINGS = ("track", "planner", "spacing", "maximum_speed", "lateral_acceleration",
                 "pedal_travel_time", "method", "warm_start")
ACCEPTANCE_CLAIM = "none; diagnostic only; requires reviewed standard"


def load(attempt: Path):
    """Return (dense data dict, summary) for a completed attempt; fail closed."""
    attempt = Path(attempt)
    summary = json.loads((attempt / "summary.json").read_text())
    if summary.get("lap_completed") is not True or "dense_polynomials_file" not in summary:
        raise ValueError(f"{attempt} lacks a completed lap with dense polynomials")
    raw = np.load(attempt / summary["dense_polynomials_file"])
    data = {key: np.asarray(raw[key]) for key in ("times", "t_end", "step", "coefficients", "bounds")}
    coefficients = data["coefficients"]
    if (coefficients.ndim != 3 or coefficients.shape[1] != 6 or coefficients.shape[2] != len(NAMES)
            or data["times"].size != coefficients.shape[0] + 1 or data["times"].size < 2):
        raise ValueError(f"{attempt}: dense polynomials have an unexpected shape")
    for key in ("times", "t_end", "step", "coefficients", "bounds"):
        if not np.all(np.isfinite(data[key])):
            raise ValueError(f"{attempt}: non-finite {key}")
    if not np.all(np.diff(data["times"]) > 0.) or not np.all(data["step"] > 0.):
        raise ValueError(f"{attempt}: times/steps must be strictly increasing/positive")
    count = coefficients.shape[0]
    if data["t_end"].shape != (count,) or data["step"].shape != (count,):
        raise ValueError(f"{attempt}: t_end/step shape inconsistent with the coefficients")
    # BDF dense intervals may be event-truncated, so ``step`` (the BDF step) need not equal the
    # interval length. The invariant of the stored representation is that every interval maps
    # into the scaled variable s = (t - t_end)/step with s_a < s_b <= 0.
    s_a = (data["times"][:-1] - data["t_end"]) / data["step"]
    s_b = (data["times"][1:] - data["t_end"]) / data["step"]
    if not np.all(s_a < s_b) or np.any(s_b > 1e-9):
        raise ValueError(f"{attempt}: t_end/step inconsistent with the interval times")
    if data["bounds"].shape != (count, len(NAMES), 2) or np.any(data["bounds"][:, :, 0] > data["bounds"][:, :, 1]):
        raise ValueError(f"{attempt}: dense bounds shape/order inconsistent with the coefficients")
    return data, summary


def evaluate(data, times):
    """Evaluate the stored piecewise polynomials at ``times`` -> (n_states, len)."""
    edges = data["times"]
    index = np.clip(np.searchsorted(edges, times, side="right") - 1, 0, data["t_end"].size - 1)
    s = (times - data["t_end"][index]) / data["step"][index]
    powers = s[:, None] ** np.arange(6)[None, :]
    return np.einsum("tj,tjn->nt", powers, data["coefficients"][index])


def sample_times(data, end):
    edges = data["times"]
    edges = edges[edges <= end]
    fractions = np.linspace(0., 1., SAMPLES_PER_INTERVAL, endpoint=False)
    inside = (edges[:-1, None] + np.diff(edges)[:, None] * fractions[None, :]).ravel()
    return np.append(inside, end)


def working_sources_match(hashes):
    """Raise unless every recorded source file still hashes to its recorded value.

    Keys are file paths (resolved from the working directory); the check is made
    against the *current working files*, not only between the levels.
    """
    for path, recorded in hashes.items():
        file = Path(path)
        if not file.is_file():
            raise ValueError(f"recorded source file is missing: {path}")
        if hashlib.sha256(file.read_bytes()).hexdigest() != recorded:
            raise ValueError(f"source hash differs from the working file: {path}")


def check_levels(summaries, labels, source_check=working_sources_match):
    """Strict hash/settings/refinement checks (v1 semantics plus working-file and warm-start checks)."""
    first = summaries[0].get("source_hashes")
    if (not isinstance(first, dict) or not first
            or not all(isinstance(k, str) and isinstance(v, str) and v for k, v in first.items())):
        raise ValueError("levels need a non-empty source hashes dict")
    if any(s.get("source_hashes") != first for s in summaries):
        raise ValueError("levels must share identical source hashes")
    if source_check is not None:
        source_check(first)
    if summaries[0]["settings"].get("warm_start") in (None, ""):
        raise ValueError("levels need a recorded warm_start (identical across levels)")
    for key in SAME_SETTINGS:
        values = {json.dumps(s["settings"].get(key), sort_keys=True) for s in summaries}
        if len(values) != 1:
            raise ValueError(f"levels differ in setting {key!r}")
    steps = [float(s["settings"]["maximum_step"]) for s in summaries]
    rtols = [float(s["settings"]["rtol"]) for s in summaries]
    if not (np.all(np.isfinite(steps)) and np.all(np.isfinite(rtols))
            and all(a > b for a, b in zip(steps, steps[1:]))
            and all(a > b for a, b in zip(rtols, rtols[1:]))):
        raise ValueError(f"levels {labels} must be strictly refined in step and tolerance")
    return steps, rtols


def pair_samples(first, second):
    end = float(min(first["times"][-1], second["times"][-1]))
    finer = first if first["times"].size >= second["times"].size else second
    times = sample_times(finer, end)
    diff = np.abs(evaluate(first, times) - evaluate(second, times))
    if not np.all(np.isfinite(diff)):
        raise ValueError("non-finite difference between dense polynomials")
    return times, diff, end


def in_windows(times, windows):
    mask = np.zeros(times.shape, dtype=bool)
    for low, high in windows:
        mask |= (times >= low) & (times <= high)
    return mask


def stats(times, diff, mask):
    """sup/RMS/p99/argmax per component over the samples selected by ``mask``.

    Returns None when no sample is selected (never a fabricated number).
    """
    if not np.any(mask):
        return None
    sub, t = diff[:, mask], times[mask]
    out = {}
    for k, name in enumerate(NAMES):
        row = sub[k]
        out[name] = {"sup": float(row.max()), "rms": float(np.sqrt(np.mean(row ** 2))),
                     "p99": float(np.percentile(row, 99.)), "argmax_time_s": float(t[int(np.argmax(row))])}
    return out


def pair_report(times, diff, windows):
    inside = in_windows(times, windows)
    return {"sampled_sup_label": "sampled sup (lower bound on the true piecewise-polynomial sup)",
            "samples": int(times.size), "samples_in_window": int(inside.sum()),
            "all": stats(times, diff, np.ones(times.shape, dtype=bool)),
            "in_window": stats(times, diff, inside),
            "out_of_window": stats(times, diff, ~inside)}


def sups(report, part="all"):
    block = report[part]
    return None if block is None else np.array([block[n]["sup"] for n in NAMES])


def event_windows(coarse, step_coarse, steering_limit):
    """Event times and merged windows computed from the coarse level only."""
    times = sample_times(coarse, float(coarse["times"][-1]))
    values = evaluate(coarse, times)
    checks = [(PEDAL, 0.), (PEDAL, 1.), (PEDAL, -1.), (STEERING, float(steering_limit)),
              (STEERING, -float(steering_limit))] + [(k, 0.) for k in WHEELS]
    events = []
    for k, threshold in checks:
        shifted = values[k] - threshold
        near = np.abs(shifted) <= CROSSING_TOLERANCE
        for t in times[near]:
            events.append((float(t), NAMES[k], threshold))
        left, right = shifted[:-1], shifted[1:]
        cross = (left * right < 0.)
        for i in np.nonzero(cross)[0]:
            t = times[i] + (times[i + 1] - times[i]) * left[i] / (left[i] - right[i])
            events.append((float(t), NAMES[k], threshold))
    events.sort()
    half = WINDOW_STEPS * float(step_coarse)
    windows = []
    for t, _, _ in events:
        low, high = t - half, t + half
        if windows and low <= windows[-1][1]:
            windows[-1][1] = max(windows[-1][1], high)
        else:
            windows.append([low, high])
    return events, [tuple(w) for w in windows]


def _strict_shrink(later, earlier):
    """Per-component strict ``later < earlier`` or None if either is not established."""
    if later is None or earlier is None:
        return None
    return dict(zip(NAMES, [bool(a < b) for a, b in zip(later, earlier)]))


def build(attempts, steering_limit, certify, source_check=working_sources_match):
    """Compute the v2 diagnostic.

    ``attempts`` is ``[coarse, medium, fine]`` or ``[coarse, medium, fine, g]``.
    ``certify(bounds)`` must return a ``certify_planar_dense_domain`` result.
    """
    if len(attempts) not in (3, 4):
        raise ValueError("need coarse, medium, fine and optionally a held-out fourth attempt")
    loaded = [load(path) for path in attempts]
    data = [d for d, _ in loaded]
    summaries = [s for _, s in loaded]
    steps, rtols = check_levels(summaries, attempts, source_check)
    if not np.isfinite(float(steering_limit)):
        raise ValueError("steering limit must be finite")
    events, windows = event_windows(data[0], steps[0], steering_limit)

    reports, raw = {}, {}
    labels = {"coarse_medium": (0, 1), "medium_fine": (1, 2), "coarse_fine": (0, 2), "fine_g": (2, 3)}
    for label, (i, j) in labels.items():
        if j >= len(data):
            continue
        times, diff, end = pair_samples(data[i], data[j])
        reports[label] = pair_report(times, diff, windows)
        reports[label]["shared_time_span_s"] = end
        raw[label] = reports[label]

    d_cm, d_mf = sups(raw["coarse_medium"]), sups(raw["medium_fine"])
    tube = TUBE_FACTOR * np.maximum(d_cm, d_mf)
    if not np.all(np.isfinite(tube)):
        raise ValueError("non-finite tube scale")

    bounds = np.asarray(data[2]["bounds"], dtype=float)
    widened = np.stack([bounds[:, :, 0] - DOMAIN_WIDENING * tube[None, :],
                        bounds[:, :, 1] + DOMAIN_WIDENING * tube[None, :]], axis=-1)
    plain, wide = certify(bounds), certify(widened)
    def usable_margins(result):
        margins = result.get("margins")
        return (isinstance(margins, dict) and bool(margins)
                and all(isinstance(v, (int, float)) and np.isfinite(v) for v in margins.values()))
    domain_ok = bool(plain.get("certified") is True and wide.get("certified") is True
                     and usable_margins(plain) and usable_margins(wide))

    d_cm_out, d_mf_out = sups(raw["coarse_medium"], "out_of_window"), sups(raw["medium_fine"], "out_of_window")
    convergence = {
        "label": "empirical convergence of sampled dense-polynomial differences; "
                 "independent of the domain-margin test",
        "observed_ratio_mf_over_cm": {n: (float(a / b) if b > 0. else None)
                                      for n, a, b in zip(NAMES, d_mf, d_cm)},
        "shrink_cm_mf_all": _strict_shrink(d_mf, d_cm),
        "shrink_cm_mf_out_of_window": _strict_shrink(d_mf_out, d_cm_out),
        "degenerate_zero_components": [n for n, a in zip(NAMES, tube) if a == 0.],
        "held_out": None,
    }
    if len(data) == 4:
        d_fg = sups(raw["fine_g"])
        d_fg_out = sups(raw["fine_g"], "out_of_window")
        convergence["held_out"] = {
            "observed_ratio_fg_over_mf": {n: (float(a / b) if b > 0. else None)
                                          for n, a, b in zip(NAMES, d_fg, d_mf)},
            "shrink_mf_fg_all": _strict_shrink(d_fg, d_mf),
            "shrink_mf_fg_out_of_window": _strict_shrink(d_fg_out, d_mf_out),
            "d_fg_within_T": dict(zip(NAMES, [bool(a <= b) for a, b in zip(d_fg, tube)])),
            "d_fg_over_T": {n: (float(a / b) if b > 0. else None) for n, a, b in zip(NAMES, d_fg, tube)},
        }
    held = convergence["held_out"]
    def all_true(flags):
        return None if flags is None else bool(all(flags.values()))
    parts = {
        "shrink_cm_mf_all": all_true(convergence["shrink_cm_mf_all"]),
        "shrink_cm_mf_out_of_window": all_true(convergence["shrink_cm_mf_out_of_window"]),
        "shrink_mf_fg_all": None if held is None else all_true(held["shrink_mf_fg_all"]),
        "shrink_mf_fg_out_of_window": None if held is None else all_true(held["shrink_mf_fg_out_of_window"]),
        "d_fg_within_T": None if held is None else all_true(held["d_fg_within_T"]),
    }
    convergence["exploratory_4b_status"] = {
        "label": "exploratory aggregate of the raw per-component flags above (which are preserved); "
                 "not part of the frozen protocol's registered outputs; no acceptance meaning",
        "parts": parts,
        "all_hold": (None if any(v is None for v in parts.values()) else bool(all(parts.values()))),
        "null_means": "not established (e.g. no held-out level, or zero out-of-window samples)",
    }
    return {
        "version": VERSION, "protocol": PROTOCOL, "non_binding": True,
        "acceptance_claim": ACCEPTANCE_CLAIM,
        "levels": [str(a) for a in attempts],
        "maximum_step": steps, "rtol": rtols,
        "tube_T_diagnostic_scale": dict(zip(NAMES, map(float, tube))),
        "tube_T_definition": f"{TUBE_FACTOR:g} * max(d_cm, d_mf) componentwise from coarse, medium, fine "
                             "only; a numerical diagnostic scale, NOT a true ODE error bound",
        "events_found": len(events),
        "events": [{"time_s": t, "component": c, "threshold": th} for t, c, th in events],
        "event_windows_s": [list(w) for w in windows],
        "event_window_half_width_s": WINDOW_STEPS * steps[0],
        "pair_differences": reports,
        "domain_margin_test_widened_2T": {
            "label": "numerical domain-margin test only; NOT a convergence test",
            "holds": domain_ok,
            "margins_plain": plain.get("margins"), "margins_widened_by_twice_T": wide.get("margins"),
            "failed_plain": plain.get("failed"), "failed_widened": wide.get("failed"),
            "scope": wide.get("scope"),
        },
        "empirical_convergence_test": convergence,
        "sup_kind": "sampled; exact union-knot sup not computed",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("coarse", type=Path)
    parser.add_argument("medium", type=Path)
    parser.add_argument("fine", type=Path)
    parser.add_argument("held_out", type=Path, nargs="?", default=None)
    args = parser.parse_args(argv)
    sys.path.insert(0, ".")
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan
    from simplified_vehicle_models.solvers.planar_dense_certificate import certify_planar_dense_domain
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.tracks.profile import load_track
    from tools.run.run_iem24_forward_driver import FORMULATION
    settings = json.loads((args.fine / "summary.json").read_text())["settings"]
    vehicle = compile_formulation(str(FORMULATION)).model
    track = load_track(settings["track"])
    count = int(np.ceil(track.lap_length / settings["spacing"]))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live = settings["planner"] == "live"
    cap = settings.get("lateral_acceleration")
    if cap is None and not live:
        cap = 2.
    plan = build_speed_plan(distance, curvature, settings["maximum_speed"], cap, 1., 2.,
                            vehicle=vehicle if live else None, live_limit=live)
    attempts = [args.coarse, args.medium, args.fine] + ([args.held_out] if args.held_out else [])
    result = build(attempts, vehicle.steering_maximum_angle,
                   lambda bounds: certify_planar_dense_domain(vehicle, plan, bounds, True))
    destination = args.fine / "tube_v2_certificate.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(destination, "domain_margin_test_widened_2T:", result["domain_margin_test_widened_2T"]["holds"],
          "(diagnostic only; no acceptance claim)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
