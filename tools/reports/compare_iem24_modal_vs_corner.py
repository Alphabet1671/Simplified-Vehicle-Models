"""Compare the IEM24 modal roll/heave setup with the independent-corner + rear ARB.

This is a suspension-response comparison under matched imposed wheel motion, not
an integrated lap or measured-vehicle validation. Writes CSV, JSON, PNG and report.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from simplified_vehicle_models.compiler.compile import compile_formulation  # noqa: E402
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
CORNER = ROOT / "models/suspensions/iem24_murphy_real_suspension.xml"
MODAL = ROOT / "models/suspensions/iem24_axle_roll_heave.xml"


def _compile_with(assembly: Path):
    vehicle_source = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
    formulation_source = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
    suspension_ref = '../suspensions/iem24_murphy_real_suspension.xml'
    vehicle_text = vehicle_source.read_text().replace(suspension_ref, assembly.resolve().as_posix())
    vehicle_text = re.sub(r'(ref|path)="\.\./', rf'\1="{(ROOT / "models").as_posix()}/', vehicle_text)
    formulation_text = formulation_source.read_text().replace(
        "../vehicles/iem24_murphy_real_planar.xml", "vehicle.xml")
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory).resolve()
        (directory / "vehicle.xml").write_text(vehicle_text)
        (directory / "formulation.xml").write_text(formulation_text)
        return compile_formulation(directory / "formulation.xml").model.suspension


def _simulations():
    corner = compile_formulation(FORMULATION).model.suspension
    modal = _compile_with(MODAL)
    return corner, modal


def _wheel_tests(corner, modal):
    # Wheel bump and roll inputs, each on front/rear axle, with a matched
    # sinusoidal displacement amplitude and frequency. The 0.1 m/s amplitude
    # velocity is matched to 20 mm at 0.796 Hz (omega=5 rad/s).
    x = np.linspace(0, 2 * np.pi, 1001)
    z_amp = 0.020
    omega = 5.0
    cases = {
        "Front heave": (np.array([1, 1, 0, 0]) * 0.5, "heave-front"),
        "Front roll": (np.array([1, -1, 0, 0]) * 0.5, "roll-front"),
        "Rear heave": (np.array([0, 0, 1, 1]) * 0.5, "heave-rear"),
        "Rear roll": (np.array([0, 0, 1, -1]) * 0.5, "roll-rear"),
    }
    rows, traces = [], {}
    for label, (pattern, _) in cases.items():
        q = z_amp * np.sin(x)
        qdot = z_amp * omega * np.cos(x)
        z = np.outer(q, pattern)
        zdot = np.outer(qdot, pattern)
        traces[label] = {}
        for name, suspension in (("Corner + rear ARB", corner), ("Modal heave/roll", modal)):
            f = np.vstack([suspension.network.forces(zi, vi).total for zi, vi in zip(z, zdot)])
            baseline = suspension.network.forces(np.zeros(4), np.zeros(4)).total
            traces[label][name] = f - baseline
        # Difference metrics over the two wheel forces on the excited axle.
        indices = (0, 1) if "Front" in label else (2, 3)
        a = traces[label]["Corner + rear ARB"][:, indices]
        b = traces[label]["Modal heave/roll"][:, indices]
        diff = b - a
        rows.append({
            "input": label,
            "peak_corner_force_N": float(np.max(np.abs(a))),
            "peak_modal_force_N": float(np.max(np.abs(b))),
            "difference_peak_N": float(np.max(np.abs(diff))),
            "difference_rms_N": float(np.sqrt(np.mean(diff**2))),
            "relative_rms_percent": float(100*np.sqrt(np.mean(diff**2))/np.sqrt(np.mean(a**2))),
            "corner_peak_dynamic_N": float(np.max(np.abs(a - a.mean(axis=0)))),
            "modal_peak_dynamic_N": float(np.max(np.abs(b - b.mean(axis=0)))),
        })
    return rows, traces, x


def _linear_metrics(corner, modal):
    z = np.zeros(4)
    zdot = np.zeros(4)
    out = {}
    for label, suspension in (("corner", corner), ("modal", modal)):
        # Stiffness and local damping Jacobians by central differences.
        h = 1e-6
        K = np.column_stack([(suspension.network.forces(z + np.eye(4)[j]*h, zdot).total -
                              suspension.network.forces(z - np.eye(4)[j]*h, zdot).total)/(2*h)
                             for j in range(4)])
        D = np.column_stack([(suspension.network.forces(z, zdot + np.eye(4)[j]*h).total -
                              suspension.network.forces(z, zdot - np.eye(4)[j]*h).total)/(2*h)
                             for j in range(4)])
        out[label] = {"stiffness_matrix_N_per_m": K.tolist(), "damping_matrix_Ns_per_m": D.tolist(),
                      "stiffness_eigenvalues_N_per_m": np.linalg.eigvalsh(K).tolist(),
                      "damping_eigenvalues_Ns_per_m": np.linalg.eigvalsh(D).tolist()}
    return out


def render(output, rows, traces, x):
    colors = {"Corner + rear ARB": "#1f4e79", "Modal heave/roll": "#c00000"}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for ax, label in zip(axes.flat, traces):
        for name, forces in traces[label].items():
            # Plot the left-side wheel force; the right-side trace differs by sign
            # for roll and matches for heave.
            index = 0 if "Front" in label else 2
            ax.plot(x, forces[:, index], color=colors[name], label=name)
        ax.set_title(label)
        ax.set_ylabel("Left-wheel force (N)")
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Input phase (rad)")
    fig.suptitle("IEM24 suspension response under matched imposed wheel motion")
    fig.tight_layout()
    fig.savefig(output / "force_response.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.7))
    labels = [r["input"] for r in rows]
    vals = [r["relative_rms_percent"] for r in rows]
    bars = ax.bar(labels, vals, color="#4472c4")
    ax.bar_label(bars, fmt="%.1f%%")
    ax.set_ylabel("Modal-vs-corner force difference RMS (%)")
    ax.set_title("Difference under identical 20 mm, 5 rad/s wheel-motion inputs")
    ax.grid(axis="y", alpha=.25)
    fig.tight_layout()
    fig.savefig(output / "force_difference.png", dpi=160)
    plt.close(fig)


def write_report(output, rows, linear, corner, modal):
    lines = [
        "# IEM24 modal roll/heave vs independent-corner + rear ARB suspension comparison",
        "",
        "**Exploratory model-to-model suspension response report. This is not a simulated lap, measured-vehicle validation, or acceptance result.** It intentionally does not follow the MIS lap-report format because no lap or telemetry comparison was requested.",
        "",
        "## Configuration",
        "",
        "- Baseline model A: independent-corner suspension with the existing rear ARB.",
        "- Baseline model B: axle modal heave/roll suspension, separate heave/roll damper tables and click settings from real car data.",
        "- Spring rates: 450 lbf/in heave at both axles; 350 lbf/in front roll; 350 lbf/in rear roll plus 120 lbf/in equivalent rear ARB contribution.",
        "- Damper clicks in modal model: heave 6 front/rear; roll 5 front and 8 rear. Corner baseline retains its own configured click values.",
        "- Each test applies one 20 mm peak sinusoidal jounce mode at angular frequency 5 rad/s (velocity peak 0.1 m/s); both networks receive identical wheel displacement and velocity vectors.",
        "- Plotted traces and peak-force metrics show dynamic force relative to each model's zero-input baseline; difference metrics use both wheels on the excited axle.",
        "",
        "## Method",
        "",
        "Front/rear heave inputs move both axle wheels equally; roll inputs move the two wheels equally and oppositely. Matched corner motions avoid interpreting the model's distinct internal coordinate conventions as different physical inputs. The roll/heave modes are examined separately; body dynamics, tires, road inputs, and vehicle trajectories are not simulated.",
        "",
        "## Force comparison",
        "",
        "| Input | Corner peak force (N) | Modal peak force (N) | Difference peak (N) | Difference RMS (N) | Difference RMS (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for r in rows:
        lines.append(f"| {r['input']} | {r['peak_corner_force_N']:.1f} | {r['peak_modal_force_N']:.1f} | {r['difference_peak_N']:.1f} | {r['difference_rms_N']:.1f} | {r['relative_rms_percent']:.1f}% |")
    lines += ["", "![Wheel-force traces for matched heave and roll inputs](force_response.png)", "", "![RMS wheel-force difference normalized by the corner-model RMS](force_difference.png)", "", "## Linearized comparison", "", "Central-difference force Jacobians at zero jounce and zero velocity (including damper branch slope) are recorded in `comparison_summary.json`.", "", "| Model | Stiffness eigenvalues (N/m) | Damping eigenvalues (N·s/m) |", "|---|---|---|"]
    for key, name in (("corner", "Corner + rear ARB"), ("modal", "Modal heave/roll")):
        k = ", ".join(f"{v:.0f}" for v in linear[key]["stiffness_eigenvalues_N_per_m"])
        d = ", ".join(f"{v:.0f}" for v in linear[key]["damping_eigenvalues_Ns_per_m"])
        lines.append(f"| {name} | {k} | {d} |")
    lines += ["", "## Reading the comparison", "", "The table reports direct force differences under the same prescribed wheel motion; it does not show which model better matches the car. Spring-rate equivalence does not make the models identical: the modal layout uses modal springs and dedicated mode damper tables/settings, while the corner model uses corner spring curves, corner dampers and the independent ARB force path. Differences therefore reflect both the force-law structure and the distinct damper data/settings.", "", "## Caveats", "", "- The independent-corner model's local spring behavior is the committed corner LUT; modal spring behavior uses scalar modal rates.", "- The modal damper table measurements are from real car data, but modal shaft motion is mapped using declared heave/roll motion ratios and the modal Jacobian.", "- Corner baseline uses configured corner damper settings; it is not retuned here to match the modal click settings.", "- This is a deterministic imposed-motion comparison only. No unsprung/body integration, road excitation, lap, or measured response data is included.", "- These are the values we did not measure: this report makes no claim that either response is validated against measured suspension forces or damper dyno output.", "", "## Reproduce", "", "```bash", "PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/reports/compare_iem24_modal_vs_corner.py --output-dir docs/validation/iem24-murphy/modal_vs_corner_arb", "```", "", "Generating tool: `tools/reports/compare_iem24_modal_vs_corner.py`. Modal damper tables are generated from real car data by `tools/model_data/import_real_mode_dampers.py`."]
    (output / "comparison_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "docs/validation/iem24-murphy/modal_vs_corner_arb")
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    corner, modal = _simulations()
    rows, traces, x = _wheel_tests(corner, modal)
    linear = _linear_metrics(corner, modal)
    _write_csv(args.output_dir / "force_comparison.csv", rows)
    (args.output_dir / "comparison_summary.json").write_text(json.dumps({"status":"exploratory suspension response only", "input_peak_jounce_m":0.02, "input_angular_frequency_rad_s":5.0, "tests":rows, "linearization":linear}, indent=2)+"\n", encoding="utf-8")
    render(args.output_dir, rows, traces, x)
    write_report(args.output_dir, rows, linear, corner, modal)
    print(args.output_dir)


def _write_csv(path, rows):
    import csv
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


if __name__ == "__main__":
    main()
