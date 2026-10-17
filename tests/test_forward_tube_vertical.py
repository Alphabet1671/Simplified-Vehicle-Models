"""Vertical 14DoF refinement tube on synthetic dense-polynomial attempts."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.gates.check_iem24_forward_tube_vertical import build

STATES = 38
INTERVALS = 10


def make_attempt(root, step, rtol, offset, model="14dof-simple", hashes=None, lap=True):
    """A constant-in-time dense solution whose value is ``offset`` above a base."""
    path = Path(root)
    path.mkdir(parents=True, exist_ok=True)
    times = np.linspace(0., 1., INTERVALS + 1)
    coefficients = np.zeros((INTERVALS, 6, STATES))
    coefficients[:, 0, :] = 1. + offset
    bounds = np.zeros((INTERVALS, STATES, 2))
    bounds[:, :, 0] = 1. + offset - 1e-3
    bounds[:, :, 1] = 1. + offset + 1e-3
    np.savez(path / "dense_polynomials.npz", times=times, t_end=times[1:],
             step=np.diff(times), coefficients=coefficients, bounds=bounds)
    summary = {"model": model, "lap_completed": lap, "dense_polynomials_file": "dense_polynomials.npz",
               "source_hashes": hashes or {"a": "1"},
               "settings": {"maximum_step": step, "rtol": rtol}}
    (path / "summary.json").write_text(json.dumps(summary))
    return path


def accept(bounds):
    return {"certified": True, "checks": {}, "margins": {}}


class VerticalTubeTests(unittest.TestCase):
    def levels(self, root, offsets=(8e-4, 2e-4, 5e-5), **fine):
        steps, rtols = (.01, .005, .0025), (1e-5, 5e-6, 2.5e-6)
        paths = []
        for index, name in enumerate(("coarse", "medium", "fine")):
            extra = fine if name == "fine" else {}
            paths.append(make_attempt(Path(root) / name, steps[index], rtols[index], offsets[index], **extra))
        return paths

    def test_geometrically_converging_levels_pass(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(*self.levels(root), accept)
        self.assertTrue(result["passed"])
        self.assertTrue(result["all_components_shrink"])
        self.assertEqual(len(result["tube"]), STATES)
        self.assertIn("not a proof", result["scope"])

    def test_non_shrinking_difference_fails(self):
        with tempfile.TemporaryDirectory() as root:  # d_cm = 1e-4 but d_mf = 7e-4
            result = build(*self.levels(root, offsets=(8e-4, 7e-4, 0.)), accept)
        self.assertFalse(result["all_components_shrink"])
        self.assertFalse(result["passed"])

    def test_failed_evaluation_domain_certificate_fails(self):
        with tempfile.TemporaryDirectory() as root:
            result = build(*self.levels(root), lambda bounds: {"certified": False})
        self.assertFalse(result["passed"])

    def test_domain_is_checked_on_the_enclosure_widened_by_twice_the_tube(self):
        seen = []
        with tempfile.TemporaryDirectory() as root:
            result = build(*self.levels(root), lambda bounds: seen.append(np.array(bounds)) or accept(bounds))
        self.assertEqual(len(seen), 2)
        tube = np.array(list(result["tube"].values()))
        growth = seen[0][:, :, 0] - seen[1][:, :, 0]
        np.testing.assert_allclose(growth, np.broadcast_to(2. * tube, growth.shape))

    def test_non_14dof_level_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(ValueError):
                build(*self.levels(root, model="planar-7dof"), accept)

    def test_incomplete_lap_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(ValueError):
                build(*self.levels(root, lap=False), accept)

    def test_different_source_hashes_are_refused(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(ValueError):
                build(*self.levels(root, hashes={"a": "2"}), accept)

    def test_levels_not_strictly_refined_are_refused(self):
        with tempfile.TemporaryDirectory() as root:
            coarse = make_attempt(Path(root) / "c", .01, 1e-5, 8e-4)
            medium = make_attempt(Path(root) / "m", .01, 5e-6, 2e-4)
            fine = make_attempt(Path(root) / "f", .0025, 2.5e-6, 5e-5)
            with self.assertRaises(ValueError):
                build(coarse, medium, fine, accept)


if __name__ == "__main__":
    unittest.main()
