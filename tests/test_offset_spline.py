"""Tests for the periodic and open offset spline basis."""

import subprocess
import sys
import tempfile
from pathlib import Path
import unittest

import numpy as np

import simplified_vehicle_models
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


try:
    from scipy.interpolate import CubicSpline  # noqa: F401
except ImportError:
    _HAS_SCIPY = False
else:
    _HAS_SCIPY = True


@unittest.skipUnless(_HAS_SCIPY, "SciPy is required to build offset splines")
class OffsetSplineTest(unittest.TestCase):
    def test_basis_partitions_unity_and_derivatives_match_finite_differences(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            spline = OffsetSpline(track, control_spacing=10.0)
            positions = np.linspace(0.0, track.lap_length, 37)
            values, first, second = spline.basis(positions)
            np.testing.assert_allclose(values.sum(axis=1), 1.0, atol=1e-12)
            # The brief compared the (n, control_count) derivative matrices
            # against evaluate()'s (n,) result for one coefficient vector, which
            # cannot broadcast. Differentiate the basis matrices directly.
            step = 1e-5
            plus = spline.basis(positions + step)
            minus = spline.basis(positions - step)
            np.testing.assert_allclose(first, (plus[0] - minus[0]) / (2 * step), atol=1e-7)
            np.testing.assert_allclose(second, (plus[1] - minus[1]) / (2 * step), atol=1e-5)

    def test_periodic_spline_has_equal_seam_values_and_derivatives(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.sin(np.linspace(0.0, 2 * np.pi, spline.control_count))
            start = spline.evaluate(coefficients, np.array([0.0]))
            end = spline.evaluate(coefficients, np.array([track.lap_length]))
            np.testing.assert_allclose(start[0], end[0], atol=1e-10)
            np.testing.assert_allclose(start[1], end[1], atol=1e-10)
            np.testing.assert_allclose(start[2], end[2], atol=1e-10)

    def test_open_spline_retains_free_endpoint_values(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            object.__setattr__(track, 'is_closed', False)
            spline = OffsetSpline(track, control_spacing=10.0)
            n, _, _ = spline.evaluate(np.linspace(-1.0, 1.0, spline.control_count),
                                      np.array([0.0, track.lap_length]))
            self.assertNotAlmostEqual(float(n[0]), float(n[1]))

    def test_open_natural_spline_has_zero_endpoint_second_derivative(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            object.__setattr__(track, 'is_closed', False)
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.linspace(-1.0, 1.0, spline.control_count)
            _, _, second = spline.evaluate(coefficients,
                                           np.array([0.0, track.lap_length]))
            np.testing.assert_allclose(second, 0.0, atol=1e-10)

    def test_scalar_distance_returns_one_row_arrays(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.zeros(spline.control_count)
            for matrix in spline.basis(0.0):
                self.assertEqual(matrix.shape, (1, spline.control_count))
            for values in spline.evaluate(coefficients, 0.0):
                self.assertEqual(values.shape, (1,))


class OffsetSplineImportTest(unittest.TestCase):
    def test_module_imports_without_scipy(self):
        source_root = Path(simplified_vehicle_models.__file__).resolve().parent.parent
        script = (
            "import sys\n"
            f"sys.path.insert(0, {str(source_root)!r})\n"
            "class _BlockSciPy:\n"
            "    def find_spec(self, fullname, path=None, target=None):\n"
            "        if fullname == 'scipy' or fullname.startswith('scipy.'):\n"
            "            raise ImportError('SciPy is blocked for this test')\n"
            "        return None\n"
            "sys.meta_path.insert(0, _BlockSciPy())\n"
            "from simplified_vehicle_models.tracks.offset_spline import OffsetSpline\n"
            "assert OffsetSpline is not None\n"
            "assert not any(name == 'scipy' or name.startswith('scipy.')\n"
            "               for name in sys.modules)\n"
        )
        completed = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
