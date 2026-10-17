import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.suspension.damper_table import (
    DamperSetting,
    DamperTable,
    DamperValve,
    damper_force,
    load_damper_npz,
)

V_C = np.array([0.0, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0])
V_R = -V_C[::-1]  # ends at zero
CLICKS = {"lsc": 4, "hsc": 3, "lsr": 4, "hsr": 3}


def _valves():
    return (
        DamperValve("lsc", "compression", "low", 4),
        DamperValve("hsc", "compression", "high", 3),
        DamperValve("lsr", "rebound", "low", 4),
        DamperValve("hsr", "rebound", "high", 3),
    )


def _compression_force():
    # F[v, lsc, hsc]; zero at the zero knot for every click combination.
    f = np.zeros((V_C.size, 4, 3))
    for i in range(1, V_C.size):
        for a in range(4):
            for b in range(3):
                f[i, a, b] = 100.0 * (1 + a) * np.sqrt(V_C[i]) + 50.0 * (1 + b) * V_C[i] ** 1.5
    return f


def _rebound_force():
    f = np.zeros((V_R.size, 4, 3))
    for i in range(V_R.size - 1):
        s = -V_R[i]
        for a in range(4):
            for b in range(3):
                f[i, a, b] = -(150.0 * (1 + a) * np.sqrt(s) + 80.0 * (1 + b) * s ** 1.5)
    return f


def _table(**overrides):
    kwargs = dict(
        velocity_axis_compression=V_C,
        velocity_axis_rebound=V_R,
        valves=_valves(),
        compression_force=_compression_force(),
        rebound_force=_rebound_force(),
        metadata={"source_sha256": "abc", "generator_version": "t", "corrections": "{}"},
    )
    kwargs.update(overrides)
    return DamperTable(**kwargs)


def _setting(lsc=1, hsc=1, lsr=1, hsr=1):
    return DamperSetting({"lsc": lsc, "hsc": hsc, "lsr": lsr, "hsr": hsr})


class DamperTableTests(unittest.TestCase):
    def test_force_at_knot_with_integer_clicks_equals_table(self):
        table = _table()
        out = damper_force(table, _setting(2, 1, 3, 2), V_C[3])
        self.assertAlmostEqual(float(out.force), table.compression_force[3, 2, 1], places=9)
        out = damper_force(table, _setting(2, 1, 3, 2), V_R[2])
        self.assertAlmostEqual(float(out.force), table.rebound_force[2, 3, 2], places=9)
        self.assertFalse(bool(out.out_of_range))

    def test_rebound_force_is_negative_and_not_double_flipped(self):
        out = damper_force(_table(), _setting(), -0.5)
        self.assertLess(float(out.force), 0.0)
        self.assertGreater(float(out.slope), 0.0)

    def test_zero_velocity_zero_force_for_all_clicks(self):
        table = _table()
        for a in range(4):
            for b in range(3):
                out = damper_force(table, _setting(a, b, a, b), 0.0)
                self.assertEqual(float(out.force), 0.0)

    def test_compression_valve_change_leaves_rebound_unchanged(self):
        table = _table()
        v = np.array([-1.5, -0.3, -0.07])
        base = damper_force(table, _setting(0, 0, 2, 1), v).force
        for lsc, hsc in ((3, 2), (1, 0), (2.5, 1.5)):
            other = damper_force(table, _setting(lsc, hsc, 2, 1), v).force
            np.testing.assert_array_equal(base, other)

    def test_rebound_valve_change_leaves_compression_unchanged(self):
        table = _table()
        v = np.array([1.5, 0.3, 0.07])
        base = damper_force(table, _setting(2, 1, 0, 0), v).force
        other = damper_force(table, _setting(2, 1, 3, 2), v).force
        np.testing.assert_array_equal(base, other)

    def test_fractional_click_blend_is_mean_of_neighbours(self):
        table = _table()
        # At a knot the velocity interpolation is exact, so the blend is exact.
        for v in (V_C[2], V_C[5], V_R[1]):
            lo = damper_force(table, _setting(2, 1, 2, 1), v).force
            hi = damper_force(table, _setting(3, 1, 3, 1), v).force
            mid = damper_force(table, _setting(2.5, 1, 2.5, 1), v).force
            self.assertAlmostEqual(float(mid), 0.5 * float(lo + hi), places=9)
        lo = damper_force(table, _setting(1, 0, 1, 0), V_C[4]).force
        hi = damper_force(table, _setting(1, 1, 1, 1), V_C[4]).force
        mid = damper_force(table, _setting(1, 0.5, 1, 0.5), V_C[4]).force
        self.assertAlmostEqual(float(mid), 0.5 * float(lo + hi), places=9)

    def test_slope_matches_central_difference(self):
        table = _table()
        setting = _setting(1.5, 2, 2.25, 0.5)
        v = np.array([0.02, 0.07, 0.18, 0.4, 0.8, 1.6, -0.02, -0.18, -0.9, -1.7])
        eps = 1e-6
        fd = (damper_force(table, setting, v + eps).force
              - damper_force(table, setting, v - eps).force) / (2 * eps)
        np.testing.assert_allclose(damper_force(table, setting, v).slope, fd, rtol=1e-6)

    def test_out_of_range_flagged_and_c1_continuous(self):
        table = _table()
        setting = _setting(1, 1, 1, 1)
        for edge, sign in ((V_C[-1], 1.0), (V_R[0], -1.0)):
            eps = 1e-7
            inside = damper_force(table, setting, edge - sign * eps)
            outside = damper_force(table, setting, edge + sign * eps)
            self.assertFalse(bool(inside.out_of_range))
            self.assertTrue(bool(outside.out_of_range))
            self.assertAlmostEqual(float(inside.force), float(outside.force), delta=1e-4)
            at_edge = damper_force(table, setting, edge)
            self.assertFalse(bool(at_edge.out_of_range))
            self.assertAlmostEqual(float(at_edge.slope), float(outside.slope), delta=1e-9)
            far = damper_force(table, setting, edge + sign * 3.0)
            self.assertTrue(bool(far.out_of_range))
            expected = float(at_edge.force) + float(at_edge.slope) * sign * 3.0
            self.assertAlmostEqual(float(far.force), expected, delta=1e-6)
            self.assertAlmostEqual(float(far.slope), float(at_edge.slope), delta=1e-9)

    def test_array_velocity_flags(self):
        out = damper_force(_table(), _setting(), np.array([-5.0, -1.0, 0.0, 1.0, 5.0]))
        np.testing.assert_array_equal(out.out_of_range, [True, False, False, False, True])
        self.assertEqual(out.force.shape, (5,))
        self.assertEqual(out.slope.shape, (5,))

    def test_non_finite_velocity_rejected(self):
        with self.assertRaises(ValueError):
            damper_force(_table(), _setting(), float("nan"))

    def test_setting_validation(self):
        table = _table()
        _setting(1, 2, 3, 0).validate(table)
        with self.assertRaises(ValueError):
            _setting(1.5).validate(table)
        _setting(1.5).validate(table, allow_fractional=True)
        with self.assertRaises(ValueError):
            _setting(4).validate(table, allow_fractional=True)  # lsc has clicks 0..3
        with self.assertRaises(ValueError):
            _setting(-0.1).validate(table, allow_fractional=True)
        with self.assertRaises(ValueError):
            DamperSetting({"lsc": 1}).validate(table)
        with self.assertRaises(ValueError):
            DamperSetting({"lsc": 1, "hsc": 1, "lsr": 1, "hsr": 1, "x": 0}).validate(table)
        with self.assertRaises(ValueError):
            damper_force(table, _setting(9), 0.1)

    def test_valve_shape_and_name_mismatch(self):
        good = _valves()
        with self.assertRaises(ValueError):  # click count disagrees with array
            _table(valves=(DamperValve("lsc", "compression", "low", 5),) + good[1:])
        with self.assertRaises(ValueError):  # duplicate name
            _table(valves=(good[0], DamperValve("lsc", "compression", "high", 3)) + good[2:])
        with self.assertRaises(ValueError):  # array rank disagrees with valve count
            _table(compression_force=_compression_force()[:, :, 0])
        with self.assertRaises(ValueError):  # velocity axis length disagrees
            _table(compression_force=_compression_force()[:-1])
        with self.assertRaises(ValueError):
            DamperValve("x", "sideways", "low", 3)
        with self.assertRaises(ValueError):
            DamperValve("x", "compression", "mid", 3)
        with self.assertRaises(ValueError):
            DamperValve("x", "compression", "low", 0)

    def test_axis_and_zero_force_validation(self):
        with self.assertRaises(ValueError):
            _table(velocity_axis_compression=V_C + 0.01)  # does not start at 0
        with self.assertRaises(ValueError):
            _table(velocity_axis_rebound=V_R - 0.01)  # does not end at 0
        bad = _compression_force()
        bad[0, 1, 1] = 1.0
        with self.assertRaises(ValueError):
            _table(compression_force=bad)
        bad = _rebound_force()
        bad[-1, 0, 0] = -1.0
        with self.assertRaises(ValueError):
            _table(rebound_force=bad)
        with self.assertRaises(ValueError):  # rebound must be signed negative
            _table(rebound_force=-_rebound_force())

    def test_arrays_read_only(self):
        table = _table()
        for array in (table.velocity_axis_compression, table.velocity_axis_rebound,
                      table.compression_force, table.rebound_force):
            self.assertFalse(array.flags.writeable)
        with self.assertRaises(TypeError):
            table.metadata["x"] = "y"


class DamperNpzTests(unittest.TestCase):
    def _write(self, directory, **overrides):
        table = _table()
        payload = dict(
            schema=np.array("damper-table-v1"),
            velocity_compression=table.velocity_axis_compression,
            velocity_rebound=table.velocity_axis_rebound,
            valve_names=np.array([v.name for v in table.valves]),
            valve_halves=np.array([v.half for v in table.valves]),
            valve_bands=np.array([v.speed_band for v in table.valves]),
            valve_clicks=np.array([v.clicks for v in table.valves]),
            force_compression=table.compression_force,
            force_rebound=table.rebound_force,
            source_sha256=np.array("deadbeef"),
            generator_version=np.array("test-1"),
            corrections=np.array(json.dumps({"note": "none"})),
        )
        payload.update(overrides)
        path = Path(directory) / "damper.npz"
        np.savez(path, **payload)
        return path, hashlib.sha256(path.read_bytes()).hexdigest()

    def test_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, digest = self._write(tmp)
            loaded = load_damper_npz(path, digest)
        reference = _table()
        np.testing.assert_array_equal(loaded.compression_force, reference.compression_force)
        np.testing.assert_array_equal(loaded.rebound_force, reference.rebound_force)
        self.assertEqual(loaded.valves, reference.valves)
        self.assertEqual(loaded.metadata["source_sha256"], "deadbeef")
        self.assertEqual(loaded.metadata["generator_version"], "test-1")
        self.assertEqual(json.loads(loaded.metadata["corrections"]), {"note": "none"})
        self.assertFalse(loaded.compression_force.flags.writeable)
        self.assertFalse(loaded.velocity_axis_rebound.flags.writeable)

    def test_sha256_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, digest = self._write(tmp)
            with self.assertRaises(ValueError):
                load_damper_npz(path, "0" * 64)
            load_damper_npz(path, digest.upper())

    def test_bad_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, digest = self._write(tmp, schema=np.array("damper-table-v0"))
            with self.assertRaises(ValueError):
                load_damper_npz(path, digest)

    def test_missing_key(self):
        with tempfile.TemporaryDirectory() as tmp:
            table = _table()
            path = Path(tmp) / "damper.npz"
            np.savez(path, schema=np.array("damper-table-v1"),
                     velocity_compression=table.velocity_axis_compression)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaises(ValueError):
                load_damper_npz(path, digest)

    def test_valve_mismatch_in_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, digest = self._write(tmp, valve_clicks=np.array([4, 4, 4, 3]))
            with self.assertRaises(ValueError):
                load_damper_npz(path, digest)

    def test_pickled_object_arrays_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, digest = self._write(tmp, valve_names=np.array(["lsc", "hsc", "lsr", {"a": 1}], dtype=object))
            with self.assertRaises(ValueError):
                load_damper_npz(path, digest)


if __name__ == "__main__":
    unittest.main()
