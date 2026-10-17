"""Check the animation panels, window visibility and layout on exact synthetic motion."""
import tempfile
import unittest
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
from matplotlib.figure import Figure

from simplified_vehicle_models.results.run_record import RunRecorder, load_run_record
from tools.anim.layout import WINDOWS, window_grid, window_visibility
from tools.anim.panels import (TrackGeometry, build_attitude_panel, build_panels,
                               build_top_panel, build_track_panel)
from tools.anim.resample import FrameSource, MissingChannelError
from tools.anim.scene import placeholder_scene
from tools.anim.style import AnimationStyle

GEOMETRY = {"wheel_positions_body_m": [[0.844, -0.62], [0.844, 0.62], [-0.691, -0.62],
                                       [-0.691, 0.62]],
            "cg_height_m": 0.3, "wheelbase_m": 1.535, "track_width_m": 1.24}
STATES = ("s", "v_x", "omega_fl", "omega_fr", "omega_rl", "omega_rr")
CONTROLS = ("throttle", "brake-force")
BASE = ("speed", "body-slip-angle", "heading-world", "position-x", "position-y",
        "acceleration-longitudinal", "acceleration-lateral", "yaw-acceleration")
WHEEL = ("wheel-steering-angle", "tire-longitudinal-force", "tire-lateral-force",
         "tire-aligning-moment", "tire-vertical-force", "throttle", "brake-force",
         "brake-fraction", "steering")
DYNAMIC = ("roll", "pitch", "heave")
QUASI = ("roll-quasi-static", "pitch-quasi-static")
SLIP, STEER, ROLL, PITCH = 0.1, 0.2, 0.05, -0.03


def make_source(directory, channels, *, model="synthetic", quasi=False, steps=61,
                state_names=STATES, duration=0.6, **keywords):
    """A record: constant slip, steering, roll and pitch, heading 0.5 rad.

    ``state_names`` sets the state layout (first two are distance and speed, the rest
    are held at 60); ``duration`` is the record length in seconds.
    """
    recorder = RunRecorder(Path(directory) / f"{model}.npz", state_names=state_names,
                           control_names=CONTROLS, channel_names=channels, precision="float64")
    for index, t in enumerate(np.linspace(0.0, duration, steps)):
        v = 20.0
        values = {
            "speed": [v], "body-slip-angle": [SLIP], "heading-world": [0.5 + t],
            "position-x": [v * t * np.cos(0.5)], "position-y": [v * t * np.sin(0.5)],
            "acceleration-longitudinal": [2.0], "acceleration-lateral": [-4.0],
            "yaw-acceleration": [0.3], "wheel-steering-angle": [STEER, STEER, 0.0, 0.0],
            "tire-longitudinal-force": [100.0, 110.0, 120.0, 130.0],
            "tire-lateral-force": [400.0, 410.0, 420.0, 430.0],
            "tire-aligning-moment": [5.0, 5.0, 5.0, 5.0],
            "tire-vertical-force": [1000.0, 1100.0, 1200.0, 1300.0],
            "throttle": [0.5], "brake-force": [50.0], "brake-fraction": [0.1],
            "steering": [STEER], "roll": [ROLL], "pitch": [PITCH], "heave": [0.01],
            "roll-quasi-static": [0.01], "pitch-quasi-static": [-0.02],
            "roll-rate": [0.0], "pitch-rate": [0.0],
        }
        rest = len(state_names) - 2
        recorder.append(t, index, [v * t, v] + [60.0] * rest, [v] + [0.0] * (rest + 1),
                        [0.5, 50.0],
                        {name: values[name] for name in channels})
    record = load_run_record(recorder.close(
        {"model": {"model": model}, "vehicle_geometry": GEOMETRY}))
    return FrameSource(record, 120.0, allow_quasi_static=quasi, **keywords)


def track_geometry():
    s = np.linspace(0.0, 2 * np.pi, 200)
    return TrackGeometry(100.0 * np.sin(s), 100.0 * np.cos(s) - 100.0, 628.0, True)


def axes_for(visibility):
    style = AnimationStyle()
    figure = Figure(figsize=style.figure_size, dpi=style.dpi)
    layout = window_grid(visibility, style)
    return figure, {name: figure.add_axes(layout[name]) for name in layout.windows}, style


class VisibilityTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def visibility(self, channels, **options):
        quasi = options.get("allow_quasi_static", False)
        source = make_source(self.directory.name, channels, quasi=quasi)
        return window_visibility(source, **options)

    def test_dynamic_model_shows_all_four_windows(self):
        v = self.visibility(BASE + WHEEL + DYNAMIC)
        self.assertEqual(v.visible, WINDOWS)
        self.assertTrue(v.attitude_is_dynamic and v.tire_arrows and v.fz_bars)
        self.assertEqual(v.hidden, ())
        self.assertFalse(v.attitude_is_approximate)

    def test_planar_hides_rear_and_right_and_says_why(self):
        v = self.visibility(BASE + WHEEL + QUASI)
        self.assertEqual(v.visible, ("track", "top"))
        self.assertFalse(v.attitude_is_dynamic)
        self.assertEqual(len(v.notes), 1)
        self.assertIn("rear/right hidden", v.notes[0])
        self.assertIn("--quasi-static-attitude", v.notes[0])
        self.assertIn("synthetic", v.notes[0])

    def test_planar_with_the_opt_in_shows_approximate_attitude(self):
        v = self.visibility(BASE + WHEEL + QUASI, allow_quasi_static=True)
        self.assertEqual(v.visible, WINDOWS)
        self.assertTrue(v.attitude_is_approximate)
        self.assertFalse(v.attitude_is_dynamic)

    def test_the_flag_changes_nothing_without_the_quasi_static_channels(self):
        plain = self.visibility(BASE)
        flagged = self.visibility(BASE, allow_quasi_static=True)
        self.assertEqual(plain.visible, flagged.visible)
        self.assertEqual(plain.visible, ("track",))

    def test_point_mass_hides_top_until_forced(self):
        v = self.visibility(BASE)
        self.assertEqual(v.visible, ("track",))
        self.assertFalse(v.tire_arrows or v.fz_bars)
        forced = self.visibility(BASE, force_top_view=True)
        self.assertEqual(forced.visible, ("track", "top"))
        self.assertFalse(forced.tire_arrows or forced.fz_bars)
        self.assertTrue(any("tire arrows hidden" in n for n in forced.notes))

    def test_no_tire_force_channels_keeps_the_top_view(self):
        channels = tuple(c for c in BASE + WHEEL + DYNAMIC
                         if c not in ("tire-longitudinal-force", "tire-lateral-force",
                                      "tire-aligning-moment"))
        v = self.visibility(channels)
        self.assertTrue(v.top and not v.tire_arrows and v.fz_bars)

    def test_no_attitude_channel_at_all_hides_both_windows(self):
        v = self.visibility(BASE + WHEEL)
        self.assertFalse(v.rear or v.right)
        self.assertIn("no roll or pitch channel", v.notes[0])


class LayoutTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def layout(self, channels, **options):
        quasi = options.get("allow_quasi_static", False)
        source = make_source(self.directory.name, channels, quasi=quasi)
        return window_grid(window_visibility(source, **options))

    def check_inside(self, layout):
        for name in layout.windows:
            left, bottom, width, height = layout[name]
            self.assertGreaterEqual(left, 0.0)
            self.assertGreaterEqual(bottom, layout.footer[3] - 1e-12)
            self.assertLessEqual(left + width, 1.0 + 1e-12)
            self.assertLessEqual(bottom + height, 1.0 + 1e-12)
        names = layout.windows
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                ax, ay, aw, ah = layout[a]
                bx, by, bw, bh = layout[b]
                overlap = (ax < bx + bw - 1e-12 and bx < ax + aw - 1e-12
                           and ay < by + bh - 1e-12 and by < ay + ah - 1e-12)
                self.assertFalse(overlap, f"{a} overlaps {b}")

    def test_full_layout_has_top_above_and_attitude_below(self):
        layout = self.layout(BASE + WHEEL + DYNAMIC)
        self.check_inside(layout)
        track, top, rear, right = (layout[n] for n in ("track", "top", "rear", "right"))
        self.assertAlmostEqual(track[3], top[3] + rear[3] + (top[1] - rear[1] - rear[3]))
        self.assertGreater(top[3], rear[3])           # the top view is the larger window
        self.assertGreater(top[1], rear[1])           # and sits above
        self.assertAlmostEqual(rear[1], right[1])     # rear and right share the lower row
        self.assertLess(rear[0], right[0])            # rear is bottom-left, right bottom-right
        self.assertAlmostEqual(top[0], rear[0])
        self.assertAlmostEqual(top[0] + top[2], right[0] + right[2])

    def test_without_attitude_the_top_view_takes_the_full_height(self):
        layout = self.layout(BASE + WHEEL)
        self.check_inside(layout)
        self.assertEqual(layout.windows, ("track", "top"))
        self.assertAlmostEqual(layout["top"][3], layout["track"][3])
        self.assertAlmostEqual(layout["top"][1], layout["track"][1])

    def test_without_top_the_attitude_windows_take_the_full_height(self):
        channels = tuple(c for c in BASE + DYNAMIC)
        layout = self.layout(channels)
        # no wheel channels -> the top view is hidden, the attitude windows grow
        self.check_inside(layout)
        self.assertEqual(layout.windows, ("track", "rear", "right"))
        for name in ("rear", "right"):
            self.assertAlmostEqual(layout[name][3], layout["track"][3])
            self.assertAlmostEqual(layout[name][1], layout["track"][1])
        self.assertAlmostEqual(layout["rear"][2], layout["right"][2])

    def test_track_only_fills_the_figure(self):
        layout = self.layout(BASE)
        self.check_inside(layout)
        left, bottom, width, height = layout["track"]
        self.assertEqual(layout.windows, ("track",))
        self.assertGreater(width, 0.95)
        self.assertGreater(height, 0.9)

    def test_freed_width_goes_to_the_top_view_first(self):
        full = self.layout(BASE + WHEEL + DYNAMIC)
        without = self.layout(BASE + WHEEL)
        self.assertGreater(without["top"][3], full["top"][3])
        self.assertGreaterEqual(without["top"][2], full["top"][2])

    def test_a_lone_rear_window_is_refused(self):
        from tools.anim.layout import WindowVisibility
        with self.assertRaises(ValueError):
            window_grid(WindowVisibility(True, True, False, False, False, False, True))


class PanelTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.source = make_source(self.directory.name, BASE + WHEEL + DYNAMIC)
        self.scene = placeholder_scene(GEOMETRY)
        self.visibility = window_visibility(self.source)
        self.figure, self.axes, self.style = axes_for(self.visibility)
        self.panels = build_panels(self.axes, self.source, self.scene, track_geometry(),
                                   self.style, self.visibility)

    def update(self, index):
        for panel in self.panels.values():
            panel.update(index)

    def test_track_marker_and_axes(self):
        self.update(40)
        track = self.panels["track"]
        x, y = self.source.scalar_at("position-x", 40), self.source.scalar_at("position-y", 40)
        self.assertEqual(track.cg_marker.get_data(), ([y], [x]))     # screen (Y east, X north)
        psi, beta = self.source.scalar_at("heading-world", 40), SLIP
        bx, by = track.body_axis.get_data()
        np.testing.assert_allclose(np.arctan2(bx[1] - bx[0], by[1] - by[0]), psi, atol=1e-12)
        vx, vy = track.velocity.get_data()
        np.testing.assert_allclose(np.arctan2(vx[1] - vx[0], vy[1] - vy[0]), psi + beta,
                                   atol=1e-12)
        # the velocity is rotated clockwise of the body axis by exactly the slip angle
        delta = np.arctan2(vx[1] - vx[0], vy[1] - vy[0]) - np.arctan2(bx[1] - bx[0],
                                                                      by[1] - by[0])
        self.assertAlmostEqual(delta, SLIP, places=12)

    def test_velocity_and_acceleration_arrow_lengths_follow_the_fixed_scale(self):
        self.update(10)
        track = self.panels["track"]
        vx, vy = track.velocity.get_data()
        # constant 20 m/s equals the run maximum, so the arrow is exactly full length
        self.assertAlmostEqual(np.hypot(vx[1] - vx[0], vy[1] - vy[0]), track.velocity_full)
        ax, ay = track.acceleration.get_data()
        self.assertAlmostEqual(np.hypot(ax[1] - ax[0], ay[1] - ay[0]), track.acceleration_full)
        self.update(30)
        vx2, _ = track.velocity.get_data()
        self.assertEqual(len(vx2), 5)

    def test_acceleration_is_the_body_vector_rotated_into_the_world(self):
        self.update(20)
        track = self.panels["track"]
        psi = self.source.scalar_at("heading-world", 20)
        ax, ay = track.acceleration.get_data()
        got = np.array([ax[1] - ax[0], ay[1] - ay[0]])             # screen (Y, X)
        scale = track.acceleration_full / track.scales["acceleration"]
        east = scale * (2.0 * np.sin(psi) + (-4.0) * np.cos(psi))
        north = scale * (2.0 * np.cos(psi) + (-4.0) * -np.sin(psi))
        np.testing.assert_allclose(got, [east, north], atol=1e-12)

    def test_traveled_path_grows_with_the_frame_index(self):
        track = self.panels["track"]
        track.update(5)
        short = len(track.traveled.get_xdata())
        track.update(50)
        self.assertEqual(short, 6)
        self.assertEqual(len(track.traveled.get_xdata()), 51)

    def test_rear_and_right_lines_show_the_recorded_angle(self):
        self.update(33)
        x, y = self.panels["rear"].reference.get_data()
        np.testing.assert_allclose(np.arctan2(-(y[1] - y[0]), x[1] - x[0]), ROLL, atol=1e-12)
        x, y = self.panels["right"].reference.get_data()
        np.testing.assert_allclose(np.arctan2(y[1] - y[0], x[1] - x[0]), PITCH, atol=1e-12)

    def test_zero_and_signed_angles_are_exact_at_1e_12(self):
        for roll, pitch in ((0.0, 0.0), (0.05, 0.05), (-0.05, -0.05)):
            source = make_source(self.directory.name, BASE + WHEEL + DYNAMIC, model="x")
            source._valid = source._valid  # noqa: B018 - synthetic record keeps its channels
            source.frames.channel[:, source._columns["roll"]] = roll
            source.frames.channel[:, source._columns["pitch"]] = pitch
            figure, axes, style = axes_for(window_visibility(source))
            rear = build_attitude_panel(axes["rear"], source, self.scene, "rear", style)
            right = build_attitude_panel(axes["right"], source, self.scene, "right", style)
            rear.update(3)
            right.update(3)
            x, y = rear.reference.get_data()
            self.assertAlmostEqual(np.arctan2(-(y[1] - y[0]), x[1] - x[0]), roll, places=12)
            x, y = right.reference.get_data()
            self.assertAlmostEqual(np.arctan2(y[1] - y[0], x[1] - x[0]), pitch, places=12)

    def test_attitude_panel_refuses_a_record_without_attitude(self):
        source = make_source(self.directory.name, BASE + WHEEL, model="flat")
        _, axes, style = axes_for(window_visibility(source))
        with self.assertRaises(MissingChannelError):
            build_attitude_panel(axes["top"], source, self.scene, "rear", style)

    def test_top_velocity_is_vertical_and_body_x_is_rotated_by_minus_the_slip(self):
        self.update(25)
        top = self.panels["top"]
        x, y = top.velocity.get_data()
        self.assertEqual(x[0], x[1])
        self.assertGreater(y[1], y[0])
        bx, by = top.body_axis.get_data()
        # body +x on screen leans to the left of vertical by beta: -beta clockwise
        np.testing.assert_allclose(np.arctan2(bx[1], by[1]), -SLIP, atol=1e-12)

    def test_tire_arrows_are_the_body_axes_rotated_by_the_steer_angle(self):
        self.update(15)
        top = self.panels["top"]
        scale = top.arrow_full / top.scales["tire_force"]
        for wheel, delta in ((0, STEER), (1, STEER), (2, 0.0), (3, 0.0)):
            x, y = top.tire_fx[wheel].get_data()
            vector = np.array([x[1] - x[0], y[1] - y[0]])
            fx = (100.0, 110.0, 120.0, 130.0)[wheel]
            # tire x = body x rotated by delta (clockwise on screen), then the -beta yaw
            angle = delta - SLIP
            np.testing.assert_allclose(vector, fx * scale * np.array([np.sin(angle),
                                                                      np.cos(angle)]),
                                       atol=1e-12)
            x, y = top.tire_fy[wheel].get_data()
            vector = np.array([x[1] - x[0], y[1] - y[0]])
            fy = (400.0, 410.0, 420.0, 430.0)[wheel]
            np.testing.assert_allclose(vector, fy * scale * np.array([np.cos(angle),
                                                                      -np.sin(angle)]),
                                       atol=1e-12)

    def test_tire_arrows_start_at_the_wheel_centres(self):
        self.update(15)
        top = self.panels["top"]
        centres = top._wheel_screen(SLIP)
        for k in range(4):
            x, y = top.tire_fx[k].get_data()
            np.testing.assert_allclose([x[0], y[0]], centres[k], atol=1e-12)

    def test_fz_bars_are_proportional_to_the_load(self):
        self.update(15)
        top = self.panels["top"]
        heights = np.array([bar.get_height() for bar in top.fz_bars])
        np.testing.assert_allclose(heights / heights[-1], np.array([1000, 1100, 1200, 1300]) / 1300)
        self.assertAlmostEqual(heights[-1], top.fz_height)

    def test_overlay_bars_show_throttle_brake_and_steer(self):
        self.update(15)
        top = self.panels["top"]
        self.assertAlmostEqual(top.bars["throttle"].get_width(), 0.5 * top.overlay_width)
        self.assertAlmostEqual(top.bars["brake"].get_width(), 0.1 * top.overlay_width)
        # steer is at the run maximum, so it fills the positive half
        self.assertAlmostEqual(top.bars["steer"].get_width(), 0.5 * top.overlay_width)
        self.assertAlmostEqual(top.bars["steer"].get_x(),
                               top.overlay_x0 + 0.5 * top.overlay_width)

    def test_legends_print_units_and_full_scales(self):
        top = " ".join(self.panels["top"].legend_lines())
        self.assertIn("N,", top)
        self.assertIn("N*m", top)
        self.assertIn("full scale = 1300 N", top)
        track = " ".join(self.panels["track"].legend_lines())
        self.assertIn("20.0 m/s", track)
        self.assertIn("m/s^2", track)

    def test_rendering_a_five_second_excerpt_twice_gives_identical_artists(self):
        def run():
            samples = []
            for index in range(0, self.source.count, 7):
                self.update(index)
                samples.append(np.concatenate([
                    np.asarray(self.panels["track"].velocity.get_xydata()).ravel(),
                    np.asarray(self.panels["top"].tire_fx[0].get_xydata()).ravel(),
                    np.asarray(self.panels["rear"].vehicle.get_segments()).ravel()]))
            return np.concatenate(samples)
        np.testing.assert_array_equal(run(), run())

    def test_updating_never_changes_the_axes_limits(self):
        before = {name: (a.get_xlim(), a.get_ylim()) for name, a in self.axes.items()}
        for index in (0, 30, 60):
            self.update(index)
        self.assertEqual(before, {name: (a.get_xlim(), a.get_ylim())
                                  for name, a in self.axes.items()})


class DegradedPanelTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.scene = placeholder_scene(GEOMETRY)

    def build_top(self, channels, **options):
        forced = options.pop("force_top_view", False)
        source = make_source(self.directory.name, channels, **options)
        visibility = window_visibility(source, force_top_view=forced)
        _, axes, style = axes_for(visibility)
        panel = build_top_panel(axes["top"], source, self.scene, style, visibility)
        return source, panel

    def test_missing_tire_forces_remove_the_arrows_and_name_it_in_the_legend(self):
        channels = tuple(c for c in BASE + WHEEL
                         if c not in ("tire-longitudinal-force", "tire-lateral-force",
                                      "tire-aligning-moment"))
        _, panel = self.build_top(channels)
        panel.update(5)
        self.assertFalse(panel.has_arrows)
        self.assertTrue(panel.has_bars)
        self.assertTrue(all(not line.get_visible() for line in panel.tire_fx))
        self.assertIn("Fx/Fy/Mz arrows: not available", " ".join(panel.legend_lines()))

    def test_missing_fz_removes_the_bars_only(self):
        channels = tuple(c for c in BASE + WHEEL if c != "tire-vertical-force")
        _, panel = self.build_top(channels)
        panel.update(5)
        self.assertTrue(panel.has_arrows and not panel.has_bars)
        self.assertEqual(panel.fz_bars, [])
        self.assertIn("Fz bars: not available", " ".join(panel.legend_lines()))

    def test_forced_top_view_on_a_point_mass_record_draws_without_wheels(self):
        _, panel = self.build_top(BASE, force_top_view=True)
        panel.update(5)
        x, y = panel.velocity.get_data()
        self.assertEqual(x[0], x[1])
        self.assertFalse(panel.has_arrows or panel.has_bars)
        self.assertTrue(any(line.startswith("slip") for line in panel.readout_lines(5)))
        text = " ".join(panel.legend_lines())
        self.assertIn("Fz bars: not available", text)

    def test_an_invalid_sample_draws_nothing_and_reads_n_a(self):
        source = make_source(self.directory.name, BASE + WHEEL + DYNAMIC)
        source._valid[:, source.record.manifest.channel_names.index("body-slip-angle")] = False
        source.frames.channel[:, source._columns["body-slip-angle"]] = np.nan
        visibility = window_visibility(source)
        _, axes, style = axes_for(visibility)
        panel = build_top_panel(axes["top"], source, self.scene, style, visibility)
        # series() masks via the validity array; rebuild the cache the panel read
        panel.update(5)
        self.assertEqual(len(panel.vehicle.get_segments()), 0)
        self.assertTrue(panel.readout_lines(5)[0].endswith("n/a"))
        self.assertEqual(len(panel.tire_fx[0].get_xdata()), 0)

    def test_quasi_static_attitude_is_labelled_approximate_on_every_panel(self):
        source = make_source(self.directory.name, BASE + WHEEL + QUASI, quasi=True)
        visibility = window_visibility(source, allow_quasi_static=True)
        _, axes, style = axes_for(visibility)
        for view in ("rear", "right"):
            panel = build_attitude_panel(axes[view], source, self.scene, view, style)
            panel.update(4)
            self.assertIn("APPROXIMATE", " ".join(panel.legend_lines()))
            self.assertIn("approx.", panel.readout_lines(4)[0])
        x, y = panel.reference.get_data()
        np.testing.assert_allclose(np.arctan2(y[1] - y[0], x[1] - x[0]), -0.02, atol=1e-12)


class RecordedRunTest(unittest.TestCase):
    """Panels on a short run of each real plant, recorded by the runner."""

    @classmethod
    def setUpClass(cls):
        from tests.test_run_record_runner import real_child
        cls.runs = {}
        for model, extra in (("planar-7dof", ("--record", "--quasi-static-attitude")),
                             ("14dof-simple", ("--record",))):
            root, code, _ = real_child(model, *extra)
            assert code == 0, (model, code)
            cls.runs[model] = load_run_record(root / "run_record.npz")

    def build(self, model, **options):
        record = self.runs[model]
        source = FrameSource(record, 120.0, allow_quasi_static=options.get("quasi", False))
        visibility = window_visibility(source, allow_quasi_static=options.get("quasi", False))
        _, axes, style = axes_for(visibility)
        scene = placeholder_scene(record.manifest.vehicle_geometry)
        panels = build_panels(axes, source, scene, track_geometry(), style, visibility)
        return source, visibility, panels

    def test_dynamic_plant_shows_all_windows_and_marks_the_recorded_pose(self):
        source, visibility, panels = self.build("14dof-simple")
        self.assertEqual(visibility.visible, WINDOWS)
        index = source.count // 2
        for panel in panels.values():
            panel.update(index)
        x, y = panels["track"].cg_marker.get_data()
        self.assertEqual((x[0], y[0]), (source.scalar_at("position-y", index),
                                        source.scalar_at("position-x", index)))
        for view, column, sign in (("rear", 0, -1.0), ("right", 1, 1.0)):
            lx, ly = panels[view].reference.get_data()
            angle = np.arctan2(sign * (ly[1] - ly[0]), lx[1] - lx[0])
            self.assertAlmostEqual(angle, source.attitude_at(index)[column], places=12)

    def test_velocity_direction_matches_the_course_of_the_cg(self):
        for model in self.runs:
            record = self.runs[model]
            moving = record.channel_values("speed") > 2.0
            t = record.time
            x = record.channel_values("position-x").astype(float)
            y = record.channel_values("position-y").astype(float)
            psi = record.channel_values("heading-world").astype(float)
            beta = record.channel_values("body-slip-angle").astype(float)
            lo, hi = np.flatnonzero(moving)[0] + 5, record.rows - 5
            course = np.arctan2(y[hi] - y[lo], x[hi] - x[lo])
            direction = np.mean(np.unwrap(psi[lo:hi] + beta[lo:hi]))
            self.assertLess(abs(course - direction), 0.05, model)
            # the body heading alone is a worse match whenever the car carries slip
            self.assertLess(abs(course - direction),
                            abs(course - np.mean(psi[lo:hi])) + 0.05, model)

    def test_planar_plant_hides_attitude_by_default_and_shows_it_approximate_on_request(self):
        _, hidden, _ = self.build("planar-7dof")
        self.assertEqual(hidden.visible, ("track", "top"))
        source, shown, panels = self.build("planar-7dof", quasi=True)
        self.assertEqual(shown.visible, WINDOWS)
        self.assertTrue(shown.attitude_is_approximate and not shown.attitude_is_dynamic)
        self.assertIn("APPROXIMATE", " ".join(panels["rear"].legend_lines()))
        attitude = [w for w in source.warnings if w.startswith("roll/pitch")]
        self.assertEqual(len(attitude), 1)
        self.assertTrue(any(w.startswith("coarse record") for w in source.warnings))

    def test_tire_force_scales_are_the_run_maximum(self):
        for model, record in self.runs.items():
            source = FrameSource(record, 120.0)
            fx = record.channel_values("tire-longitudinal-force")[
                record.channel_validity("tire-longitudinal-force")]
            fy = record.channel_values("tire-lateral-force")[
                record.channel_validity("tire-longitudinal-force")]
            self.assertAlmostEqual(source.scales()["tire_force"],
                                   float(np.max(np.hypot(fx, fy))), places=3, msg=model)

    def test_every_frame_of_the_run_updates_without_error(self):
        for model in self.runs:
            source, _, panels = self.build(model, quasi=True)
            for index in range(source.count):
                for panel in panels.values():
                    panel.update(index)


if __name__ == "__main__":
    unittest.main()
