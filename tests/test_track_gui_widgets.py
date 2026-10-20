import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
import numpy as np

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
def setUpModule():
    global app
    app = QApplication.instance() or QApplication([])


if QApplication is not None:
    from tools.track_gui.design import SectionSpec, skidpad_design
    from tools.track_gui.qt.plots import CoursePlot, CurvaturePlot, nice_step
    from tools.track_gui.qt.section_table import SectionTableModel


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class SectionTableTests(unittest.TestCase):
    def test_edit_add_move_delete(self):
        model = SectionTableModel([SectionSpec(name="a"), SectionSpec(name="b")])
        seen = []
        model.changed.connect(lambda: seen.append(1))
        self.assertTrue(model.setData(model.index(0, 2), "42.5"))
        self.assertEqual(model.sections()[0].length, 42.5)
        self.assertFalse(model.setData(model.index(0, 2), "abc"))
        self.assertFalse(model.setData(model.index(0, 0), "spiral"))
        self.assertTrue(model.setData(model.index(0, 0), "arc"))
        self.assertEqual(model.add("arc", after=0), 1)
        self.assertEqual(model.move(1, 1), 2)
        self.assertEqual([s.name for s in model.sections()], ["a", "b", ""])
        model.remove(0)
        self.assertEqual(len(model.sections()), 2)
        self.assertGreaterEqual(len(seen), 5)

    def test_unused_cells_are_not_editable(self):
        model = SectionTableModel([SectionSpec(kind="straight")])
        self.assertFalse(model.flags(model.index(0, 3)) & Qt.ItemFlag.ItemIsEditable)
        self.assertTrue(model.flags(model.index(0, 2)) & Qt.ItemFlag.ItemIsEditable)


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class PlotTests(unittest.TestCase):
    def test_nice_step(self):
        self.assertEqual(nice_step(100.0), 20.0)
        self.assertEqual(nice_step(0.0), 1.0)
        self.assertEqual(nice_step(float("nan")), 1.0)

    def test_plots_render_to_an_image(self):
        design = skidpad_design(0.0)
        course = design.build()
        plot, curve = CoursePlot(), CurvaturePlot()
        for widget in (plot, curve):
            widget.resize(400, 300)
            self.assertFalse(widget.grab().isNull())
        plot.set_course(course.x_north, course.y_east)
        curve.set_course(course.distance, course.curvature, [0, 15])
        image = plot.grab().toImage()
        colours = {image.pixelColor(x, y).name() for x in range(0, 400, 7) for y in range(0, 300, 7)}
        self.assertGreater(len(colours), 2)
        self.assertFalse(curve.grab().isNull())

    def test_equal_scale_maps_a_circle_to_a_circle(self):
        plot = CoursePlot()
        plot.resize(500, 300)
        angle = np.linspace(0, 2 * np.pi, 50)
        plot.set_course(10 * np.cos(angle), 10 * np.sin(angle))
        from PyQt6.QtCore import QRectF
        to_point, scale = plot.transform(QRectF(0, 0, 500, 300))
        a, b = to_point(10, 0), to_point(0, 10)
        self.assertAlmostEqual(abs(a.y() - to_point(-10, 0).y()), abs(b.x() - to_point(0, -10).x()), places=6)


if __name__ == "__main__":
    unittest.main()
