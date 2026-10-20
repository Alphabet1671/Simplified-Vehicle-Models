import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest

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
    from tools.vehicle_gui.model import ParameterRow
    from tools.vehicle_gui.qt.parameter_table import ParameterTableModel


@unittest.skipIf(QApplication is None, "PyQt6 is not installed")
class TableModelTests(unittest.TestCase):
    def test_non_numeric_rows_are_not_editable(self):
        model = ParameterTableModel()
        model.set_rows([ParameterRow("a", "text", "", None), ParameterRow("b", "1.0", "m", None)])
        self.assertFalse(model.flags(model.index(0, 1)) & Qt.ItemFlag.ItemIsEditable)
        self.assertTrue(model.flags(model.index(1, 1)) & Qt.ItemFlag.ItemIsEditable)
        refused = []
        edited = []
        model.refused.connect(refused.append)
        model.edited.connect(lambda name, value: edited.append((name, value)))
        self.assertFalse(model.setData(model.index(0, 1), "2.0"))
        self.assertEqual([], edited)
        self.assertEqual("a: is not a number and cannot be edited here", refused[-1])

    def test_locked_table_rejects_direct_edit(self):
        model = ParameterTableModel()
        model.set_rows([ParameterRow("a", "1.0", "", None)], locked_reason="generated")
        refused = []
        edited = []
        model.refused.connect(refused.append)
        model.edited.connect(lambda name, value: edited.append((name, value)))
        self.assertFalse(model.setData(model.index(0, 1), "2.0"))
        self.assertEqual([], edited)
        self.assertIn("read-only", refused[-1])

    def test_invalid_and_nonfinite_values_are_refused(self):
        model = ParameterTableModel()
        model.set_rows([ParameterRow("a", "1.0", "", None)])
        refused = []
        edited = []
        model.refused.connect(refused.append)
        model.edited.connect(lambda name, value: edited.append((name, value)))
        index = model.index(0, 1)
        self.assertFalse(model.setData(index, "not-a-number"))
        self.assertFalse(model.setData(index, "nan"))
        self.assertFalse(model.setData(index, "inf"))
        self.assertEqual([], edited)
        self.assertEqual("1.0", model.data(index))
        self.assertEqual(3, len(refused))

    def test_valid_value_emits_edited_without_mutating_rows(self):
        model = ParameterTableModel()
        model.set_rows([ParameterRow("a", "1.0", "", None)])
        edited = []
        model.edited.connect(lambda name, value: edited.append((name, value)))
        index = model.index(0, 1)
        self.assertTrue(model.setData(index, "2.5"))
        self.assertEqual([("a", 2.5)], edited)
        self.assertEqual("1.0", model.data(index))


if __name__ == "__main__":
    unittest.main()
