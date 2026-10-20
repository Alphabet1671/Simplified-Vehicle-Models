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


if __name__ == "__main__":
    unittest.main()
