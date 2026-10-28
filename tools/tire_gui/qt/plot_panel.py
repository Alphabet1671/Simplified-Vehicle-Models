from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from PyQt6.QtWidgets import QWidget, QVBoxLayout


class PlotPanel(QWidget):
    def __init__(self, parent=None, count=4):
        super().__init__(parent)
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvasQTAgg(self.figure)
        rows = min(2, count)
        columns = (count + rows - 1) // rows
        self.axes = self.figure.subplots(rows, columns, squeeze=False).ravel()[:count]
        layout = QVBoxLayout(self)
        layout.addWidget(self.canvas)
        self.paletteChanged()

    def paletteChanged(self):
        from PyQt6.QtGui import QPalette
        p = self.palette()
        bg = p.color(QPalette.ColorRole.Base).name()
        fg = p.color(QPalette.ColorRole.Text).name()
        self.figure.set_facecolor(bg)
        for ax in self.axes:
            ax.set_facecolor(bg)
            ax.tick_params(colors=fg)
            ax.xaxis.label.set_color(fg)
            ax.yaxis.label.set_color(fg)
            ax.title.set_color(fg)

    def message(self, text):
        for ax in self.axes:
            ax.clear()
            ax.text(.5, .5, text, ha='center', va='center', transform=ax.transAxes)
            ax.set_axis_off()
        self.canvas.draw_idle()
