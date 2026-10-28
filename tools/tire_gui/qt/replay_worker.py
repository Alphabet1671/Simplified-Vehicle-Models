from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from ..transient import simulate_event

class ReplayWorker(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, snapshot, seed, event, generation):
        super().__init__()
        from threading import Event
        self.snapshot, self.seed, self.event, self.generation = snapshot, seed, event, generation
        self._cancel = Event()

    def cancel(self):
        self._cancel.set()

    @pyqtSlot()
    def run(self):
        try:
            current = simulate_event(self.snapshot, self.event, .01, _cancel_check=self._cancel.is_set)
            # Display compares the live model only; no seed/baseline replay.
            self.finished.emit((self.generation, current, None))
        except Exception as exc:
            self.failed.emit(str(exc))
