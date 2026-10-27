from threading import Event
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from ..fit import fit_steady
from ..transient import fit_transient
from ..contracts import FitCancelled

class FitWorker(QObject):
    progress=pyqtSignal(object); finished=pyqtSignal(object); failed=pyqtSignal(str); cancelled=pyqtSignal()
    def __init__(self, snapshot, mode, selection=None, locked=frozenset(), events=(), bounds=None, runner=None):
        super().__init__(); self.snapshot=snapshot; self.mode=mode; self.selection=selection; self.locked=frozenset(locked); self.events=tuple(events); self.bounds=dict(bounds or {}); self._cancel=Event(); self.runner=runner
    def cancel(self): self._cancel.set()
    @pyqtSlot()
    def run(self):
        try:
            if self.runner is not None:
                result = self.runner(self.snapshot, self._cancel.is_set)
                if self._cancel.is_set(): self.cancelled.emit()
                else: self.finished.emit(result)
                return
            fn=fit_steady if self.mode=='steady' else fit_transient
            result=fn(self.snapshot,self.selection,self.locked,self._cancel.is_set,self.progress.emit,bounds_override=self.bounds) if self.mode=='steady' else fn(self.snapshot,self.events,self.locked,self._cancel.is_set,self.progress.emit,bounds_override=self.bounds)
            if self._cancel.is_set(): self.cancelled.emit()
            else: self.finished.emit(result)
        except FitCancelled: self.cancelled.emit()
        except Exception as exc: self.failed.emit(str(exc))
