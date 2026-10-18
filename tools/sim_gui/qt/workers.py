"""Background-friendly workers for preparing and validating study cases."""
from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal, QMutex, QMutexLocker

from tools.sim_gui.execute import PreparedCase, prepare_case, validate_case
from tools.sim_gui.study import CaseSpec, StudyDefinition


class ValidationWorker(QObject):
    """Prepare and validate cases without touching Qt widgets."""

    progress = pyqtSignal(int, int)
    caseReady = pyqtSignal(object, object)
    finished = pyqtSignal()

    def __init__(self, study_dir, definition: StudyDefinition, cases: tuple[CaseSpec, ...], parent=None) -> None:
        """Create a worker for ``cases`` in ``study_dir``."""
        super().__init__(parent)
        self.study_dir = study_dir
        self.definition = definition
        self.cases = tuple(cases)
        self._cancel_mutex = QMutex()
        self._cancelled = False

    def cancel(self) -> None:
        """Request cancellation between case validations."""
        with QMutexLocker(self._cancel_mutex):
            self._cancelled = True

    def _is_cancelled(self) -> bool:
        with QMutexLocker(self._cancel_mutex):
            return self._cancelled

    def run(self) -> None:
        """Prepare and validate every case, emitting progress and results."""
        total = len(self.cases)
        for index, case in enumerate(self.cases, 1):
            if self._is_cancelled():
                break
            try:
                prepared = prepare_case(self.study_dir, self.definition, case)
                error = validate_case(prepared)
            except Exception as exc:
                prepared = None
                error = f"{type(exc).__name__}: {exc}"
            self.caseReady.emit(prepared, error)
            self.progress.emit(index, total)
        self.finished.emit()
