"""
On-screen keyboard integration for touchscreen use.

Launches the system 'onboard' on-screen keyboard when a QLineEdit gains
focus, and dismisses it when focus moves away from text inputs.
"""

import subprocess
import shutil

from PySide6.QtCore import Qt, QEvent, QObject, QTimer
from PySide6.QtWidgets import QLineEdit, QApplication


class VirtualKeyboard(QObject):
    """Spawns/hides the system on-screen keyboard on QLineEdit focus."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._process: subprocess.Popen | None = None
        self._onboard_cmd = self._find_onboard()

        if self._onboard_cmd:
            QApplication.instance().installEventFilter(self)

    @staticmethod
    def _find_onboard() -> list[str] | None:
        """Find the onboard command, trying distrobox-host-exec fallback."""
        if shutil.which("onboard"):
            return ["onboard", "--size=1024x200"]
        if shutil.which("distrobox-host-exec"):
            return ["distrobox-host-exec", "onboard", "--size=1024x200"]
        return None

    def _show(self):
        if self._process and self._process.poll() is None:
            return  # already running
        try:
            self._process = subprocess.Popen(
                self._onboard_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    def _hide(self):
        if self._process and self._process.poll() is None:
            self._process.terminate()
            self._process = None

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            if isinstance(obj, QLineEdit):
                self._show()
        elif event.type() == QEvent.Type.FocusOut:
            if isinstance(obj, QLineEdit):
                QTimer.singleShot(100, self._check_focus)
        return False

    def _check_focus(self):
        focused = QApplication.focusWidget()
        if not isinstance(focused, QLineEdit):
            self._hide()
