"""
On-screen keyboard integration for touchscreen use.

Launches the system 'onboard' on-screen keyboard when a QLineEdit first
gains focus. Onboard stays open and manages its own visibility thereafter.
"""

import subprocess
import shutil

from PySide6.QtCore import QEvent, QObject
from PySide6.QtWidgets import QLineEdit, QApplication


class VirtualKeyboard(QObject):
    """Launches the system on-screen keyboard on first QLineEdit focus."""

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

    def _ensure_running(self):
        """Launch onboard if it's not already running."""
        if self._process and self._process.poll() is None:
            return
        try:
            self._process = subprocess.Popen(
                self._onboard_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            if isinstance(obj, QLineEdit):
                self._ensure_running()
        return False
