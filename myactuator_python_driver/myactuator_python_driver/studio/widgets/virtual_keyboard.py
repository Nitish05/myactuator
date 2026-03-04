"""
On-screen keyboard integration for touchscreen use.

Launches wvkbd (Wayland virtual keyboard) when a QLineEdit gains focus,
and hides it when focus leaves all text inputs.
"""

import subprocess
import shutil
import signal

from PySide6.QtCore import QEvent, QObject, QTimer
from PySide6.QtWidgets import QLineEdit, QApplication


class VirtualKeyboard(QObject):
    """Manages the system on-screen keyboard on QLineEdit focus."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._process: subprocess.Popen | None = None
        self._kbd_cmd = self._find_keyboard()

        if self._kbd_cmd:
            QApplication.instance().installEventFilter(self)

    @staticmethod
    def _find_keyboard() -> list[str] | None:
        """Find a virtual keyboard binary."""
        if shutil.which("wvkbd-mobintl"):
            return ["wvkbd-mobintl", "-H", "200", "-l", "full,special"]
        if shutil.which("onboard"):
            return ["onboard", "--size=1024x200"]
        return None

    def _show(self):
        """Launch the keyboard if not already running."""
        if self._process and self._process.poll() is None:
            return
        try:
            self._process = subprocess.Popen(
                self._kbd_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    def _hide(self):
        """Hide the keyboard."""
        if self._process and self._process.poll() is None:
            self._process.send_signal(signal.SIGUSR2)

    def _unhide(self):
        """Unhide the keyboard."""
        if self._process and self._process.poll() is None:
            self._process.send_signal(signal.SIGUSR1)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            if isinstance(obj, QLineEdit):
                if self._process and self._process.poll() is None:
                    self._unhide()
                else:
                    self._show()
        elif event.type() == QEvent.Type.FocusOut:
            if isinstance(obj, QLineEdit):
                QTimer.singleShot(150, self._check_focus)
        return False

    def _check_focus(self):
        focused = QApplication.focusWidget()
        if not isinstance(focused, QLineEdit):
            self._hide()
