"""
On-screen keyboard integration for touchscreen use.

Launches the system 'onboard' on-screen keyboard when a QLineEdit gains
focus, and shows it via D-Bus if it was hidden.
"""

import subprocess
import shutil

from PySide6.QtCore import QEvent, QObject
from PySide6.QtWidgets import QLineEdit, QApplication


class VirtualKeyboard(QObject):
    """Launches and shows the system on-screen keyboard on QLineEdit focus."""

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
        """Launch onboard or show it via D-Bus if already running but hidden."""
        if self._process and self._process.poll() is None:
            # Process alive but maybe hidden — tell it to show via D-Bus
            try:
                subprocess.Popen(
                    ["dbus-send", "--type=method_call", "--dest=org.onboard.Onboard",
                     "/org/onboard/Onboard/Keyboard",
                     "org.onboard.Onboard.Keyboard.Show"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError:
                pass
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
                self._show()
        return False
