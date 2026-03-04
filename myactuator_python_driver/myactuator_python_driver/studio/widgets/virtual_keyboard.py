"""
On-screen keyboard integration for touchscreen use.

Launches the system on-screen keyboard when a QLineEdit gains focus,
and hides it when focus leaves all text inputs.
"""

import subprocess
import shutil
import os

from PySide6.QtCore import QEvent, QObject, QTimer
from PySide6.QtWidgets import QLineEdit, QApplication


class VirtualKeyboard(QObject):
    """Manages the system on-screen keyboard on QLineEdit focus."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._process: subprocess.Popen | None = None
        self._kbd_cmd = self._find_keyboard()

        if self._kbd_cmd:
            self._configure_onboard()
            QApplication.instance().installEventFilter(self)

    @staticmethod
    def _configure_onboard():
        """Set onboard docking preferences via gsettings."""
        settings = [
            ("org.onboard.window", "docking-enabled", "true"),
            ("org.onboard.window", "docking-edge", "'bottom'"),
            ("org.onboard.window.landscape", "dock-expand", "true"),
            ("org.onboard.window.landscape", "dock-height", "200"),
        ]
        for schema, key, value in settings:
            try:
                subprocess.Popen(
                    ["gsettings", "set", schema, key, value],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError:
                pass

    @staticmethod
    def _find_keyboard() -> list[str] | None:
        """Find a virtual keyboard binary.

        Prefers onboard on GNOME/Mutter (wvkbd needs wlr-layer-shell).
        """
        is_gnome = "GNOME" in os.environ.get("XDG_CURRENT_DESKTOP", "").upper() or \
                   os.environ.get("GNOME_SETUP_DISPLAY", "")

        if is_gnome:
            if shutil.which("onboard"):
                return ["onboard", "--size=1024x200",
                        "-x", "0", "-y", "400",
                        "-l", "Phone", "-t", "Nightshade"]

        if shutil.which("wvkbd-mobintl"):
            return ["wvkbd-mobintl", "-H", "200", "-l", "full,special"]
        if shutil.which("onboard"):
            return ["onboard", "--size=1024x200",
                    "-x", "0", "-y", "400",
                    "-l", "Phone", "-t", "Nightshade"]
        return None

    def _is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _show(self):
        """Launch the keyboard if not already running."""
        if self._is_running():
            # Already running — try to re-show via D-Bus (onboard)
            try:
                subprocess.Popen(
                    ["dbus-send", "--type=method_call",
                     "--dest=org.onboard.Onboard",
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
                self._kbd_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    def _hide(self):
        """Hide the keyboard via D-Bus (onboard)."""
        if not self._is_running():
            return
        try:
            subprocess.Popen(
                ["dbus-send", "--type=method_call",
                 "--dest=org.onboard.Onboard",
                 "/org/onboard/Onboard/Keyboard",
                 "org.onboard.Onboard.Keyboard.Hide"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            if isinstance(obj, QLineEdit):
                self._show()
        elif event.type() == QEvent.Type.FocusOut:
            if isinstance(obj, QLineEdit):
                QTimer.singleShot(150, self._check_focus)
        return False

    def _check_focus(self):
        focused = QApplication.focusWidget()
        if not isinstance(focused, QLineEdit):
            self._hide()
