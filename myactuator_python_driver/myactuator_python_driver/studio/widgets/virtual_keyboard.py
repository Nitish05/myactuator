"""
On-screen keyboard integration for touchscreen use.

Starts onboard at app launch with auto-show enabled. Onboard handles
its own show/hide based on text field focus via AT-SPI accessibility.
"""

import subprocess
import shutil
import os


class VirtualKeyboard:
    """Launches the system on-screen keyboard at app startup."""

    def __init__(self, parent=None):
        self._process: subprocess.Popen | None = None

        # Configure and launch
        self._configure_onboard()
        self._launch()

    @staticmethod
    def _configure_onboard():
        """Set onboard preferences via gsettings."""
        settings = [
            ("org.onboard.window", "docking-enabled", "true"),
            ("org.onboard.window", "docking-edge", "'bottom'"),
            ("org.onboard.window.landscape", "dock-expand", "true"),
            ("org.onboard.window.landscape", "dock-height", "200"),
            ("org.onboard.auto-show", "enabled", "true"),
        ]
        for schema, key, value in settings:
            try:
                subprocess.run(
                    ["gsettings", "set", schema, key, value],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2,
                )
            except (OSError, subprocess.TimeoutExpired):
                pass

    def _launch(self):
        """Launch onboard in the background."""
        cmd = self._find_keyboard()
        if not cmd:
            return
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass

    @staticmethod
    def _find_keyboard() -> list[str] | None:
        """Find a virtual keyboard binary."""
        if shutil.which("onboard"):
            return ["onboard", "--size=1024x200",
                    "-x", "0", "-y", "400",
                    "-l", "Phone", "-t", "Nightshade"]
        return None
