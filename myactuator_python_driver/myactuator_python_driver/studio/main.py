#!/usr/bin/env python3
"""
Main entry point for Motor Recording Studio.

A PyQt6 desktop application for recording and playing back motor trajectories.
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# Entry points that should be killed before a fresh studio launch
_ENTRY_POINTS = [
    "driver_node",
    "motor_studio",
    "recorder_tui",
    "setup_tui",
    "calibrator_cli",
    "calibrator_app",
    "velocity_demo",
]


def _cleanup_ros_processes():
    """Kill stale ROS 2 processes from previous sessions.

    Mirrors the cleanup in scripts/motor-studio.sh so it works
    regardless of how the studio is launched (ros2 run, direct, etc.).
    """
    own_pid = os.getpid()
    pids_to_kill: list[int] = []

    for entry in _ENTRY_POINTS:
        try:
            result = subprocess.run(
                ["pgrep", "-f", entry],
                capture_output=True, text=True, timeout=5,
            )
            for line in result.stdout.strip().splitlines():
                try:
                    pid = int(line)
                    if pid != own_pid:
                        pids_to_kill.append(pid)
                except ValueError:
                    continue
        except Exception:
            continue

    if not pids_to_kill:
        # Nothing to clean — still reset the daemon
        try:
            subprocess.run(
                ["ros2", "daemon", "stop"],
                capture_output=True, timeout=10,
            )
        except Exception:
            pass
        return

    # SIGTERM first to allow graceful motor release
    for pid in pids_to_kill:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    # Wait up to 2 seconds for processes to exit
    deadline = time.monotonic() + 2.0
    remaining = list(pids_to_kill)
    while remaining and time.monotonic() < deadline:
        time.sleep(0.1)
        remaining = [p for p in remaining if _pid_alive(p)]

    # SIGKILL stragglers
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass

    # Clear stale ROS 2 graph state
    try:
        subprocess.run(
            ["ros2", "daemon", "stop"],
            capture_output=True, timeout=10,
        )
    except Exception:
        pass


def _pid_alive(pid: int) -> bool:
    """Return True if *pid* is still running."""
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main():
    """Main entry point."""
    # Suppress ROS 2 logging noise
    import logging
    logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
    logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
    logging.getLogger('rcl').setLevel(logging.CRITICAL)

    # Kill stale ROS 2 processes before anything else
    _cleanup_ros_processes()

    # Import PySide6
    try:
        from PySide6.QtWidgets import QApplication, QMessageBox
        from PySide6.QtCore import Qt
    except ImportError:
        print("Error: PySide6 is required but not installed.")
        print("Install it with: pip install PySide6")
        sys.exit(1)

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("Motor Recording Studio")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("MyActuator")

    # Apply qt-material theme if available
    try:
        from qt_material import apply_stylesheet
        apply_stylesheet(app, theme='dark_blue.xml')
    except ImportError:
        # qt-material not installed, use default dark palette
        from PySide6.QtWidgets import QStyleFactory
        from PySide6.QtGui import QPalette, QColor

        app.setStyle(QStyleFactory.create("Fusion"))

        # Dark palette
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
        palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
        palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
        palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
        palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
        palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
        palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)

        app.setPalette(palette)

    # Load custom stylesheet
    style_path = Path(__file__).parent / "resources" / "style.css"
    if style_path.exists():
        with open(style_path, 'r') as f:
            app.setStyleSheet(app.styleSheet() + f.read())

    # Check for ROS 2 availability
    try:
        import rclpy
    except ImportError:
        QMessageBox.critical(
            None,
            "ROS 2 Not Found",
            "ROS 2 Python packages (rclpy) are not available.\n\n"
            "Make sure you have:\n"
            "1. Installed ROS 2\n"
            "2. Sourced the ROS 2 setup file\n"
            "3. Built and sourced your workspace\n\n"
            "Example:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source install/setup.bash"
        )
        sys.exit(1)

    # Check for rosbag2
    try:
        import rosbag2_py
    except ImportError:
        QMessageBox.critical(
            None,
            "rosbag2 Not Found",
            "rosbag2_py is not available.\n\n"
            "Make sure ros-<distro>-rosbag2 is installed.\n\n"
            "Example:\n"
            "  sudo apt install ros-humble-rosbag2"
        )
        sys.exit(1)

    # Create and show main window
    from myactuator_python_driver.studio.main_window import MainWindow

    window = MainWindow()
    window.show()
    window.showFullScreen()

    # Run application
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
