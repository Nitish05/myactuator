#!/usr/bin/env python3
"""
Main entry point for Motor Recording Studio.

A PyQt6 desktop application for recording and playing back motor trajectories.
"""

import sys
import os
from pathlib import Path


def main():
    """Main entry point."""
    # Suppress ROS 2 logging noise
    import logging
    logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
    logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
    logging.getLogger('rcl').setLevel(logging.CRITICAL)

    # Import PyQt6
    try:
        from PyQt6.QtWidgets import QApplication, QMessageBox
        from PyQt6.QtCore import Qt
    except ImportError:
        print("Error: PyQt6 is required but not installed.")
        print("Install it with: pip install PyQt6")
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
        from PyQt6.QtWidgets import QStyleFactory
        from PyQt6.QtGui import QPalette, QColor

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

    # Run application
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
