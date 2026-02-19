#!/usr/bin/env python3
"""
Headless CLI entry point for calibration.

Wires RosBridge + RecordingManager + CalibrationController and runs a
calibration sequence from the command line. Uses QCoreApplication (no
display server required).

Usage:
    ros2 run myactuator_python_driver calibrator_cli \
        --joint joint1 --torque 0.5 [--name my_calib] [--offset 0.5]

Press Ctrl+C once to stop gracefully, twice to emergency stop.
"""

import argparse
import signal
import sys
import math

# Suppress ROS 2 logging noise (same as studio/main.py)
import logging
logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
logging.getLogger('rcl').setLevel(logging.CRITICAL)


def main():
    """Main entry point for headless calibration CLI."""

    # --- Dependency checks (print to stderr, no GUI) ---
    try:
        from PySide6.QtCore import QCoreApplication, QTimer
    except ImportError:
        print("Error: PySide6 is required but not installed.", file=sys.stderr)
        print("Install it with: pip install PySide6", file=sys.stderr)
        sys.exit(1)

    try:
        import rclpy  # noqa: F401
    except ImportError:
        print(
            "Error: ROS 2 Python packages (rclpy) are not available.\n"
            "Make sure you have sourced the ROS 2 setup file and built your workspace.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        print(
            "Error: rosbag2_py is not available.\n"
            "Install it with: sudo apt install ros-<distro>-rosbag2",
            file=sys.stderr,
        )
        sys.exit(1)

    # --- Argument parsing ---
    parser = argparse.ArgumentParser(
        description="Headless calibration CLI for torque threshold discovery.",
        epilog="Press Ctrl+C once to stop gracefully, twice to emergency stop.",
    )
    parser.add_argument(
        "--joint", required=True, help="Name of the joint to calibrate"
    )
    parser.add_argument(
        "--torque",
        required=True,
        type=float,
        help="Torque to apply in Nm (signed; negative reverses direction)",
    )
    parser.add_argument(
        "--name", default="", help="Optional recording name (auto-generated if omitted)"
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.5,
        help="Threshold offset in degrees (default: 0.5)",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=3.0,
        help="Seconds to track threshold after reversal before locking (default: 3.0)",
    )
    args = parser.parse_args()

    # --- Application setup ---
    app = QCoreApplication(sys.argv)
    app.setApplicationName("Calibrator CLI")

    # Import components after Qt app is created
    from myactuator_python_driver.studio.ros_bridge import RosBridge
    from myactuator_python_driver.studio.recording_manager import RecordingManager
    from myactuator_python_driver.calibrator.config import CalibrationConfig
    from myactuator_python_driver.calibrator.controller import CalibrationController

    # --- Create components ---
    ros_bridge = RosBridge()
    recording_manager = RecordingManager()
    controller = CalibrationController(ros_bridge, recording_manager)

    config = CalibrationConfig(
        joint_name=args.joint,
        torque_nm=args.torque,
        recording_name=args.name,
        offset_deg=args.offset,
        settle_time_sec=args.settle_time,
    )

    # --- Wire signals ---
    # Note: controller.__init__ already connects ros_bridge.joint_state_received
    # to controller._on_joint_state, so we do NOT double-connect that here.

    def on_state_changed(state):
        print(f"[calibrator] State: {state.name}")

    def on_error(msg):
        print(f"[calibrator] ERROR: {msg}", file=sys.stderr)

    def on_complete(result):
        print(f"\n[calibrator] === Calibration Complete ===")
        print(f"  Joint:        {result.joint_name}")
        print(f"  Threshold:    {result.max_position_rad:.4f} rad "
              f"({math.degrees(result.max_position_rad):.2f} deg)")
        print(f"  Extreme:      {result.extreme_position_rad:.4f} rad "
              f"({math.degrees(result.extreme_position_rad):.2f} deg)")
        print(f"  Torque:       {result.torque_nm:.3f} Nm")
        print(f"  Duration:     {result.duration_sec:.1f} s")
        print(f"  Recording:    {result.recording_name}")
        QTimer.singleShot(100, app.quit)

    def on_max_position(pos):
        print(f"[calibrator] Position: {pos:.4f} rad "
              f"({math.degrees(pos):.2f} deg)")

    def on_reversal(extreme):
        print(f"[calibrator] Reversal detected at {extreme:.4f} rad "
              f"({math.degrees(extreme):.2f} deg) — tracking threshold "
              f"for {args.settle_time:.0f}s")

    def on_threshold_locked(threshold):
        print(f"[calibrator] Threshold locked: {threshold:.4f} rad "
              f"({math.degrees(threshold):.2f} deg)")

    controller.state_changed.connect(on_state_changed)
    controller.error_occurred.connect(on_error)
    controller.calibration_complete.connect(on_complete)
    controller.max_position_updated.connect(on_max_position)
    controller.reversal_detected.connect(on_reversal)
    controller.threshold_locked.connect(on_threshold_locked)

    # --- Connection handling ---
    calibration_started = False

    def on_connection_changed(connected):
        nonlocal calibration_started
        if connected and not calibration_started:
            print("[calibrator] Connected to driver")
            # Wait 1 second for joint names to populate, then start
            QTimer.singleShot(1000, start_calibration)

    def start_calibration():
        nonlocal calibration_started
        if calibration_started:
            return
        calibration_started = True

        joint_names = ros_bridge.joint_names
        if args.joint not in joint_names:
            print(
                f"[calibrator] ERROR: Joint '{args.joint}' not found. "
                f"Available joints: {joint_names}",
                file=sys.stderr,
            )
            app.quit()
            return

        print(f"[calibrator] Starting calibration on '{args.joint}' "
              f"with {args.torque} Nm torque")
        if not controller.start_calibration(config):
            print("[calibrator] ERROR: Failed to start calibration", file=sys.stderr)
            app.quit()

    ros_bridge.connection_status_changed.connect(on_connection_changed)

    # --- Signal handling (Ctrl+C) ---
    # All prints scheduled on Qt event loop to avoid reentrant stdout writes
    stop_requested = False

    def sigint_handler(signum, frame):
        nonlocal stop_requested
        if not stop_requested and controller.is_active:
            stop_requested = True
            def _graceful():
                print("\n[calibrator] Stopping gracefully... (press Ctrl+C again to e-stop)")
                controller.stop_calibration()
            QTimer.singleShot(0, _graceful)
        else:
            def _estop():
                print("\n[calibrator] EMERGENCY STOP!")
                controller.emergency_stop()
                QTimer.singleShot(200, app.quit)
            QTimer.singleShot(0, _estop)

    signal.signal(signal.SIGINT, sigint_handler)

    # Qt needs a timer to process Python signals (SIGINT won't interrupt C++ event loop)
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)

    # --- Start ---
    print("[calibrator] Starting ROS bridge... waiting for driver connection")
    print("[calibrator] Press Ctrl+C to stop calibration")
    ros_bridge.start()

    # Run the event loop
    sys.exit(app.exec())


def main_gui():
    """GUI entry point for Torque Threshold Calibrator."""
    import logging
    logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
    logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
    logging.getLogger('rcl').setLevel(logging.CRITICAL)

    try:
        from PySide6.QtWidgets import QApplication, QMessageBox
        from PySide6.QtCore import Qt
    except ImportError:
        print("Error: PySide6 is required but not installed.")
        print("Install it with: pip install PySide6")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setApplicationName("Torque Threshold Calibrator")

    # Apply qt-material theme if available (same pattern as studio/main.py)
    try:
        from qt_material import apply_stylesheet
        apply_stylesheet(app, theme='dark_blue.xml')
    except ImportError:
        from PySide6.QtWidgets import QStyleFactory
        from PySide6.QtGui import QPalette, QColor
        app.setStyle(QStyleFactory.create("Fusion"))
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

    # Check for ROS 2 availability (use QMessageBox since we have GUI)
    try:
        import rclpy  # noqa: F401
    except ImportError:
        QMessageBox.critical(
            None,
            "ROS 2 Not Found",
            "ROS 2 Python packages (rclpy) are not available.\n\n"
            "Make sure you have:\n"
            "1. Installed ROS 2\n"
            "2. Sourced the ROS 2 setup file\n"
            "3. Built and sourced your workspace"
        )
        sys.exit(1)

    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        QMessageBox.critical(
            None,
            "rosbag2 Not Found",
            "rosbag2_py is not available.\n\n"
            "Make sure ros-<distro>-rosbag2 is installed."
        )
        sys.exit(1)

    from myactuator_python_driver.calibrator.window import CalibrationWindow
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
