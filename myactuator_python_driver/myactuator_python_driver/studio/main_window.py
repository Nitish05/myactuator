"""
Main window for Motor Recording Studio.

QMainWindow with docks, tabs, menu bar, and status bar.
"""

import time
from pathlib import Path

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QDockWidget, QStatusBar, QLabel, QMenuBar, QMenu, QMessageBox,
    QToolBar, QStackedWidget
)
from PyQt6.QtGui import QAction, QKeySequence

from sensor_msgs.msg import JointState

from myactuator_python_driver.config import PlaybackTriggerConfig, TriggerStore

from .ros_bridge import RosBridge
from .recording_manager import RecordingManager
from .widgets.control_panel import ControlPanel
from .widgets.joint_monitor import JointMonitor
from .widgets.monitor_tab import MonitorTab
from .widgets.record_tab import RecordTab
from .widgets.playback_tab import PlaybackTab
from .widgets.browse_tab import BrowseTab
from .widgets.easy_mode import EasyModeWidget
from .dialogs.trigger_dialog import TriggerDialog


class MainWindow(QMainWindow):
    """
    Main application window.

    Layout:
    - Left dock: Control Panel
    - Bottom dock: Joint Monitor
    - Central: Tab widget (Monitor, Record, Playback, Browse)
    - Status bar: Mode, Connection, Recording status
    """

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Motor Recording Studio")
        self.setMinimumSize(1000, 700)

        # Core components
        self._ros_bridge = RosBridge(self)
        self._recording_manager = RecordingManager(self)
        self._trigger_store = TriggerStore(self._recording_manager.recordings_dir)

        # State
        self._connected = False
        self._recording = False
        self._playing = False
        self._easy_mode = True

        # Setup UI
        self._setup_menu_bar()
        self._setup_toolbar()
        self._setup_central_widget()
        self._setup_docks()
        self._setup_status_bar()

        # Connect signals
        self._connect_signals()

        # Start in easy mode: hide advanced UI
        self._control_dock.setVisible(False)
        self._monitor_dock.setVisible(False)
        self._toolbar.setVisible(False)
        self.menuBar().setVisible(False)
        self._status_bar.setVisible(False)
        self._stack.setCurrentIndex(0)

        # Start ROS bridge
        self._ros_bridge.start()

        # Initial data load
        QTimer.singleShot(500, self._refresh_recordings)

    def _setup_menu_bar(self):
        """Set up the menu bar."""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        refresh_action = QAction("&Refresh Recordings", self)
        refresh_action.setShortcut(QKeySequence.StandardKey.Refresh)
        refresh_action.triggered.connect(self._refresh_recordings)
        file_menu.addAction(refresh_action)

        file_menu.addSeparator()

        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.StandardKey.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Control menu
        control_menu = menubar.addMenu("&Control")

        self._estop_action = QAction("&Emergency Stop", self)
        self._estop_action.setShortcut(QKeySequence("Escape"))
        self._estop_action.triggered.connect(self._emergency_stop)
        control_menu.addAction(self._estop_action)

        control_menu.addSeparator()

        set_zero_action = QAction("&Set Zero", self)
        set_zero_action.setShortcut(QKeySequence("Ctrl+Z"))
        set_zero_action.triggered.connect(self._set_zero)
        control_menu.addAction(set_zero_action)

        go_zero_action = QAction("&Go to Zero", self)
        go_zero_action.setShortcut(QKeySequence("Ctrl+0"))
        go_zero_action.triggered.connect(self._go_to_zero)
        control_menu.addAction(go_zero_action)

        # View menu
        view_menu = menubar.addMenu("&View")

        self._control_dock_action = QAction("Control Panel", self)
        self._control_dock_action.setCheckable(True)
        self._control_dock_action.setChecked(True)
        view_menu.addAction(self._control_dock_action)

        self._monitor_dock_action = QAction("Joint Monitor", self)
        self._monitor_dock_action.setCheckable(True)
        self._monitor_dock_action.setChecked(True)
        view_menu.addAction(self._monitor_dock_action)

        view_menu.addSeparator()

        simple_mode_action = QAction("&Simple Mode", self)
        simple_mode_action.setShortcut(QKeySequence("Ctrl+Shift+S"))
        simple_mode_action.triggered.connect(self._switch_to_easy)
        view_menu.addAction(simple_mode_action)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        about_action = QAction("&About", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

    def _setup_toolbar(self):
        """Set up the toolbar."""
        self._toolbar = QToolBar("Main Toolbar")
        self._toolbar.setMovable(False)
        self.addToolBar(self._toolbar)

        # E-Stop button in toolbar
        self._estop_btn_toolbar = self._toolbar.addAction("STOP")
        self._estop_btn_toolbar.setToolTip("Emergency Stop (Esc)")
        self._estop_btn_toolbar.triggered.connect(self._emergency_stop)

        self._toolbar.addSeparator()

        # Simple Mode button in toolbar
        self._simple_mode_toolbar = self._toolbar.addAction("Simple Mode")
        self._simple_mode_toolbar.setToolTip("Switch to Simple Mode (Ctrl+Shift+S)")
        self._simple_mode_toolbar.triggered.connect(self._switch_to_easy)

    def _setup_central_widget(self):
        """Set up the central widget with easy mode and tab pages."""
        self._stack = QStackedWidget()

        # Page 0: Easy Mode
        self._easy_mode_widget = EasyModeWidget()
        self._stack.addWidget(self._easy_mode_widget)

        # Page 1: Advanced Mode (tab widget)
        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)

        self._monitor_tab = MonitorTab()
        self._tabs.addTab(self._monitor_tab, "Monitor")

        self._record_tab = RecordTab()
        self._tabs.addTab(self._record_tab, "Record")

        self._playback_tab = PlaybackTab()
        self._tabs.addTab(self._playback_tab, "Playback")

        self._browse_tab = BrowseTab()
        self._tabs.addTab(self._browse_tab, "Browse")

        self._stack.addWidget(self._tabs)

        self.setCentralWidget(self._stack)

    def _setup_docks(self):
        """Set up dock widgets."""
        # Control panel dock (left)
        self._control_panel = ControlPanel()
        self._control_dock = QDockWidget("Control Panel", self)
        self._control_dock.setWidget(self._control_panel)
        self._control_dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable |
            QDockWidget.DockWidgetFeature.DockWidgetClosable
        )
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self._control_dock)

        # Connect dock visibility to menu action
        self._control_dock.visibilityChanged.connect(self._control_dock_action.setChecked)
        self._control_dock_action.triggered.connect(self._control_dock.setVisible)

        # Joint monitor dock (bottom)
        self._joint_monitor = JointMonitor()
        self._monitor_dock = QDockWidget("Joint Monitor", self)
        self._monitor_dock.setWidget(self._joint_monitor)
        self._monitor_dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable |
            QDockWidget.DockWidgetFeature.DockWidgetClosable
        )
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._monitor_dock)

        # Connect dock visibility to menu action
        self._monitor_dock.visibilityChanged.connect(self._monitor_dock_action.setChecked)
        self._monitor_dock_action.triggered.connect(self._monitor_dock.setVisible)

    def _setup_status_bar(self):
        """Set up the status bar."""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)

        # Mode indicator
        self._mode_label = QLabel("Mode: --")
        self._mode_label.setObjectName("statusMode")
        self._status_bar.addWidget(self._mode_label)

        # Separator
        self._status_bar.addWidget(QLabel("|"))

        # Connection indicator
        self._connection_label = QLabel("Disconnected")
        self._connection_label.setObjectName("statusConnection")
        self._connection_label.setStyleSheet("color: #ef5350;")
        self._status_bar.addWidget(self._connection_label)

        # Separator
        self._status_bar.addWidget(QLabel("|"))

        # Recording indicator
        self._recording_label = QLabel("Idle")
        self._status_bar.addWidget(self._recording_label)

        # Stretch
        self._status_bar.addPermanentWidget(QLabel(""))

        # Message area
        self._message_label = QLabel("")
        self._status_bar.addPermanentWidget(self._message_label)

    def _connect_signals(self):
        """Connect all signals."""
        # ROS bridge signals
        self._ros_bridge.joint_state_received.connect(self._on_joint_state)
        self._ros_bridge.mode_changed.connect(self._on_mode_changed)
        self._ros_bridge.connection_status_changed.connect(self._on_connection_changed)
        self._ros_bridge.trigger_state_changed.connect(self._on_trigger_state_changed)
        self._ros_bridge.status_message.connect(self._show_status_message)
        self._ros_bridge.error_message.connect(self._show_error_message)

        # Control panel signals
        self._control_panel.mode_requested.connect(self._ros_bridge.set_mode)
        self._control_panel.enable_requested.connect(self._ros_bridge.set_enabled)
        self._control_panel.set_zero_requested.connect(self._set_zero)
        self._control_panel.go_to_zero_requested.connect(self._go_to_zero)
        self._control_panel.emergency_stop_requested.connect(self._emergency_stop)

        # Record tab signals
        self._record_tab.start_recording.connect(self._start_recording)
        self._record_tab.stop_recording.connect(self._stop_recording)

        # Playback tab signals
        self._playback_tab.play_requested.connect(self._start_playback)
        self._playback_tab.stop_requested.connect(self._stop_playback)
        self._playback_tab.pause_requested.connect(self._toggle_pause)
        self._playback_tab.triggers_changed.connect(self._on_triggers_changed)
        self._playback_tab.configure_triggers_requested.connect(self._show_trigger_dialog)
        self._playback_tab.trigger_removed.connect(self._on_trigger_removed)

        # Browse tab signals
        self._browse_tab.refresh_requested.connect(self._refresh_recordings)
        self._browse_tab.delete_requested.connect(self._delete_recording)
        self._browse_tab.rename_requested.connect(self._rename_recording)
        self._browse_tab.recording_double_clicked.connect(self._load_for_playback)

        # Recording manager signals
        self._recording_manager.recording_started.connect(self._on_recording_started)
        self._recording_manager.recording_stopped.connect(self._on_recording_stopped)
        self._recording_manager.recording_frame.connect(self._on_recording_frame)
        self._recording_manager.playback_started.connect(self._on_playback_started)
        self._recording_manager.playback_stopped.connect(self._on_playback_stopped)
        self._recording_manager.playback_progress.connect(self._on_playback_progress)
        self._recording_manager.playback_frame.connect(self._on_playback_frame)
        self._recording_manager.error_occurred.connect(self._show_error_message)

        # Easy mode signals
        self._easy_mode_widget.recording_play_requested.connect(self._easy_play_recording)
        self._easy_mode_widget.pause_requested.connect(self._easy_toggle_pause)
        self._easy_mode_widget.stop_requested.connect(self._stop_playback)
        self._easy_mode_widget.emergency_stop_requested.connect(self._emergency_stop)
        self._easy_mode_widget.advanced_mode_requested.connect(self._switch_to_advanced)

    # === ROS Bridge Handlers ===

    def _on_joint_state(self, msg):
        """Handle joint state update."""
        # Update joint monitor
        self._joint_monitor.update_joint_state(msg)

        # Update monitor tab graph
        self._monitor_tab.update_joint_state(msg)

        # Record frame if recording
        if self._recording:
            timestamp_ns = int(time.time() * 1e9)
            self._recording_manager.record_frame(msg, timestamp_ns)

    def _on_mode_changed(self, mode: str):
        """Handle mode change."""
        self._mode_label.setText(f"Mode: {mode}")
        self._control_panel.set_mode(mode)

    def _on_connection_changed(self, connected: bool):
        """Handle connection status change."""
        self._connected = connected
        if connected:
            self._connection_label.setText("Connected")
            self._connection_label.setStyleSheet("color: #81c784;")
            # Set up direct publisher for low-latency playback
            pub = self._ros_bridge.get_joint_ctrl_publisher()
            if pub:
                self._recording_manager.set_direct_publisher(pub)
        else:
            self._connection_label.setText("Disconnected")
            self._connection_label.setStyleSheet("color: #ef5350;")
            self._recording_manager.set_direct_publisher(None)

        self._control_panel.set_connection_status(connected)
        self._easy_mode_widget.set_connection_status(connected)

    def _on_trigger_state_changed(self, states: dict):
        """Handle trigger state update."""
        self._playback_tab.set_trigger_states(states)

    # === Recording Handlers ===

    def _start_recording(self, name: str):
        """Start recording."""
        # Switch to free mode
        self._ros_bridge.set_mode("free")

        # Start recording
        if self._recording_manager.start_recording(name if name else None):
            self._recording = True

    def _stop_recording(self):
        """Stop recording."""
        self._recording_manager.stop_recording()
        self._recording = False

        # Switch back to position mode
        self._ros_bridge.set_mode("position")

    def _on_recording_started(self, name: str):
        """Handle recording started."""
        self._record_tab.set_recording(True, name)
        self._recording_label.setText(f"Recording: {name}")

    def _on_recording_stopped(self, name: str, frame_count: int):
        """Handle recording stopped."""
        self._record_tab.set_recording(False)
        self._recording_label.setText("Idle")
        self._show_status_message(f"Recording saved: {name} ({frame_count} frames)")
        self._refresh_recordings()

    def _on_recording_frame(self, count: int):
        """Handle recording frame."""
        self._record_tab.set_frame_count(count)

    # === Playback Handlers ===

    def _start_playback(self, recording):
        """Start playback."""
        # Send trigger config first - only triggers for this recording
        all_triggers = self._playback_tab.get_triggers()
        # Filter: triggers with no recording_name (global) OR matching this recording
        matching_triggers = [
            t for t in all_triggers
            if not t.recording_name or t.recording_name == recording.name
        ]
        if matching_triggers:
            config = PlaybackTriggerConfig(triggers=matching_triggers)
            self._ros_bridge.set_trigger_config(config)
        else:
            # Clear any triggers from previous playback
            self._ros_bridge.clear_trigger_config()

        # Switch to position mode
        self._ros_bridge.set_mode("position")

        # Start playback
        self._recording_manager.start_playback(recording)
        self._playing = True

    def _stop_playback(self):
        """Stop playback."""
        self._recording_manager.stop_playback()
        self._playing = False

        # Clear triggers
        self._ros_bridge.clear_trigger_config()

    def _toggle_pause(self):
        """Toggle playback pause."""
        self._recording_manager.toggle_pause()
        self._playback_tab.set_paused(self._recording_manager.is_paused)

    def _on_playback_started(self, name: str):
        """Handle playback started."""
        self._playback_tab.set_playing(True)
        self._easy_mode_widget.set_playing(True, name)
        self._recording_label.setText(f"Playing: {name}")

    def _on_playback_stopped(self):
        """Handle playback stopped."""
        self._playback_tab.set_playing(False)
        self._easy_mode_widget.set_playing(False)
        self._recording_label.setText("Idle")
        self._playing = False
        # Clear triggers when playback ends (natural stop or loop end)
        self._ros_bridge.clear_trigger_config()

    def _on_playback_progress(self, current_sec: float, total_sec: float):
        """Handle playback progress update."""
        self._playback_tab.set_progress(current_sec, total_sec)
        self._easy_mode_widget.set_progress(current_sec, total_sec)

    def _on_playback_frame(self, msg):
        """Handle playback frame - send to motors."""
        self._ros_bridge.send_joint_command(msg)

    # === Trigger Handling ===

    def _on_triggers_changed(self, config: PlaybackTriggerConfig):
        """Handle trigger configuration change."""
        if self._playing:
            # Update triggers during playback
            self._ros_bridge.set_trigger_config(config)

    def _on_trigger_removed(self, trigger):
        """Handle trigger removal - remove from persistent store."""
        self._trigger_store.remove(trigger)
        self._show_status_message(f"Trigger '{trigger.name}' removed")

    def _show_trigger_dialog(self):
        """Show the trigger configuration dialog."""
        joint_names = self._joint_monitor.get_joint_names()

        if not joint_names:
            QMessageBox.warning(
                self, "No Joints",
                "No joints detected. Make sure the driver is running and connected."
            )
            return

        # Get recording names for pairing
        recordings = self._recording_manager.get_recordings()
        recording_names = [r.name for r in recordings]

        # Pass callback for live position updates
        trigger = TriggerDialog.create_trigger(
            joint_names, recording_names, self,
            position_callback=self._joint_monitor.get_joint_positions
        )
        if trigger:
            # Save to persistent store
            self._trigger_store.add(trigger)
            # Add to playback tab
            self._playback_tab.add_trigger(trigger)
            self._show_status_message(f"Trigger '{trigger.name}' saved")

    # === Browse Handlers ===

    def _refresh_recordings(self):
        """Refresh the recordings list."""
        recordings = self._recording_manager.get_recordings()
        self._browse_tab.set_recordings(recordings)
        self._playback_tab.set_recordings(recordings)
        self._easy_mode_widget.set_recordings(recordings, self._trigger_store)
        # Load all saved triggers
        self._playback_tab.set_triggers(self._trigger_store.get_all())

    def _delete_recording(self, recording):
        """Delete a recording."""
        if self._recording_manager.delete_recording(recording):
            self._browse_tab.remove_recording(recording)
            self._show_status_message(f"Deleted: {recording.name}")
            self._refresh_recordings()

    def _rename_recording(self, recording, new_name: str):
        """Rename a recording."""
        if self._recording_manager.rename_recording(recording, new_name):
            self._show_status_message(f"Renamed to: {new_name}")
            self._refresh_recordings()

    def _load_for_playback(self, recording):
        """Load a recording for playback."""
        self._tabs.setCurrentWidget(self._playback_tab)
        # The playback tab will be updated when recordings refresh

    # === Motor Control ===

    def _emergency_stop(self):
        """Emergency stop."""
        self._ros_bridge.emergency_stop()
        self._stop_playback()
        self._stop_recording()

    def _set_zero(self):
        """Set current position as zero."""
        self._ros_bridge.set_zero()

    def _go_to_zero(self):
        """Go to zero position."""
        if not self._ros_bridge.joint_names:
            self._show_error_message("No joints detected - connect to driver first")
            return
        self._ros_bridge.go_to_zero()

    # === Mode Switching ===

    def _switch_to_easy(self):
        """Switch to easy mode."""
        if self._playing:
            self._stop_playback()
        self._easy_mode = True
        self._stack.setCurrentIndex(0)
        self._control_dock.setVisible(False)
        self._monitor_dock.setVisible(False)
        self._toolbar.setVisible(False)
        self.menuBar().setVisible(False)
        self._status_bar.setVisible(False)
        # Refresh so cards are up to date
        self._refresh_recordings()

    def _switch_to_advanced(self):
        """Switch to advanced mode."""
        self._easy_mode = False
        self._stack.setCurrentIndex(1)
        self._control_dock.setVisible(True)
        self._monitor_dock.setVisible(True)
        self._toolbar.setVisible(True)
        self.menuBar().setVisible(True)
        self._status_bar.setVisible(True)

    def _easy_play_recording(self, recording):
        """Handle play request from easy mode."""
        # Get matching triggers
        all_triggers = self._trigger_store.get_all()
        matching_triggers = [
            t for t in all_triggers
            if not t.recording_name or t.recording_name == recording.name
        ]
        if matching_triggers:
            config = PlaybackTriggerConfig(triggers=matching_triggers)
            self._ros_bridge.set_trigger_config(config)
        else:
            self._ros_bridge.clear_trigger_config()

        # Switch to position mode and start playback
        self._ros_bridge.set_mode("position")
        self._recording_manager.start_playback(recording)
        self._playing = True

    def _easy_toggle_pause(self):
        """Toggle pause from easy mode."""
        self._recording_manager.toggle_pause()
        paused = self._recording_manager.is_paused
        self._playback_tab.set_paused(paused)
        self._easy_mode_widget.set_paused(paused)

    # === UI Helpers ===

    def _show_status_message(self, message: str):
        """Show a status message."""
        self._message_label.setText(message)
        # Clear after 5 seconds
        QTimer.singleShot(5000, lambda: self._message_label.setText(""))

    def _show_error_message(self, message: str):
        """Show an error message."""
        self._message_label.setText(f"Error: {message}")
        self._message_label.setStyleSheet("color: #ef5350;")
        # Clear after 5 seconds
        QTimer.singleShot(5000, lambda: (
            self._message_label.setText(""),
            self._message_label.setStyleSheet("")
        ))

    def _show_about(self):
        """Show about dialog."""
        QMessageBox.about(
            self,
            "About Motor Recording Studio",
            "Motor Recording Studio v1.0.0\n\n"
            "PyQt6 desktop application for recording and playing back "
            "motor trajectories.\n\n"
            "Part of the MyActuator Python Driver package."
        )

    def closeEvent(self, event):
        """Handle window close."""
        # Stop any active operations
        if self._recording:
            self._stop_recording()
        if self._playing:
            self._stop_playback()

        # Stop ROS bridge
        self._ros_bridge.stop()

        event.accept()
