# Phase 2: UI Shell & Basic Calibration - Research

**Researched:** 2026-02-09
**Domain:** PyQt6 standalone application, form inputs with validation, live data display, ROS 2 integration
**Confidence:** HIGH

## Summary

Phase 2 wraps the Phase 1 CalibrationController with a PyQt6 GUI that lets the user configure and execute calibrations. The core technical challenge is minimal -- the controller already handles all logic (torque orchestration, position tracking, bag recording, safety). This phase is primarily UI widget composition and signal wiring, following patterns already established in Motor Studio's MainWindow, RecordTab, TriggerDialog, and ControlPanel.

The calibration app is a standalone QMainWindow (INFR-01), not a Motor Studio tab. It creates its own RosBridge and RecordingManager (same pattern as MainWindow), instantiates CalibrationController, and wires Qt signals from controller to UI labels/buttons. The UI is a single-purpose, single-window layout with no tabs, no docks, and no menus beyond an optional emergency stop shortcut. It needs: a joint dropdown (INPT-01), torque spin box (INPT-02), offset spin box (INPT-03), recording name input (INPT-04), Record/Stop button (CALB-01/CALB-05), live position and max position labels (DISP-01/DISP-02), direction indicator (DISP-03), and a connection status indicator (CONN-01/CONN-02).

**Primary recommendation:** Build CalibrationWindow as a QMainWindow that owns RosBridge, RecordingManager, and CalibrationController. Follow Motor Studio's main.py entry point pattern exactly (QApplication, dark theme, dependency checks). Wire all inputs to CalibrationConfig fields and all outputs to controller signals. Disable inputs during recording (INPT-05) by connecting to state_changed signal. Use QDoubleSpinBox for torque/offset (built-in range validation), QComboBox for joint selection (populated from RosBridge.joint_names), and QLineEdit for recording name. Register as `calibrator_app` console_scripts entry point (INFR-05).

## Standard Stack

### Core (already in project -- reuse only)
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| PyQt6 | >=6.4.0 | QMainWindow, QDoubleSpinBox, QComboBox, QLabel, signals/slots | Already used by Motor Studio; same dark theme |
| qt-material | >=2.14 | Dark theme (optional, falls back to Fusion palette) | Already used by Motor Studio main.py |
| rclpy | (ROS 2 distro) | ROS 2 Python client (via RosBridge) | Already used by driver_node and RosBridge |
| rosbag2_py | (ROS 2 distro) | Bag recording (via RecordingManager) | Already used by RecordingManager |

### Supporting (already in project)
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| Python math | stdlib | radians/degrees conversion for display | Converting internal radians to display degrees |
| Python dataclasses | stdlib | CalibrationConfig construction | Building config from UI inputs |
| Python datetime | stdlib | Auto-generated recording names | Timestamp default for recording name |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| QMainWindow | QDialog | QDialog is simpler but lacks status bar; QMainWindow matches Motor Studio pattern and allows future menu/toolbar |
| QDoubleSpinBox for torque | QLineEdit + QDoubleValidator | SpinBox provides increment buttons and enforces range without extra code |
| Inline torque range (hardcoded) | Motor-model-specific limits from DriverConfig | DriverConfig stores torque_constant but NOT rated_torque; motor model not available at runtime -- use conservative range |
| Single window layout | QDockWidget-based layout | Overkill for 5 inputs and 3 outputs; single QVBoxLayout is sufficient |

**Installation:** No new packages needed. All dependencies already satisfied by existing setup.py.

## Architecture Patterns

### Recommended Project Structure
```
myactuator_python_driver/
  calibrator/
    __init__.py              # Existing (Phase 1) - exports
    config.py                # Existing (Phase 1) - CalibrationConfig, CalibrationResult, CalibrationState
    controller.py            # Existing (Phase 1) - CalibrationController
    main.py                  # MODIFY - Add GUI entry point alongside existing CLI entry point
    window.py                # NEW - CalibrationWindow (QMainWindow)
  setup.py                   # MODIFY - Add calibrator_app console_scripts entry point
```

### Pattern 1: Window Owns Controller and ROS Components
**What:** CalibrationWindow.__init__ creates RosBridge, RecordingManager, and CalibrationController as instance attributes, exactly like MainWindow does in Motor Studio.

**When to use:** Every standalone app in this project that needs ROS communication.

**Why:** RosBridge manages rclpy lifecycle. Only one RosBridge per process. Window owns it so cleanup happens on closeEvent. Controller receives it via constructor injection.

**Example:**
```python
# Source: Existing pattern from studio/main_window.py lines 44-53
class CalibrationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Torque Threshold Calibrator")
        self.setMinimumSize(500, 600)

        # Core components (same ownership pattern as MainWindow)
        self._ros_bridge = RosBridge(self)
        self._recording_manager = RecordingManager(self)
        self._controller = CalibrationController(
            self._ros_bridge, self._recording_manager, parent=self
        )

        self._setup_ui()
        self._connect_signals()
        self._ros_bridge.start()
```

### Pattern 2: Input Disable During Recording via State Signal
**What:** Connect CalibrationController.state_changed to a method that enables/disables input widgets based on whether calibration is active.

**When to use:** INPT-05 requirement -- all configuration inputs must be disabled during active recording.

**Example:**
```python
# Source: Existing pattern from studio/widgets/record_tab.py set_recording()
def _on_state_changed(self, state: CalibrationState):
    """Enable/disable inputs based on calibration state."""
    is_active = state in (CalibrationState.RECORDING, CalibrationState.STOPPING)

    self._joint_combo.setEnabled(not is_active)
    self._torque_spin.setEnabled(not is_active)
    self._offset_spin.setEnabled(not is_active)
    self._name_edit.setEnabled(not is_active)

    # Toggle Record/Stop button text
    if state == CalibrationState.RECORDING:
        self._record_btn.setText("Stop")
        # Red style
    elif state == CalibrationState.IDLE:
        self._record_btn.setText("Record")
        # Normal style
```

### Pattern 3: Joint Dropdown Populated from RosBridge
**What:** Populate joint QComboBox when RosBridge reports new joint names via joint_state_received signal.

**When to use:** INPT-01 -- joint selector populated by connected motor joints.

**Critical detail:** RosBridge.joint_names is populated from the first /joint_states message. Before connection, it is an empty list. The combo box must be updated when joints first appear.

**Example:**
```python
# Source: Existing pattern from studio/dialogs/trigger_dialog.py _joint_combo
def _on_joint_state(self, msg):
    """Handle joint state for combo population and live display."""
    new_names = list(msg.name)
    if new_names and self._joint_combo.count() == 0:
        self._joint_combo.addItems(new_names)

    # Update live position display...
```

### Pattern 4: Live Position Display at ~20Hz
**What:** Connect CalibrationController.position_updated signal to label update. Controller already emits position at joint state rate (~50Hz from RosBridge). The UI label update is decoupled by Qt's signal queuing.

**When to use:** DISP-01, DISP-02 -- live position and running max position during recording.

**Critical detail:** Qt label setText is fast enough for 50Hz updates. No throttling needed. The controller already emits position_updated and max_position_updated signals.

**Example:**
```python
# Controller signals already exist (from Phase 1):
# position_updated = pyqtSignal(float)     # current position in radians
# max_position_updated = pyqtSignal(float) # running max position in radians

def _on_position_updated(self, position_rad: float):
    self._position_label.setText(
        f"{position_rad:.4f} rad ({math.degrees(position_rad):.2f} deg)"
    )

def _on_max_position_updated(self, max_pos_rad: float):
    self._max_position_label.setText(
        f"{max_pos_rad:.4f} rad ({math.degrees(max_pos_rad):.2f} deg)"
    )
```

### Pattern 5: Connection Status Indicator
**What:** Connect RosBridge.connection_status_changed to update a visible indicator and enable/disable the Record button.

**When to use:** Success criterion 5 -- prevents calibration when driver is not connected.

**Example:**
```python
# Source: Existing pattern from studio/main_window.py _on_connection_changed
def _on_connection_changed(self, connected: bool):
    self._connected = connected
    if connected:
        self._connection_label.setText("Connected")
        self._connection_label.setStyleSheet("color: #81c784;")
    else:
        self._connection_label.setText("Disconnected")
        self._connection_label.setStyleSheet("color: #ef5350;")

    # Disable Record button when disconnected
    # (CalibrationController.start_calibration also guards this,
    #  but UI should reflect it)
    self._update_record_button_state()
```

### Pattern 6: Torque Direction Indicator
**What:** Show which direction the joint will move based on the sign of the torque value. Update dynamically as user changes the torque spinbox.

**When to use:** DISP-03 requirement.

**Example:**
```python
def _on_torque_changed(self, value: float):
    if value > 0:
        self._direction_label.setText("Direction: Positive (increasing position)")
    elif value < 0:
        self._direction_label.setText("Direction: Negative (decreasing position)")
    else:
        self._direction_label.setText("Direction: None (zero torque)")
```

### Pattern 7: Entry Point Following Motor Studio Pattern
**What:** Create a `main_gui()` function in `calibrator/main.py` that follows the exact pattern from `studio/main.py`: suppress ROS logging, create QApplication, apply dark theme, check dependencies, create window, run event loop.

**When to use:** INFR-01 and INFR-05 -- standalone PyQt6 app with its own entry point.

**Critical detail:** The existing `main()` in `calibrator/main.py` is the CLI entry point. Add a separate `main_gui()` function for the GUI entry point, or create a new file. The entry point in setup.py will point to the GUI function.

### Anti-Patterns to Avoid

- **Putting calibration logic in the window class:** All logic is in CalibrationController (Phase 1). The window only wires signals and updates widgets. Never add ROS communication, state machine logic, or torque commands to the window.
- **Creating a second RosBridge node with the same name:** RosBridge defaults to node name 'motor_studio'. If both Motor Studio and the calibrator run simultaneously, ROS 2 will have duplicate node names. Use a different node name for the calibrator. This requires either parameterizing RosBridge's node name (minor change) or accepting that both apps cannot run simultaneously.
- **Polling for position in a QTimer:** The controller already pushes position via signals. Do not create a separate timer to read position -- this creates redundant work and potential inconsistency.
- **Modifying existing Phase 1 files for UI concerns:** Window-specific logic (label formatting, widget enable/disable) belongs in window.py. Do not add UI awareness to controller.py.
- **Using QThread for the calibration controller:** The controller is a QObject on the main thread. RosBridge handles the ROS background thread. The controller connects to RosBridge signals which are automatically queued by Qt.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| ROS 2 communication | Custom ROS node | Existing RosBridge | Already handles threading, connection monitoring |
| Bag recording | Custom writer | Existing RecordingManager | Already handles Humble/Jazzy compatibility |
| Dark theme | Custom palette | Copy from studio/main.py | Theme code is ~20 lines, already written |
| Torque range validation | Custom validator | QDoubleSpinBox.setRange() | Built-in enforcement, grays out invalid values |
| Input form layout | Manual pixel positioning | QFormLayout / QGroupBox | Standard Qt layout managers handle resize/DPI |
| Connection monitoring | Heartbeat timer | RosBridge.connection_status_changed | Already implements 2-second timeout |
| Joint name discovery | Custom ROS subscriber | RosBridge.joint_names + joint_state_received | Already populates from /joint_states |
| Recording name generation | Complex naming logic | CalibrationController auto-generates if empty | Already built in Phase 1 |

**Key insight:** The window is a thin layer of ~150-200 lines. Every data operation is delegated to existing components. The window's job is: (1) present widgets, (2) collect user input into CalibrationConfig, (3) display controller output signals.

## Common Pitfalls

### Pitfall 1: Joint Combo Box Empty at Startup
**What goes wrong:** App launches, user sees empty joint dropdown, cannot start calibration.
**Why it happens:** RosBridge.joint_names is empty until the first /joint_states message arrives. If the driver node is not running or connection takes time, the combo box stays empty.
**How to avoid:** Populate combo box from the joint_state_received signal handler, not at construction time. Show placeholder text like "(waiting for joints...)" in the combo box when empty. When the first joint state message arrives, clear the combo and add the joint names.
**Warning signs:** Combo box shows empty or "(waiting for joints...)" for more than a few seconds.

### Pitfall 2: Torque Range Not Based on Actual Motor Limits
**What goes wrong:** User enters a torque value that exceeds the motor's rated torque, potentially damaging the motor or gear reducer.
**Why it happens:** The DriverConfig stores `torque_constant` (Nm/A) per motor but NOT `rated_torque` (Nm). Motor model information is not propagated to the Python runtime. The C++ `actuator_constants.hpp` has rated_torque values but these are not exposed via pybind11 bindings.
**How to avoid:** Use a conservative fixed range for the torque spinbox (e.g., -50.0 to +50.0 Nm). The smallest motor (X4V2) has rated_torque=1.0 Nm and the largest (X15_400) has rated_torque=130.0 Nm. A range of -50 to +50 covers all common motors. Alternatively, torque validation could be added in a future enhancement by exposing rated_torque through the driver's config or ROS parameters. For now, rely on the existing TriggerDialog pattern which uses range(-50.0, 50.0).
**Warning signs:** Motor overheats, makes grinding noises, or draws excessive current during calibration.

### Pitfall 3: Record Button Enabled When Not Connected
**What goes wrong:** User clicks Record, CalibrationController.start_calibration returns False because RosBridge.is_connected is False. No useful feedback.
**Why it happens:** UI does not track connection state independently.
**How to avoid:** Track connection state in the window via connection_status_changed signal. Disable the Record button and show "Disconnected" when not connected. The controller also guards this, but the UI should prevent the click entirely.
**Warning signs:** User clicks Record and nothing happens, or sees an error message.

### Pitfall 4: Escape Key Emergency Stop Conflicts With Dialog Close
**What goes wrong:** If any dialog (e.g., confirmation) is open and user presses Escape, it closes the dialog instead of triggering emergency stop.
**Why it happens:** QDialog captures Escape for reject/close by default, consuming the key event before it reaches the main window shortcut.
**How to avoid:** For Phase 2, the calibration app has no dialogs (trigger preview is Phase 3). Emergency stop via Escape is a global shortcut on the QMainWindow. Bind it in the main window (same as Motor Studio's _estop_action pattern). If dialogs are added later, ensure the Escape shortcut uses Qt.ShortcutContext.ApplicationShortcut.
**Warning signs:** Pressing Escape during calibration does not stop torque.

### Pitfall 5: Unit Confusion Between Radians and Degrees
**What goes wrong:** User sees offset of "0.5" and thinks it is radians, when it is actually degrees. Or position displays radians when user expects degrees.
**Why it happens:** Internal representation is radians (CalibrationController), but user-facing inputs are degrees (INPT-03).
**How to avoid:** Always show units in labels ("deg" suffix on offset spinbox, "Nm" suffix on torque spinbox). Display positions in both radians and degrees (e.g., "0.4523 rad (25.92 deg)"). Use explicit conversion at the UI boundary: `math.radians(offset_spin.value())` when building CalibrationConfig.
**Warning signs:** Offset value seems unreasonably large (user thought it was degrees, it was treated as radians).

### Pitfall 6: Forgetting closeEvent Safety Cleanup
**What goes wrong:** User closes the window while calibration is active. Torque continues to be applied because the controller was not stopped.
**Why it happens:** Default QMainWindow.closeEvent just closes without cleanup.
**How to avoid:** Override closeEvent to call controller.emergency_stop() if active, then ros_bridge.stop(). Copy pattern from MainWindow.closeEvent.
**Warning signs:** Motors continue running after closing the calibration app.

## Code Examples

### Complete CalibrationWindow Layout (verified patterns from existing widgets)
```python
# Source: Follows patterns from record_tab.py, trigger_dialog.py, control_panel.py, main_window.py
def _setup_ui(self):
    central = QWidget()
    self.setCentralWidget(central)
    layout = QVBoxLayout(central)
    layout.setContentsMargins(16, 16, 16, 16)
    layout.setSpacing(16)

    # Connection status bar at top
    self._connection_label = QLabel("Disconnected")
    self._connection_label.setStyleSheet("color: #ef5350; font-weight: bold;")
    layout.addWidget(self._connection_label)

    # Configuration group (INPT-01 through INPT-04)
    config_group = QGroupBox("Configuration")
    config_layout = QFormLayout(config_group)

    self._joint_combo = QComboBox()
    config_layout.addRow("Joint:", self._joint_combo)

    self._torque_spin = QDoubleSpinBox()
    self._torque_spin.setRange(-50.0, 50.0)
    self._torque_spin.setDecimals(2)
    self._torque_spin.setSingleStep(0.1)
    self._torque_spin.setSuffix(" Nm")
    self._torque_spin.setValue(1.0)
    config_layout.addRow("Torque:", self._torque_spin)

    self._offset_spin = QDoubleSpinBox()
    self._offset_spin.setRange(0.0, 90.0)
    self._offset_spin.setDecimals(1)
    self._offset_spin.setSingleStep(0.1)
    self._offset_spin.setSuffix(" deg")
    self._offset_spin.setValue(0.5)
    config_layout.addRow("Offset:", self._offset_spin)

    self._name_edit = QLineEdit()
    self._name_edit.setPlaceholderText("Auto-generated if empty")
    config_layout.addRow("Recording:", self._name_edit)

    layout.addWidget(config_group)

    # Direction indicator (DISP-03)
    self._direction_label = QLabel("Direction: Positive")
    self._direction_label.setStyleSheet("color: #90caf9;")
    layout.addWidget(self._direction_label)

    # Live display group (DISP-01, DISP-02)
    display_group = QGroupBox("Live Data")
    display_layout = QFormLayout(display_group)

    self._position_label = QLabel("--")
    self._position_label.setStyleSheet("font-size: 18px; font-weight: bold;")
    display_layout.addRow("Position:", self._position_label)

    self._max_position_label = QLabel("--")
    self._max_position_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ff9800;")
    display_layout.addRow("Max Position:", self._max_position_label)

    layout.addWidget(display_group)

    # Record/Stop button (CALB-01, CALB-05)
    self._record_btn = QPushButton("Record")
    self._record_btn.setStyleSheet("""
        QPushButton { font-weight: bold; font-size: 16px; padding: 16px; min-height: 60px; }
    """)
    self._record_btn.clicked.connect(self._on_record_clicked)
    layout.addWidget(self._record_btn)

    # Emergency stop button (always visible)
    self._estop_btn = QPushButton("EMERGENCY STOP")
    self._estop_btn.setStyleSheet("""
        QPushButton {
            background-color: #d32f2f; color: white;
            font-weight: bold; font-size: 14px;
            border-radius: 8px; padding: 12px; min-height: 50px;
        }
        QPushButton:hover { background-color: #f44336; }
    """)
    self._estop_btn.clicked.connect(self._on_estop)
    layout.addWidget(self._estop_btn)
```

### Signal Wiring Pattern (from Phase 1 controller to Phase 2 window)
```python
# Source: Follows MainWindow._connect_signals pattern
def _connect_signals(self):
    # RosBridge -> Window
    self._ros_bridge.connection_status_changed.connect(self._on_connection_changed)
    self._ros_bridge.joint_state_received.connect(self._on_joint_state)

    # Controller -> Window (display updates)
    self._controller.state_changed.connect(self._on_state_changed)
    self._controller.position_updated.connect(self._on_position_updated)
    self._controller.max_position_updated.connect(self._on_max_position_updated)
    self._controller.error_occurred.connect(self._on_error)
    self._controller.calibration_complete.connect(self._on_calibration_complete)

    # Torque spinbox -> Direction indicator
    self._torque_spin.valueChanged.connect(self._on_torque_changed)
```

### Record/Stop Button Handler
```python
def _on_record_clicked(self):
    if self._controller.is_active:
        # Stop calibration
        self._controller.stop_calibration()
    else:
        # Build config from UI inputs
        config = CalibrationConfig(
            joint_name=self._joint_combo.currentText(),
            torque_nm=self._torque_spin.value(),
            offset_deg=self._offset_spin.value(),
            recording_name=self._name_edit.text().strip(),
        )
        if not self._controller.start_calibration(config):
            # Controller guards (not connected, already active)
            # will emit error_occurred signal
            pass
```

### GUI Entry Point (following studio/main.py exactly)
```python
# Source: Existing pattern from studio/main.py
def main_gui():
    """GUI entry point for calibration app."""
    import logging
    logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
    logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
    logging.getLogger('rcl').setLevel(logging.CRITICAL)

    try:
        from PyQt6.QtWidgets import QApplication, QMessageBox
        from PyQt6.QtCore import Qt
    except ImportError:
        print("Error: PyQt6 is required but not installed.")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setApplicationName("Torque Threshold Calibrator")

    # Apply dark theme (same pattern as studio/main.py)
    try:
        from qt_material import apply_stylesheet
        apply_stylesheet(app, theme='dark_blue.xml')
    except ImportError:
        # Fallback dark palette (copy from studio/main.py)
        ...

    # Dependency checks...
    # Create window...
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())
```

### setup.py Entry Point Registration
```python
# Source: Existing pattern in setup.py console_scripts
'console_scripts': [
    'driver_node = myactuator_python_driver.driver_node:main',
    'recorder_tui = myactuator_python_driver.recorder_tui:main',
    'setup_tui = myactuator_python_driver.setup_tui:main',
    'motor_studio = myactuator_python_driver.studio.main:main',
    'calibrator_cli = myactuator_python_driver.calibrator.main:main',
    'calibrator_app = myactuator_python_driver.calibrator.main:main_gui',  # NEW
],
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Custom ROS node per app | Reuse RosBridge QObject | Already established in Motor Studio | No new ROS nodes needed |
| QDialog for tool apps | QMainWindow with minimal layout | Established pattern for Motor Studio | Status bar for connection indicator |
| Position display in raw radians | Dual display (rad + deg) | Existing TriggerDialog pattern | User sees both units |

**Deprecated/outdated:**
- None relevant. All existing components are current and maintained.

## Open Questions

1. **Torque range validation without motor-specific limits**
   - What we know: DriverConfig stores `torque_constant` (Nm/A) per motor, but NOT `rated_torque` (Nm). The C++ actuator_constants.hpp has rated_torque values (1 Nm for X4V2, 130 Nm for X15_400) but these are not exposed through Python bindings.
   - What's unclear: Whether a fixed range (-50 to +50 Nm) is sufficient for all use cases, or whether motor-specific limits should be exposed.
   - Recommendation: Use the same range as TriggerDialog (-50 to +50 Nm) for Phase 2. This covers all common motors. Motor-specific validation is a v2 enhancement. The TriggerDialog has shipped with this range without issues.

2. **RosBridge node name conflict with Motor Studio**
   - What we know: RosBridgeWorker hardcodes `Node('motor_studio')`. If both apps run simultaneously, ROS 2 will have duplicate node names (both create a 'motor_studio' node).
   - What's unclear: Whether ROS 2 handles duplicate node names gracefully (it typically appends a suffix) or whether this causes errors.
   - Recommendation: For Phase 2, accept this limitation. Document in the UI ("Close Motor Studio before using calibrator"). A future enhancement could parameterize RosBridge's node name. This is not blocking because calibration and Motor Studio should not run simultaneously anyway (both publish mode commands).

3. **Where to put the GUI entry point function**
   - What we know: calibrator/main.py currently contains the CLI `main()` function. Phase 2 needs a GUI entry point.
   - What's unclear: Whether to add `main_gui()` to the same file or create a separate file.
   - Recommendation: Add `main_gui()` to calibrator/main.py. This is the simplest approach -- one file for all entry points, clear function naming. The CLI and GUI entry points share the same dependency checks and component creation pattern, differing only in QCoreApplication vs QApplication and adding the window.

## Sources

### Primary (HIGH confidence)
- **Existing codebase** -- All patterns derived from reading existing source code:
  - `studio/main.py` -- Application entry point, QApplication setup, dark theme, dependency checks
  - `studio/main_window.py` -- QMainWindow composition, signal wiring, RosBridge/RecordingManager ownership, connection status indicator, closeEvent cleanup
  - `studio/widgets/record_tab.py` -- Record button toggle, input disable during recording, duration display, LED status indicator
  - `studio/widgets/control_panel.py` -- QComboBox for mode selection, QDoubleSpinBox for torque limit, emergency stop button styling, section labels
  - `studio/dialogs/trigger_dialog.py` -- QFormLayout for input fields, QDoubleSpinBox(-50 to +50) for torque, QComboBox for joint selection, live position display at 20Hz, input validation
  - `studio/ros_bridge.py` -- RosBridge API (joint_names, is_connected, connection_status_changed, joint_state_received, set_mode, send_joint_command, emergency_stop)
  - `studio/recording_manager.py` -- RecordingManager API (start_recording, stop_recording, record_frame, recordings_dir)
  - `calibrator/controller.py` -- CalibrationController signals (state_changed, position_updated, max_position_updated, error_occurred, calibration_complete), methods (start_calibration, stop_calibration, emergency_stop, is_active property)
  - `calibrator/config.py` -- CalibrationConfig fields (joint_name, torque_nm, recording_name, offset_deg), CalibrationState enum
  - `setup.py` -- console_scripts entry point pattern
  - `studio/resources/style.css` -- Existing CSS classes for buttons, LEDs, status bar

### Secondary (MEDIUM confidence)
- PyQt6 official documentation (Context7 /websites/riverbankcomputing_static_pyqt6) -- Verified QDoubleSpinBox.setRange(), QComboBox signal patterns, QWidget.setEnabled(), QMainWindow layout structure

### Tertiary (LOW confidence)
- None. All critical findings verified against existing source code.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH -- all components already exist in the codebase, no new dependencies
- Architecture: HIGH -- every pattern directly copied from existing Motor Studio code (MainWindow, RecordTab, TriggerDialog, ControlPanel)
- Pitfalls: HIGH -- identified from actual code analysis (joint_names timing, torque range, connection state, closeEvent)

**Research date:** 2026-02-09
**Valid until:** 2026-03-09 (stable -- no external dependencies changing, all internal code)
