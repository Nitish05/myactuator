#!/usr/bin/env python3
"""
Curses TUI for recording and playing back motor trajectories.

Features:
- Record /joint_states to ROS bags (mcap format)
- Playback with pause/resume, speed control, looping
- Browse, rename, delete recordings
- Motor control: set zero, go to zero, mode switching, enable/disable, e-stop
"""

import os
import sys
import time
import shutil
import curses
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional, List
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import serialize_message, deserialize_message

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

import rosbag2_py
from rosidl_runtime_py.utilities import get_message


@dataclass
class RecordingInfo:
    name: str
    path: Path
    duration_sec: float
    frame_count: int
    created: datetime
    size_mb: float
    joint_names: List[str]


class RecorderTUI:
    """Curses TUI for motor recording and playback."""

    SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0]
    MODES = ["position", "velocity", "torque", "free", "disabled"]

    def __init__(self):
        self.recordings_dir = Path.home() / "motor_recordings"
        self.recordings_dir.mkdir(parents=True, exist_ok=True)

        # State
        self._running = True
        self._recording = False
        self._playing = False
        self._paused = False
        self._loop = False
        self._speed_idx = 2  # 1.0x
        self._selected_idx = 0
        self._scroll_offset = 0
        self._recordings: List[RecordingInfo] = []
        self._current_mode = "unknown"
        self._motors_enabled = True
        self._record_start_time = 0.0
        self._record_frame_count = 0
        self._playback_progress = 0.0
        self._playback_duration = 0.0
        self._last_joint_state: Optional[JointState] = None
        self._status_message = ""
        self._error_message = ""
        self._show_help = False

        # ROS2
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._bag_writer: Optional[rosbag2_py.SequentialWriter] = None
        self._playback_thread: Optional[threading.Thread] = None

        # Curses
        self._stdscr = None

        # Locks
        self._lock = threading.Lock()

    def start(self):
        """Start the TUI and ROS2 node."""
        # Suppress ROS 2 logging to avoid messing up curses TUI
        import logging
        logging.getLogger('rosbag2_cpp').setLevel(logging.CRITICAL)
        logging.getLogger('rosbag2_storage_mcap').setLevel(logging.CRITICAL)
        logging.getLogger('rcl').setLevel(logging.CRITICAL)

        # Initialize ROS2
        rclpy.init()
        self._node = Node('recorder_tui')
        self._setup_ros()

        # Start ROS spinner
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._ros_thread.start()

        # Load recordings
        self._refresh_recordings()

        # Run TUI with curses wrapper, redirecting stderr to suppress ROS warnings
        try:
            # Redirect stderr to /dev/null during TUI
            self._old_stderr = sys.stderr
            sys.stderr = open(os.devnull, 'w')
            curses.wrapper(self._run_tui)
        finally:
            sys.stderr.close()
            sys.stderr = self._old_stderr
            self._shutdown()

    def _setup_ros(self):
        """Setup ROS2 publishers, subscribers, and service clients."""
        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self._joint_state_sub = self._node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10, callback_group=cb_group)
        self._mode_sub = self._node.create_subscription(
            String, '/motor_driver/mode', self._mode_cb, 10, callback_group=cb_group)

        # Publishers
        self._joint_ctrl_pub = self._node.create_publisher(JointState, '/joint_state_ctrl', 10)
        self._set_mode_pub = self._node.create_publisher(String, '/motor_driver/set_mode', 10)
        self._set_enabled_pub = self._node.create_publisher(Bool, '/motor_driver/set_enabled', 10)

        # Service clients
        self._set_zero_client = self._node.create_client(Trigger, '/motor_driver/set_zero')
        self._estop_client = self._node.create_client(Trigger, '/motor_driver/emergency_stop')

    def _spin_ros(self):
        """Spin ROS2 executor in background thread."""
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _joint_state_cb(self, msg: JointState):
        """Handle incoming joint states."""
        with self._lock:
            self._last_joint_state = msg
            if self._recording and self._bag_writer is not None:
                try:
                    self._bag_writer.write(
                        '/joint_states',
                        serialize_message(msg),
                        self._node.get_clock().now().nanoseconds
                    )
                    self._record_frame_count += 1
                except Exception as e:
                    self._error_message = f"Record error: {e}"

    def _mode_cb(self, msg: String):
        """Handle mode updates from driver."""
        with self._lock:
            self._current_mode = msg.data

    def _refresh_recordings(self):
        """Scan recordings directory and load metadata."""
        recordings = []
        for path in self.recordings_dir.iterdir():
            if path.is_dir() and (path / "metadata.yaml").exists():
                try:
                    info = self._load_recording_info(path)
                    if info:
                        recordings.append(info)
                except Exception:
                    pass

        recordings.sort(key=lambda r: r.created, reverse=True)
        with self._lock:
            self._recordings = recordings
            if self._selected_idx >= len(recordings):
                self._selected_idx = max(0, len(recordings) - 1)

    def _load_recording_info(self, bag_path: Path) -> Optional[RecordingInfo]:
        """Load recording info from a bag directory."""
        try:
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap')
            converter_options = rosbag2_py.ConverterOptions('', '')
            reader.open(storage_options, converter_options)

            metadata = reader.get_metadata()
            duration_ns = metadata.duration.nanoseconds
            duration_sec = duration_ns / 1e9
            frame_count = sum(t.message_count for t in metadata.topics_with_message_count
                              if t.topic_metadata.name == '/joint_states')

            created = datetime.fromtimestamp(bag_path.stat().st_mtime)
            size_bytes = sum(f.stat().st_size for f in bag_path.iterdir() if f.is_file())
            size_mb = size_bytes / (1024 * 1024)

            joint_names = []
            reader.open(storage_options, converter_options)
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic == '/joint_states':
                    msg = deserialize_message(data, JointState)
                    joint_names = list(msg.name)
                    break

            return RecordingInfo(
                name=bag_path.name,
                path=bag_path,
                duration_sec=duration_sec,
                frame_count=frame_count,
                created=created,
                size_mb=size_mb,
                joint_names=joint_names,
            )
        except Exception:
            return None

    # === Recording ===

    def _start_recording(self):
        """Start recording to a new bag."""
        if self._recording or self._playing:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"recording_{timestamp}"
        bag_path = self.recordings_dir / bag_name

        try:
            storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap')
            converter_options = rosbag2_py.ConverterOptions('', '')

            self._bag_writer = rosbag2_py.SequentialWriter()
            self._bag_writer.open(storage_options, converter_options)

            topic_info = rosbag2_py.TopicMetadata(
                id=0,
                name='/joint_states',
                type='sensor_msgs/msg/JointState',
                serialization_format='cdr'
            )
            self._bag_writer.create_topic(topic_info)

            self._set_mode("free")

            with self._lock:
                self._recording = True
                self._record_start_time = time.time()
                self._record_frame_count = 0
                self._status_message = f"Recording: {bag_name}"

        except Exception as e:
            self._error_message = f"Failed to start recording: {e}"

    def _stop_recording(self):
        """Stop recording and save bag."""
        if not self._recording:
            return

        with self._lock:
            self._recording = False

        if self._bag_writer:
            del self._bag_writer
            self._bag_writer = None

        self._set_mode("position")
        self._refresh_recordings()
        self._status_message = f"Recording saved ({self._record_frame_count} frames)"

    # === Playback ===

    def _start_playback(self):
        """Start playback of selected recording."""
        if self._playing or self._recording:
            return

        with self._lock:
            if not self._recordings or self._selected_idx >= len(self._recordings):
                self._error_message = "No recording selected"
                return
            recording = self._recordings[self._selected_idx]

        self._set_mode("position")

        with self._lock:
            self._playing = True
            self._paused = False
            self._playback_progress = 0.0
            self._playback_duration = recording.duration_sec

        self._playback_thread = threading.Thread(
            target=self._playback_worker, args=(recording.path,), daemon=True)
        self._playback_thread.start()

    def _stop_playback(self):
        """Stop playback."""
        with self._lock:
            self._playing = False
            self._paused = False

    def _toggle_pause(self):
        """Toggle playback pause."""
        with self._lock:
            if self._playing:
                self._paused = not self._paused

    def _playback_worker(self, bag_path: Path):
        """Playback thread worker."""
        try:
            while True:
                reader = rosbag2_py.SequentialReader()
                storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap')
                converter_options = rosbag2_py.ConverterOptions('', '')
                reader.open(storage_options, converter_options)

                first_ts = None
                messages = []

                while reader.has_next():
                    topic, data, ts = reader.read_next()
                    if topic == '/joint_states':
                        if first_ts is None:
                            first_ts = ts
                        messages.append((ts - first_ts, data))

                if not messages:
                    break

                start_time = time.time()
                for rel_ts_ns, data in messages:
                    if not self._playing:
                        return

                    pause_start = None
                    while self._paused and self._playing:
                        if pause_start is None:
                            pause_start = time.time()
                        time.sleep(0.05)
                    if pause_start:
                        start_time += time.time() - pause_start

                    with self._lock:
                        speed = self.SPEEDS[self._speed_idx]
                    target_time = start_time + (rel_ts_ns / 1e9) / speed

                    now = time.time()
                    if target_time > now:
                        time.sleep(target_time - now)

                    with self._lock:
                        self._playback_progress = (rel_ts_ns / 1e9)

                    msg = deserialize_message(data, JointState)
                    msg.header.stamp = self._node.get_clock().now().to_msg()
                    self._joint_ctrl_pub.publish(msg)

                with self._lock:
                    if not self._loop:
                        self._playing = False
                        return

        except Exception as e:
            with self._lock:
                self._error_message = f"Playback error: {e}"
                self._playing = False

    # === Motor Control ===

    def _set_mode(self, mode: str):
        """Set motor mode."""
        msg = String()
        msg.data = mode
        self._set_mode_pub.publish(msg)
        self._status_message = f"Mode: {mode}"

    def _cycle_mode(self):
        """Cycle through modes."""
        with self._lock:
            current = self._current_mode
        try:
            idx = self.MODES.index(current)
            next_idx = (idx + 1) % len(self.MODES)
        except ValueError:
            next_idx = 0
        self._set_mode(self.MODES[next_idx])

    def _toggle_enabled(self):
        """Toggle motors enabled/disabled."""
        with self._lock:
            self._motors_enabled = not self._motors_enabled
            enabled = self._motors_enabled
        msg = Bool()
        msg.data = enabled
        self._set_enabled_pub.publish(msg)
        self._status_message = f"Motors {'enabled' if enabled else 'disabled'}"

    def _set_zero(self):
        """Call set_zero service."""
        if not self._set_zero_client.wait_for_service(timeout_sec=0.5):
            self._error_message = "set_zero service not available"
            return
        self._set_zero_client.call_async(Trigger.Request())
        self._status_message = "Set zero requested"

    def _go_to_zero(self):
        """Command motors to position zero."""
        with self._lock:
            if self._last_joint_state is None:
                self._error_message = "No joint state received"
                return
            names = list(self._last_joint_state.name)

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = names
        msg.position = [0.0] * len(names)
        self._joint_ctrl_pub.publish(msg)
        self._status_message = "Going to zero"

    def _emergency_stop(self):
        """Call emergency stop service."""
        if not self._estop_client.wait_for_service(timeout_sec=0.5):
            self._error_message = "emergency_stop service not available"
            return
        self._estop_client.call_async(Trigger.Request())
        self._status_message = "EMERGENCY STOP!"
        self._stop_playback()
        self._stop_recording()

    # === Recording Management ===

    def _delete_selected(self):
        """Delete selected recording."""
        with self._lock:
            if not self._recordings or self._selected_idx >= len(self._recordings):
                return
            recording = self._recordings[self._selected_idx]

        try:
            shutil.rmtree(recording.path)
            self._status_message = f"Deleted: {recording.name}"
            self._refresh_recordings()
        except Exception as e:
            self._error_message = f"Delete failed: {e}"

    def _rename_selected(self, stdscr):
        """Rename selected recording."""
        with self._lock:
            if not self._recordings or self._selected_idx >= len(self._recordings):
                return
            recording = self._recordings[self._selected_idx]

        # Show rename prompt
        h, w = stdscr.getmaxyx()
        curses.echo()
        curses.curs_set(1)
        stdscr.addstr(h - 2, 0, "New name (Enter to cancel): ")
        stdscr.clrtoeol()
        stdscr.refresh()

        try:
            new_name = stdscr.getstr(h - 2, 28, 50).decode('utf-8').strip()
            if new_name:
                new_path = self.recordings_dir / new_name
                if new_path.exists():
                    self._error_message = f"Name already exists: {new_name}"
                else:
                    recording.path.rename(new_path)
                    self._status_message = f"Renamed to: {new_name}"
                    self._refresh_recordings()
        except Exception as e:
            self._error_message = f"Rename failed: {e}"
        finally:
            curses.noecho()
            curses.curs_set(0)

    # === TUI ===

    def _run_tui(self, stdscr):
        """Main TUI loop with curses."""
        self._stdscr = stdscr

        # Setup curses
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.timeout(100)  # 100ms timeout for getch

        # Setup colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_CYAN, -1)     # Title
        curses.init_pair(2, curses.COLOR_GREEN, -1)    # OK/Position mode
        curses.init_pair(3, curses.COLOR_YELLOW, -1)   # Warning/Free mode
        curses.init_pair(4, curses.COLOR_RED, -1)      # Error/Recording
        curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_WHITE)  # Selected
        curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLUE)   # Header

        while self._running:
            try:
                self._draw(stdscr)
                key = stdscr.getch()
                if key != -1:
                    self._handle_key(key, stdscr)
            except curses.error:
                pass

    def _draw(self, stdscr):
        """Draw the entire TUI."""
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        if self._show_help:
            self._draw_help(stdscr, h, w)
        else:
            self._draw_header(stdscr, w)
            self._draw_recordings(stdscr, h, w)
            self._draw_state(stdscr, h, w)
            self._draw_status(stdscr, h, w)
            self._draw_help_bar(stdscr, h, w)

        stdscr.refresh()

    def _draw_header(self, stdscr, w):
        """Draw header bar."""
        header = " Motor Recorder TUI"

        # Mode with color
        with self._lock:
            mode = self._current_mode
        if mode == "position":
            mode_color = curses.color_pair(2)
        elif mode == "free":
            mode_color = curses.color_pair(3)
        elif mode in ("disabled", "torque"):
            mode_color = curses.color_pair(4)
        else:
            mode_color = curses.A_NORMAL

        # Joint positions
        pos_str = ""
        with self._lock:
            if self._last_joint_state:
                positions = [f"{n}:{p:.2f}" for n, p in
                             zip(self._last_joint_state.name[:4], self._last_joint_state.position[:4])]
                pos_str = " | " + " ".join(positions)

        try:
            stdscr.addstr(0, 0, " " * w, curses.color_pair(6))
            stdscr.addstr(0, 0, header, curses.color_pair(6) | curses.A_BOLD)
            stdscr.addstr(0, len(header), f" | Mode: ", curses.color_pair(6))
            stdscr.addstr(mode, mode_color | curses.A_BOLD)
            if pos_str and len(header) + 10 + len(mode) + len(pos_str) < w:
                stdscr.addstr(pos_str, curses.color_pair(6))
        except curses.error:
            pass

    def _draw_recordings(self, stdscr, h, w):
        """Draw recordings table."""
        # Calculate available space
        table_width = w * 2 // 3
        table_height = h - 5  # header(1) + status(2) + help(2)
        max_rows = table_height - 3  # title(1) + header(1) + border(1)

        # Draw box
        try:
            stdscr.addstr(2, 0, "+" + "-" * (table_width - 2) + "+")
            title = " Recordings "
            with self._lock:
                total = len(self._recordings)
            if total > max_rows:
                start = self._scroll_offset + 1
                end = min(self._scroll_offset + max_rows, total)
                title = f" Recordings [{start}-{end}/{total}] "
            stdscr.addstr(2, 2, title, curses.A_BOLD)

            # Column headers
            stdscr.addstr(3, 1, f"{'#':>3} {'Name':<25} {'Dur':>8} {'Frames':>7} {'Size':>7} {'Created':<16}")

            # Draw recordings
            with self._lock:
                recordings = self._recordings
                selected = self._selected_idx
                scroll = self._scroll_offset

                # Adjust scroll to keep selection visible
                if selected < scroll:
                    self._scroll_offset = selected
                    scroll = selected
                elif selected >= scroll + max_rows:
                    self._scroll_offset = selected - max_rows + 1
                    scroll = self._scroll_offset

                for i, idx in enumerate(range(scroll, min(scroll + max_rows, len(recordings)))):
                    if i + 4 >= h - 3:
                        break
                    rec = recordings[idx]
                    line = f"{idx + 1:>3} {rec.name[:25]:<25} {rec.duration_sec:>7.1f}s {rec.frame_count:>7} {rec.size_mb:>6.1f}M {rec.created.strftime('%m-%d %H:%M'):<16}"

                    if idx == selected:
                        stdscr.addstr(4 + i, 1, line[:table_width - 2], curses.color_pair(5))
                    else:
                        stdscr.addstr(4 + i, 1, line[:table_width - 2])

            # Bottom border
            stdscr.addstr(min(4 + max_rows, h - 4), 0, "+" + "-" * (table_width - 2) + "+")

        except curses.error:
            pass

    def _draw_state(self, stdscr, h, w):
        """Draw state panel."""
        state_x = w * 2 // 3 + 2
        state_width = w - state_x - 1

        try:
            stdscr.addstr(2, state_x, "[ State ]", curses.A_BOLD)

            row = 4
            if self._recording:
                elapsed = time.time() - self._record_start_time
                stdscr.addstr(row, state_x, "* RECORDING", curses.color_pair(4) | curses.A_BOLD)
                row += 1
                stdscr.addstr(row, state_x, f"  Time: {elapsed:.1f}s")
                row += 1
                stdscr.addstr(row, state_x, f"  Frames: {self._record_frame_count}")
                row += 1
            elif self._playing:
                if self._paused:
                    stdscr.addstr(row, state_x, "|| PAUSED", curses.color_pair(3) | curses.A_BOLD)
                else:
                    stdscr.addstr(row, state_x, "> PLAYING", curses.color_pair(2) | curses.A_BOLD)
                row += 1
                # Progress bar
                bar_width = state_width - 4
                if self._playback_duration > 0 and bar_width > 5:
                    pct = min(1.0, self._playback_progress / self._playback_duration)
                    filled = int(bar_width * pct)
                    bar = "[" + "#" * filled + "-" * (bar_width - filled) + "]"
                    stdscr.addstr(row, state_x, bar)
                row += 1
                stdscr.addstr(row, state_x, f"  {self._playback_progress:.1f}s / {self._playback_duration:.1f}s")
                row += 1
                stdscr.addstr(row, state_x, f"  Speed: {self.SPEEDS[self._speed_idx]}x")
                row += 1
                loop_str = "ON" if self._loop else "OFF"
                stdscr.addstr(row, state_x, f"  Loop: {loop_str}")
                row += 1
            else:
                stdscr.addstr(row, state_x, "IDLE", curses.A_DIM)
                row += 1

            row += 1
            stdscr.addstr(row, state_x, f"Speed: {self.SPEEDS[self._speed_idx]}x")
            row += 1
            loop_color = curses.color_pair(2) if self._loop else curses.A_DIM
            stdscr.addstr(row, state_x, f"Loop: {'ON' if self._loop else 'OFF'}", loop_color)

        except curses.error:
            pass

    def _draw_status(self, stdscr, h, w):
        """Draw status bar."""
        with self._lock:
            status = self._status_message
            error = self._error_message
            self._error_message = ""

        try:
            if error:
                stdscr.addstr(h - 3, 0, f" ERROR: {error}"[:w - 1], curses.color_pair(4))
            elif status:
                stdscr.addstr(h - 3, 0, f" {status}"[:w - 1], curses.color_pair(2))
            else:
                stdscr.addstr(h - 3, 0, " Ready", curses.A_DIM)
        except curses.error:
            pass

    def _draw_help_bar(self, stdscr, h, w):
        """Draw help bar at bottom."""
        help_text = " r:Rec  p:Play  Space:Pause  []:Speed  l:Loop  z:Zero  0:GoZero  m:Mode  e:Enable  Esc:E-Stop  d:Del  n:Rename  ?:Help  q:Quit "

        try:
            stdscr.addstr(h - 2, 0, " " * w, curses.color_pair(5))
            stdscr.addstr(h - 2, 0, help_text[:w], curses.color_pair(5))
        except curses.error:
            pass

    def _draw_help(self, stdscr, h, w):
        """Draw help screen."""
        help_lines = [
            "",
            "  KEYBOARD SHORTCUTS",
            "  ==================",
            "",
            "  Recording & Playback:",
            "    r       Start/Stop recording",
            "    p       Start/Stop playback",
            "    SPACE   Pause/Resume playback",
            "    [ / ]   Decrease/Increase speed",
            "    l       Toggle loop",
            "",
            "  Motor Control:",
            "    z       Set zero (current pos = 0)",
            "    0       Go to zero position",
            "    m       Cycle mode",
            "    e       Toggle enable/disable",
            "    ESC     Emergency stop",
            "",
            "  Recording Management:",
            "    UP/DOWN Select recording",
            "    PgUp/Dn Page through recordings",
            "    d       Delete selected",
            "    n       Rename selected",
            "    F5      Refresh list",
            "",
            "  General:",
            "    ?       Toggle this help",
            "    q       Quit",
            "",
            "  Press ? to return...",
        ]

        try:
            stdscr.addstr(0, 0, " " * w, curses.color_pair(6))
            stdscr.addstr(0, 0, " Help", curses.color_pair(6) | curses.A_BOLD)

            for i, line in enumerate(help_lines):
                if i + 2 < h:
                    stdscr.addstr(i + 2, 0, line[:w - 1])
        except curses.error:
            pass

    def _handle_key(self, key, stdscr):
        """Handle keyboard input."""
        h, _ = stdscr.getmaxyx()
        max_rows = h - 8

        if key == ord('q'):
            self._running = False
        elif key == ord('r'):
            if self._recording:
                self._stop_recording()
            else:
                self._start_recording()
        elif key == ord('p'):
            if self._playing:
                self._stop_playback()
            else:
                self._start_playback()
        elif key == ord(' '):
            self._toggle_pause()
        elif key == ord('['):
            with self._lock:
                if self._speed_idx > 0:
                    self._speed_idx -= 1
        elif key == ord(']'):
            with self._lock:
                if self._speed_idx < len(self.SPEEDS) - 1:
                    self._speed_idx += 1
        elif key == ord('l'):
            with self._lock:
                self._loop = not self._loop
        elif key == ord('z'):
            self._set_zero()
        elif key == ord('0'):
            self._go_to_zero()
        elif key == ord('m'):
            self._cycle_mode()
        elif key == ord('e'):
            self._toggle_enabled()
        elif key == ord('d'):
            self._delete_selected()
        elif key == ord('n'):
            self._rename_selected(stdscr)
        elif key == ord('?'):
            self._show_help = not self._show_help
        elif key == 27:  # ESC
            self._emergency_stop()
        elif key == curses.KEY_UP:
            with self._lock:
                if self._selected_idx > 0:
                    self._selected_idx -= 1
        elif key == curses.KEY_DOWN:
            with self._lock:
                if self._selected_idx < len(self._recordings) - 1:
                    self._selected_idx += 1
        elif key == curses.KEY_PPAGE:  # Page Up
            with self._lock:
                self._selected_idx = max(0, self._selected_idx - max_rows)
        elif key == curses.KEY_NPAGE:  # Page Down
            with self._lock:
                self._selected_idx = min(len(self._recordings) - 1, self._selected_idx + max_rows)
        elif key == curses.KEY_F5 or key == 18:  # F5 or Ctrl+R
            self._refresh_recordings()

    def _shutdown(self):
        """Cleanup on exit."""
        self._running = False
        self._stop_playback()
        self._stop_recording()

        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    tui = RecorderTUI()
    tui.start()


if __name__ == '__main__':
    main()
