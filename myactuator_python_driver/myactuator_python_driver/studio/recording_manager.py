"""
Recording manager for Motor Recording Studio.

Handles reading and writing ROS 2 bags for motor trajectory recording and playback.
"""

import shutil
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional

from PyQt6.QtCore import QObject, QThread, pyqtSignal

import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState

try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    get_package_prefix = None

import rosbag2_py


@dataclass
class RecordingInfo:
    """Information about a recording."""
    name: str
    path: Path
    duration_sec: float
    frame_count: int
    created: datetime
    size_mb: float
    joint_names: List[str]


class RecordingManager(QObject):
    """
    Manages recording and playback of motor trajectories.

    Recordings are stored as ROS 2 bags in the workspace recordings directory.
    """

    # Signals
    recording_started = pyqtSignal(str)  # recording name
    recording_stopped = pyqtSignal(str, int)  # recording name, frame count
    recording_frame = pyqtSignal(int)  # frame count

    playback_started = pyqtSignal(str)  # recording name
    playback_stopped = pyqtSignal()
    playback_progress = pyqtSignal(float, float)  # current_sec, total_sec
    playback_frame = pyqtSignal(object)  # JointState msg

    error_occurred = pyqtSignal(str)

    SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0]

    def __init__(self, parent=None):
        super().__init__(parent)

        # Get recordings directory
        self.recordings_dir = self._get_recordings_dir()
        self.recordings_dir.mkdir(parents=True, exist_ok=True)

        # Recording state
        self._recording = False
        self._bag_writer: Optional[rosbag2_py.SequentialWriter] = None
        self._record_start_time = 0.0
        self._record_frame_count = 0
        self._current_recording_name = ""

        # Playback state
        self._playing = False
        self._paused = False
        self._loop = False
        self._speed_idx = 2  # 1.0x
        self._playback_thread: Optional[threading.Thread] = None

        # Clock for timestamps
        self._clock = None

        # Lock for thread safety
        self._lock = threading.Lock()

    def _get_recordings_dir(self) -> Path:
        """Get the recordings directory path."""
        if get_package_prefix is not None:
            try:
                pkg_prefix = Path(get_package_prefix('myactuator_python_driver'))
                workspace_dir = pkg_prefix.parent.parent
                return workspace_dir / "recordings"
            except Exception:
                pass
        return Path.home() / "motor_recordings"

    def set_clock(self, clock):
        """Set the ROS clock for timestamps."""
        self._clock = clock

    def get_recordings(self) -> List[RecordingInfo]:
        """Get list of all recordings."""
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
        return recordings

    def _load_recording_info(self, bag_path: Path) -> Optional[RecordingInfo]:
        """Load recording info from a bag directory."""
        try:
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions('', '')
            reader.open(storage_options, converter_options)

            metadata = reader.get_metadata()
            duration_ns = metadata.duration.nanoseconds
            duration_sec = duration_ns / 1e9
            frame_count = sum(
                t.message_count for t in metadata.topics_with_message_count
                if t.topic_metadata.name == '/joint_states')

            created = datetime.fromtimestamp(bag_path.stat().st_mtime)
            size_bytes = sum(f.stat().st_size for f in bag_path.iterdir() if f.is_file())
            size_mb = size_bytes / (1024 * 1024)

            # Get joint names from first message
            joint_names = []
            reader = rosbag2_py.SequentialReader()
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

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording

    def start_recording(self, name: Optional[str] = None) -> bool:
        """Start recording to a new bag."""
        if self._recording or self._playing:
            return False

        if name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"recording_{timestamp}"

        bag_path = self.recordings_dir / name

        try:
            storage_options = rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions('', '')

            self._bag_writer = rosbag2_py.SequentialWriter()
            self._bag_writer.open(storage_options, converter_options)

            topic_info = rosbag2_py.TopicMetadata(
                name='/joint_states',
                type='sensor_msgs/msg/JointState',
                serialization_format='cdr'
            )
            self._bag_writer.create_topic(topic_info)

            with self._lock:
                self._recording = True
                self._record_start_time = time.time()
                self._record_frame_count = 0
                self._current_recording_name = name

            self.recording_started.emit(name)
            return True

        except Exception as e:
            self.error_occurred.emit(f"Failed to start recording: {e}")
            return False

    def stop_recording(self) -> Optional[str]:
        """Stop recording and save bag."""
        if not self._recording:
            return None

        with self._lock:
            self._recording = False
            name = self._current_recording_name
            frame_count = self._record_frame_count

        if self._bag_writer:
            del self._bag_writer
            self._bag_writer = None

        self.recording_stopped.emit(name, frame_count)
        return name

    def record_frame(self, msg: JointState, timestamp_ns: int):
        """Record a single frame."""
        if not self._recording or self._bag_writer is None:
            return

        try:
            self._bag_writer.write(
                '/joint_states',
                serialize_message(msg),
                timestamp_ns
            )
            with self._lock:
                self._record_frame_count += 1
                count = self._record_frame_count

            self.recording_frame.emit(count)

        except Exception as e:
            self.error_occurred.emit(f"Record error: {e}")

    def get_record_duration(self) -> float:
        """Get current recording duration in seconds."""
        if not self._recording:
            return 0.0
        return time.time() - self._record_start_time

    def get_record_frame_count(self) -> int:
        """Get current recording frame count."""
        with self._lock:
            return self._record_frame_count

    # === Playback ===

    @property
    def is_playing(self) -> bool:
        """Check if currently playing."""
        return self._playing

    @property
    def is_paused(self) -> bool:
        """Check if playback is paused."""
        return self._paused

    @property
    def loop_enabled(self) -> bool:
        """Check if loop is enabled."""
        return self._loop

    @loop_enabled.setter
    def loop_enabled(self, value: bool):
        """Set loop enabled."""
        self._loop = value

    @property
    def speed(self) -> float:
        """Get current playback speed."""
        return self.SPEEDS[self._speed_idx]

    def speed_up(self):
        """Increase playback speed."""
        with self._lock:
            if self._speed_idx < len(self.SPEEDS) - 1:
                self._speed_idx += 1

    def speed_down(self):
        """Decrease playback speed."""
        with self._lock:
            if self._speed_idx > 0:
                self._speed_idx -= 1

    def start_playback(self, recording: RecordingInfo) -> bool:
        """Start playback of a recording."""
        if self._playing or self._recording:
            return False

        with self._lock:
            self._playing = True
            self._paused = False

        self._playback_thread = threading.Thread(
            target=self._playback_worker,
            args=(recording.path, recording.duration_sec),
            daemon=True
        )
        self._playback_thread.start()

        self.playback_started.emit(recording.name)
        return True

    def stop_playback(self):
        """Stop playback."""
        with self._lock:
            self._playing = False
            self._paused = False

        self.playback_stopped.emit()

    def toggle_pause(self):
        """Toggle playback pause."""
        with self._lock:
            if self._playing:
                self._paused = not self._paused

    def _playback_worker(self, bag_path: Path, total_duration: float):
        """Playback thread worker."""
        try:
            while True:
                reader = rosbag2_py.SequentialReader()
                storage_options = rosbag2_py.StorageOptions(
                    uri=str(bag_path), storage_id='sqlite3')
                converter_options = rosbag2_py.ConverterOptions('', '')
                reader.open(storage_options, converter_options)

                first_ts = None
                messages = []

                # Load all messages
                while reader.has_next():
                    topic, data, ts = reader.read_next()
                    if topic == '/joint_states':
                        if first_ts is None:
                            first_ts = ts
                        messages.append((ts - first_ts, data))

                if not messages:
                    break

                # Play messages
                start_time = time.time()
                for rel_ts_ns, data in messages:
                    if not self._playing:
                        return

                    # Handle pause
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

                    # Emit progress
                    current_sec = rel_ts_ns / 1e9
                    self.playback_progress.emit(current_sec, total_duration)

                    # Emit frame
                    msg = deserialize_message(data, JointState)
                    self.playback_frame.emit(msg)

                # Check loop
                with self._lock:
                    if not self._loop:
                        self._playing = False
                        self.playback_stopped.emit()
                        return

        except Exception as e:
            self.error_occurred.emit(f"Playback error: {e}")
            with self._lock:
                self._playing = False
            self.playback_stopped.emit()

    # === Recording Management ===

    def delete_recording(self, recording: RecordingInfo) -> bool:
        """Delete a recording."""
        try:
            shutil.rmtree(recording.path)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Delete failed: {e}")
            return False

    def rename_recording(self, recording: RecordingInfo, new_name: str) -> bool:
        """Rename a recording."""
        try:
            new_path = self.recordings_dir / new_name
            if new_path.exists():
                self.error_occurred.emit(f"Name already exists: {new_name}")
                return False

            recording.path.rename(new_path)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Rename failed: {e}")
            return False
