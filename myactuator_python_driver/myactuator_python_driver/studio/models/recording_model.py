"""
Recording list model for Motor Recording Studio.

Provides a QAbstractTableModel for displaying recordings in a QTableView.
"""

from typing import Any, List, Optional

from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex

from myactuator_python_driver.studio.recording_manager import RecordingInfo


class RecordingModel(QAbstractTableModel):
    """
    Table model for displaying recordings.

    Columns: Name, Duration, Frames, Size, Created
    """

    COLUMNS = ["Name", "Duration", "Frames", "Size", "Created"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._recordings: List[RecordingInfo] = []

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return len(self._recordings)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return len(self.COLUMNS)

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        if not index.isValid():
            return None

        row = index.row()
        col = index.column()

        if row < 0 or row >= len(self._recordings):
            return None

        recording = self._recordings[row]

        if role == Qt.ItemDataRole.DisplayRole:
            if col == 0:
                return recording.name
            elif col == 1:
                mins = int(recording.duration_sec // 60)
                secs = recording.duration_sec % 60
                if mins > 0:
                    return f"{mins}m {secs:.1f}s"
                return f"{secs:.1f}s"
            elif col == 2:
                return str(recording.frame_count)
            elif col == 3:
                if recording.size_mb >= 1.0:
                    return f"{recording.size_mb:.1f} MB"
                return f"{recording.size_mb * 1024:.0f} KB"
            elif col == 4:
                return recording.created.strftime("%Y-%m-%d %H:%M")

        elif role == Qt.ItemDataRole.TextAlignmentRole:
            if col == 0:
                return Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
            return Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter

        elif role == Qt.ItemDataRole.ToolTipRole:
            # Show joint names in tooltip
            if recording.joint_names:
                return f"Joints: {', '.join(recording.joint_names)}"

        elif role == Qt.ItemDataRole.UserRole:
            # Return the RecordingInfo object
            return recording

        return None

    def headerData(self, section: int, orientation: Qt.Orientation,
                   role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        if role == Qt.ItemDataRole.DisplayRole:
            if orientation == Qt.Orientation.Horizontal:
                if 0 <= section < len(self.COLUMNS):
                    return self.COLUMNS[section]
            else:
                return str(section + 1)
        return None

    def set_recordings(self, recordings: List[RecordingInfo]) -> None:
        """Set the list of recordings."""
        self.beginResetModel()
        self._recordings = recordings
        self.endResetModel()

    def get_recording(self, row: int) -> Optional[RecordingInfo]:
        """Get recording at row."""
        if 0 <= row < len(self._recordings):
            return self._recordings[row]
        return None

    def get_recording_by_index(self, index: QModelIndex) -> Optional[RecordingInfo]:
        """Get recording from model index."""
        if index.isValid():
            return self.get_recording(index.row())
        return None

    def remove_recording(self, recording: RecordingInfo) -> bool:
        """Remove a recording from the model."""
        try:
            row = self._recordings.index(recording)
            self.beginRemoveRows(QModelIndex(), row, row)
            self._recordings.pop(row)
            self.endRemoveRows()
            return True
        except ValueError:
            return False

    def refresh_recording(self, recording: RecordingInfo, new_info: RecordingInfo) -> bool:
        """Update a recording's info."""
        try:
            row = self._recordings.index(recording)
            self._recordings[row] = new_info
            self.dataChanged.emit(
                self.index(row, 0),
                self.index(row, len(self.COLUMNS) - 1)
            )
            return True
        except ValueError:
            return False

    def count(self) -> int:
        """Get number of recordings."""
        return len(self._recordings)
