"""
Joint state table model for Motor Recording Studio.

Provides a QAbstractTableModel for displaying joint states in a QTableView.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from PyQt6.QtCore import Qt, QAbstractTableModel, QModelIndex
from PyQt6.QtGui import QColor


@dataclass
class JointData:
    """Data for a single joint."""
    name: str = ""
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    temperature: float = 0.0


class JointStateModel(QAbstractTableModel):
    """
    Table model for displaying joint states.

    Columns: Name, Position (rad), Velocity (rad/s), Effort (Nm), Temp (C)
    """

    COLUMNS = ["Joint", "Position (rad)", "Velocity (rad/s)", "Effort (Nm)", "Temp (C)"]

    # Temperature thresholds for coloring
    TEMP_WARNING = 50.0
    TEMP_DANGER = 70.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._joints: List[JointData] = []
        self._joint_map: Dict[str, int] = {}

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return len(self._joints)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return len(self.COLUMNS)

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        if not index.isValid():
            return None

        row = index.row()
        col = index.column()

        if row < 0 or row >= len(self._joints):
            return None

        joint = self._joints[row]

        if role == Qt.ItemDataRole.DisplayRole:
            if col == 0:
                return joint.name
            elif col == 1:
                return f"{joint.position:.4f}"
            elif col == 2:
                return f"{joint.velocity:.4f}"
            elif col == 3:
                return f"{joint.effort:.3f}"
            elif col == 4:
                return f"{joint.temperature:.1f}"

        elif role == Qt.ItemDataRole.TextAlignmentRole:
            if col == 0:
                return Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
            return Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter

        elif role == Qt.ItemDataRole.ForegroundRole:
            # Color temperature column based on value
            if col == 4:
                if joint.temperature >= self.TEMP_DANGER:
                    return QColor("#f44336")  # Red
                elif joint.temperature >= self.TEMP_WARNING:
                    return QColor("#ff9800")  # Orange
                return QColor("#81c784")  # Green

        elif role == Qt.ItemDataRole.BackgroundRole:
            # Subtle background for temperature warnings
            if col == 4:
                if joint.temperature >= self.TEMP_DANGER:
                    return QColor(244, 67, 54, 50)  # Red with alpha
                elif joint.temperature >= self.TEMP_WARNING:
                    return QColor(255, 152, 0, 50)  # Orange with alpha

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

    def update_joint_state(self, msg) -> None:
        """
        Update joint states from a JointState message.

        Args:
            msg: sensor_msgs/JointState message
        """
        # Check if joints changed
        new_names = list(msg.name)
        if new_names != [j.name for j in self._joints]:
            # Joints changed, rebuild list
            self.beginResetModel()
            self._joints = [JointData(name=n) for n in new_names]
            self._joint_map = {n: i for i, n in enumerate(new_names)}
            self.endResetModel()

        # Update values
        for i, name in enumerate(msg.name):
            if i < len(self._joints):
                joint = self._joints[i]
                if i < len(msg.position):
                    joint.position = msg.position[i]
                if i < len(msg.velocity):
                    joint.velocity = msg.velocity[i]
                if i < len(msg.effort):
                    joint.effort = msg.effort[i]

        # Emit dataChanged for all rows
        if self._joints:
            self.dataChanged.emit(
                self.index(0, 0),
                self.index(len(self._joints) - 1, len(self.COLUMNS) - 1)
            )

    def update_temperatures(self, temps: Dict[str, float]) -> None:
        """
        Update temperature values from motor status.

        Args:
            temps: Dict mapping joint name to temperature in Celsius
        """
        for name, temp in temps.items():
            if name in self._joint_map:
                idx = self._joint_map[name]
                self._joints[idx].temperature = temp

        # Emit dataChanged for temperature column
        if self._joints:
            self.dataChanged.emit(
                self.index(0, 4),
                self.index(len(self._joints) - 1, 4)
            )

    def get_joint_names(self) -> List[str]:
        """Get list of joint names."""
        return [j.name for j in self._joints]

    def get_joint_positions(self) -> Dict[str, float]:
        """Get dict of joint positions."""
        return {j.name: j.position for j in self._joints}

    def get_joint(self, name: str) -> Optional[JointData]:
        """Get joint data by name."""
        if name in self._joint_map:
            return self._joints[self._joint_map[name]]
        return None
