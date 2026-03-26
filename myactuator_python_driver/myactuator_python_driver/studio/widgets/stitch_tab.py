"""
Stitch tab widget for Motor Recording Studio.

Allows combining multiple recordings into a single stitched recording.
"""

from typing import List, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QListWidget, QListWidgetItem, QGroupBox,
    QProgressBar, QAbstractItemView,
)

from myactuator_python_driver.studio.recording_manager import RecordingInfo


class StitchTab(QWidget):
    """Tab for stitching multiple recordings into one."""

    stitch_requested = Signal(list, str)  # [RecordingInfo], output_name
    refresh_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._recordings: List[RecordingInfo] = []
        self._queue: List[RecordingInfo] = []
        self._setup_ui()
        self._connect_signals()
        self._update_state()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(12)

        # === Top: Two-panel layout ===
        panels = QHBoxLayout()
        panels.setSpacing(8)

        # Left panel: Available recordings
        avail_group = QGroupBox("Available Recordings")
        avail_layout = QVBoxLayout(avail_group)
        avail_layout.setContentsMargins(8, 12, 8, 8)

        self._avail_list = QListWidget()
        self._avail_list.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        avail_layout.addWidget(self._avail_list)

        panels.addWidget(avail_group, 1)

        # Center buttons
        btn_col = QVBoxLayout()
        btn_col.setSpacing(8)
        btn_col.addStretch()

        self._add_btn = QPushButton("Add >>")
        self._add_btn.setFixedWidth(90)
        btn_col.addWidget(self._add_btn)

        self._remove_btn = QPushButton("<< Remove")
        self._remove_btn.setFixedWidth(90)
        btn_col.addWidget(self._remove_btn)

        btn_col.addStretch()
        panels.addLayout(btn_col)

        # Right panel: Stitch queue
        queue_group = QGroupBox("Stitch Queue (playback order)")
        queue_layout = QVBoxLayout(queue_group)
        queue_layout.setContentsMargins(8, 12, 8, 8)

        self._queue_list = QListWidget()
        self._queue_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._queue_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        queue_layout.addWidget(self._queue_list)

        # Reorder buttons
        order_row = QHBoxLayout()
        order_row.setSpacing(8)

        self._up_btn = QPushButton("Up")
        self._up_btn.setFixedWidth(60)
        order_row.addWidget(self._up_btn)

        self._down_btn = QPushButton("Down")
        self._down_btn.setFixedWidth(60)
        order_row.addWidget(self._down_btn)

        order_row.addStretch()

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setFixedWidth(60)
        order_row.addWidget(self._clear_btn)

        queue_layout.addLayout(order_row)
        panels.addWidget(queue_group, 1)

        # === Top: Output section ===
        output_group = QGroupBox("Output")
        output_layout = QVBoxLayout(output_group)
        output_layout.setContentsMargins(8, 12, 8, 8)
        output_layout.setSpacing(8)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Auto-generated if empty")
        name_row.addWidget(self._name_edit)

        self._create_btn = QPushButton("Create Stitched Recording")
        self._create_btn.setEnabled(False)
        self._create_btn.setStyleSheet(
            "QPushButton { font-weight: bold; font-size: 14px; padding: 12px; }"
        )
        name_row.addWidget(self._create_btn)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.setFixedWidth(70)
        refresh_btn.clicked.connect(self.refresh_requested.emit)
        name_row.addWidget(refresh_btn)

        output_layout.addLayout(name_row)

        self._warning_label = QLabel("")
        self._warning_label.setStyleSheet("color: #ff9800; font-weight: bold;")
        self._warning_label.setVisible(False)
        output_layout.addWidget(self._warning_label)

        self._summary_label = QLabel("Add at least 2 recordings to stitch")
        self._summary_label.setStyleSheet("color: #757575;")
        output_layout.addWidget(self._summary_label)

        self._progress = QProgressBar()
        self._progress.setVisible(False)
        output_layout.addWidget(self._progress)

        main_layout.addWidget(output_group)

        main_layout.addLayout(panels, 1)

    def _connect_signals(self):
        self._add_btn.clicked.connect(self._add_to_queue)
        self._remove_btn.clicked.connect(self._remove_from_queue)
        self._up_btn.clicked.connect(self._move_up)
        self._down_btn.clicked.connect(self._move_down)
        self._clear_btn.clicked.connect(self._clear_queue)
        self._create_btn.clicked.connect(self._on_create)
        self._queue_list.model().rowsMoved.connect(self._sync_queue_from_list)

    def set_recordings(self, recordings: List[RecordingInfo]):
        """Update available recordings list."""
        self._recordings = list(recordings)
        self._avail_list.clear()
        for rec in recordings:
            item = QListWidgetItem(f"{rec.name}  ({rec.duration_sec:.1f}s, {rec.frame_count} frames)")
            item.setData(Qt.ItemDataRole.UserRole, rec)
            self._avail_list.addItem(item)

    def set_stitching(self, active: bool):
        """Enable/disable UI during stitching."""
        self._create_btn.setEnabled(not active and len(self._queue) >= 2)
        self._add_btn.setEnabled(not active)
        self._remove_btn.setEnabled(not active)
        self._up_btn.setEnabled(not active)
        self._down_btn.setEnabled(not active)
        self._clear_btn.setEnabled(not active)
        self._name_edit.setEnabled(not active)
        self._progress.setVisible(active)
        if not active:
            self._progress.setValue(0)

    def set_stitch_progress(self, current: int, total: int):
        """Update stitch progress bar."""
        self._progress.setMaximum(total)
        self._progress.setValue(current)

    def _add_to_queue(self):
        """Add selected recordings to the stitch queue."""
        for item in self._avail_list.selectedItems():
            rec = item.data(Qt.ItemDataRole.UserRole)
            if rec:
                self._queue.append(rec)
                queue_item = QListWidgetItem(
                    f"{rec.name}  ({rec.duration_sec:.1f}s)")
                queue_item.setData(Qt.ItemDataRole.UserRole, rec)
                self._queue_list.addItem(queue_item)
        self._update_state()

    def _remove_from_queue(self):
        """Remove selected item from stitch queue."""
        row = self._queue_list.currentRow()
        if row >= 0:
            self._queue_list.takeItem(row)
            self._queue.pop(row)
            self._update_state()

    def _move_up(self):
        """Move selected queue item up."""
        row = self._queue_list.currentRow()
        if row > 0:
            item = self._queue_list.takeItem(row)
            self._queue_list.insertItem(row - 1, item)
            self._queue_list.setCurrentRow(row - 1)
            self._queue[row - 1], self._queue[row] = self._queue[row], self._queue[row - 1]
            self._update_state()

    def _move_down(self):
        """Move selected queue item down."""
        row = self._queue_list.currentRow()
        if row >= 0 and row < self._queue_list.count() - 1:
            item = self._queue_list.takeItem(row)
            self._queue_list.insertItem(row + 1, item)
            self._queue_list.setCurrentRow(row + 1)
            self._queue[row], self._queue[row + 1] = self._queue[row + 1], self._queue[row]
            self._update_state()

    def _clear_queue(self):
        """Clear the stitch queue."""
        self._queue_list.clear()
        self._queue.clear()
        self._update_state()

    def _sync_queue_from_list(self):
        """Sync internal queue after drag-drop reorder."""
        new_queue = []
        for i in range(self._queue_list.count()):
            rec = self._queue_list.item(i).data(Qt.ItemDataRole.UserRole)
            if rec:
                new_queue.append(rec)
        self._queue = new_queue
        self._update_state()

    def _update_state(self):
        """Update summary, warnings, and button state."""
        count = len(self._queue)
        self._create_btn.setEnabled(count >= 2)

        if count < 2:
            self._summary_label.setText("Add at least 2 recordings to stitch")
            self._summary_label.setStyleSheet("color: #757575;")
            self._warning_label.setVisible(False)
            return

        total_dur = sum(r.duration_sec for r in self._queue)
        total_frames = sum(r.frame_count for r in self._queue)
        self._summary_label.setText(
            f"{count} segments  |  Total: {total_dur:.1f}s  |  {total_frames} frames"
        )
        self._summary_label.setStyleSheet("color: #e0e0e0;")

        # Check joint name consistency
        joint_sets = [frozenset(r.joint_names) for r in self._queue if r.joint_names]
        if joint_sets and len(set(joint_sets)) > 1:
            self._warning_label.setText(
                "Warning: recordings have different joint names")
            self._warning_label.setVisible(True)
        else:
            self._warning_label.setVisible(False)

    def _on_create(self):
        """Emit stitch request."""
        if len(self._queue) < 2:
            return
        name = self._name_edit.text().strip()
        self.stitch_requested.emit(list(self._queue), name)
