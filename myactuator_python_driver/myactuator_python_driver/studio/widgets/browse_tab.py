"""
Browse tab widget for Motor Recording Studio.

Recording file browser with management options.
"""

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QTableView, QHeaderView, QAbstractItemView,
    QMenu, QMessageBox, QInputDialog
)

from myactuator_python_driver.studio.models.recording_model import RecordingModel
from myactuator_python_driver.studio.recording_manager import RecordingInfo


class BrowseTab(QWidget):
    """
    Browse tab widget for managing recordings.

    Contains:
    - Search/filter box
    - Recordings table
    - Context menu for rename/delete
    """

    # Signals
    recording_selected = pyqtSignal(object)  # RecordingInfo
    recording_double_clicked = pyqtSignal(object)  # RecordingInfo
    refresh_requested = pyqtSignal()
    delete_requested = pyqtSignal(object)  # RecordingInfo
    rename_requested = pyqtSignal(object, str)  # RecordingInfo, new_name

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        # Search bar
        search_layout = QHBoxLayout()

        self._search_edit = QLineEdit()
        self._search_edit.setPlaceholderText("Search recordings...")
        self._search_edit.textChanged.connect(self._on_search_changed)
        search_layout.addWidget(self._search_edit)

        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self.refresh_requested)
        search_layout.addWidget(self._refresh_btn)

        layout.addLayout(search_layout)

        # Table
        self._model = RecordingModel()
        self._table = QTableView()
        self._table.setModel(self._model)

        # Table styling
        self._table.setAlternatingRowColors(True)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSortingEnabled(True)
        self._table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._show_context_menu)
        self._table.doubleClicked.connect(self._on_double_clicked)
        self._table.selectionModel().selectionChanged.connect(self._on_selection_changed)

        # Column sizing
        header = self._table.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)  # Name
        for i in range(1, len(RecordingModel.COLUMNS)):
            header.setSectionResizeMode(i, QHeaderView.ResizeMode.ResizeToContents)

        self._table.verticalHeader().setVisible(False)

        layout.addWidget(self._table, 1)

        # Info bar
        info_layout = QHBoxLayout()

        self._count_label = QLabel("0 recordings")
        info_layout.addWidget(self._count_label)

        info_layout.addStretch()

        self._selected_label = QLabel("")
        self._selected_label.setStyleSheet("color: #757575;")
        info_layout.addWidget(self._selected_label)

        layout.addLayout(info_layout)

    def _on_search_changed(self, text: str):
        """Handle search text change."""
        # Simple filtering - hide non-matching rows
        text = text.lower()
        for row in range(self._model.rowCount()):
            recording = self._model.get_recording(row)
            if recording:
                match = text in recording.name.lower()
                self._table.setRowHidden(row, not match)

    def _on_selection_changed(self):
        """Handle table selection change."""
        selection = self._table.selectionModel().selectedRows()
        if selection:
            recording = self._model.get_recording(selection[0].row())
            if recording:
                self.recording_selected.emit(recording)
                self._selected_label.setText(
                    f"Selected: {recording.name} ({recording.duration_sec:.1f}s)"
                )
        else:
            self._selected_label.setText("")

    def _on_double_clicked(self, index):
        """Handle double-click on recording."""
        recording = self._model.get_recording_by_index(index)
        if recording:
            self.recording_double_clicked.emit(recording)

    def _show_context_menu(self, pos):
        """Show context menu for recording."""
        index = self._table.indexAt(pos)
        if not index.isValid():
            return

        recording = self._model.get_recording_by_index(index)
        if not recording:
            return

        menu = QMenu(self)

        play_action = menu.addAction("Play")
        play_action.triggered.connect(lambda: self.recording_double_clicked.emit(recording))

        menu.addSeparator()

        rename_action = menu.addAction("Rename...")
        rename_action.triggered.connect(lambda: self._rename_recording(recording))

        delete_action = menu.addAction("Delete")
        delete_action.triggered.connect(lambda: self._delete_recording(recording))

        menu.addSeparator()

        open_folder_action = menu.addAction("Open Folder")
        open_folder_action.triggered.connect(lambda: self._open_folder(recording))

        menu.exec(self._table.viewport().mapToGlobal(pos))

    def _rename_recording(self, recording: RecordingInfo):
        """Show rename dialog."""
        new_name, ok = QInputDialog.getText(
            self,
            "Rename Recording",
            "New name:",
            QLineEdit.EchoMode.Normal,
            recording.name
        )
        if ok and new_name and new_name != recording.name:
            self.rename_requested.emit(recording, new_name)

    def _delete_recording(self, recording: RecordingInfo):
        """Confirm and delete recording."""
        reply = QMessageBox.question(
            self,
            "Delete Recording",
            f"Are you sure you want to delete '{recording.name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.delete_requested.emit(recording)

    def _open_folder(self, recording: RecordingInfo):
        """Open the recording folder in file manager."""
        import subprocess
        import sys

        if sys.platform == "linux":
            subprocess.Popen(["xdg-open", str(recording.path)])
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(recording.path)])
        elif sys.platform == "win32":
            subprocess.Popen(["explorer", str(recording.path)])

    # === Public methods ===

    def set_recordings(self, recordings):
        """Set the list of recordings."""
        self._model.set_recordings(recordings)
        self._count_label.setText(f"{len(recordings)} recording(s)")
        self._on_search_changed(self._search_edit.text())

    def get_model(self) -> RecordingModel:
        """Get the recording model."""
        return self._model

    def get_selected_recording(self) -> Optional[RecordingInfo]:
        """Get the currently selected recording."""
        selection = self._table.selectionModel().selectedRows()
        if selection:
            return self._model.get_recording(selection[0].row())
        return None

    def remove_recording(self, recording: RecordingInfo):
        """Remove a recording from the list."""
        self._model.remove_recording(recording)
        self._count_label.setText(f"{self._model.count()} recording(s)")

    def clear_selection(self):
        """Clear the current selection."""
        self._table.clearSelection()
        self._selected_label.setText("")
