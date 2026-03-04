"""
On-screen virtual keyboard for touchscreen use.

Auto-shows when a QLineEdit gains focus, hides on Done/Enter or focus loss.
"""

from PySide6.QtCore import Qt, QEvent
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QApplication,
    QSizePolicy
)


_ROWS = [
    list("1234567890"),
    list("qwertyuiop"),
    list("asdfghjkl"),
    list("zxcvbnm"),
]

_BTN_STYLE = """
QPushButton {
    background: #333;
    color: #eee;
    border: 1px solid #555;
    border-radius: 3px;
    min-height: 36px;
    max-height: 36px;
    font-size: 16px;
}
QPushButton:pressed {
    background: #555;
}
"""

_SPECIAL_BTN_STYLE = """
QPushButton {
    background: #444;
    color: #ccc;
    border: 1px solid #555;
    border-radius: 3px;
    min-height: 36px;
    max-height: 36px;
    font-size: 14px;
}
QPushButton:pressed {
    background: #666;
}
"""


class VirtualKeyboard(QWidget):
    """QWERTY on-screen keyboard that attaches to the bottom of a parent window."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._target: QLineEdit | None = None
        self._shifted = False

        self.setObjectName("virtualKeyboard")
        self.setStyleSheet("#virtualKeyboard { background: #222; border-top: 2px solid #555; }")
        self.setMaximumHeight(200)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self._setup_ui()
        self.hide()

        # Install app-wide event filter
        QApplication.instance().installEventFilter(self)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        self._key_buttons: list[QPushButton] = []

        for row_chars in _ROWS:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(2)
            for ch in row_chars:
                btn = QPushButton(ch)
                btn.setStyleSheet(_BTN_STYLE)
                btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                btn.clicked.connect(lambda checked=False, c=ch: self._on_key(c))
                self._key_buttons.append(btn)
                row_layout.addWidget(btn)
            layout.addLayout(row_layout)

        # Bottom row: Shift, Space, Backspace, Done
        bottom = QHBoxLayout()
        bottom.setSpacing(2)

        self._shift_btn = QPushButton("Shift")
        self._shift_btn.setStyleSheet(_SPECIAL_BTN_STYLE)
        self._shift_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._shift_btn.clicked.connect(self._on_shift)
        bottom.addWidget(self._shift_btn, 1)

        space_btn = QPushButton("Space")
        space_btn.setStyleSheet(_SPECIAL_BTN_STYLE)
        space_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        space_btn.clicked.connect(lambda: self._on_key(" "))
        bottom.addWidget(space_btn, 3)

        backspace_btn = QPushButton("Bksp")
        backspace_btn.setStyleSheet(_SPECIAL_BTN_STYLE)
        backspace_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        backspace_btn.clicked.connect(self._on_backspace)
        bottom.addWidget(backspace_btn, 1)

        done_btn = QPushButton("Done")
        done_btn.setStyleSheet(_SPECIAL_BTN_STYLE)
        done_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        done_btn.clicked.connect(self._on_done)
        bottom.addWidget(done_btn, 1)

        layout.addLayout(bottom)

    def _on_key(self, ch: str):
        if not self._target:
            return
        text = ch.upper() if self._shifted else ch
        self._target.insert(text)
        if self._shifted:
            self._shifted = False
            self._update_labels()

    def _on_backspace(self):
        if self._target:
            self._target.backspace()

    def _on_shift(self):
        self._shifted = not self._shifted
        self._update_labels()

    def _on_done(self):
        if self._target:
            self._target.clearFocus()
        self.hide()
        self._target = None

    def _update_labels(self):
        for btn in self._key_buttons:
            text = btn.text()
            if len(text) == 1 and text.isalpha():
                btn.setText(text.upper() if self._shifted else text.lower())
        self._shift_btn.setStyleSheet(
            _SPECIAL_BTN_STYLE.replace("background: #444", "background: #666")
            if self._shifted else _SPECIAL_BTN_STYLE
        )

    def _show_for(self, line_edit: QLineEdit):
        self._target = line_edit
        self._reposition()
        self.show()
        self.raise_()

    def _reposition(self):
        parent = self.parentWidget()
        if not parent:
            return
        pw = parent.width()
        self.setFixedWidth(pw)
        self.adjustSize()
        h = self.sizeHint().height()
        self.setGeometry(0, parent.height() - h, pw, h)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            if isinstance(obj, QLineEdit):
                self._show_for(obj)
        elif event.type() == QEvent.Type.FocusOut:
            if obj is self._target:
                # Delay hide to check if focus moved to another QLineEdit
                from PySide6.QtCore import QTimer
                QTimer.singleShot(50, self._check_focus)
        elif event.type() == QEvent.Type.Resize:
            if obj is self.parentWidget() and self.isVisible():
                self._reposition()
        return False

    def _check_focus(self):
        focused = QApplication.focusWidget()
        if isinstance(focused, QLineEdit):
            self._show_for(focused)
        else:
            self.hide()
            self._target = None
