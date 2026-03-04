"""
In-app virtual keyboard for touchscreen use.

A QWidget overlay that detects focus on text input widgets via an app-wide
event filter and shows either a QWERTY or numeric keypad layout. Text is
injected directly via QLineEdit.insert() — no external keyboard process or
AT-SPI bridge required.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QStackedWidget, QLineEdit, QSpinBox, QDoubleSpinBox,
    QApplication, QSizePolicy,
)
from PySide6.QtCore import Qt, QEvent, QTimer


_QWERTY_ROWS = [
    list("qwertyuiop"),
    list("asdfghjkl"),
    list("zxcvbnm"),
]

_SYMBOL_ROWS = [
    list("1234567890"),
    list("@#$%&-_+("),
    list(")!\"':;/?,"),
]

_QWERTY_H = 180
_NUMERIC_H = 200


class _KeyButton(QPushButton):
    """A keyboard button that never takes focus."""

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)


class VirtualKeyboard(QWidget):
    """In-app on-screen keyboard overlay."""

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self._main_parent = parent  # Remember the main window
        self._target: QLineEdit | None = None
        self._shifted = False
        self._symbols = False
        self._alpha_buttons: list[_KeyButton] = []
        self._symbol_buttons: list[_KeyButton] = []
        self._shift_btn: _KeyButton | None = None
        self._sym_btn: _KeyButton | None = None

        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating, True)

        self._stack = QStackedWidget()
        self._stack.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._stack.addWidget(self._build_qwerty())
        self._stack.addWidget(self._build_numeric())

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._stack)

        self._apply_style()
        self.hide()

        QApplication.instance().installEventFilter(self)

    # ── Layout builders ──────────────────────────────────────────────

    def _build_qwerty(self) -> QWidget:
        page = QWidget()
        page.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(2, 2, 2, 2)
        vbox.setSpacing(2)

        # Alpha rows
        for row_chars in _QWERTY_ROWS:
            hbox = QHBoxLayout()
            hbox.setSpacing(2)
            for ch in row_chars:
                btn = _KeyButton(ch)
                btn.clicked.connect(lambda _, c=ch: self._on_key(c))
                hbox.addWidget(btn)
                self._alpha_buttons.append(btn)
            vbox.addLayout(hbox)

        # Insert Shift at start of row 3, Backspace at end
        row3 = vbox.itemAt(2).layout()
        self._shift_btn = _KeyButton("Shift")
        self._shift_btn.setMinimumWidth(60)
        self._shift_btn.clicked.connect(self._on_shift)
        row3.insertWidget(0, self._shift_btn)

        bksp = _KeyButton("\u232b")
        bksp.setMinimumWidth(60)
        bksp.clicked.connect(self._on_backspace)
        row3.addWidget(bksp)

        # Symbol rows (hidden initially)
        for row_chars in _SYMBOL_ROWS:
            hbox = QHBoxLayout()
            hbox.setSpacing(2)
            for ch in row_chars:
                btn = _KeyButton(ch)
                btn.clicked.connect(lambda _, c=ch: self._on_key(c))
                hbox.addWidget(btn)
                self._symbol_buttons.append(btn)
                btn.hide()
            vbox.addLayout(hbox)

        # Backspace at end of symbol row 3
        sym_row3 = vbox.itemAt(5).layout()
        sym_bksp = _KeyButton("\u232b")
        sym_bksp.setMinimumWidth(60)
        sym_bksp.clicked.connect(self._on_backspace)
        sym_bksp.hide()
        sym_row3.addWidget(sym_bksp)
        self._symbol_buttons.append(sym_bksp)

        # Bottom row: [123/ABC] [Space] [Done]
        hbox = QHBoxLayout()
        hbox.setSpacing(2)

        self._sym_btn = _KeyButton("123")
        self._sym_btn.setMinimumWidth(60)
        self._sym_btn.clicked.connect(self._on_toggle_symbols)
        hbox.addWidget(self._sym_btn)

        space = _KeyButton("Space")
        space.clicked.connect(lambda: self._on_key(" "))
        hbox.addWidget(space, 3)

        done = _KeyButton("Done")
        done.setMinimumWidth(60)
        done.clicked.connect(self._on_done)
        hbox.addWidget(done)
        vbox.addLayout(hbox)

        return page

    def _build_numeric(self) -> QWidget:
        page = QWidget()
        page.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        grid = QGridLayout(page)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setSpacing(2)

        keys = [
            ("7", 0, 0), ("8", 0, 1), ("9", 0, 2),
            ("4", 1, 0), ("5", 1, 1), ("6", 1, 2),
            ("1", 2, 0), ("2", 2, 1), ("3", 2, 2),
            ("-", 3, 0), ("0", 3, 1), (".", 3, 2),
        ]
        for text, r, c in keys:
            btn = _KeyButton(text)
            btn.clicked.connect(lambda _, t=text: self._on_key(t))
            grid.addWidget(btn, r, c)

        bksp = _KeyButton("\u232b")
        bksp.clicked.connect(self._on_backspace)
        grid.addWidget(bksp, 4, 0, 1, 2)

        done = _KeyButton("Done")
        done.clicked.connect(self._on_done)
        grid.addWidget(done, 4, 2)

        return page

    # ── Event filter ─────────────────────────────────────────────────

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.FocusIn:
            target = self._resolve_target(obj)
            if target is not None:
                self._target = target
                self._show_for_target(obj)
                return False
        if event.type() == QEvent.Type.FocusOut:
            QTimer.singleShot(120, self._check_hide)
            return False
        if event.type() == QEvent.Type.Resize and obj is self.parent():
            if self.isVisible():
                self._reposition()
            return False
        return False

    def _resolve_target(self, widget) -> QLineEdit | None:
        """Return the QLineEdit to inject text into, or None."""
        if not isinstance(widget, QWidget):
            return None
        # Direct QLineEdit
        if isinstance(widget, QLineEdit):
            if widget.isReadOnly() or not widget.isEnabled():
                return None
            # Skip internal line edits of combo boxes
            parent = widget.parent()
            from PySide6.QtWidgets import QComboBox
            if isinstance(parent, QComboBox):
                return None
            return widget
        # SpinBox — get internal line edit
        if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
            if not widget.isEnabled():
                return None
            return widget.lineEdit()
        return None

    def _show_for_target(self, widget):
        """Show the appropriate keyboard page for the widget type."""
        if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
            self._stack.setCurrentIndex(1)
        elif isinstance(widget, QLineEdit):
            # Check if parent is a spin box
            parent = widget.parent()
            if isinstance(parent, (QSpinBox, QDoubleSpinBox)):
                self._stack.setCurrentIndex(1)
            else:
                self._stack.setCurrentIndex(0)
                self._reset_qwerty()
        # Reparent to the target's top-level window so we appear above dialogs
        target_window = widget.window()
        if target_window is not self.parent():
            self.setParent(target_window)
            self._apply_style()  # Re-apply after reparent
        self._reposition()
        self.show()
        self.raise_()

    def _check_hide(self):
        """Hide keyboard if focus left all input widgets."""
        focused = QApplication.focusWidget()
        if focused is None:
            self._hide_and_reparent()
            return
        # If focus is still on a relevant target, keep showing
        if self._resolve_target(focused) is not None:
            return
        self._hide_and_reparent()

    def _hide_and_reparent(self):
        """Hide and return to the main parent window."""
        self.hide()
        self._target = None
        if self.parent() is not self._main_parent:
            self.setParent(self._main_parent)
            self._apply_style()

    # ── Key actions ──────────────────────────────────────────────────

    def _on_key(self, char: str):
        if self._target is None or not self._target.isVisible():
            return
        if self._shifted and char.isalpha():
            char = char.upper()
        self._target.insert(char)
        if self._shifted:
            self._shifted = False
            self._update_key_labels()

    def _on_backspace(self):
        if self._target is None:
            return
        self._target.backspace()

    def _on_done(self):
        if self._target is not None:
            self._target.clearFocus()
        self._hide_and_reparent()

    def _on_shift(self):
        self._shifted = not self._shifted
        self._update_key_labels()

    def _on_toggle_symbols(self):
        self._symbols = not self._symbols
        # Toggle visibility of alpha vs symbol rows
        for btn in self._alpha_buttons:
            btn.setVisible(not self._symbols)
        if self._shift_btn:
            self._shift_btn.setVisible(not self._symbols)
        for btn in self._symbol_buttons:
            btn.setVisible(self._symbols)
        if self._sym_btn:
            self._sym_btn.setText("ABC" if self._symbols else "123")

    def _reset_qwerty(self):
        """Reset QWERTY to lowercase alpha mode."""
        self._shifted = False
        self._symbols = False
        for btn in self._alpha_buttons:
            btn.setVisible(True)
        if self._shift_btn:
            self._shift_btn.setVisible(True)
        for btn in self._symbol_buttons:
            btn.setVisible(False)
        if self._sym_btn:
            self._sym_btn.setText("123")
        self._update_key_labels()

    def _update_key_labels(self):
        for btn in self._alpha_buttons:
            t = btn.text()
            if t.isalpha():
                btn.setText(t.upper() if self._shifted else t.lower())
        if self._shift_btn:
            self._shift_btn.setStyleSheet(
                "background-color: #5a5a5a;" if self._shifted else ""
            )

    # ── Positioning ──────────────────────────────────────────────────

    def _reposition(self):
        p = self.parent()
        if p is None:
            return
        h = _QWERTY_H if self._stack.currentIndex() == 0 else _NUMERIC_H
        # Use screen height when window exceeds it (e.g. minSize > screen)
        screen = QApplication.primaryScreen()
        visible_h = p.height()
        if screen:
            visible_h = min(visible_h, screen.geometry().height())
        self.setGeometry(0, visible_h - h, p.width(), h)

    # ── Styling ──────────────────────────────────────────────────────

    def _apply_style(self):
        self.setStyleSheet("""
            VirtualKeyboard {
                background-color: #2a2a2a;
            }
            _KeyButton {
                background-color: #3a3a3a;
                color: #e0e0e0;
                border: 1px solid #444;
                border-radius: 4px;
                font-size: 16px;
                min-height: 36px;
            }
            _KeyButton:pressed {
                background-color: #4a4a4a;
            }
        """)
