"""共享界面控件。"""
from __future__ import annotations

import time

from PySide6.QtCore import QEvent, QPoint, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPalette, QPen
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QStyle,
                               QStyleOptionButton, QStyleOptionComboBox, QStylePainter)


def set_legend_text_color(legend, color: str):
    """强制刷新 pyqtgraph 图例文字颜色，兼容不同版本的 LegendItem。"""
    if legend is None:
        return
    legend.setLabelTextColor(color)
    for _sample, label in legend.items:
        text = getattr(label, "text", None)
        if text:
            label.setText(text, color=color)
        label.update()


def equalize_graphics_rows(graphics_widget):
    """让 GraphicsLayoutWidget 中的绘图区按可用高度等分。"""
    layout = graphics_widget.ci.layout
    rows = layout.rowCount()
    if rows <= 0 or graphics_widget.height() <= 0:
        return
    left, top, right, bottom = layout.getContentsMargins()
    spacing = sum(layout.rowSpacing(row) for row in range(max(0, rows - 1)))
    height = max(1, int((graphics_widget.height() - top - bottom - spacing) / rows))
    for row in range(rows):
        layout.setRowFixedHeight(row, height)


class AdaptiveDoubleSpinBox(QDoubleSpinBox):
    """滚轮速度越快，单次调整幅度越大；慢速滚动保留细调能力。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._last_wheel_time = None
        self.lineEdit().installEventFilter(self)

    def wheelEvent(self, event):
        if not self._wheel_active():
            event.ignore()
            return
        self._apply_wheel(event)

    def eventFilter(self, watched, event):
        if watched is self.lineEdit() and event.type() == QEvent.Type.Wheel:
            if not self._wheel_active():
                event.ignore()
                return False
            self._apply_wheel(event)
            return True
        return super().eventFilter(watched, event)

    def _wheel_active(self):
        """只有先获得焦点的数值框才响应滚轮，避免滚动页面时误改参数。"""
        return self.hasFocus() or self.lineEdit().hasFocus()

    def _apply_wheel(self, event):
        delta = event.angleDelta().y()
        if not delta:
            delta = event.pixelDelta().y() * 8
        if not delta:
            super().wheelEvent(event)
            return

        now = time.monotonic()
        elapsed = None if self._last_wheel_time is None else now - self._last_wheel_time
        self._last_wheel_time = now
        notches = abs(delta) / 120.0
        if elapsed is None or elapsed <= 0:
            speed_scale = 1.0
        else:
            notches_per_second = notches / max(elapsed, 0.02)
            speed_scale = max(0.5, min(5.0, notches_per_second / 2.0))
        amount = self.singleStep() * notches * speed_scale
        direction = 1.0 if delta > 0 else -1.0
        self.setValue(self.value() + direction * amount)
        event.accept()


class SelectedFirstComboBox(QComboBox):
    """展开时临时把当前选项放在第一行，关闭后恢复原始顺序。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._popup_snapshot = None
        self._popup_order = None

    def _snapshot_items(self):
        return [
            (self.itemText(index), dict(self.model().itemData(self.model().index(index, 0))))
            for index in range(self.count())
        ]

    def _replace_items(self, items, selected_index):
        self.blockSignals(True)
        try:
            self.clear()
            for text, data in items:
                self.addItem(text)
                index = self.count() - 1
                for role, value in data.items():
                    self.setItemData(index, value, role)
            if items:
                self.setCurrentIndex(max(0, min(selected_index, len(items) - 1)))
        finally:
            self.blockSignals(False)

    def showPopup(self):
        # Do not permanently reorder the model: otherwise every opened popup
        # changes the next popup's default order.
        if self._popup_snapshot is not None:
            self.hidePopup()
        index = self.currentIndex()
        if index > 0:
            self._popup_snapshot = self._snapshot_items()
            self._popup_order = [index] + [i for i in range(self.count()) if i != index]
            ordered = [self._popup_snapshot[i] for i in self._popup_order]
            self._replace_items(ordered, 0)
        super().showPopup()

    def hidePopup(self):
        # Qt may reset the combo's current index while closing the popup.
        # Capture the item chosen in the popup before delegating to Qt.
        view_index = self.view().currentIndex()
        popup_index = view_index.row() if view_index.isValid() else self.currentIndex()
        snapshot = self._popup_snapshot
        order = self._popup_order
        if snapshot is None:
            # No temporary reorder was needed. Commit the popup row before
            # Qt closes its private popup container.
            if view_index.isValid():
                self.setCurrentIndex(popup_index)
            super().hidePopup()
            return
        if order and 0 <= popup_index < len(order):
            selected_original_index = order[popup_index]
        else:
            selected_original_index = 0
        self.setCurrentIndex(popup_index)
        super().hidePopup()
        self._popup_snapshot = None
        self._popup_order = None
        self._replace_items(snapshot, selected_original_index)
        # Some styles commit the popup click after hidePopup() returns. Apply
        # the mapped index once more after that commit so a click cannot jump
        # back to the item that was selected before opening the popup.
        QTimer.singleShot(0, lambda: self.setCurrentIndex(selected_original_index)
                          if self.count() else None)


class ArrowOnlyComboBox(SelectedFirstComboBox):
    """仅点击右侧下拉箭头时展开，点击文字只触发行选择。"""

    clicked = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._center_text = False

    def set_center_text(self, on: bool):
        self._center_text = bool(on)
        self.update()

    def paintEvent(self, event):
        if not self._center_text:
            super().paintEvent(event)
            return
        option = QStyleOptionComboBox()
        self.initStyleOption(option)
        text = option.currentText
        option.currentText = ""
        painter = QStylePainter(self)
        painter.drawComplexControl(QStyle.CC_ComboBox, option)
        rect = self.style().subControlRect(QStyle.CC_ComboBox, option,
                                           QStyle.SC_ComboBoxEditField, self)
        painter.setPen(self.palette().color(QPalette.ColorRole.Text))
        painter.drawText(rect, Qt.AlignCenter | Qt.TextSingleLine, text)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()
            option = QStyleOptionComboBox()
            self.initStyleOption(option)
            arrow = self.style().subControlRect(
                QStyle.CC_ComboBox, option, QStyle.SC_ComboBoxArrow, self
            )
            if not arrow.contains(event.position().toPoint()):
                self.setFocus(Qt.MouseFocusReason)
                event.accept()
                return
        super().mousePressEvent(event)


class CheckBox(QCheckBox):
    """深色主题下带明确勾选标记的复选框。"""

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self.isChecked():
            return
        option = QStyleOptionButton()
        self.initStyleOption(option)
        rect = self.style().subElementRect(QStyle.SE_CheckBoxIndicator, option, self)
        if rect.isEmpty():
            return
        rect = rect.adjusted(3, 3, -3, -3)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(self.palette().color(QPalette.ColorRole.Text), 2,
                            Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(QPoint(rect.left(), rect.center().y()),
                         QPoint(rect.center().x() - 1, rect.bottom()))
        painter.drawLine(QPoint(rect.center().x() - 1, rect.bottom()),
                         QPoint(rect.right(), rect.top()))
        painter.end()
