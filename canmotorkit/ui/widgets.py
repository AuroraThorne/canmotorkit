"""共享界面控件。"""
from __future__ import annotations

from PySide6.QtCore import QPoint, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (QCheckBox, QComboBox, QStyle, QStyleOptionButton,
                               QStyleOptionComboBox)


class ArrowOnlyComboBox(QComboBox):
    """仅点击右侧下拉箭头时展开，点击文字只触发行选择。"""

    clicked = Signal()

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
        painter.setPen(QPen(QColor("#f1f1f1"), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(QPoint(rect.left(), rect.center().y()),
                         QPoint(rect.center().x() - 1, rect.bottom()))
        painter.drawLine(QPoint(rect.center().x() - 1, rect.bottom()),
                         QPoint(rect.right(), rect.top()))
        painter.end()
