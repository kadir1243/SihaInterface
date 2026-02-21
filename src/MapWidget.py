from typing import override

from PySide6.QtCore import QFile, Qt, QIODeviceBase, qCritical, QRect, QByteArray, QPoint, qInfo, QObject, QPointF
from PySide6.QtGui import QPaintEvent, QPainter, QPixmap, QImage, QMouseEvent
from PySide6.QtWidgets import QGraphicsView, QWidget, QGraphicsScene, QGraphicsSceneMouseEvent


def get_map() -> QPixmap:
    f: QFile = QFile("snapshot.jpg")
    if f.open(QIODeviceBase.OpenModeFlag.ReadOnly):
        qCritical("Cannot open file.")
    data: QByteArray = f.readAll()
    f.close()
    return QPixmap.fromImage(QImage.fromData(data))

class MapWidgetScene(QGraphicsScene): # TODO
    def __init__(self, parent: QGraphicsView):
        QGraphicsScene.__init__(self, parent)
        self.addPixmap(get_map())




class MapWidget(QGraphicsView): # TODO
    def __init__(self, parent: QWidget | None = None):
        QGraphicsView.__init__(self, parent=parent)
        self.setScene(MapWidgetScene(self))
        self.setMouseTracking(True)


    @override
    def render(self, painter: QPainter, /, target: QRect=..., source: QRect=..., aspect_ratio_mode: Qt.AspectRatioMode = ...):
        qInfo("Map rendering")

    mouse_pressed: bool = False
    def mousePressEvent(self, event: QMouseEvent, /):
        self.mouse_pressed = True
        super().mousePressEvent(event)

        if event.buttons() == Qt.MouseButton.LeftButton:
            self.last_mouse_press_pos = event.pos()

    def mouseReleaseEvent(self, event: QMouseEvent, /):
        self.mouse_pressed = False
        super().mouseReleaseEvent(event)

    last_mouse_move_pos: QPoint
    last_mouse_press_pos: QPoint = None
    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        self.last_mouse_move_pos = event.pos()

        scene_pos = self.mapToScene(self.last_mouse_move_pos)

        if self.last_mouse_press_pos is not None:
            delta = self.last_mouse_press_pos - event.pos()

            self.last_mouse_press_pos = event.pos()
            self.translate(delta.x(), delta.y())

