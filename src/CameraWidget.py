from PySide6.QtCore import QTimer
from PySide6.QtGui import QPixmap, QBrush
from PySide6.QtWidgets import QWidget, QGraphicsView, QGraphicsScene


class CameraWidget(QGraphicsView):
    timer: QTimer | None = None
    image_to_show: QPixmap | None = None

    def __init__(self, parent: QWidget | None = None):
        QGraphicsView.__init__(self, parent=parent)
        self.setScene(QGraphicsScene())

    def startUpdater(self):
        if True: # FIXME: Add this
            return
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_with_timer)

    def stopUpdater(self):
        self.timer.stop()
        self.timer = None

    def paintEvent(self, event, /):
        if self.image_to_show is not None:
            self.scene().addPixmap(self.image_to_show)
        super().paintEvent(event)

    def _update_with_timer(self):
            #TODO: Idk how this works
            if True: return
            self.scene().clear()