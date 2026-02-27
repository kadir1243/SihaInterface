from PySide6.QtCore import Qt, QByteArray, QObject, QAbstractListModel, Slot, qWarning, qDebug
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QWidget

from src.ServerConnection import TELEMETRY_DATA, SERVER_TELEMETRY_RESPONSE


class PlaneData:
    position: QGeoCoordinate
    plane_type: int # 0 for blue, 1 for red, 2 for green, 3 for yellow
    def __init__(self, position: QGeoCoordinate, plane_type: int):
        self.position = position
        self.plane_type = plane_type

class PlaneDataModel(QAbstractListModel):
    m_datas: list[PlaneData] = []

    def data(self, index, /, role=...):
        if (not index.isValid()) or index.row() < 0 or index.row() >= self.m_datas.__len__():
            return None
        data = self.m_datas[index.row()]
        if role == Qt.ItemDataRole.UserRole + 1:
            return data.position
        if role == Qt.ItemDataRole.UserRole + 2: # plane_type
            return data.plane_type
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return self.m_datas.__len__()

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        plane_type: int = Qt.ItemDataRole.UserRole + 2
        return {position: QByteArray(b"position"), plane_type: QByteArray(b"plane_type")}

class MouseInputHandler(QObject):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

    @Slot(int, QGeoCoordinate)
    def handle_mouse_input(self, button: int, coordinate: QGeoCoordinate):
        if not coordinate.isValid():
            qWarning("Invalid mouse input coordinate.")
            return
        if button == Qt.MouseButton.LeftButton:
            pass
        else:
            pass
        qDebug("You pressed the mouse button {} in coordinates {} {} {}".format(button, coordinate.altitude(), coordinate.latitude(), coordinate.longitude()))

class MapWidget(QQuickWidget):
    plane_data_model: PlaneDataModel
    mouse_input_handler: MouseInputHandler
    def __init__(self, parent: QWidget | None = None):
        QQuickWidget.__init__(self, parent)
        self.plane_data_model = PlaneDataModel()
        self.mouse_input_handler = MouseInputHandler(self)

        self.engine().rootContext().setContextProperty("datamodel", self.plane_data_model)
        self.engine().rootContext().setContextProperty("mouseInputHandler", self.mouse_input_handler)
        self.setSource("qml/map_widget.qml")
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

    def update_plane_data(self):
        self.plane_data_model.m_datas.clear()
        our_team_number: int = TELEMETRY_DATA.takim_numarasi
        for uav in SERVER_TELEMETRY_RESPONSE.konumBilgileri:
            # TODO: Add types to uav
            plane_type: int
            if our_team_number == uav.takim_numarasi:
                plane_type = 0
            else:
                plane_type = 1
            self.plane_data_model.m_datas.append(PlaneData(QGeoCoordinate(uav.iha_enlem, uav.iha_boylam), plane_type))