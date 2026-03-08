from PySide6.QtCore import Qt, QByteArray, QObject, QAbstractListModel, Slot, qWarning, qDebug, QAbstractItemModel, \
    Property, Signal
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QWidget
from pymavlink.mavutil import mavfile

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
        if (not index.isValid()) or index.row() < 0 or index.row() >= len(self.m_datas):
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
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        plane_type: int = Qt.ItemDataRole.UserRole + 2
        return {position: QByteArray(b"position"), plane_type: QByteArray(b"plane_type")}

class SpecialCoordsData:
    position: QGeoCoordinate
    coord_type: int = -1 # 0 for geofence, 1 for target coord

class SpecialCoordsDataModel(QAbstractListModel):
    m_datas: list[SpecialCoordsData] = []

    def data(self, index, /, role=...):
        if (not index.isValid()) or index.row() < 0 or index.row() >= len(self.m_datas):
            return None
        data = self.m_datas[index.row()]
        if role == Qt.ItemDataRole.UserRole + 1:
            return data.position
        if role == Qt.ItemDataRole.UserRole + 2:
            return data.coord_type
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        coord_type: int = Qt.ItemDataRole.UserRole + 2
        return {position: QByteArray(b"position"), coord_type: QByteArray(b"coord_type")}

class ModelHelper:
    @Slot(QAbstractItemModel, int, str)
    def data(self, model: QAbstractItemModel, row: int, userrole: str):
        roleNames: dict[int, QByteArray] = model.roleNames()
        for index, roleName in enumerate(roleNames):
            if userrole == roleName:
                return model.data(model.index(row, 0), index)

        return None

class MouseInputHandler(QObject):
    parent: MapWidget
    gc_cycle: int = 0
    def __init__(self, parent: MapWidget):
        super().__init__(parent)
        self.parent = parent

    @Slot(int, QGeoCoordinate)
    def handle_mouse_input_to_map(self, button: int, coordinate: QGeoCoordinate):
        if not coordinate.isValid():
            qWarning("Invalid mouse input coordinate.")
            return
        data: SpecialCoordsData = SpecialCoordsData()
        data.position = coordinate
        match button:
            case Qt.MouseButton.LeftButton.value:
                data.coord_type = 1
            case Qt.MouseButton.RightButton.value:
                data.coord_type = 2
            case Qt.MouseButton.MiddleButton.value:
                data.coord_type = self.gc_cycle + 5
            case _:
                qWarning("Invalid mouse input {}".format(button))
                return
        if data.coord_type == self.gc_cycle + 5: # I hope i will remember how this works
            qDebug("Gc cycle: " + str(self.gc_cycle))
            match self.gc_cycle:
                case 0 | 4:
                    self.parent.coords_for_geofence.gc1_v = coordinate
                    if self.gc_cycle == 4:
                        self.gc_cycle = 0
                case 1:
                    self.parent.coords_for_geofence.gc2_v = coordinate
                case 2:
                    self.parent.coords_for_geofence.gc3_v = coordinate
                case 3:
                    self.parent.coords_for_geofence.gc4_v = coordinate
            for mdata in self.parent.coord_data_model.m_datas:
                if mdata.coord_type == self.gc_cycle + 5 or mdata.coord_type == 9: # 9 is 5
                    qDebug("Removing mdata: " + str(mdata.position) + ", with type: " + str(mdata.coord_type))
                    self.parent.coord_data_model.m_datas.remove(mdata)
                    break
            self.parent.coords_for_geofence.gc_changed.emit()
            self.gc_cycle = self.gc_cycle + 1
            qDebug("next Gc cycle: " + str(self.gc_cycle))
        self.parent.coord_data_model.m_datas.append(data)
        self.parent.coord_data_model.layoutChanged.emit()

        qDebug("Pressed the mouse button {} in coordinates {} {} {}".format(button, coordinate.altitude(), coordinate.latitude(), coordinate.longitude()))

ZERO_GEO_COORDS: QGeoCoordinate = QGeoCoordinate()

class GeofenceData(QObject):
    gc1_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc2_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc3_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc4_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc_changed = Signal()
    def __init__(self, parent: MapWidget):
        super().__init__(parent)

    def read_gc1(self) -> QGeoCoordinate:
        return self.gc1_v

    def read_gc2(self) -> QGeoCoordinate:
        return self.gc2_v

    def read_gc3(self) -> QGeoCoordinate:
        return self.gc3_v

    def read_gc4(self) -> QGeoCoordinate:
        return self.gc4_v

    gc1 = Property(QGeoCoordinate, read_gc1, notify=gc_changed)
    gc2 = Property(QGeoCoordinate, read_gc2, notify=gc_changed)
    gc3 = Property(QGeoCoordinate, read_gc3, notify=gc_changed)
    gc4 = Property(QGeoCoordinate, read_gc4, notify=gc_changed)


class MapWidget(QQuickWidget):
    plane_data_model: PlaneDataModel
    coord_data_model: SpecialCoordsDataModel
    coords_for_geofence: GeofenceData = None
    mouse_input_handler: MouseInputHandler
    mavlink_connection: mavfile | None = None

    def __init__(self, parent: QWidget | None = None):
        QQuickWidget.__init__(self, parent)
        self.plane_data_model = PlaneDataModel()
        self.coord_data_model = SpecialCoordsDataModel()
        self.mouse_input_handler = MouseInputHandler(self)
        self.coords_for_geofence = GeofenceData(self)

        self.engine().rootContext().setContextProperty("plane_data_model", self.plane_data_model)
        self.engine().rootContext().setContextProperty("coord_data_model", self.coord_data_model)
        self.engine().rootContext().setContextProperty("coords_for_geofence", self.coords_for_geofence)
        self.engine().rootContext().setContextProperty("mouseInputHandler", self.mouse_input_handler)
        self.engine().rootContext().setContextProperty("very_internal_data_helper", ModelHelper())
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
        self.plane_data_model.layoutChanged.emit()
