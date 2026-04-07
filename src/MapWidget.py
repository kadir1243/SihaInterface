import math

from PySide6.QtCore import Qt, QByteArray, QObject, QAbstractListModel, Slot, qWarning, qDebug, QAbstractItemModel, \
    Property, Signal
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QWidget
from pymavlink.mavutil import mavfile

from src.ServerConnection import TelemetryResponseData, ServerAdsData


class PlaneData:
    position: QGeoCoordinate
    plane_type: int # 0 for blue, 1 for red, 2 for green, 3 for yellow
    rotation: float
    def __init__(self, position: QGeoCoordinate, plane_type: int, rotation: float):
        self.position = position
        self.plane_type = plane_type
        self.rotation = rotation

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
        if role == Qt.ItemDataRole.UserRole + 3: # rotation
            return data.rotation
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        plane_type: int = Qt.ItemDataRole.UserRole + 2
        rotation: int = Qt.ItemDataRole.UserRole + 3
        return {position: QByteArray(b"position"), plane_type: QByteArray(b"plane_type"), rotation: QByteArray(b"rotation")}

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

class AdsData:
    position: QGeoCoordinate
    size: float

class AdsDataModel(QAbstractListModel):
    m_datas: list[AdsData] = []

    def data(self, index, /, role=...):
        if (not index.isValid()) or index.row() < 0 or index.row() >= len(self.m_datas):
            return None
        data = self.m_datas[index.row()]
        if role == Qt.ItemDataRole.UserRole + 1:
            return data.position
        if role == Qt.ItemDataRole.UserRole + 2:
            return data.size
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        size: int = Qt.ItemDataRole.UserRole + 2
        return {position: QByteArray(b"position"), size: QByteArray(b"size")}

def distance(coord1: QGeoCoordinate, coord2: QGeoCoordinate) -> float:
    x1 = coord1.latitude()
    x2 = coord2.latitude()
    y1 = coord1.longitude()
    y2 = coord2.longitude()
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

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
        qDebug("Pressed the mouse button %s in coordinates %s %s %s" % (button, coordinate.altitude(), coordinate.latitude(), coordinate.longitude()))

        data: SpecialCoordsData = SpecialCoordsData()
        data.position = coordinate
        match button:
            case Qt.MouseButton.LeftButton.value:
                data.coord_type = 1
            case Qt.MouseButton.RightButton.value:
                data.coord_type = 2
                for m_data in self.parent.user_ads_data_model.m_datas:
                    d = distance(coordinate, m_data.position) * 100000
                    qDebug("Distance: %s, radius: %s" % (d, m_data.size))
                    if d <= m_data.size:
                        self.parent.user_ads_data_model.m_datas.remove(m_data)
                        self.parent.user_ads_data_model.layoutChanged.emit()
                        return # Remove ads functionality
            case Qt.MouseButton.MiddleButton.value:
                data.coord_type = self.gc_cycle + 5
            case _:
                qWarning("Invalid mouse input %s" % button)
                return
        if data.coord_type == self.gc_cycle + 5: # I hope i will remember how this works
            qDebug("Gc cycle: %s" % self.gc_cycle)
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
                    qDebug("Removing mdata: %s, with type: %s" % (mdata.position, mdata.coord_type))
                    self.parent.coord_data_model.m_datas.remove(mdata)
                    break
            self.parent.coords_for_geofence.gc_changed.emit()
            self.gc_cycle = self.gc_cycle + 1
            qDebug("next Gc cycle: %s" % self.gc_cycle)
        self.parent.coord_data_model.m_datas.append(data)
        self.parent.coord_data_model.layoutChanged.emit()

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
    server_ads_data_model: AdsDataModel
    user_ads_data_model: AdsDataModel

    def __init__(self, parent: QWidget | None = None):
        QQuickWidget.__init__(self, parent)
        self.plane_data_model = PlaneDataModel()
        self.coord_data_model = SpecialCoordsDataModel()
        self.mouse_input_handler = MouseInputHandler(self)
        self.coords_for_geofence = GeofenceData(self)
        self.server_ads_data_model = AdsDataModel()
        self.user_ads_data_model = AdsDataModel()

        self.engine().rootContext().setContextProperty("plane_data_model", self.plane_data_model)
        self.engine().rootContext().setContextProperty("coord_data_model", self.coord_data_model)
        self.engine().rootContext().setContextProperty("coords_for_geofence", self.coords_for_geofence)
        self.engine().rootContext().setContextProperty("server_ads_data_model", self.server_ads_data_model)
        self.engine().rootContext().setContextProperty("user_ads_data_model", self.user_ads_data_model)
        self.engine().rootContext().setContextProperty("mouseInputHandler", self.mouse_input_handler)
        self.setSource("qml/map_widget.qml")
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

    def update_plane_data(self, our_team_number: int, last_server_response: TelemetryResponseData):
        self.plane_data_model.m_datas.clear()
        for uav in last_server_response.konumBilgileri:
            # TODO: Add types to uav
            plane_type: int
            if our_team_number == uav.takim_numarasi:
                plane_type = 0
            else:
                plane_type = 1
            self.plane_data_model.m_datas.append(PlaneData(QGeoCoordinate(uav.iha_enlem, uav.iha_boylam), plane_type, (uav.iha_yatis * 4) + 180))
        self.plane_data_model.layoutChanged.emit()

    def update_plane_data_without_server(self, pos: QGeoCoordinate, rotation: float):
        self.plane_data_model.m_datas.clear()
        self.plane_data_model.m_datas.append(PlaneData(pos, 0, rotation))
        self.plane_data_model.layoutChanged.emit()

    def update_server_ads_data(self, ads_list: list[ServerAdsData]):
        self.server_ads_data_model.m_datas.clear()
        for ads in ads_list:
            data: AdsData = AdsData()
            data.position = QGeoCoordinate(ads.hssEnlem, ads.hssBoylam)
            data.size = ads.hssYariCap
            self.server_ads_data_model.m_datas.append(data)
        self.server_ads_data_model.layoutChanged.emit()

    def update_ads_data(self, ads: AdsData):
        self.user_ads_data_model.m_datas.append(ads)
        self.user_ads_data_model.layoutChanged.emit()
