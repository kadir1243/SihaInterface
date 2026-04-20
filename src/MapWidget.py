import math

from PySide6.QtCore import Qt, QByteArray, QObject, QAbstractListModel, Slot, qWarning, qDebug, Property, Signal, \
    QPointF
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtQuick import QQuickItem
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QWidget, QDialogButtonBox
from pymavlink.dialects.v20.all import MAV_CMD_DO_REPOSITION, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, \
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
from pymavlink.mavutil import mavfile

from src.AdvancedRepositionDialog import AdvancedRepositionDialog, DEFAULT_ALTITUDE, DEFAULT_LOITER_RADIUS, \
    DEFAULT_SPEED, DEFAULT_YAW
from src.ServerConnection import TelemetryResponseData, ServerAdsData


class PlaneData:
    position: QGeoCoordinate
    plane_type: int # 0 for green, 1 for red, 2 for blue, 3 for yellow
    rotation: float
    team_no: int
    def __init__(self, team_no: int, position: QGeoCoordinate, plane_type: int, rotation: float):
        self.team_no = team_no
        self.position = position
        self.plane_type = plane_type
        self.rotation = rotation

class PlaneDataModel(QAbstractListModel):
    m_datas: list[PlaneData]

    def __init__(self, /):
        super().__init__()
        self.m_datas = []

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
        if role == Qt.ItemDataRole.UserRole + 4: # team_no
            return data.team_no
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        plane_type: int = Qt.ItemDataRole.UserRole + 2
        rotation: int = Qt.ItemDataRole.UserRole + 3
        team_no: int = Qt.ItemDataRole.UserRole + 4
        return {position: QByteArray(b"position"), plane_type: QByteArray(b"plane_type"), rotation: QByteArray(b"rotation"), team_no: QByteArray(b"team_no")}

class SpecialCoordsData:
    position: QGeoCoordinate
    coord_type: int # 0 for geofence

class SpecialCoordsDataModel(QAbstractListModel):
    m_datas: list[SpecialCoordsData]

    def __init__(self, /):
        super().__init__()
        self.m_datas = []

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
    is_selected: bool

class AdsDataModel(QAbstractListModel):
    m_datas: list[AdsData]

    def __init__(self, /):
        super().__init__()
        self.m_datas = []

    def data(self, index, /, role=...):
        if (not index.isValid()) or index.row() < 0 or index.row() >= len(self.m_datas):
            return None
        data = self.m_datas[index.row()]
        if role == Qt.ItemDataRole.UserRole + 1:
            return data.position
        if role == Qt.ItemDataRole.UserRole + 2:
            return data.size
        if role == Qt.ItemDataRole.UserRole + 3:
            return data.is_selected
        return None

    def rowCount(self, /, parent=...):
        if parent.isValid():
            return 0
        return len(self.m_datas)

    def roleNames(self, /) -> dict[int, QByteArray]:
        position: int = Qt.ItemDataRole.UserRole + 1
        size: int = Qt.ItemDataRole.UserRole + 2
        is_selected: int = Qt.ItemDataRole.UserRole + 3
        return {position: QByteArray(b"position"), size: QByteArray(b"size"), is_selected: QByteArray(b"is_selected")}

def distance(coord1: QGeoCoordinate, coord2: QGeoCoordinate) -> float:
    x1 = coord1.latitude()
    x2 = coord2.latitude()
    y1 = coord1.longitude()
    y2 = coord2.longitude()
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def point_distance(coord1: QPointF, coord2: QPointF) -> float:
    if not hasattr(coord1, "x") or not hasattr(coord2, "y"):
        return 999999
    x1 = coord1.x()
    x2 = coord2.x()
    y1 = coord1.y()
    y2 = coord2.y()
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def geo_to_screen(center: QGeoCoordinate, zoom: float, width: float, height: float, lat: float, lon: float) -> QPointF | None: # It works now, I hope it will work forever
    if None in (center, zoom, width, height):
        return None

    def mercator_y(lat_deg: float) -> float:
        lat_rad = math.radians(lat_deg)
        return math.log(math.tan(math.pi / 4 + lat_rad / 2))

    total_pixels = 256 * (2 ** zoom)

    dx = (lon - center.longitude()) / 360.0 * total_pixels
    dy = (mercator_y(center.latitude()) - mercator_y(lat)) / (2 * math.pi) * total_pixels

    return QPointF(width / 2 + dx, height / 2 + dy)

class MouseInputHandler(QObject):
    parent: MapWidget
    ard_dialog: AdvancedRepositionDialog = None
    gc_cycle: int
    def __init__(self, parent: MapWidget):
        super().__init__(parent)
        self.parent = parent
        self.gc_cycle = 1

    @Slot(int, QGeoCoordinate)
    def handle_mouse_input_to_map_with_ctrl(self, button: int, coordinate: QGeoCoordinate):
        if not coordinate.isValid():
            qWarning("Invalid mouse input with ctrl coordinate.")
            return
        qDebug("Pressed the mouse button %s with ctrl in coordinates %s %s %s" % (button, coordinate.altitude(), coordinate.latitude(), coordinate.longitude()))
        match button:
            case Qt.MouseButton.RightButton.value:
                if not self.ard_dialog is None or self.parent.mavlink_connection is None:
                    return
                self.ard_dialog = AdvancedRepositionDialog(self.parent)
                self.ard_dialog.ui.latitude.setText(str(coordinate.latitude()))
                self.ard_dialog.ui.longitude.setText(str(coordinate.longitude()))
                self.ard_dialog.ui.altitude.setText(str(self.parent.reposition_altitude))
                if self.parent.reposition_loiter_radius != DEFAULT_LOITER_RADIUS:
                    self.ard_dialog.ui.loiter_radius.setText(str(self.parent.reposition_loiter_radius))
                if self.parent.reposition_speed != DEFAULT_SPEED:
                    self.ard_dialog.ui.speed.setText(str(self.parent.reposition_speed))
                if not math.isnan(self.parent.reposition_yaw):
                    self.ard_dialog.ui.yaw.setText(str(self.parent.reposition_yaw))
                self.ard_dialog.ui.buttons.button(QDialogButtonBox.StandardButton.Save).clicked.connect(self.ard_save)
                self.ard_dialog.ui.buttons.button(QDialogButtonBox.StandardButton.Ok).clicked.connect(self.ard_ok)
                self.ard_dialog.ui.buttons.button(QDialogButtonBox.StandardButton.Cancel).clicked.connect(self.ard_cancel)
                self.ard_dialog.finished.connect(self.ard_reset)
                self.ard_dialog.show()

    def ard_save(self):
        self.parent.reposition_altitude = float(self.ard_dialog.ui.altitude.text())
        if self.ard_dialog.ui.loiter_radius.text():
            self.parent.reposition_loiter_radius = float(self.ard_dialog.ui.loiter_radius.text())
        if self.ard_dialog.ui.speed.text():
            self.parent.reposition_speed = float(self.ard_dialog.ui.speed.text())
        if self.ard_dialog.ui.yaw.text():
            self.parent.reposition_yaw = float(self.ard_dialog.ui.yaw.text())

    def ard_ok(self):
        self.parent.target_coord.position_v = QGeoCoordinate(float(self.ard_dialog.ui.latitude.text()), float(self.ard_dialog.ui.longitude.text()))
        self.parent.target_coord.updated.emit()
        speed: float = self.return_non_null(self.ard_dialog.ui.speed.text(), self.parent.reposition_speed)
        yaw: float = self.return_non_null(self.ard_dialog.ui.yaw.text(), self.parent.reposition_yaw)
        loiter_radius: float = self.return_non_null(self.ard_dialog.ui.loiter_radius.text(), self.parent.reposition_loiter_radius)
        self.parent.mavlink_connection.mav.command_int_send(self.parent.mavlink_connection.target_system,
                                                            self.parent.mavlink_connection.target_component,
                                                            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                            MAV_CMD_DO_REPOSITION,
                                                            0,
                                                            0,
                                                            speed,
                                                            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                                                            loiter_radius, # loiter radius,
                                                            yaw,
                                                            int(self.parent.target_coord.position_v.latitude() * 10 ** 7),
                                                            int(self.parent.target_coord.position_v.longitude() * 10 ** 7),
                                                            float(self.ard_dialog.ui.altitude.text()))
        qDebug("Sent reposition command to %s %s with relative altitude %s, with loiter radius %s, with speed %s, and with yaw %s" % (
            self.parent.target_coord.position_v.latitude(), self.parent.target_coord.position_v.longitude(), float(self.ard_dialog.ui.altitude.text()),
            loiter_radius, speed, yaw))
        self.ard_dialog.close()
        self.ard_reset()

    @staticmethod
    def return_non_null(a: str, b: float) -> float:
        return b if a is None or len(a.strip()) == 0 else float(a.strip())

    def ard_cancel(self):
        self.ard_dialog.close()
        self.ard_reset()

    def ard_reset(self):
        self.ard_dialog = None

    @Slot(int, QGeoCoordinate, float, float)
    def handle_mouse_input_to_map(self, button: int, coordinate: QGeoCoordinate, mouseX: float, mouseY: float):
        if not coordinate.isValid():
            qWarning("Invalid mouse input coordinate.")
            return
        qDebug("Pressed the mouse button %s in coordinates %s %s %s" % (button, coordinate.altitude(), coordinate.latitude(), coordinate.longitude()))

        match button:
            case Qt.MouseButton.LeftButton.value:
                self.parent.selected_plane_team_no = -2
                for m_data in self.parent.user_ads_data_model.m_datas:
                    if m_data.is_selected:
                        m_data.is_selected = False
                        self.parent.user_ads_data_model.layoutChanged.emit()
                for m_data in self.parent.user_ads_data_model.m_datas:
                    d = distance(coordinate, m_data.position) * 100000
                    qDebug("Distance to ads: %s, radius: %s" % (d, m_data.size))
                    if d <= m_data.size:
                        m_data.is_selected = True
                        self.parent.user_ads_data_model.layoutChanged.emit()
                        return
                mapObject: QQuickItem = self.parent.rootObject().findChild(QQuickItem, "map")
                center: QGeoCoordinate = mapObject.property("center")
                zoom: float = mapObject.property("zoomLevel")
                width: float = mapObject.property("width")
                height: float = mapObject.property("height")
                for m_data in self.parent.plane_data_model.m_datas:
                    coordinate_point: QPointF = QPointF(mouseX, mouseY)
                    plane_point: QPointF | None = geo_to_screen(center, zoom, width, height, m_data.position.latitude(), m_data.position.longitude())
                    if plane_point is None:
                        continue
                    d = point_distance(coordinate_point, plane_point)
                    qDebug("Distance to plane: %s" % d)
                    if d <= 30: # Yeah, I selected this number randomly, I think this will be good number
                        m_data.plane_type = 2
                        self.parent.selected_plane_team_no = m_data.team_no
                        # TODO: Following the plane
                        self.parent.plane_data_model.layoutChanged.emit()
                        return
            case Qt.MouseButton.RightButton.value:
                if self.parent.mavlink_connection is None:
                    return
                self.parent.target_coord.position_v = coordinate
                self.parent.target_coord.updated.emit()
                self.parent.mavlink_connection.mav.command_int_send(self.parent.mavlink_connection.target_system,
                                                                    self.parent.mavlink_connection.target_component,
                                                                    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                    MAV_CMD_DO_REPOSITION,
                                                                    0,
                                                                    0,
                                                                    self.parent.reposition_speed,
                                                                    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                                                                    self.parent.reposition_loiter_radius,
                                                                    self.parent.reposition_yaw,
                                                                    int(coordinate.latitude() * 10**7),
                                                                    int(coordinate.longitude() * 10**7),
                                                                    self.parent.reposition_altitude)

                qDebug("Sent reposition command to %s %s with relative altitude %s, with loiter radius %s" % (coordinate.latitude(), coordinate.longitude(), self.parent.reposition_altitude, self.parent.reposition_loiter_radius))
            case Qt.MouseButton.MiddleButton.value:
                data: SpecialCoordsData = SpecialCoordsData()
                data.position = coordinate
                data.coord_type = self.gc_cycle + 5 # I hope i will remember how this works
                qDebug("Gc cycle: %s" % self.gc_cycle)
                current_cycle: int = self.gc_cycle
                match self.gc_cycle:
                    case 1:
                        self.parent.coords_for_geofence.gc1_v = coordinate
                    case 2:
                        self.parent.coords_for_geofence.gc2_v = coordinate
                    case 3:
                        self.parent.coords_for_geofence.gc3_v = coordinate
                    case 4:
                        self.parent.coords_for_geofence.gc4_v = coordinate
                        self.parent.coords_for_geofence.upload_geofence_data.emit()
                        self.gc_cycle = 0
                for mdata in self.parent.coord_data_model.m_datas:
                    if mdata.coord_type == current_cycle + 5:
                        qDebug("Removing mdata: %s, with type: %s" % (mdata.position, mdata.coord_type))
                        self.parent.coord_data_model.m_datas.remove(mdata)
                        break
                self.parent.coords_for_geofence.gc_changed.emit()
                self.gc_cycle = self.gc_cycle + 1
                qDebug("next Gc cycle: %s" % self.gc_cycle)
                self.parent.coord_data_model.m_datas.append(data)
                self.parent.coord_data_model.layoutChanged.emit()
            case _:
                qWarning("Invalid mouse input %s" % button)

ZERO_GEO_COORDS: QGeoCoordinate = QGeoCoordinate()

class GeofenceData(QObject):
    gc1_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc2_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc3_v: QGeoCoordinate = ZERO_GEO_COORDS
    gc4_v: QGeoCoordinate = ZERO_GEO_COORDS
    is_set: bool = False
    upload_geofence_data = Signal()
    gc_changed = Signal()
    def __init__(self, parent: MapWidget):
        super().__init__(parent)
        self.upload_geofence_data.connect(self.__on_set)

    def __on_set(self):
        self.is_set = True

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
class RepositionTargetHolder(QObject):
    position_v: QGeoCoordinate = QGeoCoordinate()
    updated = Signal()
    def __init__(self, parent: QWidget):
        super().__init__(parent)

    def read_position(self) -> QGeoCoordinate:
        return self.position_v

    position = Property(QGeoCoordinate, read_position, notify=updated)


class MapWidget(QQuickWidget):
    plane_data_model: PlaneDataModel
    coord_data_model: SpecialCoordsDataModel
    coords_for_geofence: GeofenceData = None
    mouse_input_handler: MouseInputHandler
    mavlink_connection: mavfile | None = None
    server_ads_data_model: AdsDataModel
    user_ads_data_model: AdsDataModel
    selected_plane_team_no: int = -2
    target_coord: RepositionTargetHolder
    reposition_altitude: float = DEFAULT_ALTITUDE
    reposition_loiter_radius: float = DEFAULT_LOITER_RADIUS
    reposition_yaw: float = DEFAULT_YAW
    reposition_speed: float = DEFAULT_SPEED

    def __init__(self, parent: QWidget | None = None):
        QQuickWidget.__init__(self, parent)
        self.plane_data_model = PlaneDataModel()
        self.coord_data_model = SpecialCoordsDataModel()
        self.mouse_input_handler = MouseInputHandler(self)
        self.coords_for_geofence = GeofenceData(self)
        self.server_ads_data_model = AdsDataModel()
        self.user_ads_data_model = AdsDataModel()
        self.target_coord = RepositionTargetHolder(self)

        self.engine().rootContext().setContextProperty("plane_data_model", self.plane_data_model)
        self.engine().rootContext().setContextProperty("coord_data_model", self.coord_data_model)
        self.engine().rootContext().setContextProperty("coords_for_geofence", self.coords_for_geofence)
        self.engine().rootContext().setContextProperty("server_ads_data_model", self.server_ads_data_model)
        self.engine().rootContext().setContextProperty("user_ads_data_model", self.user_ads_data_model)
        self.engine().rootContext().setContextProperty("mouseInputHandler", self.mouse_input_handler)
        self.engine().rootContext().setContextProperty("reposition_target_coord", self.target_coord)
        self.setSource("qml/map_widget.qml")
        self.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)

    def update_plane_data(self, our_team_number: int, last_server_response: TelemetryResponseData):
        self.plane_data_model.m_datas.clear()
        for uav in last_server_response.konumBilgileri:
            # TODO: Add types to uav
            plane_type: int
            if our_team_number == uav.takim_numarasi:
                plane_type = 0
            elif self.selected_plane_team_no == uav.takim_numarasi:
                plane_type = 2
            else:
                plane_type = 1
            self.plane_data_model.m_datas.append(PlaneData(uav.takim_numarasi, QGeoCoordinate(uav.iha_enlem, uav.iha_boylam), plane_type, (uav.iha_yatis * 4) + 180))
        self.plane_data_model.layoutChanged.emit()

    def update_plane_data_without_server(self, pos: QGeoCoordinate, rotation: float):
        self.plane_data_model.m_datas.clear()
        self.plane_data_model.m_datas.append(PlaneData(-1, pos, 2 if self.selected_plane_team_no == -1 else 0, rotation)) # TODO: Only for test
        self.plane_data_model.layoutChanged.emit()

    def update_server_ads_data(self, ads_list: list[ServerAdsData]):
        self.server_ads_data_model.m_datas.clear()
        for ads in ads_list:
            data: AdsData = AdsData()
            data.position = QGeoCoordinate(ads.hssEnlem, ads.hssBoylam)
            data.size = ads.hssYariCap
            data.is_selected = False
            self.server_ads_data_model.m_datas.append(data)
        self.server_ads_data_model.layoutChanged.emit()

    def update_ads_data(self, ads: AdsData):
        self.user_ads_data_model.m_datas.append(ads)
        self.user_ads_data_model.layoutChanged.emit()
