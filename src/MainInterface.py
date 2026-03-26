import math
from enum import Enum
from functools import partial

from PySide6.QtCore import Qt, QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug, QThread, QObject, Signal, Slot
from PySide6.QtGui import QAction
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtWidgets import QMainWindow, QPushButton, QToolButton, QTableWidgetItem, QMenu, QTableWidget
from pymavlink import mavutil
from pymavlink.dialects.v10.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message, MAVLink_message, MAVLink_heartbeat_message
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src.AddADSInterface import AddADSInterface
from src.MapWidget import ZERO_GEO_COORDS, AdsData
from src.SetGeofenceInterface import SetGeofenceInterface
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, QrCoords, \
    get_kamikaze_coords, TelemetryData, TelemetryResponseData, get_ads
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.siha_interface import Ui_MainWindow

class TrackableDataUpdate:
    @staticmethod
    def update_ground_speed(mainwindow:MainWindow, packet: MAVLink_vfr_hud_message) -> str:
        mainwindow.next_telemetry.iha_hiz = packet.groundspeed
        return str(packet.groundspeed)
    @staticmethod
    def update_air_speed(mainwindow:MainWindow, packet: MAVLink_vfr_hud_message) -> str:
        return str(packet.airspeed)
    @staticmethod
    def update_velocity(mainwindow:MainWindow, packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.vel)
    @staticmethod
    def update_altitude(mainwindow:MainWindow, packet: MAVLink_gps_raw_int_message) -> str:
        mainwindow.next_telemetry.iha_irtifa = packet.alt / 1000
        return str(packet.alt / 1000)
    @staticmethod
    def update_longitude(mainwindow:MainWindow, packet: MAVLink_gps_raw_int_message) -> str:
        mainwindow.next_telemetry.iha_boylam = packet.lon / 1e7
        return str(packet.lon / 1e7)
    @staticmethod
    def update_latitude(mainwindow:MainWindow, packet: MAVLink_gps_raw_int_message) -> str:
        mainwindow.next_telemetry.iha_enlem = packet.lat / 1e7
        return str(packet.lat / 1e7)
    @staticmethod
    def update_yaw(mainwindow:MainWindow, packet: MAVLink_attitude_message) -> str:
        yaw = math.degrees(packet.yaw)
        mainwindow.next_telemetry.iha_yatis = yaw
        return str(yaw)
    @staticmethod
    def update_pitch(mainwindow:MainWindow, packet: MAVLink_attitude_message) -> str:
        pitch = math.degrees(packet.pitch)
        mainwindow.next_telemetry.iha_yonelme = pitch
        return str(pitch)
    @staticmethod
    def update_roll(mainwindow:MainWindow, packet: MAVLink_attitude_message) -> str:
        roll = math.degrees(packet.roll)
        mainwindow.next_telemetry.iha_dikilme = roll
        return str(roll)
    @staticmethod
    def update_gps_time(mainwindow:MainWindow, packet: MAVLink_gps_raw_int_message) -> str:
        datetime = QDateTime.fromMSecsSinceEpoch(packet.time_usec)
        mainwindow.next_telemetry.gps_saati = GpsSaati(datetime.time())
        return datetime.toString()
    @staticmethod
    def update_battery_percentage(mainwindow:MainWindow, packet: MAVLink_battery_status_message) -> str:
        mainwindow.next_telemetry.iha_batarya = packet.battery_remaining
        return str(packet.battery_remaining) + "%"
    @staticmethod
    def update_arm_status(mainwindow:MainWindow, packet: MAVLink_heartbeat_message):
        arm = (packet.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if mainwindow.ui.arm_mode.currentIndex() != arm:
            mainwindow.last_arm_change_was_from_uav = True
            mainwindow.ui.arm_mode.setCurrentIndex(arm)
        index: int = packet.custom_mode
        if index > 9:
            index = index - 1  # FIXME: I hardcoded index after 9
        if mainwindow.ui.fly_mode_combobox.currentIndex() != index:
            mainwindow.last_mode_change_was_from_uav = True
            mainwindow.ui.fly_mode_combobox.setCurrentIndex(index)

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataPacketTimer(Enum):
    # (msg id, msg name, type, update interval (ms), watch value ids that uses this packet)
    BATTERY_STATUS = (147, "BATTERY_STATUS", MAVLink_battery_status_message, 1000000, [10])
    ATTITUDE = (30, "ATTITUDE", MAVLink_attitude_message, 100000, [3, 4, 5])
    GPS_RAW_INT = (24, "GPS_RAW_INT", MAVLink_gps_raw_int_message, 100000, [1, 2, 7, 8, 9])
    VFR_HUD = (74, "VFR_HUD", MAVLink_vfr_hud_message, 100000, [0, 6])
    HEARTBEAT = (0, "HEARTBEAT", MAVLink_heartbeat_message, 100000, [11])

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, is it telemetry data, is it in watch_list widget)
    GROUND_SPEED = (0, "Ground Speed", TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, False, True)
    VELOCITY = (1, "Velocity", TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    ALTITUDE = (2, "Altitude", TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    YAW = (3, "Yaw", TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True, True)
    PITCH = (4, "Pitch", TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True, True)
    ROLL = (5, "Roll", TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True, True)
    AIR_SPEED = (6, "Air Speed", TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, True, True)
    GPS_TIME = (7, "GPS Time", TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.GPS_RAW_INT, True, True)
    LONGITUDE = (8, "Longitude", TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    LATITUDE = (9, "Latitude", TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    BATTERY_PERCENTAGE = (10, "Battery Percentage", TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True, True)
    ARM_STATUS = (11, "Arm Status", TrackableDataUpdate.update_arm_status, TrackableDataPacketTimer.HEARTBEAT, False, False)

    @staticmethod
    def list() -> list[TrackableDataEnum]:
        return list(map(lambda c: c, TrackableDataEnum))

    @staticmethod
    def from_id(i: int) -> TrackableDataEnum:
        e: TrackableDataEnum
        for e in TrackableDataEnum.list():
            if e.value[0] == i:
                return e
        return None

class UAV_Modes(Enum):
    MANUAL = ('MANUAL', 0)
    CIRCLE = ('CIRCLE', 1)
    STABILIZE = ('STABILIZE', 2)
    TRAINING = ('TRAINING', 3)
    ACRO = ('ACRO', 4)
    FLY_BY_WIRE_A = ('FBWA', 5)
    FLY_BY_WIRE_B = ('FBWB', 6)
    CRUISE = ('CRUISE', 7)
    AUTOTUNE = ('AUTOTUNE', 8)
    AUTO = ('AUTO', 10)
    ReturnToLaunch = ('RTL', 11)
    LOITER = ('LOITER', 12)
    TAKEOFF = ('TAKEOFF', 13)
    AVOID_ADSB = ('AVOID_ADSB', 14)
    GUIDED = ('GUIDED', 15)
    INITIALISING = ('INITIALISING', 16)
    QSTABILIZE = ('QSTABILIZE', 17)
    QHOVER = ('QHOVER', 18)
    QLOITER = ('QLOITER', 19)
    QLAND = ('QLAND', 20)
    QRTL = ('QRTL', 21)
    QAUTOTUNE = ('QAUTOTUNE', 22)
    QACRO = ('QACRO', 23)
    THERMAL = ('THERMAL', 24)
    LOITERALTQLAND = ('LOITERALTQLAND', 25)

    @staticmethod
    def list() -> list[UAV_Modes]:
        return list(map(lambda c: c, UAV_Modes))

class UavConnection:
    connection_type: ConnectionType | None = None
    serial_port: int | None = None
    serial_band: int | None = None
    ip: str | None = None

class ServerConnection:
    ip: str | None = None
    port: int
    username: str
    password: str
    team_no: int
    telemetry_timer: QTimer

class MavlinkWorker(QObject):
    parent: MainWindow
    running: bool
    update_watch_list = Signal(int, str)
    def __init__(self, mavlink_connection: mavfile, parent: MainWindow):
        super().__init__()
        self.mavlink_connection = mavlink_connection
        self.parent = parent
        self.watch_list = parent.ui.watch_list
        self.running = False
        self.update_watch_list.connect(self._apply_watch_update)

    def _apply_watch_update(self, row: int, value: str):
        self.watch_list.setItem(row, 3, QTableWidgetItem(value))

    def trigger_update_value(self, e: TrackableDataEnum, packet: MAVLink_message):
        if e.value[5]:
            length: int = self.watch_list.rowCount()
            new_val: str | None = None
            if e.value[4]:
                new_val = e.value[2](self.parent, packet)  # Update if it is telemetry without caring it is in watch list or not
            for i in range(length):
                if self.watch_list.item(i, 0).text() == str(e.value[0]):
                    if new_val is None:
                        new_val = e.value[2](self.parent, packet)
                    self.update_watch_list.emit(i, new_val)
                    break
        else:
            e.value[2](self.parent, packet)

    def run(self):
        while self.running:
            packet: MAVLink_message = self.mavlink_connection.recv_match(blocking=True, timeout=1.0)
            if not self.running:
                break
            if packet is None:
                continue
            if packet.get_type() == "BAD_DATA":
                if all_printable(packet.data):
                    qWarning("Invalid data received: %s" % packet.data)
                else:
                    qWarning("Invalid data received")
                continue
            for e in TrackableDataPacketTimer:
                if e.value[1] == packet.get_type():
                    data_enum_values = e.value[4]
                    for i in data_enum_values:
                        self.trigger_update_value(TrackableDataEnum.from_id(i), packet)

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    geofence_dialog: SetGeofenceInterface | None = None
    add_ads_dialog: AddADSInterface | None = None
    server_connection: ServerConnection = ServerConnection()
    mavlink_connection: mavfile
    force_sending_telemetry: bool = False
    mavlink_worker: MavlinkWorker | None = None
    mavlink_thread: QThread | None = None
    next_telemetry: TelemetryData = TelemetryData()
    last_server_telemetry_response: TelemetryResponseData = TelemetryResponseData()
    last_arm_change_was_from_uav: bool = False
    last_mode_change_was_from_uav: bool = False
    plane_on_map_update_timer: QTimer = QTimer(interval=500)

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        mode: UAV_Modes
        for mode in UAV_Modes.list():
            self.ui.fly_mode_combobox.insertItem(mode.value[1], mode.value[0])
        self.ui.fly_mode_combobox.setCurrentIndex(-1)

        self.ui.actionConfigurate_UAV.triggered.connect(self._actionConfigurate_UAV)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)

        add_to_watch_menu: QMenu = QMenu(parent=self)

        for e in TrackableDataEnum.list():
            action: QAction = QAction(text=e.value[1], parent=self)
            action.triggered.connect(partial(self.add_to_watch_list, e))
            TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]] = action
            add_to_watch_menu.addAction(action)

        self.ui.add_to_watch.setPopupMode(QToolButton.ToolButtonPopupMode.MenuButtonPopup)
        self.ui.add_to_watch.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonIconOnly)
        self.ui.add_to_watch.setMenu(add_to_watch_menu)

        for e in TrackableDataEnum.list():
            if e.value[5]:
                self.add_to_watch_list(e)

        self.ui.remove_from_watch.clicked.connect(self.__remove_from_watch_signal)
        self.ui.watch_list.setColumnHidden(0, True) # hide id column
        self.ui.fly_mode_combobox.currentIndexChanged.connect(self._change_index)
        self.ui.actionForce_Send_Testing_Telemetry_Data.triggered.connect(self.__change_state_for_force_sending_telemetry)
        self.ui.get_kamikaze_coords_from_server.clicked.connect(self.__get_kamikaze_coords)
        self.ui.start_kamikaze.clicked.connect(self.__start_kamikaze)
        self.ui.set_fence.clicked.connect(self.__set_fence_clicked)
        self.server_connection.telemetry_timer = QTimer()
        self.server_connection.telemetry_timer.setInterval(500)
        self.server_connection.telemetry_timer.timeout.connect(self.__send_telemetry)
        self.ui.arm_mode.currentIndexChanged.connect(self.__setArmStatus)
        self.ui.refresh_ads.clicked.connect(self.__refresh_ads)
        self.ui.add_ads.clicked.connect(self.__add_ads_button_clicked)
        self.plane_on_map_update_timer.timeout.connect(self.__update_plane_on_map_without_server)

    def __add_ads_button_clicked(self):
        if not (self.add_ads_dialog is None):
            return
        self.add_ads_dialog = AddADSInterface(self)
        self.add_ads_dialog.show()
        self.add_ads_dialog.ui.add_new.clicked.connect(self.__add_ads_add_new_button_clicked)
        self.add_ads_dialog.ui.buttons.clicked.connect(self.__close_add_ads_dialog)

    def __close_add_ads_dialog(self):
        self.add_ads_dialog.close()
        self.add_ads_dialog = None

    def __add_ads_add_new_button_clicked(self):
        radius = float(self.add_ads_dialog.ui.ads_radius.text())
        latitude = float(self.add_ads_dialog.ui.ads_latitude.text())
        longitude = float(self.add_ads_dialog.ui.ads_longitude.text())
        data = AdsData()
        data.position = QGeoCoordinate(latitude, longitude)
        data.size = radius
        self.ui.map_view.update_ads_data(data)

    def __refresh_ads(self):
        if not self.server_connection.ip:
            return

        self.ui.map_view.update_server_ads_data(get_ads(self.server_connection.ip + ":" + str(self.server_connection.port)))

    last_mode_set_by_user: int = -1

    def __setArmStatus(self, is_arm: int):
        if self.last_arm_change_was_from_uav:
            self.last_arm_change_was_from_uav = False
            return

        is_already_tried_to_set_this_before = self.last_mode_set_by_user == is_arm
        if is_already_tried_to_set_this_before:
            qDebug("Sending armed status with force: %s" % is_arm)
        else:
            qDebug("Trying to send armed status: %s" % is_arm)
        self.last_mode_set_by_user = is_arm

        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0 if is_already_tried_to_set_this_before else 1,  # confirmation
            is_arm,  # param1: 1=arm, 0=disarm
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0  # param7
        )

    def __set_fence_clicked(self):
        if self.geofence_dialog is not None:
            return

        self.geofence_dialog = SetGeofenceInterface(self)

        if self.ui.map_view.coords_for_geofence.gc1_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc1.setText(str(self.ui.map_view.coords_for_geofence.gc1_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc1_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc2_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc2.setText(str(self.ui.map_view.coords_for_geofence.gc2_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc2_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc3_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc3.setText(str(self.ui.map_view.coords_for_geofence.gc3_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc3_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc4_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc4.setText(str(self.ui.map_view.coords_for_geofence.gc4_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc4_v.longitude()))
        self.geofence_dialog.show()
        self.geofence_dialog.ui.save.clicked.connect(self.__set_fence_dialog_save)
        self.geofence_dialog.finished.connect(self.__reset_geofence_dialog)

    def __set_fence_dialog_save(self):
        gc1 = self.geofence_dialog.ui.gc1.text().split()
        gc2 = self.geofence_dialog.ui.gc2.text().split()
        gc3 = self.geofence_dialog.ui.gc3.text().split()
        gc4 = self.geofence_dialog.ui.gc4.text().split()

        if len(gc1) != 2 or len(gc2) != 2 or len(gc3) != 2 or len(gc4) != 2:
            return
        self.ui.map_view.coords_for_geofence.gc1_v = QGeoCoordinate(float(gc1[0]), float(gc1[1]))
        self.ui.map_view.coords_for_geofence.gc2_v = QGeoCoordinate(float(gc2[0]), float(gc2[1]))
        self.ui.map_view.coords_for_geofence.gc3_v = QGeoCoordinate(float(gc3[0]), float(gc3[1]))
        self.ui.map_view.coords_for_geofence.gc4_v = QGeoCoordinate(float(gc4[0]), float(gc4[1]))
        self.ui.map_view.coords_for_geofence.gc_changed.emit()
        qDebug("New geofence coords: %s, %s, %s, %s" % (self.ui.map_view.coords_for_geofence.gc1_v, self.ui.map_view.coords_for_geofence.gc2_v, self.ui.map_view.coords_for_geofence.gc3_v, self.ui.map_view.coords_for_geofence.gc4_v))
        self.geofence_dialog.close()

    def __reset_geofence_dialog(self):
        self.geofence_dialog = None

    def __start_kamikaze(self):
        if self.uav_connection.connection_type is None:
            return
        longitude = float(self.ui.kamikaze_longitude.text())
        latitude = float(self.ui.kamikaze_latitude.text())
        qWarning("Kamikaze not implemented yet") # TODO

    def __get_kamikaze_coords(self):
        if self.server_connection.ip is None:
            return
        qr_coords: QrCoords = get_kamikaze_coords(self.server_connection.ip + ":" + str(self.server_connection.port))
        self.ui.kamikaze_longitude.setText(str(qr_coords.qrBoylam))
        self.ui.kamikaze_latitude.setText(str(qr_coords.qrEnlem))

    def __change_state_for_force_sending_telemetry(self, state: bool):
        self.force_sending_telemetry = state

    def _change_index(self, index: int):
        if self.last_mode_change_was_from_uav:
            self.last_mode_change_was_from_uav = False
            return
        if index > 8:
            index = index + 1
        qDebug("Sending mode with index: %s" % index)
        self.mavlink_connection.set_mode_apm(index)
        self.next_telemetry.iha_otonom = index == 10

    def add_to_watch_list(self, e: TrackableDataEnum):
        if not TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].isEnabled():
            return
        rowCount: int = self.ui.watch_list.rowCount()
        self.ui.watch_list.setRowCount(rowCount + 1)

        self.ui.watch_list.setItem(rowCount, 0, QTableWidgetItem(str(e.value[0])))
        self.ui.watch_list.setItem(rowCount, 1, QTableWidgetItem(e.value[1]))
        self.ui.watch_list.setItem(rowCount, 2, QTableWidgetItem(""))
        self.ui.watch_list.setItem(rowCount, 3, QTableWidgetItem("Unknown"))

        TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].setDisabled(True)

    def __remove_from_watch_signal(self):
        indexes: list[QModelIndex] = self.ui.watch_list.selectedIndexes()
        if indexes is not None:
            indexes.sort()

            rows: list[int] = list()
            i: int = 0
            for index in indexes:
                row: int = index.row()
                if row in rows:
                    continue
                rows.append(row)
                target_row: int = row - i

                if target_row > self.ui.watch_list.rowCount() or target_row < 0:
                    qWarning("You broke something, wrong target row found when trying to remove. Skipping row number %s" % target_row)
                    continue
                data_id = int(self.ui.watch_list.item(target_row, 0).text())

                TRACKABLE_DATA_ENUM_ACTIONS[data_id].setDisabled(False)
                self.ui.watch_list.removeRow(target_row)
                i = i + 1

    def _actionConfigurateServer(self):
        if self.server_connection_dialog is not None:
            return
        self.server_connection_dialog = ServerConnectionInterface(self)
        self.server_connection_dialog.show()
        if self.server_connection.ip is not None:
            self.server_connection_dialog.ui.server_ip_input.setText(self.server_connection.ip)
            self.server_connection_dialog.ui.server_port_input.setText(str(self.server_connection.port))
            self.server_connection_dialog.ui.server_login_username_input.setText(str(self.server_connection.username))
            self.server_connection_dialog.ui.server_login_password_input.setText(str(self.server_connection.password))
        self.server_connection_dialog.ui.connect.clicked.connect(lambda button: self._server_connect(button, self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect())
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    def _actionConfigurate_UAV(self):
        if self.uav_connection_dialog is not None:
            return
        self.uav_connection_dialog = FightingUAVConnectionInterface(self)
        self.uav_connection_dialog.show()
        if self.uav_connection.connection_type is not None:
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                self.uav_connection_dialog.ui.serial_band.setText(str(self.uav_connection.serial_band))
                self.uav_connection_dialog.ui.connection_type.setCurrentIndex(2 + self.uav_connection.serial_port)
            else:
                isTCP: bool = self.uav_connection_dialog.connection_type == ConnectionType.TCP
                self.uav_connection_dialog.ui.ip_address.setText(self.uav_connection.ip)
                if isTCP:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(0)
                else:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(1)
        self.uav_connection_dialog.ui.connect.clicked.connect(self._uav_connect)
        self.uav_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._uav_disconnect(button, self.uav_connection_dialog))
        self.uav_connection_dialog.finished.connect(lambda e: self._reset_dialog(True))

    def _reset_dialog(self, is_uav: bool):
        if is_uav:
            self.uav_connection_dialog = None
        else:
            self.server_connection_dialog = None

    def _uav_connect(self):
        if self.uav_connection_dialog.connection_type != ConnectionType.SERIAL:
            if not self.uav_connection_dialog.ui.ip_address.text():
                self.uav_connection_dialog.ui.ip_address.setText(self.uav_connection_dialog.ui.ip_address.placeholderText())
            if self.is_ip_address_valid(self.uav_connection_dialog.ui.ip_address.text(), True):
                self.uav_connection_dialog.ui.invalid_input_error_label.hide()
            else:
                self.uav_connection_dialog.ui.invalid_input_error_label.show()
                return
        else:
            pass # TODO: Serial connection validation

        if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
            self.uav_connection.serial_band = self.uav_connection_dialog.ui.serial_band.text()
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()

        try:
            self.uav_connection_dialog.ui.device_connection_text.setText("Trying to connect device :O")
            match self.uav_connection_dialog.connection_type:
              case ConnectionType.TCP:
                  self.mavlink_connection = mavtcp(self.uav_connection.ip, retries=1)
              case ConnectionType.UDP:
                  self.mavlink_connection = mavudp(self.uav_connection.ip, timeout=10)
              case ConnectionType.SERIAL:
                  self.mavlink_connection = mavserial(self.uav_connection.serial_port, baud=self.uav_connection.serial_band)
              case None:
                  qWarning("Connection type is null ???")
                  return
            qInfo("Successfully Connected with mavlink, Target System: %s, Target component: %s" % (self.mavlink_connection.target_system, self.mavlink_connection.target_component))
        except OSError as e:
            self.uav_connection_dialog.ui.device_connection_text.setText("Device Connection Failed :(")
            qWarning("Tried a invalid connection: %s" % str(e))
            return

        self.mavlink_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

        try:
            msg: MAVLink_heartbeat_message = self.mavlink_connection.wait_heartbeat(timeout=10)
            if msg is None:
                self.uav_connection_dialog.ui.device_connection_text.setText("Device Connection Failed :(")
                self.uav_connection_dialog.ui.invalid_input_error_label.show()
                self.uav_connection.serial_band = None
                self.uav_connection.serial_port = None
                self.uav_connection.ip = None
                self.uav_connection.connection_type = None
                self.ui.device_connection_warning.show()
                self.mavlink_connection.close()
                self.ui.map_view.mavlink_connection = None
                qWarning("Can not receive heartbeat")
                return

            qInfo("Successfully Received first heartbeat")
        except:
            self.uav_connection_dialog.ui.device_connection_text.setText("Device Connection Failed :(")

            self.uav_connection_dialog.ui.invalid_input_error_label.show()
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                qInfo("Can not connect to UAV from %s" % (str(self.uav_connection.serial_port) + "," + str(self.uav_connection.serial_band)))
            else:
                qInfo("Can not connect to UAV from %s" % self.uav_connection.ip)
            self.uav_connection.serial_band = None
            self.uav_connection.serial_port = None
            self.uav_connection.ip = None
            self.uav_connection.connection_type = None
            self.ui.device_connection_warning.show()
            self.mavlink_connection.close()
            self.ui.map_view.mavlink_connection = None
            return
        self.uav_connection_dialog.ui.device_connection_text.setText("Device Connected :)")
        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type # Only set connection_type after successfully connecting
        self.mavlink_connection.mav.request_data_stream_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )
        for e in TrackableDataPacketTimer:
            self.mavlink_connection.mav.send(self.mavlink_connection.mav.command_long_encode(self.mavlink_connection.target_system,
                                                            self.mavlink_connection.target_component,
                                                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                            0,
                                                            e.value[0],
                                                            e.value[3],
                                                            0, 0, 0, 0, 0))

        self.ui.device_connection_warning.hide()
        self.ui.map_view.mavlink_connection = self.mavlink_connection
        self.mavlink_thread = QThread(self)
        self.mavlink_thread.setObjectName("Mavlink Connection Thread")
        self.mavlink_worker = MavlinkWorker(self.mavlink_connection, self)
        self.mavlink_worker.running = True
        self.mavlink_thread.started.connect(self.mavlink_worker.run)
        self.mavlink_worker.moveToThread(self.mavlink_thread)
        self.mavlink_thread.start()
        self.ui.arm_mode.setEnabled(True)
        if self.server_connection.ip is None:
            self.plane_on_map_update_timer.start()

    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        if self.uav_connection.connection_type is None:
            return
        self.mavlink_worker.running = False
        self.mavlink_thread.quit()
        self.mavlink_thread.wait()
        self.mavlink_connection.close()
        self.uav_connection.connection_type = None
        self.ui.arm_mode.setEnabled(False)

    def _server_connect(self, button: QPushButton, dialog: ServerConnectionInterface):
        # TODO: Test connection
        if not dialog.ui.server_ip_input.text():
            dialog.ui.server_ip_input.setText(dialog.ui.server_ip_input.placeholderText())
        if not dialog.ui.server_port_input.text():
            dialog.ui.server_port_input.setText(dialog.ui.server_port_input.placeholderText())
        if not dialog.ui.server_login_username_input.text():
            dialog.ui.server_login_username_input.setText(dialog.ui.server_login_username_input.placeholderText())
        if not dialog.ui.server_login_password_input.text():
            dialog.ui.invalid_input_error_label.show()
            return
        dialog.ui.invalid_input_error_label.hide()
        self.server_connection.ip = dialog.ui.server_ip_input.text()
        self.server_connection.port = int(dialog.ui.server_port_input.text())
        self.server_connection.username = dialog.ui.server_login_username_input.text()
        self.server_connection.password = dialog.ui.server_login_password_input.text()

        try:
            dialog.ui.server_connection_text.setText("Trying to connect to server :O")
            self.server_connection.team_no = login_to_server(self.server_connection.ip + ":" + str(self.server_connection.port), self.server_connection.username, self.server_connection.password)
            self.next_telemetry.takim_numarasi = self.server_connection.team_no
            dialog.ui.server_connection_text.setText("Server Connected :)")
            self.ui.server_connection_warning.hide()
        except Exception as e:
            dialog.ui.server_connection_text.setText("Can't Connect To Server :(")
            qWarning("Can not connect to server: %s" % e)
            dialog.ui.invalid_input_error_label.show()
            self.ui.server_connection_warning.show()
            self.server_connection.ip = None
            return
        if self.plane_on_map_update_timer.isActive():
            self.plane_on_map_update_timer.stop()
        self.server_connection.telemetry_timer.start()

    def __update_plane_on_map_without_server(self):
        if self.uav_connection.connection_type is None or self.server_connection.ip is not None:
            self.plane_on_map_update_timer.stop()
            return
        self.ui.map_view.update_plane_data_without_server(QGeoCoordinate(self.next_telemetry.iha_enlem, self.next_telemetry.iha_boylam), self.next_telemetry.iha_yatis)

    def __send_telemetry(self):
        if self.uav_connection.connection_type is None and not self.force_sending_telemetry:
            qDebug("UAV not connected")
            return
        if ServerConnection.SERVER_IS_UNREACHABLE_COUNTER > 100:
            ServerConnection.SERVER_IS_UNREACHABLE_COUNTER = 0
            self._server_disconnect()
            qWarning("Server connection is not possible for 100 time, disconnecting")
            return
        self.last_server_telemetry_response = send_telemetry(self.server_connection.ip + ":" + str(self.server_connection.port),
                                                                    self.next_telemetry)
        self.ui.map_view.update_plane_data(self.next_telemetry.takim_numarasi, self.last_server_telemetry_response)

    def _server_disconnect(self):
        self.server_connection.telemetry_timer.stop()
        self.server_connection.ip = None

    @staticmethod
    def is_ip_address_valid(ip_address: str, must_have_port: bool) -> bool:
        if ip_address is not None:
            ip_address = ip_address.strip()
            ip_with_port: str = ip_address
            split: list[str] = ip_with_port.split(':')
            ip: str = split[0]
            if len(split) > 1:
                port: int = int(split[1])
                if port < 0 or port > 65535:
                    return False
            elif must_have_port:
                return False # Must have port
            ip_array: list[str] = ip.split('.')
            if len(ip_array) != 4:
                return False
            for e in ip_array:
                e: int = int(e)
                if e < 0 or e > 255:
                    return False
            return True
        return False

