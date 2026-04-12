import math
import re
from enum import Enum
from functools import partial

from PySide6.QtCore import Qt, QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug, QThread, QObject, Signal, Slot, \
    QLocale, QTranslator, QCoreApplication, QSize
from PySide6.QtGui import QAction, QIcon
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import QMainWindow, QPushButton, QToolButton, QTableWidgetItem, QMenu, QTableWidget, QDialog, \
    QComboBox, QApplication
from pymavlink import mavutil
from pymavlink.dialects.v10.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message, MAVLink_message, MAVLink_heartbeat_message, \
    MAVLink_global_position_int_message
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src.AddADSInterface import AddADSInterface
from src.CameraServerConnectionInterface import CameraServerConnectionInterface
from src.ColorSelectorInterface import ColorSelectorInterface
from src.MapWidget import ZERO_GEO_COORDS, AdsData
from src.SetGeofenceInterface import SetGeofenceInterface
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, QrCoords, \
    get_kamikaze_coords, TelemetryData, TelemetryResponseData, get_ads
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.uav_interface import Ui_MainWindow

def to_degree(x: float) -> float:
    if x < 0:
        x = x + 2 * math.pi
    return x * (180 / math.pi)

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
    def update_relative_altitude(mainwindow:MainWindow, packet: MAVLink_global_position_int_message) -> str:
        mainwindow.next_telemetry.iha_irtifa = packet.relative_alt / 1000
        return str(packet.relative_alt / 1000)
    @staticmethod
    def update_altitude(mainwindow:MainWindow, packet: MAVLink_global_position_int_message) -> str:
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
        yaw = to_degree(packet.yaw)
        mainwindow.next_telemetry.iha_yatis = (yaw - 180) / 4
        return str(yaw)
    @staticmethod
    def update_pitch(mainwindow:MainWindow, packet: MAVLink_attitude_message) -> str:
        pitch = to_degree(packet.pitch)
        mainwindow.next_telemetry.iha_yonelme = pitch
        return str(pitch)
    @staticmethod
    def update_roll(mainwindow:MainWindow, packet: MAVLink_attitude_message) -> str:
        roll = to_degree(packet.roll)
        mainwindow.next_telemetry.iha_dikilme = (roll - 180) / 4
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
        return str(arm)

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataPacketTimer(Enum):
    # (msg id, msg name, type, update interval (ms), watch value ids that uses this packet)
    BATTERY_STATUS = (147, "BATTERY_STATUS", MAVLink_battery_status_message, 1000000, [10])
    ATTITUDE = (30, "ATTITUDE", MAVLink_attitude_message, 100000, [3, 4, 5])
    GPS_RAW_INT = (24, "GPS_RAW_INT", MAVLink_gps_raw_int_message, 100000, [1, 7, 8, 9])
    VFR_HUD = (74, "VFR_HUD", MAVLink_vfr_hud_message, 100000, [0, 6])
    HEARTBEAT = (0, "HEARTBEAT", MAVLink_heartbeat_message, 100000, [11])
    GLOBAL_POSITION_INT = (33, "GLOBAL_POSITION_INT", MAVLink_global_position_int_message, 100000, [2, 12])

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, should be updated on background (telemetry etc), is it in watch_list widget)
    GROUND_SPEED = (0, lambda: QCoreApplication.translate("TrackableDataEnum", "Ground Speed", None), TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, False, True)
    VELOCITY = (1, lambda: QCoreApplication.translate("TrackableDataEnum", "Velocity", None), TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    ALTITUDE = (2, lambda: QCoreApplication.translate("TrackableDataEnum", "Altitude", None), TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, False)
    YAW = (3, lambda: QCoreApplication.translate("TrackableDataEnum", "Yaw", None), TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True, True)
    PITCH = (4, lambda: QCoreApplication.translate("TrackableDataEnum", "Pitch", None), TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True, True)
    ROLL = (5, lambda: QCoreApplication.translate("TrackableDataEnum", "Roll", None), TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True, True)
    AIR_SPEED = (6, lambda: QCoreApplication.translate("TrackableDataEnum", "Air Speed", None), TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, True, True)
    GPS_TIME = (7, lambda: QCoreApplication.translate("TrackableDataEnum", "GPS Time", None), TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.GPS_RAW_INT, True, True)
    LONGITUDE = (8, lambda: QCoreApplication.translate("TrackableDataEnum", "Longitude", None), TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    LATITUDE = (9, lambda: QCoreApplication.translate("TrackableDataEnum", "Latitude", None), TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    BATTERY_PERCENTAGE = (10, lambda: QCoreApplication.translate("TrackableDataEnum", "Battery Percentage", None), TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True, True)
    ARM_STATUS = (11, lambda: QCoreApplication.translate("TrackableDataEnum", "Arm Status", None), TrackableDataUpdate.update_arm_status, TrackableDataPacketTimer.HEARTBEAT, True, False)
    RELATIVE_ALTITUDE = (12, lambda: QCoreApplication.translate("TrackableDataEnum", "Relative Altitude", None), TrackableDataUpdate.update_relative_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, True)

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
    AUTOLAND = ('AUTOLAND', 26)

    @staticmethod
    def list() -> list[UAV_Modes]:
        return list(map(lambda c: c, UAV_Modes))

class SupportedLanguages(Enum):
    English = (0, lambda: QCoreApplication.translate("SupportedLanguages", "English", None), QLocale.Language.English, QLocale.Country.UnitedStates)
    Turkish = (1, lambda: QCoreApplication.translate("SupportedLanguages", "Turkish", None), QLocale.Language.Turkish, QLocale.Country.Turkey)
    @staticmethod
    def from_id(i: int) -> SupportedLanguages:
        e: SupportedLanguages
        for e in SupportedLanguages:
            if e.value[0] == i:
                return e
        return None

LANGUAGE_ACTIONS: dict[int, QAction] = {}

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

SERVER_IS_UNREACHABLE_COUNTER = 0

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    camera_server_connection_dialog: CameraServerConnectionInterface | None = None
    color_selector_dialog: ColorSelectorInterface | None = None
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
    current_lang: int = 0

    def __init__(self):
        QMainWindow.__init__(self)
        self.translator = QTranslator()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        mode: UAV_Modes
        for mode in UAV_Modes.list():
            self.ui.fly_mode_combobox.insertItem(mode.value[1], mode.value[0])
        self.ui.fly_mode_combobox.setCurrentIndex(-1)

        self.ui.actionConfigurate_UAV.triggered.connect(self._actionConfigurate_UAV)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)
        self.ui.actionSet_Colors.triggered.connect(self._actionConfigurateSetColors)

        add_to_watch_menu: QMenu = QMenu(parent=self)

        for e in TrackableDataEnum.list():
            action: QAction = QAction(text=e.value[1](), parent=self)
            action.setObjectName(str(e.value[0]))
            action.triggered.connect(partial(self.add_to_watch_list, e))
            TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]] = action
            add_to_watch_menu.addAction(action)

        self.ui.add_to_watch.setPopupMode(QToolButton.ToolButtonPopupMode.MenuButtonPopup)
        self.ui.add_to_watch.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonIconOnly)
        self.ui.add_to_watch.setMenu(add_to_watch_menu)

        for e in TrackableDataEnum.list():
            if e.value[5]:
                self.add_to_watch_list(e)

        for lang in SupportedLanguages:
            lang_id = lang.value[0]
            action: QAction = QAction(lang.value[1](), self)
            action.setObjectName(str(lang_id))
            action.setCheckable(True)
            action.triggered.connect(partial(self.change_lang_to, lang_id))
            LANGUAGE_ACTIONS[lang_id] = action
            self.ui.menuChange_Language.addAction(action)
        LANGUAGE_ACTIONS[0].setChecked(True)

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
        self.ui.start_stop_camera_view.toggled.connect(self.__start_stop_camera_view)
        self.ui.actionConfigurate_Camera_Stream.triggered.connect(self._actionConfigurateCameraServer)
        self.ui.remove_ads.clicked.connect(self._remove_ads)

    def _remove_ads(self):
        for m_data in self.ui.map_view.user_ads_data_model.m_datas:
            if m_data.is_selected:
                self.ui.map_view.user_ads_data_model.m_datas.remove(m_data)
                self.ui.map_view.user_ads_data_model.layoutChanged.emit()

    def _actionConfigurateCameraServer(self):
        if self.camera_server_connection_dialog is not None:
            return
        self.camera_server_connection_dialog = CameraServerConnectionInterface(self)
        self.camera_server_connection_dialog.show()
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Connected :)", None))
            self.camera_server_connection_dialog.ui.server_ip_input.setText(self.ui.camera_view.camera_server_info.ip)
            self.camera_server_connection_dialog.ui.server_port_input.setText(str(self.ui.camera_view.camera_server_info.port))
            self.camera_server_connection_dialog.ui.server_protocol_type.setCurrentIndex(self.ui.camera_view.camera_server_info.protocol.value[0])
        self.camera_server_connection_dialog.ui.connect.clicked.connect(self.connect_to_cam_server)
        self.camera_server_connection_dialog.ui.disconnect.clicked.connect(self.disconnect_from_cam_server)
        self.camera_server_connection_dialog.finished.connect(self.reset_camera_server_dialog)

    def connect_to_cam_server(self):
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.ui.camera_view.disconnect_from_server()
        self.ui.camera_view.camera_server_info.ip = self.camera_server_connection_dialog.ui.server_ip_input.text()
        self.ui.camera_view.camera_server_info.port = int(self.camera_server_connection_dialog.ui.server_port_input.text())
        self.ui.camera_view.set_protocol(self.camera_server_connection_dialog.ui.server_protocol_type.currentIndex())
        if self.ui.camera_view.connect_to_server():
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Connected :)", None))
            self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(False)
            self.ui.start_stop_camera_view.setCheckable(True)
            self.ui.record_button.setCheckable(True)
        else:
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Not Connected :(", None))
            self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
            self.ui.start_stop_camera_view.setCheckable(False)
            self.ui.record_button.setCheckable(False)

    def disconnect_from_cam_server(self):
        self.ui.camera_view.disconnect_from_server()
        self.ui.camera_view.set_no_connection_image()
        self.ui.camera_view.camera_server_info.ip = None
        self.ui.start_stop_camera_view.setCheckable(False)
        self.ui.record_button.setCheckable(False)

    def reset_camera_server_dialog(self):
        self.camera_server_connection_dialog = None

    def __start_stop_camera_view(self, b: bool) -> None:
        if b:
            self.ui.camera_view.on_play()
        else:
            self.ui.camera_view.on_pause()

    def retranslateWatcher(self):
        length: int = self.ui.watch_list.rowCount()
        for i in range(length):
            tde = TrackableDataEnum.from_id(int(self.ui.watch_list.item(i, 0).text()))

            self.ui.watch_list.setItem(i, 1, QTableWidgetItem(tde.value[1]()))
            if self.uav_connection.connection_type is None:
                self.ui.watch_list.setItem(i, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))
        for e in TRACKABLE_DATA_ENUM_ACTIONS.values():
            tde = TrackableDataEnum.from_id(int(e.objectName()))

            e.setText(tde.value[1]())

    def retranslateOpenDialogs(self):
        dialogs = [self.uav_connection_dialog,
                   self.server_connection_dialog,
                   self.color_selector_dialog,
                   self.geofence_dialog,
                   self.add_ads_dialog,
                   self.camera_server_connection_dialog]

        for dialog in dialogs:
            if dialog is not None:
                dialog.ui.retranslateUi(dialog)

    translator: QTranslator
    def change_lang_to(self, index: int):
        slang = SupportedLanguages.from_id(index)
        LANGUAGE_ACTIONS[self.current_lang].setChecked(False)
        self.current_lang = index
        locale: QLocale = QLocale(slang.value[2], slang.value[3])
        QLocale.setDefault(locale)
        QApplication.removeTranslator(self.translator)
        qDebug("Changing Language to %s" % locale.name())
        if self.translator.load(locale, "ui", "_", "ui_files/translations"):
            if QApplication.installTranslator(self.translator):
                self.ui.retranslateUi(self)
                self.retranslateWatcher()
                self.retranslateOpenDialogs()
                LANGUAGE_ACTIONS[index].setChecked(True)
                for s_lang in SupportedLanguages:
                    LANGUAGE_ACTIONS[s_lang.value[0]].setText(s_lang.value[1]())
            else:
                qWarning("Could not install translator")
        else:
            qWarning("Could not load translation to %s!" % self.translator.language())

    def __add_ads_button_clicked(self):
        if self.add_ads_dialog is not None:
            return
        self.add_ads_dialog = AddADSInterface(self)
        self.add_ads_dialog.show()
        self.add_ads_dialog.ui.add_new.clicked.connect(self.__add_ads_add_new_button_clicked)
        self.add_ads_dialog.ui.buttons.clicked.connect(self.__close_add_ads_dialog)
        self.add_ads_dialog.finished.connect(self.set_ads_dialog_to_none)

    def __close_add_ads_dialog(self):
        self.add_ads_dialog.close()
        self.add_ads_dialog = None

    def set_ads_dialog_to_none(self):
        self.add_ads_dialog = None

    def __add_ads_add_new_button_clicked(self):
        radius = float(self.add_ads_dialog.ui.ads_radius.text())
        latitude = float(self.add_ads_dialog.ui.ads_latitude.text())
        longitude = float(self.add_ads_dialog.ui.ads_longitude.text())
        data = AdsData()
        data.position = QGeoCoordinate(latitude, longitude)
        data.size = radius
        data.is_selected = False
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
        self.next_telemetry.iha_otonom = 1 if index == 10 else 0

    def add_to_watch_list(self, e: TrackableDataEnum):
        if not TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].isEnabled():
            return
        rowCount: int = self.ui.watch_list.rowCount()
        self.ui.watch_list.setRowCount(rowCount + 1)

        self.ui.watch_list.setItem(rowCount, 0, QTableWidgetItem(str(e.value[0])))
        self.ui.watch_list.setItem(rowCount, 1, QTableWidgetItem(e.value[1]()))
        self.ui.watch_list.setItem(rowCount, 2, QTableWidgetItem(""))
        self.ui.watch_list.setItem(rowCount, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))

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
            self.server_connection_dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Server Connected :)", None))
            self.server_connection_dialog.ui.server_ip_input.setText(self.server_connection.ip)
            self.server_connection_dialog.ui.server_port_input.setText(str(self.server_connection.port))
            self.server_connection_dialog.ui.server_login_username_input.setText(str(self.server_connection.username))
            self.server_connection_dialog.ui.server_login_password_input.setText(str(self.server_connection.password))
        self.server_connection_dialog.ui.connect.clicked.connect(lambda button: self._server_connect(button, self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect())
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    def _actionConfigurateSetColors(self):
        if self.color_selector_dialog is not None:
            return
        self.color_selector_dialog = ColorSelectorInterface(self)
        self.color_selector_dialog.show()

    def _actionConfigurate_UAV(self):
        if self.uav_connection_dialog is not None:
            return
        self.uav_connection_dialog = FightingUAVConnectionInterface(self)
        self.uav_connection_dialog.show()
        availablePorts = list(QSerialPortInfo.availablePorts())
        availablePorts.sort(key=lambda a: int(re.sub("\\D", "", a.portName())))
        for availablePort in availablePorts:
            self.uav_connection_dialog.ui.connection_type.addItem(availablePort.systemLocation())
        if self.uav_connection.connection_type is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                self.uav_connection_dialog.ui.serial_baud.setText(str(self.uav_connection.serial_band))
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
            if not self.uav_connection_dialog.ui.serial_baud.text():
                self.uav_connection_dialog.ui.serial_baud.setText(self.uav_connection_dialog.ui.serial_baud.placeholderText())
            # TODO: Serial connection validation

        if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
            self.uav_connection.serial_band = self.uav_connection_dialog.ui.serial_baud.text()
            self.uav_connection.serial_port = self.uav_connection_dialog.ui.connection_type.currentIndex() - 2
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()

        try:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Trying to connect device :O", None))
            match self.uav_connection_dialog.connection_type:
              case ConnectionType.TCP:
                  self.mavlink_connection = mavtcp(self.uav_connection.ip, retries=1)
              case ConnectionType.UDP:
                  self.mavlink_connection = mavudp(self.uav_connection.ip, timeout=10)
              case ConnectionType.SERIAL:
                  system_identifier = re.sub("\\d", "", QSerialPortInfo.availablePorts()[0].systemLocation())
                  self.mavlink_connection = mavserial(system_identifier + str(self.uav_connection.serial_port), baud=self.uav_connection.serial_band)
              case None:
                  qWarning("Connection type is null ???")
                  return
            qInfo("Successfully Connected with mavlink, Target System: %s, Target component: %s" % (self.mavlink_connection.target_system, self.mavlink_connection.target_component))
        except OSError as e:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))
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
                raise Exception("Connection failed")

            qInfo("Successfully Received first heartbeat")
        except:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))

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
        self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
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

        self.ui.map_view.mavlink_connection = self.mavlink_connection
        self.mavlink_thread = QThread(self)
        self.mavlink_thread.setObjectName("Mavlink Connection Thread")
        self.mavlink_worker = MavlinkWorker(self.mavlink_connection, self)
        self.mavlink_worker.running = True
        self.mavlink_thread.started.connect(self.mavlink_worker.run)
        self.mavlink_worker.moveToThread(self.mavlink_thread)
        self.mavlink_thread.start()
        self.enableFeaturesAfterUAVConnected()

    def enableFeaturesAfterUAVConnected(self):
        self.ui.arm_mode.setEnabled(True)
        self.ui.fly_mode_combobox.setEnabled(True)
        self.ui.device_connection_warning.hide()
        if self.server_connection.ip is None:
            self.plane_on_map_update_timer.start()

    def disableFeaturesAfterUAVDisconnected(self):
        self.ui.arm_mode.setEnabled(False)
        self.ui.fly_mode_combobox.setEnabled(False)
        self.ui.device_connection_warning.show()
        if self.plane_on_map_update_timer.isActive():
            self.plane_on_map_update_timer.stop()


    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        if self.uav_connection.connection_type is None:
            return
        self.mavlink_worker.running = False
        self.mavlink_thread.quit()
        self.mavlink_thread.wait()
        self.mavlink_connection.close()
        self.uav_connection.connection_type = None
        self.disableFeaturesAfterUAVDisconnected()

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
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Trying to connect to server :O", None))
            self.server_connection.team_no = login_to_server(self.server_connection.ip + ":" + str(self.server_connection.port), self.server_connection.username, self.server_connection.password)
            self.next_telemetry.takim_numarasi = self.server_connection.team_no
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Server Connected :)", None))
            self.ui.server_connection_warning.hide()
        except Exception as e:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Can't Connect To Server :(", None))
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
        self.ui.map_view.update_plane_data_without_server(QGeoCoordinate(self.next_telemetry.iha_enlem, self.next_telemetry.iha_boylam), (self.next_telemetry.iha_yatis * 4) + 180)

    def __send_telemetry(self):
        if self.uav_connection.connection_type is None and not self.force_sending_telemetry:
            qDebug("UAV not connected")
            return
        global SERVER_IS_UNREACHABLE_COUNTER
        if SERVER_IS_UNREACHABLE_COUNTER > 100:
            SERVER_IS_UNREACHABLE_COUNTER = 0
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
                return False # Must have port, yes that's the comment :3
            ip_array: list[str] = ip.split('.')
            if len(ip_array) != 4:
                return False
            for e in ip_array:
                e: int = int(e)
                if e < 0 or e > 255:
                    return False
            return True
        return False

