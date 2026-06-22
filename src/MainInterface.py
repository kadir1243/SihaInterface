import math
import os
import re
from enum import Enum
from functools import partial

os.environ['MAVLINK20'] = '1'

from PySide6.QtCore import QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug, QThread, QObject, Signal, QLocale, QTranslator, QCoreApplication
from PySide6.QtGui import QAction
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import QMainWindow, QTableWidgetItem, QMenu, QApplication, QMessageBox
from pymavlink.dialects.v20.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message, MAVLink_message, MAVLink_heartbeat_message, \
    MAVLink_global_position_int_message, MAVLink_system_time_message, MAV_CMD_DO_FENCE_ENABLE, \
    MAVLink_fence_status_message, \
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, MAV_FRAME_GLOBAL_INT, \
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAV_MISSION_TYPE_FENCE, \
    MAVLINK_MSG_ID_MISSION_ACK, MAVLINK_MSG_ID_MISSION_REQUEST, MAV_FRAME_GLOBAL, MAVLINK_MSG_ID_BAD_DATA, \
    MAVLINK_MSG_ID_COMMAND_ACK, MAV_CMD_DO_REPOSITION, MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, \
    MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_ITEM_INT, MAVLink_mission_item_int_message, \
    MAV_MISSION_ACCEPTED, MAVLINK_MSG_ID_MISSION_ITEM, MAVLink_mission_item_message, MAV_MODE_FLAG_SAFETY_ARMED, \
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_DATA_STREAM_ALL, \
    MAV_CMD_SET_MESSAGE_INTERVAL, MAV_MISSION_TYPE_MISSION
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src.AddADSInterface import AddADSInterface
from src.CameraServerConnectionInterface import CameraServerConnectionInterface
from src.ColorSelectorInterface import ColorSelectorInterface, ColorOptions
from src.MapWidget import ZERO_GEO_COORDS, AdsData, SpecialCoordsData
from src.SetGeofenceInterface import SetGeofenceInterface
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, QrCoords, \
    get_kamikaze_coords, TelemetryData, TelemetryResponseData, get_ads, send_kamikaze
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
    def update_gps_time(mainwindow:MainWindow, packet: MAVLink_system_time_message) -> str:
        datetime = QDateTime.fromMSecsSinceEpoch(int(packet.time_unix_usec / 1000))
        mainwindow.next_telemetry.gps_saati = GpsSaati(datetime.time())
        return datetime.toString()
    @staticmethod
    def update_battery_percentage(mainwindow:MainWindow, packet: MAVLink_battery_status_message) -> str:
        mainwindow.next_telemetry.iha_batarya = packet.battery_remaining
        return str(packet.battery_remaining) + "%"
    @staticmethod
    def update_arm_status(mainwindow:MainWindow, packet: MAVLink_heartbeat_message):
        arm = (packet.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if mainwindow.ui.arm_mode.currentIndex() != arm:
            mainwindow.last_arm_change_was_from_uav = True
            mainwindow.ui.arm_mode.setCurrentIndex(arm)
        index: int = packet.custom_mode
        if index > 9:
            index = index - 1  # FIXME: I hardcoded index after 9
        if mainwindow.ui.fly_mode_combobox.currentIndex() != index:
            mainwindow.last_mode_change_was_from_uav = True
            mainwindow.ui.fly_mode_combobox.setCurrentIndex(index)
        if packet.custom_mode != 15 and mainwindow.ui.map_view.target_coord.is_set and not mainwindow.ui.map_view.reposition_timer.isActive():
            mainwindow.ui.map_view.target_coord.remove_position()
        return str(arm)
    @staticmethod
    def update_breach_status(mainwindow:MainWindow, packet: MAVLink_fence_status_message) -> str:
        text = "Breached"
        if packet.breach_status == 0:
            text = "Not " + text
        return text

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

MSG_ID_2_TRACKABLE_DATA_TYPE: dict[int, TrackableDataPacketTimer] = {}

class TrackableDataPacketTimer(Enum):
    # (msg id, msg name, type, update interval (microsecond), watch value ids that uses this packet)
    BATTERY_STATUS = (147, "BATTERY_STATUS", MAVLink_battery_status_message, 1000000, [10])
    ATTITUDE = (30, "ATTITUDE", MAVLink_attitude_message, 100000, [3, 4, 5])
    GPS_RAW_INT = (24, "GPS_RAW_INT", MAVLink_gps_raw_int_message, 100000, [1, 8, 9])
    VFR_HUD = (74, "VFR_HUD", MAVLink_vfr_hud_message, 100000, [0, 6])
    HEARTBEAT = (0, "HEARTBEAT", MAVLink_heartbeat_message, 100000, [11])
    GLOBAL_POSITION_INT = (33, "GLOBAL_POSITION_INT", MAVLink_global_position_int_message, 100000, [2, 12])
    SYSTEM_TIME = (2, "SYSTEM_TIME", MAVLink_system_time_message, 1000000, [7])
    FENCE_STATUS = (162, "FENCE_STATUS", MAVLink_fence_status_message, 1000000, [13])

for ____trackable_data_packet_timer in TrackableDataPacketTimer:
    MSG_ID_2_TRACKABLE_DATA_TYPE[____trackable_data_packet_timer.value[0]] = ____trackable_data_packet_timer

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, should be updated on background (telemetry etc), is it in watch_list widget)
    GROUND_SPEED = (0, lambda: QCoreApplication.translate("TrackableDataEnum", "Ground Speed", None), TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, False, True)
    VELOCITY = (1, lambda: QCoreApplication.translate("TrackableDataEnum", "Velocity", None), TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    ALTITUDE = (2, lambda: QCoreApplication.translate("TrackableDataEnum", "Altitude", None), TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, False)
    YAW = (3, lambda: QCoreApplication.translate("TrackableDataEnum", "Yaw", None), TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True, True)
    PITCH = (4, lambda: QCoreApplication.translate("TrackableDataEnum", "Pitch", None), TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True, True)
    ROLL = (5, lambda: QCoreApplication.translate("TrackableDataEnum", "Roll", None), TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True, True)
    AIR_SPEED = (6, lambda: QCoreApplication.translate("TrackableDataEnum", "Air Speed", None), TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, True, True)
    GPS_TIME = (7, lambda: QCoreApplication.translate("TrackableDataEnum", "GPS Time", None), TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.SYSTEM_TIME, True, True)
    LONGITUDE = (8, lambda: QCoreApplication.translate("TrackableDataEnum", "Longitude", None), TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    LATITUDE = (9, lambda: QCoreApplication.translate("TrackableDataEnum", "Latitude", None), TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, False, True)
    BATTERY_PERCENTAGE = (10, lambda: QCoreApplication.translate("TrackableDataEnum", "Battery Percentage", None), TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True, True)
    ARM_STATUS = (11, lambda: QCoreApplication.translate("TrackableDataEnum", "Arm Status", None), TrackableDataUpdate.update_arm_status, TrackableDataPacketTimer.HEARTBEAT, True, False)
    RELATIVE_ALTITUDE = (12, lambda: QCoreApplication.translate("TrackableDataEnum", "Relative Altitude", None), TrackableDataUpdate.update_relative_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, True)
    BREACH_STATUS = (13, lambda: QCoreApplication.translate("TrackableDataEnum", "Fence Breach Status", None), TrackableDataUpdate.update_breach_status, TrackableDataPacketTimer.FENCE_STATUS, False, False)

    @staticmethod
    def from_id(i: int) -> TrackableDataEnum:
        e: TrackableDataEnum
        for e in TrackableDataEnum:
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
    connection_type: ConnectionType | None
    serial_port: int
    serial_baud_rate: int
    ip: str

    def __init__(self):
        self.connection_type = None
        self.serial_port = None
        self.serial_baud_rate = None
        self.ip = None

    def reset_connection_properties(self) -> None:
        self.serial_port = None
        self.serial_baud_rate = None
        self.ip = None
        self.connection_type = None

class ServerConnection:
    ip: str | None = None
    port: int
    username: str
    password: str
    team_no: int
    telemetry_timer: QTimer

    def get_address(self) -> str:
        return f"{self.ip}:{self.port}"

class MavlinkWorker(QObject):
    parent: MainWindow
    running: bool
    update_watch_list = Signal(int, str)
    create_warning = Signal(str)
    fence_mission_count: int
    waypoint_mission_count: int
    mission_fence_item_received = Signal(MAVLink_mission_item_message, int)
    mission_fence_item_int_received = Signal(MAVLink_mission_item_int_message, int)
    mission_waypoint_item_received = Signal(MAVLink_mission_item_message, int)
    mission_waypoint_item_int_received = Signal(MAVLink_mission_item_int_message, int)
    send_fence_mission_data = Signal(int, bool)
    def __init__(self, mavlink_connection: mavfile, parent: MainWindow):
        super().__init__()
        self.mavlink_connection = mavlink_connection
        self.parent = parent
        self.watch_list = parent.ui.watch_list
        self.running = False
        self.fence_mission_count = 0
        self.waypoint_mission_count = 0

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
            msgID: int = packet.get_header().msgId
            if msgID == MAVLINK_MSG_ID_BAD_DATA:
                if all_printable(packet.data):
                    qWarning("Invalid data received: %s" % packet.data)
                else:
                    qWarning("Invalid data received")
            elif msgID == MAVLINK_MSG_ID_MISSION_REQUEST_INT:
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.send_fence_mission_data.emit(packet.seq, True)
            elif msgID == MAVLINK_MSG_ID_MISSION_REQUEST: # Time to handle deprecated Legacy code
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.send_fence_mission_data.emit(packet.seq, False)
            elif msgID == MAVLINK_MSG_ID_MISSION_ACK:
                qDebug("MissionACK packet received with type %s and with mission_type %s" % (packet.type, packet.mission_type))
            elif msgID == MAVLINK_MSG_ID_MISSION_COUNT:
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.fence_mission_count = packet.count
                    qDebug("Received %s fence point" % self.fence_mission_count)
                    if self.fence_mission_count > 0:
                        self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, 0, MAV_MISSION_TYPE_FENCE)
                    else:
                        self.parent.requested_to_get_fence = False
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.waypoint_mission_count = packet.count
                    qDebug("Received %s mission waypoint" % self.waypoint_mission_count)
                    if self.waypoint_mission_count > 0:
                        self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, 0, MAV_MISSION_TYPE_MISSION)
                    else:
                        self.parent.requested_to_get_mission = False

            elif msgID == MAVLINK_MSG_ID_MISSION_ITEM_INT:
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_int_received.emit(packet, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_int_received.emit(packet, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_MISSION_ITEM:
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_received.emit(packet, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_received.emit(packet, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_COMMAND_ACK:
                command: int = packet.command
                result: int = packet.result
                qDebug("CommandACK received for command %s and result %s" % (command, result))
                if command == MAV_CMD_DO_REPOSITION:
                    if result == MAV_RESULT_DENIED:
                        self.create_warning.emit("Reposition command denied by vehicle")
                        self.parent.ui.map_view.target_coord.remove_position()
            elif msgID in MSG_ID_2_TRACKABLE_DATA_TYPE:
                e = MSG_ID_2_TRACKABLE_DATA_TYPE[msgID]
                data_enum_values = e.value[4]
                for i in data_enum_values:
                    self.trigger_update_value(TrackableDataEnum.from_id(i), packet)
            # else:
            #     qDebug("Ignoring packet with id %s" % msgID)

class ConnectionWaitWrapper(QObject):
    after_heartbeat_successfully_received = Signal(mavfile)
    after_heartbeat_not_received = Signal(mavfile)
    mavlink_connection_error = Signal()
    set_device_connection_text = Signal(str)
    mavlink_connection: mavfile
    uav_connection: UavConnection
    __parent: MainWindow
    con_thread: QThread

    def __init__(self, parent: MainWindow, uav_connection: UavConnection):
        super().__init__()
        self.uav_connection = uav_connection
        self.mavlink_connection = None
        self.__parent = parent

    def run(self):
        self.set_device_connection_text.emit(QCoreApplication.translate("UAVConnection", "Trying to connect device :O", None))
        try:
            match self.uav_connection.connection_type:
              case ConnectionType.TCP:
                  self.mavlink_connection = mavtcp(self.uav_connection.ip, retries=1)
              case ConnectionType.UDP:
                  self.mavlink_connection = mavudp(self.uav_connection.ip, timeout=10)
              case ConnectionType.SERIAL:
                  system_identifier = re.sub("\\d", "", QSerialPortInfo.availablePorts()[0].systemLocation())
                  self.mavlink_connection = mavserial(system_identifier + str(self.uav_connection.serial_port), baud=self.uav_connection.serial_baud_rate)
              case None:
                  self.mavlink_connection_error.emit()
                  qWarning("Connection type is null ???")
                  return
        except OSError as e:
            self.set_device_connection_text.emit(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))
            qWarning("Tried a invalid connection: %s" % str(e))
            self.mavlink_connection_error.emit()
            return
        qInfo("Successfully Connected with mavlink, Target System: %s, Target component: %s" % (self.mavlink_connection.target_system, self.mavlink_connection.target_component))

        self.mavlink_connection.mav.heartbeat_send(
            MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

        try:
            msg: MAVLink_heartbeat_message = self.mavlink_connection.wait_heartbeat(timeout=10)
            if msg is None:
                raise Exception("Connection failed")

            qInfo("Successfully Received first heartbeat")
            self.after_heartbeat_successfully_received.emit(self.mavlink_connection)
        except:
            self.after_heartbeat_not_received.emit(self.mavlink_connection)
        self.__parent.connection_wait_wrapper = None
        self.con_thread.quit()


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
    mavlink_connection: mavfile = None
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
        for mode in UAV_Modes:
            self.ui.fly_mode_combobox.insertItem(mode.value[1], mode.value[0])
        self.ui.fly_mode_combobox.setCurrentIndex(-1)

        self.ui.actionConfigurate_UAV.triggered.connect(self._actionConfigurate_UAV)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)
        self.ui.actionSet_Colors.triggered.connect(self._actionConfigurateSetColors)

        add_to_watch_menu: QMenu = QMenu(parent=self)

        for e in TrackableDataEnum:
            action: QAction = QAction(text=e.value[1](), parent=self)
            action.setObjectName(str(e.value[0]))
            action.triggered.connect(partial(self.add_to_watch_list, e))
            TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]] = action
            add_to_watch_menu.addAction(action)

        self.ui.add_to_watch.setMenu(add_to_watch_menu)

        for e in TrackableDataEnum:
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
        self.ui.map_view.coords_for_geofence.upload_geofence_data.connect(lambda: self.update_geofence_data(self.ui.map_view.server_ads_data_model.m_datas + self.ui.map_view.user_ads_data_model.m_datas))
        self.ui.map_view.upload_ads_data.connect(lambda: self.update_geofence_data(self.ui.map_view.server_ads_data_model.m_datas + self.ui.map_view.user_ads_data_model.m_datas))
        self.ui.actionAbout.triggered.connect(self._about)
        self.ui.actionAbout_Qt.triggered.connect(lambda: QMessageBox.aboutQt(self))
        self.fence_upload_timout = QTimer(self, singleShot=True, interval=10000)
        self.fence_upload_timout.timeout.connect(self.fence_upload_reset)
        self.fence_download_timout = QTimer(self, singleShot=True, interval=10000)
        self.fence_download_timout.timeout.connect(self.request_fence_data_timeout)
        self.mission_download_timout = QTimer(self, singleShot=True, interval=10000)
        self.mission_download_timout.timeout.connect(self.request_mission_data_timeout)
        self.ui.download_missions.clicked.connect(self.request_mission_data)
        self.ui.download_fence_data.clicked.connect(self.request_fence_data)

    def closeEvent(self, event, /):
        try:
            self._uav_disconnect()
        except:
            pass
        super().closeEvent(event)

    next_mission_order_seq_id: int = 0
    requested_to_get_mission: bool = False
    def request_mission_data(self):
        if self.mavlink_connection is None:
            return
        if self.requested_to_get_mission:
            qDebug("Tried to get missions when already getting missions from uav")
            return
        self.requested_to_get_mission = True
        self.mavlink_connection.mav.mission_request_list_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_MISSION)
        self.mission_download_timout.start()

    def request_mission_data_timeout(self):
        qWarning("Mission Download taking really long, probably vehicle connection has been lost")
        self._create_warning("Mission Download taking really long, probably vehicle connection has been lost")
        self.next_mission_order_seq_id = 0
        self.requested_to_get_mission = False
        self.ui.map_view.mission_coords_data_model.layoutChanged.emit()
        self.ui.map_view.mission_geopath.mission_geopath_changed.emit()

    def mission_waypoint_received(self, coord: QGeoCoordinate, command: int, seq: int, use_int: bool, count: int):
        if not self.requested_to_get_mission or count == 0:
            return
        qDebug("Mission received with %s coords, %s command, %s seq" % (coord, command, seq))
        if self.next_mission_order_seq_id != seq:
            qDebug("Out of order mission")
            return
        if seq == 0:
            self.ui.map_view.mission_coords_data_model.m_datas.clear()
            self.ui.map_view.mission_geopath.clear()
        coord_data: SpecialCoordsData = SpecialCoordsData()
        coord_data.position = coord
        coord_data.coord_type = 1
        self.ui.map_view.mission_coords_data_model.m_datas.append(coord_data)
        self.ui.map_view.mission_geopath.add_pos(coord)
        self.mission_download_timout.start()
        if seq + 1 != count:
            if use_int:
                self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_MISSION)
            else:
                self.mavlink_connection.mav.mission_request_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_MISSION)
            self.next_mission_order_seq_id = seq + 1
        else:
            self.mission_download_timout.stop()
            self.mavlink_connection.mav.mission_ack_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION)
            self.next_mission_order_seq_id = 0
            self.requested_to_get_mission = False
            self.ui.map_view.mission_coords_data_model.layoutChanged.emit()
            self.ui.map_view.mission_geopath.mission_geopath_changed.emit()

    def mission_waypoint_item_received(self, item: MAVLink_mission_item_message, count: int):
        command: int = item.command
        coord: QGeoCoordinate = QGeoCoordinate(item.x, item.y)
        self.mission_waypoint_received(coord, command, item.seq, False, count)

    def mission_waypoint_item_int_received(self, item: MAVLink_mission_item_int_message, count: int):
        command: int = item.command
        coord: QGeoCoordinate = QGeoCoordinate(item.x / 1e7, item.y / 1e7)
        self.mission_waypoint_received(coord, command, item.seq, True, count)

    def request_fence_data(self):
        if self.mavlink_connection is None:
            return
        if self.requested_to_get_fence:
            qDebug("Tried to get fence when already getting fence from uav")
            return
        self.requested_to_get_fence = True
        self.mavlink_connection.mav.mission_request_list_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_FENCE)
        self.fence_download_timout.start()

    def request_fence_data_timeout(self):
        qWarning("Fence Download taking really long, probably vehicle connection has been lost")
        self._create_warning("Fence Download taking really long, probably vehicle connection has been lost")
        self.next_fence_order_seq_id = 0
        self.requested_to_get_fence = False
        self.ui.map_view.user_ads_data_model.layoutChanged.emit()
        self.coord_counter = 1

    coord_counter: int = 1
    next_fence_order_seq_id: int = 0
    requested_to_get_fence: bool = False
    def mission_fence_item_int_received(self, item: MAVLink_mission_item_int_message, count: int):
        command: int = item.command
        coord: QGeoCoordinate = QGeoCoordinate(item.x / 1e7, item.y / 1e7)
        self.mission_fence_received(coord, command, item.seq, item.param1, True, count)

    # TODO: This handling assumes 4 vertex fence
    def mission_fence_received(self, coord: QGeoCoordinate, command: int, seq: int, ads_size: float, use_int: bool, count: int):
        if not self.requested_to_get_fence or count == 0:
            return
        qDebug("Fence received with %s coords, %s command, %s seq" % (coord, command, seq))
        if self.next_fence_order_seq_id != seq:
            qDebug("Out of order fence")
            return
        if command == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            if seq == 0:
                self.ui.map_view.user_ads_data_model.m_datas.clear()
            ads = AdsData()
            ads.position = coord
            ads.size = ads_size
            ads.is_selected = False
            self.ui.map_view.user_ads_data_model.m_datas.append(ads)
        elif command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            data: SpecialCoordsData = SpecialCoordsData()
            data.position = coord
            data.coord_type = self.coord_counter + 5  # I hope i will remember how this works
            match self.coord_counter:
                case 1:
                    self.ui.map_view.coord_data_model.m_datas.clear()
                    self.ui.map_view.coords_for_geofence.gc1_v = coord
                case 2:
                    self.ui.map_view.coords_for_geofence.gc2_v = coord
                case 3:
                    self.ui.map_view.coords_for_geofence.gc3_v = coord
                case 4:
                    self.ui.map_view.coords_for_geofence.gc4_v = coord
                    self.ui.map_view.coords_for_geofence.gc_changed.emit()
                    self.coord_counter = 0
            self.ui.map_view.coord_data_model.m_datas.append(data)
            self.ui.map_view.coord_data_model.layoutChanged.emit()
            self.coord_counter += 1
        self.fence_download_timout.start()
        if seq + 1 != count:
            if use_int:
                self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_FENCE)
            else:
                self.mavlink_connection.mav.mission_request_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_FENCE)
            self.next_fence_order_seq_id = seq + 1
        else:
            self.fence_download_timout.stop()
            self.mavlink_connection.mav.mission_ack_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE)
            self.next_fence_order_seq_id = 0
            self.requested_to_get_fence = False
            self.ui.map_view.user_ads_data_model.layoutChanged.emit()

    def mission_fence_item_received(self, item: MAVLink_mission_item_message, count: int):
        command: int = item.command
        coord: QGeoCoordinate = QGeoCoordinate(item.x, item.y)
        self.mission_fence_received(coord, command, item.seq, item.param1, False, count)

    def _about(self):
        QMessageBox.about(self, QCoreApplication.translate("MainWindow", u"About", None), QCoreApplication.translate("MainWindow", "Designed and developed by Muzaffer Kadir Belen to be used by ARES teknofest team", None))

    def _remove_ads(self):
        for m_data in self.ui.map_view.user_ads_data_model.m_datas:
            if m_data.is_selected:
                self.ui.map_view.user_ads_data_model.m_datas.remove(m_data)
                self.ui.map_view.user_ads_data_model.layoutChanged.emit()
                self.ui.map_view.upload_ads_data.emit()

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
            if self.mavlink_connection is None:
                self.ui.watch_list.setItem(i, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))
        for e in TRACKABLE_DATA_ENUM_ACTIONS.values():
            tde = TrackableDataEnum.from_id(int(e.objectName()))

            e.setText(tde.value[1]())

    def resetWatcherWidgetValues(self):
        length: int = self.ui.watch_list.rowCount()
        for i in range(length):
            self.ui.watch_list.setItem(i, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))

    def get_all_dialogs(self):
        return [self.uav_connection_dialog,
                   self.server_connection_dialog,
                   self.color_selector_dialog,
                   self.geofence_dialog,
                   self.add_ads_dialog,
                   self.camera_server_connection_dialog,
                   self.ui.map_view.mouse_input_handler.ard_dialog]

    def retranslateOpenDialogs(self):
        dialogs = self.get_all_dialogs()

        for dialog in dialogs:
            if dialog is not None:
                dialog.ui.retranslateUi(dialog)

        for dialog in self.color_selector_dialog.get_dialogs():
            dialog.retranslateUi()

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
        self.ui.map_view.upload_ads_data.emit()

    def __refresh_ads(self):
        if not self.server_connection.ip:
            return

        self.ui.map_view.update_server_ads_data(get_ads(self.server_connection.get_address()))

    def __setArmStatus(self, is_arm: int):
        if self.last_arm_change_was_from_uav:
            self.last_arm_change_was_from_uav = False
            return

        qDebug("Trying to send armed status: %s" % is_arm)

        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            is_arm, # 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0
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
        self.ui.map_view.coords_for_geofence.upload_geofence_data.emit()
        qDebug("New geofence coords: %s, %s, %s, %s" % (self.ui.map_view.coords_for_geofence.gc1_v, self.ui.map_view.coords_for_geofence.gc2_v, self.ui.map_view.coords_for_geofence.gc3_v, self.ui.map_view.coords_for_geofence.gc4_v))
        self.geofence_dialog.close()

    def _enable_fence(self) -> None:
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_CMD_DO_FENCE_ENABLE,
            0,
            1,
            0,0,0,0,0,0
        )

    requested_to_send_fence_with_fence: bool = False
    def send_fence_mission_data(self, index: int, use_item_int: bool):
        ads_list_len = len(self.ads_list_cache)
        if ads_list_len == 0 and not self.requested_to_send_fence_with_fence:
            return
        qDebug("Requested Mission Data at index %s" % index)
        if index >= ads_list_len:
            coords = [self.ui.map_view.coords_for_geofence.gc1_v, self.ui.map_view.coords_for_geofence.gc2_v,
                      self.ui.map_view.coords_for_geofence.gc3_v, self.ui.map_view.coords_for_geofence.gc4_v]

            qDebug("Sending fence with position %s" % coords[index - ads_list_len])

            if use_item_int:
                self.mavlink_connection.mav.mission_item_int_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL_INT,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    0,
                    0,
                    4,
                    0,
                    0,
                    0,
                    int(coords[index - ads_list_len].latitude() * 1e7),
                    int(coords[index - ads_list_len].longitude() * 1e7),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
            else:
                self.mavlink_connection.mav.mission_item_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    0,
                    0,
                    4,
                    0,
                    0,
                    0,
                    coords[index - ads_list_len].latitude(),
                    coords[index - ads_list_len].longitude(),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
        else:
            qDebug("Sending ads with position %s" % self.ads_list_cache[index].position)
            if use_item_int:
                self.mavlink_connection.mav.mission_item_int_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL_INT,
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    0,
                    0,
                    self.ads_list_cache[index].size,
                    0,
                    0,
                    0,
                    int(self.ads_list_cache[index].position.latitude() * 1e7),
                    int(self.ads_list_cache[index].position.longitude() * 1e7),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
            else:
                self.mavlink_connection.mav.mission_item_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL,
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    0,
                    0,
                    self.ads_list_cache[index].size,
                    0,
                    0,
                    0,
                    self.ads_list_cache[index].position.latitude(),
                    self.ads_list_cache[index].position.longitude(),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
        self.fence_upload_timout.start()
        size: int = ads_list_len + 4 if self.requested_to_send_fence_with_fence else 0
        if size == index + 1:
            self.fence_upload_timout.stop()
            self.ads_list_cache = []
            self.requested_to_send_fence_with_fence = False
    ads_list_cache: list[AdsData] = []

    def update_geofence_data(self, ads_list: list[AdsData]):
        has_a_fence = self.ui.map_view.coords_for_geofence.is_set
        self.mavlink_connection.mav.mission_clear_all_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_FENCE)
        fence_count: int = len(ads_list)
        if has_a_fence:
            fence_count += 4
            self.requested_to_send_fence_with_fence = True
        if fence_count == 0:
            qDebug("Fence count is 0, not sending anything for fence")
            return
        qDebug("Sending mission count: %s" % fence_count)
        self.ads_list_cache = ads_list
        self.mavlink_connection.mav.mission_count_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            fence_count,
            MAV_MISSION_TYPE_FENCE
        )
        self.fence_upload_timout.start()

    def fence_upload_reset(self):
        qWarning("Fence Upload taking really long, probably vehicle connection has been lost")
        self._create_warning("Fence Upload taking really long, probably vehicle connection has been lost")
        self.ads_list_cache = []
        self.requested_to_send_fence_with_fence = False

    def __reset_geofence_dialog(self):
        self.geofence_dialog = None

    kamikaze_start: GpsSaati

    def __start_kamikaze(self):
        if self.mavlink_connection is None:
            return
        longitude = float(self.ui.kamikaze_longitude.text())
        latitude = float(self.ui.kamikaze_latitude.text())
        kamikaze_start = self.next_telemetry.gps_saati
        qWarning("Kamikaze not implemented yet") # TODO

    def on_kamikaze_end(self, qr_text: str) -> None:
        kamikaze_end = self.next_telemetry.gps_saati
        send_kamikaze(self.server_connection.get_address(), self.kamikaze_start, kamikaze_end, qr_text)

    def __get_kamikaze_coords(self):
        if self.server_connection.ip is None:
            return
        qr_coords: QrCoords = get_kamikaze_coords(self.server_connection.get_address())
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
        self.server_connection_dialog.ui.connect.clicked.connect(lambda: self._server_connect(self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect())
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    color_options: ColorOptions = ColorOptions()
    def _actionConfigurateSetColors(self):
        if self.color_selector_dialog is not None:
            return
        self.color_selector_dialog = ColorSelectorInterface(self, self.color_options)
        self.color_selector_dialog.finished.connect(self._reset_color_configurate_screen)
        self.color_selector_dialog.show()

    def _reset_color_configurate_screen(self):
        self.color_options = self.color_selector_dialog.savedOptions
        self.color_selector_dialog = None

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
                self.uav_connection_dialog.ui.serial_baud.setText(str(self.uav_connection.serial_baud_rate))
                self.uav_connection_dialog.ui.connection_type.setCurrentIndex(2 + self.uav_connection.serial_port)
            else:
                isTCP: bool = self.uav_connection_dialog.connection_type == ConnectionType.TCP
                self.uav_connection_dialog.ui.ip_address.setText(self.uav_connection.ip)
                if isTCP:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(0)
                else:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(1)
        self.uav_connection_dialog.ui.connect.clicked.connect(self._uav_connect)
        self.uav_connection_dialog.ui.disconnect.clicked.connect(self._uav_disconnect)
        self.uav_connection_dialog.finished.connect(lambda e: self._reset_dialog(True))

    def _reset_dialog(self, is_uav: bool):
        if is_uav:
            self.uav_connection_dialog = None
        else:
            self.server_connection_dialog = None

    connection_wait_wrapper: ConnectionWaitWrapper | None = None

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
            self.uav_connection.serial_baud_rate = int(self.uav_connection_dialog.ui.serial_baud.text())
            self.uav_connection.serial_port = self.uav_connection_dialog.ui.connection_type.currentIndex() - 2
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()

        if self.connection_wait_wrapper is not None:
            qDebug("Tried to press connect when trying to connect")
            return
        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type
        self.connection_wait_wrapper = ConnectionWaitWrapper(self, self.uav_connection)
        self.connection_wait_wrapper.after_heartbeat_successfully_received.connect(self.__successful_uav_connection)
        self.connection_wait_wrapper.after_heartbeat_not_received.connect(self.__error_when_receiving_heartbeat)
        self.connection_wait_wrapper.set_device_connection_text.connect(self.uav_connection_dialog.ui.device_connection_text.setText)
        self.connection_wait_wrapper.mavlink_connection_error.connect(self.__uav_mav_connection_error)
        connection_thread: QThread = QThread(self)
        connection_thread.setObjectName("Connection Thread")
        connection_thread.started.connect(self.connection_wait_wrapper.run)
        self.connection_wait_wrapper.con_thread = connection_thread
        self.connection_wait_wrapper.moveToThread(connection_thread)
        connection_thread.start()

    def __uav_mav_connection_error(self):
        self.uav_connection.reset_connection_properties()

    def __error_when_receiving_heartbeat(self, mav_connection: mavfile):
        self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))

        self.uav_connection_dialog.ui.invalid_input_error_label.show()
        if self.uav_connection.connection_type == ConnectionType.SERIAL:
            qInfo("Can not connect to UAV from %s" % (
                        str(self.uav_connection.serial_port) + "," + str(self.uav_connection.serial_baud_rate)))
        else:
            qInfo("Can not connect to UAV from %s" % self.uav_connection.ip)
        self.uav_connection.reset_connection_properties()
        self.ui.device_connection_warning.show()
        try:
            mav_connection.close()
        except:
            pass
        self.ui.map_view.mavlink_connection = None

    def __successful_uav_connection(self, mav_connection: mavfile):
        self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
        self.mavlink_connection = mav_connection
        self.mavlink_connection.mav.request_data_stream_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_DATA_STREAM_ALL,
            10,
            1
        )
        for e in TrackableDataPacketTimer:
            self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
                                                            self.mavlink_connection.target_component,
                                                            MAV_CMD_SET_MESSAGE_INTERVAL,
                                                            0,
                                                            e.value[0],
                                                            e.value[3],
                                                            0, 0, 0, 0, 0)
        self._enable_fence()

        self.ui.map_view.mavlink_connection = self.mavlink_connection
        self.mavlink_thread = QThread(self)
        self.mavlink_thread.setObjectName("Mavlink Connection Thread")
        self.mavlink_worker = MavlinkWorker(self.mavlink_connection, self)
        self.mavlink_worker.running = True
        self.mavlink_worker.update_watch_list.connect(self._apply_watch_update)
        self.mavlink_worker.create_warning.connect(self._create_warning)
        self.mavlink_worker.mission_fence_item_received.connect(self.mission_fence_item_received)
        self.mavlink_worker.mission_fence_item_int_received.connect(self.mission_fence_item_int_received)
        self.mavlink_worker.mission_waypoint_item_received.connect(self.mission_waypoint_item_received)
        self.mavlink_worker.mission_waypoint_item_int_received.connect(self.mission_waypoint_item_int_received)
        self.mavlink_worker.send_fence_mission_data.connect(self.send_fence_mission_data)
        self.mavlink_thread.started.connect(self.mavlink_worker.run)
        self.mavlink_worker.moveToThread(self.mavlink_thread)
        self.mavlink_thread.start()
        self.enableFeaturesAfterUAVConnected()

    def _apply_watch_update(self, row: int, value: str):
        self.ui.watch_list.setItem(row, 3, QTableWidgetItem(value))

    def _create_warning(self, text: str) -> None:
        self.ui.statusbar.showMessage(text, 2000)

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


    def _uav_disconnect(self):
        if self.mavlink_connection is None:
            return
        self.mavlink_worker.running = False
        self.mavlink_thread.quit()
        self.mavlink_thread.wait()
        self.mavlink_connection.close()
        self.uav_connection.reset_connection_properties()
        self.disableFeaturesAfterUAVDisconnected()
        self.next_telemetry = TelemetryData()
        self.resetWatcherWidgetValues()

    def _server_connect(self, dialog: ServerConnectionInterface):
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
        if not ("://" in self.server_connection.ip):
            self.server_connection.ip = "http://"+self.server_connection.ip
        self.server_connection.port = int(dialog.ui.server_port_input.text())
        self.server_connection.username = dialog.ui.server_login_username_input.text()
        self.server_connection.password = dialog.ui.server_login_password_input.text()

        try:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Trying to connect to server :O", None))
            self.server_connection.team_no = login_to_server(self.server_connection.get_address(), self.server_connection.username, self.server_connection.password)
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
        if self.mavlink_connection is None or self.server_connection.ip is not None:
            self.plane_on_map_update_timer.stop()
            return
        self.ui.map_view.update_plane_data_without_server(QGeoCoordinate(self.next_telemetry.iha_enlem, self.next_telemetry.iha_boylam), (self.next_telemetry.iha_yatis * 4) + 180)

    def __send_telemetry(self):
        if self.mavlink_connection is None and not self.force_sending_telemetry:
            qDebug("UAV not connected")
            return
        global SERVER_IS_UNREACHABLE_COUNTER
        if SERVER_IS_UNREACHABLE_COUNTER > 100:
            SERVER_IS_UNREACHABLE_COUNTER = 0
            self._server_disconnect()
            qWarning("Server connection is not possible for 100 time, disconnecting")
            return
        self.last_server_telemetry_response = send_telemetry(self.server_connection.get_address(), self.next_telemetry)
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

