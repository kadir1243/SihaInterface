import copy
import math
import os
import re
from enum import Enum
from functools import partial

from src.CameraWidget import CameraServerProtocol
from src.CommonUtils import TrackableDataPacketTimer, main_and_sub_mode_to_px4_uav_mode, MSG_ID_2_TRACKABLE_DATA_TYPE, \
    PX4_UAV_Modes, KamikazeState, index_to_px4_uav_mode, SupportedLanguages, Ardupilot_UAV_Modes

os.environ['MAVLINK20'] = '1'

from PySide6.QtCore import QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug, QThread, QObject, Signal, QLocale, \
    QTranslator, QCoreApplication, QRegularExpression
from PySide6.QtGui import QAction, QDoubleValidator, Qt
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import QMainWindow, QTableWidgetItem, QMenu, QApplication, QMessageBox, QStyle, QProxyStyle
from pymavlink.dialects.v20.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message, MAVLink_message, MAVLink_heartbeat_message, \
    MAVLink_global_position_int_message, MAVLink_system_time_message, MAV_CMD_DO_FENCE_ENABLE, \
    MAVLink_fence_status_message, \
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, MAV_FRAME_GLOBAL_INT, \
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAV_MISSION_TYPE_FENCE, \
    MAVLINK_MSG_ID_MISSION_ACK, MAVLINK_MSG_ID_MISSION_REQUEST, MAV_FRAME_GLOBAL, MAVLINK_MSG_ID_BAD_DATA, \
    MAVLINK_MSG_ID_COMMAND_ACK, MAV_CMD_DO_REPOSITION, MAV_RESULT_DENIED, \
    MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_ITEM_INT, MAVLink_mission_item_int_message, \
    MAV_MISSION_ACCEPTED, MAVLINK_MSG_ID_MISSION_ITEM, MAVLink_mission_item_message, MAV_MODE_FLAG_SAFETY_ARMED, \
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_AUTOPILOT_INVALID, MAV_DATA_STREAM_ALL, \
    MAV_CMD_SET_MESSAGE_INTERVAL, MAV_MISSION_TYPE_MISSION, MAV_RESULT_TEMPORARILY_REJECTED, MAV_CMD_DO_SET_MODE, \
    MAV_MODE_FLAG_AUTO_ENABLED, MAV_AUTOPILOT_PX4, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, \
    MAV_RESULT_FAILED, MAV_RESULT_ACCEPTED, MAV_CMD_REQUEST_MESSAGE
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src.AddADSInterface import AddADSInterface
from src.CameraServerConnectionInterface import CameraServerConnectionInterface
from src.ColorSelectorInterface import ColorSelectorInterface, ColorOptions
from src.MapWidget import ZERO_GEO_COORDS, AdsData, SpecialCoordsData, CRUISE_THR_MAX
from src.SetGeofenceInterface import SetGeofenceInterface
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, QrCoords, \
    get_kamikaze_coords, TelemetryData, TelemetryResponseData, get_ads, send_kamikaze
from src.ServerConnectionInterface import ServerConnectionInterface
from src.KeybindingConfigInterface import KeybindingConfigInterface
from src.input_types import InputMapping, KeybindingsEnum
from ui_files_python.uav_interface import Ui_MainWindow

def to_degree(x: float) -> float:
    if x < 0:
        x = x + 2 * math.pi
    return x * (180 / math.pi)

def clamp(val: float, minv: float, maxv: float):
    return max(minv, min(maxv, val))

class MavlinkWorkerSignals(QObject):
    set_fly_mode = Signal(int)
    set_arm_mode = Signal(int)
    should_reposition_removed = Signal()
    change_autopilot = Signal(int)

    def __init__(self, parent):
        super().__init__(parent)

class TrackableDataUpdate:
    @staticmethod
    def update_ground_speed(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_vfr_hud_message) -> str:
        telemetry.iha_hiz = packet.groundspeed
        return str(packet.groundspeed)
    @staticmethod
    def update_air_speed(packet: MAVLink_vfr_hud_message) -> str:
        return str(packet.airspeed)
    @staticmethod
    def update_velocity(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.vel)
    @staticmethod
    def update_relative_altitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_global_position_int_message) -> str:
        telemetry.iha_irtifa = packet.relative_alt / 1000
        return str(packet.relative_alt / 1000)
    @staticmethod
    def update_altitude(packet: MAVLink_global_position_int_message) -> str:
        return str(packet.alt / 1000)
    @staticmethod
    def update_longitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_gps_raw_int_message) -> str:
        telemetry.iha_boylam = packet.lon / 1e7
        return str(packet.lon / 1e7)
    @staticmethod
    def update_latitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_gps_raw_int_message) -> str:
        telemetry.iha_enlem = packet.lat / 1e7
        return str(packet.lat / 1e7)
    @staticmethod
    def update_yaw(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        yaw: float = to_degree(packet.yaw)
        telemetry.iha_yonelme = yaw
        return str(yaw)
    @staticmethod
    def update_pitch(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        pitch: float = clamp(math.degrees(packet.pitch), -90, 90)
        telemetry.iha_dikilme = pitch
        return str(pitch)
    @staticmethod
    def update_roll(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        roll: float = clamp(math.degrees(packet.roll), -90, 90)
        telemetry.iha_yatis = roll
        return str(roll)
    @staticmethod
    def update_gps_time(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_system_time_message) -> str:
        datetime = QDateTime.fromMSecsSinceEpoch(int(packet.time_unix_usec / 1000))
        telemetry.gps_saati = GpsSaati(datetime)
        return datetime.toString()
    @staticmethod
    def update_battery_percentage(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_battery_status_message) -> str:
        telemetry.iha_batarya = packet.battery_remaining
        return str(packet.battery_remaining) + "%"
    @staticmethod
    def update_arm_status(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_heartbeat_message) -> str:
        arm: int = 1 if (packet.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0 else 0
        worker_signals.set_arm_mode.emit(arm)
        return str(arm != 0)

    @staticmethod
    def update_fly_mode(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_heartbeat_message) -> str:
        telemetry.iha_otonom = 1 if (packet.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) != 0 else 0
        worker_signals.change_autopilot.emit(packet.autopilot)
        if packet.autopilot == MAV_AUTOPILOT_PX4:
            sub_mod = packet.custom_mode >> 24
            base_mod = (packet.custom_mode >> 16) - (sub_mod << 8)
            px4mode = main_and_sub_mode_to_px4_uav_mode.get(base_mod)
            if px4mode:
                sub = px4mode.get(sub_mod)
                if sub:
                    index: int = sub.value[0]
                    worker_signals.set_fly_mode.emit(index)
                else:
                    qWarning("Can not handle px4 sub mode %s %s %s" % (packet.custom_mode, base_mod, sub_mod))
            else:
                qWarning("Can not handle px4 mode %s %s" % (packet.custom_mode, base_mod))
        elif packet.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
            index: int = packet.custom_mode
            if 27 > index >= 0:
                worker_signals.set_fly_mode.emit(index)
                if packet.custom_mode != 15:
                    worker_signals.should_reposition_removed.emit()
            else:
                qWarning("Don't know how to handle this custom mode data")
        else:
            qWarning("Unknown pilot type, fly mode unsupported")
        return ""

    @staticmethod
    def update_breach_status(packet: MAVLink_fence_status_message) -> str:
        text = "Breached"
        if packet.breach_status == 0:
            text = "Not " + text
        return text

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, should be updated on background (telemetry etc.), is it in watch_list widget, is it in watchlist at start)
    GROUND_SPEED = (0, lambda: QCoreApplication.translate("TrackableDataEnum", "Ground Speed", None), TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, True, True, True)
    VELOCITY = (1, lambda: QCoreApplication.translate("TrackableDataEnum", "Velocity", None), TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False, True, True)
    ALTITUDE = (2, lambda: QCoreApplication.translate("TrackableDataEnum", "Altitude", None), TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, True, False)
    YAW = (3, lambda: QCoreApplication.translate("TrackableDataEnum", "Yaw", None), TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    PITCH = (4, lambda: QCoreApplication.translate("TrackableDataEnum", "Pitch", None), TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    ROLL = (5, lambda: QCoreApplication.translate("TrackableDataEnum", "Roll", None), TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    AIR_SPEED = (6, lambda: QCoreApplication.translate("TrackableDataEnum", "Air Speed", None), TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, False, True, True)
    GPS_TIME = (7, lambda: QCoreApplication.translate("TrackableDataEnum", "GPS Time", None), TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.SYSTEM_TIME, True, True, True)
    LONGITUDE = (8, lambda: QCoreApplication.translate("TrackableDataEnum", "Longitude", None), TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, True, True, True)
    LATITUDE = (9, lambda: QCoreApplication.translate("TrackableDataEnum", "Latitude", None), TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, True, True, True)
    BATTERY_PERCENTAGE = (10, lambda: QCoreApplication.translate("TrackableDataEnum", "Battery Percentage", None), TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True, True, True)
    ARM_STATUS = (11, lambda: QCoreApplication.translate("TrackableDataEnum", "Arm Status", None), TrackableDataUpdate.update_arm_status, TrackableDataPacketTimer.HEARTBEAT, True, False, False)
    RELATIVE_ALTITUDE = (12, lambda: QCoreApplication.translate("TrackableDataEnum", "Relative Altitude", None), TrackableDataUpdate.update_relative_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, True, True, True)
    BREACH_STATUS = (13, lambda: QCoreApplication.translate("TrackableDataEnum", "Fence Breach Status", None), TrackableDataUpdate.update_breach_status, TrackableDataPacketTimer.FENCE_STATUS, False, True, False)
    FLY_MODE = (14, lambda: QCoreApplication.translate("TrackableDataEnum", "Fly Mode", None), TrackableDataUpdate.update_fly_mode, TrackableDataPacketTimer.HEARTBEAT, True, False, False)

    @staticmethod
    def from_id(i: int) -> TrackableDataEnum:
        e: TrackableDataEnum
        for e in TrackableDataEnum:
            if e.value[0] == i:
                return e
        return None

LANGUAGE_ACTIONS: dict[int, QAction] = {}

class UavConnection:
    connection_type: ConnectionType | None
    serial_port: str
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
    port: int = None
    username: str
    password: str
    team_no: int
    telemetry_timer: QTimer
    telemetry_thread: QThread = None

    def get_address(self) -> str:
        if not MainWindow.is_ip_address_valid(self.ip, False):
            return self.ip
        return f"{self.ip}:{self.port}"

class MavlinkWorker(QObject):
    parent: MainWindow
    running: bool
    update_watch_list = Signal(int, str)
    create_warning = Signal(str)
    connection_lost = Signal(str)
    fence_mission_count: int
    waypoint_mission_count: int
    mission_fence_item_received = Signal(int, float, float, int, float, int)
    mission_fence_item_int_received = Signal(int, float, float, int, float, int)
    mission_waypoint_item_received = Signal(int, float, float, int, int)
    mission_waypoint_item_int_received = Signal(int, float, float, int, int)
    send_fence_mission_data = Signal(int, bool)
    remove_reposition_location = Signal()
    worker_signals: MavlinkWorkerSignals
    def __init__(self, mavlink_connection: mavfile, parent: MainWindow):
        super().__init__()
        self.mavlink_connection = mavlink_connection
        self.parent = parent
        self.watch_list = parent.ui.watch_list
        self.running = False
        self.fence_mission_count = 0
        self.waypoint_mission_count = 0
        self.worker_signals = MavlinkWorkerSignals(self)

    def trigger_update_value(self, e: TrackableDataEnum, packet: MAVLink_message):
        length: int = self.watch_list.rowCount()
        new_val: str | None = None
        if e.value[4]:
            self.parent.next_telemetry.lock.lockForWrite()
            try:
                new_val = e.value[2](self.worker_signals, self.parent.next_telemetry, packet)  # Update if it is telemetry without caring it is in watch list or not
            finally:
                self.parent.next_telemetry.lock.unlock()
        for i in range(length):
            if self.watch_list.item(i, 0).text() == str(e.value[0]):
                if new_val is None:
                    new_val = e.value[2](packet)
                self.update_watch_list.emit(i, new_val)
                break

    def run(self):
        while self.running:
            try:
                packet: MAVLink_message = self.mavlink_connection.recv_match(blocking=True, timeout=1.0)
            except OSError as e:
                # Serial port died (USB glitch, EMI, unplug); the handle is
                # invalid from here on, so stop reading and tell the GUI.
                if self.running:
                    qWarning("MAVLink connection I/O error: %s" % e)
                    self.connection_lost.emit(str(e))
                break
            except Exception as e:
                qWarning("Error while reading MAVLink packet: %s" % e)
                continue
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
                packet: MAVLink_mission_item_int_message = packet
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_int_received.emit(packet.command, packet.x / 1e7, packet.y / 1e7, packet.seq, packet.param1, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_int_received.emit(packet.command, packet.x / 1e7, packet.y / 1e7, packet.seq, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_MISSION_ITEM:
                packet: MAVLink_mission_item_message = packet
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_received.emit(packet.command, packet.x, packet.y, packet.seq, packet.param1, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_received.emit(packet.command, packet.x, packet.y, packet.seq, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_COMMAND_ACK:
                command: int = packet.command
                result: int = packet.result

                if command == MAV_CMD_DO_REPOSITION:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Reposition command successfully executed")
                    elif result == MAV_RESULT_DENIED:
                        self.create_warning.emit("Reposition command denied by vehicle")
                        self.remove_reposition_location.emit()
                    else:
                        qDebug("Reposition result: %s" % result)
                elif command == MAV_CMD_DO_SET_MODE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Set mode command successfully executed")
                    elif result == MAV_RESULT_TEMPORARILY_REJECTED:
                        self.create_warning.emit("Set mode temporarily rejected by vehicle, sensor/calibration error")
                    else:
                        qDebug("Set mode result: %s" % result)
                elif command == MAV_CMD_SET_MESSAGE_INTERVAL:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Message interval successfully set")
                    elif result == MAV_RESULT_FAILED:
                        qWarning("Couldn't set some message interval")
                    else:
                        qDebug("Set message interval result: %s" % result)
                elif command == MAV_CMD_REQUEST_MESSAGE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Message request accepted")
                    else:
                        qDebug("Message request result: %s" % result)
                elif command == MAV_CMD_DO_FENCE_ENABLE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Fence enable command accepted")
                    else:
                        qDebug("Fence enable command result: %s" % result)
                else:
                    qDebug("CommandACK received for command %s and result %s" % (command, result))
            elif msgID in MSG_ID_2_TRACKABLE_DATA_TYPE:
                e = MSG_ID_2_TRACKABLE_DATA_TYPE[msgID]
                data_enum_values = e.value[4]
                for i in data_enum_values:
                    self.trigger_update_value(TrackableDataEnum.from_id(i), packet)
            # else:
            #     qDebug("Ignoring packet with id %s" % msgID)

class ConnectionWaitWrapper(QObject):
    after_heartbeat_successfully_received = Signal(mavfile)
    setup_for_autopilot = Signal(int)
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
                  self.mavlink_connection = mavserial(self.uav_connection.serial_port, baud=self.uav_connection.serial_baud_rate)
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

        try:
            msg: MAVLink_heartbeat_message = self.mavlink_connection.wait_heartbeat(timeout=10)
            if msg is None:
                raise Exception("Connection failed")

            qInfo("Successfully Received first heartbeat")
            self.setup_for_autopilot.emit(msg.autopilot)
            self.after_heartbeat_successfully_received.emit(self.mavlink_connection)
        except:
            self.after_heartbeat_not_received.emit(self.mavlink_connection)
        self.__parent.connection_wait_wrapper = None
        self.con_thread.quit()

class NoAccentStyle(QProxyStyle):
    def __init__(self, style: QStyle):
        super().__init__(style)

    def drawPrimitive(self, element, option, painter, /, widget=...):
        if element == QStyle.PrimitiveElement.PE_PanelItemViewRow:
            return

        super().drawPrimitive(element, option, painter, widget)


class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    camera_server_connection_dialog: CameraServerConnectionInterface | None = None
    color_selector_dialog: ColorSelectorInterface | None = None
    input_selector_dialog: KeybindingConfigInterface | None = None
    geofence_dialog: SetGeofenceInterface | None = None
    add_ads_dialog: AddADSInterface | None = None
    server_connection: ServerConnection = ServerConnection()
    mavlink_connection: mavfile = None
    color_options: ColorOptions
    mavlink_worker: MavlinkWorker | None = None
    mavlink_thread: QThread | None = None
    next_telemetry: TelemetryData = TelemetryData()
    last_server_telemetry_response: TelemetryResponseData = TelemetryResponseData()
    plane_on_map_update_timer: QTimer = QTimer(interval=500)
    current_lang: int
    current_pilot: int

    def __init__(self):
        QMainWindow.__init__(self)
        self.current_lang = 0
        self.translator = QTranslator()
        self.setStyle(NoAccentStyle(self.style()))
        self.color_options = ColorOptions()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_colors()

        self.kamikaze_state = KamikazeState.IDLE
        self.kamikaze_original_alt = 0.0
        self.kamikaze_target_lat = 0.0
        self.kamikaze_target_lon = 0.0
        self.kamikaze_previous_mode = None
        self.kamikaze_timer = QTimer(self)
        self.kamikaze_timer.setInterval(100)
        self.kamikaze_timer.timeout.connect(self.__kamikaze_loop)
        self.waits_for_qr = False

        self.current_pilot = MAV_AUTOPILOT_INVALID

        self.ui.actionConfigurate_UAV.triggered.connect(self._actionConfigurate_UAV)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)
        self.ui.actionSet_Colors.triggered.connect(self._actionConfigurateSetColors)

        add_to_watch_menu: QMenu = QMenu(parent=self)

        for e in TrackableDataEnum:
            action: QAction = QAction(text=e.value[1](), parent=self)
            action.setObjectName(str(e.value[0]))
            action.triggered.connect(partial(self.add_to_watch_list, e))
            TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]] = action
            if e.value[5]:
                add_to_watch_menu.addAction(action)

        self.ui.add_to_watch.setMenu(add_to_watch_menu)

        for e in TrackableDataEnum:
            if e.value[5] and e.value[6]:
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
        self.ui.fly_mode_combobox.activated.connect(self._change_index)
        self.ui.get_kamikaze_coords_from_server.clicked.connect(self.__get_kamikaze_coords)
        self.ui.start_kamikaze.clicked.connect(self.__start_kamikaze)
        self.ui.force_end_task.clicked.connect(self.__force_end_task)
        self.ui.set_fence.clicked.connect(self.__set_fence_clicked)
        self.ui.arm_mode.activated.connect(self.__setArmStatus)
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
        self.input_config = KeybindingConfigInterface.initialize_mappings()
        self.ui.map_view.set_input_config_reference(lambda: self.input_config)
        self.ui.actionChange_Input_Mapping.triggered.connect(self.__open_input_map_config_dialog)
        floatValidator: QDoubleValidator = QDoubleValidator(parent=self)
        floatValidator.setNotation(QDoubleValidator.Notation.StandardNotation)
        self.ui.kamikaze_latitude.setValidator(floatValidator)
        self.ui.kamikaze_longitude.setValidator(floatValidator)
        self.update_plane_data_signal.connect(self.__update_plane_data)
        self.ui.camera_view.qr_successfully_readed.connect(self.on_qr_found)
        self.ui.disable_enable_locking.toggled.emit(self.ui.camera_view.change_lock_state)
        self.ui.camera_view.set_mainwindow_reference(self)

    def setup_colors(self):
        self.setStyleSheet(ColorSelectorInterface.create_stylesheet(self.color_options))

    def change_autopilot(self, new_autopilot: int):
        if self.current_pilot == new_autopilot:
            return
        self.current_pilot = new_autopilot
        if new_autopilot == MAV_AUTOPILOT_PX4:
            self.ui.fly_mode_combobox.clear()
            for mode in PX4_UAV_Modes:
                self.ui.fly_mode_combobox.insertItem(mode.value[0], mode.value[1])
                if mode.value[2] == 0:
                    self.ui.fly_mode_combobox.view().setRowHidden(mode.value[0], True)
        elif new_autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.ui.fly_mode_combobox.clear()
            for mode in Ardupilot_UAV_Modes:
                self.ui.fly_mode_combobox.insertItem(mode.value[0], mode.value[1])
                if mode.value[2] == -1:
                    self.ui.fly_mode_combobox.view().setRowHidden(mode.value[0], True)
        self.ui.fly_mode_combobox.setCurrentIndex(-1)

    input_config: dict[KeybindingsEnum, InputMapping]
    def __open_input_map_config_dialog(self):
        if self.input_selector_dialog is not None:
            return
        self.input_selector_dialog = KeybindingConfigInterface(self, self.input_config)
        self.input_selector_dialog.finished.connect(self._input_map_close_screen)
        self.input_selector_dialog.show()

    def _input_map_close_screen(self):
        self.input_selector_dialog = None

    def closeEvent(self, event, /):
        try:
            self._uav_disconnect()
        except:
            pass
        try:
            self._server_disconnect()
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

    def mission_waypoint_item_received(self, command: int, x: float, y: float, seq: int, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_waypoint_received(coord, command, seq, False, count)

    def mission_waypoint_item_int_received(self, command: int, x: float, y: float, seq: int, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_waypoint_received(coord, command, seq, True, count)

    def request_fence_data(self):
        if self.mavlink_connection is None:
            return
        if self.requested_to_get_fence:
            qDebug("Tried to get fence when already getting fence from uav")
            return
        self.requested_to_get_fence = True
        self.coord_counter = 1
        self.next_fence_order_seq_id = 0
        self.mavlink_connection.mav.mission_request_list_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_FENCE)
        self.fence_download_timout.start()

    def request_fence_data_timeout(self):
        self._create_warning("Fence Download taking really long, probably vehicle connection has been lost")
        self.next_fence_order_seq_id = 0
        self.requested_to_get_fence = False
        self.ui.map_view.user_ads_data_model.layoutChanged.emit()
        self.coord_counter = 1

    coord_counter: int = 1
    next_fence_order_seq_id: int = 0
    requested_to_get_fence: bool = False
    def mission_fence_item_int_received(self, command: int, x: float, y: float, seq: int, ads_size: float, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_fence_received(coord, command, seq, ads_size, True, count)

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

    def mission_fence_item_received(self, command: int, x: float, y: float, seq: int, ads_size: float, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_fence_received(coord, command, seq, ads_size, False, count)

    def _about(self):
        QMessageBox.about(self, QCoreApplication.translate("MainWindow", u"About", None), QCoreApplication.translate("MainWindow", "Designed and developed by Muzaffer Kadir Belen to be used by ARES teknofest team", None))

    def _remove_ads(self):
        self.ui.map_view.mouse_input_handler.remove_selected_ads(None, 0, 0)

    def _actionConfigurateCameraServer(self):
        if self.camera_server_connection_dialog is not None:
            return
        self.camera_server_connection_dialog = CameraServerConnectionInterface(self)
        for protocol in CameraServerProtocol:
            self.camera_server_connection_dialog.ui.server_protocol_type.insertItem(protocol.value[0], protocol.value[1])
        self.camera_server_connection_dialog.show()
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Connected :)", None))
            self.camera_server_connection_dialog.ui.server_ip_input.setText(self.ui.camera_view.camera_server_info.ip)
            self.camera_server_connection_dialog.ui.camera_width.setText(str(self.ui.camera_view.camera_server_info.width))
            self.camera_server_connection_dialog.ui.camera_height.setText(str(self.ui.camera_view.camera_server_info.height))
            self.camera_server_connection_dialog.ui.server_protocol_type.setCurrentIndex(self.ui.camera_view.camera_server_info.protocol.value[0])
        self.camera_server_connection_dialog.ui.connect.clicked.connect(self.connect_to_cam_server)
        self.camera_server_connection_dialog.ui.disconnect.clicked.connect(self.disconnect_from_cam_server)
        self.camera_server_connection_dialog.finished.connect(self.reset_camera_server_dialog)

    def connect_to_cam_server(self):
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.ui.camera_view.disconnect_from_server()
        if not self.camera_server_connection_dialog.ui.camera_height.text():
            self.camera_server_connection_dialog.ui.camera_height.setText(self.camera_server_connection_dialog.ui.camera_height.placeholderText())
        if not self.camera_server_connection_dialog.ui.camera_width.text():
            self.camera_server_connection_dialog.ui.camera_width.setText(self.camera_server_connection_dialog.ui.camera_width.placeholderText())
        ip_and_port = self.camera_server_connection_dialog.ui.server_ip_input.text()
        if ":" in ip_and_port:
            ip_and_port = ip_and_port.split(":")
            if len(ip_and_port) != 2:
                self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
                return
            else:
                ip: str = ip_and_port[0]
                port: int
                try:
                    port = int(ip_and_port[1])
                except ValueError:
                    self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
                    return
                self.ui.camera_view.camera_server_info.ip = ip
                self.ui.camera_view.camera_server_info.port = port
        self.ui.camera_view.camera_server_info.width = int(self.camera_server_connection_dialog.ui.camera_width.text())
        self.ui.camera_view.camera_server_info.height = int(self.camera_server_connection_dialog.ui.camera_height.text())
        self.ui.camera_view.camera_server_info.recalculate_frame_size()
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
                   self.ui.map_view.mouse_input_handler.ard_dialog,
                   self.input_selector_dialog]

    def retranslateOpenDialogs(self):
        dialogs = self.get_all_dialogs()

        for dialog in dialogs:
            if dialog is not None:
                dialog.ui.retranslateUi(dialog)

        if self.color_selector_dialog is not None:
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
        radius: float
        latitude: float
        longitude: float
        try:
            radius = float(self.add_ads_dialog.ui.ads_radius.text())
            latitude = float(self.add_ads_dialog.ui.ads_latitude.text())
            longitude = float(self.add_ads_dialog.ui.ads_longitude.text())
        except:
            return
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
        if self.current_pilot != MAV_AUTOPILOT_PX4:
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
        if self.mavlink_connection is None:
            qWarning("Tried to sent geofence data when there is no mavlink connection")
            return
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
        self._create_warning("Fence Upload taking really long, probably vehicle connection has been lost")
        self.ads_list_cache = []
        self.requested_to_send_fence_with_fence = False

    def __reset_geofence_dialog(self):
        self.geofence_dialog = None

    kamikaze_start: GpsSaati
    kamikaze_state: KamikazeState
    kamikaze_original_alt: float
    kamikaze_target_lat: float
    kamikaze_target_lon: float
    kamikaze_previous_mode: int | None
    kamikaze_timer: QTimer
    waits_for_qr: bool

    def __set_param(self, name: bytes, value: float):
        if self.mavlink_connection is None:
            return
        self.mavlink_connection.mav.param_set_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            name,
            value,
            9
        )

    def __start_kamikaze(self):
        if self.kamikaze_state != KamikazeState.IDLE:
            qDebug("Cancelling Kamikaze")
            self.__finish_kamikaze()
            return

        if self.mavlink_connection is None:
            qWarning("No UAV Connection, Can not Start Kamikaze")
            return
        try:
            latitude: float = float(self.ui.kamikaze_latitude.text())
            longitude: float = float(self.ui.kamikaze_longitude.text())
        except ValueError:
            qWarning("Invalid Kamikaze Coordinates")
            return

        self.kamikaze_target_lat = latitude
        self.kamikaze_target_lon = longitude

        self.next_telemetry.lock.lockForRead()
        self.kamikaze_start = self.next_telemetry.gps_saati
        self.kamikaze_original_alt = self.next_telemetry.iha_irtifa
        self.next_telemetry.lock.unlock()
        if self.kamikaze_original_alt <= 0:
            qWarning("No Valid UAV Info Found, Cancelling Kamikaze")
            return
        if not self.ui.arm_mode.currentIndex() == 1:
            self._create_warning("UAV is not armed, refusing to start kamikaze")
            return
        if self.kamikaze_original_alt < 80.0:
            self._create_warning("Altitude %.1fm is below the 80m kamikaze minimum, refusing to start" % self.kamikaze_original_alt)
            return

        self.kamikaze_previous_mode = self.ui.fly_mode_combobox.currentIndex()
        self.kamikaze_state = KamikazeState.APPROACHING
        self.kamikaze_timer.start()

        # Dive pitch limit -45deg and turn/roll limit 55deg. Set both the old
        # (centidegree: LIM_*) and new (degree: *_DEG) ArduPlane parameter names so
        # this works regardless of firmware version (4.1+ renamed these params).
        # THR_MAX must be 100 here: FBWA clamps even RC-override throttle to it,
        # and a still-active reposition may have left it at 60.
        self.__set_param(b'THR_MAX', 100.0)
        self.__set_param(b'PTCH_LIM_MIN_DEG', -45.0)
        self.__set_param(b'LIM_PITCH_MIN', -4500.0)
        self.__set_param(b'ROLL_LIMIT_DEG', 55.0)
        self.__set_param(b'LIM_ROLL_CD', 5500.0)
        self.mavlink_connection.set_mode_apm(5)
        self.waits_for_qr = True
        qDebug("Kamikaze Started")

    def __kamikaze_loop(self):
        if self.kamikaze_state == KamikazeState.IDLE:
            return

        self.next_telemetry.lock.lockForRead()
        current_lat: float = self.next_telemetry.iha_enlem
        current_lon: float = self.next_telemetry.iha_boylam
        current_alt: float = self.next_telemetry.iha_irtifa
        current_pitch: float = self.next_telemetry.iha_dikilme
        self.next_telemetry.lock.unlock()

        if current_lat == 0 and current_lon == 0:
            return

        distance: float = MainWindow.__calculate_distance(current_lat, current_lon, self.kamikaze_target_lat, self.kamikaze_target_lon)
        roll_ch: int = self.__heading_error_to_roll_channel()

        if self.kamikaze_state == KamikazeState.APPROACHING:
            alt_error: float = 120.0 - current_alt
            pitch_offset: int = int(alt_error * 5)
            pitch_ch: int = 1500 + max(-200, min(200, pitch_offset))
            self.__send_rc_override(roll_ch, pitch_ch, 1550)
            if distance <= 105.0:
                qDebug(f"Remaining Distance is {distance:.1f}m, entering dive state")
                self.mavlink_connection.set_mode_apm(5)
                self.kamikaze_state = KamikazeState.DIVING

        elif self.kamikaze_state == KamikazeState.DIVING:
            self.__send_rc_override(1500, 1000, 1000)
            if current_alt <= 70.0:
                qDebug("Achieved target altitude, entering recovering state")
                self.mavlink_connection.set_mode_apm(5)
                self.kamikaze_state = KamikazeState.RECOVERING

        elif self.kamikaze_state == KamikazeState.RECOVERING:
            throttle_ch: int = 1950 if current_pitch > 0.0 else 1000
            self.__send_rc_override(1500, 1900, throttle_ch)
            if current_alt >= 100.0:
                qDebug("Recovering complete, returning to last mode")
                self.kamikaze_state = KamikazeState.RESUMING
                self.__finish_kamikaze()

    @staticmethod
    def __calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def __bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return math.atan2(x, y)

    def __heading_error_to_roll_channel(self) -> int:
        self.next_telemetry.lock.lockForRead()
        enlem: float = self.next_telemetry.iha_enlem
        boylam: float = self.next_telemetry.iha_boylam
        yaw: float = self.next_telemetry.iha_yonelme
        self.next_telemetry.lock.unlock()
        bearing: float = MainWindow.__bearing_to(
            enlem, boylam,
            self.kamikaze_target_lat, self.kamikaze_target_lon
        )
        current_heading: float = math.radians(yaw)
        heading_error: float = math.atan2(math.sin(bearing - current_heading), math.cos(bearing - current_heading))
        steer: float = max(-1.0, min(1.0, heading_error * 0.8))
        roll_ch: int = 1500 + int(steer * 500)
        return max(1000, min(2000, roll_ch))

    def __send_rc_override(self, roll_ch: int, pitch_ch: int, throttle_ch: int):
        if self.mavlink_connection is None:
            return
        self.mavlink_connection.mav.rc_channels_override_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            roll_ch,
            pitch_ch,
            throttle_ch,
            1500,
            0, 0, 0, 0
        )

    def __finish_kamikaze(self):
        self.kamikaze_timer.stop()
        self.kamikaze_state = KamikazeState.IDLE
        if self.mavlink_connection is not None:
            self.mavlink_connection.mav.rc_channels_override_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.__set_param(b'PTCH_LIM_MIN_DEG', -25.0)
            self.__set_param(b'LIM_PITCH_MIN', -2500.0)
            self.__set_param(b'ROLL_LIMIT_DEG', 55.0)
            self.__set_param(b'LIM_ROLL_CD', 5500.0)
            # Kamikaze start raised THR_MAX to 100 for the recovery burst;
            # bring the cruise ceiling back once the run is over.
            self.__set_param(b'THR_MAX', CRUISE_THR_MAX)
        if self.kamikaze_previous_mode is not None and self.kamikaze_previous_mode >= 0 and self.mavlink_connection is not None:
            self.mavlink_connection.set_mode_apm(self.kamikaze_previous_mode)
        if self.server_connection.ip is not None:
            self.on_kamikaze_end("")
        self.waits_for_qr = False
        qDebug("Kamikaze Completed")

    def on_qr_found(self, qr_text: str):
        self.waits_for_qr = False

    def on_kamikaze_end(self, qr_text: str) -> None:
        self.next_telemetry.lock.lockForRead()
        kamikaze_end = self.next_telemetry.gps_saati
        self.next_telemetry.lock.unlock()
        try:
            send_kamikaze(self.server_connection.get_address(), self.kamikaze_start, kamikaze_end, qr_text)
            qInfo("Kamikaze information sent with start: %s, end: %s, text: %s" % (self.kamikaze_start, kamikaze_end, qr_text))
        except Exception as e:
            self._create_warning("Could not send kamikaze info to server: %s" % e)

    def __force_end_task(self):
        if self.mavlink_connection is None:
            return
        qInfo("Force End Task requested by user")
        if self.ui.map_view.target_coord.is_set:
            self.ui.map_view.target_coord.remove_position()
        self.__set_param(b'THR_MAX', CRUISE_THR_MAX)
        self.__set_param(b'ROLL_LIMIT_DEG', 55.0)
        self.__set_param(b'LIM_ROLL_CD', 5500.0)
        if self.kamikaze_state != KamikazeState.IDLE:
            self.kamikaze_previous_mode = 10  # resume the AUTO mission route
            self.__finish_kamikaze()
        else:
            self.mavlink_connection.set_mode_apm(10)
        self._create_warning("Task force-ended, returning to AUTO mission")

    def __get_kamikaze_coords(self):
        if self.server_connection.ip is None:
            return
        try:
            qr_coords: QrCoords = get_kamikaze_coords(self.server_connection.get_address())
        except Exception as e:
            self._create_warning("Could not get kamikaze coords from server: %s" % e)
            return
        self.ui.kamikaze_longitude.setText(str(qr_coords.qrBoylam))
        self.ui.kamikaze_latitude.setText(str(qr_coords.qrEnlem))

    def _change_index(self, index: int):
        if self.current_pilot == MAV_AUTOPILOT_PX4:
            base_mode = index_to_px4_uav_mode[index].value[2]
            sub_mode = index_to_px4_uav_mode[index].value[3]
            qDebug("Sending standard mode with mode %s, sub mode %s" % (base_mode, sub_mode))
            self.mavlink_connection.set_mode_px4(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, base_mode, sub_mode)
        else:
            qDebug("Sending mode with index: %s" % index)

            self.mavlink_connection.set_mode_apm(index)

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
            if self.server_connection.port:
                self.server_connection_dialog.ui.server_port_input.setText(str(self.server_connection.port))
            self.server_connection_dialog.ui.server_login_username_input.setText(str(self.server_connection.username))
            self.server_connection_dialog.ui.server_login_password_input.setText(str(self.server_connection.password))
        self.server_connection_dialog.ui.connect.clicked.connect(lambda: self._server_connect(self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect())
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

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
            self.uav_connection_dialog.ui.connection_type.addItem(availablePort.portName())
        if self.uav_connection.connection_type is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                self.uav_connection_dialog.ui.serial_baud.setText(str(self.uav_connection.serial_baud_rate))
                idx = self.uav_connection_dialog.ui.connection_type.findText(self.uav_connection.serial_port)
                if idx >= 0:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(idx)
                else:
                    qWarning("Can not find index for serial port: %s" % self.uav_connection.serial_port)
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
            self.uav_connection.serial_port = self.uav_connection_dialog.ui.connection_type.currentText()
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()

        if self.connection_wait_wrapper is not None:
            qWarning("Tried to press connect when trying to connect, ignoring")
            return
        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type
        self.connection_wait_wrapper = ConnectionWaitWrapper(self, self.uav_connection)
        self.connection_wait_wrapper.setup_for_autopilot.connect(self.change_autopilot)
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
        if self.uav_connection_dialog is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))
            self.uav_connection_dialog.ui.invalid_input_error_label.show()

        if self.uav_connection.connection_type == ConnectionType.SERIAL:
            qInfo("Can not connect to UAV from %s" % (self.uav_connection.serial_port + "," + str(self.uav_connection.serial_baud_rate)))
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
        if self.uav_connection_dialog is not None:
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
        self._update_time_with_mavlink()
        self.__set_param(b'ROLL_LIMIT_DEG', 55.0)
        self.__set_param(b'LIM_ROLL_CD', 5500.0)
        # Full power only while the NAV_TAKEOFF item is active; TECS is capped
        # at CRUISE_THR_MAX for the rest of the flight.
        self.__set_param(b'TKOFF_THR_MAX', 100.0)
        self.__set_param(b'THR_MAX', CRUISE_THR_MAX)

        self.ui.map_view.mavlink_connection = self.mavlink_connection
        self.mavlink_thread = QThread(self)
        self.mavlink_thread.setObjectName("Mavlink Connection Thread")
        self.mavlink_worker = MavlinkWorker(self.mavlink_connection, self)
        self.mavlink_worker.running = True
        self.mavlink_worker.update_watch_list.connect(self._apply_watch_update)
        self.mavlink_worker.create_warning.connect(self._create_warning)
        self.mavlink_worker.connection_lost.connect(self.__on_connection_lost)
        self.mavlink_worker.mission_fence_item_received.connect(self.mission_fence_item_received)
        self.mavlink_worker.mission_fence_item_int_received.connect(self.mission_fence_item_int_received)
        self.mavlink_worker.mission_waypoint_item_received.connect(self.mission_waypoint_item_received)
        self.mavlink_worker.mission_waypoint_item_int_received.connect(self.mission_waypoint_item_int_received)
        self.mavlink_worker.send_fence_mission_data.connect(self.send_fence_mission_data)
        self.mavlink_worker.remove_reposition_location.connect(self.remove_reposition_location)
        self.mavlink_worker.worker_signals.set_arm_mode.connect(self.set_arm_mode)
        self.mavlink_worker.worker_signals.set_fly_mode.connect(self.set_fly_mode)
        self.mavlink_worker.worker_signals.change_autopilot.connect(self.change_autopilot)
        self.mavlink_worker.worker_signals.should_reposition_removed.connect(self.should_reposition_removed)
        self.mavlink_thread.started.connect(self.mavlink_worker.run, Qt.ConnectionType.DirectConnection)
        self.mavlink_worker.moveToThread(self.mavlink_thread)
        self.mavlink_thread.start()
        self.enableFeaturesAfterUAVConnected()

    def remove_reposition_location(self):
        if self.ui.map_view.target_coord.is_set:
            self.ui.map_view.target_coord.remove_position()

    def should_reposition_removed(self):
        if self.ui.map_view.target_coord.is_set and not self.ui.map_view.reposition_timer.isActive():
            self.ui.map_view.target_coord.remove_position()
            if self.mavlink_connection is not None and self.kamikaze_state == KamikazeState.IDLE:
                self.ui.map_view.mouse_input_handler._set_thr_max(CRUISE_THR_MAX)
                self.ui.map_view.mouse_input_handler._set_roll_limit(55.0)

    def set_arm_mode(self, index: int):
        if self.ui.arm_mode.currentIndex() != index:
            self.ui.arm_mode.setCurrentIndex(index)

    def set_fly_mode(self, index: int):
        if self.ui.fly_mode_combobox.currentIndex() != index:
            self.ui.fly_mode_combobox.setCurrentIndex(index)

    def _update_time_with_mavlink(self):
        time_ns = QDateTime.currentDateTimeUtc().toMSecsSinceEpoch() * 1000000
        time_ns += 1234 # Copied from mavproxy
        self.mavlink_connection.mav.timesync_send(0, time_ns)

    def _apply_watch_update(self, row: int, value: str):
        self.ui.watch_list.setItem(row, 3, QTableWidgetItem(value))

    def _create_warning(self, text: str) -> None:
        qWarning(text)
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


    def __on_connection_lost(self, reason: str):
        self._create_warning("UAV connection lost: %s" % reason)
        if self.kamikaze_state != KamikazeState.IDLE:
            self.kamikaze_timer.stop()
            self.kamikaze_state = KamikazeState.IDLE
            self.waits_for_qr = False
        self._uav_disconnect()

    def _uav_disconnect(self):
        if self.mavlink_connection is None:
            return
        self.mavlink_worker.running = False
        self.mavlink_thread.quit()
        self.mavlink_thread.wait()
        try:
            self.mavlink_connection.close()
        except Exception as e:
            qWarning("Error while closing MAVLink connection: %s" % e)
        self.mavlink_connection = None
        self.ui.map_view.mavlink_connection = None
        self.uav_connection.reset_connection_properties()
        self.disableFeaturesAfterUAVDisconnected()
        self.next_telemetry = TelemetryData()
        self.resetWatcherWidgetValues()
        self.ui.fly_mode_combobox.setCurrentIndex(-1)
        self.ui.arm_mode.setCurrentIndex(-1)

    def _server_connect(self, dialog: ServerConnectionInterface):
        if self.server_connection.ip:
            qDebug("Server already connected, disconnecting")
            self._server_disconnect()
        # TODO: Test connection
        if not dialog.ui.server_ip_input.text():
            dialog.ui.server_ip_input.setText(dialog.ui.server_ip_input.placeholderText())
        is_it_direct_address: bool = QRegularExpression("[\\w]+").match(dialog.ui.server_ip_input.text()).hasMatch()
        if not is_it_direct_address and not dialog.ui.server_port_input.text():
            dialog.ui.server_port_input.setText(dialog.ui.server_port_input.placeholderText())
        if not dialog.ui.server_login_username_input.text():
            dialog.ui.server_login_username_input.setText(dialog.ui.server_login_username_input.placeholderText())
        if not dialog.ui.server_login_password_input.text():
            dialog.ui.invalid_input_error_label.show()
            return
        dialog.ui.invalid_input_error_label.hide()
        if not is_it_direct_address:
            self.server_connection.port = int(dialog.ui.server_port_input.text())
        self.server_connection.ip = dialog.ui.server_ip_input.text()
        if not ("://" in self.server_connection.ip):
            self.server_connection.ip = "http://"+self.server_connection.ip
        self.server_connection.username = dialog.ui.server_login_username_input.text()
        self.server_connection.password = dialog.ui.server_login_password_input.text()

        try:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Trying to connect to server :O", None))
            self.server_connection.team_no = login_to_server(self.server_connection.get_address(), self.server_connection.username, self.server_connection.password)
            self.next_telemetry.lock.lockForWrite()
            self.next_telemetry.takim_numarasi = self.server_connection.team_no
            self.next_telemetry.lock.unlock()
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Server Connected :)", None))
            self.ui.server_connection_warning.hide()
            qInfo("Connected to server with ip: %s, username: %s" % (self.server_connection.get_address(), self.server_connection.username))
        except Exception as e:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Can't Connect To Server :(", None))
            qWarning("Can not connect to server: %s" % e)
            dialog.ui.invalid_input_error_label.show()
            self.ui.server_connection_warning.show()
            self.server_connection.ip = None
            self.server_connection.port = None
            return
        if self.plane_on_map_update_timer.isActive():
            self.plane_on_map_update_timer.stop()
        self.server_connection.telemetry_timer = QTimer()
        self.server_connection.telemetry_timer.setInterval(700)
        self.server_connection.telemetry_timer.timeout.connect(self.__send_telemetry, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread = QThread(self)
        self.server_connection.telemetry_thread.setObjectName("Telemetry Thread")
        self.server_connection.telemetry_timer.moveToThread(self.server_connection.telemetry_thread)
        self.server_connection.telemetry_thread.started.connect(self.server_connection.telemetry_timer.start, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread.finished.connect(self.server_connection.telemetry_timer.stop, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread.start()

    def __update_plane_on_map_without_server(self):
        if self.mavlink_connection is None or self.server_connection.ip is not None:
            self.plane_on_map_update_timer.stop()
            return
        self.next_telemetry.lock.lockForRead()
        enlem: float = self.next_telemetry.iha_enlem
        boylam: float = self.next_telemetry.iha_boylam
        yaw: float = self.next_telemetry.iha_yonelme
        self.next_telemetry.lock.unlock()
        self.ui.map_view.update_plane_data_without_server(QGeoCoordinate(enlem, boylam), yaw)

    def __send_telemetry(self):
        if self.mavlink_connection is None:
            qDebug("UAV not connected")
            return
        global SERVER_IS_UNREACHABLE_COUNTER
        if SERVER_IS_UNREACHABLE_COUNTER > 100:
            SERVER_IS_UNREACHABLE_COUNTER = 0
            qWarning("Server connection is not possible for 100 time, disconnecting")
            self._server_disconnect()
            return
        self.next_telemetry.lock.lockForRead()
        try:
            telemetry_snapshot = copy.copy(self.next_telemetry)
        finally:
            self.next_telemetry.lock.unlock()
        qDebug("Sending telemetry at %s" % QDateTime.currentDateTime().toString())
        response = send_telemetry(self.server_connection.get_address(), telemetry_snapshot)
        if response:
            self.last_server_telemetry_response = response
            self.update_plane_data_signal.emit(telemetry_snapshot, response)
        else:
            qWarning("Could not process telemetry response info")
    update_plane_data_signal = Signal(TelemetryData, TelemetryResponseData)
    is_processing_plane_data = False
    def __update_plane_data(self, our_data: TelemetryData, telemetry_response: TelemetryResponseData):
        if self.is_processing_plane_data:
            return
        self.is_processing_plane_data = True
        try:
            self.ui.map_view.update_plane_data(our_data, telemetry_response)
        finally:
            self.is_processing_plane_data = False

    def _server_disconnect(self):
        if self.server_connection.telemetry_thread:
            self.server_connection.telemetry_thread.quit()
            self.server_connection.telemetry_thread.wait()
            self.server_connection.telemetry_thread.deleteLater()
            self.server_connection.telemetry_thread = None
        if self.server_connection.ip is None:
            return
        self.server_connection.ip = None
        qInfo("Disconnected from server")

    @staticmethod
    def is_ip_address_valid(ip_address: str, must_have_port: bool) -> bool:
        if ip_address is not None:
            ip_address = ip_address.strip()
            ip_with_port: str = ip_address
            split: list[str] = ip_with_port.split(':')
            ip: str = split[0]
            if len(split) > 1:
                try:
                    port: int = int(split[1])
                    if port < 0 or port > 65535:
                        return False
                except:
                    return False
            elif must_have_port:
                return False # Must have port, yes that's the comment :3
            ip_array: list[str] = ip.split('.')
            if len(ip_array) != 4:
                return False
            for e in ip_array:
                try:
                    e: int = int(e)
                    if e < 0 or e > 255:
                        return False
                except:
                    return False
            return True
        return False

