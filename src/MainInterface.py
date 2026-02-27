from enum import Enum
from functools import partial

from PySide6.QtCore import Qt, QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug
from PySide6.QtGui import QAction, QStandardItem
from PySide6.QtWidgets import QMainWindow, QPushButton, QToolButton, QTableWidgetItem, QMenu
from pymavlink.dialects.v10.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src import ServerConnection
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, TELEMETRY_DATA
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.siha_interface import Ui_MainWindow

class TrackableDataUpdate:
    @staticmethod
    def update_ground_speed(packet: MAVLink_vfr_hud_message) -> str:
        return str(packet.groundspeed)
    @staticmethod
    def update_air_speed(packet: MAVLink_vfr_hud_message) -> str:
        TELEMETRY_DATA.iha_hiz = packet.airspeed
        return str(packet.airspeed)
    @staticmethod
    def update_velocity(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.vel)
    @staticmethod
    def update_altitude(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.alt)
    @staticmethod
    def update_longitude(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.lon)
    @staticmethod
    def update_latitude(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.lat)
    @staticmethod
    def update_yaw(packet: MAVLink_attitude_message) -> str:
        TELEMETRY_DATA.iha_yonelme = packet.yaw
        return str(packet.yaw)
    @staticmethod
    def update_pitch(packet: MAVLink_attitude_message) -> str:
        TELEMETRY_DATA.iha_dikilme = packet.pitch
        return str(packet.pitch)
    @staticmethod
    def update_roll(packet: MAVLink_attitude_message) -> str:
        TELEMETRY_DATA.iha_yatis = packet.roll
        return str(packet.roll)
    @staticmethod
    def update_gps_time(packet: MAVLink_gps_raw_int_message) -> str:
        TELEMETRY_DATA.gps_saati = GpsSaati(packet.time_usec)
        return QDateTime.fromMSecsSinceEpoch(packet.time_usec).toString()
    @staticmethod
    def update_battery_percentage(packet: MAVLink_battery_status_message) -> str:
        TELEMETRY_DATA.iha_batarya = packet.battery_remaining
        return str(packet.battery_remaining) + "%"

def check_mavlink_msg_is_valid(msg) -> bool:
    if not msg:
        return False
    if msg.get_type() == "BAD_DATA":
        if all_printable(msg.data):
            qWarning("Invalid data received: %s" % msg.data)
        return False
    return True

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataPacketTimer(Enum):
    # (msg id, msg name, type, update interval (ms), watch value ids that uses this packet)
    BATTERY_STATUS = (147, "BATTERY_STATUS", MAVLink_battery_status_message, 1500, [10])
    ATTITUDE = (30, "ATTITUDE", MAVLink_attitude_message, 100, [3, 4, 5])
    GPS_RAW_INT = (24, "GPS_RAW_INT", MAVLink_gps_raw_int_message, 100, [1, 2, 7, 8, 9])
    VFR_HUD = (74, "VFR_HUD", MAVLink_vfr_hud_message, 100, [0, 6])

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, is it telemetry data)
    GROUND_SPEED = (0, "Ground Speed", TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, False)
    VELOCITY = (1, "Velocity", TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False)
    ALTITUDE = (2, "Altitude", TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GPS_RAW_INT, False)
    YAW = (3, "Yaw", TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True)
    PITCH = (4, "Pitch", TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True)
    ROLL = (5, "Roll", TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True)
    AIR_SPEED = (6, "Air Speed", TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, True)
    GPS_TIME = (7, "GPS Time", TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.GPS_RAW_INT, True)
    LONGITUDE = (8, "Longitude", TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, False)
    LATITUDE = (9, "Latitude", TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, False)
    BATTERY_PERCENTAGE = (10, "Battery Percentage", TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True)

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

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    server_connection: ServerConnection = ServerConnection()
    trackable_packet_timers: dict[TrackableDataPacketTimer, QTimer] = dict()
    watcher_datas: list[int] = list()
    mavlink_connection: mavfile
    force_sending_telemetry: bool = False

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

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
            self.add_to_watch_list(e)

        self.ui.remove_from_watch.clicked.connect(self.__remove_from_watch_signal)
        self.ui.watch_list.setColumnHidden(0, True) # hide id column
        self.ui.fly_mode_combobox.currentIndexChanged.connect(self._change_index)
        self.ui.actionForce_Send_Testing_Telemetry_Data.triggered.connect(self.__change_state_for_force_sending_telemetry)

    def __change_state_for_force_sending_telemetry(self, state: bool):
        self.force_sending_telemetry = state

    def _change_index(self, index: int):
        # TODO: Is some packet needed for mavlink? IDK Someone help meeeeee
        TELEMETRY_DATA.iha_otonom = index

    def add_to_watch_list(self, e: TrackableDataEnum):
        if e.value[0] in self.watcher_datas:
            return
        rowCount: int = self.ui.watch_list.rowCount()
        self.ui.watch_list.setRowCount(rowCount + 1)

        self.ui.watch_list.setItem(rowCount, 0, QTableWidgetItem(str(e.value[0])))
        self.ui.watch_list.setItem(rowCount, 1, QTableWidgetItem(e.value[1]))
        self.ui.watch_list.setItem(rowCount, 2, QTableWidgetItem(""))
        self.ui.watch_list.setItem(rowCount, 3, QTableWidgetItem("Unknown"))

        TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].setDisabled(True)

        packet_timer_enum: TrackableDataPacketTimer = e.value[3]

        self.watcher_datas.append(e.value[0])
        if not packet_timer_enum in self.trackable_packet_timers:
            timer: QTimer = QTimer()
            timer.setInterval(packet_timer_enum.value[3])
            timer.timeout.connect(lambda: self.trigger_packet_update(packet_timer_enum))
            self.trigger_packet_update(packet_timer_enum) # fire once when adding to try to set data
            timer.start()
            self.trackable_packet_timers[packet_timer_enum] = timer

    def trigger_packet_update(self, e: TrackableDataPacketTimer):
        if self.uav_connection.connection_type is None:
            return
        qInfo("Tried to update with packet %s" % e.value[1])
        packet = self.mavlink_connection.recv_match(e.value[1])
        if check_mavlink_msg_is_valid(packet):
            data_enum_values = e.value[4]
            for i in data_enum_values:
                self.trigger_update_value(TrackableDataEnum.from_id(i), packet)

    def trigger_update_value(self, e: TrackableDataEnum, packet):
        length: int = self.ui.watch_list.rowCount()
        new_val: str | None = None
        if e.value[4]:
            new_val = e.value[2](packet) # Update if it is telemetry without caring it is in watch list or not
        for i in range(length):
            if self.ui.watch_list.item(i, 0) == e.value[0]:
                if new_val is None:
                    new_val = e.value[2](packet)
                self.ui.watch_list.setItem(i, 3, QStandardItem(new_val))
                break

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

                if data_id in self.watcher_datas:
                    self.watcher_datas.remove(data_id)
                    data_enum: TrackableDataEnum = TrackableDataEnum.from_id(data_id)
                    t = data_enum.value[3]
                    all_ids = t.value[4]
                    has_no_data_to_watch = True
                    for one_of_all_values in all_ids:
                        if one_of_all_values in self.watcher_datas or TrackableDataEnum.from_id(one_of_all_values).value[4]:
                            has_no_data_to_watch = False
                    if has_no_data_to_watch:
                        self.trackable_packet_timers.pop(t).stop()
                else:
                    qWarning("Something is broken, watcher_datas does not has trackable data?")
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
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect(button, self.server_connection_dialog))
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    def _actionConfigurate_UAV(self):
        if self.uav_connection_dialog is not None:
            return
        self.uav_connection_dialog = FightingUAVConnectionInterface(self)
        self.uav_connection_dialog.show()
        if self.uav_connection.connection_type is not None:
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                self.uav_connection_dialog.ui.serial_band = self.uav_connection.serial_band
                self.uav_connection_dialog.ui.connection_type.setCurrentIndex(2 + self.uav_connection.serial_port)
            else:
                isTCP: bool = self.uav_connection_dialog.connection_type == ConnectionType.TCP
                self.uav_connection_dialog.ui.ip_address = self.uav_connection.ip
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
        except OSError as e:
            qWarning("Tried a invalid connection: %s" % str(e))
            return

        try:
            self.mavlink_connection.wait_heartbeat(timeout=10)
        except:
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
            return
        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type # Only set connection_type after successfully connecting
        self.ui.camera_view.startUpdater()
        self.ui.device_connection_warning.hide()

    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        if self.uav_connection.connection_type is None:
            return
        self.ui.camera_view.stopUpdater()
        self.uav_connection.connection_type = None

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
            TELEMETRY_DATA.takim_numarasi = self.server_connection.team_no
            dialog.ui.server_connection_text.setText("Server Connected :)")
            self.ui.server_connection_warning.hide()
        except Exception as e:
            dialog.ui.server_connection_text.setText("Can't Connect To Server :(")
            qWarning("Can not connect to server: %s" % str(e))
            dialog.ui.invalid_input_error_label.show()
            self.ui.server_connection_warning.show()
            return
        self.server_connection.telemetry_timer = QTimer()
        self.server_connection.telemetry_timer.setInterval(500)
        self.server_connection.telemetry_timer.timeout.connect(self.__send_telemetry)
        self.server_connection.telemetry_timer.start()

    def __send_telemetry(self):
        if self.uav_connection.connection_type is None and not self.force_sending_telemetry:
            qDebug("UAV not connected")
            return
        qDebug("Sending telemetry")
        ServerConnection.SERVER_TELEMETRY_RESPONSE = send_telemetry(self.server_connection.ip + ":" + str(self.server_connection.port),
                                                                    TELEMETRY_DATA)
        self.ui.map_view.update_plane_data()

    def _server_disconnect(self, button: QPushButton, dialog: ServerConnectionInterface):
        self.server_connection.ip = None

    @staticmethod
    def is_ip_address_valid(ip_address: str, must_have_port: bool) -> bool:
        if ip_address.startswith("192.168.1.1"):
            # This address causes unknown magical errors
            return False # FIXME: Hardcoded check, (I hope) temporary.
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

