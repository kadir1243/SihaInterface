from enum import Enum
from functools import partial

from PySide6.QtCore import Qt, QTimer, QModelIndex, qInfo, qWarning
from PySide6.QtGui import QAction, QStandardItem
from PySide6.QtWidgets import QMainWindow, QPushButton, QToolButton, QTableWidgetItem, QMenu

from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.siha_interface import Ui_MainWindow


class TrackableDataUpdate:
    # TODO: Add these
    @staticmethod
    def update_speed() -> str:
        pass
    @staticmethod
    def update_velocity() -> str:
        pass
    @staticmethod
    def update_altitude() -> str:
        pass
    @staticmethod
    def update_yaw() -> str:
        pass
    @staticmethod
    def update_pitch() -> str:
        pass

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataEnum(Enum):
    # (id, name, update function, update interval (ms))
    # setting update interval to -1 disables timer
    # TODO: Add update interval
    SPEED = (0, "Speed", TrackableDataUpdate.update_speed, -1)
    VELOCITY = (1, "Velocity", TrackableDataUpdate.update_velocity, -1)
    ALTITUDE = (2, "Altitude", TrackableDataUpdate.update_altitude, -1)
    YAW = (3, "Yaw", TrackableDataUpdate.update_yaw, -1)
    PITCH = (4, "Pitch", TrackableDataUpdate.update_pitch, -1)

    @staticmethod
    def list() -> list[TrackableDataEnum]:
        return list(map(lambda c: c, TrackableDataEnum))

    @staticmethod
    def from_id(id: int) -> TrackableDataEnum:
        e: TrackableDataEnum
        for e in TrackableDataEnum.list():
            if e.value[0] == id:
                return e
        return None

class UavConnection:
    connection_type: ConnectionType | None = None
    serial_port: int | None = None
    serial_band: int | None = None
    ip: str | None = None

class TrackableData:
    timer: QTimer
    timer_started: bool = False

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    watcher_datas: dict[int, TrackableData] = dict()

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.actionConfigurate_SIHA.triggered.connect(self._actionConfigurate_SIHA)
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

    def add_to_watch_list(self, e: TrackableDataEnum):
        if e.value[0] in self.watcher_datas:
            return
        timer: QTimer = QTimer()
        interval: int = e.value[3]
        data: TrackableData = TrackableData()
        if interval != -1:
            timer.setInterval(interval)
            timer.timeout.connect(lambda: self.trigger_update_value(e))
            self.trigger_update_value(e) # fire once when adding to try to set data
            timer.start()
            data.timer_started = True

        data.timer = timer
        self.watcher_datas[e.value[0]] = data

        rowCount: int = self.ui.watch_list.rowCount()
        self.ui.watch_list.setRowCount(rowCount + 1)

        self.ui.watch_list.setItem(rowCount, 0, QTableWidgetItem(str(e.value[0])))
        self.ui.watch_list.setItem(rowCount, 1, QTableWidgetItem(e.value[1]))
        self.ui.watch_list.setItem(rowCount, 2, QTableWidgetItem(""))
        self.ui.watch_list.setItem(rowCount, 3, QTableWidgetItem("Unknown"))

        TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].setDisabled(True)

    def trigger_update_value(self, e: TrackableDataEnum):
        if self.uav_connection.connection_type is None:
            return
        length: int = self.ui.watch_list.rowCount()
        for i in range(length):
            if self.ui.watch_list.item(i, 0) == e.value[0]:
                self.ui.watch_list.setItem(i, 3, QStandardItem(e.value[2]()))
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
                    qWarning("You broke something, wrong target row found when trying to remove. Skipping")
                    continue
                data_id = int(self.ui.watch_list.item(target_row, 0).text())

                if data_id in self.watcher_datas:
                    data = self.watcher_datas.pop(data_id)
                    if data.timer_started:
                        data.timer.stop()
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
        self.server_connection_dialog.ui.connect.clicked.connect(lambda button: self._server_connect(button, self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect(button, self.server_connection_dialog))
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    def _actionConfigurate_SIHA(self):
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
            if self.is_ip_address_valid(self.uav_connection_dialog.ui.ip_address.text()):
                self.uav_connection_dialog.ui.invalid_input_error_label.hide()
            else:
                self.uav_connection_dialog.ui.invalid_input_error_label.show()
        else:
            pass # TODO: Serial connection
        qInfo("UAV Connect Button Not Implemented yet") # TODO: UAV Connect Button

        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type
        if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
            self.uav_connection.serial_band = self.uav_connection_dialog.ui.serial_band.text()
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()
        self.ui.device_connection_warning.hide()

    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        qInfo("UAV Disconnect Button Not Implemented yet") # TODO: UAV Disconnect Button
        self.uav_connection.connection_type = None

    def _server_connect(self, button: QPushButton, dialog: ServerConnectionInterface):
        qInfo("Server Connect Button Not Implemented yet") # TODO: Server Connect Button

    def _server_disconnect(self, button: QPushButton, dialog: ServerConnectionInterface):
        qInfo("Server Disconnect Button Not Implemented yet") # TODO: Server Disconnect Button

    @staticmethod
    def is_ip_address_valid(ip_address: str) -> bool:
        if ip_address is not None:
            ip_address = ip_address.strip()
            ip_with_port: str = ip_address
            split: list[str] = ip_with_port.split(':')
            ip: str = split[0]
            if len(split) > 1:
                port: int = int(split[1])
                if port < 0 or port > 65535:
                    return False
            ip_array: list[str] = ip.split('.')
            if len(ip_array) != 4:
                return False
            for e in ip_array:
                e: int = int(e)
                if e < 0 or e > 255:
                    return False
            return True
        return False

