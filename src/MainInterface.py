from enum import Enum

from PySide6.QtCore import QAbstractListModel, QObject, Qt, QTimer, QModelIndex
from PySide6.QtGui import QAction, QStandardItemModel, QStandardItem
from PySide6.QtWidgets import QMainWindow, QPushButton, QListWidgetItem, QToolButton

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

class TrackableDataEnum(Enum):
    SPEED = (0, "Speed", TrackableDataUpdate.update_speed)
    VELOCITY = (1, "Velocity", TrackableDataUpdate.update_velocity)
    ALTITUDE = (2, "Altitude", TrackableDataUpdate.update_altitude)
    YAW = (3, "Yaw", TrackableDataUpdate.update_yaw)
    PITCH = (4, "Pitch", TrackableDataUpdate.update_pitch)

    @staticmethod
    def list():
        return list(map(lambda c: c, TrackableDataEnum))

class UavConnection:
    connection_type: ConnectionType | None = None
    serial_port: int | None = None
    serial_band: int | None = None
    ip: str | None = None

class StringListModel(QAbstractListModel):
    values: dict[TrackableDataEnum, str] = dict()

    def rowCount(self, /, parent=...):
        return super().rowCount(parent)

    def data(self, index, /, role=...):
        return super().data(index, role)

    def headerData(self, section, orientation, /, role=...):
        return super().headerData(section, orientation, role)

    def __init__(self, parent: QObject | None = None):
        QAbstractListModel.__init__(self, parent)

class TrackableData:
    timer: QTimer
    index: int

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    watcher_datas: dict[TrackableDataEnum, TrackableData] = dict()
    watcher_items: QStandardItemModel

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.actionConfigurate_SIHA.triggered.connect(self._actionConfigurate_SIHA)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)
        self.watcher_items = QStandardItemModel(self)
        self.ui.watch_list.setModel(self.watcher_items)
        for e in TrackableDataEnum.list():
            self.__add_to_watch_list_signal(e)

        self.ui.remove_from_watch.clicked.connect(self.__remove_from_watch_signal)
        self.ui.watch_list.hideColumn(1) # id column

    def __add_to_watch_list_signal(self, e: TrackableDataEnum):
        timer: QTimer = QTimer()
        timer.timeout.connect(lambda: self.__add_to_watch_list_signal(e))

        data: TrackableData = TrackableData()
        data.timer = timer
        self.watcher_datas.update({e: data})
        items: list[QStandardItem] = [QStandardItem(e.value[0]), QStandardItem(e.value[1]), QStandardItem(), QStandardItem("")]

        self.watcher_items.appendRow(items)

    def __timer_update(self, e: TrackableDataEnum):
        length: int = self.watcher_items.rowCount(QModelIndex())
        for i in range(length):
            item: QStandardItem = self.watcher_items.item(i)
            if item.takeColumn(0) == e.value[0]:
                item.removeColumn(3)
                item.insertColumn(3, [QStandardItem(e.value[2]())])

    def _add_to_watch(self):
        pass

    def __remove_from_watch_signal(self):
        indexes: list[QModelIndex] = self.ui.watch_list.selectedIndexes()
        if indexes is not None:
            for index in indexes:
                self.watcher_items.removeRow(index.row(), index.parent())
        self.ui.watch_list.setModel(self.watcher_items)

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
        print("UAV Connect Button Not Implemented yet") # TODO: UAV Connect Button

        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type
        if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
            self.uav_connection.serial_band = self.uav_connection_dialog.ui.serial_band.text()
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()
        self.ui.device_connection_warning.hide()

    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        print("UAV Disconnect Button Not Implemented yet") # TODO: UAV Disconnect Button
        self.uav_connection.connection_type = None

    def _server_connect(self, button: QPushButton, dialog: ServerConnectionInterface):
        print("Server Connect Button Not Implemented yet") # TODO: Server Connect Button

    def _server_disconnect(self, button: QPushButton, dialog: ServerConnectionInterface):
        print("Server Disconnect Button Not Implemented yet") # TODO: Server Disconnect Button

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

