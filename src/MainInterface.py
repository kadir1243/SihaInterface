from enum import Enum

from PySide6.QtCore import QAbstractListModel, QObject, Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow, QPushButton, QListWidgetItem, QToolButton

from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.siha_interface import Ui_MainWindow

class TrackableDataEnum(Enum):
    SPEED = "speed"
    VELOCITY = "velocity"
    ALTITUDE = "altitude"
    YAW = "yaw"
    PITCH = "pitch"

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

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.actionConfigurate_SIHA.triggered.connect(self._actionConfigurate_SIHA)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)

    def __add_to_watch_list_signal(self, e: TrackableDataEnum):
        self.ui.watch_list.addItem(QListWidgetItem(e.value))

    def _add_to_watch(self):
        pass

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

