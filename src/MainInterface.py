from enum import Enum

from PySide6.QtCore import QObject, QRegularExpression
from PySide6.QtWidgets import QMainWindow, QPushButton

from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface
from src.ServerConnectionInterface import ServerConnectionInterface
from ui_files_python.siha_interface import Ui_MainWindow

class ConnectionType(Enum):
    SERIAL = 1
    TCP = 2
    UDP = 4

class UavConnection:
    connection_type: ConnectionType
    serial_band: int
    ip: str

    def __init__(self, connection_type: ConnectionType):
        self.connection_type = connection_type

class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection

    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.actionConfigurate_SIHA.triggered.connect(self._actionConfigurate_SIHA)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)

    def _actionConfigurateServer(self):
        dialog: ServerConnectionInterface = ServerConnectionInterface()
        dialog.show()
        dialog.ui.connect.clicked.connect(lambda button: self._server_connect(button, dialog))
        dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect(button, dialog))

    def _actionConfigurate_SIHA(self):
        dialog: FightingUAVConnectionInterface = FightingUAVConnectionInterface()
        dialog.show()
        dialog.ui.connect.clicked.connect(lambda button: self._uav_connect(button, dialog))
        dialog.ui.disconnect.clicked.connect(lambda button: self._uav_disconnect(button, dialog))

    def _uav_connect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        if self.is_ip_address_valid(dialog.ui.ip_address.text()):
            dialog.ui.invalid_input_error_label.hide()
        else:
            dialog.ui.invalid_input_error_label.show()
        print("UAV Connect Button Not Implemented yet")

    def _uav_disconnect(self, button: QPushButton, dialog: FightingUAVConnectionInterface):
        print("UAV Disconnect Button Not Implemented yet")

    def _server_connect(self, button: QPushButton, dialog: ServerConnectionInterface):
        print("Server Connect Button Not Implemented yet")

    def _server_disconnect(self, button: QPushButton, dialog: ServerConnectionInterface):
        print("Server Disconnect Button Not Implemented yet")

    @staticmethod
    def is_ip_address_valid(ip_address: str) -> bool:
        ip_address = ip_address.strip()
        if ip_address is not None:
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

