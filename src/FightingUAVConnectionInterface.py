from enum import Enum

from PySide6.QtWidgets import QComboBox, QWidget
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog

from ui_files_python.uav_connection import Ui_UAVConnection

class ConnectionType(Enum):
    SERIAL = 1
    TCP = 2
    UDP = 4


class FightingUAVConnectionInterface(QDialog):
    _ip_with_port_regex: QRegularExpression = QRegularExpression("[\\d.:]+")
    _serial_baud_regex: QRegularExpression = QRegularExpression("[\\d]+")
    ui = Ui_UAVConnection = Ui_UAVConnection()
    connection_type: ConnectionType

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui.setupUi(self)
        self.ui.invalid_input_error_label.hide()

        self.ui.ip_address.setValidator(QRegularExpressionValidator(self._ip_with_port_regex))
        self.ui.serial_baud.setValidator(QRegularExpressionValidator(self._serial_baud_regex))
        self.ui.connection_type.currentIndexChanged.connect(self.connection_type_changed)
        self.connection_type_changed(self.ui.connection_type.currentIndex()) # Trigger once to make it correctly set in start

    def add_invalid_input_specified_label(self):
        self.ui.invalid_input_error_label.show()

    def hide_invalid_input_specified_label(self):
        self.ui.invalid_input_error_label.hide()

    def connection_type_changed(self, index: int):
        isTCP: bool = index == 0
        isUDP: bool = index == 1
        if isTCP or isUDP:
            self.ui.ip_address.setEnabled(True)
            self.ui.serial_baud.setEnabled(False)
            if isTCP:
                self.connection_type = ConnectionType.TCP
            else:
                self.connection_type = ConnectionType.UDP
        else:
            self.connection_type = ConnectionType.SERIAL
            self.ui.ip_address.setEnabled(False)
            self.ui.serial_baud.setEnabled(True)

