from PySide6.QtWidgets import QComboBox
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog

from ui_files_python.connect_to_server import Ui_ServerConfig

class ServerConnectionInterface(QDialog):
    _ip_regex: QRegularExpression = QRegularExpression("[\\d.]+")
    _port_regex: QRegularExpression = QRegularExpression("[\\d]+")
    ui: Ui_ServerConfig

    def __init__(self):
        super().__init__()
        self.ui = Ui_ServerConfig()
        self.ui.setupUi(self)
        self.ui.invalid_input_error_label.hide()

        self.ui.server_ip.setValidator(QRegularExpressionValidator(self._ip_regex))
        self.ui.server_port.setValidator(QRegularExpressionValidator(self._port_regex))

