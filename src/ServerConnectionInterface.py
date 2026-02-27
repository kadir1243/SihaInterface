from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget

from ui_files_python.connect_to_server import Ui_ServerConfig


class ServerConnectionInterface(QDialog):
    _port_regex: QRegularExpression = QRegularExpression("[\\d]+")
    ui: Ui_ServerConfig

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui = Ui_ServerConfig()
        self.ui.setupUi(self)
        self.ui.invalid_input_error_label.hide()

        self.ui.server_port_input.setValidator(QRegularExpressionValidator(self._port_regex))

