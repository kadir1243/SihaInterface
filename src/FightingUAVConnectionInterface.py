from PySide6.QtWidgets import QComboBox
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog

from ui_files_python.uav_connection import Ui_UAVConnection



class FightingUAVConnectionInterface(QDialog):
    _ip_with_port_regex: QRegularExpression = QRegularExpression("[\\d.:]+")
    ui = Ui_UAVConnection

    def __init__(self):
        super().__init__()
        self.ui = Ui_UAVConnection()
        self.ui.setupUi(self)
        self.ui.invalid_input_error_label.hide()

        self.ui.ip_address.setValidator(QRegularExpressionValidator(self._ip_with_port_regex))
        self.ui.connection_type.currentIndexChanged.connect(self.connection_type_changed)
        self.connection_type_changed(0) # Trigger once to make it correctly set in start

    def add_invalid_input_specified_label(self):
        self.ui.invalid_input_error_label.show()

    def hide_invalid_input_specified_label(self):
        self.ui.invalid_input_error_label.hide()

    def connection_type_changed(self, index: int):
        if self.ui.connection_type.itemText(index) == "TCP" or self.ui.connection_type.itemText(index) == "UDP":
            self.ui.ip_address.setEnabled(True)
            self.ui.serial_band.setEnabled(False)
        else:
            self.ui.ip_address.setEnabled(False)
            self.ui.serial_band.setEnabled(True)

