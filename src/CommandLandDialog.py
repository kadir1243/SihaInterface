from PySide6.QtCore import QRegularExpression, qDebug
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget
from pymavlink.dialects.v20.all import MAV_CMD_NAV_LAND
from pymavlink.mavutil import mavfile

from ui_files_python.command_land import Ui_CommandLandDialog


class CommandLandDialog(QDialog):
    is_float_regex: QRegularExpression = QRegularExpression("[\\d.-]+")
    ui: Ui_CommandLandDialog
    mavlink_connection: mavfile

    def __init__(self, parent: QWidget, mavlink_connection: mavfile):
        QDialog.__init__(self, parent)
        self.ui = Ui_CommandLandDialog()
        self.ui.setupUi(self)
        self.mavlink_connection = mavlink_connection

        self.ui.latitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.longitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.altitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.yaw.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.abort_altitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.accepted.connect(self.send_command)

    @staticmethod
    def return_non_null(a: str, b: float) -> float:
        return b if a is None or len(a) == 0 else float(a)

    def send_command(self):
        latitude: float = self.return_non_null(self.ui.latitude.text(), float('nan'))
        longitude: float = self.return_non_null(self.ui.longitude.text(), float('nan'))
        altitude: float = self.return_non_null(self.ui.altitude.text(), float('nan'))
        yaw: float = self.return_non_null(self.ui.yaw.text(), float('nan'))

        qDebug("Sending Land command with latitude %s, longitude %s, altitude %s, yaw %s, land mode %s, abort altitude %s" % (
            latitude, longitude, altitude, yaw, self.ui.land_mode.currentText(), float(self.ui.abort_altitude.text())
        ))
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_CMD_NAV_LAND,
            0, # confirm param
            float(self.ui.abort_altitude.text()),
            self.ui.land_mode.currentIndex(),
            0, # empty param
            yaw,
            latitude,
            longitude,
            altitude
        )
