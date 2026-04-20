from PySide6.QtCore import QRegularExpression, qDebug
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget
from pymavlink.dialects.v20.all import MAV_CMD_NAV_TAKEOFF
from pymavlink.mavutil import mavfile

from ui_files_python.command_takeoff import Ui_CommandTakeoffDialog


class CommandTakeoffDialog(QDialog):
    is_float_regex: QRegularExpression = QRegularExpression("[\\d.-]+")
    ui: Ui_CommandTakeoffDialog
    mavlink_connection: mavfile
    default_longitude: float
    default_latitude: float

    def __init__(self, parent: QWidget, mavlink_connection: mavfile, default_latitude: float, default_longitude: float):
        QDialog.__init__(self, parent)
        self.default_longitude = default_longitude
        self.default_latitude = default_latitude
        self.ui = Ui_CommandTakeoffDialog()
        self.ui.setupUi(self)
        self.mavlink_connection = mavlink_connection

        self.ui.latitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.longitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.altitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.yaw.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.pitch.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.accepted.connect(self.send_command)

    @staticmethod
    def return_non_null(a: str, b: float) -> float:
        return b if a is None or len(a) == 0 else float(a)

    def send_command(self):
        latitude: float = self.return_non_null(self.ui.latitude.text(), 0)
        longitude: float = self.return_non_null(self.ui.longitude.text(), 0)
        altitude: float = self.return_non_null(self.ui.altitude.text(), 100)
        yaw: float = self.return_non_null(self.ui.yaw.text(), 0)
        min_pitch: float = self.return_non_null(self.ui.pitch.text(), 0)
        ignore_horizontal_checks: bool = self.ui.ignore_horizontal_checks.isChecked()

        qDebug("Sending Takeoff command with latitude %s, longitude %s, altitude %s, yaw %s, min pitch %s, ignore horizontal checks %s" % (
            latitude, longitude, altitude, yaw, min_pitch, ignore_horizontal_checks
        ))
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_CMD_NAV_TAKEOFF,
            0, # confirm param
            min_pitch,
            0, # empty param
            1 if ignore_horizontal_checks else 0, # NAV_TAKEOFF_FLAGS_HORIZONTAL_POSITION_NOT_REQUIRED
            yaw,
            latitude,
            longitude,
            altitude
        )

