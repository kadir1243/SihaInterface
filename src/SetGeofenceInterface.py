from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget

from ui_files_python.set_geofence import Ui_SetGeofenceDialog


class SetGeofenceInterface(QDialog):
    is_float_regex: QRegularExpression = QRegularExpression("[\\d. -]+")
    ui: Ui_SetGeofenceDialog

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui = Ui_SetGeofenceDialog()
        self.ui.setupUi(self)

        self.ui.gc1.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.gc2.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.gc3.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.gc4.setValidator(QRegularExpressionValidator(self.is_float_regex))

