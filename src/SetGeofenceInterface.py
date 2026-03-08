from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget

from ui_files_python.set_geofence import Ui_SetGeofenceDialog


class SetGeofenceInterface(QDialog):
    ui: Ui_SetGeofenceDialog

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui = Ui_SetGeofenceDialog()
        self.ui.setupUi(self)

