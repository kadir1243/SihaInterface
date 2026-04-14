from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog, QDialogButtonBox
from PySide6.QtWidgets import QWidget

from ui_files_python.advanced_reposition import Ui_AdvancedRepositionDialog

DEFAULT_ALTITUDE: int = 100
DEFAULT_LOITER_RADIUS: int = 10
DEFAULT_SPEED: float = -1
DEFAULT_YAW: float = float('nan')

class AdvancedRepositionDialog(QDialog):
    is_float_regex: QRegularExpression = QRegularExpression("[\\d.]+")
    is_int_regex: QRegularExpression = QRegularExpression("[\\d.]+")
    ui: Ui_AdvancedRepositionDialog

    def __init__(self, parent: QWidget):
        QDialog.__init__(self, parent)
        self.ui = Ui_AdvancedRepositionDialog()
        self.ui.setupUi(self)

        self.ui.longitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.latitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.yaw.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.altitude.setValidator(QRegularExpressionValidator(self.is_int_regex))
        self.ui.loiter_radius.setValidator(QRegularExpressionValidator(self.is_int_regex))
        self.ui.speed.setValidator(QRegularExpressionValidator(self.is_int_regex))

        self.ui.buttons.button(QDialogButtonBox.StandardButton.RestoreDefaults).clicked.connect(self.restore_defaults)

    def restore_defaults(self):
        self.ui.altitude.setText(str(DEFAULT_ALTITUDE))
        self.ui.loiter_radius.setText(str(DEFAULT_LOITER_RADIUS))
        self.ui.speed.setText(str(DEFAULT_SPEED))
        self.ui.yaw.setText(str(DEFAULT_YAW))
