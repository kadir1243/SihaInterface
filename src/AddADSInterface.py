from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget

from ui_files_python.add_ads import Ui_AddADS


class AddADSInterface(QDialog):
    is_float_regex: QRegularExpression = QRegularExpression("[\\d.-]+")
    ui: Ui_AddADS

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui = Ui_AddADS()
        self.ui.setupUi(self)

        self.ui.ads_radius.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.ads_latitude.setValidator(QRegularExpressionValidator(self.is_float_regex))
        self.ui.ads_longitude.setValidator(QRegularExpressionValidator(self.is_float_regex))

