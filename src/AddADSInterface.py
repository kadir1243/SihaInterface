from PySide6.QtWidgets import QDialog
from PySide6.QtWidgets import QWidget

from ui_files_python.add_ads import Ui_AddADS


class AddADSInterface(QDialog):
    ui: Ui_AddADS

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.ui = Ui_AddADS()
        self.ui.setupUi(self)

