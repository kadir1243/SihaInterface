import sys

from PySide6.QtCore import QCoreApplication
from PySide6.QtQml import QQmlDebuggingEnabler
from PySide6.QtWidgets import QApplication

from src.MainInterface import MainWindow


def start_ui():
    QCoreApplication.setApplicationName("ARES UAV Interface")
    QCoreApplication.setApplicationVersion("0.1.0")
    QCoreApplication.setOrganizationName("ARES")
    app = QApplication(sys.argv)
    QQmlDebuggingEnabler.enableDebugging(True)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    start_ui()
