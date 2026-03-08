import sys

from PySide6.QtQml import QQmlDebuggingEnabler
from PySide6.QtWidgets import QApplication
from src.MainInterface import MainWindow


def start_ui():
    app = QApplication(sys.argv)
    QQmlDebuggingEnabler.enableDebugging(True)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    start_ui()
