import sys

from PySide6.QtWidgets import QApplication
from src.MainInterface import MainWindow


def start_ui():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    start_ui()
