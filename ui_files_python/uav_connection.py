# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'uav_connection.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDialog, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget)

class Ui_UAVConnection(object):
    def setupUi(self, UAVConnection):
        if not UAVConnection.objectName():
            UAVConnection.setObjectName(u"UAVConnection")
        UAVConnection.resize(400, 300)
        self.verticalLayout_4 = QVBoxLayout(UAVConnection)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.connection_type_layout = QHBoxLayout()
        self.connection_type_layout.setObjectName(u"connection_type_layout")
        self.connection_type_label = QLabel(UAVConnection)
        self.connection_type_label.setObjectName(u"connection_type_label")

        self.connection_type_layout.addWidget(self.connection_type_label)

        self.connection_type = QComboBox(UAVConnection)
        self.connection_type.addItem(u"TCP")
        self.connection_type.addItem(u"UDP")
        self.connection_type.setObjectName(u"connection_type")

        self.connection_type_layout.addWidget(self.connection_type)


        self.verticalLayout.addLayout(self.connection_type_layout)

        self.ip_address_layout = QHBoxLayout()
        self.ip_address_layout.setObjectName(u"ip_address_layout")
        self.ip_address_label = QLabel(UAVConnection)
        self.ip_address_label.setObjectName(u"ip_address_label")

        self.ip_address_layout.addWidget(self.ip_address_label)

        self.ip_address = QLineEdit(UAVConnection)
        self.ip_address.setObjectName(u"ip_address")
        self.ip_address.setPlaceholderText(u"127.0.0.1:14550")

        self.ip_address_layout.addWidget(self.ip_address)


        self.verticalLayout.addLayout(self.ip_address_layout)

        self.serial_baud_layout = QHBoxLayout()
        self.serial_baud_layout.setObjectName(u"serial_baud_layout")
        self.serial_baud_label = QLabel(UAVConnection)
        self.serial_baud_label.setObjectName(u"serial_baud_label")

        self.serial_baud_layout.addWidget(self.serial_baud_label, 0, Qt.AlignmentFlag.AlignHCenter)

        self.serial_baud = QLineEdit(UAVConnection)
        self.serial_baud.setObjectName(u"serial_baud")
        self.serial_baud.setPlaceholderText(u"115200")

        self.serial_baud_layout.addWidget(self.serial_baud)


        self.verticalLayout.addLayout(self.serial_baud_layout)


        self.verticalLayout_4.addLayout(self.verticalLayout)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.invalid_input_error_label = QLabel(UAVConnection)
        self.invalid_input_error_label.setObjectName(u"invalid_input_error_label")
        self.invalid_input_error_label.setEnabled(False)
        self.invalid_input_error_label.setTextFormat(Qt.TextFormat.RichText)
        self.invalid_input_error_label.setScaledContents(False)
        self.invalid_input_error_label.setAlignment(Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)
        self.invalid_input_error_label.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.verticalLayout_3.addWidget(self.invalid_input_error_label)

        self.device_connection_text = QLabel(UAVConnection)
        self.device_connection_text.setObjectName(u"device_connection_text")
        self.device_connection_text.setAlignment(Qt.AlignmentFlag.AlignBottom|Qt.AlignmentFlag.AlignHCenter)
        self.device_connection_text.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.verticalLayout_3.addWidget(self.device_connection_text)


        self.verticalLayout_4.addLayout(self.verticalLayout_3)

        self.buttons = QVBoxLayout()
        self.buttons.setObjectName(u"buttons")
        self.connect = QPushButton(UAVConnection)
        self.connect.setObjectName(u"connect")

        self.buttons.addWidget(self.connect)

        self.disconnect = QPushButton(UAVConnection)
        self.disconnect.setObjectName(u"disconnect")

        self.buttons.addWidget(self.disconnect)


        self.verticalLayout_4.addLayout(self.buttons)

#if QT_CONFIG(shortcut)
        self.connection_type_label.setBuddy(self.connection_type)
        self.ip_address_label.setBuddy(self.ip_address)
        self.serial_baud_label.setBuddy(self.serial_baud)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(UAVConnection)

        self.connection_type.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(UAVConnection)
    # setupUi

    def retranslateUi(self, UAVConnection):
        UAVConnection.setWindowTitle(QCoreApplication.translate("UAVConnection", u"UAV Connection", None))
        self.connection_type_label.setText(QCoreApplication.translate("UAVConnection", u"Connection Type:", None))

        self.ip_address_label.setText(QCoreApplication.translate("UAVConnection", u"IP Address:", None))
        self.serial_baud_label.setText(QCoreApplication.translate("UAVConnection", u"Serial Baud Rate:", None))
        self.invalid_input_error_label.setText(QCoreApplication.translate("UAVConnection", u"<font color='red'>Invalid Input Specified<color>", None))
        self.device_connection_text.setText(QCoreApplication.translate("UAVConnection", u"Device Not Connected :(", None))
        self.connect.setText(QCoreApplication.translate("UAVConnection", u"Connect", None))
        self.disconnect.setText(QCoreApplication.translate("UAVConnection", u"Disconnect", None))
    # retranslateUi

