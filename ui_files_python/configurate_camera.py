# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'configurate_camera.ui'
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

class Ui_CameraConfig(object):
    def setupUi(self, CameraConfig):
        if not CameraConfig.objectName():
            CameraConfig.setObjectName(u"CameraConfig")
        CameraConfig.resize(308, 258)
        self.verticalLayout = QVBoxLayout(CameraConfig)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.camera_server_protocol_layout = QHBoxLayout()
        self.camera_server_protocol_layout.setObjectName(u"camera_server_protocol_layout")
        self.camera_server_protocol = QLabel(CameraConfig)
        self.camera_server_protocol.setObjectName(u"camera_server_protocol")

        self.camera_server_protocol_layout.addWidget(self.camera_server_protocol)

        self.server_protocol_type = QComboBox(CameraConfig)
        self.server_protocol_type.addItem(u"Osman")
        self.server_protocol_type.addItem(u"Kadir")
        self.server_protocol_type.setObjectName(u"server_protocol_type")

        self.camera_server_protocol_layout.addWidget(self.server_protocol_type)


        self.verticalLayout.addLayout(self.camera_server_protocol_layout)

        self.server_ip_layout = QHBoxLayout()
        self.server_ip_layout.setObjectName(u"server_ip_layout")
        self.server_ip_label = QLabel(CameraConfig)
        self.server_ip_label.setObjectName(u"server_ip_label")

        self.server_ip_layout.addWidget(self.server_ip_label)

        self.server_ip_input = QLineEdit(CameraConfig)
        self.server_ip_input.setObjectName(u"server_ip_input")
        self.server_ip_input.setPlaceholderText(u"192.168.1.25")

        self.server_ip_layout.addWidget(self.server_ip_input)


        self.verticalLayout.addLayout(self.server_ip_layout)

        self.server_port_layout = QHBoxLayout()
        self.server_port_layout.setObjectName(u"server_port_layout")
        self.server_port_label = QLabel(CameraConfig)
        self.server_port_label.setObjectName(u"server_port_label")

        self.server_port_layout.addWidget(self.server_port_label)

        self.server_port_input = QLineEdit(CameraConfig)
        self.server_port_input.setObjectName(u"server_port_input")
        self.server_port_input.setMaxLength(5)
        self.server_port_input.setPlaceholderText(u"9999")

        self.server_port_layout.addWidget(self.server_port_input)


        self.verticalLayout.addLayout(self.server_port_layout)

        self.information_layout = QVBoxLayout()
        self.information_layout.setObjectName(u"information_layout")
        self.invalid_input_error_label = QLabel(CameraConfig)
        self.invalid_input_error_label.setObjectName(u"invalid_input_error_label")
        self.invalid_input_error_label.setEnabled(False)
        self.invalid_input_error_label.setTextFormat(Qt.TextFormat.RichText)
        self.invalid_input_error_label.setAlignment(Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)
        self.invalid_input_error_label.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.information_layout.addWidget(self.invalid_input_error_label)

        self.camera_connection_text = QLabel(CameraConfig)
        self.camera_connection_text.setObjectName(u"camera_connection_text")
        self.camera_connection_text.setAlignment(Qt.AlignmentFlag.AlignBottom|Qt.AlignmentFlag.AlignHCenter)
        self.camera_connection_text.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.information_layout.addWidget(self.camera_connection_text)


        self.verticalLayout.addLayout(self.information_layout)

        self.button_layout = QVBoxLayout()
        self.button_layout.setObjectName(u"button_layout")
        self.connect = QPushButton(CameraConfig)
        self.connect.setObjectName(u"connect")

        self.button_layout.addWidget(self.connect)

        self.disconnect = QPushButton(CameraConfig)
        self.disconnect.setObjectName(u"disconnect")

        self.button_layout.addWidget(self.disconnect)


        self.verticalLayout.addLayout(self.button_layout)

#if QT_CONFIG(shortcut)
        self.camera_server_protocol.setBuddy(self.server_protocol_type)
        self.server_ip_label.setBuddy(self.server_ip_input)
        self.server_port_label.setBuddy(self.server_port_input)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(CameraConfig)

        QMetaObject.connectSlotsByName(CameraConfig)
    # setupUi

    def retranslateUi(self, CameraConfig):
        CameraConfig.setWindowTitle(QCoreApplication.translate("CameraConfig", u"Camera Server Configuration", None))
        self.camera_server_protocol.setText(QCoreApplication.translate("CameraConfig", u"Camera Server Protocol", None))

#if QT_CONFIG(tooltip)
        self.server_protocol_type.setToolTip(QCoreApplication.translate("CameraConfig", u"The Protocol Of Camera Server", None))
#endif // QT_CONFIG(tooltip)
        self.server_ip_label.setText(QCoreApplication.translate("CameraConfig", u"Camera Server IP", None))
#if QT_CONFIG(tooltip)
        self.server_ip_input.setToolTip(QCoreApplication.translate("CameraConfig", u"The IP of the Camera Server", None))
#endif // QT_CONFIG(tooltip)
        self.server_port_label.setText(QCoreApplication.translate("CameraConfig", u"Camera Server Port", None))
#if QT_CONFIG(tooltip)
        self.server_port_input.setToolTip(QCoreApplication.translate("CameraConfig", u"The port of the Camera Server", None))
#endif // QT_CONFIG(tooltip)
        self.invalid_input_error_label.setText(QCoreApplication.translate("CameraConfig", u"<font color='red'>Invalid Input Specified<color>", None))
        self.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", u"Camera Not Connected :(", None))
        self.connect.setText(QCoreApplication.translate("CameraConfig", u"Connect", None))
        self.disconnect.setText(QCoreApplication.translate("CameraConfig", u"Disconnect", None))
    # retranslateUi

