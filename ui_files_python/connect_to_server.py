# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'connect_to_server.ui'
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
from PySide6.QtWidgets import (QApplication, QDialog, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget)

class Ui_ServerConfig(object):
    def setupUi(self, ServerConfig):
        if not ServerConfig.objectName():
            ServerConfig.setObjectName(u"ServerConfig")
        ServerConfig.resize(400, 311)
        self.verticalLayout_3 = QVBoxLayout(ServerConfig)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.server_ip_layout = QHBoxLayout()
        self.server_ip_layout.setObjectName(u"server_ip_layout")
        self.server_ip_label = QLabel(ServerConfig)
        self.server_ip_label.setObjectName(u"server_ip_label")

        self.server_ip_layout.addWidget(self.server_ip_label)

        self.server_ip_input = QLineEdit(ServerConfig)
        self.server_ip_input.setObjectName(u"server_ip_input")
        self.server_ip_input.setPlaceholderText(u"127.0.0.1")

        self.server_ip_layout.addWidget(self.server_ip_input)


        self.verticalLayout_3.addLayout(self.server_ip_layout)

        self.server_port_layout = QHBoxLayout()
        self.server_port_layout.setObjectName(u"server_port_layout")
        self.server_port_label = QLabel(ServerConfig)
        self.server_port_label.setObjectName(u"server_port_label")

        self.server_port_layout.addWidget(self.server_port_label)

        self.server_port_input = QLineEdit(ServerConfig)
        self.server_port_input.setObjectName(u"server_port_input")
        self.server_port_input.setMaxLength(5)
        self.server_port_input.setPlaceholderText(u"5000")

        self.server_port_layout.addWidget(self.server_port_input)


        self.verticalLayout_3.addLayout(self.server_port_layout)

        self.server_login_username_layout = QHBoxLayout()
        self.server_login_username_layout.setObjectName(u"server_login_username_layout")
        self.server_login_username_label = QLabel(ServerConfig)
        self.server_login_username_label.setObjectName(u"server_login_username_label")

        self.server_login_username_layout.addWidget(self.server_login_username_label)

        self.server_login_username_input = QLineEdit(ServerConfig)
        self.server_login_username_input.setObjectName(u"server_login_username_input")
        self.server_login_username_input.setPlaceholderText(u"ares")

        self.server_login_username_layout.addWidget(self.server_login_username_input)


        self.verticalLayout_3.addLayout(self.server_login_username_layout)

        self.server_login_password_layout = QHBoxLayout()
        self.server_login_password_layout.setObjectName(u"server_login_password_layout")
        self.server_login_password_label = QLabel(ServerConfig)
        self.server_login_password_label.setObjectName(u"server_login_password_label")

        self.server_login_password_layout.addWidget(self.server_login_password_label)

        self.server_login_password_input = QLineEdit(ServerConfig)
        self.server_login_password_input.setObjectName(u"server_login_password_input")
        self.server_login_password_input.setEchoMode(QLineEdit.EchoMode.Password)

        self.server_login_password_layout.addWidget(self.server_login_password_input)


        self.verticalLayout_3.addLayout(self.server_login_password_layout)

        self.information_layout = QVBoxLayout()
        self.information_layout.setObjectName(u"information_layout")
        self.invalid_input_error_label = QLabel(ServerConfig)
        self.invalid_input_error_label.setObjectName(u"invalid_input_error_label")
        self.invalid_input_error_label.setEnabled(False)
        self.invalid_input_error_label.setTextFormat(Qt.TextFormat.RichText)
        self.invalid_input_error_label.setAlignment(Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)
        self.invalid_input_error_label.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.information_layout.addWidget(self.invalid_input_error_label)

        self.server_connection_text = QLabel(ServerConfig)
        self.server_connection_text.setObjectName(u"server_connection_text")
        self.server_connection_text.setAlignment(Qt.AlignmentFlag.AlignBottom|Qt.AlignmentFlag.AlignHCenter)
        self.server_connection_text.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)

        self.information_layout.addWidget(self.server_connection_text)


        self.verticalLayout_3.addLayout(self.information_layout)

        self.button_layout = QVBoxLayout()
        self.button_layout.setObjectName(u"button_layout")
        self.connect = QPushButton(ServerConfig)
        self.connect.setObjectName(u"connect")

        self.button_layout.addWidget(self.connect)

        self.disconnect = QPushButton(ServerConfig)
        self.disconnect.setObjectName(u"disconnect")

        self.button_layout.addWidget(self.disconnect)


        self.verticalLayout_3.addLayout(self.button_layout)

#if QT_CONFIG(shortcut)
        self.server_ip_label.setBuddy(self.server_ip_input)
        self.server_port_label.setBuddy(self.server_port_input)
        self.server_login_username_label.setBuddy(self.server_login_username_input)
        self.server_login_password_label.setBuddy(self.server_login_password_input)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(ServerConfig)

        QMetaObject.connectSlotsByName(ServerConfig)
    # setupUi

    def retranslateUi(self, ServerConfig):
        ServerConfig.setWindowTitle(QCoreApplication.translate("ServerConfig", u"Server Configuration", None))
        self.server_ip_label.setText(QCoreApplication.translate("ServerConfig", u"Server IP", None))
#if QT_CONFIG(tooltip)
        self.server_ip_input.setToolTip(QCoreApplication.translate("ServerConfig", u"The IP of the target Server", None))
#endif // QT_CONFIG(tooltip)
        self.server_port_label.setText(QCoreApplication.translate("ServerConfig", u"Server Port", None))
#if QT_CONFIG(tooltip)
        self.server_port_input.setToolTip(QCoreApplication.translate("ServerConfig", u"The port of the target Server", None))
#endif // QT_CONFIG(tooltip)
        self.server_login_username_label.setText(QCoreApplication.translate("ServerConfig", u"Username", None))
#if QT_CONFIG(tooltip)
        self.server_login_username_input.setToolTip(QCoreApplication.translate("ServerConfig", u"Username to be used to login server", None))
#endif // QT_CONFIG(tooltip)
        self.server_login_password_label.setText(QCoreApplication.translate("ServerConfig", u"Password", None))
#if QT_CONFIG(tooltip)
        self.server_login_password_input.setToolTip(QCoreApplication.translate("ServerConfig", u"Password to be used to login server", None))
#endif // QT_CONFIG(tooltip)
        self.invalid_input_error_label.setText(QCoreApplication.translate("ServerConfig", u"<font color='red'>Invalid Input Specified<color>", None))
        self.server_connection_text.setText(QCoreApplication.translate("ServerConfig", u"Server Not Connected :(", None))
        self.connect.setText(QCoreApplication.translate("ServerConfig", u"Connect", None))
        self.disconnect.setText(QCoreApplication.translate("ServerConfig", u"Disconnect", None))
    # retranslateUi

