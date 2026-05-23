# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'advanced_reposition.ui'
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
from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QDialogButtonBox,
    QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QSizePolicy, QVBoxLayout, QWidget)

class Ui_AdvancedRepositionDialog(object):
    def setupUi(self, AdvancedRepositionDialog):
        if not AdvancedRepositionDialog.objectName():
            AdvancedRepositionDialog.setObjectName(u"AdvancedRepositionDialog")
        AdvancedRepositionDialog.resize(388, 288)
        self.gridLayout = QGridLayout(AdvancedRepositionDialog)
        self.gridLayout.setObjectName(u"gridLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.latitude_layout = QHBoxLayout()
        self.latitude_layout.setObjectName(u"latitude_layout")
        self.latitude_label = QLabel(AdvancedRepositionDialog)
        self.latitude_label.setObjectName(u"latitude_label")

        self.latitude_layout.addWidget(self.latitude_label)

        self.latitude = QLineEdit(AdvancedRepositionDialog)
        self.latitude.setObjectName(u"latitude")

        self.latitude_layout.addWidget(self.latitude)


        self.verticalLayout.addLayout(self.latitude_layout)

        self.longitude_layout = QHBoxLayout()
        self.longitude_layout.setObjectName(u"longitude_layout")
        self.longitude_label = QLabel(AdvancedRepositionDialog)
        self.longitude_label.setObjectName(u"longitude_label")

        self.longitude_layout.addWidget(self.longitude_label)

        self.longitude = QLineEdit(AdvancedRepositionDialog)
        self.longitude.setObjectName(u"longitude")

        self.longitude_layout.addWidget(self.longitude)


        self.verticalLayout.addLayout(self.longitude_layout)

        self.altitude_layout = QHBoxLayout()
        self.altitude_layout.setObjectName(u"altitude_layout")
        self.altitude_label = QLabel(AdvancedRepositionDialog)
        self.altitude_label.setObjectName(u"altitude_label")

        self.altitude_layout.addWidget(self.altitude_label)

        self.altitude = QLineEdit(AdvancedRepositionDialog)
        self.altitude.setObjectName(u"altitude")

        self.altitude_layout.addWidget(self.altitude)


        self.verticalLayout.addLayout(self.altitude_layout)

        self.loiter_radius_layout = QHBoxLayout()
        self.loiter_radius_layout.setObjectName(u"loiter_radius_layout")
        self.loiter_radius_label = QLabel(AdvancedRepositionDialog)
        self.loiter_radius_label.setObjectName(u"loiter_radius_label")

        self.loiter_radius_layout.addWidget(self.loiter_radius_label)

        self.loiter_radius = QLineEdit(AdvancedRepositionDialog)
        self.loiter_radius.setObjectName(u"loiter_radius")

        self.loiter_radius_layout.addWidget(self.loiter_radius)


        self.verticalLayout.addLayout(self.loiter_radius_layout)

        self.speed_layout = QHBoxLayout()
        self.speed_layout.setObjectName(u"speed_layout")
        self.speed_label = QLabel(AdvancedRepositionDialog)
        self.speed_label.setObjectName(u"speed_label")

        self.speed_layout.addWidget(self.speed_label)

        self.speed = QLineEdit(AdvancedRepositionDialog)
        self.speed.setObjectName(u"speed")

        self.speed_layout.addWidget(self.speed)


        self.verticalLayout.addLayout(self.speed_layout)

        self.yaw_layout = QHBoxLayout()
        self.yaw_layout.setObjectName(u"yaw_layout")
        self.yaw_label = QLabel(AdvancedRepositionDialog)
        self.yaw_label.setObjectName(u"yaw_label")

        self.yaw_layout.addWidget(self.yaw_label)

        self.yaw = QLineEdit(AdvancedRepositionDialog)
        self.yaw.setObjectName(u"yaw")

        self.yaw_layout.addWidget(self.yaw)


        self.verticalLayout.addLayout(self.yaw_layout)


        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.buttons = QDialogButtonBox(AdvancedRepositionDialog)
        self.buttons.setObjectName(u"buttons")
        self.buttons.setOrientation(Qt.Orientation.Horizontal)
        self.buttons.setStandardButtons(QDialogButtonBox.StandardButton.Cancel|QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.RestoreDefaults|QDialogButtonBox.StandardButton.Save)

        self.gridLayout.addWidget(self.buttons, 1, 0, 1, 1)

#if QT_CONFIG(shortcut)
        self.latitude_label.setBuddy(self.latitude)
        self.longitude_label.setBuddy(self.longitude)
        self.altitude_label.setBuddy(self.altitude)
        self.loiter_radius_label.setBuddy(self.loiter_radius)
        self.speed_label.setBuddy(self.speed)
        self.yaw_label.setBuddy(self.yaw)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(AdvancedRepositionDialog)

        QMetaObject.connectSlotsByName(AdvancedRepositionDialog)
    # setupUi

    def retranslateUi(self, AdvancedRepositionDialog):
        AdvancedRepositionDialog.setWindowTitle(QCoreApplication.translate("AdvancedRepositionDialog", u"Advanced Reposition", None))
        self.latitude_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Latitude", None))
        self.latitude.setText("")
        self.longitude_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Longitude", None))
        self.longitude.setText("")
        self.altitude_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Altitude", None))
        self.loiter_radius_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Loiter Radius", None))
        self.speed_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Speed", None))
        self.speed.setText("")
        self.yaw_label.setText(QCoreApplication.translate("AdvancedRepositionDialog", u"Yaw", None))
        self.yaw.setText("")
    # retranslateUi

