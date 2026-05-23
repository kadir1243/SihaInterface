# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'set_geofence.ui'
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
from PySide6.QtWidgets import (QApplication, QDialog, QGridLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QSizePolicy,
    QWidget)

class Ui_SetGeofenceDialog(object):
    def setupUi(self, SetGeofenceDialog):
        if not SetGeofenceDialog.objectName():
            SetGeofenceDialog.setObjectName(u"SetGeofenceDialog")
        SetGeofenceDialog.resize(388, 269)
        self.gridLayout = QGridLayout(SetGeofenceDialog)
        self.gridLayout.setObjectName(u"gridLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(SetGeofenceDialog)
        self.label.setObjectName(u"label")

        self.horizontalLayout.addWidget(self.label)

        self.gc1 = QLineEdit(SetGeofenceDialog)
        self.gc1.setObjectName(u"gc1")

        self.horizontalLayout.addWidget(self.gc1)


        self.gridLayout.addLayout(self.horizontalLayout, 0, 0, 1, 1)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(SetGeofenceDialog)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_2.addWidget(self.label_2)

        self.gc2 = QLineEdit(SetGeofenceDialog)
        self.gc2.setObjectName(u"gc2")

        self.horizontalLayout_2.addWidget(self.gc2)


        self.gridLayout.addLayout(self.horizontalLayout_2, 1, 0, 1, 1)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_3 = QLabel(SetGeofenceDialog)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_3.addWidget(self.label_3)

        self.gc3 = QLineEdit(SetGeofenceDialog)
        self.gc3.setObjectName(u"gc3")

        self.horizontalLayout_3.addWidget(self.gc3)


        self.gridLayout.addLayout(self.horizontalLayout_3, 2, 0, 1, 1)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_4 = QLabel(SetGeofenceDialog)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_4.addWidget(self.label_4)

        self.gc4 = QLineEdit(SetGeofenceDialog)
        self.gc4.setObjectName(u"gc4")

        self.horizontalLayout_4.addWidget(self.gc4)


        self.gridLayout.addLayout(self.horizontalLayout_4, 3, 0, 1, 1)

        self.save = QPushButton(SetGeofenceDialog)
        self.save.setObjectName(u"save")

        self.gridLayout.addWidget(self.save, 4, 0, 1, 1)

#if QT_CONFIG(shortcut)
        self.label.setBuddy(self.gc1)
        self.label_2.setBuddy(self.gc2)
        self.label_3.setBuddy(self.gc3)
        self.label_4.setBuddy(self.gc4)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(SetGeofenceDialog)

        QMetaObject.connectSlotsByName(SetGeofenceDialog)
    # setupUi

    def retranslateUi(self, SetGeofenceDialog):
        SetGeofenceDialog.setWindowTitle(QCoreApplication.translate("SetGeofenceDialog", u"Set Geofence", None))
        self.label.setText(QCoreApplication.translate("SetGeofenceDialog", u"Geofence Coord 1", None))
        self.gc1.setText("")
        self.label_2.setText(QCoreApplication.translate("SetGeofenceDialog", u"Geofence Coord 2", None))
        self.gc2.setText("")
        self.label_3.setText(QCoreApplication.translate("SetGeofenceDialog", u"Geofence Coord 3", None))
        self.label_4.setText(QCoreApplication.translate("SetGeofenceDialog", u"Geofence Coord 4", None))
        self.save.setText(QCoreApplication.translate("SetGeofenceDialog", u"Save", None))
    # retranslateUi

