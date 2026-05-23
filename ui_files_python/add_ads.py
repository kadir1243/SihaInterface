# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'add_ads.ui'
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
    QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QSizePolicy, QSplitter, QVBoxLayout, QWidget)

class Ui_AddADS(object):
    def setupUi(self, AddADS):
        if not AddADS.objectName():
            AddADS.setObjectName(u"AddADS")
        AddADS.resize(388, 269)
        self.verticalLayout_2 = QVBoxLayout(AddADS)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.splitter = QSplitter(AddADS)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Orientation.Horizontal)
        self.layoutWidget = QWidget(self.splitter)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.verticalLayout = QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(self.layoutWidget)
        self.label.setObjectName(u"label")

        self.horizontalLayout.addWidget(self.label)

        self.ads_latitude = QLineEdit(self.layoutWidget)
        self.ads_latitude.setObjectName(u"ads_latitude")

        self.horizontalLayout.addWidget(self.ads_latitude)


        self.verticalLayout.addLayout(self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(self.layoutWidget)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_2.addWidget(self.label_2)

        self.ads_longitude = QLineEdit(self.layoutWidget)
        self.ads_longitude.setObjectName(u"ads_longitude")

        self.horizontalLayout_2.addWidget(self.ads_longitude)


        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_3 = QLabel(self.layoutWidget)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_3.addWidget(self.label_3)

        self.ads_radius = QLineEdit(self.layoutWidget)
        self.ads_radius.setObjectName(u"ads_radius")

        self.horizontalLayout_3.addWidget(self.ads_radius)


        self.verticalLayout.addLayout(self.horizontalLayout_3)

        self.add_new = QPushButton(self.layoutWidget)
        self.add_new.setObjectName(u"add_new")

        self.verticalLayout.addWidget(self.add_new)

        self.splitter.addWidget(self.layoutWidget)

        self.verticalLayout_2.addWidget(self.splitter)

        self.buttons = QDialogButtonBox(AddADS)
        self.buttons.setObjectName(u"buttons")
        self.buttons.setOrientation(Qt.Orientation.Horizontal)
        self.buttons.setStandardButtons(QDialogButtonBox.StandardButton.Cancel|QDialogButtonBox.StandardButton.Ok)

        self.verticalLayout_2.addWidget(self.buttons)

#if QT_CONFIG(shortcut)
        self.label.setBuddy(self.ads_latitude)
        self.label_2.setBuddy(self.ads_longitude)
        self.label_3.setBuddy(self.ads_radius)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(AddADS)
        self.buttons.accepted.connect(AddADS.accept)
        self.buttons.rejected.connect(AddADS.reject)

        QMetaObject.connectSlotsByName(AddADS)
    # setupUi

    def retranslateUi(self, AddADS):
        AddADS.setWindowTitle(QCoreApplication.translate("AddADS", u"Manage Air Defence Systems", None))
        self.label.setText(QCoreApplication.translate("AddADS", u"ADS Latitude", None))
        self.ads_latitude.setText("")
        self.label_2.setText(QCoreApplication.translate("AddADS", u"ADS Longitude", None))
        self.ads_longitude.setText("")
        self.label_3.setText(QCoreApplication.translate("AddADS", u"ADS Radius", None))
        self.add_new.setText(QCoreApplication.translate("AddADS", u"Add New ADS", None))
    # retranslateUi

