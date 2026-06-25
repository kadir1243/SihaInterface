# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'input_selector.ui'
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
    QFormLayout, QHBoxLayout, QLabel, QSizePolicy,
    QToolButton, QWidget)

class Ui_KeybindingConfig(object):
    def setupUi(self, KeybindingConfig):
        if not KeybindingConfig.objectName():
            KeybindingConfig.setObjectName(u"KeybindingConfig")
        KeybindingConfig.resize(331, 298)
        self.formLayout = QFormLayout(KeybindingConfig)
        self.formLayout.setObjectName(u"formLayout")
        self.select_item_layout = QHBoxLayout()
        self.select_item_layout.setObjectName(u"select_item_layout")
        self.select_item_label = QLabel(KeybindingConfig)
        self.select_item_label.setObjectName(u"select_item_label")

        self.select_item_layout.addWidget(self.select_item_label)

        self.select_item_button = QToolButton(KeybindingConfig)
        self.select_item_button.setObjectName(u"select_item_button")
        self.select_item_button.setText(u"#ffffff")

        self.select_item_layout.addWidget(self.select_item_button)


        self.formLayout.setLayout(0, QFormLayout.ItemRole.SpanningRole, self.select_item_layout)

        self.remove_reposition_layout = QHBoxLayout()
        self.remove_reposition_layout.setObjectName(u"remove_reposition_layout")
        self.remove_reposition_label = QLabel(KeybindingConfig)
        self.remove_reposition_label.setObjectName(u"remove_reposition_label")

        self.remove_reposition_layout.addWidget(self.remove_reposition_label)

        self.remove_reposition_button = QToolButton(KeybindingConfig)
        self.remove_reposition_button.setObjectName(u"remove_reposition_button")
        self.remove_reposition_button.setText(u"#ffffff")

        self.remove_reposition_layout.addWidget(self.remove_reposition_button)


        self.formLayout.setLayout(1, QFormLayout.ItemRole.SpanningRole, self.remove_reposition_layout)

        self.delete_manual_ads_layout = QHBoxLayout()
        self.delete_manual_ads_layout.setObjectName(u"delete_manual_ads_layout")
        self.delete_manual_ads_label = QLabel(KeybindingConfig)
        self.delete_manual_ads_label.setObjectName(u"delete_manual_ads_label")

        self.delete_manual_ads_layout.addWidget(self.delete_manual_ads_label)

        self.delete_manual_ads_button = QToolButton(KeybindingConfig)
        self.delete_manual_ads_button.setObjectName(u"delete_manual_ads_button")
        self.delete_manual_ads_button.setText(u"#ffffff")

        self.delete_manual_ads_layout.addWidget(self.delete_manual_ads_button)


        self.formLayout.setLayout(2, QFormLayout.ItemRole.SpanningRole, self.delete_manual_ads_layout)

        self.send_reposition_command_layout = QHBoxLayout()
        self.send_reposition_command_layout.setObjectName(u"send_reposition_command_layout")
        self.send_reposition_command_label = QLabel(KeybindingConfig)
        self.send_reposition_command_label.setObjectName(u"send_reposition_command_label")

        self.send_reposition_command_layout.addWidget(self.send_reposition_command_label)

        self.send_reposition_command_button = QToolButton(KeybindingConfig)
        self.send_reposition_command_button.setObjectName(u"send_reposition_command_button")
        self.send_reposition_command_button.setText(u"#ffffff")

        self.send_reposition_command_layout.addWidget(self.send_reposition_command_button)


        self.formLayout.setLayout(3, QFormLayout.ItemRole.SpanningRole, self.send_reposition_command_layout)

        self.send_adv_reposition_command_layout = QHBoxLayout()
        self.send_adv_reposition_command_layout.setObjectName(u"send_adv_reposition_command_layout")
        self.send_adv_reposition_command_label = QLabel(KeybindingConfig)
        self.send_adv_reposition_command_label.setObjectName(u"send_adv_reposition_command_label")

        self.send_adv_reposition_command_layout.addWidget(self.send_adv_reposition_command_label)

        self.send_adv_reposition_command_button = QToolButton(KeybindingConfig)
        self.send_adv_reposition_command_button.setObjectName(u"send_adv_reposition_command_button")
        self.send_adv_reposition_command_button.setText(u"#ffffff")

        self.send_adv_reposition_command_layout.addWidget(self.send_adv_reposition_command_button)


        self.formLayout.setLayout(4, QFormLayout.ItemRole.SpanningRole, self.send_adv_reposition_command_layout)

        self.add_geofence_point_layout = QHBoxLayout()
        self.add_geofence_point_layout.setObjectName(u"add_geofence_point_layout")
        self.add_geofence_point_label = QLabel(KeybindingConfig)
        self.add_geofence_point_label.setObjectName(u"add_geofence_point_label")

        self.add_geofence_point_layout.addWidget(self.add_geofence_point_label)

        self.add_geofence_point_button = QToolButton(KeybindingConfig)
        self.add_geofence_point_button.setObjectName(u"add_geofence_point_button")
        self.add_geofence_point_button.setText(u"#ffffff")

        self.add_geofence_point_layout.addWidget(self.add_geofence_point_button)


        self.formLayout.setLayout(5, QFormLayout.ItemRole.SpanningRole, self.add_geofence_point_layout)

        self.buttons = QDialogButtonBox(KeybindingConfig)
        self.buttons.setObjectName(u"buttons")
        self.buttons.setStandardButtons(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.RestoreDefaults)

        self.formLayout.setWidget(6, QFormLayout.ItemRole.SpanningRole, self.buttons)

#if QT_CONFIG(shortcut)
        self.select_item_label.setBuddy(self.select_item_button)
        self.remove_reposition_label.setBuddy(self.remove_reposition_button)
        self.delete_manual_ads_label.setBuddy(self.delete_manual_ads_button)
        self.send_reposition_command_label.setBuddy(self.send_reposition_command_button)
        self.send_adv_reposition_command_label.setBuddy(self.send_adv_reposition_command_button)
        self.add_geofence_point_label.setBuddy(self.add_geofence_point_button)
#endif // QT_CONFIG(shortcut)

        self.retranslateUi(KeybindingConfig)

        QMetaObject.connectSlotsByName(KeybindingConfig)
    # setupUi

    def retranslateUi(self, KeybindingConfig):
        KeybindingConfig.setWindowTitle(QCoreApplication.translate("KeybindingConfig", u"Keybinding Config", None))
        self.select_item_label.setText(QCoreApplication.translate("KeybindingConfig", u"Select Item", None))
        self.remove_reposition_label.setText(QCoreApplication.translate("KeybindingConfig", u"Remove Reposition Command", None))
        self.delete_manual_ads_label.setText(QCoreApplication.translate("KeybindingConfig", u"Delete Manual ADS", None))
        self.send_reposition_command_label.setText(QCoreApplication.translate("KeybindingConfig", u"Send Reposition Command", None))
        self.send_adv_reposition_command_label.setText(QCoreApplication.translate("KeybindingConfig", u"Send Advanced Reposition Command", None))
        self.add_geofence_point_label.setText(QCoreApplication.translate("KeybindingConfig", u"Add Geofence point", None))
    # retranslateUi

