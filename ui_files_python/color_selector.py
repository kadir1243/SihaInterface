# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'color_selector.ui'
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
from PySide6.QtWidgets import (QAbstractButton, QApplication, QComboBox, QDialog,
    QDialogButtonBox, QFormLayout, QHBoxLayout, QLabel,
    QSizePolicy, QToolButton, QWidget)

class Ui_ColorSelector(object):
    def setupUi(self, ColorSelector):
        if not ColorSelector.objectName():
            ColorSelector.setObjectName(u"ColorSelector")
        ColorSelector.resize(561, 796)
        self.formLayout = QFormLayout(ColorSelector)
        self.formLayout.setObjectName(u"formLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(ColorSelector)
        self.label.setObjectName(u"label")

        self.horizontalLayout.addWidget(self.label)

        self.main_window_background = QToolButton(ColorSelector)
        self.main_window_background.setObjectName(u"main_window_background")

        self.horizontalLayout.addWidget(self.main_window_background)


        self.formLayout.setLayout(0, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_7 = QLabel(ColorSelector)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_2.addWidget(self.label_7)

        self.main_window_alternate_background = QToolButton(ColorSelector)
        self.main_window_alternate_background.setObjectName(u"main_window_alternate_background")

        self.horizontalLayout_2.addWidget(self.main_window_alternate_background)


        self.formLayout.setLayout(1, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_8 = QLabel(ColorSelector)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_3.addWidget(self.label_8)

        self.splitter = QToolButton(ColorSelector)
        self.splitter.setObjectName(u"splitter")

        self.horizontalLayout_3.addWidget(self.splitter)


        self.formLayout.setLayout(2, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_9 = QLabel(ColorSelector)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_4.addWidget(self.label_9)

        self.main_window_frame = QToolButton(ColorSelector)
        self.main_window_frame.setObjectName(u"main_window_frame")

        self.horizontalLayout_4.addWidget(self.main_window_frame)


        self.formLayout.setLayout(3, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_6 = QLabel(ColorSelector)
        self.label_6.setObjectName(u"label_6")

        self.horizontalLayout_5.addWidget(self.label_6)

        self.menu_item = QToolButton(ColorSelector)
        self.menu_item.setObjectName(u"menu_item")

        self.horizontalLayout_5.addWidget(self.menu_item)


        self.formLayout.setLayout(4, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_5)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_5 = QLabel(ColorSelector)
        self.label_5.setObjectName(u"label_5")

        self.horizontalLayout_6.addWidget(self.label_5)

        self.menu_selected_item_color = QToolButton(ColorSelector)
        self.menu_selected_item_color.setObjectName(u"menu_selected_item_color")

        self.horizontalLayout_6.addWidget(self.menu_selected_item_color)


        self.formLayout.setLayout(5, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_2 = QLabel(ColorSelector)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_7.addWidget(self.label_2)

        self.button = QToolButton(ColorSelector)
        self.button.setObjectName(u"button")

        self.horizontalLayout_7.addWidget(self.button)


        self.formLayout.setLayout(6, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_7)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.label_3 = QLabel(ColorSelector)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_8.addWidget(self.label_3)

        self.selected_button = QToolButton(ColorSelector)
        self.selected_button.setObjectName(u"selected_button")

        self.horizontalLayout_8.addWidget(self.selected_button)


        self.formLayout.setLayout(7, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_8)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.label_4 = QLabel(ColorSelector)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_9.addWidget(self.label_4)

        self.hovered_button = QToolButton(ColorSelector)
        self.hovered_button.setObjectName(u"hovered_button")

        self.horizontalLayout_9.addWidget(self.hovered_button)


        self.formLayout.setLayout(8, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_9)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.label_10 = QLabel(ColorSelector)
        self.label_10.setObjectName(u"label_10")

        self.horizontalLayout_10.addWidget(self.label_10)

        self.button_background = QToolButton(ColorSelector)
        self.button_background.setObjectName(u"button_background")

        self.horizontalLayout_10.addWidget(self.button_background)


        self.formLayout.setLayout(9, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_10)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.label_11 = QLabel(ColorSelector)
        self.label_11.setObjectName(u"label_11")

        self.horizontalLayout_11.addWidget(self.label_11)

        self.selected_button_background = QToolButton(ColorSelector)
        self.selected_button_background.setObjectName(u"selected_button_background")

        self.horizontalLayout_11.addWidget(self.selected_button_background)


        self.formLayout.setLayout(10, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_11)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_12 = QLabel(ColorSelector)
        self.label_12.setObjectName(u"label_12")

        self.horizontalLayout_12.addWidget(self.label_12)

        self.hovered_button_background = QToolButton(ColorSelector)
        self.hovered_button_background.setObjectName(u"hovered_button_background")

        self.horizontalLayout_12.addWidget(self.hovered_button_background)


        self.formLayout.setLayout(11, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_12)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.label_13 = QLabel(ColorSelector)
        self.label_13.setObjectName(u"label_13")

        self.horizontalLayout_13.addWidget(self.label_13)

        self.combo_box = QToolButton(ColorSelector)
        self.combo_box.setObjectName(u"combo_box")

        self.horizontalLayout_13.addWidget(self.combo_box)


        self.formLayout.setLayout(12, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_13)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_15 = QLabel(ColorSelector)
        self.label_15.setObjectName(u"label_15")

        self.horizontalLayout_15.addWidget(self.label_15)

        self.text_box_border = QToolButton(ColorSelector)
        self.text_box_border.setObjectName(u"text_box_border")

        self.horizontalLayout_15.addWidget(self.text_box_border)


        self.formLayout.setLayout(13, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_15)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.label_16 = QLabel(ColorSelector)
        self.label_16.setObjectName(u"label_16")

        self.horizontalLayout_16.addWidget(self.label_16)

        self.text_box_background = QToolButton(ColorSelector)
        self.text_box_background.setObjectName(u"text_box_background")

        self.horizontalLayout_16.addWidget(self.text_box_background)


        self.formLayout.setLayout(14, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_16)

        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.label_14 = QLabel(ColorSelector)
        self.label_14.setObjectName(u"label_14")

        self.horizontalLayout_14.addWidget(self.label_14)

        self.combo_box_background = QToolButton(ColorSelector)
        self.combo_box_background.setObjectName(u"combo_box_background")

        self.horizontalLayout_14.addWidget(self.combo_box_background)


        self.formLayout.setLayout(15, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_14)

        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.label_17 = QLabel(ColorSelector)
        self.label_17.setObjectName(u"label_17")

        self.horizontalLayout_17.addWidget(self.label_17)

        self.icon_theme = QComboBox(ColorSelector)
        self.icon_theme.addItem("")
        self.icon_theme.addItem("")
        self.icon_theme.setObjectName(u"icon_theme")

        self.horizontalLayout_17.addWidget(self.icon_theme)


        self.formLayout.setLayout(16, QFormLayout.ItemRole.SpanningRole, self.horizontalLayout_17)

        self.buttons = QDialogButtonBox(ColorSelector)
        self.buttons.setObjectName(u"buttons")
        self.buttons.setStandardButtons(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.RestoreDefaults)

        self.formLayout.setWidget(17, QFormLayout.ItemRole.LabelRole, self.buttons)


        self.retranslateUi(ColorSelector)

        self.icon_theme.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(ColorSelector)
    # setupUi

    def retranslateUi(self, ColorSelector):
        ColorSelector.setWindowTitle(QCoreApplication.translate("ColorSelector", u"ColorSelector", None))
        self.label.setText(QCoreApplication.translate("ColorSelector", u"Main Window Background", None))
        self.main_window_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_7.setText(QCoreApplication.translate("ColorSelector", u"Main Window Alternate Background", None))
        self.main_window_alternate_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_8.setText(QCoreApplication.translate("ColorSelector", u"Splitter", None))
        self.splitter.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_9.setText(QCoreApplication.translate("ColorSelector", u"Main Window Frame", None))
        self.main_window_frame.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_6.setText(QCoreApplication.translate("ColorSelector", u"Menu Item", None))
        self.menu_item.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_5.setText(QCoreApplication.translate("ColorSelector", u"Menu Selected Item Color", None))
        self.menu_selected_item_color.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_2.setText(QCoreApplication.translate("ColorSelector", u"Button", None))
        self.button.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_3.setText(QCoreApplication.translate("ColorSelector", u"Selected Button", None))
        self.selected_button.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_4.setText(QCoreApplication.translate("ColorSelector", u"Hovered Button", None))
        self.hovered_button.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_10.setText(QCoreApplication.translate("ColorSelector", u"Button Background", None))
        self.button_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_11.setText(QCoreApplication.translate("ColorSelector", u"Selected Button Background", None))
        self.selected_button_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_12.setText(QCoreApplication.translate("ColorSelector", u"Hovered Button Background", None))
        self.hovered_button_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_13.setText(QCoreApplication.translate("ColorSelector", u"Combo Box", None))
        self.combo_box.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_15.setText(QCoreApplication.translate("ColorSelector", u"Text Box Border", None))
        self.text_box_border.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_16.setText(QCoreApplication.translate("ColorSelector", u"Text Box Background", None))
        self.text_box_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_14.setText(QCoreApplication.translate("ColorSelector", u"Combo Box Background", None))
        self.combo_box_background.setText(QCoreApplication.translate("ColorSelector", u"#ffffff", None))
        self.label_17.setText(QCoreApplication.translate("ColorSelector", u"Icon Theme", None))
        self.icon_theme.setItemText(0, QCoreApplication.translate("ColorSelector", u"Light", None))
        self.icon_theme.setItemText(1, QCoreApplication.translate("ColorSelector", u"Dark", None))

    # retranslateUi

