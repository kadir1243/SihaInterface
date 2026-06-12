# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'uav_interface.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QComboBox, QFrame,
    QGridLayout, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QMainWindow, QMenu, QMenuBar,
    QPushButton, QSizePolicy, QSplitter, QStatusBar,
    QTableWidget, QTableWidgetItem, QToolButton, QVBoxLayout,
    QWidget)

from src.CameraWidget import CameraWidget
from src.MapWidget import MapWidget

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1021, 860)
        icon = QIcon()
        icon.addFile(u"ui_files/ares-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet(u"QWidget#centralwidget, QWidget#remaining_thingies_frame, QWidget#kamikaze_panel, QWidget#target_tracking_panel\n"
"{\n"
"    color: #eff0f1;\n"
"	background-color: rgb(32, 35, 38);\n"
"	alternate-background-color: rgb(41,44,48);\n"
"    selection-background-color: #3daee9;\n"
"    selection-color: #eff0f1;\n"
"}\n"
"\n"
"QSplitter::handle:horizontal {\n"
"    border: 1px dotted darkgrey;\n"
"	color: rgb(8, 56, 56);\n"
"    height: 1px;\n"
"}\n"
"\n"
"QSplitter::handle:vertical {\n"
"    border: 1px dotted darkgrey;\n"
"	color: rgb(8, 56, 56);\n"
"    height: 1px;\n"
"}\n"
"\n"
"QMenuBar {\n"
"	background-color: rgb(29, 34, 38);\n"
"    spacing: 3px;\n"
"}\n"
"\n"
"QMenu {\n"
"	background: rgb(29, 34, 38);\n"
"}\n"
"\n"
"QMenu::item:selected {\n"
"    background: #a8a8a8;\n"
"}\n"
"\n"
"QMenu::item:pressed {\n"
"    background: #888888;\n"
"}\n"
"\n"
"QStatusBar {\n"
"    background: rgb(29, 34, 38);\n"
"}\n"
"\n"
"QMenuBar::item {\n"
"	background-color: black;\n"
"    padding: 1px 4px;\n"
"    background: tran"
                        "sparent;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QMenuBar::item:selected {\n"
"    background: #a8a8a8;\n"
"}\n"
"\n"
"QMenuBar::item:pressed {\n"
"    background: #888888;\n"
"}\n"
"\n"
"QAbstractButton {\n"
"	color: #eff0f1; \n"
"	background-color: rgb(45, 48, 51);\n"
"	selection-background-color: rgb(63, 67, 71);\n"
"}\n"
"\n"
"QAbstractButton:pressed {\n"
"	color: #eff0f1; \n"
"	background-color: rgb(67, 72, 76);\n"
"}\n"
"\n"
"QAbstractButton:checked {\n"
"	color: #eff0f1; \n"
"	background-color: rgb(67, 72, 76);\n"
"}\n"
"\n"
"QComboBox {\n"
"	color: #eff0f1; \n"
"    background-color: rgb(45, 48, 51);\n"
"    padding: 1px 18px 1px 3px;\n"
"    min-width: 6em;\n"
"}\n"
"\n"
"QComboBox QAbstractItemView {\n"
"	color: #eff0f1; \n"
"    background-color: rgb(45, 48, 51);\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"	color: #eff0f1; \n"
"	background-color: rgb(30, 30, 30);\n"
"    border-top-left-radius: 4px;\n"
"    border-top-right-radius: 4px;\n"
"    min-width: 8ex;\n"
"    padding: 2px;\n"
"}\n"
"\n"
"QTabBar::"
                        "tab:selected, QTabBar::tab:hover {\n"
"	color: #eff0f1; \n"
"	background-color: rgb(45, 45, 45);\n"
"}\n"
"\n"
"QTabWidget::pane {\n"
"    border: 1px solid rgb(41, 44, 47);\n"
"}\n"
"\n"
"QLineEdit {\n"
"	border: 2px solid rgb(45, 45, 45);\n"
"    background: rgb(45, 45, 45);\n"
"	selection-background-color: darkgray;\n"
"}")
        self.actionConfigurateServer = QAction(MainWindow)
        self.actionConfigurateServer.setObjectName(u"actionConfigurateServer")
        self.actionAir_Defence_System_Location = QAction(MainWindow)
        self.actionAir_Defence_System_Location.setObjectName(u"actionAir_Defence_System_Location")
        self.actionAbout = QAction(MainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.actionConfigurate_UAV = QAction(MainWindow)
        self.actionConfigurate_UAV.setObjectName(u"actionConfigurate_UAV")
        self.actionConfigurate_KamikazeCoords = QAction(MainWindow)
        self.actionConfigurate_KamikazeCoords.setObjectName(u"actionConfigurate_KamikazeCoords")
        self.actionForce_Send_Testing_Telemetry_Data = QAction(MainWindow)
        self.actionForce_Send_Testing_Telemetry_Data.setObjectName(u"actionForce_Send_Testing_Telemetry_Data")
        self.actionForce_Send_Testing_Telemetry_Data.setCheckable(True)
        self.actionSet_Colors = QAction(MainWindow)
        self.actionSet_Colors.setObjectName(u"actionSet_Colors")
        self.actionConfigurate_Camera_Stream = QAction(MainWindow)
        self.actionConfigurate_Camera_Stream.setObjectName(u"actionConfigurate_Camera_Stream")
        self.actionAbout_Qt = QAction(MainWindow)
        self.actionAbout_Qt.setObjectName(u"actionAbout_Qt")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout_3 = QGridLayout(self.centralwidget)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.splitter_2 = QSplitter(self.centralwidget)
        self.splitter_2.setObjectName(u"splitter_2")
        self.splitter_2.setOrientation(Qt.Orientation.Horizontal)
        self.left_side_of_gui = QWidget(self.splitter_2)
        self.left_side_of_gui.setObjectName(u"left_side_of_gui")
        self.gridLayout = QGridLayout(self.left_side_of_gui)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, -1, 3, -1)
        self.splitter_3 = QSplitter(self.left_side_of_gui)
        self.splitter_3.setObjectName(u"splitter_3")
        self.splitter_3.setOrientation(Qt.Orientation.Vertical)
        self.layoutWidget1 = QWidget(self.splitter_3)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.watcher_list_box_layout = QVBoxLayout(self.layoutWidget1)
        self.watcher_list_box_layout.setSpacing(0)
        self.watcher_list_box_layout.setObjectName(u"watcher_list_box_layout")
        self.watcher_list_box_layout.setContentsMargins(0, 0, 0, 0)
        self.server_connection_warning = QLabel(self.layoutWidget1)
        self.server_connection_warning.setObjectName(u"server_connection_warning")
        self.server_connection_warning.setTextFormat(Qt.TextFormat.RichText)

        self.watcher_list_box_layout.addWidget(self.server_connection_warning, 0, Qt.AlignmentFlag.AlignHCenter)

        self.device_connection_warning = QLabel(self.layoutWidget1)
        self.device_connection_warning.setObjectName(u"device_connection_warning")

        self.watcher_list_box_layout.addWidget(self.device_connection_warning, 0, Qt.AlignmentFlag.AlignHCenter)

        self.watch_list_frame = QFrame(self.layoutWidget1)
        self.watch_list_frame.setObjectName(u"watch_list_frame")
        self.verticalLayout_3 = QVBoxLayout(self.watch_list_frame)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.watch_list = QTableWidget(self.watch_list_frame)
        if (self.watch_list.columnCount() < 4):
            self.watch_list.setColumnCount(4)
        self.watch_list.setObjectName(u"watch_list")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(200)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.watch_list.sizePolicy().hasHeightForWidth())
        self.watch_list.setSizePolicy(sizePolicy)
        self.watch_list.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.watch_list.setProperty(u"showDropIndicator", False)
        self.watch_list.setDragDropOverwriteMode(False)
        self.watch_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)
        self.watch_list.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.watch_list.setShowGrid(False)
        self.watch_list.setGridStyle(Qt.PenStyle.NoPen)
        self.watch_list.setColumnCount(4)
        self.watch_list.horizontalHeader().setVisible(False)
        self.watch_list.horizontalHeader().setStretchLastSection(True)
        self.watch_list.verticalHeader().setVisible(False)

        self.verticalLayout_3.addWidget(self.watch_list)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.remove_from_watch = QToolButton(self.watch_list_frame)
        self.remove_from_watch.setObjectName(u"remove_from_watch")
        icon1 = QIcon()
        icon1.addFile(u"ui_files/minus-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.remove_from_watch.setIcon(icon1)

        self.horizontalLayout.addWidget(self.remove_from_watch, 0, Qt.AlignmentFlag.AlignLeft)

        self.add_to_watch = QToolButton(self.watch_list_frame)
        self.add_to_watch.setObjectName(u"add_to_watch")
        icon2 = QIcon()
        icon2.addFile(u"ui_files/plus-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.add_to_watch.setIcon(icon2)
        self.add_to_watch.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)

        self.horizontalLayout.addWidget(self.add_to_watch, 0, Qt.AlignmentFlag.AlignRight)


        self.verticalLayout_3.addLayout(self.horizontalLayout)


        self.watcher_list_box_layout.addWidget(self.watch_list_frame)

        self.splitter_3.addWidget(self.layoutWidget1)
        self.remaining_thingies_frame = QFrame(self.splitter_3)
        self.remaining_thingies_frame.setObjectName(u"remaining_thingies_frame")
        self.remaining_thingies_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.remaining_thingies_frame.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_4 = QVBoxLayout(self.remaining_thingies_frame)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.fly_mode_as_frame = QFrame(self.remaining_thingies_frame)
        self.fly_mode_as_frame.setObjectName(u"fly_mode_as_frame")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(40)
        sizePolicy1.setHeightForWidth(self.fly_mode_as_frame.sizePolicy().hasHeightForWidth())
        self.fly_mode_as_frame.setSizePolicy(sizePolicy1)
        self.fly_mode_layout = QHBoxLayout(self.fly_mode_as_frame)
        self.fly_mode_layout.setSpacing(0)
        self.fly_mode_layout.setObjectName(u"fly_mode_layout")
        self.fly_mode_layout.setContentsMargins(-1, 0, -1, 1)
        self.fly_mode_label = QLabel(self.fly_mode_as_frame)
        self.fly_mode_label.setObjectName(u"fly_mode_label")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.fly_mode_label.sizePolicy().hasHeightForWidth())
        self.fly_mode_label.setSizePolicy(sizePolicy2)
        self.fly_mode_label.setMaximumSize(QSize(16777215, 40))

        self.fly_mode_layout.addWidget(self.fly_mode_label, 0, Qt.AlignmentFlag.AlignHCenter)

        self.fly_mode_combobox = QComboBox(self.fly_mode_as_frame)
        self.fly_mode_combobox.setObjectName(u"fly_mode_combobox")
        self.fly_mode_combobox.setEnabled(False)
        self.fly_mode_combobox.setMaximumSize(QSize(16777215, 40))

        self.fly_mode_layout.addWidget(self.fly_mode_combobox)


        self.verticalLayout_4.addWidget(self.fly_mode_as_frame)

        self.frame = QFrame(self.remaining_thingies_frame)
        self.frame.setObjectName(u"frame")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy3)
        self.horizontalLayout_2 = QHBoxLayout(self.frame)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(-1, 0, -1, 0)
        self.arm_mode_label = QLabel(self.frame)
        self.arm_mode_label.setObjectName(u"arm_mode_label")
        sizePolicy2.setHeightForWidth(self.arm_mode_label.sizePolicy().hasHeightForWidth())
        self.arm_mode_label.setSizePolicy(sizePolicy2)
        self.arm_mode_label.setMaximumSize(QSize(16777215, 40))

        self.horizontalLayout_2.addWidget(self.arm_mode_label, 0, Qt.AlignmentFlag.AlignHCenter)

        self.arm_mode = QComboBox(self.frame)
        self.arm_mode.addItem(u"DISARMED")
        self.arm_mode.addItem(u"ARMED")
        self.arm_mode.setObjectName(u"arm_mode")
        self.arm_mode.setEnabled(False)
        self.arm_mode.setMaximumSize(QSize(16777215, 40))
        self.arm_mode.setMaxVisibleItems(2)
        self.arm_mode.setMaxCount(2)
        self.arm_mode.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

        self.horizontalLayout_2.addWidget(self.arm_mode)


        self.verticalLayout_4.addWidget(self.frame)

        self.widget = QWidget(self.remaining_thingies_frame)
        self.widget.setObjectName(u"widget")
        self.verticalLayout_5 = QVBoxLayout(self.widget)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.kamikaze_latitude_layout = QHBoxLayout()
        self.kamikaze_latitude_layout.setObjectName(u"kamikaze_latitude_layout")
        self.server_ip_label = QLabel(self.widget)
        self.server_ip_label.setObjectName(u"server_ip_label")
        sizePolicy3.setHeightForWidth(self.server_ip_label.sizePolicy().hasHeightForWidth())
        self.server_ip_label.setSizePolicy(sizePolicy3)
        self.server_ip_label.setTextFormat(Qt.TextFormat.PlainText)

        self.kamikaze_latitude_layout.addWidget(self.server_ip_label)

        self.kamikaze_latitude = QLineEdit(self.widget)
        self.kamikaze_latitude.setObjectName(u"kamikaze_latitude")
        self.kamikaze_latitude.setMaxLength(16)

        self.kamikaze_latitude_layout.addWidget(self.kamikaze_latitude)


        self.verticalLayout_5.addLayout(self.kamikaze_latitude_layout)

        self.kamikaze_longitude_layout = QHBoxLayout()
        self.kamikaze_longitude_layout.setObjectName(u"kamikaze_longitude_layout")
        self.server_port_label = QLabel(self.widget)
        self.server_port_label.setObjectName(u"server_port_label")
        sizePolicy3.setHeightForWidth(self.server_port_label.sizePolicy().hasHeightForWidth())
        self.server_port_label.setSizePolicy(sizePolicy3)
        self.server_port_label.setTextFormat(Qt.TextFormat.PlainText)

        self.kamikaze_longitude_layout.addWidget(self.server_port_label)

        self.kamikaze_longitude = QLineEdit(self.widget)
        self.kamikaze_longitude.setObjectName(u"kamikaze_longitude")
        self.kamikaze_longitude.setMaxLength(16)

        self.kamikaze_longitude_layout.addWidget(self.kamikaze_longitude)


        self.verticalLayout_5.addLayout(self.kamikaze_longitude_layout)

        self.get_kamikaze_coords_from_server = QPushButton(self.widget)
        self.get_kamikaze_coords_from_server.setObjectName(u"get_kamikaze_coords_from_server")

        self.verticalLayout_5.addWidget(self.get_kamikaze_coords_from_server)

        self.start_kamikaze = QPushButton(self.widget)
        self.start_kamikaze.setObjectName(u"start_kamikaze")

        self.verticalLayout_5.addWidget(self.start_kamikaze)

        self.force_end_task = QPushButton(self.widget)
        self.force_end_task.setObjectName(u"force_end_task")

        self.verticalLayout_5.addWidget(self.force_end_task)


        self.verticalLayout_4.addWidget(self.widget)

        self.splitter_3.addWidget(self.remaining_thingies_frame)

        self.gridLayout.addWidget(self.splitter_3, 0, 0, 1, 1)

        self.splitter_2.addWidget(self.left_side_of_gui)
        self.right_side_of_gui = QWidget(self.splitter_2)
        self.right_side_of_gui.setObjectName(u"right_side_of_gui")
        self.verticalLayout_8 = QVBoxLayout(self.right_side_of_gui)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.splitter = QSplitter(self.right_side_of_gui)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Orientation.Vertical)
        self.layoutWidget2 = QWidget(self.splitter)
        self.layoutWidget2.setObjectName(u"layoutWidget2")
        self.camera_horizontal_layout = QHBoxLayout(self.layoutWidget2)
        self.camera_horizontal_layout.setSpacing(0)
        self.camera_horizontal_layout.setObjectName(u"camera_horizontal_layout")
        self.camera_horizontal_layout.setContentsMargins(0, 0, 0, 0)
        self.camera_buttons = QVBoxLayout()
        self.camera_buttons.setSpacing(0)
        self.camera_buttons.setObjectName(u"camera_buttons")
        self.start_stop_camera_view = QPushButton(self.layoutWidget2)
        self.start_stop_camera_view.setObjectName(u"start_stop_camera_view")
        self.start_stop_camera_view.setMinimumSize(QSize(0, 0))
        self.start_stop_camera_view.setMaximumSize(QSize(30, 30))
        self.start_stop_camera_view.setSizeIncrement(QSize(30, 30))
        icon3 = QIcon()
        icon3.addFile(u"ui_files/stop-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        icon3.addFile(u"ui_files/start-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.On)
        self.start_stop_camera_view.setIcon(icon3)
        self.start_stop_camera_view.setIconSize(QSize(32, 32))

        self.camera_buttons.addWidget(self.start_stop_camera_view)

        self.disable_enable_locking = QPushButton(self.layoutWidget2)
        self.disable_enable_locking.setObjectName(u"disable_enable_locking")
        self.disable_enable_locking.setMinimumSize(QSize(0, 0))
        self.disable_enable_locking.setMaximumSize(QSize(30, 30))
        self.disable_enable_locking.setSizeIncrement(QSize(30, 30))
        icon4 = QIcon()
        icon4.addFile(u"ui_files/lock_to_target.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.disable_enable_locking.setIcon(icon4)

        self.camera_buttons.addWidget(self.disable_enable_locking)

        self.record_button = QPushButton(self.layoutWidget2)
        self.record_button.setObjectName(u"record_button")
        self.record_button.setMinimumSize(QSize(0, 0))
        self.record_button.setMaximumSize(QSize(30, 30))
        self.record_button.setSizeIncrement(QSize(30, 30))
        icon5 = QIcon()
        icon5.addFile(u"ui_files/record_button_offline.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        icon5.addFile(u"ui_files/record_button_online.svg", QSize(), QIcon.Mode.Normal, QIcon.State.On)
        self.record_button.setIcon(icon5)

        self.camera_buttons.addWidget(self.record_button)


        self.camera_horizontal_layout.addLayout(self.camera_buttons)

        self.camera_view = CameraWidget(self.layoutWidget2)
        self.camera_view.setObjectName(u"camera_view")

        self.camera_horizontal_layout.addWidget(self.camera_view, 0, Qt.AlignmentFlag.AlignHCenter)

        self.splitter.addWidget(self.layoutWidget2)
        self.layoutWidget3 = QWidget(self.splitter)
        self.layoutWidget3.setObjectName(u"layoutWidget3")
        self.map_horizontal_layout = QHBoxLayout(self.layoutWidget3)
        self.map_horizontal_layout.setSpacing(0)
        self.map_horizontal_layout.setObjectName(u"map_horizontal_layout")
        self.map_horizontal_layout.setContentsMargins(0, 0, 0, 0)
        self.map_buttons = QVBoxLayout()
        self.map_buttons.setObjectName(u"map_buttons")
        self.set_fence = QPushButton(self.layoutWidget3)
        self.set_fence.setObjectName(u"set_fence")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.set_fence.sizePolicy().hasHeightForWidth())
        self.set_fence.setSizePolicy(sizePolicy4)
        self.set_fence.setMinimumSize(QSize(0, 0))
        self.set_fence.setMaximumSize(QSize(30, 30))
        self.set_fence.setSizeIncrement(QSize(30, 30))
        icon6 = QIcon()
        icon6.addFile(u"ui_files/fence-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.set_fence.setIcon(icon6)

        self.map_buttons.addWidget(self.set_fence, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.add_ads = QPushButton(self.layoutWidget3)
        self.add_ads.setObjectName(u"add_ads")
        sizePolicy4.setHeightForWidth(self.add_ads.sizePolicy().hasHeightForWidth())
        self.add_ads.setSizePolicy(sizePolicy4)
        self.add_ads.setMinimumSize(QSize(0, 0))
        self.add_ads.setMaximumSize(QSize(30, 30))
        self.add_ads.setSizeIncrement(QSize(30, 30))
        self.add_ads.setIcon(icon2)

        self.map_buttons.addWidget(self.add_ads, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.remove_ads = QPushButton(self.layoutWidget3)
        self.remove_ads.setObjectName(u"remove_ads")
        sizePolicy4.setHeightForWidth(self.remove_ads.sizePolicy().hasHeightForWidth())
        self.remove_ads.setSizePolicy(sizePolicy4)
        self.remove_ads.setMinimumSize(QSize(0, 0))
        self.remove_ads.setMaximumSize(QSize(30, 30))
        self.remove_ads.setSizeIncrement(QSize(30, 30))
        self.remove_ads.setIcon(icon1)

        self.map_buttons.addWidget(self.remove_ads, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.refresh_ads = QPushButton(self.layoutWidget3)
        self.refresh_ads.setObjectName(u"refresh_ads")
        sizePolicy4.setHeightForWidth(self.refresh_ads.sizePolicy().hasHeightForWidth())
        self.refresh_ads.setSizePolicy(sizePolicy4)
        self.refresh_ads.setMinimumSize(QSize(0, 0))
        self.refresh_ads.setMaximumSize(QSize(30, 30))
        self.refresh_ads.setSizeIncrement(QSize(30, 30))
        icon7 = QIcon()
        icon7.addFile(u"ui_files/refresh-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.refresh_ads.setIcon(icon7)

        self.map_buttons.addWidget(self.refresh_ads, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.download_fence_data = QPushButton(self.layoutWidget3)
        self.download_fence_data.setObjectName(u"download_fence_data")
        sizePolicy4.setHeightForWidth(self.download_fence_data.sizePolicy().hasHeightForWidth())
        self.download_fence_data.setSizePolicy(sizePolicy4)
        self.download_fence_data.setMinimumSize(QSize(0, 0))
        self.download_fence_data.setMaximumSize(QSize(30, 30))
        self.download_fence_data.setSizeIncrement(QSize(30, 30))
        icon8 = QIcon()
        icon8.addFile(u"ui_files/download_fence-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.download_fence_data.setIcon(icon8)

        self.map_buttons.addWidget(self.download_fence_data, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.download_missions = QPushButton(self.layoutWidget3)
        self.download_missions.setObjectName(u"download_missions")
        sizePolicy4.setHeightForWidth(self.download_missions.sizePolicy().hasHeightForWidth())
        self.download_missions.setSizePolicy(sizePolicy4)
        self.download_missions.setMinimumSize(QSize(0, 0))
        self.download_missions.setMaximumSize(QSize(30, 30))
        self.download_missions.setSizeIncrement(QSize(30, 30))
        icon9 = QIcon()
        icon9.addFile(u"ui_files/download_mission-white.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.download_missions.setIcon(icon9)

        self.map_buttons.addWidget(self.download_missions, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.map_horizontal_layout.addLayout(self.map_buttons)

        self.map_view = MapWidget(self.layoutWidget3)
        self.map_view.setObjectName(u"map_view")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.map_view.sizePolicy().hasHeightForWidth())
        self.map_view.setSizePolicy(sizePolicy5)

        self.map_horizontal_layout.addWidget(self.map_view)

        self.splitter.addWidget(self.layoutWidget3)

        self.verticalLayout_8.addWidget(self.splitter)

        self.splitter_2.addWidget(self.right_side_of_gui)

        self.gridLayout_3.addWidget(self.splitter_2, 0, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1021, 20))
        self.menuConfigurations = QMenu(self.menubar)
        self.menuConfigurations.setObjectName(u"menuConfigurations")
        self.menuHelp = QMenu(self.menubar)
        self.menuHelp.setObjectName(u"menuHelp")
        self.menuTesting = QMenu(self.menubar)
        self.menuTesting.setObjectName(u"menuTesting")
        self.menuCustomize = QMenu(self.menubar)
        self.menuCustomize.setObjectName(u"menuCustomize")
        self.menuChange_Language = QMenu(self.menuCustomize)
        self.menuChange_Language.setObjectName(u"menuChange_Language")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
#if QT_CONFIG(shortcut)
        self.fly_mode_label.setBuddy(self.fly_mode_combobox)
        self.arm_mode_label.setBuddy(self.arm_mode)
        self.server_ip_label.setBuddy(self.kamikaze_latitude)
        self.server_port_label.setBuddy(self.kamikaze_longitude)
#endif // QT_CONFIG(shortcut)

        self.menubar.addAction(self.menuConfigurations.menuAction())
        self.menubar.addAction(self.menuCustomize.menuAction())
        self.menubar.addAction(self.menuTesting.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menuConfigurations.addAction(self.actionConfigurateServer)
        self.menuConfigurations.addAction(self.actionConfigurate_UAV)
        self.menuConfigurations.addAction(self.actionConfigurate_Camera_Stream)
        self.menuHelp.addAction(self.actionAbout)
        self.menuHelp.addAction(self.actionAbout_Qt)
        self.menuTesting.addAction(self.actionForce_Send_Testing_Telemetry_Data)
        self.menuCustomize.addAction(self.actionSet_Colors)
        self.menuCustomize.addAction(self.menuChange_Language.menuAction())

        self.retranslateUi(MainWindow)

        self.fly_mode_combobox.setCurrentIndex(-1)
        self.arm_mode.setCurrentIndex(-1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"UAV Controller", None))
        self.actionConfigurateServer.setText(QCoreApplication.translate("MainWindow", u"Configurate Server", None))
        self.actionAir_Defence_System_Location.setText(QCoreApplication.translate("MainWindow", u"Manage Air Defence System Locations", None))
        self.actionAbout.setText(QCoreApplication.translate("MainWindow", u"About", None))
        self.actionConfigurate_UAV.setText(QCoreApplication.translate("MainWindow", u"Configurate UAV", None))
        self.actionConfigurate_KamikazeCoords.setText(QCoreApplication.translate("MainWindow", u"Configurate Kamikaze Coords", None))
        self.actionForce_Send_Testing_Telemetry_Data.setText(QCoreApplication.translate("MainWindow", u"Force Send Testing Telemetry Data", None))
#if QT_CONFIG(statustip)
        self.actionForce_Send_Testing_Telemetry_Data.setStatusTip(QCoreApplication.translate("MainWindow", u"Start sending telemetry data even if uav not connected", None))
#endif // QT_CONFIG(statustip)
        self.actionSet_Colors.setText(QCoreApplication.translate("MainWindow", u"Theme Editor", None))
        self.actionConfigurate_Camera_Stream.setText(QCoreApplication.translate("MainWindow", u"Configurate Camera Stream", None))
        self.actionAbout_Qt.setText(QCoreApplication.translate("MainWindow", u"About Qt", None))
        self.server_connection_warning.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" font-size:14pt; color:#ff0000;\">Server not Connected</span></p></body></html>", None))
        self.device_connection_warning.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" font-size:14pt; color:#ff0000;\">Device not Connected</span></p></body></html>", None))
        self.fly_mode_label.setText(QCoreApplication.translate("MainWindow", u"Fly Mode:", None))
#if QT_CONFIG(tooltip)
        self.fly_mode_combobox.setToolTip(QCoreApplication.translate("MainWindow", u"Fly Mode", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(statustip)
        self.fly_mode_combobox.setStatusTip(QCoreApplication.translate("MainWindow", u"Fly Mode", None))
#endif // QT_CONFIG(statustip)
        self.arm_mode_label.setText(QCoreApplication.translate("MainWindow", u"Arm Mode:", None))

#if QT_CONFIG(tooltip)
        self.arm_mode.setToolTip(QCoreApplication.translate("MainWindow", u"ARM Status", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(statustip)
        self.arm_mode.setStatusTip(QCoreApplication.translate("MainWindow", u"ARM Status", None))
#endif // QT_CONFIG(statustip)
        self.server_ip_label.setText(QCoreApplication.translate("MainWindow", u"Kamikaze Latitude", None))
        self.server_port_label.setText(QCoreApplication.translate("MainWindow", u"Kamikaze Longitude", None))
        self.get_kamikaze_coords_from_server.setText(QCoreApplication.translate("MainWindow", u"Get Kamikaze Coords From Server", None))
        self.start_kamikaze.setText(QCoreApplication.translate("MainWindow", u"Start Kamikaze", None))
        self.force_end_task.setText(QCoreApplication.translate("MainWindow", u"Force End Task", None))
#if QT_CONFIG(statustip)
        self.set_fence.setStatusTip(QCoreApplication.translate("MainWindow", u"Set Geofence Coordinates", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.set_fence.setWhatsThis(QCoreApplication.translate("MainWindow", u"Set Geofence Coordinates", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(statustip)
        self.add_ads.setStatusTip(QCoreApplication.translate("MainWindow", u"Add a new ADS", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.add_ads.setWhatsThis(QCoreApplication.translate("MainWindow", u"Add a new ADS", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(statustip)
        self.remove_ads.setStatusTip(QCoreApplication.translate("MainWindow", u"Remove selected ADS", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.remove_ads.setWhatsThis(QCoreApplication.translate("MainWindow", u"Remove selected ADS", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(statustip)
        self.refresh_ads.setStatusTip(QCoreApplication.translate("MainWindow", u"Refresh ADS from server", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.refresh_ads.setWhatsThis(QCoreApplication.translate("MainWindow", u"Refresh ADS from server", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(statustip)
        self.download_fence_data.setStatusTip(QCoreApplication.translate("MainWindow", u"Download Fence", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.download_fence_data.setWhatsThis(QCoreApplication.translate("MainWindow", u"Download Fence", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(statustip)
        self.download_missions.setStatusTip(QCoreApplication.translate("MainWindow", u"Download Missions From UAV", None))
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        self.download_missions.setWhatsThis(QCoreApplication.translate("MainWindow", u"Download Missions From UAV", None))
#endif // QT_CONFIG(whatsthis)
        self.menuConfigurations.setTitle(QCoreApplication.translate("MainWindow", u"Configurations", None))
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"Help", None))
        self.menuTesting.setTitle(QCoreApplication.translate("MainWindow", u"Testing", None))
        self.menuCustomize.setTitle(QCoreApplication.translate("MainWindow", u"Customize", None))
        self.menuChange_Language.setTitle(QCoreApplication.translate("MainWindow", u"Change Language", None))
    # retranslateUi

