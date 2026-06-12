from typing import Callable

from PySide6.QtCore import qInfo, QSize
from PySide6.QtGui import QColor, QPalette, QIcon
from PySide6.QtWidgets import QDialog, QColorDialog, QToolButton, QAbstractButton, QDialogButtonBox
from PySide6.QtWidgets import QWidget

from ui_files_python.color_selector import Ui_ColorSelector

class ColorOptions:
    main_window_background: QColor
    main_window_alternate_background: QColor
    splitter: QColor
    main_window_frame: QColor
    menu_item: QColor
    menu_selected_item_color: QColor
    button: QColor
    selected_button: QColor
    hovered_button: QColor
    button_background: QColor
    selected_button_background: QColor
    hovered_button_background: QColor
    combo_box: QColor
    text_box_border: QColor
    text_box_background: QColor
    combo_box_background: QColor
    icon_color: int

    def __init__(self):
        pass

    @staticmethod
    def create_copy(copy: ColorOptions):
        self: ColorOptions = ColorOptions()
        self.main_window_background = copy.main_window_background
        self.main_window_alternate_background = copy.main_window_alternate_background
        self.splitter = copy.splitter
        self.main_window_frame = copy.main_window_frame
        self.menu_item = copy.menu_item
        self.menu_selected_item_color = copy.menu_selected_item_color
        self.button = copy.button
        self.selected_button = copy.selected_button
        self.hovered_button = copy.hovered_button
        self.button_background = copy.button_background
        self.selected_button_background = copy.selected_button_background
        self.hovered_button_background = copy.hovered_button_background
        self.combo_box = copy.combo_box
        self.text_box_border = copy.text_box_border
        self.text_box_background = copy.text_box_background
        self.combo_box_background = copy.combo_box_background
        self.icon_color = copy.icon_color
        return self


DEFAULT_COLORS: ColorOptions = ColorOptions()
DEFAULT_COLORS.main_window_frame = QColor("#eff0f1")
DEFAULT_COLORS.main_window_background = QColor(32, 35, 38)
DEFAULT_COLORS.main_window_alternate_background = QColor(41,44,48)
DEFAULT_COLORS.splitter = QColor(8, 56, 56)
DEFAULT_COLORS.menu_selected_item_color = QColor("#a8a8a8")
DEFAULT_COLORS.button = QColor("#eff0f1")
DEFAULT_COLORS.button_background = QColor(45, 48, 51)
DEFAULT_COLORS.selected_button = QColor("#eff0f1")
DEFAULT_COLORS.selected_button_background = QColor(67, 72, 76)
DEFAULT_COLORS.hovered_button = QColor("#eff0f1")
DEFAULT_COLORS.hovered_button_background = QColor(67, 72, 76)
DEFAULT_COLORS.combo_box = QColor("#eff0f1")
DEFAULT_COLORS.combo_box_background = QColor(45, 48, 51)
DEFAULT_COLORS.menu_item = QColor(0, 0, 0)
DEFAULT_COLORS.text_box_border = QColor(45, 45, 45)
DEFAULT_COLORS.text_box_background = QColor(45, 45, 45)
DEFAULT_COLORS.icon_color = 0

def create_new_stylesheet(options: ColorOptions):
    return f"""
QWidget#centralwidget, QWidget#remaining_thingies_frame, QWidget#kamikaze_panel, QWidget#target_tracking_panel
{{
color: {options.main_window_frame.name()};
background-color: {options.main_window_background.name()};
alternate-background-color: {options.main_window_alternate_background.name()};
selection-background-color: #3daee9;
selection-color: #eff0f1;
}}

QSplitter::handle:horizontal {{
border: 1px dotted darkgrey;
color: {options.splitter.name()};
height: 1px;
}}

QSplitter::handle:vertical {{
border: 1px dotted darkgrey;
color: {options.splitter.name()};
height: 1px;
}}

QMenuBar {{
background-color: rgb(29, 34, 38);
spacing: 3px;
}}

QMenu {{
background: rgb(29, 34, 38);
}}

QMenu::item:selected {{
background: {options.menu_selected_item_color.name()};
}}

QMenu::item:pressed {{
background: #888888;
}}

QStatusBar {{
background: rgb(29, 34, 38);
}}

QMenuBar::item {{
background-color: {options.menu_item.name()};
padding: 1px 4px;
background: transparent;
border-radius: 4px;
}}

QMenuBar::item:selected {{
background: {options.menu_selected_item_color.name()};
}}

QMenuBar::item:pressed {{
background: #888888;
}}

QAbstractButton {{
color: {options.button.name()}; 
background-color: {options.button_background.name()};
selection-background-color: rgb(63, 67, 71);
}}

QAbstractButton:pressed {{
color: {options.selected_button.name()}; 
background-color: {options.selected_button_background.name()};
}}

QAbstractButton:checked {{
color: {options.hovered_button.name()}; 
background-color: {options.hovered_button_background.name()};
}}

QComboBox {{
color: {options.combo_box.name()}; 
background-color: {options.combo_box_background.name()};
padding: 1px 18px 1px 3px;
min-width: 6em;
}}

QComboBox QAbstractItemView {{
color: {options.combo_box.name()}; 
background-color: {options.combo_box_background.name()};
}}

QTabBar::tab {{
color: #eff0f1; 
background-color: rgb(30, 30, 30);
border-top-left-radius: 4px;
border-top-right-radius: 4px;
min-width: 8ex;
padding: 2px;
}}

QTabBar::tab:selected, QTabBar::tab:hover {{
color: #eff0f1; 
background-color: rgb(45, 45, 45);
}}

QTabWidget::pane {{
border: 1px solid rgb(41, 44, 47);
}}

QLineEdit {{
border: 2px solid {options.text_box_border.name()}
background: {options.text_box_background.name()};
selection-background-color: darkgray;
}}
"""


class ColorSelectorInterface(QDialog):
    ui: Ui_ColorSelector
    savedOptions: ColorOptions

    def __init__(self, parent: QWidget | None, savedOptions: ColorOptions):
        QDialog.__init__(self, parent)
        self.savedOptions = savedOptions
        self.ui = Ui_ColorSelector()
        self.ui.setupUi(self)
        self.reloadButtonColors()
        self.ui.main_window_background.clicked.connect(lambda b: self.openColorSelector(self.ui.main_window_background, self.ui.main_window_background.text(), lambda c : (setattr(self.savedOptions, "main_window_background", c))))
        self.ui.main_window_alternate_background.clicked.connect(lambda b: self.openColorSelector(self.ui.main_window_alternate_background, self.ui.main_window_alternate_background.text(), lambda c: (setattr(self.savedOptions, "main_window_alternate_background", c))))
        self.ui.splitter.clicked.connect(lambda b: self.openColorSelector(self.ui.splitter, self.ui.splitter.text(), lambda c: (setattr(self.savedOptions, "splitter", c))))
        self.ui.main_window_frame.clicked.connect(lambda b: self.openColorSelector(self.ui.main_window_frame, self.ui.main_window_frame.text(), lambda c: (setattr(self.savedOptions, "main_window_frame", c))))
        self.ui.menu_item.clicked.connect(lambda b: self.openColorSelector(self.ui.menu_item, self.ui.menu_item.text(), lambda c: (setattr(self.savedOptions, "menu_item", c))))
        self.ui.menu_selected_item_color.clicked.connect(lambda b: self.openColorSelector(self.ui.menu_selected_item_color, self.ui.menu_selected_item_color.text(), lambda c: (setattr(self.savedOptions, "menu_selected_item_color", c))))
        self.ui.button.clicked.connect(lambda b: self.openColorSelector(self.ui.button, self.ui.button.text(), lambda c: (setattr(self.savedOptions, "button", c))))
        self.ui.selected_button.clicked.connect(lambda b: self.openColorSelector(self.ui.selected_button, self.ui.selected_button.text(), lambda c: (setattr(self.savedOptions, "selected_button", c))))
        self.ui.hovered_button.clicked.connect(lambda b: self.openColorSelector(self.ui.hovered_button, self.ui.hovered_button.text(), lambda c: (setattr(self.savedOptions, "hovered_button", c))))
        self.ui.button_background.clicked.connect(lambda b: self.openColorSelector(self.ui.button_background, self.ui.button_background.text(), lambda c: (setattr(self.savedOptions, "button_background", c))))
        self.ui.selected_button_background.clicked.connect(lambda b: self.openColorSelector(self.ui.selected_button_background, self.ui.selected_button_background.text(), lambda c: (setattr(self.savedOptions, "selected_button_background", c))))
        self.ui.hovered_button_background.clicked.connect(lambda b: self.openColorSelector(self.ui.hovered_button_background, self.ui.hovered_button_background.text(), lambda c: (setattr(self.savedOptions, "hovered_button_background", c))))
        self.ui.combo_box.clicked.connect(lambda b: self.openColorSelector(self.ui.combo_box, self.ui.combo_box.text(), lambda c: (setattr(self.savedOptions, "combo_box", c))))
        self.ui.text_box_border.clicked.connect(lambda b: self.openColorSelector(self.ui.text_box_border, self.ui.text_box_border.text(), lambda c: (setattr(self.savedOptions, "text_box_border", c))))
        self.ui.text_box_background.clicked.connect(lambda b: self.openColorSelector(self.ui.text_box_background, self.ui.text_box_background.text(), lambda c: (setattr(self.savedOptions, "text_box_background", c))))
        self.ui.combo_box_background.clicked.connect(lambda b: self.openColorSelector(self.ui.combo_box_background, self.ui.combo_box_background.text(), lambda c: (setattr(self.savedOptions, "combo_box_background", c))))
        self.ui.icon_theme.currentIndexChanged.connect(self.changeIconColors)
        self.ui.buttons.button(QDialogButtonBox.StandardButton.RestoreDefaults).clicked.connect(self.reset_colors)

    def reset_colors(self):
        self.savedOptions = ColorOptions.create_copy(DEFAULT_COLORS)
        window: QWidget = self.parent()
        self.updateStyleSheet(window)
        self.update_icon_colors(DEFAULT_COLORS.icon_color)
        self.reloadButtonColors()

    def changeIconColors(self, index: int) -> None:
        self.savedOptions.icon_color = index
        self.update_icon_colors(index)

    def update_icon_colors(self, index: int) -> None:
        window: QWidget = self.parent()
        self.changeColorOfButtonWithoutState("fence", window.ui.set_fence, index)
        self.changeColorOfButtonWithoutState("download_fence", window.ui.download_fence_data, index)
        self.changeColorOfButtonWithoutState("download_mission", window.ui.download_missions, index)
        self.changeColorOfButtonWithoutState("refresh", window.ui.refresh_ads, index)
        self.changeColorOfButtonWithoutState("plus", window.ui.add_to_watch, index)
        self.changeColorOfButtonWithoutState("plus", window.ui.add_ads, index)
        self.changeColorOfButtonWithoutState("minus", window.ui.remove_from_watch, index)
        self.changeColorOfButtonWithoutState("minus", window.ui.remove_ads, index)
        self.changeColorOfButtonWithOnOffState("start", "stop", window.ui.start_stop_camera_view, index)
        self.changeColorOfMainIcon("ares", window, index)
        for d in window.get_all_dialogs():
            if d is not None:
                self.changeColorOfMainIcon("ares", d, index)

    def changeColorOfButtonWithoutState(self, icon_name: str, button: QAbstractButton, color: int):
        color_suffix = "white" if color == 0 else "black"
        icon = QIcon()
        icon.addFile("ui_files/" + icon_name + "-" + color_suffix + ".svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        button.setIcon(icon)

    def changeColorOfButtonWithOnOffState(self, icon_name_on: str, icon_name_off: str, button: QAbstractButton, color: int):
        color_suffix = "white" if color == 0 else "black"
        icon = QIcon()
        icon.addFile("ui_files/" + icon_name_off + "-" + color_suffix + ".svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        icon.addFile("ui_files/" + icon_name_on + "-" + color_suffix + ".svg", QSize(), QIcon.Mode.Normal, QIcon.State.On)
        button.setIcon(icon)

    def changeColorOfMainIcon(self, icon_name: str, widget: QWidget, color: int):
        color_suffix = "white" if color == 0 else "black"
        icon = QIcon()
        icon.addFile("ui_files/" + icon_name + "-" + color_suffix + ".svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        widget.setWindowIcon(icon)

    def change_button_to_match_its_option(self, button: QAbstractButton, color_name: str):
        button.setStyleSheet(f"background-color: {color_name};")
        button.setText(color_name)

    def openColorSelector(self, button: QToolButton, selected_color_as_text: str, set_color_function: Callable[[QColor], None]) -> None:
        d = QColorDialog.getColor(initial=QColor(selected_color_as_text), parent=self, title="Choose Color")
        if d.isValid():
            qInfo("Color selected %s" % d)
            self.change_button_to_match_its_option(button, d.name())
            set_color_function(d)
        window: QWidget = self.parent()
        self.updateStyleSheet(window)

    def updateStyleSheet(self, window: QWidget):
        window.setStyleSheet(create_new_stylesheet(self.savedOptions))


    def reloadButtonColors(self):
        self.change_button_to_match_its_option(self.ui.main_window_background, self.savedOptions.main_window_background.name())
        self.change_button_to_match_its_option(self.ui.main_window_alternate_background, self.savedOptions.main_window_alternate_background.name())
        self.change_button_to_match_its_option(self.ui.splitter, self.savedOptions.splitter.name())
        self.change_button_to_match_its_option(self.ui.main_window_frame, self.savedOptions.main_window_frame.name())
        self.change_button_to_match_its_option(self.ui.menu_item, self.savedOptions.menu_item.name())
        self.change_button_to_match_its_option(self.ui.menu_selected_item_color, self.savedOptions.menu_selected_item_color.name())
        self.change_button_to_match_its_option(self.ui.button, self.savedOptions.button.name())
        self.change_button_to_match_its_option(self.ui.selected_button, self.savedOptions.selected_button.name())
        self.change_button_to_match_its_option(self.ui.hovered_button, self.savedOptions.hovered_button.name())
        self.change_button_to_match_its_option(self.ui.button_background, self.savedOptions.button_background.name())
        self.change_button_to_match_its_option(self.ui.selected_button_background, self.savedOptions.selected_button_background.name())
        self.change_button_to_match_its_option(self.ui.hovered_button_background, self.savedOptions.hovered_button_background.name())
        self.change_button_to_match_its_option(self.ui.combo_box, self.savedOptions.combo_box.name())
        self.change_button_to_match_its_option(self.ui.text_box_border, self.savedOptions.text_box_border.name())
        self.change_button_to_match_its_option(self.ui.text_box_background, self.savedOptions.text_box_background.name())
        self.change_button_to_match_its_option(self.ui.combo_box_background, self.savedOptions.combo_box_background.name())
        self.ui.icon_theme.setCurrentIndex(self.savedOptions.icon_color)
