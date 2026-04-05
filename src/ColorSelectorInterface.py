from typing import Callable

from PySide6.QtCore import qInfo
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QDialog, QColorDialog, QToolButton
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
background-color: black;
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
border: 2px solid rgb(45, 45, 45);
background: rgb(45, 45, 45);
selection-background-color: darkgray;
}}
"""


class ColorSelectorInterface(QDialog):
    ui: Ui_ColorSelector
    savedOptions: ColorOptions

    def __init__(self, parent: QWidget | None = None):
        QDialog.__init__(self, parent)
        self.savedOptions = DEFAULT_COLORS
        self.ui = Ui_ColorSelector()
        self.ui.setupUi(self)
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

    def openColorSelector(self, button: QToolButton, selected_color_as_text: str, set_color_function: Callable[[QColor], None]) -> None:
        d = QColorDialog.getColor(initial=QColor(selected_color_as_text), parent=self, title="Choose Color")
        if d.isValid():
            qInfo("Color selected %s" % d)
            button.setStyleSheet(f"background-color: {d.name()};")
            button.setText(d.name())
            set_color_function(d)
        window: QWidget = button.parent().parent()
        window.setStyleSheet(create_new_stylesheet(self.savedOptions))


    def loadColors(self, options: ColorOptions):
        self.savedOptions = options
        self.ui.main_window_background.setText(options.main_window_background.value())
        self.ui.main_window_alternate_background.setText(options.main_window_alternate_background.value())
        self.ui.splitter.setText(options.splitter.value())
        self.ui.main_window_frame.setText(options.main_window_frame.value())
        self.ui.menu_item.setText(options.menu_item.value())
        self.ui.menu_selected_item_color.setText(options.menu_selected_item_color.value())
        self.ui.button.setText(options.button.value())
        self.ui.selected_button.setText(options.selected_button.value())
        self.ui.hovered_button.setText(options.hovered_button.value())
        self.ui.button_background.setText(options.button_background.value())
        self.ui.selected_button_background.setText(options.selected_button_background.value())
        self.ui.hovered_button_background.setText(options.hovered_button_background.value())
        self.ui.combo_box.setText(options.combo_box.value())
        self.ui.text_box_border.setText(options.text_box_border.value())
        self.ui.text_box_background.setText(options.text_box_background.value())
        self.ui.combo_box_background.setText(options.combo_box_background.value())

    def saveColors(self):
        self.savedOptions.main_window_background = QColor(self.ui.main_window_background.text())
        self.savedOptions.main_window_alternate_background = QColor(self.ui.main_window_alternate_background.text())
        self.savedOptions.splitter = QColor(self.ui.splitter.text())
        self.savedOptions.main_window_frame = QColor(self.ui.main_window_frame.text())
        self.savedOptions.menu_item = QColor(self.ui.menu_item.text())
        self.savedOptions.menu_selected_item_color = QColor(self.ui.menu_selected_item_color.text())
        self.savedOptions.button = QColor(self.ui.button.text())
        self.savedOptions.selected_button = QColor(self.ui.selected_button.text())
        self.savedOptions.hovered_button = QColor(self.ui.hovered_button.text())
        self.savedOptions.button_background = QColor(self.ui.button_background.text())
        self.savedOptions.selected_button_background = QColor(self.ui.selected_button_background.text())
        self.savedOptions.hovered_button_background = QColor(self.ui.hovered_button_background.text())
        self.savedOptions.combo_box = QColor(self.ui.combo_box.text())
        self.savedOptions.text_box_border = QColor(self.ui.text_box_border.text())
        self.savedOptions.text_box_background = QColor(self.ui.text_box_background.text())
        self.savedOptions.combo_box_background = QColor(self.ui.combo_box_background.text())

