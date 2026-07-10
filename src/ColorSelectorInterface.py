from enum import Enum
from typing import Callable

from PySide6.QtCore import qInfo, QSize, QCoreApplication, qWarning
from PySide6.QtGui import QColor, QIcon, QColorConstants
from PySide6.QtWidgets import QDialog, QColorDialog, QToolButton, QAbstractButton, QDialogButtonBox, QFormLayout, \
    QHBoxLayout, QLabel, QPushButton, QComboBox
from PySide6.QtWidgets import QWidget

class ColorOptionEnumImpl:
    @staticmethod
    def get_config(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]], ea: ColorOptionEnum, attribute: ColorAttribute) -> str:
        for e in ea.value[2]:
            if e.attribute == attribute:
                return ColorSelectorInterface.get_config(d, ea, attribute, e.color).name()
        qWarning("Unknown attribute, probably bug in %s %s" % (ea.value[0], attribute.value[0]))
        return None

    @staticmethod
    def create_main_window(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.MainWindow
        return """
QWidget#centralwidget, QWidget#remaining_thingies_frame, QDialog
{
color: %s;
background-color: %s;
alternate-background-color: %s;
selection-background-color: #3daee9;
selection-color: #eff0f1;
}

QLabel {
color: %s;
}""" % (ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Frame),
               ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Background),
               ColorOptionEnumImpl.get_config(d, e, ColorAttribute.AlternateBackground),
               ColorOptionEnumImpl.get_config(d, e, ColorAttribute.TextColor))

    @staticmethod
    def create_splitter(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.Splitter
        return """
QSplitterHandle
{
border: 1px dotted darkgrey;
background: transparent;
color: %s;
height: 1px;
}
""" % (ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Color))

    @staticmethod
    def create_menu_item(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.MenuItem
        return """
QMenu::item:selected {{
color: {2};
background: {0};
}}

QMenu::item:pressed {{
color: {2};
background: {1};
}}

QMenuBar::item {{
color: {2};
background: transparent;
padding: 1px 4px;
border-radius: 4px;
}}

QMenuBar::item:selected {{
background: {0};
}}

QMenuBar::item:pressed {{
background: {1};
}}

QMenuBar {{
color: {2};
background-color: rgb(29, 34, 38);
spacing: 3px;
}}

QMenu {{
color: {2};
background: rgb(29, 34, 38);
}}

QStatusBar {{
background: rgb(29, 34, 38);
}}
""".format(ColorOptionEnumImpl.get_config(d, e, ColorAttribute.HoveredBackground),
           ColorOptionEnumImpl.get_config(d, e, ColorAttribute.SelectedBackground),
           ColorOptionEnumImpl.get_config(d, e, ColorAttribute.TextColor))

    @staticmethod
    def create_button(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.Button
        return """
QAbstractButton {
color: %s; 
background-color: %s;
selection-background-color: rgb(63, 67, 71);
}

QAbstractButton:pressed {
color: %s; 
background-color: %s;
}

QAbstractButton:checked {
color: %s; 
background-color: %s;
}
""" % (ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Color),
                   ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Background),
                   ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Selected),
                   ColorOptionEnumImpl.get_config(d, e, ColorAttribute.SelectedBackground),
                   ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Hovered),
                   ColorOptionEnumImpl.get_config(d, e, ColorAttribute.HoveredBackground))

    @staticmethod
    def create_combobox(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.ComboBox
        return """
QComboBox {{
color: {0}; 
background-color: {1};
padding: 1px 18px 1px 3px;
min-width: 6em;
}}

QComboBox QAbstractItemView {{
color: {0}; 
background-color: {1};
}}
""".format(ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Color), ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Background))

    @staticmethod
    def create_textbox(d: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]) -> str:
        e = ColorOptionEnum.TextBox
        return """
QLineEdit {{
color: {};
border: 2px solid {};
background-color: {};
selection-background-color: {};
}}
""".format(ColorOptionEnumImpl.get_config(d, e, ColorAttribute.TextColor),
           ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Border),
           ColorOptionEnumImpl.get_config(d, e, ColorAttribute.Background),
           ColorOptionEnumImpl.get_config(d, e, ColorAttribute.SelectedBackground))


class ColorAttribute(Enum):
    Color = (0, lambda: QCoreApplication.translate("ColorAttribute", "Color", None))
    AlternateColor = (1, lambda: QCoreApplication.translate("ColorAttribute", "Alternate Color", None))
    Background = (2, lambda: QCoreApplication.translate("ColorAttribute", "Background Color", None))
    AlternateBackground = (3, lambda: QCoreApplication.translate("ColorAttribute", "Background Alternate Color", None))
    Hovered = (4, lambda: QCoreApplication.translate("ColorAttribute", "Hovered Color", None))
    HoveredBackground = (5, lambda: QCoreApplication.translate("ColorAttribute", "Hovered Background Color", None))
    Selected = (6, lambda: QCoreApplication.translate("ColorAttribute", "Selected Color", None))
    SelectedBackground = (7, lambda: QCoreApplication.translate("ColorAttribute", "Selected Background Color", None))
    Border = (8, lambda: QCoreApplication.translate("ColorAttribute", "Border Color", None))
    Frame = (9, lambda: QCoreApplication.translate("ColorAttribute", "Frame Color", None))
    TextColor = (10, lambda: QCoreApplication.translate("ColorAttribute", "Text Color", None))

class ColorOption:
    attribute: ColorAttribute
    color: QColor

    def __init__(self, attribute: ColorAttribute, color: QColor):
        self.attribute = attribute
        self.color = color

class ColorOptionEnum(Enum):
    MainWindow = (0, lambda: QCoreApplication.translate("ColorOptionEnum", "Main Window", None), [ColorOption(ColorAttribute.Background, QColor(32, 35, 38)), ColorOption(ColorAttribute.AlternateBackground, QColor(41, 44, 48)), ColorOption(ColorAttribute.Frame, QColor(239, 240, 241)), ColorOption(ColorAttribute.TextColor, QColorConstants.White)], ColorOptionEnumImpl.create_main_window)
    Splitter = (1, lambda: QCoreApplication.translate("ColorOptionEnum", "Splitter", None), [ColorOption(ColorAttribute.Color, QColor(8, 56, 56))], ColorOptionEnumImpl.create_splitter)
    MenuItem = (2, lambda: QCoreApplication.translate("ColorOptionEnum", "Menu Item", None), [ColorOption(ColorAttribute.SelectedBackground, QColor(136, 136, 136)), ColorOption(ColorAttribute.HoveredBackground, QColor(168, 168, 168)), ColorOption(ColorAttribute.TextColor, QColorConstants.White)], ColorOptionEnumImpl.create_menu_item)
    Button = (3, lambda: QCoreApplication.translate("ColorOptionEnum", "Button", None), [ColorOption(ColorAttribute.Color, QColor(239, 240, 241)), ColorOption(ColorAttribute.Background, QColor(45, 48, 51)), ColorOption(ColorAttribute.Selected, QColor(239, 240, 241)), ColorOption(ColorAttribute.SelectedBackground, QColor(67, 72, 76)), ColorOption(ColorAttribute.Hovered, QColor(239, 240, 241)), ColorOption(ColorAttribute.HoveredBackground, QColor(67, 72, 76))], ColorOptionEnumImpl.create_button)
    ComboBox = (4, lambda: QCoreApplication.translate("ColorOptionEnum", "Combo Box", None), [ColorOption(ColorAttribute.Color, QColor(239, 240, 241)), ColorOption(ColorAttribute.Background, QColor(45, 48, 51))], ColorOptionEnumImpl.create_combobox)
    TextBox = (5, lambda: QCoreApplication.translate("ColorOptionEnum", "Text Box", None), [ColorOption(ColorAttribute.Border, QColor(45, 45, 45)), ColorOption(ColorAttribute.TextColor, QColorConstants.White), ColorOption(ColorAttribute.SelectedBackground, QColor(169, 169, 169)), ColorOption(ColorAttribute.Background, QColor(45, 45, 45))], ColorOptionEnumImpl.create_textbox)

class ColorOptions:
    config: dict[ColorOptionEnum, dict[ColorAttribute, QColor]]
    icon_color: int

    def __init__(self):
        self.icon_color = 0
        self.config = dict()

    @staticmethod
    def create_copy(copy: ColorOptions):
        self: ColorOptions = ColorOptions()
        self.icon_color = copy.icon_color
        for e in copy.config:
            self.config[e] = copy.config[e].copy()
        return self

class Ui_Fake:
    def retranslateUi(self, parent: ColorSelectorInterface):
        parent.icon_theme_label.setText(QCoreApplication.translate("ColorSelector", u"Icon Theme", None))
        parent.icon_theme.setItemText(0, QCoreApplication.translate("ColorSelector", u"Light", None))
        parent.icon_theme.setItemText(1, QCoreApplication.translate("ColorSelector", u"Dark", None))
        for e in parent.color_option_submenu_tuples.keys():
            t = parent.color_option_submenu_tuples[e]
            t[0].setText(e.value[1]())
            t[1].setText(QCoreApplication.translate("ColorSelectorInterface", "Customize", None))
        for k in parent.dialogs.keys():
            v = parent.dialogs[k]
            v.retranslateUi()
        parent.setWindowTitle(QCoreApplication.translate("ColorSelectorInterface", "Theme Editor", None))

class SubMenuDialog(QDialog):
    att2child: dict[ColorAttribute, QLabel]
    _parent: ColorSelectorInterface
    coe: ColorOptionEnum
    def __init__(self, parent: ColorSelectorInterface, coe: ColorOptionEnum):
        super().__init__(parent)
        self.setWindowTitle(QCoreApplication.translate("ColorSelectorInterface", "Customize {}", None).format(coe.value[1]()))
        self._parent = parent
        self.coe = coe
        self.att2child = dict()
        self.formLayout = QFormLayout(self)
        self.formLayout.setObjectName("main_layout")
        for i, e in enumerate(coe.value[2]):
            attribute: ColorAttribute = e.attribute
            color = parent.get_config(parent.savedOptions.config, coe, attribute, e.color)
            hl = QHBoxLayout()
            label = QLabel(self)
            label.setText(attribute.value[1]())
            button = QToolButton(self)
            button.clicked.connect(lambda b, button=button, c=color, a=attribute: self._openColorSelector(button, c, lambda nc, a=a: self._parent.update_config(coe, a, nc)))
            ColorSelectorInterface.change_button_to_match_its_option(button, color.name())
            hl.addWidget(label)
            hl.addWidget(button)
            self.formLayout.setLayout(i, QFormLayout.ItemRole.SpanningRole, hl)
            self.att2child[attribute] = label

    def retranslateUi(self):
        for key in self.att2child.keys():
            t = self.att2child[key]
            t.setText(key.value[1]())
        self.setWindowTitle(QCoreApplication.translate("ColorSelectorInterface", "Customize {}", None).format(self.coe.value[1]()))

    def _openColorSelector(self, button: QToolButton, selected_color_as_text: QColor, set_color_function: Callable[[QColor], None]) -> None:
        d = QColorDialog.getColor(initial=selected_color_as_text, parent=self, title="Choose Color")
        if d.isValid():
            qInfo("Color selected %s" % d)
            ColorSelectorInterface.change_button_to_match_its_option(button, d.name())
            set_color_function(d)
        window: QWidget = self._parent.parent()
        self._parent.updateStyleSheet(window)


class ColorSelectorInterface(QDialog):
    ui: Ui_Fake # I am lazy to recreate anything
    color_option_submenu_tuples = dict[ColorOptionEnum, tuple[QLabel, QPushButton]]
    dialogs: dict[ColorOptionEnum, SubMenuDialog]
    savedOptions: ColorOptions

    def __init__(self, parent: QWidget | None, savedOptions: ColorOptions):
        QDialog.__init__(self, parent)
        self.setWindowTitle(QCoreApplication.translate("ColorSelectorInterface", "Theme Editor", None))
        self.savedOptions = savedOptions
        self.color_option_submenu_tuples = dict()
        self.dialogs = dict()
        self.resize(561, self.size().height())
        self.ui = Ui_Fake()
        self.formLayout = QFormLayout(self)
        self.formLayout.setObjectName("main_layout")
        e: ColorOptionEnum
        for e in ColorOptionEnum:
            hl = QHBoxLayout()
            hl.setObjectName(e.name + "_horizontal_layout")
            label = QLabel(self)
            label.setObjectName(e.name + "_label")
            label.setText(e.value[1]())
            button = QPushButton(self)
            button.setObjectName(e.name + "_button")
            button.setText(QCoreApplication.translate("ColorSelectorInterface", "Customize", None))
            button.clicked.connect(lambda b, e=e: self._create_ui(e))
            hl.addWidget(label)
            hl.addWidget(button)
            self.formLayout.setLayout(e.value[0], QFormLayout.ItemRole.SpanningRole, hl)
            self.color_option_submenu_tuples[e] = (label, button)

        hl_icontheme = QHBoxLayout()
        hl_icontheme.setObjectName("hl_icontheme")
        self.icon_theme_label = QLabel(self)
        self.icon_theme_label.setObjectName(u"icon_theme_label")

        hl_icontheme.addWidget(self.icon_theme_label)

        self.icon_theme = QComboBox(self)
        self.icon_theme.addItem("")
        self.icon_theme.addItem("")
        self.icon_theme.setObjectName(u"icon_theme")
        self.icon_theme.setCurrentIndex(self.savedOptions.icon_color)

        hl_icontheme.addWidget(self.icon_theme)

        self.formLayout.setLayout(e.value[0] + 1, QFormLayout.ItemRole.SpanningRole, hl_icontheme)

        self.buttons = QDialogButtonBox(self)
        self.buttons.setObjectName(u"buttons")
        self.buttons.setStandardButtons(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.RestoreDefaults)

        self.formLayout.setWidget(e.value[0] + 2, QFormLayout.ItemRole.LabelRole, self.buttons)
        self.ui.retranslateUi(self)

        self.icon_theme.currentIndexChanged.connect(self.changeIconColors)
        self.buttons.button(QDialogButtonBox.StandardButton.RestoreDefaults).clicked.connect(self.reset_colors)
        self.buttons.accepted.connect(self.accepted)

    def _create_ui(self, coe: ColorOptionEnum):
        if coe in self.dialogs:
            return
        dialog = SubMenuDialog(self, coe)
        dialog.show()
        self.dialogs[coe] = dialog
        dialog.finished.connect(lambda: self.dialogs.pop(coe))

    def update_config(self, a: ColorOptionEnum, b: ColorAttribute, c: QColor):
        if not a in self.savedOptions.config:
            self.savedOptions.config[a] = dict()
        self.savedOptions.config[a][b] = c

    def get_dialogs(self) -> list[SubMenuDialog]:
        return list(self.dialogs.values())

    @staticmethod
    def get_config(c: dict[ColorOptionEnum, dict[ColorAttribute, QColor]], a: ColorOptionEnum, b: ColorAttribute, default: QColor) -> QColor:
        if a in c:
            if b in c[a]:
                return c[a][b]
        return default

    def reset_colors(self):
        self.savedOptions = ColorOptions()
        window: QWidget = self.parent()
        self.updateStyleSheet(window)
        self.update_icon_colors(self.savedOptions.icon_color)
        self.icon_theme.setCurrentIndex(self.savedOptions.icon_color)

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
        for d in self.get_dialogs():
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

    @staticmethod
    def change_button_to_match_its_option(button: QAbstractButton, color_name: str):
        button.setStyleSheet(f"background-color: {color_name};")
        button.setText(color_name)

    @staticmethod
    def create_stylesheet(options: ColorOptions) -> str:
        styleSheet: str = ""
        for e in ColorOptionEnum:
            styleSheet += e.value[3](options.config)
        styleSheet += """
        QTabBar::tab {
        color: #eff0f1; 
        background-color: rgb(30, 30, 30);
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        min-width: 8ex;
        padding: 2px;
        }

        QTabBar::tab:selected, QTabBar::tab:hover {
        color: #eff0f1; 
        background-color: rgb(45, 45, 45);
        }

        QTabWidget::pane {
        border: 1px solid rgb(41, 44, 47);
        }
        """
        return styleSheet

    def updateStyleSheet(self, window: QWidget):
        window.setStyleSheet(ColorSelectorInterface.create_stylesheet(self.savedOptions))
