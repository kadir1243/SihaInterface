from typing import Callable

from PySide6.QtCore import Qt, qInfo, QCoreApplication, QKeyCombination, qWarning
from PySide6.QtGui import QKeyEvent, QMouseEvent, QKeySequence
from PySide6.QtWidgets import QDialog, QToolButton, QDialogButtonBox
from PySide6.QtWidgets import QWidget

from src.input_types import KeybindingsEnum, InputMapping
from ui_files_python.input_selector import Ui_KeybindingConfig

MouseButtonTranslation: dict[int, Callable[[], str]] = {
    Qt.MouseButton.LeftButton.value: lambda: QCoreApplication.translate("InputMapper", "Left Mouse", None),
    Qt.MouseButton.RightButton.value: lambda: QCoreApplication.translate("InputMapper", "Right Mouse", None),
    Qt.MouseButton.MiddleButton.value: lambda: QCoreApplication.translate("InputMapper", "Middle Mouse", None),
    Qt.MouseButton.BackButton.value: lambda: QCoreApplication.translate("InputMapper", "Back Mouse", None),
    Qt.MouseButton.ForwardButton.value: lambda: QCoreApplication.translate("InputMapper", "Forward Mouse", None),
    Qt.MouseButton.TaskButton.value: lambda: QCoreApplication.translate("InputMapper", "Task Mouse", None)
    # There is actually more buttons exists, but idc them (also I don't have any way to test them)
}

def _translate_mouse_keys(i: Qt.MouseButton, mod: Qt.KeyboardModifier) -> str:
    out: str = ""

    s: str = QKeySequence(QKeyCombination(mod, Qt.Key.Key_A)).toString()
    out += s[:len(s)-1] # basically add mods to text with key 'A' than delete 'A' part
    skip_first_plus: bool = len(s) > 0

    for key in MouseButtonTranslation.keys():
        if (i.value & key) != 0:
            if len(out) != 0:
                if skip_first_plus:
                    skip_first_plus = False
                else:
                    out += "+"
            out += MouseButtonTranslation[key]()

    if out.endswith("+"):
        qWarning("Something is broken in translation: %s %s returned %s" % (i, mod, out))
        out = "" # Seems like something broke and we only got modifiers

    return out

class KeybindingConfigInterface(QDialog):
    original_config: dict[KeybindingsEnum, InputMapping]
    config: dict[KeybindingsEnum, InputMapping]
    ui: Ui_KeybindingConfig
    currently_waiting: KeybindingsEnum | None
    input_type2button: dict[KeybindingsEnum, QToolButton]

    def __init__(self, parent: QWidget, config: dict[KeybindingsEnum, InputMapping]):
        QDialog.__init__(self, parent)
        self.ui = Ui_KeybindingConfig()
        self.ui.setupUi(self)
        self.original_config = config
        self.config = {k: v for k, v in config.items()}

        self.input_type2button = {
            KeybindingsEnum.add_geofence_point: self.ui.add_geofence_point_button,
            KeybindingsEnum.send_reposition_command: self.ui.send_reposition_command_button,
            KeybindingsEnum.send_adv_reposition_command: self.ui.send_adv_reposition_command_button,
            KeybindingsEnum.remove_reposition: self.ui.remove_reposition_button,
            KeybindingsEnum.delete_manual_ads: self.ui.delete_manual_ads_button,
            KeybindingsEnum.select_item: self.ui.select_item_button
        }
        self.set_to_config(config)

        self.currently_waiting = None

        self.ui.add_geofence_point_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.add_geofence_point))
        self.ui.send_reposition_command_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.send_reposition_command))
        self.ui.send_adv_reposition_command_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.send_adv_reposition_command))
        self.ui.remove_reposition_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.remove_reposition))
        self.ui.delete_manual_ads_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.delete_manual_ads))
        self.ui.select_item_button.clicked.connect(lambda b: self.set_current_waiting(KeybindingsEnum.select_item))

        self.ui.buttons.button(QDialogButtonBox.StandardButton.RestoreDefaults).clicked.connect(self.reset_defaults)
        self.ui.buttons.accepted.connect(self._apply_and_accept)

    def _apply_and_accept(self):
        self.original_config.clear()
        self.original_config.update(self.config)
        self.accept()

    def disable_all_except(self, type: KeybindingsEnum):
        for e in self.input_type2button.keys():
            if type != e:
                self.input_type2button[e].setDisabled(True)
        self.ui.buttons.setDisabled(True)

    def enable_all_except(self, type: KeybindingsEnum):
        for e in self.input_type2button.keys():
            if type != e:
                self.input_type2button[e].setDisabled(False)
        self.ui.buttons.setDisabled(False)

    def set_to_config(self, config: dict[KeybindingsEnum, InputMapping]):
        for e in config.keys():
            value = config[e]
            def_mouse_in = value.mouse_input
            def_key_in = value.key_input
            def_mod = value.mod
            if def_mouse_in != Qt.MouseButton.NoButton:
                self.input_type2button[e].setText(_translate_mouse_keys(def_mouse_in, def_mod))
            elif def_key_in != Qt.Key.Key_unknown:
                self.input_type2button[e].setText(QKeySequence(QKeyCombination(def_mod, def_key_in)).toString())
            else:
                self.input_type2button[e].setText(QCoreApplication.translate("InputMapper", "Not Set", None))

    @staticmethod
    def initialize_mappings() -> dict[KeybindingsEnum, InputMapping]:
        config: dict[KeybindingsEnum, InputMapping] = dict()
        for e in KeybindingsEnum:
            def_mouse_in = e.value[1]
            def_key_in = e.value[2]
            def_mod = e.value[3]
            config[e] = InputMapping(e.value[0], def_mouse_in, def_key_in, def_mod)
        return config

    def reset_defaults(self):
        for e in KeybindingsEnum:
            def_mouse_in = e.value[1]
            def_key_in = e.value[2]
            def_mod = e.value[3]
            self.config[e] = InputMapping(e.value[0], def_mouse_in, def_key_in, def_mod)
        self.set_to_config(self.config)

    def set_current_waiting(self, currently_waiting: KeybindingsEnum):
        qInfo("Starting to wait for %s" % currently_waiting)
        self.disable_all_except(currently_waiting)
        self.currently_waiting = currently_waiting
        self.input_type2button[currently_waiting].setText(QCoreApplication.translate("InputMapper", "Waiting for input", None))

    def mousePressEvent(self, event: QMouseEvent, /):
        if self.currently_waiting:
            qInfo("Mouse event: %s %s" % (event.buttons(), event.modifiers()))
            self.config[self.currently_waiting] = InputMapping(self.currently_waiting.value[0], event.buttons(), Qt.Key.Key_unknown, event.modifiers())
            self.input_type2button[self.currently_waiting].setText(_translate_mouse_keys(event.buttons(), event.modifiers()))
            qInfo("Changed to %s" % _translate_mouse_keys(event.buttons(), event.modifiers()))
            self.enable_all_except(self.currently_waiting)
            self.currently_waiting = None
        else:
            super().mousePressEvent(event)

    def keyPressEvent(self, event: QKeyEvent, /):
        if self.currently_waiting and not self.currently_waiting.value[4]:
            if (event.key() == Qt.Key.Key_Control or event.key() == Qt.Key.Key_Shift or
                event.key() == Qt.Key.Key_Alt or event.key() == Qt.Key.Key_AltGr
            ):
                return
            qInfo("Keyboard event: %s %s" % (event.key(), event.modifiers()))
            if event.key() == Qt.Key.Key_Escape:
                self.config[self.currently_waiting] = InputMapping(self.currently_waiting.value[0],
                                                                   Qt.MouseButton.NoButton, Qt.Key.Key_unknown,
                                                                   Qt.KeyboardModifier.NoModifier)
                self.input_type2button[self.currently_waiting].setText(QCoreApplication.translate("InputMapper", "Not Set", None))
                self.currently_waiting = None
                return

            self.config[self.currently_waiting] = InputMapping(self.currently_waiting.value[0], Qt.MouseButton.NoButton, event.keyCombination().key(), event.modifiers())
            self.input_type2button[self.currently_waiting].setText(QKeySequence(event.keyCombination()).toString())
            qInfo("Changed to %s" % QKeySequence(event.keyCombination()).toString())
            self.enable_all_except(self.currently_waiting)
            self.currently_waiting = None
        else:
            super().keyPressEvent(event)
