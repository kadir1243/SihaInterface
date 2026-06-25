from enum import Enum

from PySide6.QtCore import Qt

class KeybindingsEnum(Enum):
    # (id, mouse_button, keyboard_button, mod, force mouse)
    select_item = (0, Qt.MouseButton.LeftButton, Qt.Key.Key_unknown, Qt.KeyboardModifier.NoModifier, True)
    remove_reposition = (1, Qt.MouseButton.RightButton, Qt.Key.Key_unknown, Qt.KeyboardModifier.ShiftModifier, False)
    delete_manual_ads = (2, Qt.MouseButton.NoButton, Qt.Key.Key_Delete, Qt.KeyboardModifier.NoModifier, False)
    send_reposition_command = (3, Qt.MouseButton.RightButton, Qt.Key.Key_unknown, Qt.KeyboardModifier.NoModifier, True)
    send_adv_reposition_command = (4, Qt.MouseButton.RightButton, Qt.Key.Key_unknown, Qt.KeyboardModifier.ControlModifier, True)
    add_geofence_point = (5, Qt.MouseButton.MiddleButton, Qt.Key.Key_unknown, Qt.KeyboardModifier.NoModifier, True)

class InputMapping:
    id: int
    mouse_input: Qt.MouseButton
    key_input: Qt.Key
    mod: Qt.KeyboardModifier

    def __init__(self, _id: int, mouse_input: Qt.MouseButton, key_input: Qt.Key, mod: Qt.KeyboardModifier) -> None:
        self.id = _id
        self.mouse_input = mouse_input
        self.key_input = key_input
        self.mod = mod

