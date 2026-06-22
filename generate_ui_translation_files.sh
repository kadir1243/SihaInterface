uv run bash -c "pyside6-lupdate -locations relative -no-ui-lines -no-obsolete src/MainInterface.py src/ColorSelectorInterface.py $(echo ui_files/*.ui) -ts ui_files/translations/ui_en_US.ts ui_files/translations/ui_tr_TR.ts"

