@echo off
del /q ui_files_python
mkdir ui_files_python
for %%i in (ui_files/*.ui) do (
   conda run -c "pyside6-uic ui_files/%%i -o ui_files_python/%%~ni.py"
)
