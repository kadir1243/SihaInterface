rm -r ui_files_python
mkdir ui_files_python
cd ui_files || exit
for entry in *.ui
do
  entryWithoutUiExtension=$(echo "$entry" | cut -d '.' -f 1)
  uv run bash -c "pyside6-uic $entry -o ../ui_files_python/$entryWithoutUiExtension".py
done
