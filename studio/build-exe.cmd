@echo off
rem Build FOC.exe - one self-contained file, no Python needed to run it.
rem
rem Output:  studio\dist\FOC.exe
rem Scratch: studio\dist\work  (safe to delete)
rem
rem Needs Python 3 with Pillow and PyInstaller:  pip install pillow pyinstaller
rem
rem Paths are absolute because --specpath makes PyInstaller resolve
rem --add-data relative to the spec file, not to this directory.
rem
rem   FOC.exe                    the window
rem   FOC.exe --selftest FILE    run the model's self-check, write FILE
rem   FOC.exe --shots DIR        save a picture of every page into DIR

setlocal
cd /d "%~dp0"

python -m PyInstaller ^
  --onefile ^
  --windowed ^
  --name FOC ^
  --icon "%~dp0..\unitymbed.ico" ^
  --add-data "%~dp0..\unitymbed.ico;." ^
  --exclude-module numpy ^
  --distpath dist ^
  --workpath dist\work ^
  --specpath dist ^
  --noconfirm ^
  --clean ^
  foc.py

if errorlevel 1 (
  echo.
  echo build failed
  exit /b 1
)

echo.
echo built dist\FOC.exe
