@echo off
rem Build foc-monitor.exe - one self-contained file, no Python needed to run it.
rem
rem Output:  dist\foc-monitor.exe
rem Scratch: dist\work  (safe to delete; build\ is the firmware's, not ours)
rem
rem Paths are absolute because --specpath makes PyInstaller resolve
rem --add-data relative to the spec file, not to this directory.
rem
rem   foc-monitor.exe            the window
rem   foc-monitor.exe --check    exercise every command, no window

setlocal
cd /d "%~dp0"

python -m PyInstaller ^
  --onefile ^
  --windowed ^
  --name foc-monitor ^
  --icon "%~dp0unitymbed.ico" ^
  --add-data "%~dp0unitymbed.ico;." ^
  --distpath dist ^
  --workpath dist\work ^
  --specpath dist ^
  --noconfirm ^
  --clean ^
  foc-monitor.py

if errorlevel 1 (
  echo.
  echo build failed
  exit /b 1
)

echo.
echo built dist\foc-monitor.exe
