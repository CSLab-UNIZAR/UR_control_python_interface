@echo off
rem Starts the web control panel and opens it in the browser.
rem Extra options are passed on, e.g.  run_webui.bat --host 192.168.0.200
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo The virtual environment is missing: run setup.bat first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -m webui %*
if errorlevel 1 pause
