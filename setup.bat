@echo off
rem Creates .venv and installs requirements.txt. Options are passed to setup.ps1,
rem e.g.  setup.bat -Recreate   or   setup.bat -Python 3.11
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1" %*
set EXITCODE=%ERRORLEVEL%
rem Keep the window open when the script was started by double-click.
echo %CMDCMDLINE% | find /i "%~nx0" >nul && pause
exit /b %EXITCODE%
