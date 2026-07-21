@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0check-architecture.ps1" %*
exit /b %ERRORLEVEL%
