@echo off
REM Quick Start Script for Windows Side
REM This script starts the bridge program

echo ==============================================
echo Starting Arma 3 Bridge Program (Windows Side)
echo ==============================================
echo.

REM Get script directory
set SCRIPT_DIR=%~dp0

REM Check if Arma 3 is running
tasklist /FI "IMAGENAME eq arma3_x64.exe" 2>NUL | find /I /N "arma3_x64.exe">NUL
if "%ERRORLEVEL%"=="0" (
    echo Arma 3 is running. Good!
) else (
    echo WARNING: Arma 3 is not running.
    echo Please start Arma 3 and load the Camera.Altis mission first.
    echo.
    set /p CONTINUE="Continue anyway? (y/n): "
    if /i not "%CONTINUE%"=="y" exit /b 1
)

echo.
echo Starting bridge program...
echo Press Ctrl+C to stop.
echo.

cd "%SCRIPT_DIR%bridge_program"
python arma3_bridge_enhanced.py

pause
