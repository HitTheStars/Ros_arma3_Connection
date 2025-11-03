@echo off
REM Arma 3 Image Collector Startup Script
REM Arma 3 图像采集器启动脚本

echo ============================================================
echo Arma 3 Image Collector
echo ============================================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.8+ from https://www.python.org/
    pause
    exit /b 1
)

REM 配置参数
set SCREENSHOTS_DIR=%USERPROFILE%\Documents\Arma 3\Screenshots
set SERVER_HOST=localhost
set SERVER_PORT=5557
set NUM_UAVS=6

echo Configuration:
echo   Screenshots dir: %SCREENSHOTS_DIR%
echo   Server: %SERVER_HOST%:%SERVER_PORT%
echo   Number of UAVs: %NUM_UAVS%
echo.

REM 检查 Screenshots 目录是否存在
if not exist "%SCREENSHOTS_DIR%" (
    echo Warning: Screenshots directory does not exist
    echo Creating directory: %SCREENSHOTS_DIR%
    mkdir "%SCREENSHOTS_DIR%"
)

echo Starting image collector...
echo.

REM 启动采集器
python arma3_image_collector.py "%SCREENSHOTS_DIR%" %SERVER_HOST% %SERVER_PORT% %NUM_UAVS%

pause
