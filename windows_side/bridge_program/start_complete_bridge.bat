@echo off
echo ========================================
echo Arma 3 ^<-^> ROS Complete Bridge
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed or not in PATH!
    echo Please install Python 3.8 or higher from https://www.python.org/
    pause
    exit /b 1
)

echo [INFO] Checking dependencies...
pip show opencv-python >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] Installing dependencies...
    pip install -r requirements_complete.txt
)

echo.
echo [INFO] Starting Complete Bridge...
echo [INFO] This bridge includes:
echo   - Image capture (6 cameras)
echo   - Stereo vision processing
echo   - Point cloud generation
echo   - TCP/IP communication with ROS
echo.
echo [INFO] Press Ctrl+C to stop.
echo.

python arma3_ros_bridge_complete.py

pause
