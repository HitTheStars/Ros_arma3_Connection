@echo off
echo ========================================
echo Arma 3 Depth Image Bridge (EGO-Planner Compatible)
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found! Please install Python 3.7+
    pause
    exit /b 1
)

echo [INFO] Checking dependencies...
pip show opencv-python >nul 2>&1
if errorlevel 1 (
    echo [INFO] Installing opencv-python...
    pip install opencv-python
)

pip show pillow >nul 2>&1
if errorlevel 1 (
    echo [INFO] Installing pillow...
    pip install pillow
)

pip show numpy >nul 2>&1
if errorlevel 1 (
    echo [INFO] Installing numpy...
    pip install numpy
)

echo.
echo [INFO] Starting Arma 3 Depth Image Bridge...
echo [INFO] Make sure:
echo   1. Arma 3 is running with the Camera mission
echo   2. Linux ROS server is running (arma3_depth_bridge.py)
echo   3. Server IP is correctly configured in arma3_ros_bridge_depth.py
echo.

python arma3_ros_bridge_depth.py

pause
