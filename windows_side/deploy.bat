@echo off
REM One-Click Deployment Script for Windows Side
REM This script sets up the Arma 3 MOD and bridge program

echo ==============================================
echo Arma 3 ^<-^> ROS Integration - Windows Deployment
echo ==============================================
echo.

REM Get script directory
set SCRIPT_DIR=%~dp0

echo Step 1/4: Checking Python installation...
echo ------------------------------------------

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH.
    echo Please install Python 3.8 or later from https://www.python.org/
    pause
    exit /b 1
)

echo Python is installed.
echo.

echo Step 2/4: Installing Python dependencies...
echo -------------------------------------------

pip install --upgrade pip
pip install pillow numpy opencv-python

echo.
echo Step 3/4: Setting up Arma 3 MOD...
echo ----------------------------------

set ARMA3_MISSIONS_DIR=%USERPROFILE%\Documents\Arma 3 - Other Profiles

REM Try to find Arma 3 missions directory
if not exist "%ARMA3_MISSIONS_DIR%" (
    echo WARNING: Arma 3 missions directory not found at:
    echo %ARMA3_MISSIONS_DIR%
    echo.
    echo Please manually copy the Camera.Altis folder to your Arma 3 missions directory.
    echo The directory is usually at:
    echo C:\Users\^<YourUsername^>\Documents\Arma 3 - Other Profiles\^<YourProfileName^>\missions\
    echo.
) else (
    echo Found Arma 3 directory: %ARMA3_MISSIONS_DIR%
    echo.
    echo Available profiles:
    dir /b "%ARMA3_MISSIONS_DIR%"
    echo.
    set /p PROFILE_NAME="Enter your Arma 3 profile name: "
    
    set MISSIONS_DIR=%ARMA3_MISSIONS_DIR%\!PROFILE_NAME!\missions
    
    if not exist "!MISSIONS_DIR!" (
        mkdir "!MISSIONS_DIR!"
    )
    
    echo Copying Camera.Altis to !MISSIONS_DIR!...
    xcopy /E /I /Y "%SCRIPT_DIR%arma3_mod\Camera.Altis" "!MISSIONS_DIR!\Camera.Altis"
    
    echo Camera.Altis mission installed successfully!
)

echo.
echo Step 4/4: Configuring network settings...
echo -----------------------------------------

set /p LINUX_IP="Enter your Linux VM IP address (e.g., 192.168.1.100): "

REM Update the bridge program with the Linux IP
powershell -Command "(gc '%SCRIPT_DIR%bridge_program\arma3_bridge_enhanced.py') -replace 'SERVER_IP = ''.*''', 'SERVER_IP = ''%LINUX_IP%''' | Out-File -encoding ASCII '%SCRIPT_DIR%bridge_program\arma3_bridge_enhanced.py'"

echo Network configuration updated.
echo Linux IP: %LINUX_IP%
echo Port: 5555

REM Create a configuration file
echo Linux VM IP Address: %LINUX_IP% > "%SCRIPT_DIR%network_config.txt"
echo ROS Bridge Port: 5555 >> "%SCRIPT_DIR%network_config.txt"
echo. >> "%SCRIPT_DIR%network_config.txt"
echo Configuration: >> "%SCRIPT_DIR%network_config.txt"
echo 1. Start Linux ROS nodes first >> "%SCRIPT_DIR%network_config.txt"
echo 2. Start Arma 3 and load Camera.Altis mission >> "%SCRIPT_DIR%network_config.txt"
echo 3. Run arma3_bridge_enhanced.py >> "%SCRIPT_DIR%network_config.txt"

echo.
echo ==============================================
echo Deployment Complete!
echo ==============================================
echo.
echo Next steps:
echo 1. Start ROS nodes on Linux VM:
echo    roslaunch arma3_ros_bridge arma3_bridge.launch
echo.
echo 2. Start Arma 3:
echo    - Open Arma 3
echo    - Go to Editor
echo    - Select Altis map
echo    - Load "Camera" mission
echo    - Click Preview to start
echo.
echo 3. Run the bridge program:
echo    cd bridge_program
echo    python arma3_bridge_enhanced.py
echo.
echo For more information, see the README.md file.
echo.
pause
