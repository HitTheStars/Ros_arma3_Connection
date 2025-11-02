@echo off
REM ROS Bridge MOD Build Script
REM This script packages the MOD into PBO files

echo ========================================
echo ROS Bridge MOD Build Script
echo ========================================
echo.

REM Check if Arma 3 Tools are installed
set "ARMA3_TOOLS=C:\Program Files (x86)\Steam\steamapps\common\Arma 3 Tools"
if not exist "%ARMA3_TOOLS%\AddonBuilder\AddonBuilder.exe" (
    echo ERROR: Arma 3 Tools not found!
    echo Please install Arma 3 Tools from Steam.
    echo Expected location: %ARMA3_TOOLS%
    pause
    exit /b 1
)

echo Arma 3 Tools found: %ARMA3_TOOLS%
echo.

REM Set paths
set "MOD_DIR=%~dp0@ROS_Bridge"
set "ADDON_SOURCE=%MOD_DIR%\addons\ros_bridge"
set "OUTPUT_DIR=%MOD_DIR%\addons"

echo MOD Directory: %MOD_DIR%
echo Addon Source: %ADDON_SOURCE%
echo Output Directory: %OUTPUT_DIR%
echo.

REM Check if source directory exists
if not exist "%ADDON_SOURCE%" (
    echo ERROR: Addon source directory not found!
    echo Expected: %ADDON_SOURCE%
    pause
    exit /b 1
)

echo Building PBO...
echo.

REM Build the PBO using AddonBuilder
"%ARMA3_TOOLS%\AddonBuilder\AddonBuilder.exe" "%ADDON_SOURCE%" "%OUTPUT_DIR%" -clear -sign=

if %ERRORLEVEL% == 0 (
    echo.
    echo ========================================
    echo BUILD SUCCESSFUL!
    echo ========================================
    echo.
    echo PBO file created: %OUTPUT_DIR%\ros_bridge.pbo
    echo.
    echo To install the MOD:
    echo 1. Copy the @ROS_Bridge folder to your Arma 3 directory
    echo 2. Copy ArmaCOM_x64.dll to your Arma 3 directory
    echo 3. Launch Arma 3 with the MOD enabled
    echo.
) else (
    echo.
    echo ========================================
    echo BUILD FAILED!
    echo ========================================
    echo.
    echo Please check the error messages above.
    echo.
)

pause
