@echo off
REM ROS Bridge MOD Quick Installation Script
REM This script creates a symbolic link to use the MOD without PBO packaging

echo ========================================
echo ROS Bridge MOD Quick Install (Dev Mode)
echo ========================================
echo.
echo This script will create a symbolic link to use the MOD
echo without needing to package it into PBO files.
echo This is useful for development and testing.
echo.

REM Check for administrator privileges
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo ERROR: This script requires administrator privileges!
    echo Please right-click and select "Run as administrator"
    pause
    exit /b 1
)

REM Try to find Arma 3 installation directory
set "ARMA3_DIR="

REM Check common Steam installation paths
if exist "C:\Program Files (x86)\Steam\steamapps\common\Arma 3\arma3_x64.exe" (
    set "ARMA3_DIR=C:\Program Files (x86)\Steam\steamapps\common\Arma 3"
)

if exist "C:\Program Files\Steam\steamapps\common\Arma 3\arma3_x64.exe" (
    set "ARMA3_DIR=C:\Program Files\Steam\steamapps\common\Arma 3"
)

if exist "D:\SteamLibrary\steamapps\common\Arma 3\arma3_x64.exe" (
    set "ARMA3_DIR=D:\SteamLibrary\steamapps\common\Arma 3"
)

if exist "E:\SteamLibrary\steamapps\common\Arma 3\arma3_x64.exe" (
    set "ARMA3_DIR=E:\SteamLibrary\steamapps\common\Arma 3"
)

REM If not found, ask user
if "%ARMA3_DIR%"=="" (
    echo Arma 3 installation not found automatically.
    echo Please enter the path to your Arma 3 installation directory:
    echo Example: C:\Program Files (x86)\Steam\steamapps\common\Arma 3
    echo.
    set /p "ARMA3_DIR=Arma 3 Directory: "
)

REM Verify the directory
if not exist "%ARMA3_DIR%\arma3_x64.exe" (
    echo.
    echo ERROR: Invalid Arma 3 directory!
    echo arma3_x64.exe not found in: %ARMA3_DIR%
    pause
    exit /b 1
)

echo.
echo Arma 3 found: %ARMA3_DIR%
echo.

REM Set paths
set "MOD_SOURCE=%~dp0@ROS_Bridge"
set "MOD_LINK=%ARMA3_DIR%\@ROS_Bridge"
set "DLL_SOURCE=%~dp0ArmaCOM_x64.dll"
set "DLL_DEST=%ARMA3_DIR%\ArmaCOM_x64.dll"

echo Creating symbolic link for MOD...
echo Source: %MOD_SOURCE%
echo Link: %MOD_LINK%
echo.

REM Remove existing link or directory
if exist "%MOD_LINK%" (
    echo Removing existing MOD directory/link...
    rmdir "%MOD_LINK%" /S /Q 2>nul
    del "%MOD_LINK%" 2>nul
)

REM Create symbolic link
mklink /D "%MOD_LINK%" "%MOD_SOURCE%"
if %ERRORLEVEL% == 0 (
    echo Symbolic link created successfully!
) else (
    echo ERROR: Failed to create symbolic link!
    pause
    exit /b 1
)

echo.

REM Copy ArmaCOM DLL
if exist "%DLL_SOURCE%" (
    echo Copying ArmaCOM_x64.dll...
    copy "%DLL_SOURCE%" "%DLL_DEST%" /Y
    if %ERRORLEVEL% == 0 (
        echo ArmaCOM_x64.dll copied successfully!
    ) else (
        echo ERROR: Failed to copy ArmaCOM_x64.dll!
        pause
        exit /b 1
    )
) else (
    echo ERROR: ArmaCOM_x64.dll not found: %DLL_SOURCE%
    pause
    exit /b 1
)

echo.
echo ========================================
echo QUICK INSTALL SUCCESSFUL!
echo ========================================
echo.
echo MOD linked to: %MOD_LINK%
echo ArmaCOM DLL installed to: %DLL_DEST%
echo.
echo You can now edit the MOD files directly in:
echo %MOD_SOURCE%
echo.
echo Changes will be reflected immediately in Arma 3!
echo.
echo IMPORTANT: To use this MOD, you MUST:
echo 1. Disable BattlEye (ArmaCOM does not work with BattlEye enabled)
echo 2. Launch Arma 3 with the -mod=@ROS_Bridge parameter
echo 3. Make sure your Linux ROS server is running
echo.
echo To launch Arma 3 with the MOD:
echo "%ARMA3_DIR%\arma3_x64.exe" -mod=@ROS_Bridge -noBE
echo.

pause
