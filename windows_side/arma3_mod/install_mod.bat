@echo off
REM ROS Bridge MOD Installation Script
REM This script installs the MOD to your Arma 3 directory

echo ========================================
echo ROS Bridge MOD Installation Script
echo ========================================
echo.

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

REM Set source and destination paths
set "MOD_SOURCE=%~dp0@ROS_Bridge"
set "MOD_DEST=%ARMA3_DIR%\@ROS_Bridge"
set "DLL_SOURCE=%~dp0ArmaCOM_x64.dll"
set "DLL_DEST=%ARMA3_DIR%\ArmaCOM_x64.dll"

echo Installing ROS Bridge MOD...
echo.

REM Copy MOD folder
if exist "%MOD_SOURCE%" (
    echo Copying @ROS_Bridge folder...
    xcopy "%MOD_SOURCE%" "%MOD_DEST%\" /E /I /Y
    if %ERRORLEVEL% == 0 (
        echo @ROS_Bridge folder copied successfully!
    ) else (
        echo ERROR: Failed to copy @ROS_Bridge folder!
        pause
        exit /b 1
    )
) else (
    echo ERROR: MOD source folder not found: %MOD_SOURCE%
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
echo INSTALLATION SUCCESSFUL!
echo ========================================
echo.
echo MOD installed to: %MOD_DEST%
echo ArmaCOM DLL installed to: %DLL_DEST%
echo.
echo IMPORTANT: To use this MOD, you MUST:
echo 1. Disable BattlEye (ArmaCOM does not work with BattlEye enabled)
echo 2. Launch Arma 3 with the -mod=@ROS_Bridge parameter
echo 3. Make sure your Linux ROS server is running
echo 4. Update the server IP in the MOD settings (default: 192.168.1.100)
echo.
echo To launch Arma 3 with the MOD:
echo "%ARMA3_DIR%\arma3_x64.exe" -mod=@ROS_Bridge -noBE
echo.

pause
