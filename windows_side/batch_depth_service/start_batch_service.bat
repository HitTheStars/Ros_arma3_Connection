@echo off
REM Batch Depth Estimation Service Startup Script
REM 批处理深度估计服务启动脚本

echo ============================================================
echo Batch Depth Estimation Service
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

REM 检查模型文件是否存在
if not exist "..\models\depth_anything_v2_vits.onnx" (
    echo Error: Model file not found
    echo Expected location: ..\models\depth_anything_v2_vits.onnx
    echo.
    echo Please download the model from:
    echo https://huggingface.co/depth-anything/Depth-Anything-V2-Small
    echo.
    pause
    exit /b 1
)

REM 配置参数
set MODEL_PATH=..\models\depth_anything_v2_vits.onnx
set NUM_UAVS=6
set BATCH_SIZE=6
set ROS_IP=192.168.1.100

echo Configuration:
echo   Model: %MODEL_PATH%
echo   Number of UAVs: %NUM_UAVS%
echo   Batch size: %BATCH_SIZE%
echo   ROS IP: %ROS_IP%
echo.
echo Starting service...
echo.

REM 启动服务
python batch_depth_service.py %MODEL_PATH% %NUM_UAVS% %BATCH_SIZE% %ROS_IP%

pause
