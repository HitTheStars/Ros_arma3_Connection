@echo off
REM Arma 3 Depth Image Sender - 快速启动脚本
REM 作者: Manus AI
REM 日期: 2025-11-03

echo ========================================
echo   Arma 3 Depth Image Sender
echo ========================================
echo.

REM 检查虚拟环境是否存在
if not exist "venv\Scripts\activate.bat" (
    echo [WARNING] 虚拟环境不存在，正在创建...
    python -m venv venv
    call venv\Scripts\activate
    echo [INFO] 安装依赖包...
    pip install -r requirements.txt
) else (
    REM 激活虚拟环境
    call venv\Scripts\activate
)

REM 检查模型文件是否存在
if not exist "models\depth_anything_v2_vits.onnx" (
    echo [ERROR] 模型文件不存在: models\depth_anything_v2_vits.onnx
    echo [INFO] 请先下载模型文件，参考 DEPLOYMENT_GUIDE.md
    pause
    exit /b 1
)

REM 设置参数（请根据实际情况修改）
set MODEL_PATH=models\depth_anything_v2_vits.onnx
set LINUX_IP=192.168.1.100
set LINUX_PORT=5555
set INPUT_TYPE=camera
set SOURCE=0
set FPS=10

REM 显示配置
echo [INFO] 配置信息:
echo   - 模型路径: %MODEL_PATH%
echo   - Linux IP: %LINUX_IP%:%LINUX_PORT%
echo   - 输入源: %INPUT_TYPE% (%SOURCE%)
echo   - 目标帧率: %FPS% FPS
echo.
echo [INFO] 启动深度图像发送器...
echo [INFO] 按 Ctrl+C 退出
echo.

REM 启动发送器
python arma3_depth_sender.py ^
  --model %MODEL_PATH% ^
  --linux-ip %LINUX_IP% ^
  --linux-port %LINUX_PORT% ^
  --input %INPUT_TYPE% ^
  --source %SOURCE% ^
  --fps %FPS%

pause
