@echo off
REM Windows 端统一启动脚本
REM 启动深度估计发送器

echo =========================================
echo   启动 Arma 3 - ROS 集成系统 (Windows 端)
echo =========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ 错误: Python 未安装或不在 PATH 中
    echo    请安装 Python 3.8-3.11
    pause
    exit /b 1
)

echo ✓ Python 已安装
echo.

REM 检查是否在正确的目录
if not exist "arma3_depth_sender.py" (
    echo ❌ 错误: 未找到 arma3_depth_sender.py
    echo    请确保在 windows_side 目录下运行此脚本
    pause
    exit /b 1
)

REM 检查模型文件是否存在
if not exist "models\depth_anything_v2_vits.onnx" (
    echo ❌ 错误: 未找到 Depth Anything V2 模型文件
    echo    请下载模型到 models\ 目录
    echo    参考: models\MODEL_DOWNLOAD_GUIDE.md
    pause
    exit /b 1
)

echo ✓ 模型文件已找到
echo.

REM 提示用户输入 Linux IP
set /p LINUX_IP="请输入 Linux VM 的 IP 地址 (例如: 192.168.1.100): "

if "%LINUX_IP%"=="" (
    echo ❌ 错误: IP 地址不能为空
    pause
    exit /b 1
)

echo.
echo =========================================
echo   启动深度估计发送器
echo =========================================
echo.
echo 配置:
echo   - Linux IP: %LINUX_IP%
echo   - Linux Port: 5555
echo   - 输入源: 摄像头 (ID: 0)
echo   - 目标帧率: 10 FPS
echo.
echo 按 Ctrl+C 停止系统
echo.

REM 启动深度发送器
python arma3_depth_sender.py ^
  --model models\depth_anything_v2_vits.onnx ^
  --linux-ip %LINUX_IP% ^
  --linux-port 5555 ^
  --input camera ^
  --source 0 ^
  --fps 10

pause
