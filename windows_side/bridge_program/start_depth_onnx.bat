@echo off
REM Arma 3 Depth Bridge 启动脚本 (ONNX + DirectML)

echo ========================================
echo   Arma 3 Depth Estimation Bridge
echo   (ONNX + DirectML)
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python 未安装或不在 PATH 中
    echo 请安装 Python 3.8+ 并添加到 PATH
    pause
    exit /b 1
)

REM 检查依赖是否安装
python -c "import onnxruntime" >nul 2>&1
if errorlevel 1 (
    echo [WARNING] 依赖未安装，正在安装...
    pip install -r requirements_depth.txt
)

REM 检查配置文件
if not exist "config_depth.json" (
    echo [ERROR] 配置文件 config_depth.json 不存在
    echo 请先复制 config_depth.json 并修改配置
    pause
    exit /b 1
)

REM 检查模型文件
if not exist "models\depth_anything_v2_vitb.onnx" (
    echo [ERROR] 模型文件不存在
    echo 请下载 Depth Anything V2 ONNX 模型到 models/ 目录
    echo 下载地址见 README.md
    pause
    exit /b 1
)

echo [INFO] 启动深度估计桥接程序...
echo [INFO] 按 Ctrl+C 退出
echo.

REM 启动桥接程序
python arma3_depth_bridge_onnx.py

pause
