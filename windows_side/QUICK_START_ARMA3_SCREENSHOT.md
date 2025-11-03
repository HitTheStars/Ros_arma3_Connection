# Arma 3 Screenshot Mode - Quick Start Guide

快速开始指南：使用屏幕截图模式从 Arma 3 获取图像并生成深度图

---

## 📋 前提条件

### 已完成
- ✅ Python 3.8-3.11 已安装
- ✅ 依赖包已安装（包括 `mss` 和 `pywin32`）
- ✅ Depth Anything V2 ONNX 模型已下载
- ✅ Arma 3 已安装

### 需要准备
- ⏳ Linux ROS VM 的 IP 地址
- ⏳ Arma 3 MOD 已安装（@ROS_Bridge）

---

## 🚀 快速启动（3 步）

### 步骤 1：启动 Linux 端深度接收器（5 分钟）

在 Linux ROS VM 中：

```bash
cd ~/Ros_arma3_Connection/linux_side
./start_system.sh
```

记录显示的 IP 地址（如 `192.168.1.100`）

---

### 步骤 2：启动 Arma 3 并加载场景（10 分钟）

1. **启动 Arma 3**（禁用 BattlEye）：
   ```cmd
   "C:\Program Files (x86)\Steam\steamapps\common\Arma 3\arma3_x64.exe" -mod=@ROS_Bridge -noBE
   ```

2. **进入 Eden Editor**：
   - 点击 `Editor` → 选择地图（如 `Altis`）
   - 创建新场景或加载现有场景

3. **添加 6 个无人机**：
   - 在地图上放置 6 个无人机
   - 命名为：`uav1`, `uav2`, `uav3`, `uav4`, `uav5`, `uav6`
   - 类型：AR-2 Darter 或其他小型无人机

4. **添加初始化脚本**：
   - 在场景属性中，找到 `Init` 字段
   - 添加以下代码：
     ```sqf
     execVM "\@ROS_Bridge\addons\ros_bridge\scripts\initUAVCameras.sqf";
     ```

5. **保存并预览**：
   - 保存场景
   - 点击 `Preview` 按钮
   - 应该看到屏幕上显示 6 个无人机的摄像头画面（3列×2行）

---

### 步骤 3：启动 Windows 端深度发送器（5 分钟）

在 Windows 命令提示符中：

```cmd
cd C:\path\to\Ros_arma3_Connection\windows_side
python arma3_depth_sender.py ^
    --model models\depth_anything_v2_vits.onnx ^
    --linux-ip 192.168.1.100 ^
    --input arma3_screenshot ^
    --region 0 ^
    --fps 10
```

**参数说明**：
- `--model`: ONNX 模型路径
- `--linux-ip`: Linux ROS VM 的 IP 地址（替换为实际 IP）
- `--input arma3_screenshot`: 使用屏幕截图模式
- `--region 0`: 截取第 0 个区域（UAV 1）
  - 可选值：0-5（对应 UAV 1-6）
- `--fps 10`: 目标帧率 10 FPS

---

## 📊 验证系统运行

### 在 Windows 端

应该看到类似输出：

```
[ScreenCapture] 初始化 Arma 3 屏幕截图器
[ScreenCapture] 窗口标题: Arma 3
[ScreenCapture] 区域索引: 0/6
[ScreenCapture] 正在检测 Arma 3 窗口...
[ScreenCapture] 找到窗口: Arma 3
[ScreenCapture] 窗口客户区: (100, 50, 1820, 1030)
[ScreenCapture] 截取区域 0: (100, 50, 673, 376)
[ScreenCapture] 区域尺寸: 573x326
[DepthEstimator] 加载 ONNX 模型: models\depth_anything_v2_vits.onnx
[DepthEstimator] 使用的执行提供器: ['DmlExecutionProvider', 'CPUExecutionProvider']
[DepthSender] 连接到 Linux ROS VM: 192.168.1.100:5555
[DepthSender] 连接成功
[DepthSender] 启动深度发送器...
[DepthSender] 目标帧率: 10 FPS
[DepthSender] 输入源: arma3_screenshot

[DepthSender] 统计 (最近 30 帧):
  - 平均推理时间: 12.3 ms
  - 平均传输时间: 15.7 ms
  - 平均总时间: 28.0 ms
  - 实际 FPS: 35.7
```

### 在 Linux 端

检查 ROS 话题：

```bash
rostopic hz /camera/depth/image_rect_raw
```

应该看到：

```
subscribed to [/camera/depth/image_rect_raw]
average rate: 10.000
    min: 0.095s max: 0.105s std dev: 0.00234s window: 30
```

---

## 🎮 使用技巧

### 切换无人机视角

要截取不同无人机的视角，修改 `--region` 参数：

```cmd
# UAV 1 (左上)
--region 0

# UAV 2 (中上)
--region 1

# UAV 3 (右上)
--region 2

# UAV 4 (左下)
--region 3

# UAV 5 (中下)
--region 4

# UAV 6 (右下)
--region 5
```

### 同时运行多个深度发送器

可以同时运行多个 Python 进程，每个截取不同的区域：

```cmd
# 终端 1: UAV 1
python arma3_depth_sender.py --model models\depth_anything_v2_vits.onnx --linux-ip 192.168.1.100 --region 0

# 终端 2: UAV 2
python arma3_depth_sender.py --model models\depth_anything_v2_vits.onnx --linux-ip 192.168.1.100 --region 1
```

**注意**: 需要修改 Linux 端接收器以支持多个连接。

### 调整帧率

根据系统性能调整帧率：

```cmd
# 低性能: 5 FPS
--fps 5

# 标准: 10 FPS (推荐)
--fps 10

# 高性能: 15 FPS
--fps 15
```

---

## 🔧 故障排除

### 问题 1: 找不到 Arma 3 窗口

**错误信息**:
```
RuntimeError: 未找到窗口: Arma 3
```

**解决方法**:
1. 确保 Arma 3 正在运行
2. 确保窗口标题包含 "Arma 3"
3. 如果窗口标题不同，修改 `Arma3ScreenCapture` 类的 `window_title` 参数

---

### 问题 2: 截图区域不对

**症状**: 截取的画面不是无人机视角

**解决方法**:
1. 检查 Arma 3 窗口是否最大化
2. 检查屏幕分辨率是否改变
3. 确保 SQF 脚本正确执行（屏幕上应该显示 6 个摄像头画面）

---

### 问题 3: 帧率太低

**症状**: 实际 FPS < 5

**解决方法**:
1. 降低目标帧率：`--fps 5`
2. 检查 GPU 是否正常工作（应该使用 DmlExecutionProvider）
3. 关闭其他占用 GPU 的程序
4. 降低 Arma 3 的图形设置

---

### 问题 4: MSS 或 pywin32 未安装

**错误信息**:
```
RuntimeError: MSS or pywin32 not installed. Please run: pip install mss pywin32
```

**解决方法**:
```cmd
pip install mss==9.0.1 pywin32==306
```

---

## 📈 性能优化

### 优化 1: 降低 R2T 分辨率

修改 `initUAVCameras.sqf` 第 34 行：

```sqf
# 从
_ctrl ctrlSetText format ["#(rgb,512,512,1)r2t(uav_cam%1,1.0)", _index];

# 改为
_ctrl ctrlSetText format ["#(rgb,256,256,1)r2t(uav_cam%1,1.0)", _index];
```

**效果**: 降低 CPU 占用，提高帧率

---

### 优化 2: 减少摄像机更新频率

修改 `initUAVCameras.sqf` 第 61 行：

```sqf
# 从
sleep 0.005; // 5 ms

# 改为
sleep 0.01; // 10 ms
```

**效果**: 降低 CPU 占用

---

### 优化 3: 使用更小的 ONNX 模型

使用 `depth_anything_v2_vits.onnx`（最小）而非 `depth_anything_v2_vitl.onnx`（最大）

**效果**: 推理时间从 ~40 ms 降低到 ~12 ms

---

## 🎯 下一步

### 集成到 EGO-Planner

深度图像已经发布到 ROS 话题 `/camera/depth/image_rect_raw`，可以直接被 EGO-Planner 使用：

```bash
roslaunch ego_planner simple_run.launch
```

### 录制数据

使用 `rosbag` 录制深度图像数据：

```bash
rosbag record /camera/depth/image_rect_raw -O arma3_depth.bag
```

### 可视化深度图像

使用 `rviz` 可视化：

```bash
rosrun rviz rviz
# 添加 Image 显示，话题选择 /camera/depth/image_rect_raw
```

---

## 📚 相关文档

- [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) - 完整部署指南
- [MOD_INSTALLATION_GUIDE.md](arma3_mod/MOD_INSTALLATION_GUIDE.md) - Arma 3 MOD 安装
- [scripts/README.md](arma3_mod/@ROS_Bridge/addons/ros_bridge/scripts/README.md) - SQF 脚本说明

---

## ❓ 常见问题

**Q: 为什么使用屏幕截图而非直接获取纹理？**

A: 直接获取纹理需要编写 C++ 扩展，实现复杂。屏幕截图方案简单、可靠，足够用于原型开发和演示。

**Q: 屏幕截图的延迟是多少？**

A: 使用 MSS 库，截图延迟约 50 ms，加上深度估计 12 ms 和传输 10 ms，总延迟约 **70-100 ms**。

**Q: 可以最小化 Arma 3 窗口吗？**

A: 不可以。屏幕截图需要窗口可见。如果需要后台运行，请使用共享内存方案（需要编写 C++ 扩展）。

**Q: 如何提高截图质量？**

A: 
1. 提高 Arma 3 的图形设置
2. 提高 R2T 分辨率（修改 SQF 脚本）
3. 使用更大的 ONNX 模型（如 `vitl` 而非 `vits`）

---

祝您使用愉快！🚁✨
