# Arma 3 - ROS 系统快速启动指南

本指南提供**一站式**的系统启动流程，让您快速运行 Arma 3 与 ROS 集成系统。

---

## 📋 目录

- [选择架构方案](#选择架构方案)
- [方案 1: 简化架构（仅状态通信）](#方案-1-简化架构仅状态通信)
- [方案 2: 深度估计架构（含视觉感知）](#方案-2-深度估计架构含视觉感知)
- [常见问题](#常见问题)

---

## 选择架构方案

本项目提供两种架构方案，请根据您的需求选择：

### 🎯 方案对比

| 特性 | 方案 1: 简化架构 | 方案 2: 深度估计架构 |
|------|-----------------|---------------------|
| **延迟** | 5-20 ms | 40-50 ms |
| **数据量** | 200 B | ~50 KB/帧 |
| **实现复杂度** | 低（500 行） | 中（2000 行） |
| **GPU 要求** | 无 | RTX 4060 或更高 |
| **适用地图范围** | < 100m²（室内） | 无限制（大地图） |
| **障碍物检测** | 仅预加载静态地图 | 实时动态检测 |
| **适用场景** | 通信测试 | **灾害救援（推荐）** |
| **EGO-Planner 输入** | 预加载点云 + 里程计 | 深度图像 + 里程计 |

### 💡 推荐选择

- **灾害救援场景**：必须使用**方案 2（深度估计）**
  - Altis 大地图（270 km²）无法预加载全部点云
  - 动态环境需要实时障碍物检测
  - 完整的感知-规划-控制闭环

- **快速测试通信**：可以先用**方案 1（简化）**
  - 仅验证 Arma 3 ↔ ROS 通信是否正常
  - 不需要 GPU，安装简单
  - 但**无法用于实际飞行**

---

## 方案 1: 简化架构（仅状态通信）

### ⏱️ 预计时间: 30 分钟

### 步骤 1: Linux 端部署（15 分钟）

#### 1.1 克隆仓库

```bash
git clone https://github.com/HitTheStars/Ros_arma3_Connection.git
cd Ros_arma3_Connection
```

#### 1.2 运行一键部署脚本

```bash
cd linux_side
chmod +x deploy.sh
./deploy.sh
```

**部署脚本会自动**:
- 安装 ROS Noetic（如未安装）
- 安装依赖包（Armadillo, Glog, SuiteSparse）
- 克隆并编译 EGO-Planner-v2
- 创建 ROS 工作空间
- 配置环境变量

#### 1.3 记录 Linux IP 地址

部署完成后，脚本会显示 Linux IP 地址，例如：

```
✓ 部署完成！
Linux IP 地址: 192.168.1.100
```

**记下这个 IP 地址**，后续配置需要用到。

---

### 步骤 2: Windows 端 MOD 安装（10 分钟）

#### 2.1 快速安装 MOD

1. 打开 `windows_side/arma3_mod/` 目录
2. **右键点击** `quick_install.bat`
3. 选择 **"以管理员身份运行"**
4. 按照提示操作

**脚本会自动**:
- 检测 Arma 3 安装路径
- 创建 `@ROS_Bridge` MOD 符号链接
- 复制 ArmaCOM 扩展
- 安装任务文件

#### 2.2 配置 MOD

编辑 `@ROS_Bridge/addons/ros_bridge/XEH_postInit.sqf`：

```sqf
ROS_ServerIP = "192.168.1.100";  // 改成你的 Linux VM IP
ROS_ServerPort = "5555";
```

**提示**: 使用记事本或 Notepad++ 打开文件。

---

### 步骤 3: 启动系统（5 分钟）

#### 3.1 启动 Linux 端

在 Linux VM 中：

```bash
cd ~/Ros_arma3_Connection/linux_side
./start.sh
```

**预期输出**:
```
[INFO] 启动 ROS 桥接节点...
[INFO] 启动 EGO-Planner...
[INFO] 等待 Arma 3 连接...
```

#### 3.2 启动 Arma 3

**方法 A: 使用启动器（推荐）**

1. 打开 Arma 3 启动器
2. **MODs** → 勾选 `@ROS_Bridge`
3. **Parameters** → **Additional Parameters** → 添加 `-noBE`
4. 点击 **PLAY**

**方法 B: 使用命令行**

```cmd
"C:\Program Files (x86)\Steam\steamapps\common\Arma 3\arma3_x64.exe" -mod=@ROS_Bridge -noBE
```

#### 3.3 加载任务

在 Arma 3 中：

1. **Editor** → **Altis** → **Load** → 选择 **Camera**
2. 点击 **Preview** (或按 `Ctrl+P`)

#### 3.4 验证连接

在 Linux 端，您应该看到：

```
[INFO] Arma 3 已连接: ('192.168.1.xxx', 12345)
[INFO] 接收到无人机状态: 6 架
```

在 Arma 3 中，按 `~` 打开控制台，应该看到：

```
[ROS_Bridge] 连接到 ROS 服务器成功
[ROS_Bridge] 发送状态数据...
```

---

### ✅ 完成！

系统现在已经运行，您可以：

- 在 Linux 端查看 ROS 话题: `rostopic list`
- 查看无人机状态: `rostopic echo /arma3/odom`
- 发送控制指令: `rostopic pub /arma3/cmd ...`

---

## 方案 2: 深度估计架构（含视觉感知）

### ⏱️ 预计时间: 60 分钟

**前提**: 已完成**方案 1**的所有步骤。

### 步骤 1: Windows 端深度估计模块安装（30 分钟）

#### 1.1 安装 Python 依赖

```cmd
cd windows_side

REM 创建虚拟环境
python -m venv venv
venv\Scripts\activate

REM 安装依赖
pip install -r requirements.txt
```

**依赖包**:
- `opencv-python==4.8.1.78`
- `numpy==1.24.3`
- `onnxruntime-directml==1.17.1`

#### 1.2 测试安装

```cmd
python test_installation.py
```

**预期输出**:
```
✓ OpenCV 版本: 4.8.1
✓ NumPy 版本: 1.24.3
✓ ONNX Runtime 版本: 1.17.1
✓ DirectML 可用
✓ 摄像头可用
✓ 所有测试通过！
```

#### 1.3 下载 Depth Anything V2 模型

**方法 A: 从 Hugging Face 下载（推荐）**

1. 访问 https://huggingface.co/depth-anything/Depth-Anything-V2-Small-ONNX
2. 下载 `depth_anything_v2_vits.onnx` (~100 MB)
3. 放置到 `windows_side/models/` 目录

**方法 B: 使用浏览器直接下载**

```
https://huggingface.co/depth-anything/Depth-Anything-V2-Small-ONNX/resolve/main/depth_anything_v2_vits.onnx
```

**验证**: 确保文件路径为 `windows_side/models/depth_anything_v2_vits.onnx`

---

### 步骤 2: Linux 端深度接收器配置（5 分钟）

深度接收器已经包含在项目中，无需额外安装。

**验证文件存在**:
```bash
ls -l ~/Ros_arma3_Connection/linux_side/ros_nodes/arma3_depth_receiver.py
ls -l ~/Ros_arma3_Connection/linux_side/start_depth_receiver.sh
```

---

### 步骤 3: 配置网络（5 分钟）

#### 3.1 查找 Linux IP

在 Linux VM 中：

```bash
hostname -I
```

假设输出为 `192.168.1.100`。

#### 3.2 测试网络连接

在 Windows 中：

```cmd
ping 192.168.1.100
```

如果 ping 不通，检查：
- VM 网络模式（推荐使用**桥接模式**）
- 防火墙设置

#### 3.3 配置防火墙（如需要）

在 Linux 中：

```bash
sudo ufw allow 5555/tcp
```

---

### 步骤 4: 启动系统（10 分钟）

#### 4.1 启动 Linux 端（两个终端）

**终端 1: 状态通信（方案 1）**

```bash
cd ~/Ros_arma3_Connection/linux_side
./start.sh
```

**终端 2: 深度接收器（方案 2）**

```bash
cd ~/Ros_arma3_Connection/linux_side
./start_depth_receiver.sh
```

#### 4.2 启动 Windows 端（两个命令行）

**命令行 1: Arma 3（方案 1）**

```cmd
"C:\...\Arma 3\arma3_x64.exe" -mod=@ROS_Bridge -noBE
```

**命令行 2: 深度发送器（方案 2）**

```cmd
cd windows_side
start_depth_sender.bat
```

**或手动指定参数**:

```cmd
python arma3_depth_sender.py ^
  --model models\depth_anything_v2_vits.onnx ^
  --linux-ip 192.168.1.100 ^
  --linux-port 5555 ^
  --input camera ^
  --source 0 ^
  --fps 10
```

**参数说明**:
- `--model`: ONNX 模型路径
- `--linux-ip`: Linux VM 的 IP 地址
- `--linux-port`: TCP 端口（默认 5555）
- `--input`: 输入源（`camera`, `video`, `arma3`）
- `--source`: 输入源参数（摄像头 ID 或视频路径）
- `--fps`: 目标帧率（默认 10）

#### 4.3 验证数据流

在 Linux 端：

```bash
# 查看深度图像话题
rostopic list | grep depth

# 查看发布频率
rostopic hz /camera/depth/image_rect_raw

# 预期输出: average rate: 10.000
```

---

### ✅ 完成！

系统现在已经运行，包含：

1. **状态通信**: Arma 3 ↔ ROS（位置、速度）
2. **深度图像**: Windows 深度估计 → Linux ROS 发布
3. **路径规划**: EGO-Planner 接收状态和深度图像

---

## 常见问题

### Q1: Linux 端部署失败？

**A**: 检查：
- ROS Noetic 是否正确安装: `rosversion -d`
- 依赖包是否安装: `dpkg -l | grep libarmadillo`
- 重新运行部署脚本: `./deploy.sh`

### Q2: Windows 端 MOD 安装失败？

**A**: 检查：
- Arma 3 路径是否正确
- 是否以**管理员身份**运行 `quick_install.bat`
- 手动创建符号链接: `mklink /D "C:\...\Arma 3\@ROS_Bridge" "C:\...\windows_side\arma3_mod\@ROS_Bridge"`

### Q3: Arma 3 无法连接到 Linux？

**A**: 检查：
- Linux IP 地址是否正确配置在 `XEH_postInit.sqf` 中
- 防火墙是否允许 TCP 5555 端口
- Linux 端是否已启动: `./start.sh`
- 网络连接: `ping <Linux_IP>`

### Q4: DirectML 不可用？

**A**: 检查：
- GPU 是否支持 DirectX 12
- 显卡驱动是否已更新
- 重新安装 `onnxruntime-directml`: `pip install --force-reinstall onnxruntime-directml==1.17.1`

### Q5: 深度图像话题无数据？

**A**: 检查：
- Windows 端深度发送器是否已启动
- Linux 端深度接收器是否已启动
- 网络连接: `ping <Linux_IP>`
- 查看日志输出是否有错误信息

### Q6: 推理速度慢（> 50ms）？

**A**: 尝试：
- 确认使用 DirectML 而非 CPU: 查看 `test_installation.py` 输出
- 使用 Small 模型而非 Large
- 降低输入分辨率（修改代码中的 `CAP_PROP_FRAME_WIDTH`）
- 关闭其他 GPU 密集型应用

### Q7: 如何切换输入源？

**A**: 修改 `start_depth_sender.bat` 中的参数：

**摄像头**:
```cmd
--input camera --source 0
```

**视频文件**:
```cmd
--input video --source "C:\path\to\video.mp4"
```

**Arma 3**（待实现）:
```cmd
--input arma3 --source 6666
```

---

## 性能监控

### 查看系统性能

**Linux 端**:

```bash
# 查看 CPU 使用率
top

# 查看 ROS 话题频率
rostopic hz /arma3/odom
rostopic hz /camera/depth/image_rect_raw

# 查看 ROS 话题延迟
rostopic delay /arma3/odom
```

**Windows 端**:

深度发送器会每 30 帧输出统计信息：

```
[DepthSender] 统计 (最近 30 帧):
  - 平均推理时间: 12.3 ms
  - 平均压缩时间: 8.1 ms
  - 平均传输时间: 15.2 ms
  - 平均总时间: 35.6 ms
  - 实际 FPS: 28.1
```

---

## 下一步

### 开发和调试

1. **修改控制逻辑**: 编辑 `linux_side/ros_nodes/ego_planner_interface.py`
2. **调整 EGO-Planner 参数**: 编辑 `linux_side/ego_planner/src/planner/plan_manage/launch/xxx.launch`
3. **添加新的 ROS 话题**: 修改 `linux_side/ros_nodes/arma3_ros_bridge.py`

### 性能优化

1. **降低延迟**: 使用更小的模型、降低分辨率、增加压缩级别
2. **提高帧率**: 使用更强的 GPU、优化代码
3. **减少带宽**: 调整压缩级别、降低分辨率

### 扩展功能

1. **多无人机**: 修改 Arma 3 任务，添加更多无人机
2. **动态建图**: 集成 SLAM 算法
3. **机器学习**: 使用 ARK 框架优化控制策略

---

## 参考文档

- **[主 README](README.md)** - 项目总览
- **[简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)** - 方案 1 详细说明
- **[深度估计模块文档](DEPTH_ESTIMATION_README.md)** - 方案 2 详细说明
- **[Windows 端部署指南](windows_side/DEPLOYMENT_GUIDE.md)** - 深度估计详细配置
- **[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)** - Arma 3 MOD 详细安装

---

## 获取帮助

如有任何问题或建议，请：

1. 查看 [常见问题](#常见问题) 部分
2. 查看相关文档
3. 提交 GitHub Issue: https://github.com/HitTheStars/Ros_arma3_Connection/issues

---

**祝您使用愉快！🚁✨**

**最后更新**: 2025-11-03  
**版本**: 2.0
