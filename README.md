# 🚁 救援物资投送无人机群体路径规划系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python: 3.8+](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)

> **基于 Arma 3 + ROS + Depth Anything V2 的无人机群体路径规划仿真系统**

一个完整的无人机群体路径规划系统，将 **Arma 3 军事模拟平台**、**ROS 机器人操作系统**和 **Depth Anything V2 深度估计模型**相结合，实现了感知-规划-控制的完整闭环。

---

## 📖 目录

- [系统概述](#系统概述)
- [核心特性](#核心特性)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [详细文档](#详细文档)
- [技术亮点](#技术亮点)
- [常见问题](#常见问题)
- [致谢](#致谢)

---

## 系统概述

本项目为**灾害救援场景**下的无人机群体路径规划提供了一个低成本、高保真的仿真验证平台。系统的核心创新在于：

1. **高保真仿真环境**：利用 Arma 3 的 270 km² Altis 地图，提供接近真实的灾害救援场景
2. **实时深度感知**：使用 Depth Anything V2 从 RGB 图像生成深度图，无需预加载 TB 级点云数据
3. **GPU 加速推理**：在 RTX 4060 上实现 12ms 深度估计，端到端延迟 < 50ms
4. **跨平台通信**：Windows (Arma 3) ↔ Linux (ROS) 通过 TCP/IP 实时数据交换
5. **完整闭环**：RGB 截取 → 深度估计 → ROS 建图 → 路径规划 → 无人机控制

### 应用场景

- ✅ 灾害救援中的物资投送
- ✅ 多无人机协同搜索
- ✅ 群体无人机编队飞行
- ✅ 路径规划算法的虚拟仿真验证

---

## 核心特性

### ✨ 深度估计模块（核心创新）

使用 **Depth Anything V2** 单目深度估计模型，从 Arma 3 的 RGB 截图实时生成深度图。

**技术优势**：
- 🚀 **GPU 加速**：RTX 4060 + DirectML，推理时间仅 **12ms**
- ⚡ **低延迟**：端到端延迟 < **50ms**，支持 **10 FPS** 实时运行
- 💡 **单目估计**：无需双目相机或 LiDAR，仅需单视角截图
- 🌍 **大地图支持**：无需预加载 TB 级点云，仅处理局部区域
- 🎯 **动态感知**：实时检测崩塌建筑、移动车辆等动态障碍物

**性能指标**：

| 指标 | 数值 |
|------|------|
| 推理时间 | 12 ms |
| 端到端延迟 | < 50 ms |
| 运行帧率 | 10 FPS |
| 传输带宽 | ~4 Mbps |

### 🎮 Arma 3 虚拟仿真

利用 Arma 3 的高保真军事模拟引擎，提供真实的飞行环境和灾害场景。

**实现方式**：
- 📝 **SQF 脚本**：程序化控制游戏内无人机
- 🔌 **ArmaCOM 扩展**：实现 TCP 客户端，连接到 ROS
- 📸 **RGB 截图**：每秒 10 次截取无人机视角图像
- 🎯 **状态反馈**：实时发送位置、速度、姿态信息

### 🤖 ROS 路径规划

使用浙江大学 FAST-Lab 的 **EGO-Planner-v2** 算法进行多无人机路径规划。

**功能**：
- 🗺️ **实时建图**：基于深度图像构建局部地图
- 🛣️ **路径规划**：避障、轨迹优化、多机协同
- 📡 **状态订阅**：接收 Arma 3 发送的无人机状态
- 🎮 **控制发布**：发送目标点指令到 Arma 3

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Windows 端 (Arma 3)                      │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Arma 3 + ROS Bridge MOD                      │  │
│  │  - 6 架无人机模型                                    │  │
│  │  - ArmaCOM 扩展 (TCP 客户端)                         │  │
│  │  - SQF 脚本 (状态发送 + 控制接收)                    │  │
│  │  - RGB 截图 (10 FPS)                                 │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │                                       │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │      Depth Anything V2 深度估计模块                  │  │
│  │  - ONNX Runtime + DirectML (GPU 加速)                │  │
│  │  - 推理时间: 12ms                                    │  │
│  │  - PNG 压缩 + TCP 传输                               │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┼────────────────────────────────────────┘
                     │ TCP/IP (Port 5555)
                     │ 状态数据 ↑  控制指令 ↓  深度图像 ↑
┌────────────────────▼────────────────────────────────────────┐
│                    Linux 端 (ROS)                           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     ROS 桥接节点 (TCP 服务器)                        │  │
│  │  - 接收状态数据 (位置、速度、姿态)                   │  │
│  │  - 接收深度图像                                      │  │
│  │  - 发布 ROS 话题:                                    │  │
│  │    * /arma3/odom (里程计)                            │  │
│  │    * /camera/depth/image_rect_raw (深度图)           │  │
│  │  - 订阅控制指令: /arma3/cmd                          │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │ ROS 话题                              │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │         EGO-Planner-v2 路径规划                      │  │
│  │  - 订阅 /arma3/odom (无人机状态)                     │  │
│  │  - 订阅 /camera/depth/image_rect_raw (深度图)        │  │
│  │  - 实时建图和障碍物检测                              │  │
│  │  - 路径规划和轨迹优化                                │  │
│  │  - 发布 /arma3/cmd (控制指令)                        │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 快速开始

### 📋 环境要求

#### Windows 端
- **操作系统**：Windows 10/11 (64-bit)
- **Arma 3**：完整版游戏（Steam）
- **Python**：3.8 - 3.11
- **GPU**：NVIDIA RTX 4060 或支持 DirectML 的 GPU
- **依赖**：OpenCV, NumPy, ONNX Runtime DirectML

#### Linux 端
- **操作系统**：Ubuntu 20.04 LTS
- **ROS**：ROS Noetic
- **Python**：3.8+
- **依赖**：Armadillo, Google Glog, SuiteSparse

#### 网络
- Windows 主机和 Linux 虚拟机需要在同一局域网
- 确保防火墙允许 TCP 端口 5555 的通信

---

### 🚀 三步启动系统

#### 步骤 1: Linux 端部署（15 分钟）

```bash
# 克隆仓库
git clone https://github.com/HitTheStars/Ros_arma3_Connection.git
cd Ros_arma3_Connection/linux_side

# 一键部署（自动安装 ROS、依赖、EGO-Planner）
chmod +x deploy.sh
./deploy.sh

# 记下显示的 Linux IP 地址
```

#### 步骤 2: Windows 端安装（20 分钟）

**2.1 安装 Arma 3 MOD**

```cmd
# 进入 MOD 目录
cd windows_side\arma3_mod

# 右键点击 quick_install.bat，选择"以管理员身份运行"
# 按照提示完成安装

# 编辑 @ROS_Bridge\addons\ros_bridge\XEH_postInit.sqf
# 修改 ROS_ServerIP 为你的 Linux IP
ROS_ServerIP = "192.168.1.100";  // 改成你的 Linux VM IP
```

**2.2 安装深度估计模块**

```cmd
# 进入 Windows 端目录
cd windows_side

# 安装 Python 依赖
pip install -r requirements.txt

# 测试安装
python test_installation.py

# 下载 Depth Anything V2 模型
# 访问 https://huggingface.co/depth-anything/Depth-Anything-V2-Small-ONNX
# 下载 depth_anything_v2_vits.onnx 到 models/ 目录
```

#### 步骤 3: 启动系统（5 分钟）

**3.1 启动 Linux 端**

```bash
cd ~/Ros_arma3_Connection/linux_side

# 启动深度接收器和 ROS 桥接
./start_depth_receiver.sh
```

**3.2 启动 Windows 端**

```cmd
# 终端 1: 启动 Arma 3
"C:\...\Arma 3\arma3_x64.exe" -mod=@ROS_Bridge -noBE

# 在 Arma 3 中加载任务: Editor → Altis → Load → Camera → Preview

# 终端 2: 启动深度发送器
cd windows_side
start_depth_sender.bat
```

**3.3 验证运行**

在 Linux 端：

```bash
# 查看 ROS 话题
rostopic list

# 查看深度图像发布频率
rostopic hz /camera/depth/image_rect_raw
# 预期输出: average rate: 10.000

# 查看无人机状态
rostopic echo /arma3/odom
```

---

## 详细文档

### 核心文档
- **[深度估计模块详细文档](DEPTH_ESTIMATION_README.md)** - 技术原理、性能优化、故障排除
- **[Windows 端部署指南](windows_side/DEPLOYMENT_GUIDE.md)** - 详细的安装和配置步骤
- **[Arma 3 MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)** - MOD 的详细安装说明
- **[模型下载指南](windows_side/models/MODEL_DOWNLOAD_GUIDE.md)** - Depth Anything V2 模型下载

### 项目结构

```
Ros_arma3_Connection/
├── README.md                        # 项目总览（本文件）
├── DEPTH_ESTIMATION_README.md       # 深度估计详细文档
├── windows_side/                    # Windows 端代码
│   ├── arma3_mod/                   # Arma 3 MOD
│   │   ├── @ROS_Bridge/             # MOD 文件夹
│   │   ├── Camera.Altis/            # 任务文件
│   │   ├── ArmaCOM_x64.dll          # ArmaCOM 扩展
│   │   ├── quick_install.bat        # 快速安装脚本
│   │   └── MOD_INSTALLATION_GUIDE.md
│   ├── arma3_depth_sender.py        # 深度估计发送器
│   ├── requirements.txt             # Python 依赖
│   ├── test_installation.py         # 安装测试
│   ├── start_depth_sender.bat       # 启动脚本
│   ├── DEPLOYMENT_GUIDE.md          # 部署指南
│   └── models/                      # ONNX 模型目录
│       └── MODEL_DOWNLOAD_GUIDE.md
├── linux_side/                      # Linux 端代码
│   ├── ros_nodes/                   # ROS 节点
│   │   ├── arma3_ros_bridge.py      # ROS 桥接节点
│   │   ├── arma3_depth_receiver.py  # 深度接收器
│   │   ├── arma3_depth_receiver.launch
│   │   ├── ego_planner_interface.py
│   │   └── arma3_bridge.launch
│   ├── ego_planner/                 # EGO-Planner-v2
│   ├── deploy.sh                    # 一键部署
│   ├── install_ego_planner.sh       # EGO-Planner 安装
│   └── start_depth_receiver.sh      # 启动脚本
└── LICENSE
```

---

## 技术亮点

### 1. 解决大地图点云预加载问题

**问题**：Arma 3 Altis 地图面积 270 km²，如果以 0.1m 分辨率预加载全地图点云：
- 点数：27,000,000,000 点
- 数据量：> 1 TB
- **结论**：内存和磁盘无法承载

**我们的方案**：使用 Depth Anything V2 实时生成深度图
- 仅处理无人机周围的局部区域（视野范围内）
- 数据量：~50 KB/帧
- **结论**：大幅降低计算和存储开销

### 2. GPU 加速实现低延迟

**对比**：

| 方案 | 推理时间 | 端到端延迟 |
|------|---------|-----------|
| CPU 推理 | 60-80 ms | > 100 ms |
| **GPU 加速（我们）** | **12 ms** | **< 50 ms** |

**技术栈**：ONNX Runtime + DirectML + RTX 4060

### 3. 跨平台通信集成

**挑战**：Arma 3 是 Windows 游戏，ROS 运行在 Linux

**解决方案**：
- Windows 端：使用 ArmaCOM 扩展创建 TCP 客户端
- Linux 端：ROS 节点作为 TCP 服务器
- 数据格式：文本协议（状态）+ 二进制协议（深度图）

### 4. 完整的感知-规划-控制闭环

```
Arma 3 RGB 截图
    ↓
Depth Anything V2 深度估计 (12ms)
    ↓
TCP/IP 传输 (15ms)
    ↓
ROS 深度图像发布
    ↓
EGO-Planner 实时建图
    ↓
路径规划和轨迹优化
    ↓
控制指令发送到 Arma 3
    ↓
无人机执行 MOVE 指令
```

---

## 常见问题

### Q1: 为什么不使用预加载的静态地图？

**A**: Altis 地图 270 km²，预加载全地图点云将达到 TB 级别，内存和磁盘无法承载。而且灾害现场有动态障碍物（崩塌建筑、移动车辆），静态地图无法应对。深度估计模块仅处理无人机周围的局部区域，大幅降低了计算和存储开销。

### Q2: 为什么选择单目深度估计而非双目？

**A**: Arma 3 不支持原生双目相机，单目深度估计更简单、延迟更低。Depth Anything V2 的精度已足够用于路径规划，而且不需要复杂的相机标定或昂贵的硬件设备。

### Q3: 延迟 50ms 是否足够？

**A**: 对于灾害救援任务，50ms 延迟完全可接受。无人机飞行速度通常 < 10 m/s，50ms 内移动距离 < 0.5m，在安全范围内。而且我们的实际延迟约 40ms，还有优化空间。

### Q4: GPU 加速的必要性？

**A**: CPU 推理时间约 60-80ms，无法满足 10 FPS 的实时性要求。GPU 加速将推理时间降低到 12ms，提升了 5-10 倍，确保了系统的实时性。

### Q5: Arma 3 无法连接到 Linux？

**A**: 检查：
- Linux IP 地址是否正确配置在 `XEH_postInit.sqf` 中
- 防火墙是否允许 TCP 5555 端口
- Linux 端是否已启动: `./start_depth_receiver.sh`
- 网络连接: `ping <Linux_IP>`

### Q6: 深度图像话题无数据？

**A**: 检查：
- Windows 端深度发送器是否已启动
- Linux 端深度接收器是否已启动
- 网络连接: `ping <Linux_IP>`
- 查看日志输出是否有错误信息

### Q7: DirectML 不可用？

**A**: 检查：
- GPU 是否支持 DirectX 12
- 显卡驱动是否已更新
- 重新安装 `onnxruntime-directml`: `pip install --force-reinstall onnxruntime-directml==1.17.1`

---

## 致谢

本项目基于以下开源项目：

- **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** - ZJU-FAST-Lab 的多无人机路径规划算法
- **[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)** - EGO-Planner 的群体版本
- **[Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2)** - 单目深度估计模型
- **[ArmaCOM](https://github.com/googleben/ArmaCOM)** - Arma 3 的通信扩展
- **[codingWithArma3](https://github.com/Daboolu/codingWithArma3)** - Arma 3 图像采集项目

感谢所有贡献者！

---

## 许可证

MIT License

---

## 联系方式

如有任何问题或建议，请提交 Issue 或 Pull Request。

**项目地址**: https://github.com/HitTheStars/Ros_arma3_Connection

---

**祝您使用愉快！🚁✨**

**最后更新**: 2025-11-03  
**版本**: 3.0（深度估计架构）
