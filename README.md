# Arma 3 <-> ROS 集成系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

一个**完整的、可运行的** Arma 3 与 ROS 集成系统，用于群体无人机路径规划。本项目将 Arma 3 游戏引擎作为高保真仿真环境，与 ROS 中的 **EGO-Planner-v2** 路径规划算法集成，并通过 **ArmaCOM 扩展**实现真正的 TCP/IP 通信，实现多无人机的自主协调飞行。

## 📋 目录

- [重要更新](#重要更新)
- [系统概述](#系统概述)
- [系统架构](#系统架构)
- [功能特性](#功能特性)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [详细文档](#详细文档)
- [项目结构](#项目结构)
- [致谢](#致谢)

---

## ⚠️ 重要更新

### 🎯 两种架构方案（2025-11-03）

本项目提供两种架构方案，您可以根据需求选择：

#### 方案 1: 简化架构（推荐用于快速验证）

**只使用 ArmaCOM 进行状态和控制通信，不需要图像处理。**

**优势**：
- ✅ 延迟降低 **90%+**（5-20 ms vs 120-300 ms）
- ✅ 数据量降低 **99.96%**（200 B vs 500 KB）
- ✅ 实现复杂度降低 **75%**（500 行 vs 2000 行）
- ✅ 稳定性更高，不依赖图像采集

**适用场景**：
- 使用预加载的静态地图（PCD 点云）
- 环境已知的路径规划验证
- 快速原型开发和测试

**详细说明**：[简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)

#### 方案 2: 深度估计架构（推荐用于动态环境）

**使用 Depth Anything V2 从 RGB 图像生成深度图，支持动态环境感知。**

**优势**：
- ✅ GPU 加速深度估计（RTX 4060 + DirectML，12ms 推理）
- ✅ 端到端延迟 < 50ms，支持 10 FPS 实时运行
- ✅ 单目深度估计，不需要双目相机或 LiDAR
- ✅ 支持动态障碍物检测和实时建图

**适用场景**：
- 动态环境的实时障碍物检测
- 未知环境的探索和建图
- 需要视觉感知的完整闭环系统

**详细说明**：[深度估计模块文档](DEPTH_ESTIMATION_README.md)

---

## 系统概述

本项目实现了 **Arma 3 游戏引擎**与 **ROS (Robot Operating System)** 之间的双向通信，用于群体无人机的路径规划和控制。系统的核心特点包括：

- **高保真仿真环境**：利用 Arma 3 的军事模拟引擎提供接近真实的飞行环境
- **真正的 TCP/IP 通信**：使用 ArmaCOM 扩展在 Arma 3 中实现 TCP 客户端
- **跨平台通信**：Windows (Arma 3) 与 Linux (ROS) 之间通过 TCP/IP 进行实时通信
- **完整的感知-规划-控制闭环**：从状态感知、路径规划到控制执行的完整流程
- **EGO-Planner-v2 集成**：使用 ZJU-FAST-Lab 的 EGO-Planner-v2 进行多无人机路径规划
- **简洁高效**：只使用必要的通信，避免复杂的图像处理

### 应用场景

- 灾害救援中的物资投送
- 多无人机协同搜索
- 群体无人机编队飞行
- 路径规划算法的虚拟仿真验证

---

## 系统架构

### 简化架构（推荐）

```
┌─────────────────────────────────────────────────────────────┐
│                    Windows 端 (Arma 3)                      │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Arma 3 + ROS Bridge MOD                      │  │
│  │  - 6 架无人机模型                                    │  │
│  │  - ArmaCOM 扩展 (TCP 客户端)                         │  │
│  │  - SQF 脚本                                          │  │
│  │    * 每 0.1 秒发送状态数据 (位置、速度、方向)        │  │
│  │    * 接收控制指令 (MOVE, GOAL)                       │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┼────────────────────────────────────────┘
                     │ TCP/IP (Port 5555)
                     │ 状态数据 ↑  控制指令 ↓
┌────────────────────▼────────────────────────────────────────┐
│                    Linux 端 (ROS)                           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     ROS 桥接节点 (TCP 服务器)                        │  │
│  │  - 接收状态数据                                      │  │
│  │  - 发布 ROS 话题 (/arma3/odom, /arma3/state)         │  │
│  │  - 订阅控制指令 (/arma3/cmd)                         │  │
│  │  - 发送控制指令到 Arma 3                             │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │ ROS 话题                              │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │         EGO-Planner-swarm                            │  │
│  │  - 订阅 /arma3/odom (无人机状态)                     │  │
│  │  - 路径规划 (不需要深度图像!)                        │  │
│  │  - 发布 /arma3/cmd (控制指令)                        │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**详细说明**：[简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)

---

## 功能特性

### ✅ 已实现功能

#### Windows 端（Arma 3 + ArmaCOM）

- ✅ **完整的 Arma 3 MOD**
  - 基于 ArmaCOM v2.0 Beta 2 的 TCP 客户端
  - SQF 脚本实现的状态发送和控制接收
  - 自动检测场景中的所有无人机
  - 实时发送位置、速度、方向等状态数据（10 Hz）
  - 接收并执行 MOVE 和 GOAL 指令
  - 完整的 MOD 结构（config.cpp、XEH 脚本等）

- ✅ **MOD 安装工具**
  - 快速安装脚本（`quick_install.bat`）- 开发模式
  - 完整安装脚本（`install_mod.bat`）- 发布模式
  - PBO 打包脚本（`build_mod.bat`）
  - 详细的安装指南（`MOD_INSTALLATION_GUIDE.md`）

#### Linux 端

- ✅ **ROS 桥接节点**
  - TCP/IP 服务器
  - 状态数据接收和发布到 ROS 话题
  - 控制指令转发
  - 低延迟（5-20 ms）

- ✅ **EGO-Planner-v2 集成**
  - 完整的 EGO-Planner-v2 代码
  - 自动化安装脚本
  - ROS 接口节点
  - 路径规划和轨迹优化
  - 支持 CPU 模式（不需要深度图像）

#### 部署和文档

- ✅ **一键部署脚本**
  - Linux 端自动化部署（`deploy.sh`）
  - Windows 端自动化部署（`deploy.bat`）
  - 快速启动脚本（`start.sh` 和 `start.bat`）

- ✅ **完整的文档**
  - 详细的主 README
  - Arma 3 MOD 安装指南（`MOD_INSTALLATION_GUIDE.md`）
  - 简化架构文档（`SIMPLIFIED_ARCHITECTURE.md`）
  - 故障排除指南

### ✅ 深度估计模块（可选增强）

- ✅ **Windows 端深度估计**
  - Depth Anything V2 ONNX 模型
  - DirectML GPU 加速（RTX 4060）
  - 推理时间 ~12ms
  - 多输入源支持（摄像头、视频、Arma 3）
  - PNG 压缩和 TCP 传输

- ✅ **Linux 端深度接收**
  - TCP 服务器（端口 5555）
  - ROS 深度图像发布节点
  - 完全兼容 EGO-Planner 输入
  - 实时性能监控

- ✅ **完整部署文档**
  - Windows 端详细安装指南
  - 模型下载和配置说明
  - 性能优化和故障排除

**详细说明**：[深度估计模块文档](DEPTH_ESTIMATION_README.md)

### 🚧 其他可选功能

- ⏳ **ARK 框架集成**：机器学习增强控制（可选）
- ⏳ **GUI 界面**：用于选择地图中的目标点
- ⏳ **Docker 容器化**：将 ROS 和 EGO-Planner 部署在 Docker 中

---

## 环境要求

### Windows 端

**基础要求**：
- **操作系统**：Windows 10/11 (64-bit)
- **Arma 3**：完整版游戏（Steam）
- **ArmaCOM**：v2.0 Beta 2（已包含在项目中）
- **BattlEye**：必须禁用（ArmaCOM 不支持 BattlEye）

**深度估计模块额外要求**（可选）：
- **Python**：3.8 - 3.11
- **GPU**：NVIDIA RTX 4060 或支持 DirectML 的 GPU
- **依赖**：OpenCV, NumPy, ONNX Runtime DirectML

### Linux 端

- **操作系统**：Ubuntu 20.04 LTS（推荐）
- **ROS**：ROS Noetic
- **Python**：3.8 或更高版本
- **依赖**：Armadillo、Google Glog、SuiteSparse

### 网络

- Windows 主机和 Linux 虚拟机/主机需要在同一局域网
- 确保防火墙允许 TCP 端口 5555 的通信

---

## 快速开始

### 第一步：Linux 端部署

```bash
# 克隆仓库
git clone https://github.com/HitTheStars/Ros_arma3_Connection.git
cd Ros_arma3_Connection/linux_side

# 运行一键部署脚本
chmod +x deploy.sh
./deploy.sh

# 部署完成后，记下显示的 Linux IP 地址
```

### 第二步：Windows 端 MOD 安装

**快速安装（推荐）**

1. 打开 `windows_side/arma3_mod/` 目录
2. **右键点击** `quick_install.bat`
3. 选择 **"以管理员身份运行"**
4. 按照提示操作

详细说明请参考：[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)

### 第三步：配置 MOD

编辑 `@ROS_Bridge/addons/ros_bridge/XEH_postInit.sqf`：

```sqf
ROS_ServerIP = "192.168.1.100";  // 改成你的 Linux VM IP
ROS_ServerPort = "5555";
```

### 第四步：启动系统

#### 1. 启动 Linux 端

```bash
cd linux_side
./start.sh
```

#### 2. 启动 Arma 3

**使用启动器**：
1. 打开 Arma 3 启动器
2. MODs → 勾选 **@ROS_Bridge**
3. Parameters → Additional Parameters → 添加 `-noBE`
4. Play

**使用命令行**：
```cmd
"C:\...\Arma 3\arma3_x64.exe" -mod=@ROS_Bridge -noBE
```

#### 3. 加载任务

- Editor → Altis → Load → Camera
- Preview (Ctrl+P)

---

## 详细文档

### 核心文档
- **[快速启动指南](QUICK_START_GUIDE.md)** - 一站式启动指导（推荐）
- **[简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)** - 方案 1：仅状态通信
- **[深度估计模块文档](DEPTH_ESTIMATION_README.md)** - 方案 2：深度感知
- **[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)** - Windows 端详细安装

### 可选功能文档
- **[深度估计部署指南](windows_side/DEPLOYMENT_GUIDE.md)** - Windows 端深度估计详细配置
- **[ARK 集成文档](docs/ARK_INTEGRATION.md)** - 可选的 ML 增强
- **[深度图像桥接](windows_side/bridge_program/DEPTH_BRIDGE_GUIDE.md)** - 可选的图像处理
- **[完整桥接指南](windows_side/bridge_program/COMPLETE_BRIDGE_GUIDE.md)** - 可选的图像采集

---

## 项目结构

```
Ros_arma3_Connection/
├── README.md                        # 项目总览（本文件）
├── QUICK_START_GUIDE.md             # 快速启动指南
├── DEPTH_ESTIMATION_README.md       # 深度估计模块文档
├── windows_side/                    # Windows 端代码
│   ├── arma3_mod/                   # Arma 3 MOD
│   │   ├── @ROS_Bridge/             # MOD 文件夹
│   │   │   ├── mod.cpp              # MOD 元数据
│   │   │   ├── addons/
│   │   │   │   └── ros_bridge/      # MOD 代码
│   │   │   │       ├── config.cpp   # MOD 配置
│   │   │   │       ├── XEH_preInit.sqf
│   │   │   │       └── XEH_postInit.sqf  # TCP 客户端
│   │   ├── Camera.Altis/            # 任务文件夹
│   │   ├── ArmaCOM_x64.dll          # ArmaCOM 扩展
│   │   ├── quick_install.bat        # 快速安装
│   │   └── MOD_INSTALLATION_GUIDE.md
│   ├── arma3_depth_sender.py        # 深度估计发送器（可选）
│   ├── requirements.txt             # Python 依赖（深度估计）
│   ├── test_installation.py         # 安装测试（深度估计）
│   ├── start_depth_sender.bat       # 启动脚本（深度估计）
│   ├── DEPLOYMENT_GUIDE.md          # 深度估计部署指南
│   ├── models/                      # ONNX 模型目录
│   │   └── MODEL_DOWNLOAD_GUIDE.md  # 模型下载指南
│   └── bridge_program/              # Python 桥接程序（旧版）
├── linux_side/                      # Linux 端代码
│   ├── ros_nodes/                   # ROS 节点
│   │   ├── arma3_ros_bridge.py      # TCP 服务器（状态通信）
│   │   ├── arma3_depth_receiver.py  # 深度接收器（可选）
│   │   ├── arma3_depth_receiver.launch  # Launch 文件（深度）
│   │   ├── ego_planner_interface.py # EGO-Planner 接口
│   │   └── arma3_bridge.launch
│   ├── ego_planner/                 # EGO-Planner-v2
│   ├── deploy.sh                    # 一键部署
│   ├── start.sh                     # 快速启动（状态通信）
│   └── start_depth_receiver.sh      # 启动脚本（深度接收）
└── docs/                            # 文档
    ├── SIMPLIFIED_ARCHITECTURE.md   # 简化架构（方案 1）
    └── ARK_INTEGRATION.md           # ARK 集成（可选）
```

---

## 致谢

本项目基于以下开源项目：

- **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** - ZJU-FAST-Lab 的多无人机路径规划算法
- **[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)** - EGO-Planner 的群体版本
- **[ArmaCOM](https://github.com/googleben/ArmaCOM)** - Arma 3 的通信扩展
- **[Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2)** - 单目深度估计模型
- **[codingWithArma3](https://github.com/Daboolu/codingWithArma3)** - Arma 3 图像采集项目
- **[ARK Robotics](https://github.com/Robotics-Ark/ark_framework)** - 机器学习框架（可选）

感谢所有贡献者！

---

## 许可证

MIT License

---

## 联系方式

如有任何问题或建议，请提交 Issue 或 Pull Request。

祝您答辩顺利！🚁✨
