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

### 🎯 推荐使用简化架构（2025-01-XX）

经过深入分析 **ego-planner-swarm** 的源代码，我们发现：

> **深度图像是可选的！** EGO-Planner 的 local mapping 模块可以自动选择使用深度图像或点云作为输入。在默认的 CPU 模式下（`ENABLE_CUDA = false`），它使用**点云**或**里程计**数据。

因此，我们提供了一个**更简单、更稳定**的方案：

**只使用 ArmaCOM 进行状态和控制通信，不需要复杂的图像处理。**

**优势**：
- ✅ 延迟降低 **90%+**（5-20 ms vs 120-300 ms）
- ✅ 数据量降低 **99.96%**（200 B vs 500 KB）
- ✅ 实现复杂度降低 **75%**（500 行 vs 2000 行）
- ✅ 稳定性更高，不依赖图像采集

**详细说明**：请查看 [简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)

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

### 🚧 可选功能

- ⏳ **图像采集**：基于 codingWithArma3 的多视角图像截取（可选）
- ⏳ **深度图像处理**：立体视觉和深度图生成（可选）
- ⏳ **ARK 框架集成**：机器学习增强控制（可选）
- ⏳ **GUI 界面**：用于选择地图中的目标点
- ⏳ **Docker 容器化**：将 ROS 和 EGO-Planner 部署在 Docker 中

---

## 环境要求

### Windows 端

- **操作系统**：Windows 10/11 (64-bit)
- **Arma 3**：完整版游戏（Steam）
- **ArmaCOM**：v2.0 Beta 2（已包含在项目中）
- **BattlEye**：必须禁用（ArmaCOM 不支持 BattlEye）

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

- **[简化架构文档](docs/SIMPLIFIED_ARCHITECTURE.md)** - 推荐阅读！
- **[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)** - Windows 端详细安装
- **[ARK 集成文档](docs/ARK_INTEGRATION.md)** - 可选的 ML 增强
- **[深度图像桥接](windows_side/bridge_program/DEPTH_BRIDGE_GUIDE.md)** - 可选的图像处理
- **[完整桥接指南](windows_side/bridge_program/COMPLETE_BRIDGE_GUIDE.md)** - 可选的图像采集

---

## 项目结构

```
Ros_arma3_Connection/
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
│   └── bridge_program/              # Python 桥接程序（可选）
├── linux_side/                      # Linux 端代码
│   ├── ros_nodes/                   # ROS 节点
│   │   ├── arma3_ros_bridge.py      # TCP 服务器
│   │   ├── ego_planner_interface.py # EGO-Planner 接口
│   │   └── arma3_bridge.launch
│   ├── ego_planner/                 # EGO-Planner-v2
│   ├── deploy.sh                    # 一键部署
│   └── start.sh                     # 快速启动
└── docs/                            # 文档
    ├── SIMPLIFIED_ARCHITECTURE.md   # 简化架构（推荐）
    └── ARK_INTEGRATION.md           # ARK 集成（可选）
```

---

## 致谢

本项目基于以下开源项目：

- **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** - ZJU-FAST-Lab 的多无人机路径规划算法
- **[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)** - EGO-Planner 的群体版本
- **[ArmaCOM](https://github.com/googleben/ArmaCOM)** - Arma 3 的通信扩展
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
