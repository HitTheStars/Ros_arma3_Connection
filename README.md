# Arma 3 <-> ROS 集成系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

一个**完整的、可运行的** Arma 3 与 ROS 集成系统，用于群体无人机路径规划。本项目将 Arma 3 游戏引擎作为高保真仿真环境，与 ROS 中的 **EGO-Planner-v2** 路径规划算法集成，并通过 **ArmaCOM 扩展**实现真正的 TCP/IP 通信，可选集成 **ARK Robotics 框架**以增强机器学习能力，实现多无人机的自主协调飞行。

## 📋 目录

- [重要更新](#重要更新)
- [系统概述](#系统概述)
- [系统架构](#系统架构)
- [功能特性](#功能特性)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [详细安装指南](#详细安装指南)
- [使用说明](#使用说明)
- [项目结构](#项目结构)
- [故障排除](#故障排除)
- [致谢](#致谢)

---

## 重要更新

### ✅ 完整的 ArmaCOM 集成

我们现在提供了**完整的、可运行的** Arma 3 MOD，包括：

- **ArmaCOM 扩展**：v2.0 Beta 2，支持 TCP/IP 通信
- **完整的 MOD 结构**：config.cpp、SQF 脚本、自动化安装
- **PBO 打包工具**：用于发布版本
- **快速安装模式**：用于开发和测试
- **详细的安装指南**：[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)

### 🚀 快速开始

**Windows 端（Arma 3）**：
```cmd
cd windows_side/arma3_mod
quick_install.bat  # 以管理员身份运行
```

**Linux 端（ROS）**：
```bash
cd linux_side
./deploy.sh
./start.sh
```

---

## 系统概述

本项目实现了 **Arma 3 游戏引擎**与 **ROS (Robot Operating System)** 之间的双向通信，用于群体无人机的路径规划和控制。系统的核心特点包括：

- **高保真仿真环境**：利用 Arma 3 的军事模拟引擎提供接近真实的飞行环境
- **真正的 TCP/IP 通信**：使用 ArmaCOM 扩展在 Arma 3 中实现 TCP 客户端
- **多视角图像采集**：同时采集 6 个视角的无人机图像，用于立体视觉处理
- **跨平台通信**：Windows (Arma 3) 与 Linux (ROS) 之间通过 TCP/IP 进行实时通信
- **完整的感知-规划-控制闭环**：从图像采集、点云生成、路径规划到控制执行的完整流程
- **EGO-Planner-v2 集成**：使用 ZJU-FAST-Lab 的 EGO-Planner-v2 进行多无人机路径规划
- **ARK 框架集成（可选）**：支持 ARK Robotics 框架，提供扩散策略、模仿学习等先进 ML 算法

### 应用场景

- 灾害救援中的物资投送
- 多无人机协同搜索
- 群体无人机编队飞行
- 路径规划算法的虚拟仿真验证

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Windows 主机                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Arma 3 游戏引擎                          │  │
│  │  - 6 架无人机模型                                    │  │
│  │  - 6 个摄像头视角（Render-to-Texture）               │  │
│  │  - ROS Bridge MOD (SQF + ArmaCOM)                    │  │
│  │    * TCP 客户端                                      │  │
│  │    * 状态数据发送                                    │  │
│  │    * 控制指令接收                                    │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │                                        │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │     完整桥接程序（基于 codingWithArma3）             │  │
│  │  - 图像捕获（PIL.ImageGrab，6 个视角）               │  │
│  │  - 立体视觉处理（视差计算）                          │  │
│  │  - 3D 点云生成                                       │  │
│  │  - TCP/IP 通信（点云数据传输）                       │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┼────────────────────────────────────────┘
                     │ TCP/IP (Port 5555)
                     │ 状态数据 + 控制指令
                     │ (图像数据可选)
┌────────────────────▼────────────────────────────────────────┐
│                  Linux 虚拟机 / 主机                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │           ROS Noetic 环境                             │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  arma3_ros_bridge 节点                         │  │  │
│  │  │  - TCP/IP 服务器                               │  │  │
│  │  │  - 状态数据接收和发布                          │  │  │
│  │  │  - 控制指令转发                                │  │  │
│  │  └────────────┬───────────────────────────────────┘  │  │
│  │               │                                        │  │
│  │  ┌────────────▼───────────────────────────────────┐  │  │
│  │  │  stereo_vision 节点（可选）                    │  │  │
│  │  │  - 多视角图像配准                              │  │  │
│  │  │  - 视差计算                                    │  │  │
│  │  │  - 点云生成                                    │  │  │
│  │  └────────────┬───────────────────────────────────┘  │  │
│  │               │                                        │  │
│  │  ┌────────────▼───────────────────────────────────┐  │  │
│  │  │  EGO-Planner-v2                                │  │  │
│  │  │  - 点云处理                                    │  │  │
│  │  │  - 路径规划                                    │  │  │
│  │  │  - 轨迹优化                                    │  │  │
│  │  └────────────┬───────────────────────────────────┘  │  │
│  │               │                                        │  │
│  │  ┌────────────▼───────────────────────────────────┐  │  │
│  │  │  ego_planner_interface 节点                    │  │  │
│  │  │  - 状态转换                                    │  │  │
│  │  │  - 目标设置                                    │  │  │
│  │  │  - 控制指令生成                                │  │  │
│  │  └────────────┬───────────────────────────────────┘  │  │
│  │               │                                        │  │
│  │  ┌────────────▼───────────────────────────────────┐  │  │
│  │  │  ARK 集成节点（可选）                          │  │  │
│  │  │  - ML 增强控制                                 │  │  │
│  │  │  - 扩散策略                                    │  │  │
│  │  │  - 模仿学习                                    │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 功能特性

### ✅ 已实现功能

#### Windows 端（Arma 3 + ArmaCOM）

- ✅ **完整的 Arma 3 MOD**
  - 基于 ArmaCOM v2.0 Beta 2 的 TCP 客户端
  - SQF 脚本实现的状态发送和控制接收
  - 自动检测场景中的所有无人机
  - 实时发送位置、速度、方向等状态数据
  - 接收并执行 MOVE 和 GOAL 指令
  - 完整的 MOD 结构（config.cpp、XEH 脚本等）

- ✅ **MOD 安装工具**
  - 快速安装脚本（`quick_install.bat`）- 开发模式
  - 完整安装脚本（`install_mod.bat`）- 发布模式
  - PBO 打包脚本（`build_mod.bat`）
  - 详细的安装指南（`MOD_INSTALLATION_GUIDE.md`）

- ✅ **图像采集（可选）**
  - 基于 codingWithArma3 项目的图像截取
  - 多线程图像采集（6 个视角）
  - JPEG 压缩
  - Python 桥接程序

#### Linux 端

- ✅ **ROS 桥接节点**
  - TCP/IP 服务器
  - 状态数据接收和发布到 ROS 话题
  - 控制指令转发
  - 图像数据接收（可选）

- ✅ **图像处理节点**
  - 多视角图像配准
  - 立体视觉处理
  - 视差计算
  - 点云生成

- ✅ **EGO-Planner-v2 集成**
  - 完整的 EGO-Planner-v2 代码
  - 自动化安装脚本
  - ROS 接口节点
  - 路径规划和轨迹优化

- ✅ **ARK 框架集成（可选）**
  - ARK 集成节点
  - 扩散策略支持
  - 模仿学习支持
  - 自动化安装脚本

#### 部署和文档

- ✅ **一键部署脚本**
  - Linux 端自动化部署（`deploy.sh`）
  - Windows 端自动化部署（`deploy.bat`）
  - 快速启动脚本（`start.sh` 和 `start.bat`）

- ✅ **完整的文档**
  - 详细的主 README
  - Arma 3 MOD 安装指南（`MOD_INSTALLATION_GUIDE.md`）
  - ARK 集成文档（`ARK_INTEGRATION.md`）
  - 故障排除指南

### 🚧 待完善功能

- ⏳ **GUI 界面**：用于选择地图中的目标点
- ⏳ **Docker 容器化**：将 ROS 和 EGO-Planner 部署在 Docker 中
- ⏳ **实时性能优化**：降低延迟，提高帧率
- ⏳ **更多传感器模拟**：激光雷达、IMU 等
- ⏳ **真实无人机测试**：Sim-to-Real 转移

---

## 环境要求

### Windows 端

- **操作系统**：Windows 10/11 (64-bit)
- **Arma 3**：完整版游戏（Steam）
- **ArmaCOM**：v2.0 Beta 2（已包含在项目中）
- **Python**：3.8 或更高版本（可选，用于图像采集）
- **BattlEye**：必须禁用（ArmaCOM 不支持 BattlEye）

### Linux 端

- **操作系统**：Ubuntu 20.04 LTS（推荐）
- **ROS**：ROS Noetic
- **Python**：3.8 或更高版本
- **依赖**：
  - cv_bridge
  - image_transport
  - pcl
  - Armadillo
  - Google Glog
  - SuiteSparse

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

**方式 1：快速安装（推荐，用于开发和测试）**

1. 打开 `windows_side/arma3_mod/` 目录
2. **右键点击** `quick_install.bat`
3. 选择 **"以管理员身份运行"**
4. 按照提示操作

**方式 2：完整安装（用于发布）**

1. 运行 `build_mod.bat` 打包 PBO 文件（需要 Arma 3 Tools）
2. 运行 `install_mod.bat` 安装 MOD

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

#### 4. （可选）启动 Python 图像采集

```batch
cd windows_side
start.bat
```

---

## 详细安装指南

### Linux 端详细安装

请参考主 README 中的 "详细安装指南" 部分，或运行：

```bash
cd linux_side
./deploy.sh
```

脚本会自动安装：
- ROS Noetic
- EGO-Planner-v2
- 所有依赖
- ROS 节点

### Windows 端详细安装

请参考：[MOD 安装指南](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md)

该指南包含：
- ArmaCOM 扩展安装
- MOD 安装（快速模式和完整模式）
- 网络配置
- 故障排除
- 开发指南

---

## 使用说明

### 基本使用流程

1. **启动 Linux 端 ROS 节点**

   ```bash
   cd linux_side
   ./start.sh
   ```

   你会看到：
   ```
   [ INFO] [1234567890.123]: Arma3 ROS Bridge started
   [ INFO] [1234567890.124]: Waiting for connection on 0.0.0.0:5555...
   ```

2. **启动 Arma 3 并加载 MOD**

   ```cmd
   arma3_x64.exe -mod=@ROS_Bridge -noBE
   ```

   在编辑器中加载 Camera 任务并预览。

3. **观察连接**

   在 Arma 3 的日志文件中（`arma3.rpt`），你应该看到：
   ```
   [ROS Bridge] Post-Init started
   [ROS Bridge] Initializing UAVs...
   [ROS Bridge] Found 6 UAVs
   [ROS Bridge] Initializing TCP client...
   [ROS Bridge] Successfully connected to ROS server!
   ```

4. **观察数据流**

   在 Linux 端：

   ```bash
   # 查看话题列表
   rostopic list

   # 查看状态数据
   rostopic echo /arma3/uav_status

   # 使用 rviz 可视化
   rviz
   ```

### 通信协议

#### Arma 3 → ROS

**状态数据**（每 0.1 秒）：
```
STATUS:UAV0:X,Y,Z,VX,VY,VZ,DIR|UAV1:X,Y,Z,VX,VY,VZ,DIR|...
```

**图像就绪通知**（每 0.5 秒）：
```
IMAGE_READY:timestamp
```

#### ROS → Arma 3

**移动指令**：
```
MOVE:X,Y,Z
```

**目标设置**：
```
GOAL:X,Y,Z
```

### 使用 EGO-Planner 进行路径规划

1. **设置目标点**

   ```bash
   rostopic pub /goal geometry_msgs/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 100.0, z: 50.0}}}"
   ```

2. **观察路径规划**

   ```bash
   rostopic echo /ego_planner/trajectory
   rviz -d ego_planner_config.rviz
   ```

3. **控制无人机**

   规划的路径会自动转换为 MOVE 指令，通过 TCP 发送回 Arma 3。

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
│   │   │   │       ├── XEH_postInit.sqf
│   │   │   │       └── functions/   # 函数库
│   │   │   └── keys/                # 签名密钥
│   │   ├── Camera.Altis/            # 任务文件夹
│   │   ├── ArmaCOM_x64.dll          # ArmaCOM 扩展
│   │   ├── quick_install.bat        # 快速安装
│   │   ├── install_mod.bat          # 完整安装
│   │   ├── build_mod.bat            # PBO 打包
│   │   └── MOD_INSTALLATION_GUIDE.md
│   └── bridge_program/              # Python 桥接程序（可选）
│       ├── arma3_bridge_enhanced.py
│       └── requirements.txt
├── linux_side/                      # Linux 端代码
│   ├── ros_nodes/                   # ROS 节点
│   │   ├── arma3_ros_bridge.py
│   │   ├── ego_planner_interface.py
│   │   ├── ark_integration.py
│   │   ├── arma3_bridge.launch
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── image_processing/            # 图像处理
│   │   └── stereo_vision.py
│   ├── ego_planner/                 # EGO-Planner-v2
│   ├── install_ego_planner.sh
│   ├── install_ark.sh
│   ├── deploy.sh
│   └── start.sh
├── docs/                            # 文档
│   └── ARK_INTEGRATION.md
└── README.md
```

---

## 故障排除

### 问题 1：MOD 未加载

**症状**：日志中没有 `[ROS Bridge]` 相关信息

**解决方案**：
1. 确保 MOD 正确安装到 Arma 3 目录
2. 确保启动参数包含 `-mod=@ROS_Bridge -noBE`
3. 检查 `arma3.rpt` 日志文件

### 问题 2：无法连接到 ROS 服务器

**症状**：`[ROS Bridge] ERROR: Failed to connect`

**解决方案**：
1. 确保 ROS 服务器已启动
2. 检查 IP 地址和端口配置
3. 检查防火墙设置
4. 测试网络连接：`telnet <Linux_IP> 5555`

### 问题 3：ArmaCOM 扩展未加载

**症状**：`[ROS Bridge] ERROR: Failed to create TCP client`

**解决方案**：
1. 确认 `ArmaCOM_x64.dll` 在 Arma 3 目录
2. 确保使用 `-noBE` 参数（禁用 BattlEye）
3. 查看 Arma 3 日志中的扩展加载信息

更多故障排除信息，请参考：[MOD 安装指南 - 故障排除](windows_side/arma3_mod/MOD_INSTALLATION_GUIDE.md#故障排除)

---

## 致谢

本项目使用或参考了以下开源项目：

- **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** - ZJU-FAST-Lab 的多无人机路径规划算法
- **[ArmaCOM](https://github.com/googleben/ArmaCOM)** - googleben 的 Arma 3 通信扩展
- **[codingWithArma3](https://github.com/Daboolu/codingWithArma3)** - Daboolu 的 Arma 3 图像截取项目
- **[ARK Robotics](https://github.com/Robotics-Ark/ark_framework)** - ARK 机器人框架
- **ROS Noetic** - Robot Operating System

特别感谢所有开源贡献者！

---

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 联系方式

如有问题或建议，请通过以下方式联系：

- **GitHub Issues**: [https://github.com/HitTheStars/Ros_arma3_Connection/issues](https://github.com/HitTheStars/Ros_arma3_Connection/issues)
- **项目主页**: [https://github.com/HitTheStars/Ros_arma3_Connection](https://github.com/HitTheStars/Ros_arma3_Connection)

---

**祝您使用愉快！Happy Flying! 🚁**
