# Arma 3 与 ROS 集成系统 (Ros_arma3_Connection)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

一个完整的、可运行的 Arma 3 与 ROS 集成系统，用于群体无人机路径规划。本项目将 Arma 3 游戏引擎作为高保真仿真环境，与 ROS 中的 EGO-Planner-v2 路径规划算法集成，实现多无人机的自主协调飞行。

## 📋 目录

- [系统概述](#系统概述)
- [系统架构](#系统架构)
- [功能特性](#功能特性)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [详细安装指南](#详细安装指南)
- [使用说明](#使用说明)
- [配置说明](#配置说明)
- [故障排除](#故障排除)
- [开发指南](#开发指南)
- [致谢](#致谢)

---

## 系统概述

本项目实现了 **Arma 3 游戏引擎**与 **ROS (Robot Operating System)** 之间的双向通信，用于群体无人机的路径规划和控制。系统的核心特点包括：

- **高保真仿真环境**：利用 Arma 3 的军事模拟引擎提供接近真实的飞行环境
- **多视角图像采集**：同时采集 6 个视角的无人机图像，用于立体视觉处理
- **跨平台通信**：Windows (Arma 3) 与 Linux (ROS) 之间通过 TCP/IP 进行实时通信
- **完整的感知-规划-控制闭环**：从图像采集、点云生成、路径规划到控制执行的完整流程

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
│  │  - 3 架无人机模型                                    │  │
│  │  - 6 个摄像头视角                                    │  │
│  │  - SQF 脚本控制                                      │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↕                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Windows 端桥接程序 (Python)                  │  │
│  │  - 图像采集 (6 个视角)                              │  │
│  │  - JPEG 压缩                                         │  │
│  │  - TCP 客户端                                        │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                        ↕ TCP/IP (Port 5555)
┌─────────────────────────────────────────────────────────────┐
│                  Linux 虚拟机 (ROS)                          │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         ROS 桥接节点 (arma3_ros_bridge)             │  │
│  │  - TCP 服务器                                       │  │
│  │  - 图像接收和发布                                   │  │
│  │  - 控制指令转发                                     │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↕                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │      立体视觉处理 (stereo_vision)                   │  │
│  │  - 多视角图像配准                                   │  │
│  │  - 视差计算                                         │  │
│  │  - 点云生成                                         │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↕                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │      EGO-Planner 接口 (ego_planner_interface)       │  │
│  │  - 状态转换                                         │  │
│  │  - 目标设置                                         │  │
│  │  - 可视化                                           │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↕                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │      EGO-Planner-v2 (路径规划)                      │  │
│  │  - 轨迹生成                                         │  │
│  │  - 碰撞避免                                         │  │
│  │  - 多无人机协调                                     │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 功能特性

### ✅ 已实现功能

- **多视角图像采集**：同时采集 6 个视角的无人机图像
- **图像压缩和传输**：JPEG 压缩，TCP/IP 传输
- **立体视觉处理**：从多视角图像生成 3D 点云
- **ROS 集成**：完整的 ROS 节点和话题通信
- **EGO-Planner 接口**：与 EGO-Planner-v2 的无缝集成
- **实时可视化**：RViz 可视化无人机状态和点云

### 🚧 待完善功能

- **ArmaCOM 集成**：通过 ArmaCOM 扩展实现更高效的 Arma 3 通信
- **GUI 界面**：用于设置目标点和监控系统状态
- **Docker 部署**：将 EGO-Planner 部署在 Docker 容器中
- **真实无人机测试**：Sim-to-Real 转移

---

## 环境要求

### Windows 端

| 软件 | 版本 | 说明 |
|------|------|------|
| **Windows** | 10/11 | 64 位操作系统 |
| **Arma 3** | 最新版 | Steam 版本或独立版本 |
| **Python** | 3.8+ | 用于桥接程序 |
| **Pillow** | 10.0.0+ | Python 图像处理库 |

### Linux 端

| 软件 | 版本 | 说明 |
|------|------|------|
| **Ubuntu** | 20.04 LTS | 推荐使用 LTS 版本 |
| **ROS** | Noetic | ROS 1 版本 |
| **Python** | 3.8+ | ROS 节点开发 |
| **OpenCV** | 4.2+ | 图像处理 |
| **EGO-Planner-v2** | 最新版 | 路径规划算法 |

### 网络要求

- Windows 主机和 Linux 虚拟机需要在**同一局域网**内
- 推荐使用**桥接模式**配置虚拟机网络
- 确保防火墙允许 **TCP 端口 5555** 的通信

---

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/HitTheStars/Ros_arma3_Connection.git
cd Ros_arma3_Connection
```

### 2. Windows 端设置

```powershell
# 安装 Python 依赖
cd windows_side/bridge_program
pip install -r requirements.txt

# 配置 Arma 3 MOD
# 将 windows_side/arma3_mod/init.sqf 复制到 Arma 3 任务文件夹
```

### 3. Linux 端设置

```bash
# 创建 ROS 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 复制 ROS 节点
cp -r /path/to/Ros_arma3_Connection/linux_side/ros_nodes ~/catkin_ws/src/arma3_ros_bridge

# 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. 运行系统

**Linux 端：**
```bash
# 启动 ROS 节点
roslaunch arma3_ros_bridge arma3_bridge.launch
```

**Windows 端：**
```powershell
# 启动 Arma 3 并加载任务
# 然后运行桥接程序
python arma3_bridge.py --host <Linux_VM_IP>
```

---

## 详细安装指南

### Windows 端安装

#### 1. 安装 Python 和依赖

```powershell
# 下载并安装 Python 3.8+ (https://www.python.org/downloads/)

# 安装依赖
cd windows_side/bridge_program
pip install -r requirements.txt
```

#### 2. 配置 Arma 3 MOD

```powershell
# 复制 MOD 文件到 Arma 3 任务文件夹
# 路径示例：C:\Users\YourName\Documents\Arma 3\missions\

# 1. 创建新任务文件夹（例如：UAV_Mission.Altis）
# 2. 将 windows_side/arma3_mod/init.sqf 复制到任务文件夹中
```

#### 3. 启动 Arma 3 并加载任务

1. 启动 Arma 3
2. 点击 **Editor** (编辑器)
3. 选择地图 **Altis**
4. 点击 **Open** 打开您创建的任务
5. 点击 **Preview** 预览任务

#### 4. 配置网络

```powershell
# 查看 Linux 虚拟机的 IP 地址
# 在 Linux 中运行：ip addr show

# 编辑 arma3_bridge.py，修改 ROS 主机 IP
# 默认值：192.168.1.100
```

#### 5. 运行桥接程序

```powershell
cd windows_side/bridge_program
python arma3_bridge.py --host <Linux_VM_IP> --port 5555
```

---

### Linux 端安装

#### 1. 安装 ROS Noetic

```bash
# 添加 ROS 源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加密钥
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安装 ROS
sudo apt update
sudo apt install ros-noetic-desktop-full

# 初始化 rosdep
sudo rosdep init
rosdep update

# 配置环境
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2. 创建 ROS 工作空间

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆本项目
git clone https://github.com/HitTheStars/Ros_arma3_Connection.git

# 将 ROS 节点复制到工作空间
cp -r Ros_arma3_Connection/linux_side/ros_nodes ~/catkin_ws/src/arma3_ros_bridge
cp Ros_arma3_Connection/linux_side/image_processing/stereo_vision.py ~/catkin_ws/src/arma3_ros_bridge/scripts/

# 设置可执行权限
chmod +x ~/catkin_ws/src/arma3_ros_bridge/arma3_ros_bridge.py
chmod +x ~/catkin_ws/src/arma3_ros_bridge/ego_planner_interface.py
chmod +x ~/catkin_ws/src/arma3_ros_bridge/scripts/stereo_vision.py

# 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make

# 配置环境
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

#### 3. 安装 EGO-Planner-v2

```bash
# 克隆 EGO-Planner-v2
cd ~/catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git

# 安装依赖
sudo apt install ros-noetic-nlopt

# 编译
cd ~/catkin_ws
catkin_make
```

#### 4. 安装 Python 依赖

```bash
# 安装 OpenCV 和其他依赖
sudo apt install python3-opencv python3-numpy python3-pil python3-cv-bridge
pip3 install opencv-python numpy pillow
```

#### 5. 配置网络

```bash
# 查看本机 IP 地址
ip addr show

# 确保防火墙允许 TCP 端口 5555
sudo ufw allow 5555/tcp
```

---

## 使用说明

### 启动系统

#### 步骤 1：启动 Linux 端 ROS 节点

```bash
# 在 Linux 虚拟机中

# 启动 ROS Master
roscore

# 在新终端中启动 Arma3 桥接节点
roslaunch arma3_ros_bridge arma3_bridge.launch

# 节点将等待 Arma 3 的连接...
```

#### 步骤 2：启动 Arma 3 任务

1. 在 Windows 中启动 Arma 3
2. 加载并预览您创建的 UAV 任务
3. 等待任务初始化完成（会显示 "UAV initialization complete!"）

#### 步骤 3：启动 Windows 端桥接程序

```powershell
# 在 Windows PowerShell 中
cd windows_side/bridge_program
python arma3_bridge.py --host <Linux_VM_IP> --port 5555
```

#### 步骤 4：启动 EGO-Planner

```bash
# 在 Linux 中启动 EGO-Planner
roslaunch ego_planner single_run_in_sim.launch
```

#### 步骤 5：可视化

```bash
# 在新终端中启动 RViz
rviz
```

在 RViz 中添加以下显示：
- **Image**：显示来自 Arma 3 的图像 (`/arma3/camera_0/image_raw`)
- **PointCloud2**：显示生成的点云 (`/arma3/pointcloud`)
- **PoseStamped**：显示无人机位置 (`/arma3/uav_0/pose`)
- **MarkerArray**：显示目标点 (`/ego_planner/goals`)

---

## 配置说明

### 修改无人机数量

在 `windows_side/arma3_mod/init.sqf` 中：

```sqf
// 修改无人机数量
_uavCount = 3;  // 改为您需要的数量
```

在 `linux_side/ros_nodes/arma3_bridge.launch` 中：

```xml
<arg name="num_uavs" default="3"/>  <!-- 改为相同的数量 -->
```

### 修改图像采集区域

在 `windows_side/bridge_program/arma3_bridge.py` 中：

```python
self.regions = [
    (0, 255, 455, 511),      # 区域 1 (左, 上, 右, 下)
    (455, 255, 911, 511),    # 区域 2
    # ... 根据您的屏幕分辨率调整
]
```

### 修改通信端口

在 `windows_side/bridge_program/arma3_bridge.py` 中：

```python
bridge = Arma3Bridge(ros_host='192.168.1.100', ros_port=5555)
```

在 `linux_side/ros_nodes/arma3_bridge.launch` 中：

```xml
<arg name="listen_port" default="5555"/>
```

---

## 故障排除

### 问题 1：无法连接到 ROS

**症状**：Windows 端显示 "Failed to connect to ROS"

**解决方案**：
1. 检查 Linux 虚拟机的 IP 地址是否正确
2. 确保防火墙允许 TCP 端口 5555
3. 确保虚拟机网络模式为桥接模式
4. 使用 `ping` 命令测试网络连通性

```bash
# 在 Windows 中
ping <Linux_VM_IP>

# 在 Linux 中
ping <Windows_IP>
```

### 问题 2：图像无法显示

**症状**：RViz 中看不到图像

**解决方案**：
1. 检查图像采集区域是否正确
2. 确保 Arma 3 窗口没有被其他窗口遮挡
3. 检查 ROS 话题是否有数据：

```bash
rostopic list
rostopic echo /arma3/camera_0/image_raw
```

### 问题 3：点云为空

**症状**：立体视觉节点生成的点云为空

**解决方案**：
1. 检查是否接收到所有 6 个摄像头的图像
2. 调整立体匹配参数（在 `stereo_vision.py` 中）
3. 检查摄像头标定参数是否正确

### 问题 4：EGO-Planner 无法规划路径

**症状**：无人机不移动

**解决方案**：
1. 检查点云数据是否正确发布
2. 检查无人机的当前位置和目标位置
3. 查看 EGO-Planner 的日志输出

```bash
rosnode list
rosnode info /ego_planner_node
```

### 问题 5：系统延迟过大

**症状**：控制响应缓慢

**解决方案**：
1. 降低图像分辨率
2. 降低图像采集频率（在 `arma3_bridge.py` 中修改 `time.sleep(0.5)`）
3. 使用更快的网络连接
4. 关闭不必要的后台程序

---

## 开发指南

### 项目结构

```
Ros_arma3_Connection/
├── windows_side/
│   ├── arma3_mod/
│   │   └── init.sqf                 # Arma 3 任务初始化脚本
│   └── bridge_program/
│       ├── arma3_bridge.py          # Windows 端桥接程序
│       └── requirements.txt         # Python 依赖
├── linux_side/
│   ├── ros_nodes/
│   │   ├── arma3_ros_bridge.py      # ROS 桥接节点
│   │   ├── ego_planner_interface.py # EGO-Planner 接口
│   │   ├── arma3_bridge.launch      # ROS launch 文件
│   │   ├── CMakeLists.txt           # CMake 配置
│   │   └── package.xml              # ROS 包配置
│   └── image_processing/
│       └── stereo_vision.py         # 立体视觉处理节点
├── docs/                            # 文档
├── assets/                          # 资源文件
└── README.md                        # 本文件
```

### 添加新的 ROS 节点

1. 在 `linux_side/ros_nodes/` 中创建新的 Python 文件
2. 在 `CMakeLists.txt` 中添加新节点
3. 在 `arma3_bridge.launch` 中添加节点启动配置

### 修改通信协议

通信协议定义在以下文件中：
- **Windows 端**：`windows_side/bridge_program/arma3_bridge.py`
- **Linux 端**：`linux_side/ros_nodes/arma3_ros_bridge.py`

消息格式：
```
[4 bytes: 消息类型]
[可变长度: 消息体]
```

### 调试技巧

1. **查看 ROS 话题**：
```bash
rostopic list
rostopic echo /arma3/camera_0/image_raw
```

2. **查看 ROS 节点**：
```bash
rosnode list
rosnode info /arma3_ros_bridge
```

3. **查看日志**：
```bash
rosrun rqt_console rqt_console
```

4. **录制数据**：
```bash
rosbag record -a
```

---

## 致谢

本项目基于以下开源项目：

- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) - 浙江大学 FAST Lab 的多无人机路径规划算法
- [ArmaCOM](https://github.com/googleben/ArmaCOM) - Arma 3 与外部程序的通信扩展
- [ROS](https://www.ros.org/) - Robot Operating System

特别感谢所有为这些项目做出贡献的开发者。

---

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 联系方式

如有问题或建议，请通过以下方式联系：

- **GitHub Issues**: [https://github.com/HitTheStars/Ros_arma3_Connection/issues](https://github.com/HitTheStars/Ros_arma3_Connection/issues)

---

## 更新日志

### v1.0.0 (2025-11-03)

- ✅ 初始版本发布
- ✅ 实现 Windows 端图像采集和传输
- ✅ 实现 Linux 端 ROS 桥接节点
- ✅ 实现立体视觉处理
- ✅ 实现 EGO-Planner 接口
- ✅ 完整的部署指南

---

**祝您使用愉快！如果本项目对您有帮助，请给我们一个 ⭐ Star！**
