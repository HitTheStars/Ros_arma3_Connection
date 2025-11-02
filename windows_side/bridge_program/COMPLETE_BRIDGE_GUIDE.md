# Arma 3 <-> ROS 完整桥接程序使用指南

## 概述

本桥接程序集成了 [codingWithArma3](https://github.com/Daboolu/codingWithArma3) 项目的图像采集和立体视觉处理功能，提供了一个**完整的、端到端的**解决方案，用于从 Arma 3 获取多视角图像、生成 3D 点云，并通过 TCP/IP 发送到 ROS。

### 主要功能

1. **多视角图像采集**：从 Arma 3 窗口截取 6 个摄像头视角的图像
2. **立体视觉处理**：使用双目立体视觉算法计算视差图
3. **3D 点云生成**：从视差图生成 3D 点云数据
4. **TCP/IP 通信**：将点云和状态数据发送到 ROS 服务器
5. **控制指令接收**：从 ROS 接收 MOVE 和 GOAL 指令

### 与 ArmaCOM MOD 的关系

- **ArmaCOM MOD**：负责 Arma 3 内部的状态数据发送和控制指令接收
- **完整桥接程序**：负责图像采集、立体视觉处理和点云生成

两者可以**同时运行**，提供完整的感知-规划-控制闭环：
- ArmaCOM MOD → 状态数据（位置、速度、方向）
- 完整桥接程序 → 点云数据（环境感知）
- ROS → 控制指令（路径规划结果）

---

## 系统要求

### 硬件要求

- **CPU**：Intel i5 或更高（推荐 i7）
- **内存**：8 GB 或更高（推荐 16 GB）
- **显卡**：支持 DirectX 11 的独立显卡

### 软件要求

- **操作系统**：Windows 10/11 (64-bit)
- **Python**：3.8 或更高版本
- **Arma 3**：完整版游戏
- **依赖库**：
  - Pillow（图像采集）
  - OpenCV（立体视觉处理）
  - NumPy（数值计算）

---

## 安装指南

### 第一步：安装 Python 依赖

```cmd
cd windows_side\bridge_program
pip install -r requirements_complete.txt
```

### 第二步：配置 ROS 服务器地址

编辑 `config.json` 文件：

```json
{
  "ros_server": {
    "ip": "192.168.1.100",  // 改成你的 Linux VM IP
    "port": 5555
  }
}
```

或者直接编辑 `arma3_ros_bridge_complete.py` 中的配置：

```python
# ROS 服务器配置
SERVER_IP = '192.168.1.100'  # 修改为你的 Linux VM IP
SERVER_PORT = 5555
```

### 第三步：配置图像采集区域

默认配置适用于 1920x1080 分辨率的屏幕。如果你的分辨率不同，需要调整 `config.json` 中的 `regions`：

```json
{
  "image_capture": {
    "regions": [
      [0, 255, 455, 511],      // 左上
      [455, 255, 911, 511],    // 中上
      [911, 255, 1365, 511],   // 右上
      [0, 511, 455, 767],      // 左下
      [455, 511, 911, 767],    // 中下
      [911, 511, 1365, 767]    // 右下
    ]
  }
}
```

---

## 使用指南

### 基本使用流程

#### 1. 启动 ROS 服务器（Linux 端）

```bash
cd linux_side
./start.sh
```

#### 2. 启动 Arma 3 并加载 MOD

```cmd
arma3_x64.exe -mod=@ROS_Bridge -noBE
```

在编辑器中加载 Camera 任务并预览。

#### 3. 启动完整桥接程序

```cmd
cd windows_side\bridge_program
start_complete_bridge.bat
```

或者直接运行：

```cmd
python arma3_ros_bridge_complete.py
```

#### 4. 观察数据流

在 Linux 端，使用 ROS 工具查看数据：

```bash
# 查看点云话题
rostopic echo /arma3/point_cloud

# 使用 rviz 可视化
rviz
```

### 高级使用

#### 自定义立体视觉参数

如果你需要重新标定摄像头，可以修改 `arma3_ros_bridge_complete.py` 中的 `STEREO_CONFIG`：

```python
STEREO_CONFIG = {
    'image_size': (960, 540),
    'left_intrinsics': np.array([...]),  // 左摄像头内参
    'left_distortion': np.array([...]),  // 左摄像头畸变
    'right_intrinsics': np.array([...]), // 右摄像头内参
    'right_distortion': np.array([...]), // 右摄像头畸变
    'R': np.array([...]),                // 旋转矩阵
    'T': np.array([...])                 // 平移向量
}
```

这些参数可以通过 `codingWithArma3` 项目中的标定工具获得。

#### 调整点云采样率

如果点云数据太大，可以调整最大点数：

```python
# 在 send_pointcloud 方法中
if len(pointcloud) > 10000:  // 修改这个值
    indices = np.random.choice(len(pointcloud), 10000, replace=False)
    pointcloud = pointcloud[indices]
```

---

## 工作原理

### 图像采集

桥接程序使用 `PIL.ImageGrab` 从 Arma 3 窗口截取 6 个区域的图像：

```
┌─────────┬─────────┬─────────┐
│  左上   │  中上   │  右上   │
│ (0)     │ (1)     │ (2)     │
├─────────┼─────────┼─────────┤
│  左下   │  中下   │  右下   │
│ (3)     │ (4)     │ (5)     │
└─────────┴─────────┴─────────┘
```

### 立体视觉处理

使用图像 1（中上）和图像 2（右上）作为左右视图：

1. **图像校正**：使用 `cv2.remap` 校正图像畸变
2. **视差计算**：使用 `cv2.StereoBM` 计算视差图
3. **3D 重投影**：使用 `cv2.reprojectImageTo3D` 生成 3D 点云
4. **坐标转换**：将点云从摄像头坐标系转换到世界坐标系

### 数据传输

点云数据以 JSON 格式通过 TCP/IP 发送：

```json
{
  "type": "POINTCLOUD",
  "timestamp": 1234567890.123,
  "points": [
    [x1, y1, z1],
    [x2, y2, z2],
    ...
  ]
}
```

---

## 性能优化

### 图像采集频率

默认为 2 Hz（每 0.5 秒）。可以调整：

```python
self.image_capture = ImageCaptureThread(interval=0.5)  // 修改这个值
```

### 点云处理频率

默认为 10 Hz（每 0.1 秒）。可以调整主循环中的 `time.sleep(0.1)`。

### 内存优化

如果内存占用过高，可以：

1. 减少点云最大数量
2. 增加图像采集间隔
3. 降低图像分辨率

---

## 故障排除

### 问题 1：无法连接到 ROS 服务器

**症状**：`[TCP Client] Connection failed: [Errno 10061]`

**解决方案**：
1. 确保 ROS 服务器已启动
2. 检查 IP 地址和端口配置
3. 检查防火墙设置
4. 测试网络连接：`telnet <Linux_IP> 5555`

### 问题 2：图像采集失败

**症状**：`[Image Capture] Error: ...`

**解决方案**：
1. 确保 Arma 3 窗口可见（不能最小化）
2. 检查截图区域配置是否正确
3. 确保 Pillow 库已正确安装

### 问题 3：点云数据为空

**症状**：`Generated 0 points`

**解决方案**：
1. 检查左右图像是否正确采集
2. 调整立体匹配参数（`numDisparities`、`blockSize`）
3. 确保场景中有足够的纹理特征

### 问题 4：性能问题

**症状**：程序运行缓慢，帧率低

**解决方案**：
1. 降低图像采集频率
2. 减少点云最大数量
3. 关闭不必要的后台程序
4. 升级硬件（CPU、内存）

---

## 与 codingWithArma3 的对比

本桥接程序基于 [codingWithArma3](https://github.com/Daboolu/codingWithArma3) 项目，但做了以下改进：

| 特性 | codingWithArma3 | 本桥接程序 |
|------|-----------------|------------|
| 通信方式 | ZMQ | TCP/IP |
| 数据格式 | 自定义 | JSON |
| ROS 集成 | 需要额外开发 | 内置支持 |
| 配置方式 | 硬编码 | 配置文件 |
| 文档 | 基础 | 详细 |
| 易用性 | 中等 | 高 |

---

## 参考资料

- [codingWithArma3 GitHub 仓库](https://github.com/Daboolu/codingWithArma3)
- [OpenCV 立体视觉文档](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
- [ROS 点云处理](http://wiki.ros.org/pcl)

---

## 许可证

本桥接程序基于 MIT 许可证，部分代码来自 [codingWithArma3](https://github.com/Daboolu/codingWithArma3) 项目。

---

**作者**：Manus AI  
**最后更新**：2024-11-02
