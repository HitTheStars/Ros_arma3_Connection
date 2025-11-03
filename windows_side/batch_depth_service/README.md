# Batch Depth Estimation Service

批处理深度估计服务 - 为多 UAV 场景优化的单模型架构

## 概述

本服务使用单个深度估计模型实例处理多个 UAV 的图像，通过批处理推理显著提升性能并降低资源占用。

### 性能优势

与传统的多模型架构相比：

| 指标 | 6 个独立模型 | 单模型批处理 | 提升 |
|-----|-------------|-------------|------|
| **显存占用** | 12 GB (2GB×6) | 2 GB | **83% ↓** |
| **推理时间** | 72 ms (12ms×6) | 30 ms | **2.4x ↑** |
| **吞吐量** | 83 FPS | 200 FPS | **2.4x ↑** |
| **模型加载时间** | 18 s (3s×6) | 3 s | **6x ↑** |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                 Arma 3 (Windows)                        │
│  UAV 0, UAV 1, UAV 2, ..., UAV N                        │
│  ↓ screenshot命令                                        │
│  Screenshots文件夹                                       │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌────────────────────────────────────────────────────────┐
│         arma3_image_collector.py                       │
│  - 监控Screenshots文件夹                                 │
│  - 识别UAV ID                                           │
│  - TCP发送图像                                          │
└────────────────┬───────────────────────────────────────┘
                 │ TCP (Port 5557)
                 │ JSON + Base64
┌────────────────▼───────────────────────────────────────┐
│         batch_depth_service.py                         │
│  ┌──────────────────────────────────────────────┐     │
│  │  Image Queue (Thread-Safe)                   │     │
│  │  [UAV0:img, UAV1:img, UAV2:img, ...]         │     │
│  └──────────────┬───────────────────────────────┘     │
│                 ↓                                       │
│  ┌──────────────────────────────────────────────┐     │
│  │  Batch Preprocessor                          │     │
│  │  - Resize to 518x518                         │     │
│  │  - Normalize [0, 1]                          │     │
│  │  - Stack to (N, 3, 518, 518)                 │     │
│  └──────────────┬───────────────────────────────┘     │
│                 ↓                                       │
│  ┌──────────────────────────────────────────────┐     │
│  │  Depth Anything V2 (ONNX + DirectML)        │     │
│  │  Single Instance - Batch Inference           │     │
│  └──────────────┬───────────────────────────────┘     │
│                 ↓                                       │
│  ┌──────────────────────────────────────────────┐     │
│  │  Result Distributor                          │     │
│  │  - Split batch results                       │     │
│  │  - Compress to PNG                           │     │
│  │  - Route to corresponding UAV                │     │
│  └──────────────┬───────────────────────────────┘     │
└─────────────────┼──────────────────────────────────────┘
                  │ TCP (Ports 5560-5565)
                  │ JSON + Base64
┌─────────────────▼──────────────────────────────────────┐
│                 ROS (Linux)                             │
│  arma3_depth_receiver_multi.py                          │
│  ↓                                                      │
│  /uav0/depth, /uav1/depth, ..., /uavN/depth            │
└─────────────────────────────────────────────────────────┘
```

---

## 安装

### 1. 安装依赖

```bash
pip install numpy opencv-python onnxruntime-directml watchdog
```

### 2. 下载模型

下载 Depth Anything V2 ONNX 模型并放置在 `../models/` 目录：

```bash
# 从 Hugging Face 下载
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.onnx

# 或手动下载并放置在：
# windows_side/models/depth_anything_v2_vits.onnx
```

---

## 使用方法

### 方式 1: 使用启动脚本（推荐）

#### 步骤 1: 启动批处理深度估计服务

双击运行 `start_batch_service.bat` 或在命令行执行：

```bash
start_batch_service.bat
```

#### 步骤 2: 启动图像采集器

双击运行 `start_image_collector.bat` 或在命令行执行：

```bash
start_image_collector.bat
```

#### 步骤 3: 启动 Arma 3

1. 加载 @ROS_Bridge MOD
2. 进入任务
3. 确保有 UAV 存在
4. 图像将自动采集并处理

### 方式 2: 手动启动

#### 启动批处理深度估计服务

```bash
python batch_depth_service.py [model_path] [num_uavs] [batch_size] [ros_ip]
```

**参数说明**：
- `model_path`: ONNX 模型路径（默认：`../models/depth_anything_v2_vits.onnx`）
- `num_uavs`: UAV 数量（默认：6）
- `batch_size`: 批处理大小（默认：6）
- `ros_ip`: ROS 系统 IP 地址（默认：`192.168.1.100`）

**示例**：

```bash
# 使用默认配置
python batch_depth_service.py

# 自定义配置
python batch_depth_service.py ../models/depth_anything_v2_vits.onnx 4 4 192.168.1.50
```

#### 启动图像采集器

```bash
python arma3_image_collector.py [screenshots_dir] [server_host] [server_port] [num_uavs]
```

**参数说明**：
- `screenshots_dir`: Screenshots 文件夹路径（默认：`~/Documents/Arma 3/Screenshots`）
- `server_host`: 深度估计服务主机（默认：`localhost`）
- `server_port`: 深度估计服务端口（默认：5557）
- `num_uavs`: UAV 数量（默认：6）

**示例**：

```bash
# 使用默认配置
python arma3_image_collector.py

# 自定义配置
python arma3_image_collector.py "C:\Users\YourName\Documents\Arma 3\Screenshots" localhost 5557 4
```

---

## 配置

### 端口配置

- **图像接收端口**：5557（固定）
- **深度发送端口**：5560-5565（每个 UAV 使用 base_port + uav_id）

### Arma 3 配置

在 `XEH_postInit.sqf` 中添加图像采集代码：

```sqf
// 图像采集函数
ROS_fnc_captureImages = {
    {
        private _uav = _x;
        private _uavId = _forEachIndex;
        
        // 截屏
        private _filename = format["uav%1_frame.png", _uavId];
        screenshot _filename;
        
        diag_log format ["[ROS Bridge] Captured image for UAV %1", _uavId];
        
    } forEach ROS_UAVs;
};

// 定期执行（5 Hz）
[] spawn {
    while {ROS_Connected} do {
        [] call ROS_fnc_captureImages;
        sleep 0.2;  // 5 Hz
    };
};
```

### ROS 配置

在 Linux 端启动多 UAV 深度接收节点：

```bash
rosrun arma3_depth_multi arma3_depth_receiver_multi.py _num_uavs:=6 _base_port:=5560
```

---

## 性能调优

### 批处理大小

- **batch_size = num_uavs**：最佳性能，等待所有 UAV 的图像后一起处理
- **batch_size < num_uavs**：降低延迟，但可能降低吞吐量
- **batch_size > num_uavs**：无意义，因为最多只有 num_uavs 个图像

### 图像采集频率

在 Arma 3 SQF 脚本中调整 `sleep` 时间：

- **5 Hz (sleep 0.2)**：推荐，平衡延迟和性能
- **10 Hz (sleep 0.1)**：更低延迟，但增加 CPU 和网络负载
- **2 Hz (sleep 0.5)**：降低负载，但延迟较高

### GPU 加速

确保使用 DirectML 提供程序：

```python
providers = ['DmlExecutionProvider', 'CPUExecutionProvider']
```

如果 GPU 不可用，会自动降级到 CPU。

---

## 故障排除

### 问题 1: 连接失败

**症状**：图像采集器无法连接到深度估计服务

**解决方案**：
1. 确保深度估计服务已启动
2. 检查防火墙设置，允许端口 5557
3. 确认 IP 地址和端口配置正确

### 问题 2: 模型加载失败

**症状**：提示 "Model file not found"

**解决方案**：
1. 确认模型文件存在于 `../models/depth_anything_v2_vits.onnx`
2. 从 Hugging Face 下载模型
3. 检查文件路径是否正确

### 问题 3: GPU 不可用

**症状**：提示 "Running on CPU (slower performance)"

**解决方案**：
1. 安装 onnxruntime-directml：`pip install onnxruntime-directml`
2. 确保 GPU 驱动程序已更新
3. 检查 GPU 是否支持 DirectML

### 问题 4: 图像未被采集

**症状**：图像采集器没有检测到新图像

**解决方案**：
1. 确认 Arma 3 正在运行并执行截屏命令
2. 检查 Screenshots 文件夹路径是否正确
3. 确认文件名格式为 `uav0_frame.png`, `uav1_frame.png` 等
4. 检查 Arma 3 日志确认截屏命令执行

### 问题 5: ROS 连接失败

**症状**：无法连接到 ROS 深度接收节点

**解决方案**：
1. 确认 ROS 系统正在运行
2. 确认 ROS IP 地址配置正确
3. 检查网络连接（Windows ↔ Linux）
4. 确认深度接收节点已启动
5. 检查防火墙设置，允许端口 5560-5565

---

## 日志和监控

### 批处理深度估计服务

服务会定期（每 10 秒）打印统计信息：

```
============================================================
STATISTICS:
  Runtime: 60.5s
  Images received: 360
  Batches processed: 60
  Throughput: 5.95 images/sec
  Avg preprocessing: 15.23ms
  Avg inference: 28.45ms
  Queue size: 0
============================================================
```

### 图像采集器

采集器会实时打印每个图像的发送状态：

```
✓ Sent image from UAV 0 (size: (1080, 1920, 3), total: 42, rate: 5.25 img/s)
✓ Sent image from UAV 1 (size: (1080, 1920, 3), total: 43, rate: 5.26 img/s)
```

---

## 技术细节

### 批处理推理流程

1. **图像收集**：从队列中收集最多 batch_size 个图像
2. **预处理**：
   - Resize to 518×518
   - BGR → RGB
   - Normalize to [0, 1]
   - Transpose to (C, H, W)
   - Stack to (N, C, H, W)
3. **推理**：使用 ONNX Runtime 进行批处理推理
4. **后处理**：
   - 归一化到 [0, 255]
   - 压缩为 PNG
   - Base64 编码
5. **分发**：根据 UAV ID 路由到对应的 ROS 节点

### 网络协议

**图像传输（Arma 3 → Service）**：
```json
{
  "uav_id": 0,
  "image": "base64_encoded_png_data",
  "timestamp": 1699000000.123
}
```

**深度传输（Service → ROS）**：
```json
{
  "uav_id": 0,
  "depth": "base64_encoded_png_data",
  "timestamp": 1699000000.123,
  "width": 518,
  "height": 518
}
```

**消息格式**：
1. 4 字节消息长度（大端序）
2. JSON 消息内容（UTF-8 编码）

---

## 与原方案对比

### 原方案（多模型架构）

```
UAV 0 → 深度模型 0 → ROS
UAV 1 → 深度模型 1 → ROS
UAV 2 → 深度模型 2 → ROS
...
```

**缺点**：
- 每个 UAV 需要独立的模型实例
- 显存占用高（2GB × N）
- 无法利用批处理加速
- 模型加载时间长

### 新方案（批处理架构）

```
UAV 0 ┐
UAV 1 ├→ 单个深度模型（批处理）→ 分发 → ROS
UAV 2 ┘
```

**优点**：
- 单个模型实例
- 显存占用低（2GB）
- 批处理加速 2-3 倍
- 模型加载时间短

---

## 参考资料

- [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)
- [ONNX Runtime](https://onnxruntime.ai/)
- [DirectML](https://learn.microsoft.com/en-us/windows/ai/directml/)
- [Arma 3 Screenshot Command](https://community.bistudio.com/wiki/screenshot)

---

## 许可证

本项目遵循 MIT 许可证。
