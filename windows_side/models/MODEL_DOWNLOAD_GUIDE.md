# Depth Anything V2 ONNX 模型下载指南

## 模型选择

Depth Anything V2 提供三种尺寸的模型：

| 模型 | 文件名 | 大小 | RTX 4060 推理时间 | 精度 | 推荐用途 |
|------|--------|------|-------------------|------|----------|
| **Small** | `depth_anything_v2_vits.onnx` | ~100 MB | 10-15 ms | 良好 | **实时应用（推荐）** |
| Base | `depth_anything_v2_vitb.onnx` | ~350 MB | 20-30 ms | 很好 | 平衡性能和精度 |
| Large | `depth_anything_v2_vitl.onnx` | ~1.3 GB | 40-60 ms | 最佳 | 离线处理 |

**推荐**: 对于 10 FPS 实时应用，使用 **Small 模型**。

---

## 下载方法

### 方法 1: 从 Hugging Face 下载（推荐）

#### 步骤 1: 访问 Hugging Face

打开浏览器，访问：

**Depth Anything V2 ONNX 仓库**:
- https://huggingface.co/depth-anything/Depth-Anything-V2-Small-ONNX

或搜索：
- https://huggingface.co/models?search=depth-anything-v2-onnx

#### 步骤 2: 下载模型文件

在仓库页面中，找到 **Files and versions** 标签，下载以下文件之一：

- `depth_anything_v2_vits.onnx` (Small, 推荐)
- `depth_anything_v2_vitb.onnx` (Base)
- `depth_anything_v2_vitl.onnx` (Large)

#### 步骤 3: 放置模型文件

将下载的 `.onnx` 文件放置在本目录：

```
windows_side/models/depth_anything_v2_vits.onnx
```

---

### 方法 2: 使用 Git LFS 克隆（高级）

如果您安装了 Git LFS，可以直接克隆整个仓库：

```bash
# 安装 Git LFS（如果尚未安装）
git lfs install

# 克隆仓库
git clone https://huggingface.co/depth-anything/Depth-Anything-V2-Small-ONNX

# 复制模型文件
cp Depth-Anything-V2-Small-ONNX/depth_anything_v2_vits.onnx ./
```

---

### 方法 3: 从 PyTorch 转换（专家级）

如果 Hugging Face 上没有 ONNX 模型，可以从 PyTorch 模型转换。

#### 步骤 1: 安装依赖

```bash
pip install torch torchvision onnx
```

#### 步骤 2: 克隆 Depth Anything V2 仓库

```bash
git clone https://github.com/DepthAnything/Depth-Anything-V2.git
cd Depth-Anything-V2
```

#### 步骤 3: 下载 PyTorch 权重

从 [Depth Anything V2 Releases](https://github.com/DepthAnything/Depth-Anything-V2/releases) 下载：

- `depth_anything_v2_vits.pth` (Small)
- `depth_anything_v2_vitb.pth` (Base)
- `depth_anything_v2_vitl.pth` (Large)

#### 步骤 4: 转换为 ONNX

创建转换脚本 `export_onnx.py`：

```python
import torch
import torch.onnx
from depth_anything_v2.dpt import DepthAnythingV2

# 加载模型
model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384])
model.load_state_dict(torch.load('depth_anything_v2_vits.pth', map_location='cpu'))
model.eval()

# 创建示例输入
dummy_input = torch.randn(1, 3, 518, 518)

# 导出为 ONNX
torch.onnx.export(
    model,
    dummy_input,
    'depth_anything_v2_vits.onnx',
    export_params=True,
    opset_version=14,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size', 2: 'height', 3: 'width'},
        'output': {0: 'batch_size', 2: 'height', 3: 'width'}
    }
)

print("ONNX 模型已导出: depth_anything_v2_vits.onnx")
```

运行：

```bash
python export_onnx.py
```

#### 步骤 5: 复制模型文件

```bash
cp depth_anything_v2_vits.onnx /path/to/Ros_arma3_Connection/windows_side/models/
```

---

## 验证模型

### 方法 1: 使用 ONNX Runtime

创建测试脚本 `test_model.py`：

```python
import onnxruntime as ort
import numpy as np

# 加载模型
model_path = "depth_anything_v2_vits.onnx"
session = ort.InferenceSession(model_path, providers=['DmlExecutionProvider', 'CPUExecutionProvider'])

print(f"✓ 模型加载成功: {model_path}")
print(f"  执行提供器: {session.get_providers()}")

# 获取输入输出信息
input_info = session.get_inputs()[0]
output_info = session.get_outputs()[0]

print(f"  输入名称: {input_info.name}")
print(f"  输入形状: {input_info.shape}")
print(f"  输出名称: {output_info.name}")
print(f"  输出形状: {output_info.shape}")

# 测试推理
dummy_input = np.random.randn(1, 3, 518, 518).astype(np.float32)
output = session.run([output_info.name], {input_info.name: dummy_input})[0]

print(f"✓ 推理测试成功")
print(f"  输出形状: {output.shape}")
```

运行：

```cmd
python test_model.py
```

### 方法 2: 使用完整测试

运行项目的安装测试脚本：

```cmd
cd ..
python test_installation.py
```

---

## 模型信息

### Small 模型 (depth_anything_v2_vits.onnx)

- **编码器**: Vision Transformer Small (ViT-S)
- **参数量**: ~25M
- **输入尺寸**: 518 x 518 x 3
- **输出尺寸**: 518 x 518 x 1
- **推理时间**: 10-15 ms (RTX 4060 + DirectML)
- **精度**: 良好（适合实时应用）

### Base 模型 (depth_anything_v2_vitb.onnx)

- **编码器**: Vision Transformer Base (ViT-B)
- **参数量**: ~97M
- **输入尺寸**: 518 x 518 x 3
- **输出尺寸**: 518 x 518 x 1
- **推理时间**: 20-30 ms (RTX 4060 + DirectML)
- **精度**: 很好（平衡性能和精度）

### Large 模型 (depth_anything_v2_vitl.onnx)

- **编码器**: Vision Transformer Large (ViT-L)
- **参数量**: ~335M
- **输入尺寸**: 518 x 518 x 3
- **输出尺寸**: 518 x 518 x 1
- **推理时间**: 40-60 ms (RTX 4060 + DirectML)
- **精度**: 最佳（适合离线处理）

---

## 常见问题

### Q1: 下载速度慢怎么办？

**A**: 
- 使用 Hugging Face 镜像站（如 hf-mirror.com）
- 使用下载工具（如 IDM、迅雷）
- 联系我们获取百度网盘链接

### Q2: 模型文件损坏？

**A**: 
- 检查文件大小是否正确
- 重新下载
- 使用 `onnx.checker.check_model()` 验证

### Q3: 找不到 ONNX 模型？

**A**: 
- 检查 Hugging Face 仓库是否更新
- 尝试搜索其他用户上传的版本
- 使用方法 3 从 PyTorch 转换

### Q4: 可以使用其他深度估计模型吗？

**A**: 
可以！只需：
1. 将模型转换为 ONNX 格式
2. 修改 `arma3_depth_sender.py` 中的预处理和后处理
3. 确保输出格式为 16-bit 深度图

---

## 参考资料

- [Depth Anything V2 GitHub](https://github.com/DepthAnything/Depth-Anything-V2)
- [Depth Anything V2 论文](https://arxiv.org/abs/2406.09414)
- [Hugging Face 模型库](https://huggingface.co/models?search=depth-anything)
- [ONNX 官方文档](https://onnx.ai/)

---

**最后更新**: 2025-11-03  
**作者**: Manus AI
