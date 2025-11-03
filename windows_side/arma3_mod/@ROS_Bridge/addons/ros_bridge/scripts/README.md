# UAV Camera Scripts

这个目录包含用于 Arma 3 无人机摄像头渲染的 SQF 脚本。

## 文件说明

### initUAVCameras.sqf

**功能**：为 6 个无人机创建摄像头并渲染到屏幕

**使用方法**：

1. **在 Eden Editor 中**：
   - 创建 6 个无人机，命名为 `uav1`, `uav2`, `uav3`, `uav4`, `uav5`, `uav6`
   - 在场景的 `init.sqf` 中添加：
     ```sqf
     execVM "\@ROS_Bridge\addons\ros_bridge\scripts\initUAVCameras.sqf";
     ```

2. **在 Debug Console 中**：
   - 按 `Esc` → `Debug Console`
   - 粘贴脚本内容并执行

3. **修改无人机名称**：
   - 如果您的无人机名称不是 `uav1-6`，请修改脚本第 16 行：
     ```sqf
     _uavs = [your_uav1, your_uav2, ...];
     ```

## 屏幕布局

脚本会将 6 个无人机的摄像头画面渲染到屏幕上，布局如下：

```
┌─────────┬─────────┬─────────┐
│  UAV 1  │  UAV 2  │  UAV 3  │
├─────────┼─────────┼─────────┤
│  UAV 4  │  UAV 5  │  UAV 6  │
└─────────┴─────────┴─────────┘
```

- 每个摄像头占屏幕的 1/6（3列×2行）
- 使用 Render-to-Texture (R2T) 技术

## 与 Python 深度估计模块配合使用

1. **启动 Arma 3**：
   - 运行脚本，确保 6 个摄像头画面显示在屏幕上

2. **启动 Python 深度估计模块**：
   ```bash
   python arma3_depth_sender.py \
       --model models/depth_anything_v2_vits.onnx \
       --linux-ip 192.168.1.100 \
       --input arma3_screenshot \
       --region 0
   ```
   
   - `--region 0-5`：选择要截取的无人机视角（0=UAV1, 1=UAV2, ...）

3. **Python 会自动**：
   - 检测 Arma 3 窗口
   - 截取对应区域的画面
   - 生成深度图像
   - 发送到 Linux ROS VM

## 性能优化

- **分辨率**：R2T 分辨率设置为 512×512（第 34 行）
  - 可以修改为 `#(rgb,1024,1024,1)r2t(...)` 以提高质量
  - 更高分辨率会降低帧率

- **更新频率**：摄像机每 5ms 更新一次（第 61 行）
  - 可以修改 `sleep 0.005` 为更大的值以降低 CPU 占用

## 常见问题

### Q: 画面不显示？

A: 检查以下几点：
1. 确保 Picture-in-Picture (PiP) 已启用（视频设置）
2. 确保无人机名称正确
3. 检查 Debug Console 是否有错误信息

### Q: 画面位置不对？

A: 修改脚本第 26-32 行的坐标计算逻辑

### Q: 如何隐藏摄像头画面？

A: 注释掉第 29-32 行（创建 RscPicture 控件的代码）

## 基于

- **codingWithArma3** 项目：https://github.com/Daboolu/codingWithArma3
- 感谢原作者提供的灵感和代码

## 许可

MIT License
