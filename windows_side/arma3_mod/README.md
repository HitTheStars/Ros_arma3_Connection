# Arma 3 MOD 使用指南

## 目录结构

```
arma3_mod/
├── Camera.Altis/          # 完整的 Arma 3 任务文件夹
│   ├── init.sqf          # 任务初始化脚本（6 个无人机摄像头）
│   └── mission.sqm       # 任务场景文件
├── init.sqf              # 简化版初始化脚本
└── README.md             # 本文件
```

## 什么是 Camera.Altis？

`Camera.Altis` 是一个完整的 Arma 3 任务文件夹，包含了：

- **6 架无人机**：在 Altis 地图上预设的无人机
- **6 个摄像头视角**：每架无人机都有一个摄像头，显示在屏幕顶部
- **实时渲染**：使用 Arma 3 的 Render-to-Texture 技术实时渲染摄像头画面

## 如何使用

### 方法 1：直接使用 Camera.Altis 任务

1. **复制任务文件夹到 Arma 3 任务目录**

   在 Windows 上，Arma 3 的任务目录通常在：
   ```
   C:\Users\<你的用户名>\Documents\Arma 3 - Other Profiles\<你的游戏名>\missions\
   ```

   将 `Camera.Altis` 文件夹复制到这个目录下。

2. **启动 Arma 3**

   - 打开 Arma 3
   - 点击 "Editor"（编辑器）
   - 选择 "Altis" 地图
   - 点击 "Load"（加载）
   - 选择 "Camera" 任务

3. **运行任务**

   - 在编辑器中，点击 "Preview"（预览）或按 `Ctrl+P`
   - 任务将启动，你会看到 6 个摄像头视角显示在屏幕顶部

### 方法 2：在自己的任务中使用

如果你想在自己的任务中使用这个摄像头系统：

1. **创建你的任务**

   - 在 Arma 3 编辑器中创建一个新任务
   - 添加 6 架无人机，命名为 `uav1`, `uav2`, `uav3`, `uav4`, `uav5`, `uav6`

2. **复制 init.sqf**

   - 将 `Camera.Altis/init.sqf` 的内容复制到你的任务的 `init.sqf` 文件中
   - 或者直接将 `Camera.Altis/init.sqf` 复制到你的任务文件夹中

3. **保存并运行**

   - 保存你的任务
   - 运行任务，摄像头系统将自动启动

## init.sqf 代码说明

`init.sqf` 是任务的初始化脚本，在任务开始时自动执行。我们的脚本做了以下事情：

### 1. 定义无人机数组

```sqf
_uavs = [uav1, uav2, uav3, uav4, uav5, uav6];
```

这里定义了 6 架无人机。**重要**：无人机的名称必须与编辑器中的名称一致。

### 2. 为每架无人机创建摄像头

```sqf
{
    private _uav = _x;
    private _index = _forEachIndex;
    
    // 启动无人机
    _uav engineOn true;
    _uav action ["autoHover", _uav];
    _uav flyInHeight 10;
    _uav limitSpeed 100;
    
    // 创建摄像头...
} forEach _uavs;
```

对于每架无人机：
- 启动引擎
- 开启悬停模式
- 设置飞行高度为 10 米
- 限制最大速度为 100 km/h

### 3. 创建显示控件

```sqf
private _ctrl = findDisplay 46 ctrlCreate ["RscPicture", 1000 + _index];
_ctrl ctrlSetPosition [_xPos, _yPos, _boxWidth, _screenHeight];
_ctrl ctrlSetText format ["#(rgb,512,512,1)r2t(uav_cam%1,1.0)", _index];
_ctrl ctrlCommit 0;
```

创建一个 RscPicture 控件，用于显示摄像头画面。

### 4. 创建并绑定摄像机

```sqf
private _cam = "camera" camCreate [0, 0, 0];
_cam camSetFov 0.75;
_cam cameraEffect ["Internal", "Back", format ["uav_cam%1", _index]];
```

创建一个摄像机对象，并绑定到 Render-to-Texture。

### 5. 实时更新摄像机位置

```sqf
[_cam, _uav] spawn {
    params ["_cam", "_uav"];
    while {true} do {
        _cam setPos (_uav modelToWorld [0, 0.3, 0]);
        // 设置摄像机方向...
        sleep 0.005;
    };
};
```

在一个循环中，每 5 毫秒更新一次摄像机的位置和方向，使其跟随无人机。

## 摄像头布局

摄像头在屏幕上的布局如下：

```
┌─────────┬─────────┬─────────┐
│  Cam 1  │  Cam 2  │  Cam 3  │
├─────────┼─────────┼─────────┤
│  Cam 4  │  Cam 5  │  Cam 6  │
└─────────┴─────────┴─────────┘
```

每个摄像头占据屏幕宽度的 1/3，高度的 1/3。

## 与 ROS 集成

这个 MOD 与 Windows 端的桥接程序配合使用：

1. **桥接程序捕获摄像头画面**

   `arma3_bridge_enhanced.py` 使用 `PIL.ImageGrab` 捕获屏幕上的 6 个摄像头区域。

2. **发送到 ROS**

   捕获的图像被压缩为 JPEG 格式，通过 TCP/IP 发送到 Linux 端的 ROS 节点。

3. **ROS 处理**

   ROS 节点接收图像，进行立体视觉处理，生成点云，用于路径规划。

## 自定义

### 修改无人机数量

如果你想使用不同数量的无人机：

1. 修改 `init.sqf` 中的 `_uavs` 数组
2. 修改桥接程序中的 `regions` 数组
3. 调整摄像头布局的计算

### 修改摄像头视角

在 `init.sqf` 中，你可以调整摄像头的位置和角度：

```sqf
// 摄像头位置：无人机前方 0.3 米
_cam setPos (_uav modelToWorld [0, 0.3, 0]);

// 摄像头俯视角度：15 度
private _pitch = 15 * (pi / 180);
```

### 修改摄像头分辨率

在 `init.sqf` 中：

```sqf
// 当前分辨率：512x512
_ctrl ctrlSetText format ["#(rgb,512,512,1)r2t(uav_cam%1,1.0)", _index];

// 修改为 1024x1024
_ctrl ctrlSetText format ["#(rgb,1024,1024,1)r2t(uav_cam%1,1.0)", _index];
```

**注意**：更高的分辨率会增加性能开销。

## 故障排除

### 问题 1：摄像头没有显示

**可能原因**：
- 无人机名称不匹配
- 任务没有正确加载

**解决方案**：
1. 检查编辑器中的无人机名称是否为 `uav1` 到 `uav6`
2. 确保 `init.sqf` 在任务文件夹的根目录
3. 重新加载任务

### 问题 2：摄像头画面是黑色的

**可能原因**：
- 摄像机位置不正确
- Render-to-Texture 没有正确绑定

**解决方案**：
1. 检查无人机是否已启动（引擎开启）
2. 检查摄像机的 FOV 设置
3. 尝试调整摄像机的位置

### 问题 3：性能问题

**可能原因**：
- 6 个摄像头同时渲染会消耗大量资源

**解决方案**：
1. 降低摄像头分辨率
2. 减少无人机数量
3. 降低游戏图形设置
4. 增加 `sleep` 时间（降低更新频率）

## 参考资料

- [Arma 3 Scripting Commands](https://community.bistudio.com/wiki/Category:Arma_3:_Scripting_Commands)
- [SQF 语言入门](https://blog.csdn.net/qq_35697906/article/details/123844766)
- [Render-to-Texture 教程](https://community.bistudio.com/wiki/Render_to_Texture)

## 下一步

1. **测试 MOD**：在 Arma 3 中运行任务，确保摄像头正常工作
2. **运行桥接程序**：启动 `arma3_bridge_enhanced.py`，开始捕获和发送图像
3. **启动 ROS**：在 Linux 端启动 ROS 节点，接收图像并进行处理
4. **集成 EGO-Planner**：使用生成的点云进行路径规划

祝你使用愉快！
