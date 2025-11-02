# ROS Bridge MOD - 完整安装和使用指南

## 目录

- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细安装步骤](#详细安装步骤)
- [配置说明](#配置说明)
- [使用说明](#使用说明)
- [故障排除](#故障排除)
- [开发指南](#开发指南)

---

## 系统要求

### 必需

- **Arma 3**：完整版游戏（Steam）
- **ArmaCOM 扩展**：v2.0 Beta 2 或更高版本（已包含）
- **操作系统**：Windows 10/11 (64-bit)
- **BattlEye**：必须禁用（ArmaCOM 不支持 BattlEye）

### 可选

- **Arma 3 Tools**：用于打包 PBO 文件（仅发布版本需要）
- **Linux 虚拟机**：运行 ROS 和 EGO-Planner

---

## 快速开始

### 方式 1：快速安装（推荐，用于开发和测试）

1. **以管理员身份运行** `quick_install.bat`
2. 输入您的 Arma 3 安装路径（如果脚本未自动检测到）
3. 脚本会创建符号链接，无需打包 PBO
4. 启动 Arma 3：
   ```
   arma3_x64.exe -mod=@ROS_Bridge -noBE
   ```

### 方式 2：完整安装（用于发布）

1. 运行 `build_mod.bat` 打包 PBO 文件
2. 运行 `install_mod.bat` 安装 MOD
3. 启动 Arma 3：
   ```
   arma3_x64.exe -mod=@ROS_Bridge -noBE
   ```

---

## 详细安装步骤

### 第一步：准备文件

确保您有以下文件：

```
windows_side/arma3_mod/
├── @ROS_Bridge/                    # MOD 文件夹
│   ├── mod.cpp                     # MOD 元数据
│   ├── addons/
│   │   └── ros_bridge/             # MOD 代码
│   │       ├── config.cpp          # MOD 配置
│   │       ├── XEH_preInit.sqf     # 预初始化脚本
│   │       ├── XEH_postInit.sqf    # 后初始化脚本
│   │       └── functions/          # 函数库
│   └── keys/                       # 签名密钥（可选）
├── ArmaCOM_x64.dll                 # ArmaCOM 扩展
├── quick_install.bat               # 快速安装脚本
├── install_mod.bat                 # 完整安装脚本
└── build_mod.bat                   # PBO 打包脚本
```

### 第二步：安装 ArmaCOM 扩展

ArmaCOM 是一个 Arma 3 扩展，允许 SQF 代码与外部程序通信。

**自动安装**（推荐）：
- 运行 `quick_install.bat` 或 `install_mod.bat`，脚本会自动复制 `ArmaCOM_x64.dll` 到 Arma 3 目录

**手动安装**：
1. 找到您的 Arma 3 安装目录：
   ```
   C:\Program Files (x86)\Steam\steamapps\common\Arma 3\
   ```
2. 将 `ArmaCOM_x64.dll` 复制到该目录

### 第三步：安装 MOD

#### 选项 A：快速安装（开发模式）

**优点**：
- 无需打包 PBO
- 可以直接编辑 SQF 脚本
- 更改立即生效

**步骤**：
1. 右键点击 `quick_install.bat`
2. 选择 **"以管理员身份运行"**
3. 按照提示操作
4. 脚本会创建一个符号链接：
   ```
   C:\...\Arma 3\@ROS_Bridge -> path\to\@ROS_Bridge
   ```

#### 选项 B：完整安装（发布模式）

**优点**：
- 更专业的分发方式
- 可以在 Steam Workshop 发布

**步骤**：

1. **打包 PBO**（可选）：
   - 确保已安装 **Arma 3 Tools**（从 Steam 下载）
   - 运行 `build_mod.bat`
   - 脚本会使用 AddonBuilder 打包 PBO 文件

2. **安装 MOD**：
   - 运行 `install_mod.bat`
   - 脚本会将 MOD 复制到 Arma 3 目录

### 第四步：配置 MOD

编辑 `@ROS_Bridge/addons/ros_bridge/XEH_postInit.sqf`，修改 ROS 服务器的 IP 地址：

```sqf
// 找到这一行
ROS_ServerIP = "192.168.1.100";  // 改成你的 Linux VM IP

// 如果需要，也可以修改端口
ROS_ServerPort = "5555";
```

### 第五步：启动 Arma 3

**方式 1：使用启动器**

1. 打开 Arma 3 启动器
2. 点击 **"MODs"**
3. 勾选 **"@ROS_Bridge"**
4. 点击 **"Parameters"**
5. 在 **"Additional Parameters"** 中添加：`-noBE`
6. 点击 **"Play"**

**方式 2：使用命令行**

创建一个批处理文件 `launch_arma3_with_ros.bat`：

```batch
@echo off
cd "C:\Program Files (x86)\Steam\steamapps\common\Arma 3"
start arma3_x64.exe -mod=@ROS_Bridge -noBE
```

运行该批处理文件即可启动 Arma 3。

**重要**：`-noBE` 参数会禁用 BattlEye，这是必需的，因为 ArmaCOM 不支持 BattlEye。

---

## 配置说明

### 网络配置

#### 查找 Linux VM 的 IP 地址

在 Linux 虚拟机中运行：

```bash
hostname -I
```

输出示例：`192.168.1.100`

#### 配置防火墙

确保 Linux 防火墙允许端口 5555 的 TCP 连接：

```bash
sudo ufw allow 5555/tcp
```

#### 测试连接

在 Windows 上测试连接：

```cmd
telnet 192.168.1.100 5555
```

如果连接成功，说明网络配置正确。

### MOD 参数

可以在 `XEH_postInit.sqf` 中修改以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROS_ServerIP` | `"192.168.1.100"` | ROS 服务器的 IP 地址 |
| `ROS_ServerPort` | `"5555"` | ROS 服务器的端口号 |
| `ROS_ImageInterval` | `0.5` | 发送图像通知的间隔（秒） |
| `ROS_StatusInterval` | `0.1` | 发送状态数据的间隔（秒） |

---

## 使用说明

### 基本工作流程

1. **启动 Linux 端的 ROS 服务器**
   ```bash
   cd linux_side
   ./start.sh
   ```

2. **启动 Arma 3 并加载 MOD**
   ```
   arma3_x64.exe -mod=@ROS_Bridge -noBE
   ```

3. **在 Arma 3 中创建任务**
   - 打开编辑器
   - 放置多个无人机（UAV）
   - 保存并预览任务

4. **MOD 会自动**：
   - 检测场景中的所有无人机
   - 连接到 ROS 服务器
   - 开始发送状态数据
   - 接收并执行控制指令

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

### 查看日志

Arma 3 的日志文件位于：
```
C:\Users\<你的用户名>\AppData\Local\Arma 3\arma3.rpt
```

搜索 `[ROS Bridge]` 可以查看 MOD 的日志输出。

---

## 故障排除

### 问题 1：MOD 未加载

**症状**：
- 日志中没有 `[ROS Bridge]` 相关信息
- 游戏中没有任何反应

**可能原因**：
1. MOD 未正确安装
2. MOD 未在启动参数中启用
3. BattlEye 未禁用

**解决方案**：

1. 检查 MOD 是否存在：
   ```
   C:\...\Arma 3\@ROS_Bridge\
   ```

2. 确保启动参数包含 `-mod=@ROS_Bridge -noBE`

3. 检查日志文件：
   ```
   C:\Users\<你的用户名>\AppData\Local\Arma 3\arma3.rpt
   ```

### 问题 2：无法连接到 ROS 服务器

**症状**：
```
[ROS Bridge] ERROR: Failed to connect: ...
```

**可能原因**：
1. ROS 服务器未启动
2. IP 地址或端口配置错误
3. 防火墙阻止连接
4. 网络不通

**解决方案**：

1. 确保 ROS 服务器已启动：
   ```bash
   rostopic list
   ```

2. 检查 IP 地址：
   ```bash
   hostname -I
   ```

3. 检查防火墙：
   ```bash
   sudo ufw status
   sudo ufw allow 5555/tcp
   ```

4. 测试连接：
   ```cmd
   telnet <Linux_IP> 5555
   ```

5. 检查 MOD 配置：
   - 打开 `XEH_postInit.sqf`
   - 确认 `ROS_ServerIP` 和 `ROS_ServerPort` 正确

### 问题 3：ArmaCOM 扩展未加载

**症状**：
```
[ROS Bridge] ERROR: Failed to create TCP client: ...
```

**可能原因**：
1. `ArmaCOM_x64.dll` 未正确安装
2. BattlEye 阻止了扩展加载

**解决方案**：

1. 确认 DLL 文件存在：
   ```
   C:\...\Arma 3\ArmaCOM_x64.dll
   ```

2. 确保使用 `-noBE` 参数启动 Arma 3

3. 查看 Arma 3 日志中的扩展加载信息

### 问题 4：无人机未被检测到

**症状**：
```
[ROS Bridge] WARNING: No UAVs found in mission!
```

**可能原因**：
1. 场景中没有放置无人机
2. 无人机类型不正确

**解决方案**：

1. 在编辑器中放置无人机：
   - 打开编辑器
   - Units → Empty → UAV
   - 放置多个无人机

2. 确保使用的是 UAV 类型的单位，而不是普通飞机

### 问题 5：数据传输延迟很大

**症状**：
- ROS 端接收到的数据有明显延迟（> 1 秒）

**可能原因**：
1. 网络带宽不足
2. 数据发送频率太高
3. 使用 WiFi 而不是有线网络

**解决方案**：

1. 降低数据发送频率：
   ```sqf
   ROS_ImageInterval = 1.0;  // 从 0.5 增加到 1.0
   ROS_StatusInterval = 0.2;  // 从 0.1 增加到 0.2
   ```

2. 使用有线网络而不是 WiFi

3. 检查网络带宽：
   ```cmd
   ping <Linux_IP>
   ```

---

## 开发指南

### 修改 SQF 脚本

如果使用快速安装（符号链接），可以直接编辑 SQF 文件：

```
path\to\@ROS_Bridge\addons\ros_bridge\XEH_postInit.sqf
```

更改会立即生效，无需重新打包。

### 添加新功能

1. **添加新的通信协议**：
   - 编辑 `XEH_postInit.sqf` 中的 `ROS_fnc_handleCallback` 函数
   - 添加新的消息类型处理

2. **添加新的传感器**：
   - 在 `ROS_fnc_initUAVs` 函数中添加传感器初始化代码
   - 在主循环中添加传感器数据发送代码

3. **添加新的控制指令**：
   - 在 `ROS_fnc_handleCallback` 函数中添加新的指令解析
   - 实现相应的控制逻辑

### 调试技巧

1. **使用 `diag_log` 输出调试信息**：
   ```sqf
   diag_log format ["[DEBUG] Variable value: %1", _myVariable];
   ```

2. **查看实时日志**：
   - 使用文本编辑器打开 `arma3.rpt`
   - 启用"自动刷新"功能

3. **使用 Arma 3 的调试控制台**：
   - 按 `Esc` 打开菜单
   - 点击 "Debug Console"
   - 输入 SQF 代码进行测试

### 打包发布

1. **编辑 MOD 元数据**：
   - 修改 `mod.cpp` 中的版本号、描述等信息

2. **打包 PBO**：
   ```cmd
   build_mod.bat
   ```

3. **测试 PBO**：
   - 安装打包后的 MOD
   - 在 Arma 3 中测试所有功能

4. **发布到 Steam Workshop**（可选）：
   - 使用 Arma 3 Tools 的 Publisher 工具
   - 上传 `@ROS_Bridge` 文件夹

---

## 常见问题 (FAQ)

### Q: 为什么必须禁用 BattlEye？

**A**: ArmaCOM 是一个第三方扩展，BattlEye 会将其视为潜在的作弊工具并阻止加载。这是 ArmaCOM 的限制，无法绕过。

### Q: 可以在多人游戏中使用吗？

**A**: 理论上可以，但需要服务器也禁用 BattlEye 并加载 MOD。不推荐在公共服务器上使用。

### Q: 为什么图像数据不是通过 ArmaCOM 传输的？

**A**: Arma 3 的扩展只能传输字符串数据，传输大量图像数据会非常低效。我们使用 Python 桥接程序直接捕获屏幕图像并通过 TCP 发送。

### Q: 可以同时使用多个 MOD 吗？

**A**: 可以，使用逗号分隔多个 MOD：
```
-mod=@ROS_Bridge,@OtherMOD
```

### Q: 如何卸载 MOD？

**A**: 删除以下文件和文件夹：
- `C:\...\Arma 3\@ROS_Bridge\`
- `C:\...\Arma 3\ArmaCOM_x64.dll`

---

## 参考资料

- [ArmaCOM GitHub](https://github.com/googleben/ArmaCOM)
- [ArmaCOM 文档](https://github.com/googleben/ArmaCOM/blob/master/Documentation.md)
- [Arma 3 Scripting Commands](https://community.bistudio.com/wiki/Category:Scripting_Commands_Arma_3)
- [Arma 3 Extensions](https://community.bistudio.com/wiki/Extensions)
- [ROS Noetic 文档](http://wiki.ros.org/noetic)
- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)

---

## 联系方式

如有问题或建议，请通过以下方式联系：

- **GitHub Issues**: [https://github.com/HitTheStars/Ros_arma3_Connection/issues](https://github.com/HitTheStars/Ros_arma3_Connection/issues)
- **项目主页**: [https://github.com/HitTheStars/Ros_arma3_Connection](https://github.com/HitTheStars/Ros_arma3_Connection)

---

**祝您使用愉快！Happy Flying! 🚁**
