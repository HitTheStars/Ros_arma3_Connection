# ARK 框架集成指南

## 什么是 ARK？

**ARK (Autonomous Robotics Kit)** 是一个轻量级的、Python 优先的机器人框架，专为机器人学习和开发而设计。ARK 提供了一个模块化的、基于节点的架构，用于管理传感器、执行器、仿真和控制策略之间的通信。

与传统的 ROS 框架不同，ARK 优先考虑易用性、快速原型设计，并与 PyBullet、MuJoCo 和现代机器学习库无缝集成，使其成为研究和实际机器人部署的理想选择。

## 为什么集成 ARK？

在我们的 Arma 3 与 ROS 集成系统中，ARK 框架提供了以下增强功能：

1. **机器学习增强控制**
   - 使用扩散策略 (Diffusion Policies) 训练无人机控制策略
   - 通过模仿学习从人类演示中学习飞行行为
   - 支持 VLA (Vision-Language-Action) 模型

2. **轻量级替代方案**
   - ARK 比 ROS 更轻量，安装和配置更简单
   - Python 优先，无需 C++ 编程
   - 更快的原型设计和迭代

3. **Sim-to-Real 工作流**
   - 在 Arma 3 仿真中训练策略
   - 无缝转移到真实无人机
   - 支持多种仿真器（PyBullet、MuJoCo、Genesis）

4. **现代 ML 库集成**
   - 与 PyTorch、TensorFlow 等深度学习框架无缝集成
   - 支持 Gym 风格的环境接口
   - 易于集成预训练模型

## 系统架构（带 ARK）

```
┌─────────────────────────────────────────────────────────────┐
│                  Linux 虚拟机 (ROS + ARK)                    │
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
│  │      ARK 集成节点 (ark_integration)                 │  │
│  │  - 接收 ROS 数据                                    │  │
│  │  - ARK 策略推理                                     │  │
│  │  - 发布 ML 增强指令                                 │  │
│  └──────────────────────────────────────────────────────┘  │
│                        ↕                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │      EGO-Planner 接口 (ego_planner_interface)       │  │
│  │  - 融合 ARK 和 EGO-Planner 的输出                  │  │
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

## 安装 ARK 框架

### 方法 1：使用提供的安装脚本

我们提供了一个自动化安装脚本，可以一键安装 ARK 框架：

```bash
cd linux_side
chmod +x install_ark.sh
./install_ark.sh
```

### 方法 2：手动安装

如果您想手动安装 ARK，请按照以下步骤操作：

```bash
# 1. 创建 Conda 环境
conda create -n ark_env python=3.10 -y
conda activate ark_env

# 2. 创建 ARK 工作空间
mkdir -p ~/ark_workspace
cd ~/ark_workspace

# 3. 克隆并安装 ARK 框架
git clone https://github.com/Robotics-Ark/ark_framework.git
cd ark_framework
pip install -e .
cd ..

# 4. 克隆并安装 ARK Types
git clone https://github.com/Robotics-Ark/ark_types.git
cd ark_types
pip install -e .
cd ..

# 5. 验证安装
ark --help
```

## 使用 ARK 集成

### 启动带 ARK 的系统

```bash
# 1. 激活 Conda 环境
conda activate ark_env

# 2. 激活 ROS 环境
source ~/catkin_ws/devel/setup.bash

# 3. 启动带 ARK 的 ROS 节点
roslaunch arma3_ros_bridge arma3_bridge_with_ark.launch
```

### 禁用 ARK 集成

如果您不想使用 ARK，可以通过以下方式禁用：

```bash
# 使用原始的 launch 文件
roslaunch arma3_ros_bridge arma3_bridge.launch

# 或者在带 ARK 的 launch 文件中禁用
roslaunch arma3_ros_bridge arma3_bridge_with_ark.launch use_ark:=false
```

## ARK 与 ROS 的协同工作

ARK 和 ROS 可以在同一系统中协同工作：

1. **ROS 负责**：
   - 与 Arma 3 的通信
   - 图像处理和点云生成
   - EGO-Planner 路径规划

2. **ARK 负责**：
   - 机器学习策略推理
   - 扩散策略生成
   - 模仿学习

3. **集成节点负责**：
   - 在 ROS 和 ARK 之间传递数据
   - 融合 ARK 和 EGO-Planner 的输出
   - 发布最终的控制指令

## 训练自定义策略

ARK 框架允许您训练自定义的无人机控制策略。以下是一个简单的示例：

```python
import ark
from ark import ArkEnv

# 创建 Gym 风格的环境
env = ArkEnv(
    robot='quadrotor',
    sensors=['camera', 'imu'],
    task='navigation'
)

# 训练扩散策略
from ark.policies import DiffusionPolicy

policy = DiffusionPolicy()
policy.train(env, num_epochs=100)

# 保存策略
policy.save('uav_navigation_policy.pth')
```

训练完成后，您可以在 `ark_integration.py` 中加载并使用这个策略。

## 示例应用

### 1. 扩散策略导航

使用 ARK 的扩散策略进行无人机导航：

```python
from ark.policies import DiffusionPolicy

# 加载预训练的策略
policy = DiffusionPolicy.load('uav_navigation_policy.pth')

# 在 ARK 集成节点中使用
def ark_navigation_callback(observation):
    action = policy.predict(observation)
    return action
```

### 2. 模仿学习

从人类演示中学习飞行行为：

```python
from ark.learning import ImitationLearning

# 加载演示数据
demonstrations = load_demonstrations('flight_demos.pkl')

# 训练模仿学习模型
il_model = ImitationLearning()
il_model.train(demonstrations)

# 使用模型
action = il_model.predict(current_state)
```

## 故障排除

### 问题 1：ARK 未安装

**症状**：启动节点时显示 "ARK framework not installed"

**解决方案**：
1. 确保已运行 `install_ark.sh` 脚本
2. 确保已激活 `ark_env` Conda 环境
3. 运行 `ark --help` 验证安装

### 问题 2：Conda 环境冲突

**症状**：ROS 和 ARK 的 Python 环境冲突

**解决方案**：
1. 确保先激活 `ark_env`，再激活 ROS 环境
2. 或者在 ROS 环境中直接安装 ARK（不推荐）

### 问题 3：ARK 节点无法启动

**症状**：`ark_integration.py` 无法启动

**解决方案**：
1. 检查 `use_ark` 参数是否设置为 `true`
2. 查看 ROS 日志：`rosrun rqt_console rqt_console`
3. 确保 ARK 框架已正确安装

## 参考资料

- [ARK 官方文档](https://arkrobotics.notion.site/ARK-Home-22be053d9c6f8096bcdbefd6276aba61)
- [ARK GitHub 仓库](https://github.com/Robotics-Ark/ark_framework)
- [ARK 研究论文](https://arxiv.org/abs/2506.21628)
- [ARK 教程视频](https://www.youtube.com/watch?v=hTFl_bdYt7U)

## 下一步

1. **探索 ARK 示例**：查看 ARK 仓库中的示例代码
2. **训练自定义策略**：使用 ARK 训练适合您应用的策略
3. **集成到项目**：将训练好的策略集成到 `ark_integration.py` 中
4. **实验和优化**：在 Arma 3 仿真中测试和优化策略

---

**注意**：ARK 集成是可选的。如果您只需要基本的路径规划功能，可以不安装 ARK，直接使用 EGO-Planner。
