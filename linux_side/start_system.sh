#!/bin/bash
# Linux 端统一启动脚本
# 启动所有必要的 ROS 节点（深度接收器 + ROS 桥接 + EGO-Planner）

set -e

echo "========================================="
echo "  启动 Arma 3 - ROS 集成系统 (Linux 端)"
echo "========================================="
echo ""

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: ROS 环境未配置"
    echo "   请运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo "✓ ROS 版本: $ROS_DISTRO"

# 获取本机 IP 地址
IP_ADDR=$(hostname -I | awk '{print $1}')
echo "✓ Linux IP 地址: $IP_ADDR"
echo ""

# 检查 ROS Master 是否已运行
if ! pgrep -x "rosmaster" > /dev/null; then
    echo "启动 ROS Master..."
    roscore &
    sleep 3
    echo "✓ ROS Master 已启动"
else
    echo "✓ ROS Master 已在运行"
fi

echo ""
echo "========================================="
echo "  启动深度接收器和 ROS 桥接节点"
echo "========================================="
echo ""

# 启动深度接收器 launch 文件
# 该 launch 文件会同时启动：
# 1. arma3_depth_receiver.py - 深度图像接收器
# 2. arma3_ros_bridge.py - ROS 桥接节点（状态通信）
echo "启动 ROS 节点..."
roslaunch ros_nodes arma3_depth_receiver.launch &

sleep 5

echo ""
echo "========================================="
echo "  系统启动完成"
echo "========================================="
echo ""
echo "✓ 深度接收器已启动，监听端口 5555"
echo "✓ ROS 桥接节点已启动"
echo ""
echo "等待 Arma 3 连接..."
echo ""
echo "ROS 话题列表:"
rostopic list | grep -E "(arma3|camera)" || echo "  (等待数据发布)"
echo ""
echo "按 Ctrl+C 停止系统"
echo ""

# 保持脚本运行
wait
