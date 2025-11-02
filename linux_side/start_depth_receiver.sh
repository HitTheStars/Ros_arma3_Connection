#!/bin/bash
# Arma 3 Depth Receiver 快速启动脚本

echo "========================================"
echo "  Arma 3 Depth Image Receiver"
echo "========================================"
echo ""

# 检查 ROS 是否安装
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS 未安装或未 source"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查 catkin workspace 是否存在
if [ ! -d "$HOME/catkin_ws" ]; then
    echo "[WARNING] catkin_ws 不存在，正在创建..."
    mkdir -p $HOME/catkin_ws/src
    cd $HOME/catkin_ws
    catkin_make
fi

# Source catkin workspace
source $HOME/catkin_ws/devel/setup.bash

# 检查 arma3_ros_bridge 包是否存在
if [ ! -d "$HOME/catkin_ws/src/arma3_ros_bridge" ]; then
    echo "[WARNING] arma3_ros_bridge 包不存在，正在创建符号链接..."
    ln -s $(pwd)/ros_nodes $HOME/catkin_ws/src/arma3_ros_bridge
    cd $HOME/catkin_ws
    catkin_make
    source $HOME/catkin_ws/devel/setup.bash
fi

echo "[INFO] 启动深度图像接收节点..."
echo "[INFO] 按 Ctrl+C 退出"
echo ""

# 启动 ROS 节点
roslaunch arma3_ros_bridge arma3_depth_receiver.launch
