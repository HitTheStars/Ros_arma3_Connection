#!/bin/bash
# Quick Start Script for Linux Side
# This script starts all ROS nodes

set -e

echo "=============================================="
echo "Starting Arma 3 <-> ROS Bridge (Linux Side)"
echo "=============================================="
echo ""

# Source ROS
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "Error: ROS Noetic is not installed."
    exit 1
fi

# Source catkin workspace
if [ -f "~/catkin_ws/devel/setup.bash" ]; then
    source ~/catkin_ws/devel/setup.bash
fi

# Check if user wants to use ARK
echo "Do you want to start with ARK integration? (y/n)"
read -n 1 -r USE_ARK
echo ""

if [[ $USE_ARK =~ ^[Yy]$ ]]; then
    echo "Starting ROS nodes with ARK integration..."
    roslaunch arma3_ros_bridge arma3_bridge_with_ark.launch
else
    echo "Starting ROS nodes without ARK..."
    roslaunch arma3_ros_bridge arma3_bridge.launch
fi
