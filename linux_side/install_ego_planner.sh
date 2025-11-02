#!/bin/bash
# EGO-Planner-v2 Installation and Setup Script
# This script installs and configures EGO-Planner-v2 for multi-UAV path planning

set -e

echo "======================================="
echo "EGO-Planner-v2 Installation Script"
echo "======================================="

# Check if ROS is installed
if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
    echo "Error: ROS Noetic is not installed. Please install ROS Noetic first."
    echo "Visit: http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

# Source ROS
source /opt/ros/noetic/setup.bash

# Create catkin workspace if it doesn't exist
CATKIN_WS=~/catkin_ws
if [ ! -d "$CATKIN_WS" ]; then
    echo "Creating catkin workspace..."
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
fi

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-stage-ros \
    ros-noetic-map-server \
    ros-noetic-laser-geometry \
    ros-noetic-interactive-markers \
    ros-noetic-tf \
    ros-noetic-pcl-* \
    ros-noetic-libg2o \
    ros-noetic-rviz \
    libarmadillo-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libdw-dev

# Copy EGO-Planner to catkin workspace
echo "Copying EGO-Planner to catkin workspace..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EGO_PLANNER_DIR="$SCRIPT_DIR/ego_planner"

# Use main_ws as the primary workspace
if [ -d "$EGO_PLANNER_DIR/main_ws/src" ]; then
    echo "Copying main_ws to catkin workspace..."
    cp -r $EGO_PLANNER_DIR/main_ws/src/* $CATKIN_WS/src/
else
    echo "Error: EGO-Planner main_ws not found at $EGO_PLANNER_DIR"
    exit 1
fi

# Build the workspace
echo "Building catkin workspace..."
cd $CATKIN_WS
catkin_make

# Source the workspace
source $CATKIN_WS/devel/setup.bash

echo "======================================="
echo "EGO-Planner-v2 installed successfully!"
echo "======================================="
echo ""
echo "To use EGO-Planner, source the workspace:"
echo "  source ~/catkin_ws/devel/setup.bash"
echo ""
echo "To run EGO-Planner simulation:"
echo "  roslaunch ego_planner simple_run.launch"
echo ""
echo "To integrate with Arma 3:"
echo "  roslaunch arma3_ros_bridge arma3_bridge.launch"
echo ""
