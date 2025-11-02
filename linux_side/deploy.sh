#!/bin/bash
# One-Click Deployment Script for Linux Side
# This script installs all dependencies and sets up the ROS environment

set -e

echo "=============================================="
echo "Arma 3 <-> ROS Integration - Linux Deployment"
echo "=============================================="
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if running on Ubuntu
if [ ! -f "/etc/lsb-release" ]; then
    echo "Error: This script is designed for Ubuntu. Exiting."
    exit 1
fi

# Check Ubuntu version
source /etc/lsb-release
if [ "$DISTRIB_RELEASE" != "20.04" ]; then
    echo "Warning: This script is tested on Ubuntu 20.04. You are running $DISTRIB_RELEASE."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Step 1/6: Installing ROS Noetic..."
echo "-----------------------------------"

# Check if ROS is already installed
if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
    echo "Installing ROS Noetic..."
    
    # Setup sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Setup keys
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    # Update and install
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full
    
    # Initialize rosdep
    sudo rosdep init
    rosdep update
    
    # Setup environment
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source /opt/ros/noetic/setup.bash
    
    # Install build tools
    sudo apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
    echo "ROS Noetic installed successfully!"
else
    echo "ROS Noetic is already installed."
    source /opt/ros/noetic/setup.bash
fi

echo ""
echo "Step 2/6: Installing Python dependencies..."
echo "-------------------------------------------"

# Install Python packages
sudo apt-get install -y python3-pip
pip3 install --upgrade pip
pip3 install opencv-python numpy pillow

echo ""
echo "Step 3/6: Installing EGO-Planner-v2..."
echo "--------------------------------------"

# Run EGO-Planner installation script
if [ -f "$SCRIPT_DIR/install_ego_planner.sh" ]; then
    chmod +x "$SCRIPT_DIR/install_ego_planner.sh"
    "$SCRIPT_DIR/install_ego_planner.sh"
else
    echo "Warning: install_ego_planner.sh not found. Skipping EGO-Planner installation."
fi

echo ""
echo "Step 4/6: Setting up ROS workspace..."
echo "--------------------------------------"

# Create catkin workspace if it doesn't exist
CATKIN_WS=~/catkin_ws
if [ ! -d "$CATKIN_WS" ]; then
    echo "Creating catkin workspace..."
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
fi

# Copy ROS nodes to catkin workspace
echo "Copying ROS nodes..."
ROS_PKG_DIR=$CATKIN_WS/src/arma3_ros_bridge
mkdir -p $ROS_PKG_DIR

# Copy all ROS node files
cp -r $SCRIPT_DIR/ros_nodes/* $ROS_PKG_DIR/
cp -r $SCRIPT_DIR/image_processing $ROS_PKG_DIR/

# Build the workspace
cd $CATKIN_WS
catkin_make

# Source the workspace
source $CATKIN_WS/devel/setup.bash
echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

echo ""
echo "Step 5/6: Installing ARK framework (optional)..."
echo "-------------------------------------------------"

read -p "Do you want to install ARK framework for ML-enhanced control? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [ -f "$SCRIPT_DIR/install_ark.sh" ]; then
        chmod +x "$SCRIPT_DIR/install_ark.sh"
        "$SCRIPT_DIR/install_ark.sh"
    else
        echo "Warning: install_ark.sh not found. Skipping ARK installation."
    fi
else
    echo "Skipping ARK installation."
fi

echo ""
echo "Step 6/6: Network configuration..."
echo "-----------------------------------"

# Get IP address
IP_ADDR=$(hostname -I | awk '{print $1}')
echo "Your Linux VM IP address is: $IP_ADDR"
echo ""
echo "IMPORTANT: Configure the Windows bridge program to connect to this IP."
echo "Edit windows_side/bridge_program/arma3_bridge_enhanced.py:"
echo "  SERVER_IP = '$IP_ADDR'"
echo ""

# Create a configuration file
cat > $SCRIPT_DIR/network_config.txt << EOF
Linux VM IP Address: $IP_ADDR
ROS Bridge Port: 5555

Windows Configuration:
1. Edit arma3_bridge_enhanced.py
2. Set SERVER_IP = '$IP_ADDR'
3. Set SERVER_PORT = 5555
EOF

echo "Network configuration saved to: $SCRIPT_DIR/network_config.txt"

echo ""
echo "=============================================="
echo "Deployment Complete!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "1. Start ROS nodes:"
echo "   roslaunch arma3_ros_bridge arma3_bridge.launch"
echo ""
echo "2. On Windows, run the bridge program:"
echo "   python arma3_bridge_enhanced.py"
echo ""
echo "3. Start Arma 3 and load the Camera.Altis mission"
echo ""
echo "For more information, see the README.md file."
echo ""
