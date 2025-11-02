#!/bin/bash
# ARK Framework Installation Script
# This script installs the ARK framework for machine learning-enhanced UAV control

set -e

echo "====================================="
echo "ARK Framework Installation Script"
echo "====================================="

# Check if conda is installed
if ! command -v conda &> /dev/null; then
    echo "Error: Conda is not installed. Please install Miniconda or Anaconda first."
    echo "Visit: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# Create ARK workspace
echo "Creating ARK workspace..."
mkdir -p ~/ark_workspace
cd ~/ark_workspace

# Create conda environment
echo "Creating conda environment 'ark_env'..."
conda create -n ark_env python=3.10 -y
source $(conda info --base)/etc/profile.d/conda.sh
conda activate ark_env

# Clone and install ARK framework
echo "Cloning ARK framework..."
if [ ! -d "ark_framework" ]; then
    git clone https://github.com/Robotics-Ark/ark_framework.git
fi

cd ark_framework
echo "Installing ARK framework..."
pip install -e .
cd ..

# Clone and install ARK types
echo "Cloning ARK types..."
if [ ! -d "ark_types" ]; then
    git clone https://github.com/Robotics-Ark/ark_types.git
fi

cd ark_types
echo "Installing ARK types..."
pip install -e .
cd ..

# Verify installation
echo "Verifying ARK installation..."
ark --help

echo "====================================="
echo "ARK Framework installed successfully!"
echo "====================================="
echo ""
echo "To use ARK, activate the environment:"
echo "  conda activate ark_env"
echo ""
echo "To integrate ARK with ROS:"
echo "  1. Activate both conda and ROS environments"
echo "  2. Run: roslaunch arma3_ros_bridge arma3_bridge_with_ark.launch"
echo ""
