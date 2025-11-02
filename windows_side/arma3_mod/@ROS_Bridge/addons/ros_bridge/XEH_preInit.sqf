// ROS Bridge Pre-Init Script
// This script runs before the mission starts

diag_log "[ROS Bridge] Pre-Init started";

// Initialize global variables
ROS_Version = "1.0.0";
ROS_ModName = "ROS Bridge for Arma 3";

diag_log format ["[ROS Bridge] %1 v%2 loaded", ROS_ModName, ROS_Version];
diag_log "[ROS Bridge] Pre-Init completed";
