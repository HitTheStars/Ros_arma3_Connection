cmake_minimum_required(VERSION 3.0.2)
project(arma3_ros_bridge)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  tf
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs
    visualization_msgs
    cv_bridge
    tf
)

## Install Python scripts
catkin_install_python(PROGRAMS
  arma3_ros_bridge.py
  ego_planner_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  ../image_processing/stereo_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install rviz config
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
