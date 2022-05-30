cmake_minimum_required(VERSION 3.0.2)
project(ur5_data_collect_fw)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    moveit_ros_planning_interface
  DEPENDS 
)