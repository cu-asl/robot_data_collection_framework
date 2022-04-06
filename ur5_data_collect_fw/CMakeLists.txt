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
  # gazebo
)

# find_package(Eigen3 REQUIRED)

# find_package(gazebo REQUIRED)
# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# add_library(hello_world SHARED plugins/hello_world.cpp)
# target_link_libraries(hello_world ${GAZEBO_LIBRARIES})


# add_executable(hello_worlds plugins/hello_world.cpp)
# target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  joints.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Object.srv
  PoseGoal.srv
  PoseGoal_RPY.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_data_collect_fw
  CATKIN_DEPENDS 
    moveit_ros_planning_interface
  DEPENDS 
    # system_lib
    # EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
# include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur5_data_collect_fw.cpp
# )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# add_executable(pick_place src/pick_place.cpp)
# target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(robot_block_simulator src/robot_block_simulator.cpp)
# target_link_libraries(robot_block_simulator ${catkin_LIBRARIES})
# install(TARGETS robot_block_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})