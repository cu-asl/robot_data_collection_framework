# RobotDataCollectionFramework
This repository contains all file needed to run a framework to do data collection using ROS and Gazebo

## Installation
### Install ROS
1. Install at least ros-base
2. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
3. source ~/.bashrc
4. sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
5. sudo rosdep init && rosdep update
### Install catkin tools
1. wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
2. sudo apt update
3. sudo apt-get install python3-catkin-tools
### Install this package and requirements
1. sudo apt install ros-noetic-moveit ros-noetic-controller-interface ros-noetic-control-toolbox ros-noetic-eigen-conversions ros-noetic-gazebo-ros ros-noetic-controller-manager ros-noetic-tf-conversions ros-noetic-robot-state-publisher ros-noetic-rqt ros-noetic-rqt-controller-manager ros-noetic-rqt-publisher ros-noetic-rqt-image-view ros-noetic-rqt-graph ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-robot-mechanism-controllers ros-noetic-joint-state-publisher ros-noetic-joint-state-controller ros-noetic-transmission-interface ros-noetic-position-controllers ros-noetic-joint-trajectory-controller
2. mkdir -p ~/catkin_ws/src 
3. Download and upzip this package to the folder in 2
4. cd ~/catkin_ws/src
5. git clone https://github.com/ros-industrial/universal_robot.git 
6. git clone https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
7. git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
8. git clone https://github.com/JenniferBuehler/general-message-pkgs.git
9. cd ~/catkin_ws
10. catkin build
11. echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
12. source ~/.bashrc
13. roscd ur5_data_collect_fw/scripts
14. chmod +x *
### Install python3 library
1. sudo apt install python3-pip
2. pip3 install pandas psutil urdfpy
### Demo
1. roscd ur5_data_collect_fw/scripts
2. python3 run_loop.py -task 1 -open config/world_random_peg/1 -timeout 60 -num_open 3
