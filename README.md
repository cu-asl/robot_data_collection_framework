# RobotDataCollectionFramework
This repository contains all file needed to run a framework to do data collection using ROS and Gazebo

## Installation
### Install ROS
Following the ubuntu install of ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
2. sudo apt install curl # if you haven't already installed curl
3. curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
4. sudo apt update
5. sudo apt install ros-noetic-ros-base
6. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
7. source ~/.bashrc
8. sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
9. sudo rosdep init && rosdep update
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
## Demo
### Peg in Hole
#### Execute the following Commands
1. roscd ur5_data_collect_fw/scripts
2. python3 run_loop.py -task 1 -open config/world_random_peg/1 -timeout 60 -num_open 3
#### Result while running
Gazebo will be openned and the simulation of peg inserting to a hole will run
![Screenshot from 2022-06-01 20-59-41](https://user-images.githubusercontent.com/91130166/171422712-e7116999-9dfd-48e1-9d78-c3cf910fbe8c.png)
#### Result after the program is finished
35 files should be created in the newest folder (highest number) in ur5_data_collect_fw/rec
![Screenshot from 2022-06-01 21-03-34](https://user-images.githubusercontent.com/91130166/171423484-9c31adcb-4b58-4b31-8f70-6868c939c364.png)
### Pick and Place
#### Execute the following Commands
1. roscd ur5_data_collect_fw/scripts
2. python3 run_loop.py -task 3 -open config/world_random_pick/1 -timeout 240 -num_open 3
#### Result while running
Gazebo will be openned and the simulation of picking and placing objects will run
![Screenshot from 2022-06-01 21-10-32](https://user-images.githubusercontent.com/91130166/171431801-007b54e9-9043-45d9-9fc1-2625ae0fd677.png)
#### Result after the program is finished
34 files should be created in the newest folder (highest number) in ur5_data_collect_fw/rec
![Screenshot from 2022-06-01 21-41-48](https://user-images.githubusercontent.com/91130166/171431925-7b77493a-c0a7-44a2-9769-facc4d01eb4a.png)
