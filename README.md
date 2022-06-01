# RobotDataCollectionFramework
This repository contains all file needed to run a framework to do data collection using ROS and Gazebo

## Installation
### Install ROS
Following the ubuntu install of ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init && rosdep update
```
### Install catkin tools
```
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install python3-catkin-tools
```
### Install this package and requirements
```
sudo apt install ros-noetic-moveit ros-noetic-controller-interface ros-noetic-control-toolbox ros-noetic-eigen-conversions ros-noetic-gazebo-ros ros-noetic-controller-manager ros-noetic-tf-conversions ros-noetic-robot-state-publisher ros-noetic-rqt ros-noetic-rqt-controller-manager ros-noetic-rqt-publisher ros-noetic-rqt-image-view ros-noetic-rqt-graph ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-robot-mechanism-controllers ros-noetic-joint-state-publisher ros-noetic-joint-state-controller ros-noetic-transmission-interface ros-noetic-position-controllers ros-noetic-joint-trajectory-controller
mkdir -p ~/catkin_ws/src 
Download and upzip this package to the folder in 2
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git 
git clone https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
roscd ur5_data_collect_fw/scripts
chmod +x *
```
### Install python3 library
```
sudo apt install python3-pip
pip3 install pandas psutil urdfpy
```
## Demo
### Peg in Hole
#### Execute the following Commands
```
roscd ur5_data_collect_fw/scripts
python3 run_loop.py -task 1 -open config/world_random_peg/1 -timeout 60 -num_open 3
```
#### Result while running
Gazebo will be opened and the simulation of peg inserting to a hole will run

<img src="https://user-images.githubusercontent.com/91130166/171422712-e7116999-9dfd-48e1-9d78-c3cf910fbe8c.png" width="600">

#### Result after the program is finished
35 files should be created in the newest folder (highest number) in ur5_data_collect_fw/rec

<img src="https://user-images.githubusercontent.com/91130166/171423484-9c31adcb-4b58-4b31-8f70-6868c939c364.png" width="600">

### Pick and Place
#### Execute the following Commands
```
roscd ur5_data_collect_fw/scripts
python3 run_loop.py -task 3 -open config/world_random_pick/1 -timeout 240 -num_open 3
```
#### Result while running
Gazebo will be opened and the simulation of picking and placing objects will run

<img src="https://user-images.githubusercontent.com/91130166/171431801-007b54e9-9043-45d9-9fc1-2625ae0fd677.png" width="600">

#### Result after the program is finished
34 files should be created in the newest folder (highest number) in ur5_data_collect_fw/rec

<img src="https://user-images.githubusercontent.com/91130166/171431925-7b77493a-c0a7-44a2-9769-facc4d01eb4a.png" width="600">

### If want to stop the program
1. ``ctrl`` + ``z`` at the terminal running run_loop.py
2. In terminal, type ``ps aux | grep roslaunch``
3. Kill the second process from the last
<img src="https://user-images.githubusercontent.com/91130166/171435686-76b97179-1205-4cb3-8f0e-d7959a33f0ed.png" width="600">

