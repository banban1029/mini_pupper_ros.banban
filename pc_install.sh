#!/bin/bash
######################################################################################
# ROS2
#
# This stack will consist of ROS2 install
#
# To install
#    ./pc_install.sh
######################################################################################

# Update package lists
cd ~
sudo apt update
mkdir minipupper_ws
cd ~/minipupper_ws
# Install ROS 2 Humble setup scripts
if ! [ -d "ros2_setup_scripts_ubuntu" ]; then
  git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
fi
~/minipupper_ws/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash

# Create ROS 2 workspace and clone Mini Pupper ROS repository
mkdir -p ~/minipupper_ws/ros2_ws/src
cd ~/minipupper_ws/ros2_ws/src
if ! [ -d "mini_pupper_ros.banban" ]; then
  git clone https://github.com/banban1029/mini_pupper_ros.banban -b ros2-dev mini_pupper_ros.banban

fi
vcs import < mini_pupper_ros.banban/.minipupper.repos --recursive

# Install dependencies and build the ROS 2 packages
cd ~/minipupper_ws/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
sudo apt install -y ros-humble-rqt*
pip3 install simple_pid
colcon build --symlink-install
