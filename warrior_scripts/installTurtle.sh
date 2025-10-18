#!/bin/bash

# #################################
# Basic Includes
# 
# #################################

# Set username variable
USERNAME=fire

sudo apt install -y git

# #################################
# ROS 2.0 Install
# From https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# #################################

# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# Add ROS 2 repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-ros-base ros-jazzy-ament-cmake ros-dev-tools
sudo apt install -y ros-jazzy-urdf ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
sudo apt install -y ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs
sudo apt install -y ros-jazzy-common-interfaces
sudo apt install -y python3-colcon-common-extensions
# Update package cache after installing ROS packages
sudo apt update

# #################################
# Turtle Bot and Gazebo Sim Install
# https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
# #################################

sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential -y
sudo apt install ros-jazzy-hls-lfcd-lds-driver -y
sudo apt install ros-jazzy-turtlebot3-msgs -y
sudo apt install ros-jazzy-dynamixel-sdk -y
sudo apt install ros-jazzy-xacro -y
sudo apt install libudev-dev -y
mkdir -p /home/$USERNAME/turtlebot3_ws/src && cd /home/$USERNAME/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/ld08_driver.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/coin_d4_driver
cd /home/$USERNAME/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd /home/$USERNAME/turtlebot3_ws/

echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc
source /opt/ros/jazzy/setup.bash
# Fix ownership of workspace to user
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/turtlebot3_ws
colcon build --symlink-install --parallel-workers 1
echo 'source /home/$USERNAME/turtlebot3_ws/install/setup.bash' >> /home/$USERNAME/.bashrc
source /home/$USERNAME/turtlebot3_ws/install/setup.bash

# Source ROS environment for ros2 commands
source /opt/ros/jazzy/setup.bash
source /home/$USERNAME/turtlebot3_ws/install/setup.bash

sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /home/$USERNAME/.bashrc
. /home/$USERNAME/.bashrc
echo 'export LDS_MODEL=LDS-02' >> /home/$USERNAME/.bashrc # If you are using LDS-02
. /home/$USERNAME/.bashrc

# #################################
# Aliases
# #################################
echo "alias sorce='source ~/.bashrc'" >> /home/$USERNAME/.bashrc

# #################################
# OpenCR
# #################################
# sudo dpkg --add-architecture armhf  
# sudo apt-get update  
# sudo apt-get install libc6:armhf -y
# export OPENCR_PORT=/dev/ttyACM0  
# export OPENCR_MODEL=burger
# rm -rf ./opencr_update.tar.bz2
# wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2   
# tar -xvf opencr_update.tar.bz2 --overwrite
# cd ./opencr_update  
# ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr  