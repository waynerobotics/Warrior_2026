#!/bin/bash
# #################################
# Basic Includes
# 
# #################################

# Check if script is run as root/sudo
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo bash $0"
    exit 1
fi

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
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ament-cmake
sudo apt install -y ros-jazzy-urdf ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher

# #################################
# Turtle Bot and Gazebo Sim Install
# https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
# #################################

sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic -y
sudo apt install ros-jazzy-cartographer -y
sudo apt install ros-jazzy-cartographer-ros -y
sudo apt install ros-jazzy-navigation2 -y
sudo apt install ros-jazzy-nav2-bringup -y

source /opt/ros/jazzy/setup.bash
mkdir -p /home/$USERNAME/turtlebot3_ws/src
cd /home/$USERNAME/turtlebot3_ws/src/
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
sudo apt install python3-colcon-common-extensions -y
cd /home/$USERNAME/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
# Fix ownership of workspace to user
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/turtlebot3_ws
colcon build --symlink-install
echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc
echo 'source /home/$USERNAME/turtlebot3_ws/install/setup.bash' >> /home/$USERNAME/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /home/$USERNAME/.bashrc

# #################################
# ROS 2 Websocket suite and API
# https://github.com/RobotWebTools/rosbridge_suite
# #################################
sudo apt install -y ros-jazzy-rosbridge-suite ros-jazzy-rosapi

# #################################
# Aliases
# #################################
echo "alias sorce='source ~/.bashrc'" >> /home/$USERNAME/.bashrc




