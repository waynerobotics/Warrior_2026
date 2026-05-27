#!/bin/bash
# #################################
# Warrior 2026 Dependency Installer
#
# Installs everything needed to run:
#   - warrior.launch.py        (hardware / real robot)
#   - warrior.gazebo.launch.py (Gazebo Harmonic simulation)
# #################################

# Check if script is run as root/sudo
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo bash $0"
    exit 1
fi

# Set username variable (user who owns the workspace)
USERNAME=fire
WARRIOR_WS=/home/$USERNAME/ros2_ws

# #################################
# Locale
# #################################
apt update && apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# #################################
# ROS 2 Jazzy Repository
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# #################################
apt install -y software-properties-common curl git
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# #################################
# Gazebo Harmonic Repository
# https://gazebosim.org/docs/harmonic/install_ubuntu/
# #################################
curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt update && apt upgrade -y

# #################################
# ROS 2 Jazzy — Core
# #################################
apt install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-ament-cmake \
    python3-colcon-common-extensions \
    python3-rosdep

# #################################
# Robot Description / URDF
# #################################
apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-urdf \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui

# #################################
# ros2_control — Hardware & Simulation
# Covers: controller_manager, diff_drive_controller,
#         joint_state_broadcaster, hardware_interface
# #################################
apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    libeigen3-dev

# #################################
# Gazebo Harmonic + ROS-Gz Bridge
# Fixes: libgz_ros2_control-system.so not found
# #################################
apt install -y \
    gz-harmonic \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control

# #################################
# Teleop / Joystick
# Used in both launch files
# #################################
apt install -y \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard

# #################################
# Navigation (warrior_navigation)
# #################################
apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-robot-localization

# #################################
# GPS / Sensor msgs (warrior_gps)
# #################################
apt install -y \
    ros-jazzy-nmea-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs

# #################################
# ROS 2 WebSocket bridge (optional, for remote UI)
# #################################
apt install -y \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-rosapi

# #################################
# Build the Warrior workspace
# #################################
source /opt/ros/jazzy/setup.bash

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
sudo -u $USERNAME rosdep update

# Fix ownership and build
chown -R $USERNAME:$USERNAME $WARRIOR_WS
cd $WARRIOR_WS
sudo -u $USERNAME bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# #################################
# .bashrc — source ROS and workspace
# #################################
BASHRC=/home/$USERNAME/.bashrc

grep -qxF 'source /opt/ros/jazzy/setup.bash' $BASHRC || \
    echo 'source /opt/ros/jazzy/setup.bash' >> $BASHRC

grep -qxF "source $WARRIOR_WS/install/setup.bash" $BASHRC || \
    echo "source $WARRIOR_WS/install/setup.bash" >> $BASHRC

grep -qxF "alias sorce='source ~/.bashrc'" $BASHRC || \
    echo "alias sorce='source ~/.bashrc'" >> $BASHRC

echo ""
echo "======================================"
echo " Warrior dependencies installed!"
echo " Run: source ~/.bashrc"
echo " Then: ros2 launch warrior_bringup warrior.gazebo.launch.py"
echo "======================================"
