#!/bin/bash
# Install ROS 2 Humble + Gazebo Fortress on Ubuntu 22.04 (WSL2) and set up ~/.bashrc.
set -euo pipefail

if [[ "$EUID" -ne 0 ]]; then
    echo "Run with sudo: sudo bash $(basename "$0")" >&2
    exit 1
fi

USERNAME="${SUDO_USER:-$USER}"
BASHRC="/home/$USERNAME/.bashrc"

# ROS 2 apt repo
apt update
apt install -y locales software-properties-common curl gnupg lsb-release
locale-gen en_US.UTF-8
add-apt-repository -y universe
install -d -m 0755 /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 Humble desktop + Gazebo Fortress + ros2_control (needed by warrior_control)
apt update
apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control \
    libeigen3-dev

# ~/.bashrc lines
add_line() {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}
add_line 'source /opt/ros/humble/setup.bash'
add_line "alias sorce='source ~/.bashrc'"
add_line "alias ros_rebuild='cd ~/ros2_ws && colcon build && source install/setup.bash'"

echo "Done. Run: source ~/.bashrc"
