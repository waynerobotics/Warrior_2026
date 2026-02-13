#!/bin/bash
set -e  # Exit on error

# ---------- Environment Initialization ----------
# Set locale (avoid ROS2 locale warning)
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# ROS distribution (allow external override)
: "${ROS_DISTRO:=humble}"

# GPU visualization support (allow RViz / IsaacSim)
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# Terminal colors
echo '' >> ~/.bashrc
echo "export PS1='\[\e[37;40m\]\[\e[1m\][\u\[\e[33;40m\]@\[\e[31;40m\]\H\[\e[35;40m\]:\[\e[36;40m\]\W\[\e[37;40m\]]\[\e[34;40m\]<\d \t>\[\e[0m\]$'" >> ~/.bashrc


# ---------- Load ROS2 Environment ----------
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "🧠 Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "⚠️  ROS2 environment not found at /opt/ros/${ROS_DISTRO}"
fi

# ---------- Load User Workspace ----------
USERNAME=$(whoami)
ROS_WS="/home/${USERNAME}/warrior_ws"
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    echo "🚀 Sourcing workspace: ${ROS_WS}/install/setup.bash"
    source "${ROS_WS}/install/setup.bash"
else
    echo "ℹ️  Workspace not built yet (no install/setup.bash found)"
fi

# ---------- Enable bash completion & ROS2 autocompletion ----------
if [ -f /etc/bash_completion ]; then
    source /etc/bash_completion
fi
echo "" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
[ -f "${ROS_WS}/install/setup.bash" ] && echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc


# ---------- Fix Permissions for Input Devices ----------
sudo setfacl -m u:${USERNAME}:rwx /dev/input/*

# ---------- VIM settings ----------
echo "set number" >> ~/.vimrc
echo "syntax on" >> ~/.vimrc
echo "set tabstop=4" >> ~/.vimrc
echo "set ignorecase" >> ~/.vimrc
echo "set encoding=utf-8" >> ~/.vimrc


# ---------- Environment Prompt ----------
echo "✅ Warrior container ready."
echo "   ROS_DISTRO: ${ROS_DISTRO}"
echo "   Workspace : ${ROS_WS}"
echo "   GPU       : ${NVIDIA_VISIBLE_DEVICES}"
echo "---------------------------------------------"

# ---------- Execute User Command ----------
# If no arguments are provided, start a bash shell
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi