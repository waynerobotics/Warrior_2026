# =========================================================
# Warrior Docker Image for x86_64 Platform with CUDA
# =========================================================

# ------------ Arguments ------------
ARG CUDA_VERSION=12.6.0
ARG UBUNTU_VERSION=22.04


# ------------ Base Image ------------
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

LABEL maintainer="Yihao Cai <yihaocai007@gmail.com>" \
      version="v2.0.0" \
      description="X86_64 Platform: ROS2 Humble + CUDA 12.6 + Robotics Libraries" \
      license="Apache-2.0"


# ------------ System Environment ------------
ARG USER_UID=1000
ARG USERNAME=igvc
ARG HOME_PATH=/home/${USERNAME}
ARG THIRD_PARTY_PATH=${HOME_PATH}/third_party

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8


# ------------ Basic Tools ------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget gnupg2 lsb-release ca-certificates git vim build-essential \
    cmake python3 python3-pip locales iproute2 net-tools bash-completion \
    software-properties-common nmap lsof libglfw3-dev locate liburdfdom-headers-dev \
    liburdfdom-dev liboctomap-dev \
    && locale-gen en_US.UTF-8


# ------------ Install ROS2 ------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-rosinstall python3-vcstool python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-backward-ros ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 

RUN rosdep init || true && rosdep update


# ------------ Install Robotics Dependencies ------------
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    python3-opencv \
    python3-pyqt5 \
    python3-yaml \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*



# ------------ Create user ------------
RUN useradd -m ${USERNAME} && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USERNAME}
WORKDIR ${HOME_PATH}


# ------------ ROS2 Workspace ------------
ARG MAKE_JOBS=10
ARG ROS2_WS_NAME=warrior_ws
ARG ROS2_WS_PATH=/home/${USERNAME}/${ROS2_WS_NAME}
SHELL ["/bin/bash", "-c"]


# ------------ PyTorch + torchvision + torchaudio ------------
RUN pip3 install --upgrade pip && \
    pip3 install torch==2.3.0 torchvision==0.18.0 torchaudio==2.3.0 --index-url https://download.pytorch.org/whl/cu121


# ------------ Extra Dependencies ------------
RUN sudo apt-get update && sudo apt-get install -y gnome-terminal dbus-x11 usbutils joystick jstest-gtk kmod
RUN mkdir -p ${HOME_PATH}/.config/jstest-gtk


# ------------ Set Paths ------------
RUN echo '' >> ${HOME_PATH}/.bashrc
RUN echo '################## Add CUDA Library ##################' >> ${HOME_PATH}/.bashrc
RUN echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> ${HOME_PATH}/.bashrc
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> ${HOME_PATH}/.bashrc


# ------------ Set Other ENV ------------
ENV ROS_DOMAIN_ID=0
ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
ENV PYTHONPATH=/usr/lib/python3/dist-packages:${ROS2_WS_PATH}/install/lib/python3.10/site-packages


# Switch to root for SSH git clone
USER root


# ------------ Pull private repo ------------
RUN --mount=type=ssh \
    mkdir -p ~/.ssh && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts && \
    mkdir -p warrior_ws/src/ && cd warrior_ws/src/ && \
    git clone git@github.com:waynerobotics/Warrior_2026.git && \
    chown -R ${USERNAME}:${USERNAME} ${HOME_PATH}/warrior_ws


# ------------ Build custom ROS2 packages ------------
USER ${USERNAME}
WORKDIR ${ROS2_WS_PATH}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --packages-up-to \
        warrior_bringup \
        unitree_lidar_ros2 \
        unitree_l2_lidar \
        insta360_camera \
        neural_engine \
        omnivision \
      --symlink-install || true


# ------------ Entrypoint ------------
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]