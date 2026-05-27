# ======================================================
# Warrior Docker Image for Nvidia Jetson with CUDA
# ======================================================

# ------------ Base Image ------------
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

LABEL maintainer="Yihao Cai <yihaocai007@gmail.com>" \
      version="v2.0.0" \
      description="Nvidia Jetson: ROS2 Humble + CUDA 12.6 + Robotics Libraries" \
      license="Apache-2.0"


# ------------ System Environment ------------
ARG USER_UID=1000
ARG USERNAME=warrior    
ARG HOME_PATH=/home/${USERNAME}

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
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-connextdds

RUN rosdep init || true && rosdep update


# ------------ Create user ------------
RUN useradd -m ${USERNAME} && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN mkdir -p ${THIRD_PARTY_PATH} && chown -R ${USERNAME}:${USERNAME} ${THIRD_PARTY_PATH}
USER ${USERNAME}
WORKDIR ${HOME_PATH}

# ------------ ROS2 Workspace ------------
ARG MAKE_JOBS=$(nproc)
ARG ROS2_WS_NAME=edge_ws
ARG ROS2_WS_PATH=/home/${USERNAME}/${ROS2_WS_NAME}
SHELL ["/bin/bash", "-c"]


# ----------- Install PyTorch and toolboxs for JetPack 6.2 -------------
# reference: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
# download source: https://download.pytorch.org/whl/torch/
WORKDIR ${HOME_PATH}
RUN mkdir torch_libs && cd torch_libs && \
    wget https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl -O torch-2.3.0-cp310-cp310-linux_aarch64.whl && \
    wget https://nvidia.box.com/shared/static/xpr06qe6ql3l6rj22cu3c45tz1wzi36p.whl -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl && \
    wget https://nvidia.box.com/shared/static/9agsjfee0my4sxckdpuk9x9gt8agvjje.whl -O torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl && \
    pip3 install torch-2.3.0-cp310-cp310-linux_aarch64.whl torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl
    # cd .. && rm -rf torch_libs


    # ------------ Set Paths ------------
RUN echo '' >> ${HOME_PATH}/.bashrc
RUN echo '################## Add CUDA Library ##################' >> ${HOME_PATH}/.bashrc
RUN echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> ${HOME_PATH}/.bashrc
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> ${HOME_PATH}/.bashrc
RUN echo '' >> ${HOME_PATH}/.bashrc



# ------------ Set Other ENV ------------
ENV ROS_DOMAIN_ID=0
ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
ENV PYTHONPATH=/usr/lib/python3/dist-packages:${ROS2_WS_PATH}/install/lib/python3.10/site-packages


# ------------ Entrypoint ------------
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]