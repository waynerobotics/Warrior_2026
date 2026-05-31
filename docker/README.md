# Warrior Docker

ROS 2 Humble + CUDA 12.6 container for the Warrior platform.

## Layout

```
docker/
  dockerfile/
    x86_64.Dockerfile      # Ubuntu 22.04 + CUDA + Humble (built INSIDE the image)
    jetson.Dockerfile      # Jetson L4T variant
  scripts/
    install_docker.sh      # one-time host bootstrap (Ubuntu 24.04)
    build.sh               # auto-detects arch and builds the image
    start.sh               # launches the container w/ GPU + X11 + USB
    entrypoint.sh          # sources ROS + workspace inside the container
```

Note: the container is **Humble on Ubuntu 22.04**, but the host can be Ubuntu 24.04 (or any modern Linux with Docker + NVIDIA Container Toolkit). The two distros are decoupled.

## First-time host setup (Ubuntu 24.04)

```bash
cd docker/scripts
./install_docker.sh
# log out / back in so the docker group membership takes effect
```

`install_docker.sh` installs:
- Docker Engine + Buildx + Compose plugin from Docker's official repo
- NVIDIA Container Toolkit (only if `nvidia-smi` is present), wired to the Docker runtime so `--gpus all` works

## Build the image

```bash
cd docker/scripts
./build.sh
```

The script auto-picks `x86_64.Dockerfile` or `jetson.Dockerfile` based on `uname -m`. It also forwards an SSH agent so the in-image `git clone` of `Warrior_2026` can authenticate.

## Run the container

```bash
cd docker/scripts
./start.sh
```

Inside the container the [entrypoint.sh](scripts/entrypoint.sh) sources `/opt/ros/humble/setup.bash` and the built workspace, then drops you into bash.

## Perception sub-packages

The image's colcon step builds:
- `warrior_bringup` (and its deps)
- `unitree_lidar_ros2` — upstream Unitree L2 driver (vendored under `warrior_perception/unilidar_sdk2/`)
- `unitree_l2_lidar` — thin launch+config wrapper around `unitree_lidar_ros2_node` with our preferred params
- `insta360_camera` — Python USB/UVC driver for the Insta360 X4 / X5
- `neural_engine` — TensorRT multitask net; consumes images, publishes seg mask + 2D detections
- `omnivision` — 360° camera + LiDAR fusion; yaw calibration GUIs

To launch from inside the container:

```bash
ros2 launch unitree_l2_lidar unitree_l2.launch.py
ros2 launch insta360_camera insta360.launch.py
```

`start.sh` runs with `--privileged --net host`, which gives the container access to `/dev/video*` (camera), `/dev/ttyUSB*` / `/dev/ttyACM*` (serial) on the host, and shares the host's network stack (so the container reaches the Unitree L2 LiDAR at `192.168.1.62` via whatever NIC the host has on that subnet).

### Host network requirement for the Unitree L2 LiDAR

The L2 defaults to Ethernet/UDP with a fixed IP of `192.168.1.62`. The host NIC connected to it must be on `192.168.1.0/24` — typically `192.168.1.2/24`. Configure once with NetworkManager:

```bash
sudo nmcli con modify "<your-eth-conn>" \
    connection.id unitree-l2 \
    ipv4.method manual \
    ipv4.addresses 192.168.1.2/24 \
    ipv4.gateway "" \
    ipv4.never-default yes \
    ipv6.method ignore \
    connection.autoconnect yes
sudo nmcli con up unitree-l2
```

(`ipv4.never-default yes` prevents this interface from being used as the default route, so internet still flows through WiFi / your other NIC.)
