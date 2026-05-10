#!/bin/bash
# ============================================================================
# Warrior 2026 Layered Dependency Installer
#
# Target stack: ROS 2 Humble + Gazebo Fortress (a.k.a. Ignition Fortress)
# Target OS:    Ubuntu 22.04 (Jammy)
#
# This installer is split into sections that mirror the bring-up gates
# documented in warrior_scripts/README.md. Each section installs only the
# packages needed for one layer of the stack so that we can verify each
# layer independently before moving on.
#
# Run with no args (or `all`) to run every IMPLEMENTED section in order.
# Run with one or more section names to run only those sections.
#
# Usage:
#   sudo bash installWarriorDependencies.sh             # full install
#   sudo bash installWarriorDependencies.sh core        # ROS+Gz base only
#   sudo bash installWarriorDependencies.sh -h          # help
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USERNAME="${SUDO_USER:-$USER}"
WARRIOR_WS="${WARRIOR_WS:-/home/$USERNAME/ros2_ws}"

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: sudo bash $(basename "$0") [SECTION...]

Layered dependency installer for the Warrior 2026 ROS 2 stack.
Each section corresponds to a bring-up gate (see warrior_scripts/README.md).

Sections (in install order):
  core           ROS 2 Humble + Gazebo Fortress base (description & sim min.)
  control        [pending] ros2_control + gz_ros2_control + Eigen
  teleop         [pending] joy / teleop_twist_joy
  localization   [pending] robot_localization (EKF)
  navigation     [pending] Nav2 stack + tf2 tools
  gps            [pending] AprilTag + GPS bridge deps
  hardware       [pending] Real-robot serial + hardware deps
  workspace      [pending] colcon build the full ros2_ws

Special:
  all            Run every implemented section in order (default if no args)
  -h, --help     Show this help

Examples:
  sudo bash $(basename "$0")               # full install (== all)
  sudo bash $(basename "$0") core          # just ROS + Gz Fortress base
  sudo bash $(basename "$0") core control  # core then control (when ready)
EOF
}

log() { echo ">>> [$1] $2"; }

require_root() {
    if [[ "$EUID" -ne 0 ]]; then
        echo "ERROR: This script must be run with sudo" >&2
        echo "Usage: sudo bash $(basename "$0")" >&2
        exit 1
    fi
}

check_ubuntu_22_04() {
    if [[ ! -f /etc/os-release ]]; then
        echo "ERROR: cannot detect OS (no /etc/os-release)" >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${VERSION_ID:-}" != "22.04" ]]; then
        echo "ERROR: This installer targets Ubuntu 22.04 (Jammy)." >&2
        echo "       Detected: ${PRETTY_NAME:-unknown}" >&2
        echo "       ROS 2 Humble is not supported on other releases." >&2
        exit 1
    fi
}

# ============================================================================
# SECTION: core
# ----------------------------------------------------------------------------
# Installs ROS 2 Humble + Gazebo Fortress (Ignition) and the minimum packages
# needed to load the Warrior URDF, render TF in RViz, and spawn the robot
# into an empty Gazebo world.
#
# Validates bring-up gates: G1 (URDF/TF in RViz) and G2 (empty Gz world).
# Does NOT install: ros2_control, controllers, Nav2, EKF, joy, hardware deps.
# ============================================================================
section_core() {
    log core "Locale"
    apt update
    apt install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    log core "ROS 2 apt repository"
    apt install -y software-properties-common curl git gnupg lsb-release ca-certificates
    add-apt-repository -y universe
    install -d -m 0755 /usr/share/keyrings
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        > /etc/apt/sources.list.d/ros2.list

    log core "System update"
    apt update
    apt upgrade -y

    log core "ROS 2 Humble desktop + dev tools"
    apt install -y \
        ros-humble-desktop \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        build-essential \
        cmake

    log core "Robot description toolchain"
    apt install -y \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rviz2

    log core "Gazebo Fortress (Ignition) + ros_gz bridge"
    # ros-humble-ros-gz-* on Humble pairs with Gazebo Fortress (Tier 1).
    # The Fortress simulator itself comes in as a transitive dep
    # (ignition-fortress) from the ROS apt repo - no extra repo needed.
    apt install -y \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-image \
        ros-humble-ros-gz-interfaces

    log core "rosdep init / update"
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        rosdep init
    fi
    sudo -u "$USERNAME" rosdep update

    log core ".bashrc source lines + user extras"
    BASHRC="/home/$USERNAME/.bashrc"
    grep -qxF 'source /opt/ros/humble/setup.bash' "$BASHRC" \
        || echo 'source /opt/ros/humble/setup.bash' >> "$BASHRC"
    if [[ -f "$WARRIOR_WS/install/setup.bash" ]]; then
        grep -qxF "source $WARRIOR_WS/install/setup.bash" "$BASHRC" \
            || echo "source $WARRIOR_WS/install/setup.bash" >> "$BASHRC"
    fi

    # The block below was captured from the pre-wipe ~/.bashrc so the fresh
    # install reproduces the user's working environment. Edit here, not in
    # ~/.bashrc directly - the next install run will refresh this block.
    # Idempotency: bounded by the BEGIN/END markers; a re-run replaces the
    # block in place rather than appending.
    if grep -qxF '# >>> warrior-bashrc-extras >>>' "$BASHRC"; then
        # Strip existing block before re-appending the current version.
        sed -i '/^# >>> warrior-bashrc-extras >>>$/,/^# <<< warrior-bashrc-extras <<<$/d' "$BASHRC"
    fi
    cat >> "$BASHRC" <<'EOF'
# >>> warrior-bashrc-extras >>>
# Managed by warrior_scripts/installWarriorDependencies.sh - do not edit by hand.

# Convenience aliases
alias sorce='source ~/.bashrc'
alias ign_gazebo='ign gazebo'
alias ros_rebuild='cd ~/ros2_ws && colcon build && source install/setup.bash'

# ROS 2 MCP virtualenv activator (venv at ~/ros2_mcp_env must exist separately)
alias mcp_activate='source /opt/ros/humble/setup.bash && source ~/ros2_mcp_env/bin/activate'

# Unitree Go2 sim launch shortcut (requires the unitree_go2_sim package)
alias go2_sim='ros2 launch unitree_go2_sim unitree_go2_launch.py'

# CUDA/cuDNN libraries for faster-whisper GPU acceleration
# Path uses python3.10 (Ubuntu 22.04 default); update if Python version changes.
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/cudnn/lib:$LD_LIBRARY_PATH"

# NOTE: pre-wipe .bashrc also sourced /opt/ros/jazzy/setup.bash. Dropped here
# because the fresh install is Humble-only - re-add manually if you install
# Jazzy alongside.
# <<< warrior-bashrc-extras <<<
EOF
    chown "$USERNAME:$USERNAME" "$BASHRC"

    log core "Done. Verify with: source ~/.bashrc && ros2 --help && ign gazebo --version"
}

# ============================================================================
# Pending sections - to be implemented as each bring-up layer is validated.
# ============================================================================
section_pending() {
    echo "[$1] section is not yet implemented."
    echo "     See warrior_scripts/README.md for the planned scope and the"
    echo "     bring-up gate that this section unlocks."
}

# ----------------------------------------------------------------------------
# Dispatch
# ----------------------------------------------------------------------------
run_section() {
    case "$1" in
        core)         section_core ;;
        control|teleop|localization|navigation|gps|hardware|workspace)
                      section_pending "$1" ;;
        *)
            echo "ERROR: Unknown section: $1" >&2
            usage
            exit 1
            ;;
    esac
}

main() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        exit 0
    fi

    require_root
    check_ubuntu_22_04

    if [[ "$#" -eq 0 || "${1:-}" == "all" ]]; then
        # NOTE: Add new sections to this list as they are implemented.
        # Order matters - later sections may depend on earlier ones.
        run_section core
        # run_section control        # pending
        # run_section teleop         # pending
        # run_section localization   # pending
        # run_section navigation     # pending
        # run_section gps            # pending
        # run_section hardware       # pending
        # run_section workspace      # pending
    else
        for section in "$@"; do
            run_section "$section"
        done
    fi

    echo ""
    echo "============================================="
    echo " Warrior installer finished."
    echo " Run: source ~/.bashrc"
    echo "============================================="
}

main "$@"
