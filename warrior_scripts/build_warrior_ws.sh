#!/bin/bash
set -e

# ==========================================
# 📋 Usage
# ==========================================
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
    -w, --workspace DIR         Specify the ROS 2 workspace directory (default: warrior_ws*)
    -h, --help                  Show this help message

Example:
    $0                                # Auto-detect workspace
    $0 -w /path/to/warrior_ws         # Specify workspace manually
    $0 --workspace ~/warrior_ws       # Using long option
EOF
    exit 0
}

# ==========================================
# 🔍 Auto-detect workspace (default: warrior_ws*)  
# ==========================================
find_workspace_root() {
    local dir="$1"
    while [[ "$dir" != "/" ]]; do
        local base=$(basename "$dir")
        if [[ $base == warrior_ws* ]]; then
            echo "$dir"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

# ==========================================
# 🎯 Parse command line arguments
# ==========================================
WORKSPACE_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--workspace)
            WORKSPACE_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "❌ Unknown option: $1"
            usage
            ;;
    esac
done

# ==========================================
# 🔍 Determine workspace directory
# Priority:
#   1) Command line argument
#   2) Environment variable WARRIOR_WORKSPACE
#   3) Auto-detection
# ==========================================

# ---- Priority 1: Command line ----
if [[ -n "$WORKSPACE_DIR" ]]; then
    echo "✅ Using workspace from command line: $WORKSPACE_DIR"

# ---- Priority 2: Environment variable ----
elif [[ -n "$WARRIOR_WORKSPACE" ]]; then
    WORKSPACE_DIR="$WARRIOR_WORKSPACE"
    echo "✅ Using workspace from environment variable: $WORKSPACE_DIR"

# ---- Priority 3: Auto-detect ----
else
    SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    WORKSPACE_DIR=$(find_workspace_root "$SCRIPT_PATH")

    if [[ -z "$WORKSPACE_DIR" ]]; then
        echo "❌ Error: Could not auto-detect workspace (warrior_ws*)"
        echo "💡 Tip:"
        echo "   - Use -w option"
        echo "   - Or export WARRIOR_WORKSPACE=/your/ws"
        exit 1
    fi

    echo "✅ Auto-detected workspace: $WORKSPACE_DIR"
fi

# ---- Final validation ----
if [[ ! -d "$WORKSPACE_DIR" ]]; then
    echo "❌ Error: Workspace directory does not exist: $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR"

# ==========================================
# ⚙️ Args for colcon build
# ==========================================
BUILD_ARGS="--symlink-install"

# ==========================================
# 🏗️ Start sequential builds
# ==========================================
echo "⚙️ Starting builds under workspace: $WORKSPACE_DIR"
echo "------------------------------------------------------------"

echo "🔹 [1/1] Building warrior_bringup..."
colcon build --packages-up-to warrior_bringup ${BUILD_ARGS}

echo "------------------------------------------------------------"
echo "✅ All builds completed successfully in: $WORKSPACE_DIR"
echo "------------------------------------------------------------"