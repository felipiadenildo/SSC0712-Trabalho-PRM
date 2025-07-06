#!/bin/bash
# .devcontainer/ros-setup.sh
# ROS 2 Development Environment Setup
# Version: 2.1

set -euo pipefail

# ---- Configuration ----
ROS_DISTRO="humble"
WORKSPACE="/ros2_ws"
LOG_FILE="${WORKSPACE}/setup.log"
CACHE_DIR="/var/cache/ros"
GPU_CHECK=true

# ---- Color Definitions ----
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'
BLUE='\033[0;34m'; NC='\033[0m'

# ---- Helper Functions ----
header() { echo -e "\n${BLUE}== ${1} ==${NC}"; }
success() { echo -e "${GREEN}✓ ${1}${NC}"; }
warning() { echo -e "${YELLOW}⚠ ${1}${NC}"; }
fail() { echo -e "${RED}✗ ${1}${NC}"; exit 1; }

# ---- System Checks ----
check_gpu() {
    header "GPU ACCELERATION CHECK"
    if [[ "${GPU_CHECK}" != "true" ]]; then
        warning "GPU check disabled by config"
        return
    fi

    if lspci | grep -qi "nvidia"; then
        if command -v nvidia-smi &>/dev/null; then
            success "NVIDIA GPU detected:"
            nvidia-smi --query-gpu=name --format=csv,noheader
            return 0
        else
            warning "NVIDIA GPU found but drivers not installed"
        fi
    fi
    
    warning "Using software rendering (Gazebo will be slower)"
    export LIBGL_ALWAYS_SOFTWARE=1
    return 1
}

# ---- FastDDS Configuration ----
configure_fastdds() {
    header "CONFIGURING FASTDDS"
    local dds_conf="${WORKSPACE}/src/prm/.devcontainer/fastdds.xml"
    
    if [[ -f "${dds_conf}" ]]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="${dds_conf}"
        success "Using custom FastDDS config"
    else
        warning "No custom FastDDS config found, using defaults"
    fi
}

# ---- Workspace Setup ----
setup_workspace() {
    header "WORKSPACE SETUP"
    
    # Create cache directory
    sudo mkdir -p "${CACHE_DIR}"
    sudo chown -R ros:ros "${CACHE_DIR}"

    # Source ROS environment
    source "/opt/ros/${ROS_DISTRO}/setup.bash"

    # Install dependencies
    if ! rosdep install --from-paths src --ignore-src -y --skip-keys="gz_ros2_control"; then
        warning "Some dependencies failed to install"
    fi

    # Build workspace
    colcon build \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --parallel-workers $(nproc)
}

# ---- Quick Check Mode ----
quick_check() {
    header "QUICK SYSTEM CHECK"
    check_gpu
    configure_fastdds
    
    echo -e "\n${BLUE}=== ENVIRONMENT STATUS ===${NC}"
    ros2 doctor || warning "ROS 2 doctor reported issues"
    
    success "Environment ready"
}

# ---- Full Setup Mode ----
full_setup() {
    check_gpu
    configure_fastdds
    setup_workspace
    
    header "SETUP COMPLETED"
    echo -e "${GREEN}✓ Workspace built successfully${NC}"
    echo -e "Run ${BLUE}./run.sh${NC} to start simulation"
}

# ---- Main Execution ----
case "${1:-}" in
    quick-check)
        quick_check
        ;;
    full)
        time full_setup | tee "${LOG_FILE}"
        ;;
    *)
        echo "Usage: ${0} [full|quick-check]"
        exit 1
        ;;
esac