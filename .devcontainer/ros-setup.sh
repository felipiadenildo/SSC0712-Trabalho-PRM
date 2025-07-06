#!/bin/bash
set -euo pipefail

# ======================================
# ROS 2 Development Container Setup
# ======================================

# ---- Helper Functions ----
header() {
    echo -e "\n${BLUE}== ${1} ==${NC}"
}

success() {
    echo -e "${GREEN}✓ ${1}${NC}"
}

warning() {
    echo -e "${YELLOW}⚠ ${1}${NC}"
}

fail() {
    echo -e "${RED}✗ ${1}${NC}"
    exit 1
}

# ---- GPU Acceleration Check ----
check_gpu() {
    header "GPU ACCELERATION CHECK"
    if command -v nvidia-smi &>/dev/null; then
        if nvidia-smi | grep -q 'NVIDIA-SMI'; then
            success "NVIDIA GPU detected:"
            nvidia-smi --query-gpu=name --format=csv,noheader
        else
            warning "NVIDIA driver installed but no GPU detected"
        fi
    elif lsmod | grep -q 'nouveau'; then
        warning "Open-source Nouveau driver detected (performance limited)"
    else
        warning "No NVIDIA GPU acceleration available"
    fi
}

# ---- ROS Package Validation ----
validate_ros_packages() {
    header "ROS PACKAGE VALIDATION"
    local missing=0
    
    while read -r pkg; do
        if ! ros2 pkg list | grep -q "^${pkg}$"; then
            fail "Missing required package: ${pkg}"
            ((missing++))
        fi
    done < <(rosdep keys --from-paths src 2>/dev/null | grep '^ros-')

    (( missing == 0 )) && success "All ROS dependencies satisfied"
}

# ---- Workspace Health Check ----
health_check() {
    header "WORKSPACE HEALTH CHECK"
    
    # Build check
    if [ ! -f "install/setup.bash" ]; then
        warning "Workspace not built - running initial build"
        build_workspace
    fi

    # Test file permissions
    if [ ! -w "src" ]; then
        fail "Workspace src directory not writable"
    fi

    success "Workspace health OK"
}

# ---- Build Function ----
build_workspace() {
    header "BUILDING WORKSPACE"
    colcon build \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --parallel-workers $(nproc)
}

# ---- Main Execution ----
main() {
    if [[ "$1" == "--quick-check" ]]; then
        check_gpu
        health_check
        exit 0
    fi
    check_gpu
    validate_ros_packages
    health_check
    
    header "ENVIRONMENT SETUP"
    source "/opt/ros/humble/setup.bash"
    source "install/setup.bash" 2>/dev/null || warning "Workspace not yet built"

    success "Development environment ready\n"
    echo -e "Next steps:"
    echo -e "  ${BLUE}1. Edit your code in src/prm/${NC}"
    echo -e "  ${BLUE}2. Run 'colcon build' to rebuild${NC}"
    echo -e "  ${BLUE}3. Use './run.sh' to launch simulations${NC}"
}

# Execute
main "$@"

# --------------------------
# 1. ROS Core Environment
# --------------------------
header "Setting up ROS 2 environment"
step "Sourcing core ROS installation"
source "/opt/ros/humble/setup.bash"

# --------------------------
# 2. Dependency Management
# --------------------------
header "Handling dependencies"
step "Updating rosdep"
rosdep update

step "Installing workspace dependencies"
rosdep install \
    --from-paths src \
    --ignore-src \
    -y \
    --skip-keys="gz_ros2_control"

# --------------------------
# 3. Workspace Build
# --------------------------
header "Building workspace"
step "Running colcon build"
colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --parallel-workers $(nproc)

# --------------------------
# 4. Finalization
# --------------------------
header "Finalizing environment"
step "Sourcing workspace"
source "install/setup.bash"

echo -e "\n\033[1;32m✓ Development environment ready\033[0m"