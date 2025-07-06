#!/bin/bash
# .devcontainer/library-scripts/ros-deps.sh
# ROS 2 Dependency Management Script
# Version: 1.2
# Author: felipiadenildo

set -euo pipefail

# ---- Color Definitions ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---- Configuration ----
ROS_DISTRO="humble"
WORKSPACE="/ros2_ws"
LOCK_FILE="${WORKSPACE}/.rosdeps.lock"
CACHE_DIR="/var/cache/rosdeps"
SKIP_KEYS="gz_ros2_control"

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

# ---- Dependency Installation ----
install_dependencies() {
    header "ROS DEPENDENCY RESOLUTION"
    
    # Check for package.xml files
    if ! find "${WORKSPACE}/src" -name package.xml | grep -q .; then
        warning "No ROS packages found in src/"
        return 0
    fi

    # Create cache directory
    sudo mkdir -p "${CACHE_DIR}"
    sudo chown -R $(whoami) "${CACHE_DIR}"

    # Update rosdep if cache is stale (1 day)
    if [ ! -f "${LOCK_FILE}" ] || \
       [ $(find "${LOCK_FILE}" -mtime +1 2>/dev/null) ]; then
        header "UPDATING ROSDEP"
        rosdep update || warning "rosdep update failed (network issue?)"
        touch "${LOCK_FILE}"
    fi

    # Install dependencies
    header "INSTALLING DEPENDENCIES"
    rosdep install \
        --from-paths "${WORKSPACE}/src" \
        --ignore-src \
        -y \
        --skip-keys="${SKIP_KEYS}" \
        --rosdistro="${ROS_DISTRO}" \
        --simulate 2>&1 | tee "${CACHE_DIR}/rosdeps.log"

    # Check for errors in simulation
    if grep -q "ERROR" "${CACHE_DIR}/rosdeps.log"; then
        fail "Dependency resolution failed"
    fi

    # Actual installation
    if ! rosdep install \
        --from-paths "${WORKSPACE}/src" \
        --ignore-src \
        -y \
        --skip-keys="${SKIP_KEYS}" \
        --rosdistro="${ROS_DISTRO}"; then
        warning "Some dependencies failed to install"
        return 1
    fi

    success "All dependencies installed"
    return 0
}

# ---- System Dependency Check ----
check_system_deps() {
    header "SYSTEM REQUIREMENTS CHECK"
    local missing=0
    
    declare -A REQUIRED_TOOLS=(
        ["rosdep"]="ros-humble-rosdep"
        ["colcon"]="python3-colcon-common-extensions"
        ["pip"]="python3-pip"
    )

    for cmd in "${!REQUIRED_TOOLS[@]}"; do
        if ! command -v "${cmd}" &>/dev/null; then
            warning "Missing required tool: ${cmd}"
            echo "Install with: sudo apt install ${REQUIRED_TOOLS[$cmd]}"
            ((missing++))
        fi
    done

    (( missing > 0 )) && fail "Missing essential tools"
    success "All system dependencies met"
}

# ---- Main Execution ----
main() {
    # Verify environment
    if [ ! -d "${WORKSPACE}/src" ]; then
        fail "Workspace directory not found at ${WORKSPACE}"
    fi

    check_system_deps
    install_dependencies

    # Post-install verification
    if [ -f "${WORKSPACE}/src/prm/package.xml" ]; then
        header "PACKAGE VALIDATION"
        if ! rosdep check --from-paths "${WORKSPACE}/src/prm"; then
            warning "Package validation reported issues"
        else
            success "All package dependencies satisfied"
        fi
    fi
}

# Execute
main "$@"