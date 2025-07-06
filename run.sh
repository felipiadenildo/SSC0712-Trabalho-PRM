#!/bin/bash

# =============================================
# ROS 2 Project Control Script - Enhanced
# Version: 3.0
# =============================================

# --- Configuration ---
ROS2_WORKSPACE="/ros2_ws"
GAZEBO_WORLD="empty_arena.sdf"
LAUNCH_PACKAGE="prm"
TERMINAL_CMD="gnome-terminal -- /bin/bash -ic"  # Default for Ubuntu
TERMINAL_EMULATORS=("gnome-terminal" "konsole" "xterm")

# Detect available terminal
for term in "${TERMINAL_EMULATORS[@]}"; do
    if command -v "${term}" >/dev/null 2>&1; then
        case "${term}" in
            "gnome-terminal") TERMINAL_CMD="gnome-terminal -- /bin/bash -ic" ;;
            "konsole") TERMINAL_CMD="konsole -e /bin/bash -ic" ;;
            "xterm") TERMINAL_CMD="xterm -e /bin/bash -ic" ;;
        esac
        break
    fi
done

# Terminal configurations
declare -A TERMINAL_CONFIG=(
    ["SIM"]="ROS2 - SIMULATION [Gazebo]|orange|Gazebo"
    ["ROBOT"]="ROS2 - ROBOT [Control]|blue|RViz"
    ["CMD"]="ROS2 - COMMAND [Manual]|green|Terminal"
)

# --- Functions ---
cleanup() {
    echo -e "\n=== CLEANUP PROCESS ==="
    
    # Kill terminals and processes
    for term_type in "${!TERMINAL_CONFIG[@]}"; do
        IFS='|' read -r title color _ <<< "${TERMINAL_CONFIG[$term_type]}"
        pkill -f "${title}" && echo "[✓] ${title} closed"
    done
    
    killall -q gzserver gzclient rviz2 ros2 launch
    
    # System resource report
    echo -e "\n=== SYSTEM RESOURCES ==="
    top -bn1 | head -5
    [[ -x "$(command -v nvidia-smi)" ]] && nvidia-smi --query-gpu=utilization.gpu --format=csv
    
    echo "Cleanup complete."
}

show_help() {
    cat <<EOF

PROJECT CONTROL SCRIPT
Usage: ./run.sh [options]

Launch Options (combinable):
  i    Launch simulation environment
  r    Launch robot control and RViz
  t    Open command terminal

Examples:
  ./run.sh i     # Simulation only
  ./run.sh ir    # Simulation + Robot
  ./run.sh irt   # Full system

Management Options:
  stop   Terminate all processes
  help   Show this help
  build  Rebuild workspace

EOF
}

launch_terminal() {
    local term_type=$1
    IFS='|' read -r title color icon <<< "${TERMINAL_CONFIG[$term_type]}"
    local cmd=$2
    
    ${TERMINAL_CMD} "echo -e '\033]0;${title}\007'; ${cmd}; exec bash" &
    echo "[✓] ${icon} launched (${title})"
}

validate_environment() {
    if [ ! -d "${ROS2_WORKSPACE}/src" ]; then
        echo "ERROR: Workspace not found at ${ROS2_WORKSPACE}" >&2
        exit 1
    fi
    
    if ! source "${ROS2_WORKSPACE}/install/setup.bash" 2>/dev/null; then
        echo "Workspace not built - compiling..."
        if ! (cd "${ROS2_WORKSPACE}" && colcon build); then
            echo "ERROR: Build failed" >&2
            exit 1
        fi
        source "${ROS2_WORKSPACE}/install/setup.bash"
    fi
}

# --- Argument Handling ---
case "${1:-}" in
    stop)
        cleanup
        exit 0
        ;;
    help|"")
        show_help
        exit 0
        ;;
    build)
        echo "=== BUILDING WORKSPACE ==="
        cd "${ROS2_WORKSPACE}" && colcon build
        exit $?
        ;;
    *)
        # Continue to main execution
        ;;
esac

# --- Main Execution ---
validate_environment

# Terminal Launch Logic
[[ "$1" == *i* ]] && launch_terminal "SIM" \
    "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py world:=${GAZEBO_WORLD}"

[[ "$1" == *r* ]] && launch_terminal "ROBOT" \
    "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py"

[[ "$1" == *t* ]] && launch_terminal "CMD" \
    "echo 'Manual Command Terminal'; cd ${ROS2_WORKSPACE}; echo 'Workspace ready'"

echo -e "\n=== SYSTEM READY ==="
echo "Use './run.sh stop' to terminate all processes"
exit 0