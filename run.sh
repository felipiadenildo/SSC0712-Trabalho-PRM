#!/bin/bash

# =============================================
# ROS 2 Project Control Script
# Version: 2.0
# Author: felipiadenildo
# Project: SSC0712-Trabalho-PRM
# =============================================

# --- Configuration ---
ROS2_WORKSPACE="/ros2_ws"
GAZEBO_WORLD="empty_arena.sdf"
LAUNCH_PACKAGE="prm"
TERMINAL="xterm" #gnome-terminal" # Change to 'xterm' or 'konsole' if needed

# Terminal titles and colors
SIM_TITLE="ROS2 - SIMULATION [Gazebo]"
SIM_COLOR="orange"
ROBOT_TITLE="ROS2 - ROBOT [Control]"
ROBOT_COLOR="blue"
TEST_TITLE="ROS2 - COMMAND [Manual]"
TEST_COLOR="green"

# --- Functions ---

cleanup() {
    echo -e "\n=== CLEANUP PROCESS ==="
    echo "Terminating all simulation processes..."
    
    # Kill terminals by title
    pkill -f "$SIM_TITLE" && echo "[✓] Simulation terminal closed"
    pkill -f "$ROBOT_TITLE" && echo "[✓] Robot terminal closed"
    pkill -f "$TEST_TITLE" && echo "[✓] Command terminal closed"
    
    # Kill ROS2 and Gazebo processes
    killall -q gzserver gzclient rviz2 ros2 launch
    echo "Cleanup complete. All systems shutdown."
}

show_help() {
    echo -e "\n=== PROJECT CONTROL SCRIPT ==="
    echo "Usage: ./run.sh [options]"
    echo -e "\nLaunch Options (can be combined):"
    echo "  i     Launch simulation environment"
    echo "  r     Launch robot control and RViz"
    echo "  t     Open command terminal for manual control"
    echo -e "\nExamples:"
    echo "  ./run.sh i     (Simulation only)"
    echo "  ./run.sh ir    (Simulation + Robot)"
    echo "  ./run.sh irt   (Full system)"
    echo -e "\nManagement Options:"
    echo "  stop   Terminate all processes"
    echo "  help   Show this help message"
    echo -e "\nDebug Options:"
    echo "  build  Rebuild workspace only"
}

launch_terminal() {
    local title=$1
    local color=$2
    local command=$3
    
    $TERMINAL --title="$title" \
              --window-with-profile="$color" \
              -- bash -ic "$command; exec bash"
}

# --- Main Execution ---

# Handle stop command
if [[ "$1" == "stop" ]]; then
    cleanup
    exit 0
fi

# Handle help command
if [[ -z "$1" || "$1" == "help" ]]; then
    show_help
    exit 0
fi

# Handle build command
if [[ "$1" == "build" ]]; then
    echo "=== BUILDING WORKSPACE ==="
    cd "$ROS2_WORKSPACE" && colcon build
    exit $?
fi

# --- Build and Setup ---
echo -e "\n=== INITIALIZING WORKSPACE ==="
cd "$ROS2_WORKSPACE" || { echo "ERROR: Workspace not found"; exit 1; }

echo "Building workspace..."
if ! colcon build; then
    echo "ERROR: Build failed"
    exit 1
fi

echo "Sourcing environment..."
source install/setup.bash || { echo "ERROR: Setup failed"; exit 1; }

# --- Launch Terminals ---
echo -e "\n=== LAUNCHING SYSTEMS ==="

# Simulation Terminal
if [[ "$1" == *i* ]]; then
    launch_terminal "$SIM_TITLE" "$SIM_COLOR" \
        "ros2 launch $LAUNCH_PACKAGE inicia_simulacao.launch.py world:=$GAZEBO_WORLD"
    echo "[✓] Simulation launched"
fi

# Robot Terminal
if [[ "$1" == *r* ]]; then
    launch_terminal "$ROBOT_TITLE" "$ROBOT_COLOR" \
        "ros2 launch $LAUNCH_PACKAGE carrega_robo.launch.py"
    echo "[✓] Robot control launched"
fi

# Command Terminal
if [[ "$1" == *t* ]]; then
    launch_terminal "$TEST_TITLE" "$TEST_COLOR" \
        "echo 'Use this terminal for manual commands'; \
         echo 'ROS2 workspace ready'; \
         cd $ROS2_WORKSPACE"
    echo "[✓] Command terminal ready"
fi

echo -e "\n=== SYSTEM READY ==="
echo "Use './run.sh stop' to terminate all processes"
exit 0