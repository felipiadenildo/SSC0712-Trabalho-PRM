#!/bin/bash
set -e

# Health checks first
if ! nvidia-smi &>/dev/null; then
    echo "WARNING: GPU acceleration not detected" >&2
fi

# Then source environments
[ -f "/opt/ros/humble/setup.bash" ] && source "/opt/ros/humble/setup.bash"
[ -f "/ros2_ws/install/setup.bash" ] && source "/ros2_ws/install/setup.bash"

exec "$@"