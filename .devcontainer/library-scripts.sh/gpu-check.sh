#!/bin/bash
# .devcontainer/library-scripts/gpu-check.sh

if lspci | grep -qi nvidia; then
    if command -v nvidia-smi &>/dev/null; then
        echo "NVIDIA GPU detected:"
        nvidia-smi --query-gpu=name --format=csv,noheader
        exit 0
    fi
fi

echo "No supported GPU acceleration detected"
echo "Gazebo will fall back to software rendering"
exit 1