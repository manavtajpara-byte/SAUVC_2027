#!/bin/bash
# ==============================================================================
# SAUVC 2027 All-In-One Unified Simulation Runner
# Supports:
#   - Option 1 (Default): Gazebo Physics Simulator (`./run_all.sh gazebo`)
#   - Option 2: RViz 3D Visualizer (`./run_all.sh rviz`)
# ==============================================================================

# Source ROS 2 Humble and Workspace
source /opt/ros/humble/setup.bash
source /home/manav/Desktop/SAUVC/sauvc_ws/install/setup.bash

# Clean up any stale processes before starting
pkill -9 -f sauvc 2>/dev/null
pkill -9 -f rviz 2>/dev/null
pkill -9 -f rqt_image_view 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
sleep 0.5

MODE=${1:-gazebo}

echo "======================================================================"
echo "    LAUNCHING SAUVC ALL-IN-ONE SIMULATION & DASHBOARD ENVIRONMENT"
echo "======================================================================"
echo "  - Mode: $MODE"
echo "  - Arena: SAUVC Finals Competition Pool (25m x 16m x 2m)"
echo "  - Speed: 50% reduced for smooth realistic kinematics"
echo "  - Cameras: Front + Downward Underwater Vision Feeds"
echo "  - Controls: Start, Stop/Pause, Reset & Manual Steering"
echo "======================================================================"

if [ "$MODE" = "rviz" ]; then
    ros2 launch sauvc_arena_sim sauvc_arena.launch.py
else
    ros2 launch sauvc_arena_sim sauvc_gazebo.launch.py
fi
