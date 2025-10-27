#!/bin/bash
# Build script for deployment (Raspberry Pi) - excludes desktop packages

set -e

echo "Building EvaRobot for deployment (minimal build)..."
echo "Skipping desktop packages: mapping, viz, planning, motion, perception, examples"

cd "$(dirname "$0")/../.."

colcon build --symlink-install \
    --packages-skip \
        evarobot_mapping \
        evarobot_viz \
        evarobot_planning \
        evarobot_motion \
        evarobot_perception \
        evarobot_cpp_examples \
        evarobot_py_examples

echo ""
echo "Build complete!"
echo "Source the workspace with: source install/setup.bash"
