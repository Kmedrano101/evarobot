#!/bin/bash
# Build script for desktop (full build with all packages)

set -e

echo "Building EvaRobot for desktop (full build)..."
echo "Including all packages: deployment + desktop"

cd "$(dirname "$0")/../.."

colcon build --symlink-install

echo ""
echo "Build complete!"
echo "Source the workspace with: source install/setup.bash"
