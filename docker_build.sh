#!/bin/bash
# Build ros2_yolos_cpp in a ROS 2 Humble Docker container
# Usage: ./docker_build.sh [humble|jazzy]

set -e

ROS_DISTRO="${1:-humble}"
IMAGE="ros:${ROS_DISTRO}-ros-base"
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=========================================="
echo "Building ros2_yolos_cpp in Docker"
echo "ROS Distro: ${ROS_DISTRO}"
echo "Workspace: ${WORKSPACE_DIR}"
echo "=========================================="

# Pull the image if needed
docker pull ${IMAGE}

# Run build in container
docker run --rm -it \
    -v "${WORKSPACE_DIR}:/ros2_ws/src/ros2_yolos_cpp:rw" \
    -w /ros2_ws \
    ${IMAGE} \
    /bin/bash -c "
        set -e
        echo '>>> Installing dependencies...'
        apt-get update -qq
        apt-get install -y -qq \
            build-essential \
            cmake \
            libopencv-dev \
            curl \
            ros-${ROS_DISTRO}-image-transport \
            ros-${ROS_DISTRO}-vision-msgs \
            ros-${ROS_DISTRO}-cv-bridge \
            ros-${ROS_DISTRO}-rclcpp-components \
            ros-${ROS_DISTRO}-lifecycle > /dev/null

        echo '>>> Sourcing ROS 2...'
        source /opt/ros/${ROS_DISTRO}/setup.bash

        echo '>>> Building with colcon...'
        colcon build --packages-select ros2_yolos_cpp \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers console_direct+

        echo '>>> Build complete!'
    "
