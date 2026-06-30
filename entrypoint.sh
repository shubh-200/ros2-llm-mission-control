#!/bin/bash
set -e

# Source the ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source the custom workspace
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute the command passed to the docker container
exec "$@"