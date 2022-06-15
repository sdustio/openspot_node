#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup openspot environment
source "/openspot/install/setup.sh"

exec "$@"
