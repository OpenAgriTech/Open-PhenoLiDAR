#!/bin/bash
#set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source "/workspace/devel/setup.bash"

exec "$@"
