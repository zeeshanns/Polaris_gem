#!/bin/bash
# ros_entrypoint.sh - sources ROS and workspace then exec passed cmd
set -e

# Source ROS
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

# Source workspace if built
if [ -f /workspace/catkin_ws/devel/setup.bash ]; then
  source /workspace/catkin_ws/devel/setup.bash
fi

# If user passes commands, run them. Otherwise launch an interactive shell.
if [ $# -gt 0 ]; then
  exec "$@"
else
  exec /bin/bash
fi
