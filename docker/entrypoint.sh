#!/bin/bash
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
exec "$@"
