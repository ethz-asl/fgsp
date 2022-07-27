#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
colcon build --packages-select maplab_msgs
colcon build --packages-select fgsp
