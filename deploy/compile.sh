#!/usr/bin/env bash

cd src/liegroups
python3 setup.py install --user
touch COLCON_IGNORE

cd ../pygsp
python3 setup.py install --user
touch COLCON_IGNORE

cd ../../
source /opt/ros/humble/setup.bash
colcon build --packages-select maplab_msgs
colcon build --packages-select fgsp
