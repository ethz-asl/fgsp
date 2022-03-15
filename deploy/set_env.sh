#!/usr/bin/env bash

. /usr/home/ws/devel/setup.bash
host_ip=$(ip route | awk '/default/ {print $3}')
export ROS_MASTER_URI=http://${host_ip}:11311
export ROS_HOSTNAME=localhost
exec "$@"
