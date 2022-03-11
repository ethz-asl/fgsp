# Name of the robot, will be used to determine which mission to use for the lookup
string robot_name

# Sensor type/frame (S) of the requested sensor point (NCAMERA, LIDAR or IMU)
string sensor_type

# Timestamp of desired robot pose corresponding to the requested points
time timestamp

# Point in sensor frame (S)
geometry_msgs/Point p_S
