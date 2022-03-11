# Name of robot
string robot_name

# LC type (CAMERA, LIDAR)
string sensor_type

# LC expressed in frame
string frame

# Transformation between source b to target a
geometry_msgs/PoseWithCovariance T_a_b

# Timestamp of target a
time timestamp_a

# Timestamp of source b
time timestamp_b
