# Header
std_msgs/Header header

# Name of the robot
string[] robot_name_from
string[] robot_name_to

# Submap IDs
int64[] id_from
int64[] id_to

# Timestamps of the nodes in which the LC is expressed
time[] timestamp_from
time[] timestamp_to

# Delta pose between the submaps
geometry_msgs/PoseWithCovariance[] T_a_b
