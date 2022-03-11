# Name of Robot
std_msgs/String robot_name
# 3D position calculated in world frame
geometry_msgs/PointStamped point
# The detection image from darknet
sensor_msgs/Image image
# The compressed detection image from darknet
sensor_msgs/CompressedImage compressed_image
# Filtered Label of Artifact
std_msgs/String class_label
# Filtered Probability of Artifact
float32 probability
# Transformation from Velodyne Frame to 3D Position of Artifact in world frame (ASL)
geometry_msgs/TransformStamped transform
