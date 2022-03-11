# Status of the map lookup
int8 status
# Status options are:
int8 StatusUndefined = -1     # Undefined
int8 StatusSuccess = 0        # Success
int8 StatusNoSuchMission = 1  # No such robot name / mission
int8 StatusNoSuchSensor = 2   # No such sensor
# Timestamp outside range of mission (Upper bound, lookup might work later)
int8 StatusPoseNotYetAvailable = 3
# Timestamp outside range of mission (Lower bound, Lookup will never work)
int8 StatusPoseNeverAvailable = 4

# Points in global map frame (G)
geometry_msgs/Point p_G

# Sensor origin in global map frame (G)
geometry_msgs/Point sensor_p_G
