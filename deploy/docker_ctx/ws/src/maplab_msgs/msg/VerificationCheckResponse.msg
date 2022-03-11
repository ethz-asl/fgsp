# Defines the states of the submaps
int8[] status

# Fixed states
int8 StatusFailure = -1       # Failure
int8 StatusSuccess = 0        # Success
int8 StatusNoSuchMission = 1  # No such robot name
int8 StatusAlreadyExists = 2  # Request already exists
