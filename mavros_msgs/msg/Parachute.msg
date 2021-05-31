# Patachute msg

std_msgs/Header header

# Chute status
bool enabled
bool released

# Chute release reasons
bool SINK_RATE
bool ACCEL_FALLING
bool CONTROL_LOSS
bool MISSION_ITEM
bool MANUAL
float32 time_to_release  # in ms

# STANDBY enabled / disabled
bool standby