# MAVLink TERRAIN_REQUEST
# https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST

std_msgs/Header header

float64 latitude    # WGS-84 degrees
float64 longitude   # WGS-84 degrees
uint16 grid_spacing # Grid spacing in meters
uint64 mask         # Bitmask of requested 4x4 terrain blocks
