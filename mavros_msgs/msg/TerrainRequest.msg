# MAVLink TERRAIN_REQUEST
# https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST

std_msgs/Header header

int32 lat           # Latitude degE7
int32 lon           # Longitude degE7
uint16 grid_spacing # Grid spacing in meters
uint64 mask         # Bitmask of requested 4x4 terrain blocks
