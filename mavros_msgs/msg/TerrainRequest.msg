# MAVLink TERRAIN_REQUEST
# https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST
#
# NOTE: lat/lon use int32 degE7 to match the MAVLink wire format exactly.
# The terrain server must echo these values back in TERRAIN_DATA without
# modification; converting through float64 risks rounding mismatches.

std_msgs/Header header

int32 lat           # Latitude degE7
int32 lon           # Longitude degE7
uint16 grid_spacing # Grid spacing in meters
uint64 mask         # Bitmask of requested 4x4 terrain blocks
