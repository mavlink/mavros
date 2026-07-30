# MAVLink TERRAIN_DATA
# https://mavlink.io/en/messages/common.html#TERRAIN_DATA

float64 latitude    # WGS-84 degrees
float64 longitude   # WGS-84 degrees
uint16 grid_spacing # Grid spacing in meters
uint8 gridbit       # Index of the 4x4 block inside the grid
int16[16] data      # Terrain elevation values in meters
