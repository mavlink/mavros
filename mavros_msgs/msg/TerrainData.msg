# MAVLink TERRAIN_DATA
# https://mavlink.io/en/messages/common.html#TERRAIN_DATA

int32 lat           # Latitude degE7
int32 lon           # Longitude degE7
uint16 grid_spacing # Grid spacing in meters
uint8 gridbit       # Index of the 4x4 block inside the grid
int16[16] data      # Terrain elevation values in meters
