# Message for TERRAIN_REPORT
# https://mavlink.io/en/messages/common.html#TERRAIN_REPORT

std_msgs/Header header

float64 latitude
float64 longitude
uint16 spacing
float32 terrain_height # in meters, terrain height
float32 current_height # in meters, vehicle height above terrain
uint16 pending
uint16 loaded 
