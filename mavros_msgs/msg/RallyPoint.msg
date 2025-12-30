# Rally Point message based on
# https://mavlink.io/en/messages/ardupilotmega.html#RALLY_POINT

# Flags
uint8 FAVORABLE_WIND = 1
uint8 LAND_IMMEDIATELY = 2

std_msgs/Header header

uint8 idx
uint8 count
float32 latitude
float32 longitude
int16 altitude
int16 break_altitude
uint16 land_direction
uint8 flags