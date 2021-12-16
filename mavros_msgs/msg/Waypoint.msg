# Waypoint.msg
#
# ROS representation of MAVLink MISSION_ITEM
# See mavlink documentation



# see enum MAV_FRAME
uint8 frame
uint8 FRAME_GLOBAL = 0
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_MISSION = 2
uint8 FRAME_GLOBAL_REL_ALT = 3
uint8 FRAME_LOCAL_ENU = 4
uint8 FRAME_GLOBAL_INT = 5
uint8 FRAME_GLOBAL_RELATIVE_ALT_INT = 6
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9
uint8 FRAME_GLOBAL_TERRAIN_ALT = 10
uint8 FRAME_GLOBAL_TERRAIN_ALT_INT = 11
uint8 FRAME_BODY_FRD = 12
uint8 FRAME_RESERVED_13 = 13
uint8 FRAME_RESERVED_14 = 14
uint8 FRAME_RESERVED_15 = 15
uint8 FRAME_RESERVED_16 = 16
uint8 FRAME_RESERVED_17 = 17
uint8 FRAME_RESERVED_18 = 18
uint8 FRAME_RESERVED_19 = 19
uint8 FRAME_LOCAL_FRD = 20
uint8 FRAME_LOCAL_FLU = 21

# see enum MAV_CMD and CommandCode.msg
uint16 command

bool is_current
bool autocontinue
# meaning of this params described in enum MAV_CMD
float32 param1
float32 param2
float32 param3
float32 param4
float64 x_lat
float64 y_long
float64 z_alt
