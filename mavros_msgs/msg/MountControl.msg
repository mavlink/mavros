# MAVLink message: DO_MOUNT_CONTROL
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL

std_msgs/Header header

uint8 mode # See enum MAV_MOUNT_MODE.
uint8 MAV_MOUNT_MODE_RETRACT = 0
uint8 MAV_MOUNT_MODE_NEUTRAL = 1
uint8 MAV_MOUNT_MODE_MAVLINK_TARGETING = 2
uint8 MAV_MOUNT_MODE_RC_TARGETING = 3
uint8 MAV_MOUNT_MODE_GPS_POINT = 4

float32 pitch # pitch degrees or degrees/second depending on mount mode.
float32 roll # roll degrees or degrees/second depending on mount mode.
float32 yaw # yaw degrees or degrees/second depending on mount mode.
float32 altitude  # altitude depending on mount mode.
float32 latitude # latitude in degrees * 1E7, set if appropriate mount mode.
float32 longitude # longitude in degrees * 1E7, set if appropriate mount mode.