# MAVLink message: GIMBAL_MANAGER_INFORMATION
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION

std_msgs/Header header

uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_MANAGER_CAP_FLAGS
#GIMBAL_MANAGER_CAP_FLAGS
uint32 CAP_FLAGS_HAS_RETRACT = 1 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
uint32 CAP_FLAGS_HAS_NEUTRAL = 2 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
uint32 CAP_FLAGS_HAS_ROLL_AXIS = 4 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
uint32 CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
uint32 CAP_FLAGS_HAS_ROLL_LOCK = 16 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
uint32 CAP_FLAGS_HAS_PITCH_AXIS = 32 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
uint32 CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
uint32 CAP_FLAGS_HAS_PITCH_LOCK = 128 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
uint32 CAP_FLAGS_HAS_YAW_AXIS = 256 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
uint32 CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
uint32 CAP_FLAGS_HAS_YAW_LOCK = 1024 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
uint32 CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
uint32 CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536 # Gimbal manager supports to point to a local position.
uint32 CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 # Gimbal manager supports to point to a global latitude, longitude, altitude position.

uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.
float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
float32 pitch_min # Minimum pitch angle (positive: up, negative: down)
float32 pitch_max # Maximum pitch angle (positive: up, negative: down)
float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)
float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)