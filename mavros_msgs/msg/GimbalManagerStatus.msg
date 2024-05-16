# MAVLink message: GIMBAL_MANAGER_STATUS
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS

std_msgs/Header header

uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
#GIMBAL_MANAGER_FLAGS
uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.

uint8 sysid_primary     # System ID of MAVLink component with primary control, 0 for none.
uint8 compid_primary    # Component ID of MAVLink component with primary control, 0 for none.
uint8 sysid_secondary   # System ID of MAVLink component with secondary control, 0 for none.
uint8 compid_secondary  # Component ID of MAVLink component with secondary control, 0 for none.