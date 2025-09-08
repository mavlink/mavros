# MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW
# Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is 
# reused as such by the plugin
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL

uint8 target_system         # System ID
uint8 target_component      # Component ID

uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
#GIMBAL_MANAGER_FLAGS
uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                        # device ids: 154, 171-175

float32 pitch # Pitch angle (positive: up, negative: down, NaN to be ignored).
float32 yaw # Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
float32 pitch_rate # Pitch angular rate (positive: up, negative: down, NaN to be ignored).
float32 yaw_rate # Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).