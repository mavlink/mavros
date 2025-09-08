# MAVLink message: GIMBAL_MANAGER_SET_ATTITUDE
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE

uint8 target_system         # System ID
uint8 target_component      # Component ID

uint32 flags                # High level gimbal manager flags to use (bitwise) - See GIMBAL_MANAGER_FLAGS
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

geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.
float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.
float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.