# MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS

std_msgs/Header header

uint8 target_system             # System ID
uint8 target_component          # Component ID

uint16 flags                    # Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS
#GIMBAL_DEVICE_FLAGS
uint16 FLAGS_RETRACT = 1      # Set to retracted safe position (no stabilization), takes presedence over all other flags.
uint16 FLAGS_NEUTRAL = 2      # Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
uint16 FLAGS_ROLL_LOCK = 4    # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
uint16 FLAGS_PITCH_LOCK = 8   # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
uint16 FLAGS_YAW_LOCK = 16    # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).

geometry_msgs/Quaternion q      # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
float32 angular_velocity_x      # X component of angular velocity (NaN if unknown)
float32 angular_velocity_y      # Y component of angular velocity (NaN if unknown)
float32 angular_velocity_z      # Z component of angular velocity (NaN if unknown)

uint32 failure_flags            # Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS
#GIMBAL_DEVICE_ERROR_FLAGS
uint32 ERROR_FLAGS_AT_ROLL_LIMIT = 1          # Gimbal device is limited by hardware roll limit.
uint32 ERROR_FLAGS_AT_PITCH_LIMIT = 2         # Gimbal device is limited by hardware pitch limit.
uint32 ERROR_FLAGS_AT_YAW_LIMIT = 4           # Gimbal device is limited by hardware yaw limit.
uint32 ERROR_FLAGS_ENCODER_ERROR = 8          # There is an error with the gimbal encoders.
uint32 ERROR_FLAGS_POWER_ERROR = 16           # There is an error with the gimbal power source.
uint32 ERROR_FLAGS_MOTOR_ERROR = 32           # There is an error with the gimbal motor's.
uint32 ERROR_FLAGS_SOFTWARE_ERROR = 64        # There is an error with the gimbal's software.
uint32 ERROR_FLAGS_COMMS_ERROR = 128          # There is an error with the gimbal's communication.
uint32 ERROR_FLAGS_CALIBRATION_RUNNING = 256  # Gimbal is currently calibrating.
