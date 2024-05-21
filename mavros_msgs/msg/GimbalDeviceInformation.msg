# MAVLink message: GIMBAL_DEVICE_INFORMATION
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION

std_msgs/Header header

string vendor_name # Name of the gimbal vendor.
string model_name # Name of the gimbal model.
string custom_name # Custom name of the gimbal given to it by the user.
uint32 firmware_version # Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
uint32 hardware_version # Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
uint64 uid # UID of gimbal hardware (0 if unknown).

uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS
#GIMBAL_DEVICE_CAP_FLAGS
uint32 CAP_FLAGS_HAS_RETRACT = 1 # Gimbal device supports a retracted position
uint32 CAP_FLAGS_HAS_NEUTRAL = 2 # Gimbal device supports a horizontal, forward looking position, stabilized
uint32 CAP_FLAGS_HAS_ROLL_AXIS = 4 # Gimbal device supports rotating around roll axis.
uint32 CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Gimbal device supports to follow a roll angle relative to the vehicle
uint32 CAP_FLAGS_HAS_ROLL_LOCK = 16 # Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
uint32 CAP_FLAGS_HAS_PITCH_AXIS = 32 # Gimbal device supports rotating around pitch axis.
uint32 CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Gimbal device supports to follow a pitch angle relative to the vehicle
uint32 CAP_FLAGS_HAS_PITCH_LOCK = 128 # Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
uint32 CAP_FLAGS_HAS_YAW_AXIS = 256 # Gimbal device supports rotating around yaw axis.
uint32 CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
uint32 CAP_FLAGS_HAS_YAW_LOCK = 1024 # Gimbal device supports locking to an absolute heading (often this is an option available)
uint32 CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Gimbal device supports yawing/panning infinetely (e.g. using slip disk).

uint16 custom_cap_flags # Bitmap for use for gimbal-specific capability flags.
float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
float32 pitch_min # Minimum pitch angle (positive: up, negative: down)
float32 pitch_max # Maximum pitch angle (positive: up, negative: down)
float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)
float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)