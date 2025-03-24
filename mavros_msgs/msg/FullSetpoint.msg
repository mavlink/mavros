# MAVLink message: SET_POSITION_TARGET_LOCAL_NED
# https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

std_msgs/Header header
uint16 type_mask # Bitmask to indicate which dimensions should be ignored by the vehicle. 
# https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK

geometry_msgs/Vector3 position # x, y, z position in NED frame (m)
geometry_msgs/Vector3 velocity # x, y, z velocity in NED frame (m/s)
geometry_msgs/Vector3 acceleration # x, y, z acceleration in NED frame (m/s^2)
float32 yaw # yaw setpoint (radians)
float32 yaw_rate # yaw rate setpoint (radians/s)
