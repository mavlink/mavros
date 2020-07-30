# HilStateQuaternion.msg
#
# ROS representation of MAVLink HIL_STATE_QUATERNION
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION

std_msgs/Header header

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 linear_velocity
geographic_msgs/GeoPoint geo
float32 ind_airspeed
float32 true_airspeed
