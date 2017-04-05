# HilStateQuaternion.msg
#
# ROS representation of MAVLink HIL_STATE_QUATERNION
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION

std_msgs/Header header
sensor_msgs/Imu imu
geometry_msgs/Vector3 linear_velocity
geographic_msgs/GeoPoint geo
uint16 ind_airspeed
uint16 true_airspeed
