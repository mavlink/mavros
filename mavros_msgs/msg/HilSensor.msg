# HilSensor.msg
#
# ROS representation of MAVLink HIL_SENSOR
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#HIL_SENSOR

std_msgs/Header header

geometry_msgs/Vector3 acc
geometry_msgs/Vector3 gyro
geometry_msgs/Vector3 mag
float32 abs_pressure
float32 diff_pressure
float32 pressure_alt
float32 temperature
uint32 fields_updated
