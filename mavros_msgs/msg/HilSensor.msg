# HilSensor.msg
#
# ROS representation of MAVLink HIL_SENSOR
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR

std_msgs/Header header
float32 xacc
float32 yacc
float32 zacc
float32 xgyro
float32 ygyro
float32 zgyro
float32 xmag
float32 ymag
float32 zmag
float32 abs_pressure
float32 diff_pressure
float32 pressure_alt
flat32 temperature
uint32 fields_updated