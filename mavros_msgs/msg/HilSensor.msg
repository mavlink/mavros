# HilSensor.msg
#
# ROS representation of MAVLink HIL_SENSOR
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR

std_msgs/Header header
float32[3] acc # has xacc, yacc, zacc
float32[3] gyro # has xgyro, ygyro, zgyro
float32[3] mag # has xmag, ymag, zmag
float32[3] pressure # has abs_pressure, diff_presuure, pressure_alt
float32 temperature
uint32 fields_updated