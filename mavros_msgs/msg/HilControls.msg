# HilControls.msg
#
# ROS representation of MAVLink HIL_CONTROLS
# (deprecated, use HIL_ACTUATOR_CONTROLS instead)
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_CONTROLS

std_msgs/Header header
float32 roll_ailerons
float32 pitch_elevator
float32 yaw_rudder
float32 throttle
float32[4] aux
uint8 mode
uint8 nav_mode
