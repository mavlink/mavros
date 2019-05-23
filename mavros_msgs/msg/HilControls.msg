# HilControls.msg
#
# ROS representation of MAVLink HIL_CONTROLS
# (deprecated, use HIL_ACTUATOR_CONTROLS instead)
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#HIL_CONTROLS

std_msgs/Header header
float32 roll_ailerons
float32 pitch_elevator
float32 yaw_rudder
float32 throttle
float32 aux1
float32 aux2
float32 aux3
float32 aux4
uint8 mode
uint8 nav_mode
