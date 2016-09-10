# HilActuatorControls.msg
#
# ROS representation of MAVLink HIL_ACTUATOR_CONTROLS
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_ACTUATOR_CONTROLS

std_msgs/Header header
float32[16] controls
uint8 mode
uint64 flags
