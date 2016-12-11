# ActuatorControlTarget.msg
#
# ROS representation of MAVLink HIL_ACTUATOR_CONTROLS
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#ACTUATOR_CONTROL_TARGET

std_msgs/Header header
float32[8] controls
uint8 group_mlx
