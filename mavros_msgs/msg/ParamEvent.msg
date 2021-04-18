# Parameter Event
#
# That messages replaces mavros_msgs/Param from mavros v1.
# Reason for that: ROS2 have native message for parameters
#
# ROS2 also have it's own ParameterEvent stream, which could be used
# to get FCU updates too. But that message is simpler to use.

std_msgs/Header header

string param_id
rcl_interfaces/ParameterValue value
uint16 param_index
uint16 param_count
