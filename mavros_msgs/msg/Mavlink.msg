# Mavlink message transport type.
#
# Used to transport mavlink_message_t via ROS topic

std_msgs/Header header
uint8 len
uint8 seq
uint8 sysid
uint8 compid
uint8 msgid
uint64[] payload64
