# Mavlink message transport type.
#
# Used to transport mavlink_message_t via ROS topic
#
# :is_valid: flag meaning that there CRC16 error or message is unknown by libmavconn.
#            You may simply drop all non valid messages. Used for GCS Bridge to transport
#            unknown messages.
#
# Please use mavros_msgs::mavlink::convert() from <mavros_msgs/mavlink_convert.h>
# to convert between ROS and MAVLink message type

std_msgs/Header header
bool is_valid

uint8 len
uint8 seq
uint8 sysid
uint8 compid
uint8 msgid
uint16 checksum
uint64[] payload64
