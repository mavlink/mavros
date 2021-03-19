# Mavlink message transport type.
#
# Used to transport mavlink_message_t via ROS topic
#
# :framing_status:
#       Frame decoding status: OK, CRC error, bad Signature (mavlink v2.0)
#       You may simply drop all non valid messages.
#       Used for GCS Bridge to transport unknown messages.
#
# :magic:
#       STX byte, used to determine protocol version v1.0 or v2.0.
#
# Please use mavros_msgs::mavlink::convert() from <mavros_msgs/mavlink_convert.hpp>
# to convert between ROS and MAVLink message type

# mavlink_framing_t enum
uint8 FRAMING_OK = 1
uint8 FRAMING_BAD_CRC = 2
uint8 FRAMING_BAD_SIGNATURE = 3

# stx values
uint8 MAVLINK_V10 = 254
uint8 MAVLINK_V20 = 253

std_msgs/Header header
uint8 framing_status

uint8 magic		# STX byte
uint8 len
uint8 incompat_flags
uint8 compat_flags
uint8 seq
uint8 sysid
uint8 compid
uint32 msgid		# 24-bit message id
uint16 checksum
uint64[<=33] payload64	# max size: (255+2+7)/8
uint8[<=13] signature	# optional signature, max size: 13
