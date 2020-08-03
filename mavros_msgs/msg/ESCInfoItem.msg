# ESCInfoItem.msg
#
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_INFO

std_msgs/Header header

uint16 failure_flags
uint32 error_count
uint8 temperature

