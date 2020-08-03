# ESCInfo.msg
#
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_INFO

std_msgs/Header header

uint16 counter
uint8 count
uint8 connection_type
uint8 info
mavros_msgs/ESCInfoItem[] esc_info

