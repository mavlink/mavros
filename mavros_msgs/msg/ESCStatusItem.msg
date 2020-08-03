# ESCStatusItem.msg
#
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_STATUS

std_msgs/Header header

int32 rpm
float32 voltage
float32 current
