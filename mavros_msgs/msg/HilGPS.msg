# HilControls.msg
#
# ROS representation of MAVLink HIL_GPS
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#HIL_GPS

std_msgs/Header header
uint8 fix_type
geographic_msgs/GeoPoint geo
uint16 eph
uint16 epv
uint16 vel
int16 vn
int16 ve
int16 vd
uint16 cog
uint8 satellites_visible
