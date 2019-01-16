# RTCM message for the gps_rtk plugin
# The gps_rtk plugin will fragment the data if necessary and 
# forward it to the FCU via Mavlink through the available link.
# data should be <= 4*180, higher will be discarded.
std_msgs/Header header
uint8[] data
