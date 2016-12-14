# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see http://mavlink.org/messages/common MAV_STATE
#

std_msgs/Header header
bool connected
bool armed
bool guided
string mode
uint8 system_status
