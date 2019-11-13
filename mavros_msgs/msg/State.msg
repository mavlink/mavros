# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see https://mavlink.io/en/messages/common.html#MAV_STATE
#

std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
