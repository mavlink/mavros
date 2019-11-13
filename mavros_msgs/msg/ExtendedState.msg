# Extended autopilot state
#
# https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE

uint8 VTOL_STATE_UNDEFINED = 0
uint8 VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VTOL_STATE_MC = 3
uint8 VTOL_STATE_FW = 4

uint8 LANDED_STATE_UNDEFINED = 0
uint8 LANDED_STATE_ON_GROUND = 1
uint8 LANDED_STATE_IN_AIR = 2
uint8 LANDED_STATE_TAKEOFF = 3
uint8 LANDED_STATE_LANDING = 4

std_msgs/Header header
uint8 vtol_state
uint8 landed_state
