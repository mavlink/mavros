# Extended autopilot state

uint8 VTOL_STATE_UNDEFINED = 0
uint8 VTOL_STATE_TRANSITION_TO_FW = 0
uint8 VTOL_STATE_TRANSITION_TO_MC = 0
uint8 VTOL_STATE_MC = 0
uint8 VTOL_STATE_FW = 0

uint8 LANDED_STATE_UNDEFINED = 0
uint8 LANDED_STATE_ON_GROUND = 0
uint8 LANDED_STATE_IN_AIR = 0

std_msgs/Header header
uint8 vtol_state
uint8 landed_state
