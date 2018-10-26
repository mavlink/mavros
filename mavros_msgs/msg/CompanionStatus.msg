# MAVLink message: COMPANIONSTATUS
# http://mavlink.org/messages/common#COMPANIONSTATUS

std_msgs/Header header

uint8 state        # See enum COMPANION_STATE
uint8 type         # See enum COMPANION_TYPE
uint16 pid         # Process ID of the source

uint8 COMPANION_STATE_HEALTHY = 0
uint8 COMPANION_STATE_TIMEOUT = 1
uint8 COMPANION_STATE_ABORT = 2
uint8 COMPANION_STATE_STARTING = 3
uint8 COMPANION_STATE_COMPONENT_FAIL = 4

uint8 COMPANION_TYPE_GENERIC = 0
uint8 COMPANION_TYPE_AVOIDANCE = 1
uint8 COMPANION_TYPE_VIO = 2
