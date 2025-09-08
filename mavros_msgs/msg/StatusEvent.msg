# EVENT message representation
# https://mavlink.io/en/messages/common.html#EVENT

# Severity levels
uint8 EMERGENCY = 0
uint8 ALERT = 1
uint8 CRITICAL = 2
uint8 ERROR = 3
uint8 WARNING = 4
uint8 NOTICE = 5
uint8 INFO = 6
uint8 DEBUG = 7

# Fields
std_msgs/Header header
uint8 severity
uint32 px4_id
uint8[40] arguments
uint16 sequence
