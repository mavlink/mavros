# Mavros message: COMPONENTSTATUS

std_msgs/Header header

uint8 component               # See enum MAV_COMPONENT

uint8 cpu_usage               # percentage
int8 temperature              # deg celsius
uint32 ram_usage               # MB
uint32 ram_total              # MB
uint32 storage_free           # MB
uint32 storage_total          # MB 
uint32 network_send_rate      # KB/s
uint32 network_receive_rate   # KB/s
uint32 time_usec              # us
uint32 uptime                 # ms

