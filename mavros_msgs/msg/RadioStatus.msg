# RADIO_STATUS message

std_msgs/Header header

# message data
uint8 rssi
uint8 remrssi
uint8 txbuf
uint8 noise
uint8 remnoise
uint16 rxerrors
uint16 fixed

# calculated
float32 rssi_dbm
float32 remrssi_dbm
