# Override RC Input
# Currently MAVLink defines override for 8 channel

# Mavlink 2.0 now supports upto 18 channels
# https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE

uint16 CHAN_RELEASE=0
uint16 CHAN_NOCHANGE=65535

uint16[] channels
