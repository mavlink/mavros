# Override RC Input
# Currently MAVLink defines override for 18 channels

# https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE

uint16 CHAN_RELEASE=0
uint16 CHAN_NOCHANGE=65535

uint16[18] channels
