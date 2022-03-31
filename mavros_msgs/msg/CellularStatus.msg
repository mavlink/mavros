#Follows https://mavlink.io/en/messages/common.html#CELLULAR_STATUS specification

uint8 status
uint8 failure_reason
uint8 type
uint8 quality
uint16 mcc
uint16 mnc
uint16 lac