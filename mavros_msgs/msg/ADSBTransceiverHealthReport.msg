# Health report from a uAvionix ADS-B transceiver
#
# https://mavlink.io/en/messages/uAvionix.html#UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT

# UAVIONIX_ADSB_RF_HEALTH (bitmask)
# Defined in the uAvionix dialect, which is not part of the `common` module
# used by mavros_cog, so these constants are declared manually.
uint8 RF_HEALTH_OK = 1
uint8 RF_HEALTH_FAIL_TX = 2
uint8 RF_HEALTH_FAIL_RX = 16

std_msgs/Header header

uint8 rf_health		# ADS-B transponder status, UAVIONIX_ADSB_RF_HEALTH bit field
