# The location and information of an ADSB vehicle
#
# http://mavlink.org/messages/common#ADSB_VEHICLE

# TODO
# [[[cog:
# ]]]
# [[[end]]]

std_msgs/Header header

uint32 ICAO_address
string callsign

float64 latitude
float64 longitude
float32 altitude 	# AMSL

float32 heading		# deg
float32 hor_velocity	# m/s
float32 ver_velocity	# m/s

uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
uint8 emitter_type	# Type from ADSB_EMITTER_TYPE enum

duration tslc
uint16 flags
uint16 squawk
