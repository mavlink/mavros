# The location and information of an ADSB vehicle
#
# https://mavlink.io/en/messages/common.html#ADSB_VEHICLE

# [[[cog:
# from pymavlink.dialects.v20 import common
#
# def decl_enum(ename, pfx='', bsz=8):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     cog.outl("# " + ename)
#     for k, e in enum:
#         sn = e.name[len(ename) + 1:]
#         l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
#         if e.description:
#             l += ' ' * (40 - len(l)) + ' # ' + e.description
#         cog.outl(l)
#
# decl_enum('ADSB_ALTITUDE_TYPE', 'ALT_')
# decl_enum('ADSB_EMITTER_TYPE', 'EMITTER_')
# decl_enum('ADSB_FLAGS', 'FLAG_', 16)
# ]]]
# ADSB_ALTITUDE_TYPE
uint8 ALT_PRESSURE_QNH = 0               # Altitude reported from a Baro source using QNH reference
uint8 ALT_GEOMETRIC = 1                  # Altitude reported from a GNSS source
# ADSB_EMITTER_TYPE
uint8 EMITTER_NO_INFO = 0
uint8 EMITTER_LIGHT = 1
uint8 EMITTER_SMALL = 2
uint8 EMITTER_LARGE = 3
uint8 EMITTER_HIGH_VORTEX_LARGE = 4
uint8 EMITTER_HEAVY = 5
uint8 EMITTER_HIGHLY_MANUV = 6
uint8 EMITTER_ROTOCRAFT = 7
uint8 EMITTER_UNASSIGNED = 8
uint8 EMITTER_GLIDER = 9
uint8 EMITTER_LIGHTER_AIR = 10
uint8 EMITTER_PARACHUTE = 11
uint8 EMITTER_ULTRA_LIGHT = 12
uint8 EMITTER_UNASSIGNED2 = 13
uint8 EMITTER_UAV = 14
uint8 EMITTER_SPACE = 15
uint8 EMITTER_UNASSGINED3 = 16
uint8 EMITTER_EMERGENCY_SURFACE = 17
uint8 EMITTER_SERVICE_SURFACE = 18
uint8 EMITTER_POINT_OBSTACLE = 19
# ADSB_FLAGS
uint16 FLAG_VALID_COORDS = 1
uint16 FLAG_VALID_ALTITUDE = 2
uint16 FLAG_VALID_HEADING = 4
uint16 FLAG_VALID_VELOCITY = 8
uint16 FLAG_VALID_CALLSIGN = 16
uint16 FLAG_VALID_SQUAWK = 32
uint16 FLAG_SIMULATED = 64
uint16 FLAG_VERTICAL_VELOCITY_VALID = 128
uint16 FLAG_BARO_VALID = 256
uint16 FLAG_SOURCE_UAT = 32768
# [[[end]]] (checksum: a34f2a081739921b6e3e443ed0516d8d)

std_msgs/Header header

uint32 ICAO_address
string callsign

float64 latitude
float64 longitude
float32 altitude 	# AMSL

float32 heading		# deg [0..360)
float32 hor_velocity	# m/s
float32 ver_velocity	# m/s

uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
uint8 emitter_type	# Type from ADSB_EMITTER_TYPE enum

duration tslc		# Duration from last communication, seconds [0..255]
uint16 flags		# ADSB_FLAGS bit field
uint16 squawk		# Squawk code
