# Remote ID message - System
# https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('MAV_ODID_OPERATOR_LOCATION_TYPE', 'LOCATION_TYPE_')
# mavros_cog.idl_decl_enum('MAV_ODID_CLASSIFICATION_TYPE', 'CLASSIFICATION_TYPE_')
# ]]]
# MAV_ODID_OPERATOR_LOCATION_TYPE
uint8 LOCATION_TYPE_TAKEOFF = 0          # The location of the operator is the same as the take-off location.
uint8 LOCATION_TYPE_LIVE_GNSS = 1        # The location of the operator is based on live GNSS data.
uint8 LOCATION_TYPE_FIXED = 2            # The location of the operator is a fixed location.
# MAV_ODID_CLASSIFICATION_TYPE
uint8 CLASSIFICATION_TYPE_UNDECLARED = 0 # The classification type for the UA is undeclared.
uint8 CLASSIFICATION_TYPE_EU = 1         # The classification type for the UA follows EU (European Union) specifications.
# [[[end]]] (checksum: 518034079daae18afbc49769bc1a1031)

std_msgs/Header header
string id_or_mac
uint8 operator_location_type
uint8 classification_type
int32 operator_latitude
int32 operator_longitude
uint16 area_count
uint16 area_radius
float32 area_ceiling
float32 area_floor
uint8 category_eu
uint8 class_eu
float32 operator_altitude_geo
