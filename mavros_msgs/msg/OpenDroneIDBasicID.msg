# Remote ID message - Basic ID
# https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('MAV_ODID_ID_TYPE', 'ID_TYPE_')
# mavros_cog.idl_decl_enum('MAV_ODID_UA_TYPE', 'UA_TYPE_')
# ]]]
# MAV_ODID_ID_TYPE
uint8 ID_TYPE_NONE = 0                   # No type defined.
uint8 ID_TYPE_SERIAL_NUMBER = 1          # Manufacturer Serial Number (ANSI/CTA-2063 format).
uint8 ID_TYPE_CAA_REGISTRATION_ID = 2    # CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].
uint8 ID_TYPE_UTM_ASSIGNED_UUID = 3      # UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
uint8 ID_TYPE_SPECIFIC_SESSION_ID = 4    # A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO.
# MAV_ODID_UA_TYPE
uint8 UA_TYPE_NONE = 0                   # No UA (Unmanned Aircraft) type defined.
uint8 UA_TYPE_AEROPLANE = 1              # Aeroplane/Airplane. Fixed wing.
uint8 UA_TYPE_HELICOPTER_OR_MULTIROTOR = 2 # Helicopter or multirotor.
uint8 UA_TYPE_GYROPLANE = 3              # Gyroplane.
uint8 UA_TYPE_HYBRID_LIFT = 4            # VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
uint8 UA_TYPE_ORNITHOPTER = 5            # Ornithopter.
uint8 UA_TYPE_GLIDER = 6                 # Glider.
uint8 UA_TYPE_KITE = 7                   # Kite.
uint8 UA_TYPE_FREE_BALLOON = 8           # Free Balloon.
uint8 UA_TYPE_CAPTIVE_BALLOON = 9        # Captive Balloon.
uint8 UA_TYPE_AIRSHIP = 10               # Airship. E.g. a blimp.
uint8 UA_TYPE_FREE_FALL_PARACHUTE = 11   # Free Fall/Parachute (unpowered).
uint8 UA_TYPE_ROCKET = 12                # Rocket.
uint8 UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13 # Tethered powered aircraft.
uint8 UA_TYPE_GROUND_OBSTACLE = 14       # Ground Obstacle.
uint8 UA_TYPE_OTHER = 15                 # Other type of aircraft not listed earlier.
# [[[end]]] (checksum: 5c8d15e3412f1859012762424293d945)

std_msgs/Header header
string id_or_mac
uint8 id_type
uint8 ua_type
string uas_id
