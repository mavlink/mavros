# The location and information of an ADSB vehicle
#
# https://pixhawk.ethz.ch/mavlink/#ADSB_VEHICLE

std_msgs/Header header

uint32 ICAO_address
geographic_msgs/GeoPoint lla
string altitude_type # Type from ADSB_ALTITUDE_TYPE enum
uint16 heading  # cog
uint16 hor_velocity
int16 ver_velocity
string callsign  # callsign, 8+null
string emitter_type  # Type from ADSB_EMITTER_TYPE enum
std_msgs/Time tslc
uint16 data_status  # Flags from ADSB_FLAGS
uint16 squawk
